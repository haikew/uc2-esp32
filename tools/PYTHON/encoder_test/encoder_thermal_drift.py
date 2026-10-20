#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial>=3.5"]
# ///
"""
X-axis thermal drift test over the CAN bus with the firmware's CORRECT mode
(axismode 2) holding the place: thermal_drift.py, but at every sample the
firmware is asked to put X back where the encoder had it.

Every --interval seconds (default 10 min):
  1. the X encoder and step counter are read and a photo is taken ("before":
     the drift since the last sample, nothing corrected yet);
  2. X is sent an absolute move to the place (--speed). The step counter is
     normally already there, so the stepper does not turn, but the firmware
     still verifies the move against the encoder and corrects it when it is
     out of its tolerance (a few counts, from the calibration scatter; 2
     corrections at most). If the last verify was within tolerance it took
     the encoder as the new step counter, so this move then turns the motor
     by that difference;
  3. once X has been quiet for --settle seconds, the encoder and step counter
     are read again and a second photo is taken ("after").
So one run gives the drift without correction (before) and what the
firmware's correction leaves of it (after).

The place is the step position of X at the start. Sample 0 does the same as
every other sample: its "after" encoder count is the reference, dCounts* are
the raw count minus that, dUm* = dCounts * --um-per-count (1.95 um for the
linear encoder). corrSteps = how far the step counter moved during the hold
(corrections and adoption).

Safety: X must stand still, be calibrated (otherwise the firmware runs CORRECT
open loop) and have no latched fault. No hold move is sent if the step counter
or the encoder is more than --max-hold steps off the place; a latched fault
or such a jump stops the holding, the run goes on with photos and readings
only. The axis mode at the start is set back at the end.
Bring the stage to the place to watch, in focus, before the start.

The firmware only checks the position at the end of a move: between two
samples nothing holds X.

Only Ctrl+C ends the run early: a failed read or photo leaves that field empty.
Photos (Hikrobot camera, MVS SDK, client closed) go to
<csv name>_photos/before/0000_before_<time>.png and .../after/0000_after_<time>.png,
about 30 MB each.

Usage:
    uv run tools/PYTHON/encoder_test/encoder_thermal_drift.py --photo-test --gain 23
    uv run tools/PYTHON/encoder_test/encoder_thermal_drift.py --port COM7 --gain 23 --quick
    uv run tools/PYTHON/encoder_test/encoder_thermal_drift.py --port COM7 --gain 23 --hours 6 --interval 300

Output: one CSV row per sample, '#' lines are run metadata
(pandas: read_csv(..., comment='#')).
"""
import argparse
import csv
import ctypes
import json
import os
import sys
import time
from datetime import datetime

import serial

# results (CSV, photos) go to encoder_test_data/ next to this script
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "encoder_test_data")

MODES = {0: "OPEN_LOOP", 1: "MONITOR", 2: "CORRECT", 3: "SERVO"}
HEALTH = {0: "OK", 1: "DEGRADED", 2: "FAULT"}
FAULTS = {0: "NONE", 1: "STALL", 2: "LOST_STEPS", 3: "DIVERGENCE", 4: "TIMEOUT",
          5: "CAL_INVALID", 6: "CAL_FAILED", 7: "ENC_NOISE"}
# stage travel per count of the linear encoder
UM_PER_COUNT = 2000 / 1024

# slave object dictionary (main/src/canopen/UC2_OD_Indices.h)
OD_POSITION, OD_STATUS = 0x2001, 0x2004
OD_MEASURED, OD_POS_ERR, OD_MODE, OD_HEALTH, OD_FAULT = 0x2040, 0x2041, 0x2042, 0x2043, 0x2044
OD_CALIBRATED, OD_CPS_Q16, OD_BACKLASH, OD_RAW = 0x2046, 0x2049, 0x204A, 0x204B


def open_master(port, baud):
    """Open the master's serial port without pulsing DTR / RTS: on the
    USB-UART bridge those lines reset the ESP32."""
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


class Axis:
    """One motor axis on a CAN slave, reached through the master's serial port."""

    def __init__(self, ser, node, sub, stepper=None):
        self.ser, self.node, self.sub, self.stepper = ser, node, sub, stepper

    def sdo(self, index, value=None, typ="i32", timeout=3.0, tries=3):
        """SDO read (value is None) or write through the master's SDO bridge."""
        req = {"node": self.node, "index": index, "sub": self.sub, "type": typ,
               "op": "r" if value is None else "w"}
        if value is not None:
            req["value"] = value
        for _ in range(tries):
            self.ser.reset_input_buffer()
            self.ser.write((json.dumps({"task": "/can_act", "sdo": req}) + "\n").encode())
            t_end = time.time() + timeout
            while time.time() < t_end:
                line = self.ser.readline().decode(errors="ignore")
                i = line.find("{")
                if i < 0:
                    continue
                try:
                    resp = json.loads(line[i:])
                except ValueError:
                    continue
                if not isinstance(resp, dict) or resp.get("index") != index or "status" not in resp:
                    continue
                if resp["status"] == "ok":
                    return resp.get("value")
                break  # SDO failed on the bus: try again
            time.sleep(0.2)
        raise RuntimeError(f"SDO {'read' if value is None else 'write'} 0x{index:04X} sub {self.sub} "
                           f"on node {self.node} failed - is this port the CAN master, "
                           f"and the slave on the bus?")

    def position(self):
        """(step position, is running)"""
        return self.sdo(OD_POSITION), bool(self.sdo(OD_STATUS, typ="u8") & 1)

    def reading(self):
        """Step counter, raw count, firmware posErr, health and fault, read
        right after each other."""
        return {"counter": self.sdo(OD_POSITION), "raw": self.sdo(OD_RAW),
                "measured": self.sdo(OD_MEASURED), "posErr": self.sdo(OD_POS_ERR),
                "health": self.sdo(OD_HEALTH, typ="u8"), "fault": self.sdo(OD_FAULT, typ="u8")}

    def move_to(self, target, speed, accel=None):
        """Absolute move through the master's /motor_act: the slave runs it
        in its axis mode. Does not wait."""
        stepper = {"stepperid": self.stepper, "position": target, "speed": speed, "isabs": 1}
        if accel is not None:
            stepper["acceleration"] = accel
        self.ser.write((json.dumps({"task": "/motor_act",
                                    "motor": {"steppers": [stepper]}}) + "\n").encode())

    def wait_quiet(self, settle, timeout):
        """Wait until X has not been running and neither the step counter nor
        the encoder has changed for `settle` s (the firmware's verify and
        corrections come after the move itself). Returns the seconds it took."""
        t0 = time.time()
        last, quiet_since = None, None
        while time.time() - t0 < timeout:
            time.sleep(0.2)
            pos, running = self.position()
            now = (pos, self.sdo(OD_RAW))
            if running or now != last:
                last, quiet_since = now, None
            else:
                quiet_since = quiet_since or time.time()
                if time.time() - quiet_since >= settle:
                    return round(time.time() - t0, 1)
        raise TimeoutError(f"X did not come to rest within {timeout:g} s")


class Camera:
    """First Hikrobot camera found, through the MVS SDK's Python wrapper."""

    def __init__(self, gain=None, exposure=None):
        sdk = os.getenv("MVCAM_COMMON_RUNENV")
        if not sdk:
            raise RuntimeError("MVCAM_COMMON_RUNENV is not set - is the Hikrobot MVS SDK installed?")
        sys.path.append(os.path.join(sdk, "Samples", "Python", "MvImport"))
        import MvCameraControl_class as mv
        self.mv = mv
        self.cam = None
        mv.MvCamera.MV_CC_Initialize()
        devices = mv.MV_CC_DEVICE_INFO_LIST()
        self.check(mv.MvCamera.MV_CC_EnumDevices(mv.MV_GIGE_DEVICE | mv.MV_USB_DEVICE, devices),
                   "enum devices")
        if devices.nDeviceNum == 0:
            raise RuntimeError("no Hikrobot camera found")
        info = ctypes.cast(devices.pDeviceInfo[0], ctypes.POINTER(mv.MV_CC_DEVICE_INFO)).contents
        cam = mv.MvCamera()
        self.check(cam.MV_CC_CreateHandle(info), "create handle")
        self.cam = cam
        self.check(cam.MV_CC_OpenDevice(mv.MV_ACCESS_Exclusive, 0),
                   "open device (is it still open in the MVS client?)")
        # free running, otherwise no frame arrives without a trigger
        self.check(cam.MV_CC_SetEnumValue("TriggerMode", mv.MV_TRIGGER_MODE_OFF), "trigger mode off")
        if gain is not None:
            cam.MV_CC_SetEnumValueByString("GainAuto", "Off")  # not every model has it
            self.check(cam.MV_CC_SetFloatValue("Gain", gain), f"set gain {gain}")
        if exposure is not None:
            cam.MV_CC_SetEnumValueByString("ExposureAuto", "Off")
            self.check(cam.MV_CC_SetFloatValue("ExposureTime", exposure), f"set exposure {exposure}")
        value = mv.MVCC_FLOATVALUE()
        self.settings = {}
        for key in ("ExposureTime", "Gain"):
            if cam.MV_CC_GetFloatValue(key, value) == 0:
                self.settings[key] = round(value.fCurValue, 2)
        print(f"camera: {self.settings}")

    @staticmethod
    def check(ret, what):
        if ret != 0:
            raise RuntimeError(f"camera: {what} failed, ret = 0x{ret:x}")

    def photo(self, path, skip=2):
        """Save a frame taken from now on; the type follows the file extension."""
        mv, cam = self.mv, self.cam
        image_type = {".bmp": mv.MV_Image_Bmp, ".png": mv.MV_Image_Png,
                      ".jpg": mv.MV_Image_Jpeg, ".tif": mv.MV_Image_Tif}[os.path.splitext(path)[1]]
        self.check(cam.MV_CC_StartGrabbing(), "start grabbing")
        try:
            # the first frames after the start are skipped (auto exposure settles)
            for i in range(skip + 1):
                frame = mv.MV_FRAME_OUT()
                ctypes.memset(ctypes.byref(frame), 0, ctypes.sizeof(frame))
                self.check(cam.MV_CC_GetImageBuffer(frame, 5000), "get frame")
                try:
                    if i < skip:
                        continue
                    image = mv.MV_CC_IMAGE()
                    ctypes.memset(ctypes.byref(image), 0, ctypes.sizeof(image))
                    image.nWidth = frame.stFrameInfo.nExtendWidth
                    image.nHeight = frame.stFrameInfo.nExtendHeight
                    image.enPixelType = frame.stFrameInfo.enPixelType
                    image.pImageBuf = frame.pBufAddr
                    image.nImageBufSize = frame.stFrameInfo.nFrameLenEx
                    image.nImageLen = frame.stFrameInfo.nFrameLenEx
                    param = mv.MV_CC_SAVE_IMAGE_PARAM()
                    ctypes.memset(ctypes.byref(param), 0, ctypes.sizeof(param))
                    param.enImageType = image_type
                    param.iMethodValue = 1
                    param.nQuality = 99  # jpg only
                    self.check(cam.MV_CC_SaveImageToFileEx2(image, param, path), f"save {path}")
                finally:
                    cam.MV_CC_FreeImageBuffer(frame)
        finally:
            cam.MV_CC_StopGrabbing()

    def close(self):
        if self.cam:
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
        self.mv.MvCamera.MV_CC_Finalize()


def keep_awake(on):
    """Stop Windows from going to sleep on idle while the run is going."""
    if sys.platform == "win32":
        ES_CONTINUOUS, ES_SYSTEM_REQUIRED = 0x80000000, 0x00000001
        ctypes.windll.kernel32.SetThreadExecutionState(
            ES_CONTINUOUS | (ES_SYSTEM_REQUIRED if on else 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="serial port of the CAN master")
    ap.add_argument("--baud", type=int, default=921600,
                    help="baud rate of the master's serial port")
    ap.add_argument("--node", type=int, default=11, help="CAN node id of the X slave")
    ap.add_argument("--sub", type=int, default=2,
                    help="SDO sub-index of the axis on the slave (its "
                         "REMOTE_MOTOR_AXIS_ID + 1, X = 2)")
    ap.add_argument("--stepper", type=int, default=1,
                    help="stepperid the master routes to that slave (X = 1)")
    ap.add_argument("--y-node", type=int, default=12, help="CAN node id of the Y slave")
    ap.add_argument("--y-sub", type=int, default=3,
                    help="SDO sub-index of the axis on the Y slave (axis id + 1, Y = 3)")
    ap.add_argument("--z-node", type=int, default=13, help="CAN node id of the Z slave")
    ap.add_argument("--z-sub", type=int, default=4,
                    help="SDO sub-index of the axis on the Z slave (axis id + 1, Z = 4)")
    ap.add_argument("--interval", type=float, default=600,
                    help="seconds between two samples (default 600 = 10 min)")
    ap.add_argument("--hours", type=float, default=12, help="length of the test")
    ap.add_argument("--speed", type=int, default=10000,
                    help="speed of the hold move in steps/s (the firmware's "
                         "corrections run at their own speed)")
    ap.add_argument("--accel", type=int, default=None,
                    help="acceleration of the hold move (default: as set in the firmware)")
    ap.add_argument("--settle", type=float, default=3.0,
                    help="X must be quiet this many seconds before the 'after' reading")
    ap.add_argument("--hold-timeout", type=float, default=60.0,
                    help="seconds the hold may take before the run gives up holding")
    ap.add_argument("--max-hold", type=int, default=2000,
                    help="no hold move if the step counter or the encoder is more "
                         "than this many steps off the place")
    ap.add_argument("--um-per-count", type=float, default=UM_PER_COUNT,
                    help="stage travel per encoder count in um, for the dUm columns "
                         "(default: the linear encoder)")
    ap.add_argument("--quick", action="store_true",
                    help="rehearsal: the same sequence, but only 4 samples 20 s apart")
    ap.add_argument("--gain", type=float, default=None,
                    help="camera gain in dB (default: as set in the camera)")
    ap.add_argument("--exposure", type=float, default=None,
                    help="camera exposure time in microseconds (default: as set "
                         "in the camera)")
    ap.add_argument("--photo-format", choices=("bmp", "png", "jpg", "tif"), default="png")
    ap.add_argument("--photo-test", action="store_true",
                    help="only take one photo and exit")
    ap.add_argument("--out", default=None, help="CSV file (default: timestamped name in encoder_test_data/)")
    args = ap.parse_args()
    os.makedirs(DATA_DIR, exist_ok=True)
    if args.photo_test:
        camera = Camera(args.gain, args.exposure)
        try:
            path = os.path.join(DATA_DIR, f"photo_test_{datetime.now():%Y%m%d_%H%M%S}.{args.photo_format}")
            camera.photo(path)
            print(f"photo saved to {path}")
        finally:
            camera.close()
        return
    if not args.port:
        ap.error("--port is required")
    if args.interval <= 0 or args.hours <= 0:
        ap.error("--interval and --hours must be > 0")
    if args.quick:
        args.interval, args.hours = 20.0, 3 * 20 / 3600
    samples = int(args.hours * 3600 / args.interval + 1e-6) + 1
    out = args.out or os.path.join(DATA_DIR, f"encoder_thermal_drift_x_{datetime.now():%Y%m%d_%H%M%S}.csv")

    camera = Camera(args.gain, args.exposure)  # first, so a camera problem shows up early
    photos = os.path.splitext(out)[0] + "_photos"
    for sub in ("before", "after"):
        os.makedirs(os.path.join(photos, sub), exist_ok=True)
    ser = open_master(args.port, args.baud)
    keep_awake(True)
    axis, mode0, mode_changed, done = None, None, False, 0
    try:
        time.sleep(2.0)
        axis = Axis(ser, args.node, args.sub, args.stepper)
        r = axis.reading()
        running = axis.position()[1]
        mode0 = axis.sdo(OD_MODE, typ="u8")
        calibrated = axis.sdo(OD_CALIBRATED, typ="u8")
        q16, backlash = axis.sdo(OD_CPS_Q16), abs(axis.sdo(OD_BACKLASH))
        steps_per_count = 65536 / abs(q16) if q16 else 0
        print(f"X (node {args.node}): position = {r['counter']}, running = {int(running)}, "
              f"rawCounts = {r['raw']}, posErr = {r['posErr']}, axismode = {mode0} "
              f"({MODES.get(mode0, '?')}), health = {HEALTH.get(r['health'], r['health'])}, "
              f"calibrated = {calibrated}")
        print(f"calibration: {steps_per_count:.3f} steps per count, backlash {backlash} steps")
        if running:
            raise SystemExit("the X axis is running: start when the stage stands still")
        if r["health"] == 2:
            raise SystemExit(f"X has a latched fault {r['fault']} ({FAULTS.get(r['fault'], '?')}): "
                             f'clear it first with {{"stepperid": {args.stepper}, "axisreset": 1}}')
        if not calibrated:
            raise SystemExit("X is not calibrated: CORRECT would run open loop. Calibrate it "
                             "first (write 1 to OD 0x2048)")
        place = r["counter"]
        if mode0 != 2:
            print(f"axismode {mode0} -> 2 (CORRECT), set back to {mode0} at the end")
            axis.sdo(OD_MODE, 2, "u8")
            mode_changed = True
            time.sleep(0.5)
        # the other two axes have no encoder: only their step counters, if they answer
        others = {}
        for name, a in (("y", Axis(ser, args.y_node, args.y_sub)),
                        ("z", Axis(ser, args.z_node, args.z_sub))):
            try:
                print(f"{name.upper()} (node {a.node}): position, running = {a.position()}")
                others[name] = a
            except RuntimeError as e:
                print(f"{name.upper()} is not recorded: {e}")
        print(f"{samples} samples every {args.interval:g} s ({args.hours:g} h), X held at {place} "
              f"in CORRECT mode")

        with open(out, "w", newline="") as f:
            f.write(f"# test=encoder_thermal_drift_x port={args.port} node={args.node} "
                    f"stepper={args.stepper} started={datetime.now().isoformat(timespec='seconds')}\n")
            f.write(f"# interval={args.interval:g} hours={args.hours:g} mode=2 mode_at_start={mode0} "
                    f"place={place} speed={args.speed} accel={args.accel} settle={args.settle:g} "
                    f"max_hold={args.max_hold} encoder=linear um_per_count={args.um_per_count:.5f} "
                    f"camera={camera.settings}\n")
            f.write(f"# steps_per_count={steps_per_count:.4f} backlash_steps={backlash} "
                    f"y_node={args.y_node if 'y' in others else None} "
                    f"z_node={args.z_node if 'z' in others else None}\n")
            w = csv.writer(f)
            w.writerow(["time_iso", "sample", "elapsed_s", "place",
                        "counterBefore", "rawBefore", "posErrBefore", "dCountsBefore", "dUmBefore",
                        "holdSent", "holdSec",
                        "counterAfter", "rawAfter", "posErrAfter", "dCountsAfter", "dUmAfter",
                        "corrSteps", "health", "fault", "y", "z",
                        "photoBefore", "photoAfter", "note"])
            f.flush()
            print(f"\n{'time':>14} {'sample':>6} {'elapsed':>8} {'dCntBef':>7} {'dUmBef':>7} "
                  f"{'hold':>5} {'corr':>6} {'dCntAft':>7} {'dUmAft':>7} {'health':>8}")
            ref = None  # raw count after the hold of sample 0
            holding = True
            drift_before, drift_after = [], []
            t0 = time.time()
            for i in range(samples):
                # fixed schedule from t0, so the reads, holds and photos do not add up
                while time.time() < t0 + i * args.interval:
                    time.sleep(min(1.0, max(0.0, t0 + i * args.interval - time.time())))
                now = datetime.now()
                elapsed = round(time.time() - t0, 1)
                note = []
                row = dict.fromkeys(["counterBefore", "rawBefore", "posErrBefore", "dCountsBefore",
                                     "dUmBefore", "holdSent", "holdSec", "counterAfter", "rawAfter",
                                     "posErrAfter", "dCountsAfter", "dUmAfter", "corrSteps",
                                     "health", "fault"], "")

                def photo(sub):
                    path = os.path.join(photos, sub, f"{i:04d}_{sub}_{datetime.now():%Y%m%d_%H%M%S}"
                                                     f".{args.photo_format}")
                    try:
                        camera.photo(path)
                    except RuntimeError as e:  # one missing photo should not end the run
                        note.append(str(e))
                        path = f"FAILED: {e}"
                    return path

                # 1. before: the drift since the last hold
                before = None
                try:
                    before = axis.reading()
                    row.update(counterBefore=before["counter"], rawBefore=before["raw"],
                               posErrBefore=before["posErr"])
                    if ref is not None:
                        d = before["raw"] - ref
                        row.update(dCountsBefore=d, dUmBefore=round(d * args.um_per_count, 2))
                        drift_before.append(d)
                except RuntimeError as e:  # one missing reading should not end the run
                    note.append(str(e))
                photo_before = photo("before")

                # 2. hold: an absolute move to the place, the firmware verifies and corrects
                if holding and before:
                    off_counter = place - before["counter"]
                    off_encoder = before["measured"] - place
                    if before["health"] == 2:
                        holding = False
                        note.append(f"fault {before['fault']} ({FAULTS.get(before['fault'], '?')}) "
                                    f"latched: no more holds")
                    elif max(abs(off_counter), abs(off_encoder)) > args.max_hold:
                        holding = False
                        note.append(f"step counter {off_counter:+d} / encoder {off_encoder:+d} steps "
                                    f"off the place (limit {args.max_hold}): no more holds")
                    else:
                        try:
                            axis.move_to(place, args.speed, args.accel)
                            row.update(holdSent=off_counter,
                                       holdSec=axis.wait_quiet(args.settle, args.hold_timeout))
                        except (RuntimeError, TimeoutError) as e:
                            holding = False
                            note.append(f"hold failed, no more holds: {e}")

                # 3. after: what the correction left
                try:
                    after = axis.reading()
                    ref = after["raw"] if ref is None else ref
                    d = after["raw"] - ref
                    row.update(counterAfter=after["counter"], rawAfter=after["raw"],
                               posErrAfter=after["posErr"], dCountsAfter=d,
                               dUmAfter=round(d * args.um_per_count, 2),
                               health=HEALTH.get(after["health"], after["health"]),
                               fault=FAULTS.get(after["fault"], after["fault"]))
                    drift_after.append(d)
                    if before:
                        row["corrSteps"] = after["counter"] - before["counter"]
                    if after["health"] == 2 and holding:
                        holding = False
                        note.append(f"fault {after['fault']} ({FAULTS.get(after['fault'], '?')}) "
                                    f"latched during the hold: no more holds")
                except RuntimeError as e:
                    note.append(str(e))
                photo_after = photo("after")

                yz = {"y": "", "z": ""}
                for name, a in others.items():
                    try:
                        yz[name] = a.sdo(OD_POSITION)
                    except RuntimeError as e:
                        note.append(str(e))
                print(f"{now:%m-%d %H:%M:%S} {i:>6} {elapsed:>8} {row['dCountsBefore']:>7} "
                      f"{row['dUmBefore']:>7} {row['holdSent']:>5} {row['corrSteps']:>6} "
                      f"{row['dCountsAfter']:>7} {row['dUmAfter']:>7} {row['health']:>8}")
                for n in note:
                    print(f"    ! {n}")
                w.writerow([now.isoformat(timespec="milliseconds"), i, elapsed, place]
                           + list(row.values()) + [yz["y"], yz["z"], photo_before, photo_after,
                                                   "; ".join(note)])
                f.flush()
                done += 1

            def span(d):
                return (f"{min(d):+d} .. {max(d):+d} counts = {min(d) * args.um_per_count:+.2f} .. "
                        f"{max(d) * args.um_per_count:+.2f} um") if d else "-"
            print(f"\ndone, {done} samples. Encoder against the reference: "
                  f"before the hold {span(drift_before)}, after {span(drift_after)}")
    except KeyboardInterrupt:
        print(f"\nstopped by Ctrl+C after {done} samples")
    finally:
        if mode_changed:
            try:
                axis.sdo(OD_MODE, mode0, "u8")
                print(f"axismode set back to {mode0} ({MODES.get(mode0, '?')})")
            except RuntimeError as e:
                print(f"axismode could not be set back to {mode0}: {e}")
        keep_awake(False)
        ser.close()
        camera.close()

    print(f"raw data saved to {out}, photos in {photos}")


if __name__ == "__main__":
    main()
