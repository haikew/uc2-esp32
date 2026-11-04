#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial>=3.5"]
# ///
"""
X-axis thermal drift test over the CAN bus, open loop (axismode 0) with the
compensation done by this script, as in ultra_test_can_linear.py:
encoder_thermal_drift.py without the firmware's CORRECT mode.

Every --interval seconds (default 10 min):
  1. the X encoder and step counter are read and a photo is taken ("before":
     the drift since the last sample, nothing compensated yet);
  2. if the raw count is more than --comp-counts (3) counts off the reference,
     the X target is shifted by the missing steps (offsetSteps) and X moves
     there, then the encoder is read again; repeated up to --comp-tries (3)
     times (compSteps = what was added at this sample, compTries how often).
     A correction against the last direction loses the backlash at the first
     try, the next one catches up;
  3. a second photo is taken ("after"), after the compensation if there was one.
So one run gives the drift without compensation (before) and what the
compensation leaves of it (after).

The place is the step position of X at the start, the raw count of the first
reading the reference. dCounts* are the raw count minus the reference,
dUm* = dCounts * --um-per-count (1.95 um for the linear encoder).
--preload N: every compensating move (and once at sample 0, before the
reference is taken) backs off N steps to the --side and comes back, so X
always arrives from the same side. Off by default.
Steps per count come from the firmware's calibration (signed), or
--steps-per-count if X is not calibrated; a few % off only costs another try.

Safety: X must stand still and have no latched fault. If the total offset
would get above --max-offset steps, nothing more is compensated; the run goes
on with photos and readings only. A failed move also stops the compensation.
The axis mode at the start is set back at the end.
Bring the stage to the place to watch, in focus, before the start.

Only Ctrl+C ends the run early: a failed read or photo leaves that field empty.
Photos (Hikrobot camera, MVS SDK, client closed) go to
<csv name>_photos/before/0000_before_<time>.png and .../after/0000_after_<time>.png,
about 30 MB each.

Usage:
    uv run tools/PYTHON/encoder_test/encoder_thermal_drift_open_loop.py --photo-test
    uv run tools/PYTHON/encoder_test/encoder_thermal_drift_open_loop.py --port COM7 --quick
    uv run tools/PYTHON/encoder_test/encoder_thermal_drift_open_loop.py --port COM7 --hours 6 --interval 300

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
# steps per count of the linear encoder, signed: 24 h run of 2026-10-07
# (296064 steps between the two places = 47416 counts, same direction)
STEPS_PER_COUNT = 6.244
# camera with the lamp on, as in encoder_backlash.py
GAIN = 5.0          # dB
EXPOSURE = 100000   # us

# slave object dictionary (main/src/canopen/UC2_OD_Indices.h)
OD_POSITION, OD_STATUS = 0x2001, 0x2004
OD_MEASURED, OD_POS_ERR, OD_MODE, OD_HEALTH, OD_FAULT = 0x2040, 0x2041, 0x2042, 0x2043, 0x2044
OD_CALIBRATED, OD_CPS_Q16, OD_RAW = 0x2046, 0x2049, 0x204B


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
        """Step counter, raw count, health and fault, read right after each other."""
        return {"counter": self.sdo(OD_POSITION), "raw": self.sdo(OD_RAW),
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
        """Wait until X has not been running, its step counter has not changed
        and the encoder has stayed within 1 count (a count on its edge may
        flicker) for `settle` s. Returns the seconds it took."""
        t0 = time.time()
        last, quiet_since = None, None
        while time.time() - t0 < timeout:
            time.sleep(0.2)
            pos, running = self.position()
            raw = self.sdo(OD_RAW)
            if running or last is None or pos != last[0] or abs(raw - last[1]) > 1:
                last, quiet_since = (pos, raw), None
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
    ap.add_argument("--comp-counts", type=int, default=3,
                    help="compensate when the encoder is more than this many counts "
                         "off the reference")
    ap.add_argument("--comp-tries", type=int, default=3,
                    help="compensating moves per sample at most")
    ap.add_argument("--preload", type=int, default=0,
                    help="back-off distance in steps before every compensating move, "
                         "so X always arrives from --side (default 0: straight there)")
    ap.add_argument("--side", type=int, choices=(1, -1), default=1,
                    help="with --preload: X arrives from this side, 1 = from larger "
                         "step positions (backs off +preload), -1 = from smaller")
    ap.add_argument("--steps-per-count", type=float, default=STEPS_PER_COUNT,
                    help="signed steps per encoder count, used if X is not calibrated "
                         "(default: the linear encoder of the 24 h run)")
    ap.add_argument("--max-offset", type=int, default=2000,
                    help="largest total compensation in steps; beyond it nothing "
                         "more is compensated")
    ap.add_argument("--speed", type=int, default=10000,
                    help="speed of the compensating moves in steps/s")
    ap.add_argument("--accel", type=int, default=None,
                    help="acceleration of the compensating moves (default: as set in the firmware)")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="X must be quiet this many seconds after a move")
    ap.add_argument("--move-timeout", type=float, default=30.0,
                    help="seconds a compensating move may take")
    ap.add_argument("--um-per-count", type=float, default=UM_PER_COUNT,
                    help="stage travel per encoder count in um, for the dUm columns "
                         "(default: the linear encoder)")
    ap.add_argument("--quick", action="store_true",
                    help="rehearsal: the same sequence, but only 4 samples 20 s apart")
    ap.add_argument("--gain", type=float, default=GAIN, help="camera gain in dB")
    ap.add_argument("--exposure", type=float, default=EXPOSURE,
                    help="camera exposure time in microseconds")
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
    out = args.out or os.path.join(DATA_DIR, f"encoder_thermal_drift_open_loop_x_"
                                             f"{datetime.now():%Y%m%d_%H%M%S}.csv")

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
        q16 = axis.sdo(OD_CPS_Q16)
        print(f"X (node {args.node}): position = {r['counter']}, running = {int(running)}, "
              f"rawCounts = {r['raw']}, axismode = {mode0} ({MODES.get(mode0, '?')}), "
              f"health = {HEALTH.get(r['health'], r['health'])}, calibrated = {calibrated}")
        if running:
            raise SystemExit("the X axis is running: start when the stage stands still")
        if r["health"] == 2:
            raise SystemExit(f"X has a latched fault {r['fault']} ({FAULTS.get(r['fault'], '?')}): "
                             f'clear it first with {{"stepperid": {args.stepper}, "axisreset": 1}}')
        # signed: counts and steps may run in opposite directions
        from_cal = bool(calibrated and q16)
        k = 65536 / q16 if from_cal else args.steps_per_count
        print(f"{k:+.3f} steps per count ({'firmware calibration' if from_cal else '--steps-per-count'})")
        place = r["counter"]
        if mode0 != 0:
            print(f"axismode {mode0} -> 0 (OPEN_LOOP), set back to {mode0} at the end")
            axis.sdo(OD_MODE, 0, "u8")
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
        print(f"{samples} samples every {args.interval:g} s ({args.hours:g} h), X at {place}, "
              f"compensated when more than {args.comp_counts} counts off")

        def goto(target):
            """Open-loop move to `target`, wait until X is quiet there. Sent once
            more if X is not there (a command lost on the way)."""
            for _ in range(2):
                axis.move_to(target, args.speed, args.accel)
                axis.wait_quiet(args.settle, args.move_timeout)
                pos = axis.position()[0]
                if pos == target:
                    return
            raise RuntimeError(f"X stopped at {pos}, not at {target}")

        def approach(target):
            """To `target`, with --preload first backing off to the --side."""
            if args.preload:
                goto(target + args.side * args.preload)
            goto(target)

        with open(out, "w", newline="") as f:
            f.write(f"# test=encoder_thermal_drift_open_loop_x port={args.port} node={args.node} "
                    f"stepper={args.stepper} started={datetime.now().isoformat(timespec='seconds')}\n")
            f.write(f"# interval={args.interval:g} hours={args.hours:g} mode=0 mode_at_start={mode0} "
                    f"place={place} comp_counts={args.comp_counts} comp_tries={args.comp_tries} "
                    f"preload={args.preload} side={args.side} max_offset={args.max_offset} "
                    f"speed={args.speed} accel={args.accel} settle={args.settle:g} "
                    f"encoder=linear um_per_count={args.um_per_count:.5f} camera={camera.settings}\n")
            f.write(f"# steps_per_count={k:.4f} from_calibration={from_cal} "
                    f"y_node={args.y_node if 'y' in others else None} "
                    f"z_node={args.z_node if 'z' in others else None}\n")
            w = csv.writer(f)
            w.writerow(["time_iso", "sample", "elapsed_s", "place",
                        "counterBefore", "rawBefore", "dCountsBefore", "dUmBefore",
                        "compTries", "compSteps", "offsetSteps",
                        "counterAfter", "rawAfter", "dCountsAfter", "dUmAfter",
                        "y", "z", "photoBefore", "photoAfter", "note"])
            f.flush()
            print(f"\n{'time':>14} {'sample':>6} {'elapsed':>8} {'dCntBef':>7} {'dUmBef':>7} "
                  f"{'tries':>5} {'comp':>6} {'offset':>6} {'dCntAft':>7} {'dUmAft':>7}")
            ref = None     # raw count of the first reading (with --preload: after the first approach)
            offset = 0     # steps added to the place: the sum of the compensations
            compensating = True
            drift_before, drift_after = [], []
            t0 = time.time()
            for i in range(samples):
                # fixed schedule from t0, so the reads, moves and photos do not add up
                while time.time() < t0 + i * args.interval:
                    time.sleep(min(1.0, max(0.0, t0 + i * args.interval - time.time())))
                now = datetime.now()
                elapsed = round(time.time() - t0, 1)
                note = []
                row = dict.fromkeys(["counterBefore", "rawBefore", "dCountsBefore", "dUmBefore",
                                     "compTries", "compSteps", "offsetSteps", "counterAfter",
                                     "rawAfter", "dCountsAfter", "dUmAfter"], "")

                def photo(sub):
                    path = os.path.join(photos, sub, f"{i:04d}_{sub}_{datetime.now():%Y%m%d_%H%M%S}"
                                                     f".{args.photo_format}")
                    try:
                        camera.photo(path)
                    except RuntimeError as e:  # one missing photo should not end the run
                        note.append(str(e))
                        path = f"FAILED: {e}"
                    return path

                # 1. before: the drift since the last compensation
                resid = None
                try:
                    before = axis.reading()
                    row.update(counterBefore=before["counter"], rawBefore=before["raw"])
                    ref = before["raw"] if ref is None else ref
                    resid = before["raw"] - ref
                    row.update(dCountsBefore=resid, dUmBefore=round(resid * args.um_per_count, 2))
                    drift_before.append(resid)
                except RuntimeError as e:  # one missing reading should not end the run
                    note.append(str(e))
                photo_before = photo("before")

                # 2. compensation; with --preload, sample 0 approaches once and
                # takes the reference after that
                tries = comp = 0
                try:
                    if i == 0 and args.preload and ref is not None:
                        approach(place)
                        ref = axis.sdo(OD_RAW)
                    while (compensating and resid is not None and abs(resid) > args.comp_counts
                           and tries < args.comp_tries):
                        step = -round(resid * k)
                        if abs(offset + step) > args.max_offset:
                            compensating = False
                            note.append(f"encoder {resid:+d} counts off needs {step:+d} steps, total "
                                        f"offset {offset + step:+d} (limit {args.max_offset}): "
                                        f"no more compensation")
                            break
                        offset += step
                        comp += step
                        tries += 1
                        old = resid
                        approach(place + offset)
                        resid = axis.sdo(OD_RAW) - ref
                        if abs(resid) > abs(old):
                            compensating = False
                            note.append(f"the compensation made it worse ({old:+d} -> {resid:+d} "
                                        f"counts; sign of the steps per count?): no more compensation")
                            break
                    if compensating and resid is not None and abs(resid) > args.comp_counts:
                        note.append(f"still {resid:+d} counts off after {tries} tries")
                except (RuntimeError, TimeoutError) as e:
                    compensating = False
                    note.append(f"move failed, no more compensation: {e}")
                row.update(compTries=tries, compSteps=comp, offsetSteps=offset)

                # 3. after: what the compensation left
                try:
                    after = axis.reading()
                    row.update(counterAfter=after["counter"], rawAfter=after["raw"])
                    if ref is not None:
                        d = after["raw"] - ref
                        row.update(dCountsAfter=d, dUmAfter=round(d * args.um_per_count, 2))
                        drift_after.append(d)
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
                      f"{row['dUmBefore']:>7} {tries:>5} {comp:>6} {offset:>6} "
                      f"{row['dCountsAfter']:>7} {row['dUmAfter']:>7}")
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
            print(f"\ndone, {done} samples. Encoder against the reference: before the "
                  f"compensation {span(drift_before)}, after {span(drift_after)}; "
                  f"offset {offset:+d} steps")
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
