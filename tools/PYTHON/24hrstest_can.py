#!/usr/bin/env python3
"""
X-axis 24 hour test over the CAN bus: go back and forth between two
positions and take a photo at each visit.

Same test as 24hrstest.py, but the PC is connected to the USB port of the
CAN MASTER and the X axis + encoder sit on a CAN slave (--node, default 11).
Moves go through the master's /motor_act (the master forwards them as SDO
writes), everything else is read and written in the slave's object
dictionary through the master's SDO bridge
({"task": "/can_act", "sdo": {...}}): position 0x2001, running 0x2004,
measured 0x2040, posErr 0x2041, axis mode 0x2042, health 0x2043,
fault 0x2044, calibrated 0x2046, calibrate 0x2048, counts per step 0x2049,
backlash 0x204A, raw counts 0x204B.
--status only prints the axis status read over CAN and does not move.

Two absolute positions are given: --scale-pos (the scale lines of the
calibration slide) and --sample-pos (the sample). Every --interval seconds
(default 10 min) the axis moves to the other position and a photo is taken
there, for --hours hours (default 24): scale, sample, scale, sample, ...
With the defaults that is 145 photos, 73 of the scale and 72 of the sample.

All moves are done in --mode 0 (open loop) by default: the firmware does
not use the encoder, this script does the compensation with its own data.
The raw encoder count at the first arrival at each place is the reference
for that place. At every later arrival the count is compared with it
(encDriftCounts). If it is more than --comp-counts (5) counts off, all X
targets are shifted by the missing steps (offsetSteps), the axis backs off
--preload steps towards the place it came from and approaches again, so the
backlash stays the same; this is repeated up to 3 times (compSteps = what
was added at this visit, residualCounts = what is left). The steps per count
come from our own references (the travel between the two places divided by
the counts between them), not from the firmware's calibration. A single
correction above --max-comp steps or a total above --max-offset steps ends
the run instead of being applied. --no-compensate only records.
Modes 1 = monitor, 2 = correct, 3 = servo are available, but the firmware's
watchdog stops the axis in all three once posErr passes its lag limit, see
below. In modes 2 and 3 the firmware adopts the encoder
position as the new step counter after every move, so compare
"measured"/"commanded" there, not rawCounts.

Calibration: modes 1..3 only work with a valid encoder calibration in the
firmware (sign, counts per step, backlash), otherwise "measured"/"posErr"
are wrong and CORRECT/SERVO move the axis to the wrong place. So with
--mode 1..3 the script first runs the firmware's calibration (SDO 0x2048 = 1:
the axis sweeps +5500 steps from where it is and back, the result is stored
in the slave), --no-calibrate skips it. Over CAN only the resulting counts
per step and backlash can be read back, not the fit quality.
After that the first leg (to the sample position) is run open loop (the
axis mode is set to 0 for this leg, there is no per-move override over CAN)
as a scale check: if posErr changes by more than --max-drift steps on it, a
closed-loop run is aborted there. In modes 1..3 the firmware's watchdog
stops the axis and latches LOST_STEPS as soon as posErr passes its lag limit
(about 136 steps), and posErr grows with the travel when the calibrated
counts per step are slightly off - over the ~296000 steps between the two
places that needs the scale to be right to about 0.03 %.

Safety, for an unattended run:
  - nothing moves before the camera and all three axes have answered and all
    targets passed the --limit / --z-limit checks;
  - the order is always X, then Y, then Z into focus (with a safe Z: Z
    retracts first); every move waits until the axis has stopped at its
    target before the next one starts. Z only ever goes to the two focus
    positions (and the safe Z, if set);
  - any SDO failure, move timeout or latched axis fault ends the run; the
    axes stay where they are, nothing is retried and no fault is cleared;
  - only a failed photo is skipped.

The two places are 3D positions (x, y, z), set as the constants SLIDE_XYZ
and SAMPLE_XYZ at the top of this file (the command line overrides them).
Y (--y-node, default 12) and the focus Z (--z-node, default 13) are further
CAN slaves, moved open loop (no encoder). At every visit: Z retracts to
SAFE_Z / --safe-z (if one is set), X moves, Y moves, Z goes to the focus of
that place, then the photo is taken. With a safe Z, Z is also retracted
before the calibration sweep and always comes into focus from the safe
position, so its backlash is the same at every photo. Without one, Z stays
in focus while X and Y move. After the last visit the stage goes back to the
scale lines (x, y, z in focus), see --no-return-to-scale; if the run ends
early (error, Ctrl+C) the axes stay where they are.

Backlash: the scale position is always approached from the sample position
and the other way round, also the very first time, so every photo of one
position is taken after the same leg.

The photos are taken with the Hikrobot camera (needs the MVS SDK installed,
and the MVS client must not have the camera open) and saved as
<csv name>_photos/scale/0000_scale_<time>.png and .../sample/0001_sample_<time>.png,
so xmas_tree_shift.ipynb can be pointed at the scale folder (the first photo
is the reference). Exposure, gain etc. are used as they are set in the
camera unless --gain is given. --photo-test only takes one photo, to check
the camera and the picture, and does not move the axis.
A full-size png is about 30 MB, so 24 h need about 5 GB of disk space.

The PC is kept awake while the script runs. Stop early with Ctrl+C.

Connect to the USB port of the CAN master.

Usage:
    uv run tools/PYTHON/24hrstest_can.py --photo-test --gain 23
    uv run tools/PYTHON/24hrstest_can.py --port COM7 --status
    uv run tools/PYTHON/24hrstest_can.py --port COM7 --speed 100000 --accel 1000000 --gain 23 --quick
    uv run tools/PYTHON/24hrstest_can.py --port COM7 --speed 100000 --accel 1000000 --gain 23

Output: the table is printed and also saved as CSV, one row per visit.
Lines starting with '#' are run metadata (pandas: read_csv(..., comment='#')).
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

# ---- the two places: absolute step positions (x, y, z) ----------------------
# Used when --scale-pos/-y/-z and --sample-pos/-y/-z are not given.
SLIDE_XYZ = (147808, 328052, 41438)    # scale lines of the calibration slide
SAMPLE_XYZ = (-148202, 328022, 43168)  # sample
# None = Z is not retracted: X and Y move with Z still at the focus of the
# place they leave, then Z goes to the new focus. X has been traversed by hand
# at both focus heights without touching anything, so this is the default.
# A number = Z goes there before X / Y move, and comes into focus afterwards.
SAFE_Z = None
# ------------------------------------------------------------------------------

MODES = {0: "OPEN_LOOP", 1: "MONITOR", 2: "CORRECT", 3: "SERVO"}
FAULTS = {0: "NONE", 1: "STALL", 2: "LOST_STEPS", 3: "DIVERGENCE", 4: "TIMEOUT",
          5: "CAL_INVALID", 6: "CAL_FAILED", 7: "ENC_NOISE"}
# CORRECT / SERVO adopt the encoder position as the new step counter, so the
# move ends near the target, not exactly on it
ARRIVE_TOL = 200
# the firmware's calibration sweeps preload 500 + 10 x 500 steps in + and back
CAL_SWEEP = 5500

# slave object dictionary (main/src/canopen/UC2_OD_Indices.h)
OD_POSITION, OD_STATUS = 0x2001, 0x2004
OD_MEASURED, OD_POS_ERR, OD_MODE, OD_HEALTH, OD_FAULT = 0x2040, 0x2041, 0x2042, 0x2043, 0x2044
OD_CALIBRATED, OD_CALIBRATE, OD_CPS_Q16, OD_BACKLASH, OD_RAW = 0x2046, 0x2048, 0x2049, 0x204A, 0x204B
OD_RESET = 0x2045  # 1 = trust encoder, 2 = trust steps, 3 = force rehome


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

    def __init__(self, ser, node, sub, stepper):
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

    def feedback(self):
        """(commanded, measured, posErr, rawCounts), like the firmware's log line."""
        return (self.sdo(OD_POSITION), self.sdo(OD_MEASURED), self.sdo(OD_POS_ERR),
                self.sdo(OD_RAW))

    def status(self):
        pos, running = self.position()
        return {"position": pos, "isRunning": int(running),
                "measured": self.sdo(OD_MEASURED), "posErr": self.sdo(OD_POS_ERR),
                "axismode": self.sdo(OD_MODE, typ="u8"), "health": self.sdo(OD_HEALTH, typ="u8"),
                "fault": self.sdo(OD_FAULT, typ="u8"),
                "calibrated": self.sdo(OD_CALIBRATED, typ="u8"), "rawCounts": self.sdo(OD_RAW)}

    def check_fault(self, what):
        if self.sdo(OD_HEALTH, typ="u8") == 2:  # latched: motion is refused
            fault = self.sdo(OD_FAULT, typ="u8")
            raise RuntimeError(f"axis fault {fault} ({FAULTS.get(fault, '?')}) during {what}\n"
                               f'clear it with {{"stepperid": {self.stepper}, "axisreset": 1}}')

    def set_mode(self, mode):
        self.sdo(OD_MODE, mode, "u8")
        time.sleep(0.5)

    def calibrate(self, timeout=120.0):
        """Run the firmware's encoder calibration (sign, counts per step,
        backlash). The axis sweeps +5500 steps from where it is and comes back.
        Returns what can be read back over CAN."""
        start = self.position()[0]
        self.sdo(OD_CALIBRATE, 1, "u8")
        t0 = time.time()
        moved, idle = False, None
        while True:
            pos, running = self.position()
            moved = moved or pos != start or running
            if moved and not running and pos == start:
                idle = idle or time.time()
                if time.time() - idle > 2.0:  # the sweep pauses between its segments
                    break
            else:
                idle = None
            if not moved and time.time() - t0 > 15:
                raise RuntimeError("calibration did not start (the axis never moved)")
            if time.time() - t0 > timeout:
                raise TimeoutError("calibration did not finish")
            time.sleep(0.1)
        time.sleep(1.0)  # the slave publishes the result on its next loop
        fault = self.sdo(OD_FAULT, typ="u8")
        if fault == 6 or not self.sdo(OD_CALIBRATED, typ="u8"):
            raise RuntimeError(f"axis calibration failed (fault {fault} {FAULTS.get(fault, '?')})")
        q16 = self.sdo(OD_CPS_Q16)  # signed: the sign is the count direction
        return {"countsPerStep": abs(q16) / 65536, "countSign": 1 if q16 >= 0 else -1,
                "backlashSteps": abs(self.sdo(OD_BACKLASH))}

    def move_to(self, target, speed, accel, tol=0, settle=1.0, timeout=60.0, encoder=True):
        """Absolute move in the axis's current mode, wait until arrived,
        return (commanded, measured, posErr, rawCounts). encoder=False is for
        an axis without encoder (focus): only the step position is used."""
        stepper = {"stepperid": self.stepper, "position": target, "speed": speed, "isabs": 1}
        if accel is not None:
            stepper["acceleration"] = accel
        self.ser.write((json.dumps({"task": "/motor_act",
                                    "motor": {"steppers": [stepper]}}) + "\n").encode())
        arrived = None
        t_end = time.time() + timeout
        while time.time() < t_end:
            time.sleep(0.2)
            pos, running = self.position()
            if encoder:
                self.check_fault(f"move to {target}")
            if not running and abs(pos - target) <= tol:
                arrived = arrived or time.time()
                if time.time() - arrived > settle:  # corrections / adoption follow the move
                    return self.feedback() if encoder else (pos, pos, 0, 0)
            else:
                arrived = None
        raise TimeoutError(f"move to {target} did not finish")


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
    ap.add_argument("--status", action="store_true",
                    help="only print the axis status read over CAN and exit")
    ap.add_argument("--scale-pos", type=int, default=SLIDE_XYZ[0],
                    help="absolute X position of the scale lines (steps)")
    ap.add_argument("--sample-pos", type=int, default=SAMPLE_XYZ[0],
                    help="absolute X position of the sample (steps)")
    ap.add_argument("--scale-y", type=int, default=SLIDE_XYZ[1],
                    help="Y position at the scale lines")
    ap.add_argument("--sample-y", type=int, default=SAMPLE_XYZ[1],
                    help="Y position at the sample")
    ap.add_argument("--y-node", type=int, default=12, help="CAN node id of the Y slave")
    ap.add_argument("--y-sub", type=int, default=3,
                    help="SDO sub-index of the axis on the Y slave (axis id + 1, Y = 3)")
    ap.add_argument("--y-stepper", type=int, default=2,
                    help="stepperid the master routes to the Y slave (Y = 2)")
    ap.add_argument("--y-speed", type=int, default=5000)
    ap.add_argument("--y-accel", type=int, default=None,
                    help="Y acceleration (default: not sent, firmware default)")
    ap.add_argument("--scale-z", type=int, default=SLIDE_XYZ[2],
                    help="focus (Z) position at the scale lines")
    ap.add_argument("--sample-z", type=int, default=SAMPLE_XYZ[2],
                    help="focus (Z) position at the sample")
    ap.add_argument("--safe-z", type=int, default=SAFE_Z,
                    help="retracted Z position: Z goes here before every X / Y "
                         "move (default: Z is not retracted)")
    ap.add_argument("--compensate", action=argparse.BooleanOptionalAction, default=True,
                    help="mode 0: correct X ourselves when the encoder at a place is "
                         "off its first arrival there (--no-compensate: only record)")
    ap.add_argument("--comp-counts", type=int, default=5,
                    help="compensate when the encoder is more than this many counts off")
    ap.add_argument("--preload", type=int, default=2000,
                    help="back-off distance for the re-approach after a correction")
    ap.add_argument("--max-comp", type=int, default=2000,
                    help="largest single correction in steps; beyond it the run ends")
    ap.add_argument("--max-offset", type=int, default=5000,
                    help="largest total correction in steps; beyond it the run ends")
    ap.add_argument("--max-drift", type=int, default=80,
                    help="closed-loop runs only: largest posErr change allowed on "
                         "the open-loop check leg. The firmware latches a "
                         "LOST_STEPS fault as soon as posErr passes its lag limit "
                         "(about 136 steps with the present calibration)")
    ap.add_argument("--z-node", type=int, default=13, help="CAN node id of the Z slave")
    ap.add_argument("--z-sub", type=int, default=4,
                    help="SDO sub-index of the axis on the Z slave (axis id + 1, Z = 4)")
    ap.add_argument("--z-stepper", type=int, default=3,
                    help="stepperid the master routes to the Z slave (Z = 3)")
    ap.add_argument("--z-speed", type=int, default=5000)
    ap.add_argument("--z-accel", type=int, default=None,
                    help="Z acceleration (default: not sent, firmware default)")
    ap.add_argument("--z-limit", type=int, default=20000,
                    help="refuse Z positions farther than this from where Z is now")
    ap.add_argument("--interval", type=float, default=600,
                    help="seconds between two visits (default 600 = 10 min)")
    ap.add_argument("--hours", type=float, default=24, help="length of the test")
    ap.add_argument("--return-to-scale", action=argparse.BooleanOptionalAction, default=True,
                    help="after the last visit go back to the scale lines (x, y, z), "
                         "if the test did not end there anyway")
    ap.add_argument("--quick", action="store_true",
                    help="rehearsal: the same sequence, but only 4 visits "
                         "(scale, sample, scale, sample) 20 s apart")
    ap.add_argument("--limit", type=int, default=400000,
                    help="refuse X / Y positions farther than this from where "
                         "the axis is now (guards against a typo)")
    ap.add_argument("--mode", type=int, choices=sorted(MODES), default=0,
                    help="axis mode for every move: 0 = open loop (default; the "
                         "encoder is only recorded), 1 = monitor, 2 = correct, "
                         "3 = servo. In 1..3 the firmware stops the axis when "
                         "posErr passes its lag limit")
    ap.add_argument("--calibrate", action=argparse.BooleanOptionalAction, default=None,
                    help="run the firmware's encoder calibration first (default: "
                         "yes if --mode is 1..3, --no-calibrate to skip)")
    ap.add_argument("--speed", type=int, default=10000)
    ap.add_argument("--accel", type=int, default=None,
                    help="acceleration (default: not sent, firmware default)")
    ap.add_argument("--gain", type=float, default=None,
                    help="camera gain in dB (default: as set in the camera)")
    ap.add_argument("--exposure", type=float, default=None,
                    help="camera exposure time in microseconds (default: as set "
                         "in the camera)")
    ap.add_argument("--photo-format", choices=("bmp", "png", "jpg", "tif"), default="png")
    ap.add_argument("--photo-delay", type=float, default=2.0,
                    help="seconds to wait after the move before a photo")
    ap.add_argument("--photo-test", action="store_true",
                    help="only take one photo and exit, the axis is not moved")
    ap.add_argument("--out", default=None, help="CSV file (default: timestamped name)")
    args = ap.parse_args()
    if args.photo_test:
        camera = Camera(args.gain, args.exposure)
        try:
            path = f"photo_test_{datetime.now():%Y%m%d_%H%M%S}.{args.photo_format}"
            camera.photo(path)
            print(f"photo saved to {path}")
        finally:
            camera.close()
        return
    if args.status:
        if not args.port:
            ap.error("--port is required")
        ser = open_master(args.port, args.baud)
        try:
            time.sleep(1.0)
            axis = Axis(ser, args.node, args.sub, args.stepper)
            print(f"X status (node {args.node}): {axis.status()}")
            print(f"countsPerStep = {axis.sdo(OD_CPS_Q16) / 65536:.5f}, "
                  f"backlash = {axis.sdo(OD_BACKLASH)} steps")
            for name, a in (("Y", Axis(ser, args.y_node, args.y_sub, args.y_stepper)),
                            ("Z", Axis(ser, args.z_node, args.z_sub, args.z_stepper))):
                print(f"{name} (node {a.node}): position, running = {a.position()}")
        finally:
            ser.close()
        return
    if not args.port:
        ap.error("--port is required")
    if args.scale_pos == args.sample_pos:
        ap.error("--scale-pos and --sample-pos are the same")
    if args.interval <= 0 or args.hours <= 0:
        ap.error("--interval and --hours must be > 0")
    if args.safe_z is not None and min(args.scale_z, args.sample_z) < args.safe_z \
            < max(args.scale_z, args.sample_z):
        ap.error("--safe-z lies between the two focus positions, so it is closer "
                 "than the focus of one of the places")
    if args.calibrate is None:
        args.calibrate = args.mode > 0
    out = args.out or f"24hrstest_can_x_{datetime.now():%Y%m%d_%H%M%S}.csv"
    places = {"scale": args.scale_pos, "sample": args.sample_pos}
    ys = {"scale": args.scale_y, "sample": args.sample_y}
    focus = {"scale": args.scale_z, "sample": args.sample_z}
    if args.quick:
        args.interval, args.hours = 20.0, 3 * 20 / 3600
    visits = round(args.hours * 3600 / args.interval) + 1 if args.quick \
        else int(args.hours * 3600 / args.interval) + 1

    camera = Camera(args.gain, args.exposure)  # before anything moves, so a camera problem shows up early
    photos = os.path.splitext(out)[0] + "_photos"
    for name in places:
        os.makedirs(os.path.join(photos, name), exist_ok=True)
    ser = open_master(args.port, args.baud)
    keep_awake(True)
    try:
        time.sleep(2.0)
        axis = Axis(ser, args.node, args.sub, args.stepper)
        status = axis.status()
        print(f"axis status (node {args.node}): {status}")
        current = status["position"]

        print(f"current = {current}, scale = {args.scale_pos}, sample = {args.sample_pos}, "
              f"{visits} visits every {args.interval:g} s ({args.hours:g} h), "
              f"speed = {args.speed}, accel = {args.accel or 'default'}, "
              f"mode = {args.mode} ({MODES[args.mode]})")
        for name, pos in places.items():
            if abs(pos - current) > args.limit:
                raise SystemExit(f"{name} position {pos} is more than {args.limit} steps "
                                 f"away from the axis ({current}), see --limit")

        def timeout_for(distance):
            return 30 + 3 * abs(distance) / args.speed

        def y_to(target):
            """Open-loop Y move (no encoder), returns the Y step position."""
            return yaxis.move_to(target, args.y_speed, args.y_accel, 0, 0.5,
                                 30 + 3 * abs(target - ypos[0]) / args.y_speed, encoder=False)[0]

        def z_to(target):
            """Open-loop focus move (no encoder), returns the Z step position."""
            return zaxis.move_to(target, args.z_speed, args.z_accel, 0, 0.5,
                                 30 + 3 * abs(target - zpos[0]) / args.z_speed, encoder=False)[0]

        yaxis = Axis(ser, args.y_node, args.y_sub, args.y_stepper)
        ypos = [yaxis.position()[0]]
        print(f"Y (node {args.y_node}): current = {ypos[0]}, scale = {args.scale_y}, "
              f"sample = {args.sample_y}, speed = {args.y_speed}")
        for name, y in ys.items():
            if abs(y - ypos[0]) > args.limit:
                raise SystemExit(f"{name} Y position {y} is more than {args.limit} steps "
                                 f"away from Y ({ypos[0]}), see --limit")

        zaxis = Axis(ser, args.z_node, args.z_sub, args.z_stepper)
        zpos = [zaxis.position()[0]]
        print(f"Z (node {args.z_node}): current = {zpos[0]}, scale = {args.scale_z}, "
              f"sample = {args.sample_z}, safe = {args.safe_z}, speed = {args.z_speed}")
        z_all = {**focus, **({} if args.safe_z is None else {"safe": args.safe_z})}
        for name, z in z_all.items():
            if abs(z - zpos[0]) > args.z_limit:
                raise SystemExit(f"{name} Z position {z} is more than {args.z_limit} steps "
                                 f"away from Z ({zpos[0]}), see --z-limit")
        if args.safe_z is None:
            print("no safe Z set: X and Y move with Z at the focus of the place they leave")
        else:
            # retract before anything moves in X (the calibration sweep too)
            zpos[0] = z_to(args.safe_z)

        cal = None
        if args.calibrate:
            print(f"calibrating the encoder (the axis sweeps +{CAL_SWEEP} steps and back) ...")
            cal = axis.calibrate()
            print(f"calibration ok: countsPerStep = {cal['countsPerStep']:.5f}, "
                  f"countSign = {cal['countSign']}, backlash = {cal['backlashSteps']} steps")
            status = axis.status()
        elif args.mode:
            # the two places were taught in as step positions: re-anchor the encoder
            # to the step counter (no motion), so an old posErr is not "corrected"
            print(f"posErr was {status['posErr']} steps: re-anchoring the encoder to the step counter")
            axis.sdo(OD_RESET, 2, "u8")
            time.sleep(1.0)
            status = axis.status()
        if status.get("fault") == 5:
            print("WARNING: the firmware reports CAL_INVALID - its calibration was only "
                  "rescaled after a microstep change, run with --calibrate")
        if args.mode and not status.get("calibrated"):
            raise SystemExit("the axis is not calibrated: the firmware would run every move "
                             "open loop. Run with --calibrate")

        with open(out, "w", newline="") as f:
            f.write(f"# test=24hrstest_can_x port={args.port} node={args.node} "
                    f"started={datetime.now().isoformat(timespec='seconds')}\n")
            f.write(f"# scale_pos={args.scale_pos} sample_pos={args.sample_pos} "
                    f"interval={args.interval:g} hours={args.hours:g} speed={args.speed} "
                    f"accel={args.accel} mode={args.mode} camera={camera.settings}\n")
            f.write(f"# scale_y={args.scale_y} sample_y={args.sample_y} y_node={args.y_node} "
                    f"y_speed={args.y_speed} y_accel={args.y_accel}\n")
            f.write(f"# scale_z={args.scale_z} sample_z={args.sample_z} safe_z={args.safe_z} "
                    f"z_node={args.z_node} z_speed={args.z_speed} z_accel={args.z_accel}\n")
            if cal:
                f.write("# calibration " + " ".join(f"{k}={v}" for k, v in cal.items()) + "\n")
            w = csv.writer(f)
            w.writerow(["time_iso", "speed", "visit", "place", "move", "target",
                        "commanded", "rawCounts", "measured", "posErr",
                        "encDriftCounts", "compSteps", "residualCounts", "offsetSteps",
                        "y", "z", "photo"])
            f.flush()
            raw0 = {}      # place -> raw encoder count at the first arrival (the reference)
            offset = [0]   # steps added to every X target: the sum of our corrections
            # every place is reached coming from the other one
            side = {"scale": 1 if args.sample_pos > args.scale_pos else -1,
                    "sample": 1 if args.scale_pos > args.sample_pos else -1}
            fw_steps_per_count = 65536 / (axis.sdo(OD_CPS_Q16) or 65536)  # signed

            def steps_per_count():
                """Steps per encoder count from our own two references (the
                travel between the two places), the firmware's value until then."""
                if len(raw0) == 2 and raw0["scale"] != raw0["sample"]:
                    return (args.scale_pos - args.sample_pos) / (raw0["scale"] - raw0["sample"])
                return fw_steps_per_count

            def x_to(x, mode, distance):
                return axis.move_to(x, args.speed, args.accel, ARRIVE_TOL if mode >= 2 else 0,
                                    2.0 if mode >= 2 else 1.0, timeout_for(distance))

            def go(visit, place, mode=args.mode, photo=True, into_focus=False):
                target = places[place]
                move = target - prev[0]
                if args.safe_z is not None:  # retract, so nothing can touch while X / Y move
                    zpos[0] = z_to(args.safe_z)
                if mode != cur_mode[0]:  # over CAN a move follows the axis mode
                    axis.set_mode(mode)
                    cur_mode[0] = mode
                commanded, measured, pos_err, raw = x_to(target + offset[0], mode, move)
                prev[0] = target
                # Our own compensation (open loop only): the encoder must read what
                # it read at the first arrival at this place. If it is off by more
                # than --comp-counts, shift all X targets by the missing steps and
                # approach the place again from the same side (same backlash).
                enc_drift = resid = comp = ""
                if mode == 0:
                    enc_drift = resid = raw - raw0.setdefault(place, raw)
                    comp = 0
                    for _ in range(3):
                        if not args.compensate or abs(resid) <= args.comp_counts:
                            break
                        step = -round(resid * steps_per_count())
                        if abs(step) > args.max_comp or abs(offset[0] + step) > args.max_offset:
                            raise SystemExit(
                                f"encoder at '{place}' is {resid} counts off its reference: that "
                                f"needs {step} steps (limit {args.max_comp}, --max-comp) and a "
                                f"total offset of {offset[0] + step} steps (limit {args.max_offset}, "
                                f"--max-offset). Not compensating, the run ends here")
                        offset[0] += step
                        comp += step
                        print(f"    compensating '{place}': encoder {resid:+d} counts off -> "
                              f"{step:+d} steps (offset now {offset[0]:+d})")
                        x_to(target + offset[0] + side[place] * args.preload, mode, args.preload)
                        commanded, measured, pos_err, raw = x_to(target + offset[0], mode, args.preload)
                        resid = raw - raw0[place]
                    if args.compensate and abs(resid) > args.comp_counts:
                        print(f"    ! WARNING: '{place}' still {resid:+d} counts off after 3 corrections")
                ypos[0] = y_to(ys[place])
                if photo or into_focus or args.safe_z is None:  # into focus (from the safe Z, if any)
                    zpos[0] = z_to(focus[place])
                now = datetime.now()
                path = ""
                if photo:
                    time.sleep(args.photo_delay)
                    path = os.path.join(photos, place,
                                        f"{visit:04d}_{place}_{now:%Y%m%d_%H%M%S}.{args.photo_format}")
                    try:
                        camera.photo(path)
                    except RuntimeError as e:  # one missing photo should not end 24 h
                        print(f"    ! {e}")
                        path = f"FAILED: {e}"
                print(f"{now:%m-%d %H:%M:%S} {visit:>5} {place:>7} {move:>8} {target:>10} "
                      f"{commanded:>10} {measured:>10} {pos_err:>7} {raw:>10} "
                      f"{enc_drift:>8} {comp:>6} {resid:>6} {ypos[0]:>8} {zpos[0]:>8}")
                w.writerow([now.isoformat(timespec="milliseconds"), args.speed, visit, place,
                            move, target, commanded, raw, measured, pos_err,
                            enc_drift, comp, resid, offset[0], ypos[0], zpos[0], path])
                f.flush()
                return pos_err

            print(f"\n{'time':>14} {'visit':>5} {'place':>7} {'move':>8} {'target':>10} "
                  f"{'commanded':>10} {'measured':>10} {'posErr':>7} {'rawCounts':>10} {'encDrift':>8} {'comp':>6} {'resid':>6} {'y':>8} {'z':>8}")
            prev = [current]
            cur_mode = [status["axismode"]]
            # the sample must be reached from the scale side the first time too (its
            # encoder reference is taken there): if the axis starts close to it,
            # back off towards the scale first
            if (current - args.sample_pos) * side["sample"] < args.preload:
                if cur_mode[0] != 0:
                    axis.set_mode(0)
                    cur_mode[0] = 0
                back = args.sample_pos + side["sample"] * args.preload
                x_to(back, 0, back - current)
                prev[0] = current = back
            # first to the sample, so the scale is approached from the sample the
            # very first time too. This leg is open loop and is the scale check:
            # the encoder must see (about) as many steps as were commanded
            first = args.sample_pos - current
            err0 = axis.sdo(OD_POS_ERR)
            drift = go(-1, "sample", mode=0, photo=False) - err0
            print(f"scale check: after {first} open-loop steps posErr changed by {drift} steps "
                  f"({100 * drift / first if first else 0:+.3f} %)")
            if args.mode and abs(drift) > args.max_drift:
                # in modes 1..3 the firmware stops the axis and latches LOST_STEPS as
                # soon as posErr passes its lag limit, and CORRECT would move the axis
                # by this much away from the place that was taught in
                raise SystemExit(
                    f"scale check failed: posErr changed by {drift} steps (limit {args.max_drift}, "
                    f"--max-drift). The calibrated counts per step do not fit this "
                    f"{abs(first)}-step travel well enough for a closed-loop mode; "
                    f"the axis is left at the sample position. Use --mode 0, or see --max-drift")

            t0 = time.time()
            for visit in range(visits):
                # fixed schedule from t0, so the move and photo times do not add up
                while time.time() < t0 + visit * args.interval:
                    time.sleep(min(1.0, max(0.0, t0 + visit * args.interval - time.time())))
                go(visit, "scale" if visit % 2 == 0 else "sample")
            if args.return_to_scale and (visits - 1) % 2:
                # the last visit was the sample: end where the test started,
                # at the scale lines (no photo, logged as one more row)
                go(visits, "scale", photo=False, into_focus=True)
            print(f"\ndone, {visits} visits")
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C")
    finally:
        keep_awake(False)
        ser.close()
        camera.close()

    print(f"raw data saved to {out}, photos in {photos}")
    if args.mode:
        print(f"note: the axis is left in axismode {args.mode} ({MODES[args.mode]})")


if __name__ == "__main__":
    main()
