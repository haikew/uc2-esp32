#!/usr/bin/env python3
"""
X-axis backlash (reversal) test, bidirectional approach. RAW DATA ONLY.

The encoder data is read from the firmware's own log line:
    axis 1: commanded=10724 measured=10724 posErr=0 rawCounts=-3417
(commanded = step counter, rawCounts = raw encoder count)

Connect to the USB port of the node where the X axis + encoder are LOCAL.
Use `pio device monitor` to find the ESP32 port.

Usage:
    uv pip install pyserial
    uv run tools/PYTHON/test_backlash_x.py --port COM7 --speeds 20000 --accel 160000
    uv run tools/PYTHON/test_backlash_x.py --port COM7 --positions 0,5000,10000 --approach 2000 --speeds 5000,20000 --cycles 5

Method (per speed, per cycle, per target position P = start + offset):
    from below:  move to P - D (pre), then to P (target)  -> record
    from above:  move to P + D (pre), then to P (target)  -> record
  The commanded position at the two "target" rows is identical, so the
  rawCounts difference between them is the reversal/lost motion in encoder
  counts, independent of the counts-per-step scale.
  D (--approach) must be larger than the expected backlash.
  Moves are forced open-loop ("closedloop":0); check posErr in the data to
  confirm the axis was not corrected.

Output: one CSV row per settled move (pre and target), no processing.
Lines starting with '#' are run metadata (pandas: read_csv(..., comment='#')).
"""
import argparse
import csv
import json
import re
import sys
import time
from datetime import datetime

import serial

LOG = re.compile(r"axis 1: commanded=(-?\d+) measured=(-?\d+) "
                 r"posErr=(-?\d+) rawCounts=(-?\d+)")
FIELDS = ("commanded", "measured", "posErr", "rawCounts")


def read_log(ser, timeout):
    """Return the next feedback log line as a dict, or None on timeout."""
    t_end = time.time() + timeout
    while time.time() < t_end:
        m = LOG.search(ser.readline().decode(errors="ignore"))
        if m:
            return dict(zip(FIELDS, (int(v) for v in m.groups())))
    return None


def move_and_settle(ser, target, speed, accel, settle, move_timeout=60.0):
    """Absolute open-loop move; wait for commanded == target, wait `settle` s,
    then take a fresh log line. Returns (values, fresh). fresh=0 means the
    board printed nothing after settling and the arrival line was used."""
    ser.reset_input_buffer()
    # DeviceRouter builds read "acceleration", the legacy parser reads "accel"
    ser.write((json.dumps({"task": "/motor_act", "motor": {"steppers": [{
        "stepperid": 1, "position": target, "speed": speed,
        "acceleration": accel, "accel": accel,
        "isabs": 1, "closedloop": 0}]}}) + "\n").encode())

    last = None
    t_end = time.time() + move_timeout
    while time.time() < t_end:
        v = read_log(ser, 0.5)
        if v:
            last = v
            if v["commanded"] == target:
                break
    else:
        raise TimeoutError(f"move to {target} did not finish")

    time.sleep(settle)
    ser.reset_input_buffer()
    v = read_log(ser, 2.0)
    return (v, 1) if v else (last, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--positions", default="0,5000,10000,15000,20000",
                    help="target offsets from the start position, in steps")
    ap.add_argument("--approach", type=int, default=2000,
                    help="pre-position distance D in steps (must exceed backlash)")
    ap.add_argument("--speeds", default="10000", help="e.g. 10000,20000,40000")
    ap.add_argument("--accel", type=int, default=100000, help="acceleration for all moves")
    ap.add_argument("--cycles", type=int, default=5, help="cycles per speed")
    ap.add_argument("--settle", type=float, default=1.0, help="wait after arrival [s]")
    ap.add_argument("--out", default=None, help="CSV file (default: timestamped name)")
    ap.add_argument("--yes", action="store_true", help="skip the travel-range confirmation")
    args = ap.parse_args()

    offsets = [int(p) for p in args.positions.split(",")]
    speeds = [int(v) for v in args.speeds.split(",")]
    D = args.approach
    out = args.out or f"backlash_x_{datetime.now():%Y%m%d_%H%M%S}.csv"

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(2.0)
    first = read_log(ser, 5.0)
    if first is None:
        sys.exit("no 'axis 1: commanded=...' log line within 5 s "
                 "(does the firmware log while idle?)")
    start = first["commanded"]

    lo, hi = start + min(offsets) - D, start + max(offsets) + D
    print(f"start = {start} steps")
    print(f"the stage will travel between {lo} and {hi} steps (commanded)")
    if not args.yes and input("inside the safe travel range? [y/N] ").strip().lower() != "y":
        sys.exit("aborted")

    t0 = time.time()
    with open(out, "w", newline="") as f:
        f.write(f"# test=backlash_x_bidirectional port={args.port} "
                f"started={datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"# start_commanded={start} positions={args.positions} approach={D} "
                f"cycles={args.cycles} speeds={args.speeds} accel={args.accel} "
                f"settle={args.settle}\n")
        w = csv.writer(f)
        w.writerow(["time_iso", "t_s", "speed", "cycle", "pos_idx", "target", "approach",
                    "phase", "move_to", *FIELDS, "fresh"])

        for speed in speeds:
            for c in range(1, args.cycles + 1):
                for i, off in enumerate(offsets):
                    P = start + off
                    for approach, pre in (("below", P - D), ("above", P + D)):
                        for phase, tgt in (("pre", pre), ("target", P)):
                            v, fresh = move_and_settle(ser, tgt, speed,
                                                       args.accel, args.settle)
                            w.writerow([datetime.now().isoformat(timespec="milliseconds"),
                                        f"{time.time() - t0:.3f}", speed, c, i, P, approach,
                                        phase, tgt, *(v[k] for k in FIELDS), fresh])
                            f.flush()
                            print(f"speed {speed} cyc {c} pos {i} {approach:>5} {phase:>6}: "
                                  + " ".join(f"{k}={v[k]}" for k in FIELDS)
                                  + ("" if fresh else "  (not fresh)"))

    move_and_settle(ser, start, speeds[0], args.accel, 0.2)  # return, not recorded
    ser.close()
    print(f"\nraw data saved to {out}")


if __name__ == "__main__":
    main()