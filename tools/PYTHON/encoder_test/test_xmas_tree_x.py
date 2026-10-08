#!/usr/bin/env python3
"""
X-axis open-loop "christmas tree" test: growing zig-zag, then back to start.

From the start position (where the axis is when the script starts, unless
--start is given) the axis moves left 2000, right 4000, left 6000,
right 8000, ... up to --max steps (each leg is --step longer than the one
before), then returns to the start position. All moves are open loop
unless --mode is given (1 = monitor, 2 = correct, 3 = servo; needs a valid
axis calibration). In modes 2 and 3 the firmware re-zeroes the encoder after
every move, so compare "measured"/"commanded" there, not rawCounts.
With --cycles N the whole tree (including the return to start) is repeated
N times before the AFTER photo.

Take a photo of the calibration slide before and after the run (the script
waits for Enter at both points) and compare them by cross correlation, as
part 6.5 of encoder_test.ipynb does for two photos, to get the accumulated
open-loop error (the xmas tree part of that notebook was taken out on
2026-10-08: it is part 7 of encoder_test.ipynb in commit be53de2).

Before the first photo the start position is approached from the same side
as the final return move, so backlash is the same in both photos.

The encoder position is read from the firmware's own log line:
    axis 1: commanded=10724 measured=10724 posErr=0 rawCounts=-3417
(commanded = step counter, measured = encoder position in steps,
rawCounts = raw encoder count)

Connect to the USB port of the node where the X axis + encoder are LOCAL.
use pio device monitor to see the esp32 port

Usage:
    uv install pyserial
    uv run tools/PYTHON/encoder_test/test_xmas_tree_x.py --port COM7
    ## current maximum are 100000, acceleration 1000000, max 200000
    uv run tools/PYTHON/encoder_test/test_xmas_tree_x.py --port COM7 --speed 100000 --max 2000 --accel 1000000 --cycles 1 --mode 0
    uv run tools/PYTHON/encoder_test/test_xmas_tree_x.py --port COM7 --speed 100000 --max 200000 --accel 1000000 --cycles 10 --mode 0
    uv run tools/PYTHON/encoder_test/test_xmas_tree_x.py --port COM7 --max 20000 --cycles 5

Output: the table is printed and also saved as CSV, one row per move end.
Lines starting with '#' are run metadata (pandas: read_csv(..., comment='#')).
"""
import argparse
import csv
import json
import os
import re
import time
from datetime import datetime

import serial

# results (CSV, photos) go to <repo>/encoder_test_data, which git ignores
DATA_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "encoder_test_data"))

LOG = re.compile(r"axis 1: commanded=(-?\d+) measured=(-?\d+) posErr=(-?\d+) "
                 r"rawCounts=(-?\d+)")
# watchdog trip / latched fault / async event (DIVERGENCE only warns, the rest latch)
FAULT = re.compile(r'refused: fault \d+ latched|AxisWatchdog: \w+ trip|"axisEvent"')

MODES = {0: "OPEN_LOOP", 1: "MONITOR", 2: "CORRECT", 3: "SERVO"}
# CORRECT / SERVO adopt the encoder position as the new step counter, so the
# move ends near the target, not exactly on it
ARRIVE_TOL = 200


def send(ser, stepper):
    ser.write((json.dumps({"task": "/motor_act",
                           "motor": {"steppers": [stepper]}}) + "\n").encode())


def read_position(ser, timeout=5.0):
    """Wait for the next feedback log line, return (commanded, measured, posErr, rawCounts)."""
    t_end = time.time() + timeout
    while time.time() < t_end:
        m = LOG.search(ser.readline().decode(errors="ignore"))
        if m:
            return tuple(int(v) for v in m.groups())
    raise TimeoutError("no 'axis 1: commanded=...' log line from the board")


def move_to(ser, target, speed, accel, mode=0, timeout=60.0):
    """Absolute move in the given axis mode, wait until arrived, return the last log line values."""
    ser.reset_input_buffer()
    stepper = {"stepperid": 1, "position": target, "speed": speed,
               "isabs": 1, "closedloop": mode}
    if accel is not None:
        # DeviceRouter builds read "acceleration", the legacy parser reads "accel"
        stepper["acceleration"] = accel
        stepper["accel"] = accel
    send(ser, stepper)
    tol = ARRIVE_TOL if mode >= 2 else 0
    settle = 2.0 if mode >= 2 else 1.0  # corrections / adoption follow the move
    arrived = None
    last = None
    t_end = time.time() + timeout
    while time.time() < t_end:
        line = ser.readline().decode(errors="ignore")
        if FAULT.search(line):
            if "DIVERGENCE" in line:
                print(f"    ! {line.strip()}")
            else:
                raise RuntimeError(f"axis fault during move to {target}: {line.strip()}\n"
                                   'clear it with {"stepperid": 1, "axisreset": 1}')
        m = LOG.search(line)
        if m:
            vals = tuple(int(v) for v in m.groups())
            if abs(vals[0] - target) <= tol:
                if arrived is None or (last and vals[0] != last[0]):
                    arrived = time.time()  # (re)start settling while it still moves
            else:
                arrived = None
            last = vals
        if arrived and time.time() - arrived > settle:  # let it settle
            return last
    raise TimeoutError(f"move to {target} did not finish")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--start", type=int, default=None,
                    help="start/end position (default: where the axis is now)")
    ap.add_argument("--step", type=int, default=2000, help="first leg, and growth per leg")
    ap.add_argument("--max", type=int, default=100000, help="longest leg")
    ap.add_argument("--cycles", type=int, default=1,
                    help="how many times the tree is run before the AFTER photo")
    ap.add_argument("--mode", type=int, choices=sorted(MODES), default=0,
                    help="axis mode for every move: 0 = open loop (default), "
                         "1 = monitor (warn only), 2 = correct, 3 = servo")
    ap.add_argument("--speed", type=int, default=10000)
    ap.add_argument("--accel", type=int, default=100000,
                    help="acceleration for all X moves")
    ap.add_argument("--first-dir", type=int, choices=(-1, 1), default=-1,
                    help="direction of the first leg in steps (-1 = 'left')")
    ap.add_argument("--preload", type=int, default=2000,
                    help="approach the start from this far away, on the side "
                         "the final return comes from (0 = off)")
    ap.add_argument("--out", default=None, help="CSV file (default: timestamped name in encoder_test_data/)")
    args = ap.parse_args()
    if args.cycles < 1:
        ap.error("--cycles must be >= 1")
    out = args.out or os.path.join(DATA_DIR, f"xmas_tree_x_{datetime.now():%Y%m%d_%H%M%S}.csv")

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(2.0)
    current = read_position(ser)[0]
    if args.start is None:
        args.start = current

    # absolute targets: start-2000, start+2000, start-4000, start+4000, ...
    targets = []
    pos, direction = args.start, args.first_dir
    for length in range(args.step, args.max + 1, args.step):
        pos += direction * length
        targets.append((direction * length, pos))
        direction = -direction
    last_side = 1 if targets[-1][1] > args.start else -1

    def timeout_for(distance):
        return 30 + 3 * abs(distance) / args.speed

    lo = min(t for _, t in targets)
    hi = max(t for _, t in targets)
    print(f"current = {current}, start = {args.start}, {len(targets)} legs "
          f"x {args.cycles} cycles, range {lo} .. {hi}, speed = {args.speed}, "
          f"accel = {args.accel or 'default'}, mode = {args.mode} ({MODES[args.mode]})")

    if args.mode:
        # the firmware's verify/correct step follows the axis's configured mode,
        # not the per-move "closedloop" override, so set both
        send(ser, {"stepperid": 1, "axismode": args.mode})
        time.sleep(0.5)

    with open(out, "w", newline="") as f:
        f.write(f"# test=xmas_tree_x port={args.port} "
                f"started={datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"# start={args.start} step={args.step} max={args.max} "
                f"speed={args.speed} accel={args.accel} first_dir={args.first_dir} "
                f"preload={args.preload} cycles={args.cycles} mode={args.mode}\n")
        w = csv.writer(f)
        w.writerow(["time_iso", "speed", "cycle", "leg", "move", "target",
                    "commanded", "rawCounts", "measured", "posErr"])

        def go(cycle, leg, move, target):
            commanded, measured, pos_err, raw = move_to(
                ser, target, args.speed, args.accel, args.mode,
                timeout_for(target - prev[0]))
            prev[0] = target
            print(f"{cycle:>5} {leg:>7} {move:>8} {target:>10} {commanded:>10} "
                  f"{measured:>10} {raw:>10}")
            w.writerow([datetime.now().isoformat(timespec="milliseconds"),
                        args.speed, cycle, leg, move, target,
                        commanded, raw, measured, pos_err])
            f.flush()

        print(f"\n{'cycle':>5} {'leg':>7} {'move':>8} {'target':>10} "
              f"{'commanded':>10} {'measured':>10} {'rawCounts':>10}")
        prev = [current]
        if args.preload:
            pre = args.start + last_side * args.preload
            go(0, "preload", pre - current, pre)
        go(0, "start", args.start - prev[0], args.start)
        input("\n>>> take the BEFORE photo, then press Enter to start the run ")

        # every cycle ends back at the start, approached from the same side
        for cycle in range(1, args.cycles + 1):
            for i, (move, target) in enumerate(targets, 1):
                go(cycle, i, move, target)
            go(cycle, "return", args.start - prev[0], args.start)
        print("\n>>> take the AFTER photo now")

    ser.close()
    print(f"raw data saved to {out}")
    if args.mode:
        print(f"note: axis 1 is left in axismode {args.mode} ({MODES[args.mode]})")


if __name__ == "__main__":
    main()
