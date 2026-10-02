#!/usr/bin/env python3
"""
X-axis lost-step test with the linear encoder: prints raw encoder data only.

The axis moves back and forth between start and start+steps, --cycles round
trips at each --speeds value. No verdict is printed; compare the columns
yourself.

The encoder position is read from the firmware's own log line:
    axis 1: commanded=10724 measured=10724 posErr=0 rawCounts=-3417
(commanded = step counter, rawCounts = raw encoder count)

Connect to the USB port of the node where the X axis + encoder are LOCAL.
use pio device monitor to see the esp32 port

Usage:
    uv install pyserial
    uv run tools/PYTHON/test_lost_steps_x.py --port COM7 --steps 30000 --speeds 50000 --accel 160000 --cycles 2
    uv run tools/PYTHON/test_lost_steps_x.py --port COM7 --steps 100000 --speeds 10000  --cycles 20
    python test_lost_steps_x.py --port COM7 --speeds 5000,10000,20000,40000,80000 --accel 50000

Output: the table is printed and also saved as CSV, one row per move end.
Lines starting with '#' are run metadata (pandas: read_csv(..., comment='#')).
"""
import argparse
import csv
import json
import re
import time
from datetime import datetime

import serial

LOG = re.compile(r"axis 1: commanded=(-?\d+) .*rawCounts=(-?\d+)")


def read_position(ser, timeout=5.0):
    """Wait for the next feedback log line, return (commanded, rawCounts)."""
    t_end = time.time() + timeout
    while time.time() < t_end:
        m = LOG.search(ser.readline().decode(errors="ignore"))
        if m:
            return tuple(int(v) for v in m.groups())
    raise TimeoutError("no 'axis 1: commanded=...' log line from the board")


def move_to(ser, target, speed, accel):
    """Absolute open-loop move, wait until arrived, return the last log line values."""
    ser.reset_input_buffer()
    # DeviceRouter builds read "acceleration", the legacy parser reads "accel"
    ser.write((json.dumps({"task": "/motor_act", "motor": {"steppers": [{
        "stepperid": 1, "position": target, "speed": speed,
        "acceleration": accel, "accel": accel,
        "isabs": 1, "closedloop": 0}]}}) + "\n").encode())
    arrived = None
    last = None
    t_end = time.time() + 60
    while time.time() < t_end:
        m = LOG.search(ser.readline().decode(errors="ignore"))
        if m:
            last = tuple(int(v) for v in m.groups())
            if last[0] == target and arrived is None:
                arrived = time.time()
        if arrived and time.time() - arrived > 1.0:  # let it settle
            return last
    raise TimeoutError(f"move to {target} did not finish")


def print_row(speed, cycle, end, vals):
    commanded, raw = vals
    print(f"{speed:>7} {cycle:>5} {end:>4} {commanded:>10} {raw:>10}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--steps", type=int, default=20000, help="travel per leg")
    ap.add_argument("--speeds", default="10000", help="e.g. 10000,20000,40000")
    ap.add_argument("--accel", type=int, default=100000, help="acceleration for all moves")
    ap.add_argument("--cycles", type=int, default=10, help="round trips per speed")
    ap.add_argument("--out", default=None, help="CSV file (default: timestamped name)")
    args = ap.parse_args()
    speeds = [int(v) for v in args.speeds.split(",")]
    out = args.out or f"lost_steps_x_{datetime.now():%Y%m%d_%H%M%S}.csv"

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(2.0)
    home_pos = read_position(ser)[0]
    far_pos = home_pos + args.steps
    print(f"home = {home_pos}, far = {far_pos} steps, accel = {args.accel}\n")
    print(f"{'speed':>7} {'cycle':>5} {'end':>4} {'commanded':>10} {'rawCounts':>10}")

    with open(out, "w", newline="") as f:
        f.write(f"# test=lost_steps_x port={args.port} "
                f"started={datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"# home={home_pos} far={far_pos} steps={args.steps} "
                f"speeds={args.speeds} accel={args.accel} cycles={args.cycles}\n")
        w = csv.writer(f)
        w.writerow(["time_iso", "speed", "cycle", "end", "target", "commanded", "rawCounts"])

        for speed in speeds:
            for c in range(1, args.cycles + 1):
                for end, target in (("far", far_pos), ("home", home_pos)):
                    vals = move_to(ser, target, speed, args.accel)
                    print_row(speed, c, end, vals)
                    w.writerow([datetime.now().isoformat(timespec="milliseconds"),
                                speed, c, end, target, *vals])
                    f.flush()
            print()

    ser.close()
    print(f"raw data saved to {out}")


if __name__ == "__main__":
    main()
