#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial>=3.5", "numpy>=2", "pillow", "matplotlib"]
# ///
"""
X-axis backlash, measured optically: photos of the scale lines after an
approach from below and after an approach from above.

Before the start bring the scale lines of the calibration slide into the
picture and into focus: where X is then is position 0. Y and Z are not moved.

  1. a photo at position 0: the tick period of the ruler in it gives the
     pixel size (--tick-um is the distance between two small ticks);
  2. X goes slowly (--slow-speed) --step (100) steps in + to position 1; the
     photo there is the reference all later photos are compared with. The
     shift of the picture over these steps is the direction of + in the
     picture and a check of the pixel size (px per step);
  3. --rounds (10) times, at --speed / --accel:
        from below:  to position 1 - D, then to position 1  -> photo
        from above:  to position 1 + D, then to position 1  -> photo
     D is --approach. The commanded position of the two photos is the same,
     so how far the picture moved between them is the backlash.
  4. X goes back to position 0.

The shift of a photo against the reference is found by cross correlation
(the code of 24hrstest_shift.py). x is the place of the stage along +X in um,
against the slow reference; backlash = x from above - x from below, positive
when the stage stops short of the target (the usual sign). The mean and the
standard deviation over the rounds are printed and written at the end of the
CSV. The ruler is periodic: a backlash near a multiple of --tick-um may be a
wrong correlation peak, look at the photos then.

The moves are open loop (axis mode 0; the firmware's CORRECT mode would add
its own backlash feed-forward). The raw encoder count is only recorded, and
used as a guard: the run ends when an arrival is more than --max-counts
counts away from the first arrival from the same side (lost steps), or when
the scale lines are no longer in the picture (--min-corr). X stays where it
is then.

With --accel 100000 the axis only reaches 100000 steps/s after 50000 steps,
so with a shorter --approach the approach is slower than --speed: the peak
speed is printed before anything moves.

The PC is connected to the USB port of the CAN master, the X axis sits on a
CAN slave (--node, default 11): Axis and Camera are the ones of
24hrstest_can_rotational.py. The MVS client must not have the camera open.

Usage:
    uv run tools/PYTHON/encoder_test/test_backlash_x.py --port COM6
    uv run tools/PYTHON/encoder_test/test_backlash_x.py --port COM7 --approach 100000 --rounds 10

Output: one CSV row per photo of a round, the photos in <csv name>_photos.
Lines starting with '#' are run metadata and the result
(pandas: read_csv(..., comment='#')).
"""
import argparse
import csv
import importlib.util
import os
import statistics
import sys
import time
from datetime import datetime

# results (CSV, photos) go to <repo>/encoder_test_data, which git ignores
DATA_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "encoder_test_data"))


def load(name):
    """A script next to this file as a module (their names start with a digit)."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), name)
    spec = importlib.util.spec_from_file_location("m" + os.path.splitext(name)[0], path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def mean_std(values):
    return statistics.mean(values), statistics.stdev(values) if len(values) > 1 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="serial port of the CAN master")
    ap.add_argument("--baud", type=int, default=921600,
                    help="baud rate of the master's serial port")
    ap.add_argument("--node", type=int, default=11, help="CAN node id of the X slave")
    ap.add_argument("--sub", type=int, default=2,
                    help="SDO sub-index of the axis on the slave (X = 2)")
    ap.add_argument("--stepper", type=int, default=1,
                    help="stepperid the master routes to that slave (X = 1)")
    ap.add_argument("--step", type=int, default=100,
                    help="position 1 = position 0 + this many steps")
    ap.add_argument("--slow-speed", type=int, default=1000,
                    help="speed of the move to position 1 and back to position 0")
    ap.add_argument("--approach", type=int, default=20000,
                    help="distance D in steps the approaches start from, on both sides "
                         "of position 1 (must exceed the backlash)")
    ap.add_argument("--speed", type=int, default=100000, help="speed of the approaches")
    ap.add_argument("--accel", type=int, default=100000, help="acceleration for all moves")
    ap.add_argument("--rounds", type=int, default=10)
    ap.add_argument("--tick-um", type=float, default=10.0,
                    help="distance between two small ruler ticks in um")
    ap.add_argument("--px-per-tick", type=float, default=None,
                    help="tick period in px (default: measured in the photo at position 0)")
    ap.add_argument("--um-per-step", type=float, default=0.3125,
                    help="stage travel per motor step, to give the backlash in steps too")
    ap.add_argument("--min-corr", type=float, default=0.5,
                    help="lowest correlation with the reference that still counts as "
                         "'the scale lines are in the picture' (same picture = 1)")
    ap.add_argument("--max-counts", type=int, default=50,
                    help="the run ends when the encoder at an arrival is more than this "
                         "many counts off the first arrival from the same side")
    ap.add_argument("--gain", type=float, default=5.0, help="camera gain in dB")
    ap.add_argument("--exposure", type=float, default=None,
                    help="camera exposure time in microseconds (default: as set "
                         "in the camera)")
    ap.add_argument("--photo-format", choices=("bmp", "png", "jpg", "tif"), default="png")
    ap.add_argument("--photo-delay", type=float, default=2.0,
                    help="seconds to wait after the move before a photo")
    ap.add_argument("--out", default=None, help="CSV file (default: timestamped name in encoder_test_data/)")
    ap.add_argument("--yes", action="store_true", help="skip the travel-range confirmation")
    args = ap.parse_args()

    can, shift = load("24hrstest_can_rotational.py"), load("24hrstest_shift.py")
    D = args.approach
    out = args.out or os.path.join(DATA_DIR, f"backlash_x_{datetime.now():%Y%m%d_%H%M%S}.csv")
    photos = os.path.splitext(out)[0] + "_photos"

    camera = can.Camera(args.gain, args.exposure)  # before anything moves, so a camera problem shows up early
    ser = None

    def photo(name):
        """(path, gray image) of a photo taken --photo-delay after the move."""
        time.sleep(args.photo_delay)
        path = os.path.join(photos, f"{name}.{args.photo_format}")
        camera.photo(path)
        return path, shift.load_gray(path)

    try:
        ser = can.open_master(args.port, args.baud)
        time.sleep(2.0)
        axis = can.Axis(ser, args.node, args.sub, args.stepper)
        status = axis.status()
        print(f"axis status (node {args.node}): {status}")
        if status["isRunning"] or status["health"] == 2:
            raise SystemExit("X is running or has a latched fault: nothing was moved")
        p0 = status["position"]
        p1 = p0 + args.step
        peak = min(args.speed, int((args.accel * D) ** 0.5))
        print(f"position 0 = {p0}, position 1 = {p1}, {args.rounds} rounds")
        print(f"X will travel between {p1 - D} and {p1 + D} steps "
              f"(+- {D * args.um_per_step / 1000:.2f} mm around the scale lines), "
              f"peak speed of an approach = {peak} steps/s"
              + ("" if peak >= args.speed else f" (--speed {args.speed} needs --approach "
                                               f">= {args.speed ** 2 // args.accel})"))
        if not args.yes and input("inside the safe travel range? [y/N] ").strip().lower() != "y":
            raise SystemExit("aborted")
        if status["axismode"] != 0:
            print(f"axis mode {status['axismode']} -> 0 (open loop)")
            axis.set_mode(0)

        os.makedirs(photos, exist_ok=True)
        _, gray = photo("00_pos0")
        # before the first move: no ruler in the picture ends the run here
        px_per_tick = args.px_per_tick or shift.tick_period(gray)
        um_per_px = args.tick_um / px_per_tick
        pos0 = shift.Reference(gray)

        axis.move_to(p1, args.slow_speed, args.accel)
        ref_path, gray = photo("00_pos1")
        dx01, dy01, corr01 = pos0.shift(gray)
        ref = shift.Reference(gray)
        sign = 1 if dx01 >= 0 else -1  # +X steps move the picture in sign * x
        px_per_step, expected = abs(dx01) / args.step, args.um_per_step / um_per_px
        print(f"tick period = {px_per_tick:.2f} px = {args.tick_um:g} um -> {um_per_px:.4f} um/px")
        print(f"position 0 -> 1: {args.step} steps moved the picture by {dx01:+.2f} px "
              f"(dy {dy01:+.2f}, corr {corr01:.2f}) = {px_per_step:.3f} px per step, "
              f"expected {expected:.3f} at {args.um_per_step} um/step")
        if not 0.8 < px_per_step / expected < 1.2:
            print("    ! these do not agree: X did not come to position 0 in + (then the steps "
                  "took up the backlash first), a wrong correlation peak, or --tick-um / "
                  "--um-per-step is off. The sign of the backlash below may be wrong")

        with open(out, "w", newline="") as f:
            f.write(f"# test=backlash_x_optical port={args.port} node={args.node} "
                    f"started={datetime.now().isoformat(timespec='seconds')}\n")
            f.write(f"# pos0={p0} pos1={p1} approach={D} rounds={args.rounds} "
                    f"speed={args.speed} accel={args.accel} peak_speed={peak} "
                    f"slow_speed={args.slow_speed} photo_delay={args.photo_delay} "
                    f"camera={camera.settings}\n")
            f.write(f"# reference={ref_path} px_per_tick={px_per_tick:.3f} tick_um={args.tick_um:g} "
                    f"um_per_px={um_per_px:.5f} um_per_step={args.um_per_step} "
                    f"pos0_to_pos1_dx_px={dx01:.3f} px_per_step={px_per_step:.4f} "
                    f"x_sign={sign}\n")
            w = csv.writer(f)
            w.writerow(["time_iso", "round", "side", "pre", "target", "commanded", "rawCounts",
                        "dx_px", "dy_px", "x_um", "y_um", "corr", "photo"])
            print(f"\n{'round':>5} {'side':>6} {'dx [px]':>8} {'dy [px]':>8} {'x [um]':>8} "
                  f"{'y [um]':>8} {'corr':>5} {'raw':>8}  backlash [um]")

            seen = {"below": [], "above": []}  # side -> (x um, y um, raw count) per round
            for n in range(1, args.rounds + 1):
                for side, pre in (("below", p1 - D), ("above", p1 + D)):
                    axis.move_to(pre, args.speed, args.accel)
                    commanded, _, _, raw = axis.move_to(p1, args.speed, args.accel)
                    path, gray = photo(f"{n:02d}_{side}")
                    dx, dy, corr = ref.shift(gray)
                    x, y = sign * dx * um_per_px, dy * um_per_px
                    w.writerow([datetime.now().isoformat(timespec="milliseconds"), n, side, pre,
                                p1, commanded, raw, round(dx, 3), round(dy, 3), round(x, 4),
                                round(y, 4), round(corr, 4), path])
                    f.flush()
                    seen[side].append((x, y, raw))
                    print(f"{n:>5} {side:>6} {dx:>8.2f} {dy:>8.2f} {x:>8.2f} {y:>8.2f} "
                          f"{corr:>5.2f} {raw:>8}"
                          + (f"  {x - seen['below'][-1][0]:+.2f}" if side == "above" else ""))
                    if corr < args.min_corr:
                        raise SystemExit(f"correlation {corr:.2f} < {args.min_corr}: the scale "
                                         f"lines are not in the picture as in the reference "
                                         f"(lost steps?). X stays where it is")
                    if abs(raw - seen[side][0][2]) > args.max_counts:
                        raise SystemExit(f"encoder at {raw}, {seen[side][0][2]} at the first "
                                         f"arrival from {side}: lost steps. X stays where it is")

            def say(line):
                print(line)
                f.write(f"# {line}\n")

            below, above = seen["below"], seen["above"]
            bx, sx = mean_std([a[0] - b[0] for a, b in zip(above, below)])
            by, sy = mean_std([a[1] - b[1] for a, b in zip(above, below)])
            print()
            say(f"backlash x = {bx:+.2f} +- {sx:.2f} um = {bx / args.um_per_step:+.1f} +- "
                f"{sx / args.um_per_step:.1f} steps (mean +- std of {args.rounds} rounds, "
                f"from above - from below), y = {by:+.2f} +- {sy:.2f} um")
            for side, rows in seen.items():
                x, s = mean_std([r[0] for r in rows])
                say(f"from {side}: x = {x:+.2f} +- {s:.2f} um against the slow reference "
                    f"({min(r[0] for r in rows):+.2f} .. {max(r[0] for r in rows):+.2f})")
            say(f"encoder: from above - from below = "
                f"{statistics.mean(a[2] - b[2] for a, b in zip(above, below)):+.1f} counts")

        axis.move_to(p0, args.slow_speed, args.accel)  # back to where the run started
        print(f"\nsaved {out}, photos in {photos}")
    finally:
        camera.close()
        if ser:
            ser.close()


if __name__ == "__main__":
    sys.exit(main())
