#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["numpy>=2", "matplotlib", "pillow"]
# ///
"""
Optical check of a 24hrstest_can_*.py run: how far did the picture move?

The test saves a photo at every visit, in <csv name>_photos/scale and
<csv name>_photos/sample. For each of the two places the first photo is the
reference and every later photo is compared with it: the shift is found by
cross correlation of the two images (FFT, sub-pixel peak by a parabola fit),
the same way as in xmas_tree_shift.ipynb. dx, dy = how far the image content
moved (positive = right / down in the photo).

The pixel size comes from the tick period of the horizontal ruler in the
first scale photo (--tick-um is the distance between two small ticks, CHECK
YOUR SLIDE); --px-per-tick sets the period by hand if the automatic one is
wrong. A shift close to a multiple of the tick period in a scale photo may
be a wrong correlation peak.

The shifts are joined with the rows of the run (encoder residual, lost-step
events). The visits at which lost steps were compensated are marked: in the
table (event column), in the CSV and in the plots (red lines, labelled with
the visit and the compensated steps; the photo of that visit is taken right
after the recovery, the next scale photo is the first return after it).
The summary also compares every photo with the photo before and the photo
after it at the same place: that leaves out the slow drift, so it shows
whether a recovery lands somewhere else than a normal visit does.

Two plots are saved:
  <csv name>_shift.png          dx and dy of both places, and the encoder
                                residual of the run;
  <csv name>_shift_encoder.png  what the magnetic encoder reports against
                                what the photo shows: dcount (raw count
                                minus the count at the first photo of the
                                place, left axis) and the optical dx in um
                                (right axis). The two axes share the zero
                                and are scaled with --um-per-count, so the
                                two curves lie on each other where the
                                encoder is right.

Correlating the photos takes a few minutes; --replot reads the shifts from
<csv name>_shift.csv again and only makes the summary and the plots.

Usage:
    uv run tools/PYTHON/encoder_test/24hrstest_shift.py encoder_test_data/24hrstest_can_x_20261003_110333.csv
    uv run tools/PYTHON/encoder_test/24hrstest_shift.py run.csv --tick-um 10 --px-per-tick 82.8
    uv run tools/PYTHON/encoder_test/24hrstest_shift.py run.csv --replot

Output: the table is printed and saved as <csv name>_shift.csv.
"""
import argparse
import csv
import os
import sys
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

PLACES = ("scale", "sample")
IMAGE_EXT = (".png", ".jpg", ".jpeg", ".tif", ".tiff", ".bmp")
# AS5311: one pole pair of the magnetic strip is 2 mm = 1024 counts
UM_PER_COUNT = 2000 / 1024


def load_gray(path):
    return np.asarray(Image.open(path).convert("L"), dtype=np.float32)


def tick_period(im, min_lag=5):
    """Period of the small ruler ticks in px: autocorrelation of the intensity
    profile along x, over the rows that cross the most vertical lines."""
    edges = np.abs(np.diff(im, axis=1)).sum(axis=1)
    edges -= np.median(edges)
    prof = im[edges > 0.5 * edges.max()].mean(axis=0)
    prof = (prof - prof.mean()) * np.hanning(prof.size)
    ac = np.fft.irfft(np.abs(np.fft.rfft(prof, 2 * prof.size)) ** 2)[:prof.size // 2]
    ac /= ac[0]
    peaks = [i for i in range(min_lag, ac.size - 1)
             if ac[i] > ac[i - 1] and ac[i] >= ac[i + 1] and ac[i] > 0.3]
    if not peaks:
        raise SystemExit("no ruler ticks found in the first scale photo, set --px-per-tick")
    # the peaks at 2, 3, ... periods give the period more exactly: follow them
    # as long as the next one is where it should be
    n, i = 1, peaks[0]
    for p in peaks[1:]:
        if abs(p - (n + 1) * i / n) > 0.1 * peaks[0]:
            break
        n, i = n + 1, p
    # parabola through the 3 points around the peak
    return (i + 0.5 * (ac[i - 1] - ac[i + 1]) / (ac[i - 1] - 2 * ac[i] + ac[i + 1])) / n


class Reference:
    """The first photo of a place; shift() compares another photo with it."""

    def __init__(self, ref, highpass_px=100):
        self.shape = h, w = ref.shape
        self.win = np.outer(np.hanning(h), np.hanning(w)).astype(np.float32)
        # drop the slow background (vignetting does not move with the stage)
        fy = np.fft.fftfreq(h)[:, None]
        fx = np.fft.rfftfreq(w)[None, :]
        self.keep = np.hypot(fx, fy) >= 1 / highpass_px
        self.A = self.spectrum(ref)
        self.energy = self.sum_sq(self.A)

    def spectrum(self, im):
        return np.fft.rfft2((im - im.mean()) * self.win) * self.keep

    def sum_sq(self, F):
        """Sum of the squared pixels of the filtered image (Parseval; the
        half spectrum of rfft2 holds every column but the first and the
        Nyquist one twice)."""
        p = np.abs(F) ** 2
        h, w = self.shape
        full = 2 * p.sum() - p[:, 0].sum() - (p[:, -1].sum() if w % 2 == 0 else 0)
        return full / (h * w)

    def shift(self, img):
        """(dx, dy, corr) of img relative to the reference; corr = height of
        the correlation peak, 1 = the same picture, only shifted."""
        if img.shape != self.shape:
            raise SystemExit(f"photos must all have the same size: {img.shape} / {self.shape}")
        h, w = self.shape
        B = self.spectrum(img)
        cc = np.fft.irfft2(np.conj(self.A) * B, s=self.shape)
        py, px = np.unravel_index(np.argmax(cc), cc.shape)

        def subpixel(cm, c0, cp):
            return 0.5 * (cm - cp) / (cm - 2 * c0 + cp)

        dy = py + subpixel(cc[py - 1, px], cc[py, px], cc[(py + 1) % h, px])
        dx = px + subpixel(cc[py, px - 1], cc[py, px], cc[py, (px + 1) % w])
        if dy > h / 2:
            dy -= h
        if dx > w / 2:
            dx -= w
        return float(dx), float(dy), float(cc[py, px] / np.sqrt(self.energy * self.sum_sq(B)))


def num(value):
    """A CSV field as a number, None if it is empty."""
    return float(value) if value not in ("", None) else None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv", help="CSV file of the 24hrstest_can_*.py run")
    ap.add_argument("--photos", default=None,
                    help="folder with scale/ and sample/ (default: <csv name>_photos)")
    ap.add_argument("--tick-um", type=float, default=10.0,
                    help="distance between two small ruler ticks in um")
    ap.add_argument("--px-per-tick", type=float, default=None,
                    help="tick period in px (default: measured in the first scale photo)")
    ap.add_argument("--highpass", type=float, default=100,
                    help="structures larger than this many px are ignored")
    ap.add_argument("--um-per-count", type=float, default=UM_PER_COUNT,
                    help="um per encoder count, only used to scale the two axes of the "
                         "encoder plot against each other (default: AS5311, 2 mm / 1024)")
    ap.add_argument("--replot", action="store_true",
                    help="do not look at the photos: read the shifts from the CSV of an "
                         "earlier run of this script, only the summary and the plots are made")
    ap.add_argument("--out", default=None, help="CSV file (default: <csv name>_shift.csv)")
    args = ap.parse_args()

    base = os.path.splitext(args.csv)[0]
    photos = args.photos or base + "_photos"
    out = args.out or base + "_shift.csv"
    with open(args.csv, newline="") as f:
        run = {int(r["visit"]): r for r in csv.DictReader(l for l in f if not l.startswith("#"))}
    t_start = datetime.fromisoformat(min(r["time_iso"] for r in run.values()))
    events = sorted(v for v, r in run.items() if r.get("event"))
    print(f"compensated lost-step events: {len(events)} (visits {', '.join(map(str, events))})")

    def measure():
        """Every photo against the reference of its place, in the order of the
        run. Prints the table, writes the CSV, returns (rows, um per px)."""
        files = {}
        for place in PLACES:
            folder = os.path.join(photos, place)
            if not os.path.isdir(folder):
                raise SystemExit(f"no photos in {folder}")
            # file names start with the visit: 0019_sample_<time>.png
            files[place] = sorted((int(n.split("_")[0]), os.path.join(folder, n))
                                  for n in os.listdir(folder) if n.lower().endswith(IMAGE_EXT))
            if len(files[place]) < 2:
                raise SystemExit(f"need at least 2 photos in {folder}")

        refs = {place: load_gray(files[place][0][1]) for place in PLACES}
        period = tick_period(refs["scale"])
        px_per_tick = args.px_per_tick or period
        um_per_px = args.tick_um / px_per_tick
        print(f"tick period = {period:.2f} px (used: {px_per_tick:.2f}), one tick = "
              f"{args.tick_um:g} um -> {um_per_px:.4f} um/px")
        for place in PLACES:
            print(f"{place}: {len(files[place])} photos, reference = "
                  f"{os.path.basename(files[place][0][1])}")
        print()
        print(f"{'time':>14} {'visit':>5} {'place':>7} {'dx [px]':>8} {'dy [px]':>8} "
              f"{'dx [um]':>8} {'dy [um]':>8} {'corr':>5} {'resid':>5}  event")

        refs = {place: Reference(ref, args.highpass) for place, ref in refs.items()}
        result = []
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_iso", "hours", "visit", "place", "dx_px", "dy_px", "dx_um", "dy_um",
                        "corr", "rawCounts", "residualCounts", "compSteps", "offsetSteps", "event",
                        "lostSteps", "afterEvent", "photo"])
            for visit, place, path in sorted((v, p, path) for p in PLACES for v, path in files[p]):
                row = run.get(visit)
                if row is None or row["place"] != place:
                    print(f"    ! {os.path.basename(path)}: no row for this visit in {args.csv}")
                    continue
                dx, dy, corr = refs[place].shift(load_gray(path))
                t = datetime.fromisoformat(row["time_iso"])
                # the scale photo after an event is the first return after the recovery
                after = visit - 1 if visit - 1 in events else ""
                r = dict(visit=visit, place=place, hours=(t - t_start).total_seconds() / 3600,
                         dx=dx * um_per_px, dy=dy * um_per_px, corr=corr,
                         raw=num(row["rawCounts"]), resid=num(row["residualCounts"]),
                         event=visit in events, after=after != "")
                result.append(r)
                note = (f"{row['event']}, {num(row['compSteps']):+.0f} steps compensated"
                        if r["event"] else f"first return after visit {after}" if r["after"] else "")
                print(f"{t:%m-%d %H:%M:%S} {visit:>5} {place:>7} {dx:>8.2f} {dy:>8.2f} "
                      f"{r['dx']:>8.2f} {r['dy']:>8.2f} {corr:>5.2f} {row['residualCounts']:>5}  "
                      f"{note}")
                w.writerow([row["time_iso"], round(r["hours"], 4), visit, place, round(dx, 3),
                            round(dy, 3), round(r["dx"], 4), round(r["dy"], 4), round(corr, 4),
                            row["rawCounts"], row["residualCounts"], row["compSteps"],
                            row["offsetSteps"], row.get("event", ""), row.get("lostSteps", ""),
                            after, path])
                f.flush()
        return result, um_per_px

    def read_shifts():
        """The rows of an earlier run of this script, from its CSV."""
        if not os.path.exists(out):
            raise SystemExit(f"--replot: {out} does not exist, run without --replot first")
        with open(out, newline="") as f:
            rows = list(csv.DictReader(f))
        result = [dict(visit=int(s["visit"]), place=s["place"], hours=float(s["hours"]),
                       dx=float(s["dx_um"]), dy=float(s["dy_um"]), corr=float(s["corr"]),
                       raw=num(s["rawCounts"]), resid=num(s["residualCounts"]),
                       event=bool(s["event"]), after=s["afterEvent"] != "") for s in rows]
        scale = [float(s["dx_um"]) / float(s["dx_px"]) for s in rows if abs(float(s["dx_px"])) > 1]
        print(f"shifts read from {out}")
        return result, float(np.median(scale)) if scale else float("nan")

    result, um_per_px = read_shifts() if args.replot else measure()

    print()
    for place in PLACES:
        rs = [r for r in result if r["place"] == place]
        # without the slow drift: every photo against the mean of the photo
        # before and the photo after it at the same place
        for i, r in enumerate(rs):
            near = rs[max(i - 1, 0):i] + rs[i + 1:i + 2]
            r["ddx"], r["ddy"] = (r[a] - np.mean([q[a] for q in near]) for a in ("dx", "dy"))
        marked = "event" if place == "sample" else "after"
        groups = {"all": rs,
                  "after a lost-step recovery" if place == "sample"
                  else "first return after a recovery": [r for r in rs if r[marked]],
                  "the others": [r for r in rs if not r[marked]]}
        print(f"{place}: shift against the first photo [um]")
        for name, g in groups.items():
            if g:
                x, y = np.array([r["dx"] for r in g]), np.array([r["dy"] for r in g])
                print(f"  {name:<30} n = {len(g):>3}  dx {x.mean():+6.2f} +- {x.std():.2f} "
                      f"({x.min():+.2f} .. {x.max():+.2f})  dy {y.mean():+6.2f} +- {y.std():.2f} "
                      f"({y.min():+.2f} .. {y.max():+.2f})")
        print(f"{place}: against the photos before and after it (no slow drift) [um]")
        for name, g in list(groups.items())[1:]:
            if g:
                x, y = np.array([r["ddx"] for r in g]), np.array([r["ddy"] for r in g])
                print(f"  {name:<30} n = {len(g):>3}  dx rms {np.sqrt((x ** 2).mean()):.2f} "
                      f"(max {np.abs(x).max():.2f})  dy rms {np.sqrt((y ** 2).mean()):.2f} "
                      f"(max {np.abs(y).max():.2f})")
        for r in rs:
            if r[marked]:
                print(f"    visit {r['visit']:>3}: dx {r['ddx']:+.2f}  dy {r['ddy']:+.2f}")
        # what the encoder saw of it: the residual of the run against the optical shift
        g = [r for r in rs if r["resid"] is not None]
        c = np.array([r["resid"] for r in g])
        if len(g) > 2 and c.std() > 0:
            for axis in ("dx", "dy"):
                v = np.array([r[axis] for r in g])
                slope = np.polyfit(c, v, 1)[0]
                print(f"  {axis} against the encoder residual: {slope:+.3f} um / count, "
                      f"r = {np.corrcoef(c, v)[0, 1]:+.2f}")
        low = [r["visit"] for r in rs if r["corr"] < 0.5 * np.median([q["corr"] for q in rs])]
        if low:
            print(f"  ! low correlation (out of focus / wrong peak?) at visits {low}")

    def mark_events(axes):
        """A red line at every compensated lost-step event in all axes,
        labelled 'visit: compensated steps' above the first one."""
        for visit in events:
            hours = (datetime.fromisoformat(run[visit]["time_iso"])
                     - t_start).total_seconds() / 3600
            for ax in axes:
                ax.axvline(hours, color="red", ls="--", lw=0.8, alpha=0.7)
            axes[0].annotate(f"{visit}: {num(run[visit]['compSteps']):+.0f}", (hours, 1.02),
                             xycoords=("data", "axes fraction"), ha="center", va="bottom",
                             fontsize=7, color="red", rotation=90)
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.axhline(0, color="k", lw=0.5)
        axes[-1].set_xlabel(f"hours since {t_start:%Y-%m-%d %H:%M}")

    def ring_label(place):
        return "after a lost-step recovery" if place == "sample" else "first return after a recovery"

    fig, axes = plt.subplots(3, 1, figsize=(13, 9.5), sharex=True)
    for ax, place in zip(axes, PLACES):
        rs = [r for r in result if r["place"] == place]
        marked = [r for r in rs if r["event" if place == "sample" else "after"]]
        for axis, color in (("dx", "tab:blue"), ("dy", "tab:orange")):
            ax.plot([r["hours"] for r in rs], [r[axis] for r in rs], ".-", color=color,
                    lw=0.8, label=axis)
            ax.plot([r["hours"] for r in marked], [r[axis] for r in marked], "o", mfc="none",
                    mec="red", ms=9, label=ring_label(place) if axis == "dx" else None)
        ax.set_ylabel(f"{place}: shift [um]")
        ax.legend(loc="upper left", ncol=3, fontsize=8)
    for place, color in zip(PLACES, ("tab:green", "tab:purple")):
        rs = [r for r in result if r["place"] == place and r["resid"] is not None]
        axes[2].plot([r["hours"] for r in rs], [r["resid"] for r in rs], ".-", color=color,
                     lw=0.8, label=place)
    axes[2].set_ylabel("encoder residual [counts]")
    axes[2].legend(loc="upper left", ncol=2, fontsize=8)
    mark_events(axes)
    fig.suptitle(f"{os.path.basename(args.csv)}: shift of the photos against the first one "
                 f"({um_per_px:.4f} um/px), red = {len(events)} compensated lost-step events "
                 f"(visit: steps)", fontsize=10, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(base + "_shift.png", dpi=130)

    # the encoder against the photo: dcount on the left, dx on the right axis
    k = args.um_per_count
    fig, axes = plt.subplots(2, 1, figsize=(13, 7.5), sharex=True)
    for ax, place in zip(axes, PLACES):
        rs = [r for r in result if r["place"] == place and r["raw"] is not None]
        marked = [r for r in rs if r["event" if place == "sample" else "after"]]
        hours = [r["hours"] for r in rs]
        dcount = [r["raw"] - rs[0]["raw"] for r in rs]  # against the first photo, as dx is
        ax.plot(hours, dcount, "s-", color="tab:green", ms=3.5, lw=0.8,
                label="magnetic encoder: dcount")
        ax.set_ylabel(f"{place}: encoder dcount [counts]", color="tab:green")
        ax2 = ax.twinx()
        ax2.plot(hours, [r["dx"] for r in rs], ".-", color="tab:blue", lw=0.8,
                 label="optical: dx")
        ax2.plot([r["hours"] for r in marked], [r["dx"] for r in marked], "o", mfc="none",
                 mec="red", ms=9, label=ring_label(place))
        ax2.set_ylabel(f"{place}: optical dx [um]", color="tab:blue")
        # the same zero and 1 count = k um on both axes
        top = 1.15 * max(max(abs(c) for c in dcount) * k, max(abs(r["dx"]) for r in rs))
        ax.set_ylim(-top / k, top / k)
        ax2.set_ylim(-top, top)
        lines = ax.get_legend_handles_labels(), ax2.get_legend_handles_labels()
        ax.legend(lines[0][0] + lines[1][0], lines[0][1] + lines[1][1], loc="upper left",
                  ncol=3, fontsize=8)
    mark_events(axes)
    fig.suptitle(f"{os.path.basename(args.csv)}: magnetic encoder (dcount) against the optical "
                 f"position (dx)\naxes scaled 1 count = {k:.3f} um, red = {len(events)} "
                 f"compensated lost-step events (visit: steps)", fontsize=10, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(base + "_shift_encoder.png", dpi=130)
    print(f"\nsaved {out}, {base}_shift.png and {base}_shift_encoder.png")


if __name__ == "__main__":
    sys.exit(main())
