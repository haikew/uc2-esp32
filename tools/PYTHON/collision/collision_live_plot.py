#!/usr/bin/env python3
# /// script
# dependencies = ["pyserial>=3.5", "matplotlib"]
# ///
"""
Live plot of the collision sensor's raw count from the GPIO board
(node 60, debug build). The plot shows the last WINDOW seconds.
Close the window or press Ctrl+C to stop.

With SAVE = True every sample since the start goes to
collision_test_data/collision_live_plot_<time>.csv.

Change the port below, then run:
    uv run tools/PYTHON/collision/collision_live_plot.py
"""
import os
import time
from datetime import datetime

import matplotlib.pyplot as plt
import serial

# settings
GPIO_PORT = "COM9"     # GPIO board (node 60)
WINDOW = 30           # how many seconds to show in the plot
SAVE = True           # True: save all samples to CSV

# save CSV in a subfolder of this script's folder
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "collision_test_data")

# open without pulsing DTR / RTS, which would reset the ESP32
gpio = serial.Serial()
gpio.port, gpio.baudrate, gpio.timeout = GPIO_PORT, 115200, 0.1
gpio.dtr = False
gpio.rts = False
gpio.open()
gpio.reset_input_buffer()

if SAVE:
    os.makedirs(DATA_DIR, exist_ok=True)
    csv_name = os.path.join(DATA_DIR, f"collision_live_plot_{datetime.now():%Y%m%d_%H%M%S}.csv")
    csv_file = open(csv_name, "w")
    csv_file.write("t_pc_s,t_board_ms,raw,filtered,baseline,sigma,deviation,trip\n")

# live plot
plt.ion()
fig, ax = plt.subplots(figsize=(10, 4))
curve, = ax.plot([], [])
ax.set_xlabel("time [s]")
ax.set_ylabel("raw count")
ax.grid(alpha=0.3)

# trip light in the top-right corner: grey = no trip value yet, green = no trip, red = trip
light = ax.text(0.98, 0.95, "trip ?", transform=ax.transAxes, ha="right", va="top",
                color="white", fontweight="bold", fontsize=14,
                bbox=dict(boxstyle="round,pad=0.5", facecolor="grey", edgecolor="none"))

times = []
raws = []
trip = None        # last trip value from the board: None (not seen yet), False or True
trip_seen = False  # any trip=1 since the last redraw, so a short trip still lights up
t0 = time.time()
last_draw = 0

try:
    while plt.fignum_exists(fig.number):  # until the window is closed
        line = gpio.readline().decode(errors="ignore")
        if "GpioCanSlave loop:" in line:
            # "... filt=4088 raw=4095 base=4056 ..." -> {"filt": "4088", "raw": "4095", ...}
            v = dict(word.split("=", 1) for word in line.split() if "=" in word)
            t = round(time.time() - t0, 3)

            if SAVE:
                # a cut-off or mixed-up line can miss fields; those cells stay empty
                board_ms = line[1:line.index("]")].strip() if "]" in line else ""  # "[146475][I]..." -> "146475"
                csv_file.write(f"{t},{board_ms},{v.get('raw', '')},{v.get('filt', '')},{v.get('base', '')},"
                               f"{v.get('sigma', '')},{v.get('dev', '')},{v.get('trip', '')}\n")

            if v.get("trip") in ("0", "1"):
                trip = v["trip"] == "1"
                trip_seen = trip_seen or trip

            # no raw count -> nothing to plot, the CSV row above keeps the rest
            if not v.get("raw", "").isdigit():
                print(f"{t:7.2f} s   no raw count: {line.strip()}")
                continue

            times.append(t)
            raws.append(int(v["raw"]))
            print(f"{t:7.2f} s   raw = {raws[-1]}")

            # the plot only needs the last WINDOW seconds; the CSV has the rest
            while times[0] < t - WINDOW:
                times.pop(0)
                raws.pop(0)

        # redraw 5 times per second
        if raws and time.time() - last_draw > 0.2:
            curve.set_data(times, raws)
            ax.set_xlim(max(0, times[-1] - WINDOW), max(WINDOW, times[-1]))
            ax.relim()
            ax.autoscale_view(scalex=False)  # y follows what is on screen
            ax.set_title(f"raw = {raws[-1]}")
            if trip is not None:
                light.set_text("TRIP" if trip_seen else "no trip")
                light.get_bbox_patch().set_facecolor("red" if trip_seen else "green")
                trip_seen = trip  # still tripped -> stays red at the next redraw
            plt.pause(0.01)
            last_draw = time.time()
except KeyboardInterrupt:
    pass

gpio.close()
if SAVE:
    csv_file.close()
    print(f"saved {csv_name}")
