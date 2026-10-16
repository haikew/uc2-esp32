#!/usr/bin/env python3
# /// script
# dependencies = ["pyserial>=3.5"]
# ///
"""
Move a stepper back and forth through the MASTER and record the collision
sensor log of the GPIO board (node 60, debug build) at the same time.

Change the settings below, then run:
    uv run tools/PYTHON/collision/collision_go_back.py

Output in collision_test_data/ next to this script:
    collision_go_back_<time>.csv  one row per sample ("GpioCanSlave loop:" line)
    collision_go_back_<time>.log  every line the board sent, with the PC time
leg = 0 before the first move, then 1, 2, ... (odd = go, even = back).
"""
import json
import os
import time
from datetime import datetime

import serial

# ---- settings ---------------------------------------------------------------
MASTER_PORT = "COM6"   # master: sends the moves
GPIO_PORT = "COM9"     # GPIO board (node 60): sends the sensor log
STEPPER = 0            # 0 = A
STEPS = 10000          # distance of one leg
SPEED = 100000
ACCEL = 1000000
CYCLES = 20            # one cycle = go + back
WAIT = 0.2             # seconds per leg (the move + a pause)
# ------------------------------------------------------------------------------

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "collision_test_data")


def open_port(port, baud):
    """Open without pulsing DTR / RTS, which would reset the ESP32."""
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.1
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def read_for(seconds, gpio, csv_file, log_file, t0, leg):
    """Read the GPIO board for `seconds` and save what it sends."""
    t_end = time.time() + seconds
    while time.time() < t_end:
        line = gpio.readline().decode(errors="ignore").strip()
        if line == "" or "gpio: GPIO[" in line:  # nothing, or analogRead's line
            continue
        t = round(time.time() - t0, 3)
        log_file.write(f"{t} {line}\n")

        if "GpioCanSlave loop:" in line:
            # "[146475][I][...] ... filt=4088 raw=4095 base=4056 ..." -> numbers
            board_ms = line[1:line.index("]")].strip()
            v = dict(word.split("=") for word in line.split() if "=" in word)
            csv_file.write(f"{t},{board_ms},{v['raw']},{v['filt']},{v['base']},"
                           f"{v['sigma']},{v['dev']},{v['trip']},{leg}\n")
        elif "TRIP" in line or "cleared" in line:
            print(f"  leg {leg}: {line}")


def main():
    os.makedirs(DATA_DIR, exist_ok=True)
    name = os.path.join(DATA_DIR, f"collision_go_back_{datetime.now():%Y%m%d_%H%M%S}")
    master = open_port(MASTER_PORT, 921600)
    gpio = open_port(GPIO_PORT, 115200)
    gpio.reset_input_buffer()

    csv_file = open(name + ".csv", "w")
    log_file = open(name + ".log", "w")
    csv_file.write("t_pc_s,t_board_ms,raw,filtered,baseline,sigma,deviation,trip,leg\n")
    t0 = time.time()

    leg = 0
    read_for(WAIT, gpio, csv_file, log_file, t0, leg)  # quiet, before the first move
    for cycle in range(1, CYCLES + 1):
        for steps in (STEPS, -STEPS):  # go, then back
            leg += 1
            move = {"task": "/motor_act", "motor": {"steppers": [{
                "stepperid": STEPPER, "position": steps, "speed": SPEED,
                "acceleration": ACCEL, "accel": ACCEL, "isabs": 0}]}}
            master.write((json.dumps(move) + "\n").encode())
            read_for(WAIT, gpio, csv_file, log_file, t0, leg)
        print(f"cycle {cycle}/{CYCLES} done")
    read_for(WAIT, gpio, csv_file, log_file, t0, leg)  # quiet, after the last move

    csv_file.close()
    log_file.close()
    master.close()
    gpio.close()
    print(f"saved {name}.csv and {name}.log")


if __name__ == "__main__":
    main()
