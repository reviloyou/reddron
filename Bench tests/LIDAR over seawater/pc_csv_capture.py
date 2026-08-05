#!/usr/bin/env python3
"""
TFmini-S capture script. Refer to 5.08.2026 in README.md for specifics about bench test excecution

Runs on LAPTOP, not the ESP32. It reads the CSV lines that main.cpp prints
over USB, prompts you for the manual columns (angle, rig height, condition),
captures a fixed number of samples per point (default 200), and appends them to
one CSV file. The firmware streams continuously and never counts; the counting
to 200 lives here. 

Output CSV columns:
    wall_clock, burst_id, t_ms, distance_cm, strength, temp_c,
    angle_deg, rig_height_m, condition

The data is saved RAW. No dropout filtering, no classification. The per-burst
summary printed at the rig is only a live sanity check and never touches the
saved rows. Surface vs bottom vs specular gets decided later in the analysis.

Usage:
    pip install pyserial
    python capture.py                        # auto-detect port, save to tfmini_log.csv
    python capture.py --port COM5            # force a port (Windows)
    python capture.py --port /dev/ttyUSB0    # Linux
    python capture.py --out boresight.csv --samples 200
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime, timezone

import serial
from serial.tools import list_ports



BAUD = 115200
STRENGTH_MIN = 100      # Benewake reliability floor, used only for the live summary


def pick_port(preferred):
    """Return a serial port path. Use --port if given, else auto-detect or prompt."""
    if preferred:
        return preferred
    ports = list(list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Plug in the ESP32 and check the cable/driver.")
    if len(ports) == 1:
        print(f"Using the only serial port found: {ports[0].device} ({ports[0].description})")
        return ports[0].device
    print("Available serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    while True:
        choice = input("Pick a port number: ").strip()
        if choice.isdigit() and 0 <= int(choice) < len(ports):
            return ports[int(choice)].device
        print("Invalid choice, try again.")


def parse_line(line):
    """Parse one ESP32 CSV line 't_ms,distance_cm,strength,temp_c'.
    Returns (t_ms, distance_cm, strength, temp_c) or None if not a data line."""
    parts = line.strip().split(",")
    if len(parts) != 4:
        return None
    try:
        t_ms = int(parts[0])
        dist = int(parts[1])
        strength = int(parts[2])
        temp_c = float(parts[3])
    except ValueError:
        return None          # header line or garbage, skip
    return (t_ms, dist, strength, temp_c)


def capture_point(ser, n):
    """Read n valid samples from the serial port. Returns a list of tuples."""
    samples = []
    ser.reset_input_buffer()   # drop stale bytes so this point starts clean
    empty_reads = 0
    while len(samples) < n:
        raw = ser.readline().decode("ascii", errors="replace")
        if not raw:
            empty_reads += 1
            if empty_reads % 3 == 0:
                print("  ...no data. Is the sensor powered and the LED flashing?")
            continue
        empty_reads = 0
        parsed = parse_line(raw)
        if parsed is None:
            continue
        samples.append(parsed)
        if len(samples) % 25 == 0:
            print(f"  {len(samples)}/{n}", end="\r", flush=True)
    print(f"  {n}/{n} done      ")
    return samples


def summarize(samples):
    """Live sanity check only. Does not modify saved data."""
    n = len(samples)
    dists = [s[1] for s in samples]
    strengths = [s[2] for s in samples]
    dropouts = sum(1 for s in samples if s[2] < STRENGTH_MIN or s[1] == 0)
    valid = n - dropouts
    mean_d = sum(dists) / n
    std_d = (sum((d - mean_d) ** 2 for d in dists) / n) ** 0.5
    mean_s = sum(strengths) / n
    print(f"  valid returns: {valid}/{n} ({100 * valid / n:.1f}%)   dropouts: {dropouts}")
    print(f"  distance mean: {mean_d:.1f} cm   std: {std_d:.2f} cm   strength mean: {mean_s:.0f}")


def next_burst_id(path):
    """Continue burst numbering if the file already has data, else start at 0."""
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return 0
    try:
        with open(path, newline="") as rf:
            rows = list(csv.reader(rf))[1:]
            ids = [int(r[1]) for r in rows if len(r) > 1 and r[1].isdigit()]
            return max(ids) if ids else 0
    except Exception:
        return 0


def main():
    ap = argparse.ArgumentParser(description="TFmini-S Phase 0 capture")
    ap.add_argument("--port", default=None, help="serial port (auto-detect if omitted)")
    ap.add_argument("--out", default="tfmini_log.csv", help="output CSV path")
    ap.add_argument("--samples", type=int, default=200, help="samples per point")
    args = ap.parse_args()

    port = pick_port(args.port)
    print(f"Opening {port} at {BAUD} baud...")
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}")

    time.sleep(1.5)            # let the ESP32 finish its auto-reset and boot
    ser.reset_input_buffer()

    fieldnames = ["wall_clock", "burst_id", "t_ms", "distance_cm", "strength",
                  "temp_c", "angle_deg", "rig_height_m", "condition"]
    new_file = not os.path.exists(args.out) or os.path.getsize(args.out) == 0
    burst_id = next_burst_id(args.out)

    f = open(args.out, "a", newline="")
    writer = csv.writer(f)
    if new_file:
        writer.writerow(fieldnames)
        f.flush()

    print(f"\nLogging to {args.out}. Ctrl+C to quit.\n")

    try:
        while True:
            print("--- New measurement point ---")
            angle = input("  angle (deg): ").strip()
            height = input("  rig height (m): ").strip()
            condition = input("  condition (e.g. wall / calm_deep / calm_shallow / ripples): ").strip()

            burst_id += 1
            print(f"Capturing {args.samples} samples for burst {burst_id}...")
            samples = capture_point(ser, args.samples)

            wall = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            for (t_ms, dist, strength, temp_c) in samples:
                writer.writerow([wall, burst_id, t_ms, dist, strength,
                                 f"{temp_c:.1f}", angle, height, condition])
            f.flush()

            summarize(samples)
            print(f"  saved burst {burst_id} to {args.out}\n")

            again = input("Another point? [Y/n]: ").strip().lower()
            if again in ("n", "no"):
                break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        f.close()
        ser.close()
        print("Port closed. File saved.")


if __name__ == "__main__":
    main()