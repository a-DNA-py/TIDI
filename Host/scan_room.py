"""
Capture a LiDAR scan from tidi_esp32 over USB, or re-plot a saved CSV, and build a rough 3D model.

Install:  pip3 install -r Host/requirements.txt

Usage:
  python3 scan_room.py --port /dev/cu.<port> --out <name>.csv   # scan over USB, then plot
  python3 scan_room.py --file <name>.csv                             # re-plot a saved scan

CSV columns: az (deg from the azimuth servo's centre, + anticlockwise), small (tilt angle: 90 = level, 0 = straight up), dist_cm, strength.
Outputs: the CSV and room.obj (mesh, opens in Blender / any 3D viewer).
Close the Serial Monitor before running (the port can only be used by one program).
"""
import argparse
import csv
import math
import time

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

FLIP_AZ = False   # set True if the model comes out mirrored
# Where the TF-Luna's beam starts, in the tilting head, from the MG90S axis.
ARM_M = 0.03      # along the arm, at right angles to the beam: up when the head is level
FWD_M = 0.01      # along the line of sight


def run_scan(port, out_csv):
    import serial
    ser = serial.Serial(port, 115200, timeout=10)
    time.sleep(2)
    ser.reset_input_buffer()
    ser.write(b"s")
    rows = []
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["az", "small", "dist_cm", "strength"])
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line == "END":
                break
            parts = line.split(",")
            if len(parts) != 4:
                print(line)
                continue
            try:
                r = [int(x) for x in parts]
            except ValueError:
                continue
            rows.append(r)
            w.writerow(r)
            print(f"az {r[0]:4d}  small {r[1]:3d}  dist {r[2]:4d} cm   ({len(rows)} points)")
    ser.close()
    print(f"Saved {len(rows)} points to {out_csv}")
    return rows


def load_csv(path):
    with open(path) as f:
        rd = csv.reader(f)
        next(rd)
        return [[int(x) for x in r] for r in rd if len(r) == 4]


def build_model(rows, max_jump=0.3):
    """Convert (az, small, dist) to XYZ and connect neighbouring grid points into quads."""
    azs = sorted({r[0] for r in rows})
    sms = sorted({r[1] for r in rows})
    ai = {a: i for i, a in enumerate(azs)}
    si = {s: j for j, s in enumerate(sms)}
    P = np.full((len(azs), len(sms), 3), np.nan)
    D = np.full((len(azs), len(sms)), np.nan)

    for az, sm, d, _ in rows:
        if d <= 0:
            continue
        r = d / 100.0 + FWD_M                 # metres, from the arm's end
        a = math.radians(-az if FLIP_AZ else az)
        e = math.radians(90 - sm)             # small=90 -> horizon, 0 -> straight up (tidi_esp32's tilt angle)
        h = r * math.cos(e) - ARM_M * math.sin(e)   # origin on the MG90S axis
        v = r * math.sin(e) + ARM_M * math.cos(e)
        P[ai[az], si[sm]] = [h * math.cos(a), h * math.sin(a), v]
        D[ai[az], si[sm]] = r

    faces = []
    na, ns = len(azs), len(sms)
    wrap = (azs[-1] - azs[0]) >= 350          # join -175 and 180 columns
    for i in range(na if wrap else na - 1):
        i2 = (i + 1) % na
        for j in range(ns - 1):
            quad = [(i, j), (i2, j), (i2, j + 1), (i, j + 1)]
            ds = [D[q] for q in quad]
            if np.isnan(ds).any():
                continue
            if max(ds) / min(ds) > 1 + max_jump:   # don't bridge depth jumps (edges, doorways)
                continue
            faces.append([P[q] for q in quad])

    pts = P[~np.isnan(D)]
    return pts, faces


def save_obj(faces, path):
    with open(path, "w") as f:
        n = 1
        for quad in faces:
            for v in quad:
                f.write(f"v {v[0]:.3f} {v[1]:.3f} {v[2]:.3f}\n")
            f.write(f"f {n} {n + 1} {n + 2} {n + 3}\n")
            n += 4
    print(f"Saved mesh ({len(faces)} faces) to {path}")


def plot(pts, faces):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection="3d")
    if faces:
        ax.add_collection3d(Poly3DCollection(faces, alpha=0.45, facecolor="lightsteelblue",
                                             edgecolor="gray", linewidths=0.2))
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=pts[:, 2], cmap="viridis", s=5)
    ax.scatter([0], [0], [0], c="red", s=60, marker="^", label="scanner")
    lim = np.abs(pts).max() if len(pts) else 1
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title(f"Room scan: {len(pts)} points")
    ax.legend()
    plt.show()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="the ESP32's serial port, from `arduino-cli board list`")
    ap.add_argument("--file", help="Load an existing CSV instead of scanning")
    ap.add_argument("--out", default="scan.csv")
    args = ap.parse_args()

    if args.file:
        rows = load_csv(args.file)
    elif args.port:
        rows = run_scan(args.port, args.out)
    else:
        ap.error("give --port to scan or --file to load")

    pts, faces = build_model(rows)
    save_obj(faces, "room.obj")
    plot(pts, faces)