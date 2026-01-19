#!/usr/bin/env python3
import re
import time
from collections import deque

import serial
import matplotlib.pyplot as plt

# ===================== Serial settings =====================
PORT = "/dev/ttyUSB0"
BAUD = 115200
TIMEOUT = 0.2

# ===================== Plot settings =====================
HISTORY = 200           # number of points to keep
UPDATE_EVERY = 1        # update plot every N samples
AXIS_LIMITS = None      # set to e.g. (-2, 2, -2, 2, 0, 5) for fixed axes [xmin,xmax,ymin,ymax,zmin,zmax]

# Expected line format from ESP32:
# "X,Y,Z = 0.123 0.045 1.678 (m)"
PAT = re.compile(r"X\s*,\s*Y\s*,\s*Z\s*=\s*([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)")

def main():
    print(f"Opening serial {PORT} @ {BAUD} ...")
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)

    # Flush any boot garbage
    ser.reset_input_buffer()

    xs = deque(maxlen=HISTORY)
    ys = deque(maxlen=HISTORY)
    zs = deque(maxlen=HISTORY)

    # Matplotlib interactive 3D plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("IR Marker Position (ESP32 Stereo)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # Create artists once
    trail_line, = ax.plot([], [], [], linewidth=1)   # trail
    point_scatter = ax.scatter([], [], [], s=40)      # current point

    sample_count = 0
    last_plot_t = time.time()

    def refresh_plot():
        ax.cla()
        ax.set_title("IR Marker Position (ESP32 Stereo)")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")

        if len(xs) == 0:
            return

        # Trail + current point
        ax.plot(xs, ys, zs, linewidth=1)
        ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], s=60)

        # Axis limits
        if AXIS_LIMITS is not None:
            xmin, xmax, ymin, ymax, zmin, zmax = AXIS_LIMITS
            ax.set_xlim(xmin, xmax)
            ax.set_ylim(ymin, ymax)
            ax.set_zlim(zmin, zmax)
        else:
            # Auto-scale with a little padding
            pad = 0.05
            ax.set_xlim(min(xs) - pad, max(xs) + pad)
            ax.set_ylim(min(ys) - pad, max(ys) + pad)
            ax.set_zlim(max(0.0, min(zs) - pad), max(zs) + pad)

        plt.draw()
        plt.pause(0.001)

    print("Listening... (Ctrl+C to stop)")
    try:
        while True:
            line = ser.readline()
            if not line:
                continue

            try:
                s = line.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            m = PAT.search(s)
            if not m:
                continue

            x = float(m.group(1))
            y = float(m.group(2))
            z = float(m.group(3))

            xs.append(x)
            ys.append(y)
            zs.append(z)

            sample_count += 1
            print(f"X={x:.3f}  Y={y:.3f}  Z={z:.3f}  (n={len(xs)})")

            if sample_count % UPDATE_EVERY == 0:
                # Limit plot refresh rate slightly
                now = time.time()
                if now - last_plot_t >= 0.02:
                    refresh_plot()
                    last_plot_t = now

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
