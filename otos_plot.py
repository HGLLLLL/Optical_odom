#!/usr/bin/env python3
"""Live trajectory plotter for the SparkFun OTOS test sketch (otos_test.ino).

Usage: python3 otos_plot.py [--port /dev/ttyUSB0] [--baud 115200] [--log path.csv]
"""
import argparse
import math
import re
import threading
import time

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import serial

LINE_RE = re.compile(r"X:\s*(-?[\d.]+)\s*cm\s+Y:\s*(-?[\d.]+)\s*cm\s+H:\s*(-?[\d.]+)\s*deg")


class OtosReader:
    def __init__(self, port, baud, log_path=None):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.2
        # Keep DTR/RTS low so opening the port doesn't reset the ESP32
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()

        self.lock = threading.Lock()
        self.xs, self.ys = [], []
        self.pose = (0.0, 0.0, 0.0)
        self.distance = 0.0
        self.status = "Waiting for data..."
        self.log = open(log_path, "w") if log_path else None
        if self.log:
            self.log.write("t,x_cm,y_cm,h_deg\n")
        self.t0 = time.time()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                raw = self.ser.readline().decode(errors="replace").strip()
            except serial.SerialException as e:
                self.status = f"Serial error: {e}"
                return
            if not raw:
                continue
            m = LINE_RE.search(raw)
            if not m:
                self.status = raw[:60]
                # ESP32 rebooted or tracking was reset: start a fresh path
                if "Tracking reset" in raw or "OTOS test" in raw:
                    self.clear()
                continue
            x, y, h = map(float, m.groups())
            with self.lock:
                if self.xs:
                    self.distance += math.hypot(x - self.xs[-1], y - self.ys[-1])
                self.xs.append(x)
                self.ys.append(y)
                self.pose = (x, y, h)
                self.status = "Tracking"
            if self.log:
                self.log.write(f"{time.time() - self.t0:.3f},{x},{y},{h}\n")
                self.log.flush()

    def send(self, cmd):
        self.ser.write(cmd.encode())

    def clear(self):
        with self.lock:
            self.xs.clear()
            self.ys.clear()
            self.distance = 0.0

    def snapshot(self):
        with self.lock:
            return list(self.xs), list(self.ys), self.pose, self.distance, self.status


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", default=None, help="optional CSV output path")
    args = ap.parse_args()

    reader = OtosReader(args.port, args.baud, args.log)

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title("OTOS Live Trajectory")
    plt.subplots_adjust(bottom=0.15)
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.4)
    ax.plot(0, 0, "k+", markersize=12, label="origin")
    path_line, = ax.plot([], [], "-", color="tab:blue", lw=1.5, label="path")
    cur_dot, = ax.plot([], [], "o", color="tab:red", markersize=8, label="current")
    heading_line, = ax.plot([], [], "-", color="tab:red", lw=2.5)
    info = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", family="monospace",
                   bbox=dict(boxstyle="round", fc="white", alpha=0.85))
    ax.legend(loc="lower right")

    def on_reset(_):
        reader.send("r")
        reader.clear()

    def on_clear(_):
        reader.clear()

    def on_calib(_):
        reader.send("c")
        reader.clear()

    buttons = []
    for i, (label, cb) in enumerate([("Reset origin (r)", on_reset),
                                     ("Clear path", on_clear),
                                     ("Calibrate IMU (c)", on_calib)]):
        b = Button(fig.add_axes([0.1 + i * 0.28, 0.03, 0.25, 0.06]), label)
        b.on_clicked(cb)
        buttons.append(b)

    def update(_):
        xs, ys, (x, y, h), dist, status = reader.snapshot()
        path_line.set_data(xs, ys)
        if xs:
            cur_dot.set_data([x], [y])
            # Heading arrow length scales with the view so it stays visible
            xmin, xmax = min(xs + [0]), max(xs + [0])
            ymin, ymax = min(ys + [0]), max(ys + [0])
            span = max(xmax - xmin, ymax - ymin, 10.0)
            L = span * 0.08
            hr = math.radians(h)
            heading_line.set_data([x, x + L * math.cos(hr)], [y, y + L * math.sin(hr)])
            pad = span * 0.15
            cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
            half = span / 2 + pad
            ax.set_xlim(cx - half, cx + half)
            ax.set_ylim(cy - half, cy + half)
        info.set_text(f"X: {x:8.2f} cm\nY: {y:8.2f} cm\nH: {h:8.2f} deg\n"
                      f"Dist from origin: {math.hypot(x, y):7.2f} cm\n"
                      f"Path length:      {dist:7.2f} cm\n"
                      f"Points: {len(xs)}   [{status}]")
        return path_line, cur_dot, heading_line, info

    _anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
