#!/usr/bin/env python3
"""Reproducible scroll benchmark for RP2350 gea targets.

Drives the firmware's GEADEV DRAG synthetic-touch command (see
rp2350_gea_platform.cpp: startSyntheticDrag) with a fixed gesture script and
summarizes the gea.rp2350.perf / .phase / .refresh / .text_cache counters the
firmware prints once per second (GEA_RP2350_PERF_LOG, and
GEA_RP2350_FRAME_PHASE_PERF / GEA_RP2350_REFRESH_DETAIL_PERF when compiled in).

Usage:
  python3 rp2350-scroll-bench.py [--port /dev/cu.usbmodemXXX] [--label name]
                                 [--drags 6] [--out capture.log]

The workload: N full-height up-drags (content scrolls down) with momentum,
then N down-drags back, ~1.4s apart. "Active" seconds are those with
flush_calls >= 5; idle seconds are excluded from the scroll averages.
"""

import argparse
import glob
import re
import sys
import threading
import time

import serial


def find_port(explicit):
    if explicit:
        return explicit
    # Identify the RP2350 by its once-per-second gea.rp2350.* stats lines
    # (GEA_RP2350_PERF_LOG) — a GEADEV:PONG alone could be any gea device on
    # another board.
    for dev in sorted(glob.glob("/dev/cu.usbmodem*")):
        try:
            s = serial.Serial(dev, 115200, timeout=0.2)
        except Exception:
            continue
        try:
            s.reset_input_buffer()
            deadline = time.time() + 2.5
            buf = b""
            while time.time() < deadline:
                try:
                    buf += s.read(1024)
                except serial.SerialException:
                    break  # port vanished mid-probe (device re-enumerating)
                if b"gea.rp2350." in buf:
                    return dev
        finally:
            s.close()
    return None


class Capture(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.serial = serial.Serial(port, 115200, timeout=0.5)
        self.lines = []
        self.ok_drags = 0
        self.stop_flag = False
        self.t0 = time.time()

    def run(self):
        while not self.stop_flag:
            raw = self.serial.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").rstrip()
            if not line:
                continue
            self.lines.append((time.time() - self.t0, line))
            if line.startswith("GEADEV:OK DRAG"):
                self.ok_drags += 1

    def send(self, cmd):
        self.serial.write((cmd + "\n").encode())
        self.serial.flush()


def kv_ints(line):
    return {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", line)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--label", default="run")
    ap.add_argument("--drags", type=int, default=6)
    ap.add_argument("--out")
    args = ap.parse_args()

    port = find_port(args.port)
    if not port:
        print("ERROR: no GEADEV-responding serial port found", file=sys.stderr)
        return 1
    print(f"[{args.label}] port={port}")

    cap = Capture(port)
    cap.start()
    time.sleep(2.5)  # settle + one idle second of stats

    gestures = [(225, 520, 225, 120)] * args.drags + [(225, 120, 225, 520)] * args.drags
    for i, (x1, y1, x2, y2) in enumerate(gestures):
        before = cap.ok_drags
        cap.send(f"GEADEV DRAG {x1} {y1} {x2} {y2} 25 16")
        deadline = time.time() + 4
        while cap.ok_drags == before and time.time() < deadline:
            time.sleep(0.05)
        if cap.ok_drags == before:
            print(f"WARN: drag {i} not acknowledged", file=sys.stderr)
        time.sleep(1.4)  # let momentum play out

    time.sleep(3)  # idle tail
    cap.stop_flag = True
    time.sleep(0.8)

    if args.out:
        with open(args.out, "w") as f:
            for ts, line in cap.lines:
                f.write(f"{ts:7.2f} {line}\n")

    perf = [kv_ints(l) for _, l in cap.lines if "gea.rp2350.perf " in l]
    phase = [kv_ints(l) for _, l in cap.lines if "gea.rp2350.phase " in l]
    text = [kv_ints(l) for _, l in cap.lines if "gea.rp2350.text_cache " in l]

    active = [p for p in perf if p.get("flush_calls", 0) >= 5]
    if not active:
        print("no active seconds captured — did the drags scroll anything?")
        return 1

    def mean(xs):
        return sum(xs) / len(xs)

    fps = [p["fps"] for p in active]
    tot = [p["total_ms"] for p in active]
    txw = [p["txwait_ms"] for p in active]
    kpx = [p["flush_kpx"] for p in active]

    # phase lines aligned to active seconds by index in the shared cadence
    act_idx = [i for i, p in enumerate(perf) if p.get("flush_calls", 0) >= 5]
    aphase = [phase[i] for i in act_idx if i < len(phase)]
    work = [p["work_ms"] for p in aphase] if aphase else [0]
    avg_work = [p["avg_work_us"] for p in aphase] if aphase else [0]
    worst = max((p["worst_work_us"] for p in aphase), default=0)
    over = sum(p["over_budget"] for p in aphase) if aphase else 0
    frames = sum(p["frames"] for p in aphase) if aphase else 0

    tc_calls = sum(t.get("calls", 0) for t in text)
    tc_hits = sum(t.get("hits", 0) for t in text)

    print(f"[{args.label}] active_seconds={len(active)} ok_drags={cap.ok_drags}")
    print(f"[{args.label}] fps mean={mean(fps):.1f} min={min(fps)} max={max(fps)}")
    print(f"[{args.label}] work_ms/s mean={mean(work):.0f} avg_work_us={mean(avg_work):.0f} worst_work_us={worst} over_budget={over}/{frames}")
    print(f"[{args.label}] flush total_ms/s mean={mean(tot):.0f} txwait_ms/s mean={mean(txw):.0f} kpx/s mean={mean(kpx):.0f}")
    print(f"[{args.label}] text_cache calls={tc_calls} hits={tc_hits}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
