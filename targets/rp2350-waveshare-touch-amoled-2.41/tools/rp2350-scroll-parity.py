#!/usr/bin/env python3
"""Pixel-parity capture for RP2350 scroll-path changes.

Drives GEADEV DRAG gestures to the scroll clamps (deterministic positions),
takes GEADEV SCREENSHOT captures (logical row order — scroll-offset invariant)
and saves the decoded RGB565 pixel arrays. Run once against a reference build
and once against a candidate build, then compare with --compare a.bin b.bin.

  python3 rp2350-scroll-parity.py --label ref --outdir /tmp/parity
  python3 rp2350-scroll-parity.py --label new --outdir /tmp/parity
  python3 rp2350-scroll-parity.py --compare /tmp/parity/ref_bottom.pix /tmp/parity/new_bottom.pix
"""

import argparse
import base64
import glob
import os
import struct
import sys
import time

import serial


def find_port(explicit):
    if explicit:
        return explicit
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
                    break
                if b"gea.rp2350." in buf:
                    return dev
        finally:
            s.close()
    return None


def read_line(port, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = port.readline().decode("utf-8", "replace").rstrip()
        if line:
            yield line
    return


def drag(port, x1, y1, x2, y2, steps=25, delay=16):
    port.reset_input_buffer()
    port.write(f"GEADEV DRAG {x1} {y1} {x2} {y2} {steps} {delay}\n".encode())
    port.flush()
    for line in read_line(port, 4):
        if line.startswith("GEADEV:OK DRAG"):
            return True
        if line.startswith("GEADEV:ERR"):
            print("drag error:", line, file=sys.stderr)
            return False
    print("drag not acknowledged", file=sys.stderr)
    return False


def screenshot(port):
    port.reset_input_buffer()
    port.write(b"GEADEV SCREENSHOT\n")
    port.flush()
    width = height = None
    chunks = []
    # Bulk-read the whole dump: a full-screen dithered frame RLE-encodes to
    # ~1MB of base64, and per-line reads fall behind the CDC stream (the OS
    # input buffer overflows and drops bytes mid-line -> corrupt payload).
    deadline = time.time() + 60
    buf = b""
    while time.time() < deadline:
        chunk = port.read(65536)
        if chunk:
            buf += chunk
            if b"GEADEV:SCREENSHOT END" in buf:
                break
        elif b"GEADEV:SCREENSHOT END" in buf:
            break
    in_shot = False
    for raw in buf.split(b"\n"):
        line = raw.decode("utf-8", "replace").rstrip()
        if line.startswith("GEADEV:SCREENSHOT BEGIN"):
            in_shot = True
            for tok in line.split():
                if tok.startswith("width="):
                    width = int(tok[6:])
                if tok.startswith("height="):
                    height = int(tok[7:])
            continue
        if line.startswith("GEADEV:SCREENSHOT END"):
            break
        if in_shot and line.startswith("GEADEV:DATA "):
            chunks.append(line[len("GEADEV:DATA "):].strip())
    if not width or not chunks:
        return None, 0, 0
    data = base64.b64decode("".join(chunks))
    pixels = []
    for i in range(0, len(data) - 3, 4):
        count, value = struct.unpack_from("<HH", data, i)
        pixels.extend([value] * count)
    n = width * height
    if len(pixels) != n:
        print(f"WARN: decoded {len(pixels)} px, expected {n}", file=sys.stderr)
        pixels = (pixels + [0] * n)[:n]
    return pixels, width, height


def save(path, pixels, width, height):
    with open(path, "wb") as f:
        f.write(struct.pack("<HH", width, height))
        f.write(struct.pack(f"<{len(pixels)}H", *pixels))
    print("saved", path)


def save_png(path, pixels, width, height):
    try:
        from PIL import Image
    except ImportError:
        return
    rgb = bytearray(width * height * 3)
    for i, v in enumerate(pixels):
        # framebuffer is panel-endian: swap bytes back before unpacking 565
        v = ((v & 0xFF) << 8) | (v >> 8)
        r = (v >> 11) & 0x1F
        g = (v >> 5) & 0x3F
        b = v & 0x1F
        rgb[i * 3 + 0] = (r << 3) | (r >> 2)
        rgb[i * 3 + 1] = (g << 2) | (g >> 4)
        rgb[i * 3 + 2] = (b << 3) | (b >> 2)
    Image.frombytes("RGB", (width, height), bytes(rgb)).save(path)
    print("saved", path)


def compare(a, b):
    da = open(a, "rb").read()
    db = open(b, "rb").read()
    if da == db:
        print("IDENTICAL")
        return 0
    wa, ha = struct.unpack_from("<HH", da, 0)
    pa = struct.unpack_from(f"<{wa*ha}H", da, 4)
    pb = struct.unpack_from(f"<{wa*ha}H", db, 4)
    diff = [i for i in range(len(pa)) if pa[i] != pb[i]]
    rows = sorted({i // wa for i in diff})
    print(f"DIFFER: {len(diff)} px, rows {rows[:10]}{'...' if len(rows) > 10 else ''} ({len(rows)} rows)")
    return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--label", default="run")
    ap.add_argument("--outdir", default=".")
    ap.add_argument("--drags", type=int, default=10)
    ap.add_argument("--compare", nargs=2)
    args = ap.parse_args()

    if args.compare:
        return compare(*args.compare)

    dev = find_port(args.port)
    if not dev:
        print("no rp2350 port found", file=sys.stderr)
        return 1
    port = serial.Serial(dev, 115200, timeout=0.5)
    os.makedirs(args.outdir, exist_ok=True)
    print(f"[{args.label}] port={dev}")

    for name, gesture in (("bottom", (225, 520, 225, 120)), ("top", (225, 120, 225, 520))):
        for _ in range(args.drags):
            drag(port, *gesture)
            time.sleep(1.3)
        time.sleep(2)  # settle
        pixels, w, h = screenshot(port)
        if pixels is None:
            print("screenshot failed", file=sys.stderr)
            return 1
        base = os.path.join(args.outdir, f"{args.label}_{name}")
        save(base + ".pix", pixels, w, h)
        save_png(base + ".png", pixels, w, h)
    port.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
