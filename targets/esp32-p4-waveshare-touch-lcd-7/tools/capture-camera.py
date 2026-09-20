#!/usr/bin/env python3
"""Capture a full-resolution still + a 1080p H.264 clip from the ESP32-P4 camera.

The device records to its microSD card (using the board's hardware JPEG / H.264
encoders), then this tool pulls the on-card files off over the USB-serial console
with a chunked, CRC-verified transfer that re-requests any corrupted chunk — so the
pulled files are bit-perfect copies of what's on the card.

    GEADEV CAMSTILL          -> writes /sdcard/frame.jpg, returns size
    GEADEV CAMCLIP [seconds]  -> writes /sdcard/clip.h264, returns size/frames/fps
    GEADEV STAT  <path>       -> file size
    GEADEV GET   <path> <off> <len> -> CRC-tagged base64 chunk

Outputs land in targets/esp32-p4-waveshare-touch-lcd-7/captures/:
  - frame_<ts>.jpg   1920x1080 hardware-JPEG still
  - clip_<ts>.h264   1080p hardware-H.264 elementary stream
  - clip_<ts>.mp4    remuxed with ffmpeg (if installed)

Examples:
    python3 tools/capture-camera.py                 # still + 10s clip
    python3 tools/capture-camera.py --clip-only --seconds 10
    python3 tools/capture-camera.py --still-only
"""
import argparse
import array
import base64
import binascii
import fcntl
import glob
import json
import os
import select
import shutil
import subprocess
import sys
import termios
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
TARGET_DIR = os.path.dirname(THIS_DIR)
REPO = os.path.abspath(os.path.join(TARGET_DIR, "..", ".."))
CAPTURES = os.path.join(TARGET_DIR, "captures")
CHUNK = 64 * 1024  # bytes per GET request — small enough that a dropped line is rare
                   # per chunk, so the CRC-mismatch retry reliably converges.

# CRC matching the device's crc32z() in camera.cpp (reflected poly 0xEDB88820). Used
# only to verify each pulled chunk end-to-end; it just has to agree with the device.
_CRC_TABLE = [0] * 256
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = ((_c >> 1) ^ (0xEDB88820 if (_c & 1) else 0)) & 0xFFFFFFFF
    _CRC_TABLE[_i] = _c


def device_crc32(data):
    c = 0xFFFFFFFF
    for b in data:
        c = ((c >> 8) ^ _CRC_TABLE[(c ^ b) & 0xFF]) & 0xFFFFFFFF
    return (c ^ 0xFFFFFFFF) & 0xFFFFFFFF


class SerialPort:
    """The GEADEV console opened without touching DTR/RTS: a modem-line pulse
    drops a USB-Serial-JTAG board into ROM download mode."""

    def __init__(self, port, baud, trace=False):
        self.path = port
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = (attrs[2] | termios.CLOCAL | termios.CREAD | termios.CS8) & ~termios.CSIZE
        attrs[2] |= termios.CS8
        attrs[3] = 0
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        if baud != 115200:
            set_baud_macos(self.fd, baud)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.buffer = bytearray()
        self.trace = trace

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def write_line(self, line):
        data = (line.rstrip("\r\n") + "\n").encode("ascii")
        view = memoryview(data)
        offset = 0
        while offset < len(data):
            _, writable, _ = select.select([], [self.fd], [], 5.0)
            if not writable:
                continue
            try:
                offset += os.write(self.fd, view[offset:])
            except BlockingIOError:
                continue

    def read_line(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            newline = self.buffer.find(b"\n")
            if newline >= 0:
                raw = self.buffer[:newline]
                del self.buffer[: newline + 1]
                return raw.rstrip(b"\r").decode("utf-8", "replace")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            ready, _, _ = select.select([self.fd], [], [], remaining)
            if not ready:
                return None
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if chunk:
                self.buffer.extend(chunk)


def board_port():
    """Pick the console when exactly one USB-serial device is attached; with
    several, the caller must say which one with --port (addresses are not
    stable board identities, so nothing here guesses)."""
    candidates = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise SystemExit("no USB serial device found; pass --port")
    raise SystemExit("several USB serial devices found; pass --port for the P4 console: " + ", ".join(candidates))


def set_baud_macos(fd, baud):
    """Set an arbitrary baud the macOS way (termios has no B921600). The P4 console
    runs at 921600 and its USB-CDC line coding follows it, so the host must match."""
    IOSSIOSPEED = 0x80045402
    fcntl.ioctl(fd, IOSSIOSPEED, array.array("i", [baud]), True)


def parse_kv(line):
    kv = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            kv[k] = v
    return kv


def drain(port, quiet_for=0.4, max_wait=3.0):
    """Read+discard pending serial lines until the channel is quiet, so a command we
    send next isn't corrupted by a backlog of perf-log spam."""
    deadline = time.monotonic() + max_wait
    while time.monotonic() < deadline:
        if port.read_line(quiet_for) is None:
            return


class CommandCorrupted(Exception):
    pass


def command(port, cmd, ok_prefix, timeout):
    """Send `cmd`, read lines until one starts with `ok_prefix` (returns its kv dict).
    Raises CommandCorrupted on 'unknown-command' (garbled in transit) or a matching
    'GEADEV:ERR <verb>' failure. Tolerates interleaved log lines."""
    verb = cmd.split()[1] if len(cmd.split()) > 1 else ""
    drain(port)
    port.write_line(cmd)
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"no response to {cmd!r}")
        line = port.read_line(min(remaining, 30.0))
        if line is None:
            continue
        if line.startswith("GEADEV:ERR unknown-command"):
            raise CommandCorrupted(line)
        if line.startswith(f"GEADEV:ERR {verb}"):
            raise RuntimeError(line)
        if line.startswith(ok_prefix):
            return parse_kv(line)
        if port.trace and line.startswith("GEADEV:"):
            print(line, file=sys.stderr)


def command_retry(port, cmd, ok_prefix, timeout, retries=4):
    last = None
    for attempt in range(retries):
        try:
            return command(port, cmd, ok_prefix, timeout)
        except CommandCorrupted as e:
            last = e
            print(f"  (command garbled, retry {attempt + 1}/{retries})", file=sys.stderr)
            time.sleep(0.3)
    raise RuntimeError(f"{cmd!r} kept getting corrupted: {last}")


def read_until(port, end_marker, overall_timeout, quiet_eos=5.0):
    """Drain raw bytes into one bytearray until `end_marker` appears (or the channel
    goes quiet after data). Tight loop so the OS serial buffer never overflows."""
    buf = bytearray(port.buffer)
    port.buffer = bytearray()
    deadline = time.monotonic() + overall_timeout
    search_from = 0
    last_recv = time.monotonic()
    while True:
        idx = buf.find(end_marker, search_from)
        if idx >= 0:
            return bytes(buf)
        search_from = max(0, len(buf) - len(end_marker))
        now = time.monotonic()
        if len(buf) > 0 and now - last_recv > quiet_eos:
            return bytes(buf)
        if now >= deadline:
            return bytes(buf)
        r, _, _ = select.select([port.fd], [], [], min(deadline - now, 2.0))
        if not r:
            continue
        chunk = os.read(port.fd, 1 << 16)
        if chunk:
            buf += chunk
            last_recv = time.monotonic()


def get_chunk(port, path, offset, length, retries=6):
    """Pull [offset, offset+length) of `path` and verify its CRC. Retries the chunk on
    any corruption (bad base64, dropped line, CRC mismatch). Returns the bytes."""
    begin_pfx = b"GEADEV:GET BEGIN"
    end_pfx = b"GEADEV:GET END"
    for attempt in range(retries):
        drain(port, quiet_for=0.2, max_wait=1.5)
        port.write_line(f"GEADEV GET {path} {offset} {length}")
        raw = read_until(port, end_pfx, overall_timeout=120.0)
        begin = None
        chunks = []
        for line in raw.split(b"\n"):
            line = line.rstrip(b"\r")
            if begin is None:
                if line.startswith(begin_pfx):
                    begin = parse_kv(line.decode("ascii", "replace"))
                continue
            if line.startswith(end_pfx):
                break
            if line.startswith(b"GEADEV:DATA "):
                chunks.append(line[len(b"GEADEV:DATA "):].strip())
        if begin is None:
            continue  # never saw BEGIN — retry
        try:
            data = base64.b64decode(b"".join(chunks), validate=True)
        except binascii.Error:
            continue  # a line was garbled — retry the chunk
        want_len = int(begin.get("len", -1))
        want_crc = int(begin.get("crc", "x"), 16) if begin.get("crc") else None
        if want_len >= 0 and len(data) != want_len:
            continue
        if want_crc is not None and device_crc32(data) != want_crc:
            continue
        return data
    raise RuntimeError(f"chunk @{offset} ({length}B) of {path} failed CRC after {retries} tries")


def pull_file(port, remote_path, local_path, size):
    t0 = time.monotonic()
    with open(local_path, "wb") as out:
        offset = 0
        while offset < size:
            want = min(CHUNK, size - offset)
            data = get_chunk(port, remote_path, offset, want)
            if not data:
                break
            out.write(data)
            offset += len(data)
            pct = 100 * offset / size if size else 100
            print(f"\r    pulling {os.path.basename(local_path)}: {offset}/{size} ({pct:.0f}%)",
                  end="", file=sys.stderr)
    dt = time.monotonic() - t0
    rate = (offset / 1024.0) / dt if dt > 0 else 0
    print(f"\r    pulled {os.path.basename(local_path)}: {offset} bytes in {dt:.1f}s ({rate:.0f} KB/s)   ",
          file=sys.stderr)
    return offset


def capture_still(port, ts):
    print("→ CAMSTILL (full-resolution JPEG to SD) ...")
    kv = command_retry(port, "GEADEV CAMSTILL", "GEADEV:CAMSTILL OK", timeout=40.0)
    path, size = kv["path"], int(kv["bytes"])
    out = os.path.join(CAPTURES, f"frame_{ts}.jpg")
    got = pull_file(port, path, out, size)
    print(f"  saved {out}  {kv.get('width','?')}x{kv.get('height','?')}  {got} bytes")
    return out


def capture_clip(port, ts, seconds):
    print(f"→ CAMCLIP {seconds:g}s (1080p hardware H.264 to SD) ...")
    kv = command_retry(port, f"GEADEV CAMCLIP {seconds:g}", "GEADEV:CAMCLIP OK",
                       timeout=seconds + 60.0)
    path, size = kv["path"], int(kv["bytes"])
    fps, frames = kv.get("fps", "?"), kv.get("frames", "?")
    raw = os.path.join(CAPTURES, f"clip_{ts}.h264")
    got = pull_file(port, path, raw, size)
    print(f"  saved {raw}  {kv.get('width','?')}x{kv.get('height','?')}  {frames} frames @ {fps}fps  {got} bytes")

    mp4 = os.path.join(CAPTURES, f"clip_{ts}.mp4")
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        print("  ffmpeg not found on PATH — leaving the raw .h264 (plays in VLC).")
        return raw
    rate = "30"
    try:
        if float(fps) > 1.0:
            rate = f"{float(fps):.3f}"
    except (TypeError, ValueError):
        pass
    try:
        subprocess.run([ffmpeg, "-y", "-r", rate, "-i", raw, "-c", "copy", mp4],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"  remuxed → {mp4}  ({rate} fps)")
        return mp4
    except subprocess.CalledProcessError as e:
        print(f"  ffmpeg remux failed ({e}); the raw .h264 is still valid.")
        return raw


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="serial console of the P4 (default: the only attached USB serial device)")
    parser.add_argument("--baud", type=int, default=921600, help="device console baud (default 921600)")
    parser.add_argument("--seconds", type=float, default=10.0, help="clip length (default 10)")
    parser.add_argument("--still-only", action="store_true")
    parser.add_argument("--clip-only", action="store_true")
    parser.add_argument("--trace", action="store_true", help="echo GEADEV lines while waiting")
    args = parser.parse_args()

    os.makedirs(CAPTURES, exist_ok=True)
    port = SerialPort(args.port or board_port(), args.baud, trace=args.trace)
    ts = time.strftime("%Y%m%d-%H%M%S")

    def robust_ping():
        for _ in range(8):
            drain(port)
            port.write_line("GEADEV PING")
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                line = port.read_line(deadline - time.monotonic())
                if line is None:
                    break
                if line.startswith("GEADEV:PONG"):
                    return line
        raise RuntimeError("device did not answer PING")

    try:
        print(f"connected: {robust_ping()}  (port {port.path})")
        if not args.clip_only:
            capture_still(port, ts)
        if not args.still_only:
            capture_clip(port, ts, args.seconds)
    finally:
        port.close()


if __name__ == "__main__":
    main()
