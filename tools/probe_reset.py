#!/usr/bin/env python3
"""Ловля первых байтов сразу после открытия порта (DTR-сброс → баннер).

Проверка: если при открытии порта плата сбрасывается и скетч жив,
первые байты должны быть "LTC reader init OK", далее D/TC-строки.
"""
import os
import sys
import time
import termios

PORT = os.environ.get("LTC_PORT", "/dev/cu.usbserial-A5069RR4")
BAUDS = [int(x) for x in (sys.argv[1].split(",") if len(sys.argv) > 1 else ["115200"])]
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
SAVE = sys.argv[3] if len(sys.argv) > 3 else "/tmp/ltc_uart_capture.txt"

for baud in BAUDS:
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = termios.IGNBRK          # iflag: игнор break (мусор)
    attrs[1] = 0                       # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    attrs[3] = 0                       # lflag: raw
    attrs[4] = attrs[5] = baud
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)

    buf = b""
    t0 = time.monotonic()
    while time.monotonic() - t0 < SECS:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            time.sleep(0.002)
            continue
        buf += chunk
    os.close(fd)

    nul = buf.count(0)
    printable = sum(1 for b in buf if 32 <= b < 127)
    print(f"\n=== {baud} бод: {len(buf)} байт, NUL={nul}, printable={printable}")
    with open(SAVE, "wb") as f:
        f.write(buf)
    print(f"сохранено: {SAVE}")
    print("первые 100:", repr(buf[:100]))
    if printable > 20:
        txt = buf.decode("latin1")
        import re
        runs = re.findall(r"[ -~]{6,}", txt)
        from collections import Counter
        for s, n in Counter(runs).most_common(5):
            print(f"  {n}x {s[:80]!r}")
