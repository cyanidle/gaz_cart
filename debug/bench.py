#!/usr/bin/env python3
"""Serial bench client for a wheel module (USART2 through the ST-LINK VCP).

No dependencies, no protocol knowledge: it writes lines and prints what the
module sends, timestamped. Use it with the scripts next to it —

    python3 debug/bench.py script debug/bench_speed_sweep.txt
    python3 debug/bench.py send "S" 0.5
    python3 debug/bench.py read 3

Port: $BENCH_PORT, default /dev/ttyACM0. 115200 8N1, no flow control.
Script syntax: `> line` sends, `@ seconds` and `? seconds` both drain the port
for that long (`?` marks a deliberate observation, `@` a wait), `#` comments.

The module's serial protocol is documented in App/serial_line.h: `T` is a
normalized -1..1 speed command, `Q` an open-loop voltage, `V1` switches
telemetry to the diagnostics line, `K` reports the stop/limit counters and `W`
dumps the loop trace. Never run these against a powered cart.
"""

import os
import select
import sys
import termios
import time

PORT = os.environ.get("BENCH_PORT", "/dev/ttyACM0")


def open_port():
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = termios.IGNBRK
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


class Reader:
    def __init__(self, fd, t0):
        self.fd = fd
        self.t0 = t0
        self.buf = b""

    def pump(self, timeout):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return
        data = os.read(self.fd, 4096)
        if not data:
            return
        self.buf += data
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            text = line.rstrip(b"\r").decode("ascii", "replace")
            print(f"[{time.time() - self.t0:7.2f}s] {text}", flush=True)

    def drain(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.pump(max(0.0, deadline - time.time()))


def main():
    mode = sys.argv[1]
    fd = open_port()
    t0 = time.time()
    if mode == "read":
        reader = Reader(fd, t0)
        reader.drain(float(sys.argv[2]))
    elif mode == "send":
        line = sys.argv[2]
        wait = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
        os.write(fd, line.encode() + b"\n")
        print(f"[{time.time() - t0:7.2f}s] > {line}", flush=True)
        reader = Reader(fd, t0)
        reader.drain(wait)
    elif mode == "script":
        reader = Reader(fd, t0)
        with open(sys.argv[2]) as f:
            for raw in f:
                entry = raw.strip()
                if not entry or entry.startswith("#"):
                    continue
                op, _, arg = entry.partition(" ")
                if op == ">":
                    os.write(fd, arg.encode() + b"\n")
                    print(f"[{time.time() - t0:7.2f}s] > {arg}", flush=True)
                elif op == "@":
                    reader.drain(float(arg))
                elif op == "?":
                    reader.drain(float(arg))
                else:
                    print(f"bad script entry: {entry}", file=sys.stderr)
                    sys.exit(2)
    else:
        print(f"unknown mode {mode}", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
