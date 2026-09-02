#!/usr/bin/env python3
"""
capture_serial.py - interactive serial terminal + file logger for the
wheeled-bipedal robot.

Use this INSTEAD of the Arduino Serial Monitor (only one program can own the
COM port at a time). It:
  * echoes all serial output to your console,
  * appends every line to a timestamped session log (logs/session_*.log),
  * automatically extracts each "---- LOG BEGIN ... END ----" burst-log block
    into its own CSV file (logs/trace_*.csv), with a proper header row,
  * forwards your keystrokes to the robot so you can still drive it
    (H, R, B, T, L, digits, and typed tuning like "kp 0.2<Enter>").

Instant keys (H/B/K/space/L/digits/-) are sent the moment you press them, so
the spacebar / K kill switch stays instant. Typed tuning commands work too -
just type "kp 0.2" and press Enter.

Quit the terminal with Ctrl-C (this does NOT stop the robot; use space/K first).

Requires: pip install pyserial

Examples:
  python capture_serial.py --port COM5
  python capture_serial.py            # auto-detect if exactly one port
  python capture_serial.py --list     # list available ports and exit
"""

import argparse
import datetime as dt
import os
import sys
import threading

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

# Windows single-keypress input (gives an instant, un-buffered kill switch).
try:
    import msvcrt
    HAVE_MSVCRT = True
except ImportError:
    HAVE_MSVCRT = False


LOG_BEGIN = "---- LOG BEGIN"
LOG_END = "---- LOG END"


def pick_port(explicit):
    ports = list(list_ports.comports())
    if explicit:
        return explicit
    if len(ports) == 1:
        print(f"[auto] using {ports[0].device} ({ports[0].description})")
        return ports[0].device
    if not ports:
        sys.exit("No serial ports found. Plug in the board or pass --port.")
    print("Multiple ports found - pass one with --port:")
    for p in ports:
        print(f"  {p.device}  {p.description}")
    sys.exit(1)


def header_from_begin_line(line):
    """Extract 't_s,pitch,...' from '---- LOG BEGIN (t_s,pitch,...) ----'."""
    if "(" in line and ")" in line:
        return line[line.index("(") + 1: line.rindex(")")].strip()
    return None


class Reader(threading.Thread):
    """Reads serial lines: mirror to console, append to session log, and
    split out each burst-log block into its own CSV."""

    def __init__(self, ser, outdir):
        super().__init__(daemon=True)
        self.ser = ser
        self.outdir = outdir
        self.running = True
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_path = os.path.join(outdir, f"session_{stamp}.log")
        self.session = open(self.session_path, "a", encoding="utf-8", buffering=1)
        print(f"[log] session -> {self.session_path}")
        self._csv = None
        self._trace_n = 0

    def _open_trace(self, begin_line):
        self._trace_n += 1
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.outdir, f"trace_{stamp}_{self._trace_n}.csv")
        self._csv = open(path, "w", encoding="utf-8", buffering=1)
        hdr = header_from_begin_line(begin_line)
        if hdr:
            self._csv.write(hdr + "\n")
        print(f"[log] capturing trace -> {path}")

    def _close_trace(self):
        if self._csv:
            self._csv.close()
            print(f"[log] trace saved ({self._csv.name})")
            self._csv = None

    def run(self):
        buf = b""
        while self.running:
            try:
                data = self.ser.read(256)
            except serial.SerialException:
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").rstrip("\r")
                sys.stdout.write(line + "\n")
                sys.stdout.flush()
                self.session.write(line + "\n")

                if LOG_BEGIN in line:
                    self._open_trace(line)
                elif LOG_END in line:
                    self._close_trace()
                elif self._csv is not None:
                    # data rows inside a block are comma-separated numbers
                    self._csv.write(line + "\n")

    def stop(self):
        self.running = False
        self._close_trace()
        try:
            self.session.close()
        except Exception:
            pass


def input_loop_msvcrt(ser):
    """Windows: forward each keypress immediately (instant kill switch)."""
    print("[ready] type commands; keys are sent instantly. Ctrl-C to quit.")
    while True:
        ch = msvcrt.getch()
        if ch in (b"\x03",):          # Ctrl-C
            raise KeyboardInterrupt
        if ch in (b"\x00", b"\xe0"):  # arrow/function key prefix - swallow next
            msvcrt.getch()
            continue
        ser.write(ch)
        # local echo so you can see what you type
        if ch in (b"\r", b"\n"):
            sys.stdout.write("\n")
        else:
            try:
                sys.stdout.write(ch.decode("utf-8", errors="replace"))
            except Exception:
                pass
        sys.stdout.flush()


def input_loop_lines(ser):
    """Fallback (non-Windows): line-buffered. Kill needs Enter after space/K."""
    print("[ready] type commands + Enter. NOTE: kill = press K or space then Enter.")
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        # send the raw characters; firmware handles instant keys + typed lines
        ser.write(line.encode("utf-8"))


def main():
    ap = argparse.ArgumentParser(description="Serial terminal + file logger for wheeled-bipedal.")
    ap.add_argument("--port", help="serial port, e.g. COM5 (auto-detected if only one)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--outdir", default="logs", help="directory for session + trace files")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = ap.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device}  {p.description}")
        return

    port = pick_port(args.port)
    os.makedirs(args.outdir, exist_ok=True)

    ser = serial.Serial(port, args.baud, timeout=0.1)
    print(f"[open] {port} @ {args.baud}")

    reader = Reader(ser, args.outdir)
    reader.start()

    try:
        if HAVE_MSVCRT:
            input_loop_msvcrt(ser)
        else:
            input_loop_lines(ser)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[exit] closing (robot keeps its current state - use space/K to stop it)")
        reader.stop()
        ser.close()


if __name__ == "__main__":
    main()
