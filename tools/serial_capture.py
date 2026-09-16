"""Capture N seconds of USB Serial/JTAG output to a text file.

    python serial_capture.py COM5 20 out.log

Passive capture: this script must NOT reset the board, or the state you are
trying to observe is destroyed before the first byte arrives.  See
_serial_port.py - a plain serial.Serial(...) open asserts DTR/RTS, which on the
ESP32-S3 USB Serial/JTAG *is* the reset sequence.
"""
import sys
import time

from _serial_port import open_port


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    out = sys.argv[3] if len(sys.argv) > 3 else "serial.log"

    ser = open_port(port)
    buf = bytearray()
    end = time.time() + secs
    try:
        while time.time() < end:
            data = ser.read(4096)
            if data:
                buf += data
    finally:
        ser.close()

    text = buf.decode("utf-8", "replace")
    with open(out, "w", encoding="utf-8") as f:
        f.write(text)
    print(text[-4000:])


if __name__ == "__main__":
    main()
