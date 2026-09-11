"""Capture N seconds of USB Serial/JTAG output to a text file.

    python serial_capture.py COM5 20 out.log
"""
import sys
import time

import serial


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    out = sys.argv[3] if len(sys.argv) > 3 else "serial.log"

    ser = serial.Serial(port, 115200, timeout=0.2)
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
