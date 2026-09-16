"""Reset the board over USB Serial/JTAG and capture the boot log.

    python serial_reset_capture.py COM5 15 boot.log

The ESP32-S3 USB Serial/JTAG reset is driven by the DTR/RTS lines the same way
esptool does it: RTS is the EN (reset) line and DTR selects the boot mode.

This is the one script that *wants* a reset, so the open must be done without
one (open_port(), see _serial_port.py) and the reset sequence is then issued
explicitly below.  Opening the port the plain way would reset the chip early
and the boot log would start mid-stream.
"""

import sys
import time

from _serial_port import open_port


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    out = sys.argv[3] if len(sys.argv) > 3 else "boot.log"

    ser = open_port(port)

    # enter the reset sequence (matches esptool's ClassicReset)
    ser.setDTR(False)   # IO0 = high -> normal boot
    ser.setRTS(True)    # EN = low  -> hold in reset
    time.sleep(0.1)
    ser.setRTS(False)   # release reset
    time.sleep(0.05)
    ser.reset_input_buffer()

    buf = bytearray()
    end = time.time() + secs
    while time.time() < end:
        data = ser.read(4096)
        if data:
            buf += data
    ser.close()

    text = buf.decode("utf-8", "replace")
    with open(out, "w", encoding="utf-8") as f:
        f.write(text)
    print(text)


if __name__ == "__main__":
    main()
