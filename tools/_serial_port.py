"""Open the ESP32-S3 USB Serial/JTAG port without resetting the board.

Why this exists: pyserial's `serial.Serial(port, ...)` asserts DTR and RTS as
part of opening the port, and on the ESP32-S3's built-in USB Serial/JTAG that is
exactly the reset sequence - so a "passive" capture would silently reboot the
board and destroy the state you were trying to observe.  (Measured: uptime went
18 s -> 6 s after a plain open.)

Constructing the object first, setting `rts`/`dtr` to False *before* `open()`,
and only then opening it leaves the chip running.  Verified: uptime continued
26 s -> 33 s.

`serial.Serial(port, ..., rts=False, dtr=False)` does NOT work - pyserial
rejects those as unexpected keyword arguments on Windows.

Self-check (note the argument order - host first, then port):

    python tools/_serial_port.py http://10.1.41.103 COM5
"""
from __future__ import annotations

import serial


def open_port(port: str = "COM5", baudrate: int = 115200, timeout: float = 0.2) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.timeout = timeout
    ser.rts = False
    ser.dtr = False
    ser.open()
    return ser


if __name__ == "__main__":
    # Quick self-check: opening must not disturb a running board.
    import json
    import sys
    import time
    import urllib.request

    def uptime(host: str, retries: int = 15):
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        for _ in range(retries):
            try:
                url = "%s/data?t=%d" % (host, time.time() * 1000)
                with opener.open(url, timeout=8) as resp:
                    data = json.loads(resp.read())
                if "uptime_s" in data:
                    return data["uptime_s"]
            except Exception:      # noqa: BLE001 - just retry
                pass
            time.sleep(2)
        return None

    host = sys.argv[1] if len(sys.argv) > 1 else "http://10.1.41.103"
    before = uptime(host)
    ser = open_port(sys.argv[2] if len(sys.argv) > 2 else "COM5")
    time.sleep(2.0)
    ser.close()
    time.sleep(3)
    after = uptime(host)
    print(f"uptime before={before} after={after}")
    print("OK - no reset" if before is not None and after is not None and after >= before
          else "WARNING - the board appears to have rebooted")
