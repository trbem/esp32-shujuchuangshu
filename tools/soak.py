"""Poll /data for a while and report whether the board stays healthy.

Watches for the things a short capture cannot show: a slow heap leak, a reboot
(boot_id / uptime going backwards), the LCD frame counter stalling, or the
telemetry queue failing to drain.

    python soak.py http://10.1.41.103 360 15
"""
from __future__ import annotations

import json
import sys
import time
import urllib.request


def fetch(base: str) -> dict:
    # Note the parentheses: "%s/data?t=%d" % (base, ms) - without them Python
    # parses this as (base + "...%d" % secs) * 1000, i.e. a ~24 KB URL made by
    # repeating the string 1000 times, and the board answers 414 URI Too Long.
    url = "%s/data?t=%d" % (base, time.time() * 1000)
    req = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
    # This machine has an HTTP proxy configured, which urllib picks up from the
    # environment and happily sends a request for 10.1.41.103 to - the proxy then
    # answers 431.  Same reason curl needs --noproxy '*' here.  An empty
    # ProxyHandler disables proxying for this opener.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(req, timeout=15) as resp:
        return json.loads(resp.read().decode())


def main(argv: list[str]) -> int:
    base = argv[1] if len(argv) > 1 else "http://10.1.41.103"
    seconds = int(argv[2]) if len(argv) > 2 else 360
    period = int(argv[3]) if len(argv) > 3 else 15

    deadline = time.time() + seconds
    started = time.time()
    rows: list[dict] = []
    errors = 0

    print(f"# soaking {base} for {seconds}s, sample every {period}s")
    print(f"{'t':>5} {'uptime':>7} {'heap_int':>9} {'heap_psram':>10} {'fps':>5} "
          f"{'lcd_fps':>8} {'lcd_frames':>10} {'lcd_err':>8} {'queue':>6} {'drop':>5} {'ok':>5}")

    while time.time() < deadline:
        t = int(time.time() - started)
        try:
            d = fetch(base)
        except Exception as exc:                      # noqa: BLE001 - report and retry
            errors += 1
            print(f"{t:>5}  --- request failed: {exc}")
            time.sleep(period)
            continue

        rows.append(d)
        print(f"{t:>5} {d.get('uptime_s', 0):>7} {d.get('heap_int', 0):>9} "
              f"{d.get('heap_psram', 0):>10} {d.get('fps', 0):>5.1f} "
              f"{d.get('lcd_fps', 0):>8.1f} {d.get('lcd_frames', 0):>10} "
              f"{d.get('lcd_err', 0):>8} {d.get('telemetry_queue', 0):>6} "
              f"{d.get('telemetry_dropped', 0):>5} {str(d.get('upload_ok')):>5}")
        time.sleep(period)

    print("\n# verdict")
    if len(rows) < 2:
        print(f"  not enough samples ({len(rows)}); {errors} request error(s)")
        return 1

    first, last = rows[0], rows[-1]
    span = last["uptime_s"] - first["uptime_s"]

    # a reboot shows up as uptime going backwards
    reboots = [r for a, r in zip(rows, rows[1:]) if r["uptime_s"] < a["uptime_s"]]
    print(f"  uptime span        : {span}s over {len(rows)} samples")
    print(f"  reboots            : {len(reboots)}")
    print(f"  request errors     : {errors}")

    drift = last["heap_int"] - first["heap_int"]
    print(f"  internal SRAM drift: {drift:+d} B  ({first['heap_int']} -> {last['heap_int']})")
    print(f"  PSRAM drift        : {last['heap_psram'] - first['heap_psram']:+d} B")

    lcd_delta = last["lcd_frames"] - first["lcd_frames"]
    if span > 0:
        print(f"  LCD frames pushed  : {lcd_delta} in {span}s = {lcd_delta / span:.1f} fps")
    print(f"  lcd_frames monotonic: {all(b['lcd_frames'] >= a['lcd_frames'] for a, b in zip(rows, rows[1:]))}")
    print(f"  lcd_err             : {first.get('lcd_err', 0)} -> {last.get('lcd_err', 0)}")
    print(f"  upload_ok ever true : {any(r.get('upload_ok') for r in rows)}")
    print(f"  telemetry_dropped   : {first.get('telemetry_dropped')} -> {last.get('telemetry_dropped')}")

    stalled = span > 0 and lcd_delta < 0.5 * span * 20   # under ~20 fps average is a stall
    # Failed polls are reported but not fatal: this link genuinely drops responses
    # now and then (2.4 GHz loss + a 1.5 s LWIP RTO).  Only treat a majority of
    # failures as a problem.
    ok = (len(reboots) == 0 and errors < len(rows)
          and not stalled
          and last.get("lcd_err", 0) == first.get("lcd_err", 0)
          and last.get("telemetry_dropped", 0) == first.get("telemetry_dropped", 0))
    if stalled:
        print("  !! the viewfinder stopped pushing frames")
    print(f"\n  RESULT: {'PASS' if ok else 'CHECK'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
