"""Raw-socket HTTP probe for the ESP32-S3-EYE dashboard.

Prints the full response (status line, headers, body length, body head) so we
never have to trust curl's -w formatting.

    python http_probe.py 10.1.42.176 /
    python http_probe.py 10.1.42.176 /data
"""
import socket
import sys


def probe(host, path, port=80, timeout=5):
    req = (f"GET {path} HTTP/1.1\r\n"
           f"Host: {host}\r\n"
           f"User-Agent: http_probe/1.0\r\n"
           f"Connection: close\r\n"
           f"\r\n")
    s = socket.create_connection((host, port), timeout=timeout)
    try:
        s.sendall(req.encode())
        chunks = []
        while True:
            b = s.recv(65536)
            if not b:
                break
            chunks.append(b)
    finally:
        s.close()
    return b"".join(chunks)


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "10.1.42.176"
    path = sys.argv[2] if len(sys.argv) > 2 else "/"

    try:
        raw = probe(host, path)
    except OSError as e:
        print(f"CONNECT FAILED: {e!r}")
        return 1

    head, _, body = raw.partition(b"\r\n\r\n")
    print("=== status + headers ===")
    for line in head.split(b"\r\n"):
        print("   ", line.decode("utf-8", "replace"))
    print("=== body ===")
    print("    length:", len(body))
    if body:
        print("    head:", body[:200].decode("utf-8", "replace").replace("\n", "\\n"))
    else:
        print("    (EMPTY BODY)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
