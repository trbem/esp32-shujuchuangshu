"""Convert the 24-bit BMP that /shot returns into a PNG.

Handy because a browser/chat viewer renders PNG inline but often refuses .bmp.
Pure standard library (struct + zlib) so it runs on any Python without Pillow.

    python bmp2png.py shot.bmp shot.png
"""
from __future__ import annotations

import struct
import sys
import zlib


def read_bmp(path: str) -> tuple[int, int, bytes]:
    """Return (width, height, RGB rows top-down)."""
    with open(path, "rb") as fh:
        data = fh.read()

    if data[:2] != b"BM":
        raise ValueError("not a BMP file")

    pixel_offset = struct.unpack_from("<I", data, 10)[0]
    width, height = struct.unpack_from("<ii", data, 18)
    bpp = struct.unpack_from("<H", data, 28)[0]
    compression = struct.unpack_from("<I", data, 30)[0]
    if bpp != 24 or compression != 0:
        raise ValueError(f"only uncompressed 24-bit BMP is supported (bpp={bpp}, comp={compression})")

    top_down = height < 0
    height = abs(height)
    stride = (width * 3 + 3) & ~3  # rows are padded to 4 bytes

    rows: list[bytes] = []
    for y in range(height):
        src_y = y if top_down else height - 1 - y  # BMP stores bottom-up by default
        start = pixel_offset + src_y * stride
        bgr = data[start:start + width * 3]
        rgb = bytearray(width * 3)
        rgb[0::3] = bgr[2::3]  # B -> R
        rgb[1::3] = bgr[1::3]  # G -> G
        rgb[2::3] = bgr[0::3]  # R -> B
        rows.append(bytes(rgb))
    return width, height, b"".join(rows)


def write_png(path: str, width: int, height: int, rgb: bytes) -> None:
    raw = bytearray()
    for y in range(height):
        raw.append(0)  # filter type 0 (None) per scanline
        raw += rgb[y * width * 3:(y + 1) * width * 3]

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as fh:
        fh.write(png)


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__.strip())
        return 2
    width, height, rgb = read_bmp(argv[1])
    write_png(argv[2], width, height, rgb)
    print(f"{argv[1]} -> {argv[2]} ({width}x{height})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
