#!/usr/bin/env python3
"""Pack a Moto G7 Plus (lake) `fastboot boot` image around an arm64 Image.

  mklakeboot.py STOCK_BOOT IMAGE [PAYLOAD] OUTPUT

STOCK_BOOT is a backup of the phone's own boot partition (Motorola boot
header v1, 4K pages, gzip Image + one appended DTB). ABL only boots a DTB
whose qcom,msm-id/board-id match and applies the stock DTBO to it, so the
stock DTB is reused as is; the shim only reads /chosen from it. The ramdisk
is PAYLOAD (an xnu payload from mkpayload.py) or an empty gzip cpio.
"""
import gzip
import hashlib
import struct
import sys
import zlib
from pathlib import Path

PAGE = 4096


def align(data: bytes) -> bytes:
    return data + bytes((-len(data)) % PAGE)


def stock_parts(path: Path):
    data = path.read_bytes()
    if data[:8] != b"ANDROID!":
        sys.exit(f"{path}: not an Android boot image")
    ksz = struct.unpack_from("<I", data, 8)[0]
    page, version = struct.unpack_from("<II", data, 36)
    if (page, version) != (PAGE, 1):
        sys.exit(f"{path}: expected boot header v1 with 4K pages, got v{version}/{page}")
    z = zlib.decompressobj(31)
    z.decompress(data[PAGE:PAGE + ksz])
    dtb = z.unused_data
    if dtb[:4] != b"\xd0\x0d\xfe\xed" or struct.unpack_from(">I", dtb, 4)[0] != len(dtb):
        sys.exit(f"{path}: expected exactly one DTB appended to the gzip kernel")
    return data[:PAGE], dtb


def empty_cpio() -> bytes:
    # newc trailer only; ABL dislikes a zero-size ramdisk on some builds
    name = b"TRAILER!!!\0"
    raw = b"070701" + b"00000000" * 11 + b"%08X" % len(name) + b"00000000" + name
    raw += bytes((-len(raw)) % 4)
    return gzip.compress(raw + bytes((-len(raw)) % 512), mtime=0)


def main() -> None:
    args = sys.argv[1:]
    if len(args) not in (3, 4):
        sys.exit(__doc__)
    header, dtb = stock_parts(Path(args[0]))
    kernel = gzip.compress(Path(args[1]).read_bytes(), compresslevel=9, mtime=0) + dtb
    ramdisk = Path(args[2]).read_bytes() if len(args) == 4 else empty_cpio()

    hdr = bytearray(header)
    struct.pack_into("<I", hdr, 8, len(kernel))
    struct.pack_into("<I", hdr, 16, len(ramdisk))
    hdr[64:576] = b"lake-shim".ljust(512, b"\0")   # cmdline: nothing reads it
    hdr[608:1632] = bytes(1024)                      # extra_cmdline
    digest = hashlib.sha1()
    for part in (kernel, ramdisk, b""):              # kernel, ramdisk, second
        digest.update(part)
        digest.update(struct.pack("<I", len(part)))
    digest.update(struct.pack("<I", 0))              # recovery DTBO (v1)
    hdr[576:608] = digest.digest().ljust(32, b"\0")
    blob = bytes(hdr) + align(kernel) + align(ramdisk)
    Path(args[-1]).write_bytes(blob)
    print(f"{args[-1]}: {len(blob):,} bytes; SHA256 {hashlib.sha256(blob).hexdigest()}")


if __name__ == "__main__":
    main()
