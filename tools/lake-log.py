#!/usr/bin/env python3
"""Print the lake RAM log (shim/lakelog.h).

  lake-log.py            on the phone, as root, under any Linux with /dev/mem
  lake-log.py RAW        parse a raw dump of the 1MB log region

The log lives in the unused tail of the splash carve-out, which Linux maps
no-map: read() of /dev/mem refuses it on arm64, mmap() works.
"""
import mmap
import os
import struct
import sys

LOG_BASE, LOG_SIZE = 0x9de00000, 0x100000


def read_devmem() -> bytes:
    fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
    m = mmap.mmap(fd, LOG_SIZE, mmap.MAP_SHARED, mmap.PROT_READ, offset=LOG_BASE)
    os.close(fd)
    return bytes(m)


def main() -> None:
    raw = open(sys.argv[1], "rb").read() if len(sys.argv) > 1 else read_devmem()
    if raw[:8] != b"LAKELOG1":
        sys.exit(f"no lake log (magic {raw[:8]!r})")
    boots, length = struct.unpack_from("<II", raw, 8)
    print(f"# boots={boots} len={length}")
    sys.stdout.write(raw[24:24 + length].decode(errors="replace"))


if __name__ == "__main__":
    main()
