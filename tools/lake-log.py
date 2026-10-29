#!/usr/bin/env python3
"""Print the lake RAM log (shim/lakelog.h).

  lake-log.py            on the phone, as root, under any Linux with /dev/mem
  lake-log.py RAW        parse a raw dump of the 1MB log region

Header: "LAKELOG1", u32 boots, u32 text length, u64 xnu start.s breadcrumb
(0x11 entry .. 0x15 jump to arm_init, 0x21 arm_init), then the text. The
last 64 bytes may hold an early exception record ("LAKEEXC1").

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
    boots, length, crumb = struct.unpack_from("<IIQ", raw, 8)
    print(f"# boots={boots} len={length} xnu start.s breadcrumb={crumb:#x}")
    sys.stdout.write(raw[24:24 + length].decode(errors="replace"))
    exc = raw[LOG_SIZE - 64:]
    if exc[:8] == b"LAKEEXC1":   # early exception, xnu 0084 LAKE_LOWVEC
        slot, esr, elr, far, spsr = struct.unpack_from("<5Q", exc, 8)
        print(f"\n# early exception: vector slot {slot} ESR={esr:#x} ELR={elr:#x} FAR={far:#x} SPSR={spsr:#x}")


if __name__ == "__main__":
    main()
