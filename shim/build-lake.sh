#!/bin/bash
# Build the lake shim with clang/lld and pack a Motorola boot v1 image
# (gzip Image + stock appended DTB) for `fastboot boot`.
# Usage: LAKE_STOCK_BOOT=stock-boot.img build-lake.sh [payload]
set -euo pipefail
cd "$(dirname "$0")"
CC="clang --target=aarch64-none-elf"
# -fno-jump-tables: clang otherwise turns switches into tables of absolute pointers
CFLAGS="${EXTRA_CFLAGS:-} -O2 -fno-jump-tables -ffreestanding -nostdlib -fno-pic -mgeneral-regs-only -mstrict-align -fno-stack-protector -Wall -Wextra"
OUT=out-lake
SRCS="start.S lake.c console.c lakelog.c spmi.c sdhc.c touch.c mmu.c fdt.c loader.c"
mkdir -p $OUT
objs=()
for f in $SRCS; do
	$CC $CFLAGS -c $f -o $OUT/${f%.*}.o
	objs+=($OUT/${f%.*}.o)
done
# The image runs wherever ABL puts it with no relocation pass: only the
# header's image_size (start.o, linked at 0) may be an absolute address.
abs=$(llvm-readelf -r "${objs[@]:1}" | grep -c ABS64 || true)
[ "$abs" = 0 ] || { echo "error: $abs absolute relocations in C objects" >&2; exit 1; }
ld.lld -static -T shim.ld --build-id=none -N "${objs[@]}" -o $OUT/shim.elf
llvm-objcopy -O binary $OUT/shim.elf $OUT/Image
# ABL copies the file; pad to image_size so .bss/stacks exist as zeroed bytes.
size=$(llvm-nm $OUT/shim.elf | awk '$3=="_end"{print strtonum("0x"$1)}')
truncate -s "$size" $OUT/Image
: "${LAKE_STOCK_BOOT:?set LAKE_STOCK_BOOT to a backup of the stock boot partition}"
python3 ../tools/mklakeboot.py "$LAKE_STOCK_BOOT" $OUT/Image ${1:+"$1"} $OUT/shim-boot.img
