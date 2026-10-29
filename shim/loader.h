#pragma once
#include <stdint.h>

/* Boot the SWEETXNU payload found in the ABL ramdisk; returns only on error. */
const char *loader_boot(uintptr_t dtb);
/* The payload's xnu command line, or 0 without a payload. */
const char *loader_cmdline(uintptr_t dtb);
