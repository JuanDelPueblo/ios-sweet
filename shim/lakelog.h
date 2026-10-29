#pragma once
#include <stdint.h>

/*
 * Text log in the unused tail of the splash carve-out (cont_splash is
 * 0x9d400000 + 35MB, the panel only scans out the first 9.8MB). Verified
 * 2026-10-29: it survives a warm reset, ABL, fastboot and a Linux boot.
 */
#define LOG_BASE	0x9f000000UL
#define LOG_SIZE	0x00100000UL

void log_init(void);
uint32_t log_boots(void);
void log_puts(const char *s);
