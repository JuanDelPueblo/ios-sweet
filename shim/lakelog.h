#pragma once
#include <stdint.h>

/*
 * Text log in the unused tail of the splash carve-out (cont_splash is
 * 0x9d400000 + 35MB, the panel only scans out the first 9.8MB): it survives
 * a warm reset, ABL, fastboot and a Linux boot. It stays inside the 32MB
 * block 0x9c000000-0x9dffffff that XNU maps Normal-NC for its early console
 * (the next block holds the venus firmware carve-out at 0x9f800000).
 */
#define LOG_BASE	0x9de00000UL
#define LOG_SIZE	0x00100000UL

void log_init(void);
uint32_t log_boots(void);
void log_puts(const char *s);
