/*
 * Persistent RAM log: "LAKELOG1", boot counter, text length, then text.
 * Written with the MMU off (Device memory) or through a Device mapping, so
 * plain byte stores land in DRAM. Read back from Linux with
 * tools/lake-log.py.
 */
#include "lakelog.h"

struct lakelog {
	char magic[8];
	uint32_t boots, len;
	uint64_t pad;
};

#define HDR	((volatile struct lakelog *)LOG_BASE)
#define TEXT	((volatile char *)(LOG_BASE + sizeof(struct lakelog)))
#define CAP	(LOG_SIZE - sizeof(struct lakelog))

static const char magic[8] __attribute__((nonstring)) = "LAKELOG1";

void log_init(void)
{
	int ok = 1;

	for (int i = 0; i < 8; i++)
		ok &= HDR->magic[i] == magic[i];
	HDR->boots = ok ? HDR->boots + 1 : 1;
	HDR->len = 0;
	for (int i = 0; i < 8; i++)
		HDR->magic[i] = magic[i];
	__asm__ volatile("dsb sy" ::: "memory");
}

uint32_t log_boots(void)
{
	return HDR->boots;
}

void log_puts(const char *s)
{
	uint32_t len = HDR->len;

	while (*s && len < CAP)
		TEXT[len++] = *s++;
	HDR->len = len;
	__asm__ volatile("dsb sy" ::: "memory");
}
