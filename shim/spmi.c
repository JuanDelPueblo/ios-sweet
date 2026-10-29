/*
 * One-byte register access to the PMICs through the SDM660 SPMI PMIC
 * arbiter (v3, same channel layout as v2), following Linux spmi-pmic-arb.c:
 * the "core" APID table maps a PPID (sid << 8 | peripheral) to a channel;
 * writes go through "chnls", reads through "obsrvr", EE 0 (apps).
 */
#include "spmi.h"

#define CORE_BASE	0x0800f000UL
#define CHNLS_BASE	0x08400000UL
#define OBSRVR_BASE	0x09400000UL
#define CORE_SIZE	0x1000
#define APID_MAP(n)	(CORE_BASE + 0x800 + 4 * (n))
#define EE		0

#define ARB_CMD		0x00
#define ARB_STATUS	0x08
#define ARB_WDATA0	0x10
#define ARB_RDATA0	0x18

#define OP_EXT_WRITEL	0
#define OP_EXT_READL	1

#define STATUS_DONE	1u

static inline uint32_t rd32(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline void wr32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

static int find_apid(uint16_t ppid)
{
	for (int n = 0; 0x800 + 4 * n < CORE_SIZE; n++) {
		uint32_t v = rd32(APID_MAP(n));
		if (v && ((v >> 8) & 0xfff) == ppid)
			return n;
	}
	return -1;
}

static int run(uintptr_t base, uint32_t off, uint8_t opc, uint16_t addr)
{
	wr32(base + off + ARB_CMD, (uint32_t)opc << 27 | (addr & 0xff) << 4);	/* byte count 1 */
	for (int i = 0; i < 1000000; i++) {
		uint32_t st = rd32(base + off + ARB_STATUS);
		if (st & STATUS_DONE)	/* error bits are only valid with DONE */
			return st & ~STATUS_DONE;
	}
	return -2;
}

static int chan(uint8_t sid, uint16_t addr, uint32_t *off)
{
	int apid = find_apid((uint16_t)(sid << 8 | addr >> 8));

	if (apid < 0)
		return -1;
	*off = 0x1000 * EE + 0x8000 * (uint32_t)apid;
	return 0;
}

int spmi_read(uint8_t sid, uint16_t addr, uint8_t *val)
{
	uint32_t off;
	int rc;

	if (chan(sid, addr, &off))
		return -1;
	rc = run(OBSRVR_BASE, off, OP_EXT_READL, addr);
	if (!rc)
		*val = rd32(OBSRVR_BASE + off + ARB_RDATA0) & 0xff;
	return rc;
}

int spmi_write(uint8_t sid, uint16_t addr, uint8_t val)
{
	uint32_t off;

	if (chan(sid, addr, &off))
		return -1;
	wr32(CHNLS_BASE + off + ARB_WDATA0, val);
	return run(CHNLS_BASE, off, OP_EXT_WRITEL, addr);
}
