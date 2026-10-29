/*
 * Milestone L4 probe: can bare metal talk to the microSD card on sdhc_2?
 *
 * ABL leaves SDCC2 unclocked (its RCG is root-off) and nothing tells us
 * whether the card's rails (PM660L L5B VDD, L2B VDD-IO, owned by the RPM)
 * are on. This clocks the controller from XO (19.2 MHz), sets the SDC2 pads
 * like Linux, powers the bus with the MSM PWRCTL handshake (sdhci-msm v5),
 * then runs the SD identification sequence at 400 kHz and, if the card
 * answers, READS block 0 (the MBR) by PIO. Nothing is ever written to the
 * card: there is no write command in this file.
 */
#include <stdint.h>
#include "console.h"
#include "sdhc.h"

#define GCC_BASE		0x00100000UL
#define GCC_SDCC2_APPS_CBCR	0x14004
#define GCC_SDCC2_AHB_CBCR	0x14008
#define GCC_SDCC2_CMD_RCGR	0x14010
#define RCG_UPDATE		(1U << 0)
#define RCG_ROOT_OFF		(1U << 31)
#define CBCR_CLK_OFF		(1U << 31)

#define TLMM_NORTH		0x03900000UL
#define TLMM_SDC2_CTL		(TLMM_NORTH + 0x9b000)
#define TLMM_GPIO_IN(n)		(TLMM_NORTH + 0x1000UL * (n) + 0x4)
#define CD_GPIO			54	/* card detect, active high */

#define HC			0x0c084000UL	/* sdhc_2 */
#define SDHCI_ARG		0x08
#define SDHCI_XFER		0x0c	/* transfer mode (16) | command (16) << 16 */
#define SDHCI_RESP0		0x10
#define SDHCI_BUF		0x20
#define SDHCI_PSTATE		0x24
#define SDHCI_HOST		0x28
#define SDHCI_PWR		0x29
#define SDHCI_CLK		0x2c
#define SDHCI_TOUT		0x2e
#define SDHCI_RESET		0x2f
#define SDHCI_INT		0x30	/* normal (16) | error (16) << 16 */
#define SDHCI_INT_EN		0x34
#define SDHCI_BLK		0x04	/* block size (16) | count (16) << 16 */
#define SDHCI_VER		0xfe
#define MSM_VENDOR_SPEC		0x20c	/* sdhci-msm v5 offsets */
#define MSM_VENDOR_SPEC_POR	0xa9c
#define MSM_PWRCTL_STATUS	0x240
#define MSM_PWRCTL_CLEAR	0x248
#define MSM_PWRCTL_CTL		0x24c
#define PWRCTL_BUS_ON		(1U << 1)
#define PWRCTL_IO_HIGH		(1U << 3)
#define PWRCTL_BUS_SUCCESS	(1U << 0)
#define PWRCTL_IO_SUCCESS	(1U << 2)

#define INT_CMD_DONE		(1U << 0)
#define INT_XFER_DONE		(1U << 1)
#define INT_BUF_RD		(1U << 5)
#define INT_ERR			(1U << 15)

#define R_NONE			0x00
#define R_136			0x09	/* R2: 136-bit, CRC */
#define R_48			0x1a	/* R1/R6/R7: 48-bit, CRC, index */
#define R_48_NOCRC		0x02	/* R3 */
#define DATA_PRESENT		0x20

static inline uint32_t rd32(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline uint16_t rd16(uintptr_t a) { return *(volatile uint16_t *)a; }
static inline uint8_t rd8(uintptr_t a) { return *(volatile uint8_t *)a; }
static inline void wr32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline void wr16(uintptr_t a, uint16_t v) { *(volatile uint16_t *)a = v; }
static inline void wr8(uintptr_t a, uint8_t v) { *(volatile uint8_t *)a = v; }

static uint64_t ticks(void)
{
	uint64_t v;
	__asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(v));
	return v;
}

static void udelay(unsigned us)
{
	uint64_t end = ticks() + (uint64_t)us * 192 / 10;	/* 19.2 MHz */
	while (ticks() < end)
		;
}

/* Wait until (reg & mask) == want; 0 on success. */
static int poll32(uintptr_t a, uint32_t mask, uint32_t want, unsigned us)
{
	uint64_t end = ticks() + (uint64_t)us * 192 / 10;
	while ((rd32(a) & mask) != want)
		if (ticks() > end)
			return -1;
	return 0;
}

/* Ack whatever the MSM core asks for (Linux: sdhci_msm_handle_pwr_irq). */
static uint32_t pwrctl_ack(void)
{
	uint32_t st = rd32(HC + MSM_PWRCTL_STATUS), ack = 0;

	if (!st)
		return 0;
	wr32(HC + MSM_PWRCTL_CLEAR, st);
	if (st & (PWRCTL_BUS_ON | 1U))		/* bus on/off */
		ack |= PWRCTL_BUS_SUCCESS;
	if (st & (PWRCTL_IO_HIGH | (1U << 2)))	/* io high/low */
		ack |= PWRCTL_IO_SUCCESS;
	wr32(HC + MSM_PWRCTL_CTL, ack);
	return st;
}

/* Returns 0, or -1 timeout, or the error interrupt bits. */
static int cmd(unsigned idx, uint32_t arg, unsigned flags, uint32_t *resp)
{
	if (poll32(HC + SDHCI_PSTATE, 1, 0, 10000))
		return -2;
	wr32(HC + SDHCI_INT, 0xffffffff);
	wr32(HC + SDHCI_ARG, arg);
	wr32(HC + SDHCI_XFER, (flags & DATA_PRESENT ? 0x10 : 0) |	/* read */
			      (uint32_t)(idx << 8 | flags) << 16);
	uint64_t end = ticks() + 192000 * 10;	/* 100 ms */
	uint32_t st;
	while (!((st = rd32(HC + SDHCI_INT)) & (INT_CMD_DONE | INT_ERR)))
		if (ticks() > end)
			return -1;
	if (st & INT_ERR) {
		wr8(HC + SDHCI_RESET, 0x06);	/* CMD and DATA lines */
		poll32(HC + SDHCI_RESET & ~3UL, 0x06U << 24, 0, 10000);
		return (int)(st >> 16);
	}
	if (resp)
		for (int i = 0; i < 4; i++)
			resp[i] = rd32(HC + SDHCI_RESP0 + 4 * i);
	return 0;
}

static int read_block(uint32_t lba, uint32_t *buf)
{
	wr32(HC + SDHCI_BLK, 512 | 1U << 16);
	int rc = cmd(17, lba, R_48 | DATA_PRESENT, 0);
	if (rc)
		return rc;
	uint64_t end = ticks() + 192000 * 50;
	while (!(rd32(HC + SDHCI_INT) & (INT_BUF_RD | INT_ERR)))
		if (ticks() > end)
			return -3;
	if (rd32(HC + SDHCI_INT) & INT_ERR)
		return (int)(rd32(HC + SDHCI_INT) >> 16);
	for (int i = 0; i < 128; i++)
		buf[i] = rd32(HC + SDHCI_BUF);
	wr32(HC + SDHCI_INT, INT_BUF_RD);
	return poll32(HC + SDHCI_INT, INT_XFER_DONE, INT_XFER_DONE, 100000) ? -4 : 0;
}

void test_sdhc(void)
{
	uint32_t r[4] = { 0 }, mbr[128];
	int rc;

	con_hex2("SD CD", "gpio54=", rd32(TLMM_GPIO_IN(CD_GPIO)) & 1, "sdc2pad=", rd32(TLMM_SDC2_CTL));

	/* Clocks: AHB branch, then the apps RCG from XO (CFG 0: src XO, no divider) */
	wr32(GCC_BASE + GCC_SDCC2_AHB_CBCR, rd32(GCC_BASE + GCC_SDCC2_AHB_CBCR) | 1);
	wr32(GCC_BASE + GCC_SDCC2_CMD_RCGR + 4, 0);
	wr32(GCC_BASE + GCC_SDCC2_CMD_RCGR, rd32(GCC_BASE + GCC_SDCC2_CMD_RCGR) | RCG_UPDATE);
	poll32(GCC_BASE + GCC_SDCC2_CMD_RCGR, RCG_UPDATE, 0, 10000);
	wr32(GCC_BASE + GCC_SDCC2_APPS_CBCR, rd32(GCC_BASE + GCC_SDCC2_APPS_CBCR) | 1);
	int ck = poll32(GCC_BASE + GCC_SDCC2_APPS_CBCR, CBCR_CLK_OFF, 0, 10000) |
		 poll32(GCC_BASE + GCC_SDCC2_AHB_CBCR, CBCR_CLK_OFF, 0, 10000);
	con_hex2("SD CLK", "rcg=", rd32(GCC_BASE + GCC_SDCC2_CMD_RCGR), "apps=",
		 rd32(GCC_BASE + GCC_SDCC2_APPS_CBCR));
	if (ck) {
		con_str("SD", "clocks stay off, stop");
		return;
	}

	/* Pads as Linux sdc2_state_on: clk 16 mA no pull, cmd/data 10 mA pull-up */
	wr32(TLMM_SDC2_CTL, 7U << 6 | 3U << 11 | 4U << 3 | 3U << 9 | 4U);

	/* Reset, power the bus at 3.3 V (ack the PWRCTL requests), 400 kHz */
	wr8(HC + SDHCI_RESET, 1);
	udelay(1000);
	wr32(HC + MSM_VENDOR_SPEC, MSM_VENDOR_SPEC_POR);
	uint32_t p0 = pwrctl_ack();
	wr32(HC + SDHCI_INT_EN, 0xffffffff);
	wr8(HC + SDHCI_PWR, 0x0f);
	udelay(1000);
	uint32_t p1 = pwrctl_ack();
	udelay(1000);
	p1 |= pwrctl_ack() << 8;
	wr8(HC + SDHCI_TOUT, 0x0e);
	wr16(HC + SDHCI_CLK, 24 << 8 | 1);	/* 19.2 MHz / (2 * 24) = 400 kHz */
	int stable = poll32(HC + SDHCI_CLK & ~3UL, 2, 2, 10000);
	wr16(HC + SDHCI_CLK, 24 << 8 | 1 | 4);
	udelay(2000);	/* >= 74 clocks */
	con_hex2("SD HC", "ver=", rd16(HC + SDHCI_VER), "pstate=", rd32(HC + SDHCI_PSTATE));
	con_hex2(" pwr", "pwrctl=", p0 << 16 | p1, "pwr/clk=", (uint32_t)rd8(HC + SDHCI_PWR) << 16 |
		 rd16(HC + SDHCI_CLK) | (stable ? 0x80000000U : 0));

	/* Identification: CMD0, CMD8 (R7), ACMD41 until ready, CMD2, CMD3, CMD7 */
	cmd(0, 0, R_NONE, 0);
	udelay(1000);
	rc = cmd(8, 0x1aa, R_48, r);
	con_hex2("CMD8", "rc=", rc, "r7=", r[0]);
	if (rc) {
		con_str("SD", "no answer to CMD8: no card power?");
		return;
	}
	uint32_t ocr = 0;
	for (int i = 0; i < 200; i++) {
		if (cmd(55, 0, R_48, 0) || cmd(41, 0x40ff8000, R_48_NOCRC, r))
			break;
		ocr = r[0];
		if (ocr & 0x80000000U)
			break;
		udelay(10000);
	}
	con_hex("OCR", ocr);
	if (!(ocr & 0x80000000U))
		return;
	rc = cmd(2, 0, R_136, r);
	con_hex2("CID", "rc=", rc, "w3=", r[3]);
	rc = cmd(3, 0, R_48, r);
	uint32_t rca = r[0] >> 16;
	con_hex2("CMD3", "rc=", rc, "rca=", rca);
	rc = cmd(7, rca << 16, R_48 | 1, r);	/* R1b */
	con_hex2("CMD7", "rc=", rc, "r1=", r[0]);
#ifdef SD_R1B_NOW
	/* xnu's order: ACMD6 straight after CMD7's busy response */
	int n55 = cmd(55, rca << 16, R_48, 0);
	int n6 = n55 ? 0 : cmd(6, 2, R_48, r);
	con_hex2("ACMD6 now", "rc55=", (uint32_t)n55, "rc6=", (uint32_t)n6);
	con_hex2(" st", "int=", rd32(HC + SDHCI_INT), "pstate=", rd32(HC + SDHCI_PSTATE));
#endif

	rc = read_block(0, mbr);
	const uint8_t *b = (const uint8_t *)mbr;
	con_hex2("MBR", "rc=", rc, "sig=", (uint32_t)b[510] << 8 | b[511]);
	con_hex2(" types", "p1p2=", (uint32_t)b[0x1c2] << 8 | b[0x1d2], "p3=", b[0x1e2]);
	con_hex2(" p2", "lba=", mbr[0x1d6 / 4] >> 16 | mbr[0x1da / 4] << 16, 0, 0);
#ifdef SD_FAST
	/* What xnu does next: ACMD6 (4-bit bus), 19.2 MHz, one 8-sector CMD18 */
	uint32_t p2 = mbr[0x1d6 / 4] >> 16 | mbr[0x1da / 4] << 16;
	int rc55 = cmd(55, rca << 16, R_48, 0);
	int rc6 = rc55 ? 0 : cmd(6, 2, R_48, r);
	con_hex2("ACMD6", "rc55=", (uint32_t)rc55, "rc6=", (uint32_t)rc6);
	con_hex2(" st", "int=", rd32(HC + SDHCI_INT), "pstate=", rd32(HC + SDHCI_PSTATE));
	if (rc55 || rc6)
		return;
	wr8(HC + SDHCI_HOST, rd8(HC + SDHCI_HOST) | 0x02);
	wr16(HC + SDHCI_CLK, 1);
	poll32(HC + SDHCI_CLK & ~3UL, 2, 2, 10000);
	wr16(HC + SDHCI_CLK, 1 | 4);
	udelay(1000);
	wr32(HC + SDHCI_BLK, 512 | 8U << 16);
	if (poll32(HC + SDHCI_PSTATE, 3, 0, 10000)) {
		con_hex("CMD18 busy", rd32(HC + SDHCI_PSTATE));
		return;
	}
	wr32(HC + SDHCI_INT, 0xffffffff);
	wr32(HC + SDHCI_ARG, p2 + 2);		/* the HFS+ volume header sector */
	wr32(HC + SDHCI_XFER, 0x36 | (18U << 8 | R_48 | DATA_PRESENT) << 16);
	uint32_t first = 0, st = 0;
	for (int b = 0; b < 8; b++) {
		uint64_t end = ticks() + 192000 * 100;
		while (!((st = rd32(HC + SDHCI_INT)) & (INT_BUF_RD | INT_ERR)))
			if (ticks() > end)
				break;
		if (!(st & INT_BUF_RD)) {
			con_hex2("CMD18", "block=", b, "int=", st);
			return;
		}
		wr32(HC + SDHCI_INT, INT_BUF_RD);
		for (int i = 0; i < 128; i++) {
			uint32_t v = rd32(HC + SDHCI_BUF);
			if (b == 0 && i == 0)
				first = v;
		}
	}
	int done = poll32(HC + SDHCI_INT, INT_XFER_DONE, INT_XFER_DONE, 100000);
	con_hex2("CMD18", "hfs=", __builtin_bswap32(first), "done/int=", (uint32_t)done << 31 | rd32(HC + SDHCI_INT));
#endif
}
