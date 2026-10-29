/*
 * Touch probe: can bare metal read lake's touchscreen?
 *
 * The panel is a Novatek NT36672A (touch and display in one IC), so the
 * touch side is powered whenever ABL has the display on. Its firmware
 * answers at I2C address 0x01 on BLSP1 QUP3 (GPIO 10/11), reset is GPIO 66
 * (active low), the event line GPIO 67. ABL leaves the QUP3 core clock off.
 *
 * This clocks QUP3 from XO, muxes the pins, drives the QUP as an I2C master
 * (QUP v2 tags, FIFO mode, polled, 400 kHz), reads the controller
 * parameters (0x78) and then polls the touch reports (0x00) for a while,
 * logging every change and drawing a dot where each finger is. Reads only.
 */
#include <stdint.h>
#include "console.h"
#include "touch.h"

#define GCC_BASE		0x00100000UL
#define GCC_BLSP1_AHB_CBCR	0x17004
#define GCC_APCS_CLOCK_VOTE	0x52004
#define GCC_BLSP1_AHB_VOTE	(1U << 17)
#define GCC_QUP3_I2C_CBCR	0x1d008
#define GCC_QUP3_I2C_RCGR	0x1d020		/* CMD; CFG at +4 */
#define RCG_UPDATE		(1U << 0)
#define CBCR_CLK_OFF		(1U << 31)

#define TLMM_NORTH		0x03900000UL
#define GPIO_CFG(n)		(TLMM_NORTH + 0x1000UL * (n))
#define GPIO_IO(n)		(GPIO_CFG(n) + 4)
#define GPIO_SDA		10
#define GPIO_SCL		11
#define GPIO_TS_RESET		66
#define GPIO_TS_IRQ		67
#define FUNC_BLSP_I2C3		2

#define QUP			0x0c177000UL	/* blsp_i2c3, qcom,i2c-qup-v2.2.1 */
#define QUP_CONFIG		0x000
#define QUP_STATE		0x004
#define QUP_IO_MODE		0x008
#define QUP_SW_RESET		0x00c
#define QUP_OPERATIONAL		0x018
#define QUP_ERROR_FLAGS		0x01c
#define QUP_HW_VERSION		0x030
#define QUP_MX_OUTPUT_CNT	0x100
#define QUP_OUT_FIFO		0x110
#define QUP_MX_WRITE_CNT	0x150
#define QUP_MX_INPUT_CNT	0x200
#define QUP_MX_READ_CNT		0x208
#define QUP_IN_FIFO		0x218
#define QUP_I2C_CLK_CTL		0x400
#define QUP_I2C_STATUS		0x404
#define QUP_I2C_MASTER_GEN	0x408

#define STATE_RESET		0
#define STATE_RUN		1
#define STATE_PAUSE		3
#define STATE_VALID		(1U << 2)
#define STATE_I2C_MAST_GEN	(1U << 4)
#define CONFIG_I2C		(2U << 8 | 7)	/* I2C mini core, N = 8 bits */
#define CONFIG_NO_INPUT		(1U << 7)
#define IO_REPACK		(3U << 14)
#define V2_TAGS_EN		1
#define CONFIG_DURING_RUN	(1U << 31)
#define OP_OUT_SVC		(1U << 8)
#define OP_IN_SVC		(1U << 9)
#define OP_OUTPUT_DONE		(1U << 10)
#define OP_INPUT_DONE		(1U << 11)
#define I2C_BUS_ACTIVE		(1U << 8)
#define I2C_ERRORS		0x38000fcU
#define I2C_NACK		(1U << 3)
#define QUP_ERRORS		0x7cU
/* 400 kHz from 19.2 MHz, Linux i2c-qup fast mode: fs_div 28, hs_div 3 */
#define CLK_CTL_400K		(14U << 16 | 3U << 8 | 28)

#define TAG_START		0x81
#define TAG_DATA_WR		0x82
#define TAG_DATA_RD_STOP	0x87

#define NVT_ADDR		0x01
#define NVT_TOUCH		0x00
#define NVT_PARAMS		0x78
#define NVT_PARAMS_SIZE		15

static inline uint32_t rd32(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline void wr32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

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

static int poll32(uintptr_t a, uint32_t mask, uint32_t want, unsigned us)
{
	uint64_t end = ticks() + (uint64_t)us * 192 / 10;
	while ((rd32(a) & mask) != want)
		if (ticks() > end)
			return -1;
	return 0;
}

static unsigned in_fifo, out_fifo;
static uint32_t last_bus, last_qup, last_op;

static int qup_state(uint32_t s)
{
	if (poll32(QUP + QUP_STATE, STATE_VALID, STATE_VALID, 1000))
		return -1;
	wr32(QUP + QUP_STATE, s);
	return poll32(QUP + QUP_STATE, STATE_VALID | 3, STATE_VALID | s, 1000);
}

/* Errors latch in I2C_STATUS / ERROR_FLAGS: remember and clear them. */
static int qup_failed(void)
{
	uint32_t bus = rd32(QUP + QUP_I2C_STATUS) & I2C_ERRORS;
	uint32_t qup = rd32(QUP + QUP_ERROR_FLAGS) & QUP_ERRORS;

	if (!bus && !qup)
		return 0;
	last_bus = bus;
	last_qup = qup;
	wr32(QUP + QUP_I2C_STATUS, bus);
	wr32(QUP + QUP_ERROR_FLAGS, qup);
	return 1;
}

static int qup_wait_op(uint32_t mask, unsigned us)
{
	uint64_t end = ticks() + (uint64_t)us * 192 / 10;
	uint32_t op;

	while (!((op = rd32(QUP + QUP_OPERATIONAL)) & mask)) {
		if (qup_failed())
			return -2;
		if (ticks() > end) {
			last_op = op;
			return -1;
		}
	}
	wr32(QUP + QUP_OPERATIONAL, op & (OP_OUT_SVC | OP_IN_SVC));
	return qup_failed() ? -2 : 0;
}

static void fifo_put(const uint8_t *b, unsigned n)
{
	for (unsigned i = 0; i < n; i += 4) {
		uint32_t w = 0;
		for (unsigned j = 0; j < 4 && i + j < n; j++)
			w |= (uint32_t)b[i + j] << (8 * j);
		wr32(QUP + QUP_OUT_FIFO, w);
	}
}

/*
 * Write the register byte, repeated start, read n bytes (as Linux i2c-qup
 * does two messages in FIFO mode). 0, -1 timeout, -2 bus/QUP error (NACK
 * in last_bus bit 3), -3 bad state.
 */
static int i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, unsigned n)
{
	int rc;

	wr32(QUP + QUP_SW_RESET, 1);
	if (poll32(QUP + QUP_STATE, STATE_VALID | 3, STATE_VALID | STATE_RESET, 1000))
		return -3;
	wr32(QUP + QUP_CONFIG, CONFIG_I2C);
	wr32(QUP + QUP_I2C_MASTER_GEN, V2_TAGS_EN);
	if (poll32(QUP + QUP_STATE, STATE_I2C_MAST_GEN, STATE_I2C_MAST_GEN, 1000))
		return -3;
	wr32(QUP + QUP_MX_OUTPUT_CNT, 0);
	wr32(QUP + QUP_MX_INPUT_CNT, 0);
	wr32(QUP + QUP_IO_MODE, IO_REPACK);

	/* Message 1: START addr+W, one data byte (the register), no stop */
	const uint8_t wr[5] = { TAG_START, (uint8_t)(addr << 1), TAG_DATA_WR, 1, reg };
	wr32(QUP + QUP_MX_WRITE_CNT, sizeof(wr));
	wr32(QUP + QUP_CONFIG, CONFIG_I2C | CONFIG_NO_INPUT);
	if (qup_state(STATE_RUN))
		return -3;
	wr32(QUP + QUP_I2C_CLK_CTL, CLK_CTL_400K);
	if (qup_state(STATE_PAUSE))
		return -3;
	fifo_put(wr, sizeof(wr));
	if (qup_state(STATE_RUN))
		return -3;
	if ((rc = qup_wait_op(OP_OUT_SVC | OP_OUTPUT_DONE, 20000)))
		goto out;
	if (qup_state(STATE_PAUSE))
		return -3;

	/* Message 2: repeated START addr+R, n bytes, stop; 2 tag bytes come back first */
	const uint8_t rt[4] = { TAG_START, (uint8_t)(addr << 1 | 1), TAG_DATA_RD_STOP, (uint8_t)n };
	wr32(QUP + QUP_MX_WRITE_CNT, CONFIG_DURING_RUN | sizeof(rt));
	wr32(QUP + QUP_MX_READ_CNT, CONFIG_DURING_RUN | (n + 2));
	wr32(QUP + QUP_CONFIG, CONFIG_I2C);
	fifo_put(rt, sizeof(rt));
	if (qup_state(STATE_RUN))
		return -3;
	if ((rc = qup_wait_op(OP_INPUT_DONE, 20000)))
		goto out;
	for (unsigned got = 0; got < n + 2;) {
		uint32_t w = rd32(QUP + QUP_IN_FIFO);
		for (unsigned j = 0; j < 4 && got < n + 2; j++, got++)
			if (got >= 2)
				buf[got - 2] = (uint8_t)(w >> (8 * j));
	}
	if (poll32(QUP + QUP_I2C_STATUS, I2C_BUS_ACTIVE, 0, 5000))
		rc = -1;
out:
	qup_state(STATE_RESET);
	return rc;
}

static void dot(int x, int y, uint32_t c)
{
	for (int dy = -6; dy <= 6; dy++)
		for (int dx = -6; dx <= 6; dx++) {
			int px = x + dx, py = y + dy;
			if (px >= 0 && px < FB_W && py >= 0 && py < FB_H)
				con_fb[py * (FB_STRIDE / 4) + px] = c;
		}
}

static int clocks_on(void)
{
	wr32(GCC_BASE + GCC_APCS_CLOCK_VOTE, rd32(GCC_BASE + GCC_APCS_CLOCK_VOTE) | GCC_BLSP1_AHB_VOTE);
	wr32(GCC_BASE + GCC_QUP3_I2C_RCGR + 4, 0);	/* CFG: source XO, no divider */
	wr32(GCC_BASE + GCC_QUP3_I2C_RCGR, rd32(GCC_BASE + GCC_QUP3_I2C_RCGR) | RCG_UPDATE);
	poll32(GCC_BASE + GCC_QUP3_I2C_RCGR, RCG_UPDATE, 0, 10000);
	wr32(GCC_BASE + GCC_QUP3_I2C_CBCR, rd32(GCC_BASE + GCC_QUP3_I2C_CBCR) | 1);
	int rc = poll32(GCC_BASE + GCC_QUP3_I2C_CBCR, CBCR_CLK_OFF, 0, 10000) |
		 poll32(GCC_BASE + GCC_BLSP1_AHB_CBCR, CBCR_CLK_OFF, 0, 10000);
	con_hex2("TS CLK", "qup3=", rd32(GCC_BASE + GCC_QUP3_I2C_CBCR), "ahb=",
		 rd32(GCC_BASE + GCC_BLSP1_AHB_CBCR));
	return rc;
}

void test_touch(unsigned poll_s)
{
	uint8_t p[NVT_PARAMS_SIZE] = { 0 };
	int rc;

	con_hex2("TS GPIO", "rst=", rd32(GPIO_CFG(GPIO_TS_RESET)) << 8 | rd32(GPIO_IO(GPIO_TS_RESET)),
		 "irq=", rd32(GPIO_CFG(GPIO_TS_IRQ)) << 8 | rd32(GPIO_IO(GPIO_TS_IRQ)));
	if (clocks_on()) {
		con_str("TS", "QUP3 clocks stay off, stop");
		return;
	}
	/* SDA/SCL: blsp_i2c3, 2 mA, no pull (Linux i2c3_default) */
	wr32(GPIO_CFG(GPIO_SDA), FUNC_BLSP_I2C3 << 2);
	wr32(GPIO_CFG(GPIO_SCL), FUNC_BLSP_I2C3 << 2);

	wr32(QUP + QUP_SW_RESET, 1);
	if (poll32(QUP + QUP_STATE, STATE_VALID, STATE_VALID, 10000)) {
		con_str("TS", "QUP3 never valid, stop");
		return;
	}
	uint32_t io = rd32(QUP + QUP_IO_MODE);
	static const unsigned blk[4] = { 4, 16, 32, 0 };
	out_fifo = blk[io & 3] * (2U << ((io >> 2) & 7));
	in_fifo = blk[(io >> 5) & 3] * (2U << ((io >> 7) & 7));
	con_hex2("TS QUP", "ver=", rd32(QUP + QUP_HW_VERSION), "io=", io);
	con_hex2(" fifo", "in=", in_fifo, "out=", out_fifo);

	rc = i2c_read(NVT_ADDR, NVT_PARAMS, p, NVT_PARAMS_SIZE);
	con_hex2("TS 0x78", "rc=", (uint32_t)rc, "bus/qup=", last_bus << 8 | last_qup);
	if (rc) {
		/* Out of reset (active low, Linux waits 100 ms) and try again */
		wr32(GPIO_IO(GPIO_TS_RESET), 2);
		wr32(GPIO_CFG(GPIO_TS_RESET), 1U << 9);
		udelay(150000);
		rc = i2c_read(NVT_ADDR, NVT_PARAMS, p, NVT_PARAMS_SIZE);
		con_hex2("TS 0x78", "rc=", (uint32_t)rc, "bus/qup=", last_bus << 8 | last_qup);
		if (rc) {
			con_hex("TS op", last_op);
			return;
		}
	}
	unsigned w = (unsigned)p[4] << 8 | p[5], h = (unsigned)p[6] << 8 | p[7];
	unsigned max = p[9];
	con_hex2("TS SIZE", "w=", w, "h=", h);
	con_hex2("TS PARM", "max=", max << 8 | p[10], "irq/id=", (uint32_t)p[11] << 8 | p[14]);
	con_hex2(" raw", "0-3=", (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3],
		 "8-b=", (uint32_t)p[8] << 24 | p[9] << 16 | p[10] << 8 | p[11]);
	if (max == 0 || max > 10)
		max = 10;
	if (max * 6 + 2 > in_fifo)
		max = (in_fifo - 2) / 6;
	if (w == 0 || w > 4096 || h == 0 || h > 4096)
		w = FB_W, h = FB_H;

	con_str("TS", "TOUCH THE SCREEN NOW");
	uint8_t prev[60] = { 0 }, ev[60];
	unsigned lines = 0, errors = 0, edges = 0, reads = 0;
	uint32_t irq_prev = rd32(GPIO_IO(GPIO_TS_IRQ)) & 1;
	uint64_t end = ticks() + (uint64_t)poll_s * 19200000;
	while (ticks() < end) {
		uint32_t irq = rd32(GPIO_IO(GPIO_TS_IRQ)) & 1;
		edges += irq != irq_prev;
		irq_prev = irq;
		rc = i2c_read(NVT_ADDR, NVT_TOUCH, ev, max * 6);
		reads++;
		if (rc) {
			if (errors++ < 3)
				con_hex2("TS rd", "rc=", (uint32_t)rc, "bus/qup=", last_bus << 8 | last_qup);
			udelay(10000);
			continue;
		}
		for (unsigned i = 0; i < max; i++) {
			const uint8_t *t = ev + 6 * i;
			int same = 1;
			for (int k = 0; k < 6; k++)
				same &= t[k] == prev[6 * i + k];
			if (same || t[0] == 0xff)
				continue;
			unsigned x = (unsigned)t[1] << 4 | t[3] >> 4, y = (unsigned)t[2] << 4 | (t[3] & 0xf);
			unsigned type = t[0] & 7, slot = t[0] >> 3;
			if (type == 1 || type == 2)
				dot((int)(x * FB_W / w), (int)(y * FB_H / h), i ? GREEN : YELLOW);
			if (lines < 24 || type != 2) {
				lines++;
				con_hex2("TS ev", "slot/type=", slot << 8 | type, "x/y=", x << 16 | y);
			}
		}
		for (unsigned k = 0; k < max * 6; k++)
			prev[k] = ev[k];
		udelay(10000);
	}
	con_hex2("TS END", "reads=", reads, "err/edges=", errors << 16 | edges);
}
