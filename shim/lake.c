/*
 * Lake (Moto G7 Plus, SDM636) boot shim. With an xnu payload in the ramdisk
 * CPU0 hands over to the first gold core, which boots it (milestone L3);
 * without one, milestones L1-L2: one boot answers the questions that decide
 * the XNU port on this SoC.
 *   - SPMI: PM660 PON reboot reason, so every reset lands in fastboot
 *   - CPU0 is a Kryo 260 Silver (A53, no 16K granule): PSCI CPU_ON the first
 *     gold core (MPIDR 0x100) and CPU_OFF CPU0
 *   - on the gold core: SMP for the other gold cores, GICv3 + virtual timer
 *     (INTID 19), the microSD rails and controller (L2), then the 16K MMU
 * Everything on screen also goes to the RAM log (lakelog.h).
 */
#include <stdint.h>
#include "console.h"
#include "fdt.h"
#include "lakelog.h"
#include "loader.h"
#include "mmu.h"
#include "sdhc.h"
#include "spmi.h"
#include "touch.h"

#define PSHOLD			0x010ac000UL
#define GCC_BASE		0x00100000UL
#define GCC_SDCC1_APPS_CBCR	0x16004
#define GCC_SDCC2_APPS_CBCR	0x14004
#define GCC_SDCC2_AHB_CBCR	0x14008
#define GCC_QUP3_I2C_CBCR	0x1d008


#define APSS_WDT_BASE		0x17817000UL	/* qcom,msm-watchdog, kpss register layout */
#define WDT_RST			0x04
#define WDT_EN			0x08
#define WDT_BARK_TIME		0x10
#define WDT_BITE_TIME		0x14
#define WDT_HZ			32768

#define PON_PERPH_TYPE		0x804	/* PM660 = SID 0, PON at 0x800 */
#define PON_PERPH_SUBTYPE	0x805
#define PON_SOFT_RB_SPARE	0x88f	/* reboot reason in bits 7:1 (pm8998-style PON) */
#define PON_REASON_BOOTLOADER	(2 << 1)

#define GICD_BASE		0x17a00000UL
#define GICR_BASE		0x17b00000UL
#define GICR_SIZE		0x100000UL
#define GICR_PE_SIZE		0x20000UL
#define GICR_TYPER		0x08
#define GICR_WAKER		0x14
#define GICR_SGI		0x10000
#define GICR_IGROUPR0		(GICR_SGI + 0x080)
#define GICR_ISENABLER0		(GICR_SGI + 0x100)
#define GICR_ICENABLER0		(GICR_SGI + 0x180)
#define GICR_IPRIORITYR		(GICR_SGI + 0x400)
#define GICR_ISPENDR0		(GICR_SGI + 0x200)
#define GICR_ISACTIVER0		(GICR_SGI + 0x300)
#define VTIMER_INTID		19	/* sdm630.dtsi timer: <GIC_PPI 3 ...> */
#define SGI_INTID		1

#define PSCI_VERSION		0x84000000UL
#define PSCI_CPU_OFF		0x84000002UL
#define PSCI_CPU_ON		0xc4000003UL
#define PSCI_AFFINITY_INFO	0xc4000004UL
#define PSCI_MIGRATE_INFO_TYPE	0x84000006UL
#define PSCI_SYSTEM_RESET	0x84000009UL
#define PSCI_FEATURES		0x8400000aUL
#define PSCI_SYSTEM_RESET2	0xc4000012UL

#define NR_CPUS			8
#define GOLD0_IDX		4	/* secondary_entry context id = Linux cpu index order */
#define GOLD_MPIDR(idx)		(0x100UL + (idx) - GOLD0_IDX)

#define rsr(r) ({ uint64_t _v; __asm__ volatile("mrs %0, " r : "=r"(_v)); _v; })
#define wsr(r, v) __asm__ volatile("msr " r ", %0" : : "r"((uint64_t)(v)))
#define isb() __asm__ volatile("isb" ::: "memory")
#define dsb() __asm__ volatile("dsb sy" ::: "memory")

static inline uint32_t rd32(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline uint64_t rd64(uintptr_t a) { return *(volatile uint64_t *)a; }
static inline void wr32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline void wr8(uintptr_t a, uint8_t v) { *(volatile uint8_t *)a = v; }

extern char secondary_entry[];

static const char *stage;	/* set at runtime: no absolute pointers in data */
static uintptr_t boot_dtb;
static const char *xnu_cmdline;	/* payload present */
static volatile uint32_t gold_alive, cpu0_handed;
static volatile uint64_t cpu0_off_ret;
static volatile uint64_t sec_mpidr[NR_CPUS], sec_mmfr0[NR_CPUS];
static volatile uint32_t irq_count, irq_other, irq_last;
static volatile int irq_src;

static uint64_t smc(uint64_t fn, uint64_t a1, uint64_t a2, uint64_t a3)
{
	register uint64_t x0 __asm__("x0") = fn;
	register uint64_t x1 __asm__("x1") = a1;
	register uint64_t x2 __asm__("x2") = a2;
	register uint64_t x3 __asm__("x3") = a3;
	__asm__ volatile("smc #0"
			 : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
			 :
			 : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11",
			   "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

static uint64_t now(void)
{
	isb();
	return rsr("cntpct_el0");
}

static void delay_ms(unsigned ms)
{
	uint64_t end = now() + rsr("cntfrq_el0") / 1000 * ms;
	while (now() < end)
		;
}

/* Arm the PON reboot reason; returns the SPMI status of the write. */
static int set_reason_bootloader(void)
{
	uint8_t v = 0;
	int rc = spmi_read(0, PON_SOFT_RB_SPARE, &v);

	if (rc)
		return rc;
	return spmi_write(0, PON_SOFT_RB_SPARE, (v & 1) | PON_REASON_BOOTLOADER);
}

/*
 * Hardware safety net for hangs: nobody pets the APSS watchdog, so it
 * bites (resets the SoC) after secs; the PON reason then lands in fastboot.
 */
static void wdt_arm(unsigned secs)
{
	uint32_t bark = secs * WDT_HZ + WDT_HZ;	/* after the bite: no bark IRQ */

	if (bark > 0xfffff)
		bark = 0xfffff;		/* 20-bit counter: 32 s would wrap to 0 */
	wr32(APSS_WDT_BASE + WDT_EN, 0);
	wr32(APSS_WDT_BASE + WDT_RST, 1);
	wr32(APSS_WDT_BASE + WDT_BARK_TIME, bark);
	wr32(APSS_WDT_BASE + WDT_BITE_TIME, secs * WDT_HZ);
	wr32(APSS_WDT_BASE + WDT_EN, 1);
	dsb();
}

static void __attribute__((noreturn)) reset_to_fastboot(unsigned after_ms)
{
	con_str("RESET", "to fastboot");
	delay_ms(after_ms);
	set_reason_bootloader();
	dsb();
	smc(PSCI_SYSTEM_RESET, 0, 0, 0);
	wr32(PSHOLD, 0);
	for (;;)
		__asm__ volatile("wfe");
}

/* ---- exceptions ---- */

static void irq_handler(void)
{
	uint64_t iar = rsr("icc_iar1_el1");
	uint32_t id = iar & 0xffffff;

	if (id >= 1020 && id <= 1023)
		return;	/* spurious */
	irq_last = id;
	if (id == (uint32_t)irq_src)
		irq_count++;
	else
		irq_other++;
	if (id == VTIMER_INTID)
		wsr("cntv_tval_el0", rsr("cntfrq_el0") / 100);	/* 10 ms */
	wsr("icc_eoir1_el1", iar);
	isb();
}

void exc_handler(uint64_t slot, uint64_t *regs)
{
	if (slot == 5) {	/* IRQ from current EL, SPx */
		irq_handler();
		return;
	}
	con_clear(BLUE);
	switch (slot & 3) {
	case 0: con_str("PANIC", "SYNC"); break;
	case 1: con_str("PANIC", "IRQ"); break;
	case 2: con_str("PANIC", "FIQ"); break;
	default: con_str("PANIC", "SERROR"); break;
	}
	con_hex2("SLOT", "n=", slot, "cpu=", rsr("mpidr_el1"));
	con_str("STAGE", stage);
	con_hex("ESR", rsr("esr_el1"));
	con_hex("ELR", rsr("elr_el1"));
	con_hex("FAR", rsr("far_el1"));
	con_hex("SPSR", rsr("spsr_el1"));
	con_hex("X0", regs[0]);
	con_hex("X1", regs[1]);
	con_hex("X30", regs[29]);	/* stashed lr is just above the frame */
	reset_to_fastboot(10000);
}

/* ---- gold cluster ---- */

static uintptr_t find_gicr(void)
{
	uint64_t aff = rsr("mpidr_el1");
	uint32_t want = (aff & 0xffffff) | ((aff >> 8) & 0xff000000);

	for (uintptr_t off = 0; off < GICR_SIZE; off += GICR_PE_SIZE) {
		uint64_t typer = rd64(GICR_BASE + off + GICR_TYPER);
		if ((uint32_t)(typer >> 32) == want)
			return GICR_BASE + off;
		if (typer & (1 << 4))	/* Last */
			break;
	}
	return 0;
}

static void gicd_wait_rwp(void)
{
	for (int i = 0; i < 1000000 && (rd32(GICD_BASE) & (1U << 31)); i++)
		;
}

/* Run one interrupt source for ~300 ms and report what the GIC saw. */
static void irq_window(const char *label, uintptr_t r, int src)
{
	uint64_t f = rsr("cntfrq_el0");

	irq_count = irq_other = 0;
	irq_last = 0x3ff;
	irq_src = src;
	if (src == VTIMER_INTID) {
		wsr("cntv_tval_el0", f / 100);
		wsr("cntv_ctl_el0", 1);
	}
	isb();
	uint64_t t = now() + f / 50;
	while (now() < t)
		;
	uint32_t pend = rd32(r + GICR_ISPENDR0);
	uint32_t hppi = rsr("icc_hppir1_el1");

	__asm__ volatile("msr daifclr, #2" ::: "memory");
	t = now() + f * 3 / 10;
	uint64_t next = now();
	for (int sent = 0; now() < t;)
		if (src == SGI_INTID && sent < 10 && now() >= next) {
			uint64_t m = rsr("mpidr_el1");
			wsr("icc_sgi1r_el1", ((m >> 8) & 0xff) << 16 | (uint64_t)SGI_INTID << 24 |
					     (1U << (m & 0xf)));
			isb();
			sent++;
			next += f / 100;
		}
	__asm__ volatile("msr daifset, #2" ::: "memory");
	wsr("cntv_ctl_el0", 0);
	isb();

	con_hex2(label, "irqs=", irq_count, "other=", irq_other);
	con_hex2(" pre", "pend=", pend, "hppi=", hppi);
}

static void test_gic(void)
{
	stage = "gic";
	wsr("icc_sre_el1", rsr("icc_sre_el1") | 1);
	isb();
	if (!(rsr("icc_sre_el1") & 1)) {
		con_str("GIC", "no sysreg iface");
		return;
	}
	uint32_t dctlr = rd32(GICD_BASE);
	wr32(GICD_BASE, dctlr | 0x13);	/* EnableGrp1NS, EnableGrp1A, ARE_NS, as Linux */
	gicd_wait_rwp();
	con_hex2("GICD", "was=", dctlr, "now=", rd32(GICD_BASE));

	uintptr_t r = find_gicr();
	if (!r) {
		con_str("GICR", "not found");
		return;
	}
	uint32_t waker = rd32(r + GICR_WAKER);
	if (waker & 2) {
		wr32(r + GICR_WAKER, waker & ~2U);
		for (int i = 0; i < 100000 && (rd32(r + GICR_WAKER) & 4); i++)
			;
	}
	con_hex2("GICR", "base=", r, "waker=", rd32(r + GICR_WAKER));

	wr32(r + GICR_IGROUPR0, 0xffffffff);
	for (int id = 0; id < 32; id++)
		wr8(r + GICR_IPRIORITYR + id, 0xa0);
	wr32(r + GICR_ISENABLER0, (1U << VTIMER_INTID) | (1U << SGI_INTID));
	wsr("icc_ctlr_el1", rsr("icc_ctlr_el1") & ~2ULL);	/* EOImode 0 */
	wsr("icc_pmr_el1", 0xf0);
	wsr("icc_bpr1_el1", 0);
	wsr("icc_igrpen1_el1", 1);
	isb();

	stage = "gic vtimer";
	irq_window("VTIMER", r, VTIMER_INTID);
	stage = "gic sgi";
	irq_window("SGI1", r, SGI_INTID);
	wr32(r + GICR_ICENABLER0, 0xffffffff);
	isb();
}

static void test_smp(void)
{
	int64_t ret[NR_CPUS];

	stage = "smp";
	for (int i = GOLD0_IDX + 1; i < NR_CPUS; i++)
		ret[i] = smc(PSCI_CPU_ON, GOLD_MPIDR(i), (uintptr_t)secondary_entry, i);
	delay_ms(200);
	for (int i = GOLD0_IDX + 1; i < NR_CPUS; i++) {
		char label[] = "CPU0";
		label[3] = '0' + i;
		con_hex2(label, "ret=", (uint32_t)ret[i], "mpidr=",
			 sec_mpidr[i] ? (uint32_t)sec_mpidr[i] : 0xdead0000);
		con_hex(" mmfr0", sec_mmfr0[i]);
	}
}

static void test_mmu(void)
{
	struct mmu_info mi;

	stage = "mmu";
	int rc = mmu_enable(&mi);
	con_hex2("MMU", "rc=", rc, "l3=", mi.n_l3);
	con_hex2("IMAGE", "lo=", mi.img_lo, "hi=", mi.img_hi);
	con_hex("SCTLR", mi.sctlr_after);
	con_hex("TCR", rsr("tcr_el1"));
	con_hex("PAR img", mmu_probe((uintptr_t)test_mmu));
	con_hex("PAR fb", mmu_probe(FB_PHYS + 0x1000));
	con_hex("PAR log", mmu_probe(LOG_BASE));
}

/* Value of "key=N" (decimal) in a command line, else def. */
static unsigned arg_uint(const char *cmdline, const char *key, unsigned def)
{
	for (const char *s = cmdline; *s; s++) {
		if (s != cmdline && s[-1] != ' ')
			continue;
		const char *k = key, *v = s;
		while (*k && *v == *k)
			k++, v++;
		if (*k || *v != '=')
			continue;
		unsigned n = 0;
		for (v++; *v >= '0' && *v <= '9'; v++)
			n = n * 10 + (unsigned)(*v - '0');
		return n;
	}
	return def;
}

/* Gold core: boot xnu with the MMU and caches off, as from ABL. */
static void __attribute__((noreturn)) gold_boot_xnu(void)
{
	stage = "loader";
	unsigned wdt = arg_uint(xnu_cmdline, "lakewdt", 0);
	if (wdt) {
		if (wdt > 31)
			wdt = 31;	/* 20-bit bite counter at 32768 Hz */
		con_hex("WDT bite s", wdt);
		wdt_arm(wdt);
	}
	con_str("LOADER", loader_boot(boot_dtb));
	reset_to_fastboot(10000);
}

static void gold_main(void)
{
	gold_alive = 1;
	dsb();
	while (!cpu0_handed)
		;
	delay_ms(50);
	if (xnu_cmdline)
		gold_boot_xnu();
	stage = "gold";
	con_skip();
	con_hex2("GOLD", "midr=", rsr("midr_el1"), "mpidr=", rsr("mpidr_el1"));
	con_hex("MMFR0", rsr("id_aa64mmfr0_el1"));
	con_hex2("CPU0", "affinfo=", smc(PSCI_AFFINITY_INFO, 0, 0, 0), "offret=", cpu0_off_ret);

	test_smp();
	test_gic();
#ifdef SD_LATE
	con_hex("SD LATE wait s", SD_LATE);
	delay_ms(SD_LATE * 1000);
#endif
	test_sdhc();
	test_mmu();
	stage = "touch";
	con_clear(BLUE);
	test_touch(25);

	stage = "done";
	con_str("DONE", "milestone L2");
#ifdef WDT_TEST
	set_reason_bootloader();
	con_hex2("WDT", "en=", rd32(APSS_WDT_BASE + WDT_EN), "bite=", rd32(APSS_WDT_BASE + WDT_BITE_TIME));
	wdt_arm(10);
	con_hex2("WDT", "armed en=", rd32(APSS_WDT_BASE + WDT_EN), "bite=", rd32(APSS_WDT_BASE + WDT_BITE_TIME));
	for (;;)
		__asm__ volatile("wfe");
#endif
	reset_to_fastboot(5000);
}

void secondary_main(uint64_t idx)
{
	if (idx == GOLD0_IDX)
		gold_main();
	sec_mmfr0[idx] = rsr("id_aa64mmfr0_el1");
	sec_mpidr[idx] = rsr("mpidr_el1") | (1ULL << 63);
	dsb();
	smc(PSCI_CPU_OFF, 0, 0, 0);
}

/* ---- CPU0 (silver, from ABL) ---- */

void shim_main(uint64_t dtb)
{
	uint64_t v = 0;
	uint8_t type = 0, sub = 0, spare = 0;

	log_init();
	stage = "cpu0";
	boot_dtb = dtb;
	xnu_cmdline = loader_cmdline(dtb);
	con_clear(RED);
	con_str("SHIM", xnu_cmdline ? "lake L3: xnu on gold" : "lake milestone L2");
	con_hex2("BOOT", "n=", log_boots(), "el=", rsr("CurrentEL") >> 2);
	con_hex2("CPU0", "midr=", rsr("midr_el1"), "mpidr=", rsr("mpidr_el1"));
	con_hex("MMFR0", rsr("id_aa64mmfr0_el1"));
	con_hex2("DTB", "at=", dtb, "magic=", dtb ? __builtin_bswap32(rd32(dtb)) : 0);
	if (fdt_chosen_u64(dtb, "linux,initrd-start", &v))
		con_hex("INITRD", v);

	stage = "spmi";
	int r1 = spmi_read(0, PON_PERPH_TYPE, &type);
	int r2 = spmi_read(0, PON_PERPH_SUBTYPE, &sub);
	int r3 = spmi_read(0, PON_SOFT_RB_SPARE, &spare);
	con_hex2("PON", "type=", type << 8 | sub, "rc=", (r1 & 0xff) << 16 | (r2 & 0xff) << 8 | (r3 & 0xff));
	int w = set_reason_bootloader();
	uint8_t after = 0;
	int r4 = spmi_read(0, PON_SOFT_RB_SPARE, &after);
	con_hex2("SPARE", "was=", spare, "now=", after | (uint32_t)(w & 0xff) << 16 | (uint32_t)(r4 & 0xff) << 8);

	stage = "clocks";
	con_hex2("SDCC2", "ahb=", rd32(GCC_BASE + GCC_SDCC2_AHB_CBCR), "apps=",
		 rd32(GCC_BASE + GCC_SDCC2_APPS_CBCR));
	con_hex2("CLK", "qup3=", rd32(GCC_BASE + GCC_QUP3_I2C_CBCR), "sdcc1=",
		 rd32(GCC_BASE + GCC_SDCC1_APPS_CBCR));

	stage = "psci";
	con_hex2("PSCI", "ver=", smc(PSCI_VERSION, 0, 0, 0), "migr=", smc(PSCI_MIGRATE_INFO_TYPE, 0, 0, 0));
	con_hex2("FEAT", "off=", smc(PSCI_FEATURES, PSCI_CPU_OFF, 0, 0), "rst2=",
		 smc(PSCI_FEATURES, PSCI_SYSTEM_RESET2, 0, 0));

	stage = "cpu_on gold";
	uint64_t ret = smc(PSCI_CPU_ON, GOLD_MPIDR(GOLD0_IDX), (uintptr_t)secondary_entry, GOLD0_IDX);
	con_hex2("CPU_ON", "mpidr=", GOLD_MPIDR(GOLD0_IDX), "ret=", ret);
	for (int i = 0; i < 1000 && !gold_alive; i++)
		delay_ms(1);
	if (!gold_alive) {
		con_str("GOLD", "did not start");
		reset_to_fastboot(5000);
	}
	con_str("CPU0", "gold is up, CPU_OFF");
	cpu0_handed = 1;
	dsb();
	cpu0_off_ret = smc(PSCI_CPU_OFF, 0, 0, 0);	/* returns only on failure */
	dsb();
	for (;;)
		__asm__ volatile("wfe");
}
