/*
 * Copyright (C) 2014, Broadcom Corporation. All Rights Reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/clocksource.h>
#include <linux/clk-provider.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/proc-fns.h>
#include <asm/smp_scu.h>
#include <asm/pgtable.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/bcm_iproc_smc.h>

#include "smp.h"

#define ACPAL_ACPAL_SYNC_REG    0x18018000
#define ACPAL_CNTL_REG_OFFSET	0x004
void __iomem *acpal_sync_reg;
EXPORT_SYMBOL_GPL(acpal_sync_reg);
bool _bcm_system_needs_acpal_sync;
EXPORT_SYMBOL_GPL(_bcm_system_needs_acpal_sync);
/*
 * L2 configuration:
 * Enable L2 cache instruction prefetch, data prefetch, early BRESP
 * and shared attribute override
 */
#define L2_AUX_VAL	((1 << L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT) |	\
			 (1 << L2X0_AUX_CTRL_INSTR_PREFETCH_SHIFT) |	\
			 (1 << L2X0_AUX_CTRL_EARLY_BRESP_SHIFT)	|	\
			 (1 << L2X0_AUX_CTRL_SHARE_OVERRIDE_SHIFT))

static void __init iproc_map_io(void)
{
	struct map_desc desc;

	desc.virtual = VMALLOC_END - SZ_2M;
	desc.pfn = __phys_to_pfn(scu_a9_get_base());
	desc.length = SZ_1M;
	desc.type = MT_DEVICE;

	iotable_init(&desc, 1);
}

static void __init iproc_nsp_map_io(void)
{
	iproc_map_io();
	debug_ll_io_init();
}

static int iproc_data_abort_handler(unsigned long addr, unsigned int fsr,
				    struct pt_regs *regs)
{
	/* This happens because u-boot already inited the L2 cache, and when
	 * Linux init's the L2 cache it causes this.  There really only should
	 * be one of these, but we'll mask all of them off until we can get
	 * rid of that issue with u-boot.
	 */
	return 0;
}

static void iproc_enable_data_prefetch_aborts(void)
{
	u32 x;

	/* Install our hook */
	hook_fault_code(16 + 6, iproc_data_abort_handler, SIGBUS, 0,
			"imprecise external data abort");

	/* Enable external aborts - clear "A" bit in CPSR */

	/* Read CPSR */
	asm("mrs %0, cpsr" : "=&r"(x) : : );

	x &= ~PSR_A_BIT;

	/* Update CPSR, affect bits 8-15 */
	asm("msr cpsr_x, %0; nop; nop" : : "r"(x) : "cc");
}

static void __init bcm_nsp_timer_init(void)
{
	iproc_enable_data_prefetch_aborts();

	/* Initialize all clocks declared in device tree */
	of_clk_init(NULL);

	//clocksource_of_init();
	timer_probe();
}

static const struct of_device_id l2x0_ids[] __initconst = {
	{ .compatible = "arm,pl310-cache"},
	{}
};

static int __init bcm_smc_l2_cache_enable(u32 aux_val, u32 aux_mask)
{
	bcm_iproc_smc(SSAPI_ENABLE_L2_CACHE, aux_val, aux_mask, 0, 0);
	return 0;
}

#ifdef CONFIG_PL330_DMA
#include <linux/delay.h>

#define DMAC_RESET_CTRL         0x18114800
#define DMAC_RESET_CTRL_MASK    0xFFFFFFFE
#define DMAC_RESET_CTRL_SHIFT   0
#define DMAC_RESET_TIMEOUT      100

static void __init pl330_init(void)
{
	u32 val, timeout = 0;
	void __iomem *dmac_reset_ctrl = ioremap_nocache(DMAC_RESET_CTRL, SZ_4);

	BUG_ON(dmac_reset_ctrl == NULL);

	/* bring the DMAC block out of reset */
	val = readl(dmac_reset_ctrl);
	val &= DMAC_RESET_CTRL_MASK;
	val |= (1 << DMAC_RESET_CTRL_SHIFT);
	writel(val, dmac_reset_ctrl);

	val &= DMAC_RESET_CTRL_MASK;
	writel(val, dmac_reset_ctrl);

	while (readl(dmac_reset_ctrl) &
			~DMAC_RESET_CTRL_MASK) {
		udelay(10);
		if (timeout++ > DMAC_RESET_TIMEOUT)
			BUG_ON(1);
	}

	iounmap(dmac_reset_ctrl);
}
#endif

static void __init bcm_nsp_init(void)
{
	struct device_node *l2_np;
	int iproc_smc_init = 0;
	u32 reg_data;

#ifdef CONFIG_PL330_DMA
	/*
	 * Note DMAC init needs to be done before platform bus population,
	 * where the AMBA bus probe will read DMAC registers for AMBA device
	 * ID. If DMAC is not brought out of reset before that, CPU will hang
	 * on the read request
	 */
	pl330_init();
#endif
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

	/* Obtain SMC init status.
	 * For secure boot, SMC will not be enabled.
	 * For non-secure boot, SMC will be enabled.
	 * init will fail for incorrect node entry.
	 */
	iproc_smc_init = bcm_iproc_smc_init();
	pr_info("bcm_iproc_smc_init=0x%x\n", iproc_smc_init);

	l2_np = of_find_matching_node(NULL, l2x0_ids);

	if (l2_np) {
		const char *l2_status;

		/*
		 * Need to check to make sure L2 is not explicitly disabled in
		 * device tree
		 *
		 * L2 should only be enabled when its device node is not
		 * "disabled" in device tree
		 */
		l2_status = of_get_property(l2_np, "status", NULL);

		if (l2_status == NULL || strcmp(l2_status, "disabled") != 0) {
			/*
			 * For non-secure boot, SMC will be required
			 * to access pl310.For secure boot,l2x0_of_init func
			 * can access pl310 directly.
			 */
			if (!iproc_smc_init)
				bcm_smc_l2_cache_enable(L2_AUX_VAL, ~0UL);

//#if 0
			l2x0_of_init(L2_AUX_VAL, ~0UL);
//#endif
		}
	}


	_bcm_system_needs_acpal_sync = false;
	acpal_sync_reg = ioremap(ACPAL_ACPAL_SYNC_REG, SZ_256);
	reg_data = readl_relaxed(acpal_sync_reg + ACPAL_CNTL_REG_OFFSET);
	if (reg_data & 1) {
		/*subject to cache line corruption under certain conditions
		if ACPAL is disabled*/
		pr_info("COHR: NSP system with ACPAL disabled\n");
	} else {
		if (0 == (reg_data & 2)) {
			/*system with ACPAL enabled.early response enabled*/
			pr_err("COHR: Enabling ACPAL_SYNC\n");
			_bcm_system_needs_acpal_sync = true;
		}
		if (6 != (reg_data & 6)) {
			/*system with ACPAL enabled. early response and break
			burst transaction can be unstable.*/
			pr_alert("COHR: Recommend disabling early response\n");
			pr_alert(" Recommend disabling burst transaction\n");
		}
	}
}

static void __init bcm_nsp_init_early(void)
{
	l2x0_of_init(0x0A150000, ~(0x000F0000));
}

static const char const *bcm_nsp_dt_compat[] = {
	"brcm,nsp",
	NULL,
};

DT_MACHINE_START(NSP_DT, "NSP SoC")
	.smp = smp_ops(iproc_smp_ops),
	.init_early = bcm_nsp_init_early,
	.init_machine = bcm_nsp_init,
	.map_io = iproc_nsp_map_io,
	.init_time = bcm_nsp_timer_init,
	.dt_compat = bcm_nsp_dt_compat,
MACHINE_END
