/*
 * Copyright (C) 2013, Broadcom Corporation. All Rights Reserved.
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

#include <asm/smp_scu.h>
#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <linux/spinlock.h>

#include "hotplug.h"
#include "memory.h"
#include "smp.h"

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */

static DEFINE_SPINLOCK(boot_lock);
static struct of_device_id lut_offset_table[] = {
        {.compatible = "brcm,smp_lut_offset"},
        { /* end of list */ },
};

/*
 * Use SCU config register to count number of cores
 */
static inline unsigned int get_core_count(void)
{
	void __iomem *scu_base;
	unsigned int count;

	if (!scu_a9_has_base())
		return 1;

	scu_base = ioremap_nocache(scu_a9_get_base(), SZ_4);
	if (!scu_base) {
		pr_err("Error remapping SCU\n");
		return 1;
	}

	count = scu_get_core_count(scu_base);
	iounmap(scu_base);

	return count;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init iproc_smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	pr_debug("%s: Enter ncores %d\n", __func__, ncores);

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	pr_debug("%s: Leave ncores %d\n", __func__, ncores);
}

void iproc_secondary_init(unsigned int cpu)
{
	pr_debug("%s: cpu %d awake\n", __func__, cpu);

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	pen_release = -1;

	smp_wmb();

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);

	pr_debug("%s: cpu %d ready\n", __func__, cpu);
}

int iproc_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	pr_debug("%s: Enter CPU%d\n", __func__, cpu);

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 * Note that "pen_release" is the hardware CPU ID, whereas
	 * "cpu" is Linux's internal ID.
	 */
	pen_release = cpu_logical_map(cpu);

	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	/*
	 * Now the secondary CPU must start marching on its
	 * own.
	 */
	dsb_sev();

	/* wait at most 1 second for the secondary to wake up */

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
		clean_dcache_area((void *)&pen_release, sizeof(pen_release));
	}

	/*
	 * Now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	pr_debug("%s: Leave pen-release %d\n", __func__, pen_release);

	return pen_release != -1 ? -ENOSYS : 0;
}

static unsigned int __init iproc_determine_lut_offset(void)
{
	unsigned int offset;
	struct device_node *np;

	np = of_find_matching_node(NULL, lut_offset_table);
	if (np)
		if (!of_property_read_u32(np, "lut-offset", &offset))
			return offset;

	if (of_machine_is_compatible("brcm,nsp"))
		offset = SOC_ROM_LUT_OFF_NSP;
	else
		offset = SOC_ROM_LUT_OFF_NS;

	return offset;
}

static void __init
iproc_wakeup_secondary(unsigned cpu, void (*_sec_entry_va) (void))
{
	void __iomem *rombase;
	phys_addr_t lut_pa;
	u32 offset;
	u32 val;

	pr_debug("%s: Enter cpu %d\n", __func__, cpu);

#ifdef CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED
	lut_pa = 0x000FF000 & PAGE_MASK;
	offset = 0x000FF000 & ~PAGE_MASK;
#else  /* def CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED */
	lut_pa = SOC_ROM_BASE_PA & PAGE_MASK;
	offset = SOC_ROM_BASE_PA & ~PAGE_MASK;
#endif /* def CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED */
	offset += iproc_determine_lut_offset();

	rombase = ioremap(lut_pa, PAGE_SIZE);
	if (rombase == NULL)
		return;
	val = virt_to_phys(_sec_entry_va);

	writel(val, rombase + offset);

	smp_wmb(); /* probably not needed - io regs are not cached */

	dsb_sev(); /* Exit WFI */
	mb();

	iounmap(rombase);

	pr_debug("%s: Leave cpu %d\n", __func__, cpu);
}

void __init iproc_smp_prepare_cpus(unsigned int max_cpus)
{
	void __iomem *scu_base;
	int i;

	pr_debug("%s: init %d cpus\n", __func__, max_cpus);

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	scu_base = ioremap_nocache(scu_a9_get_base(), SZ_4);
	if (!scu_base) {
		pr_err("Error remapping SCU\n");
		return;
	}

	/* scu_base is defined in get_core_count */
	scu_enable(scu_base);
	iounmap(scu_base);
	iproc_wakeup_secondary(max_cpus, iproc_secondary_startup);
}

struct smp_operations iproc_smp_ops __initdata = {
	.smp_init_cpus = iproc_smp_init_cpus,
	.smp_prepare_cpus = iproc_smp_prepare_cpus,
	.smp_secondary_init = iproc_secondary_init,
	.smp_boot_secondary = iproc_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die = iproc_cpu_die,
#endif
};
