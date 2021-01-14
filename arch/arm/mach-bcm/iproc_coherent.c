/*
 * Copyright 2014 Broadcom Corporation.  All rights reserved.
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <asm/smp_plat.h>
#include <asm/cacheflush.h>

#define REV_ID_MASK		0x03
#define REV_ID_SHIFT		15
#define REV_ID_A0		0x00

#define ACPAL_SYNC_REG			0x000
#define ACPAL_CONTROL_REG		0x004
#define ACPAL_ERR_RESP_INTR_STATUS	0x008
#define ACPAL_ERR_RESP_INTR_MASK	0x00c
#define ACPAL_ERR_RESP_ADDRESS		0x010
#define ACPAL_ERR_RESP_AWID		0x014
#define ACPAL_ERR_RESP_OTHER_ATTR	0x018


#define IHOST_SCU_CONTROL		0x000
#define IHOST_SCU_FILTER_START		0x040
#define IHOST_SCU_FILTER_END		0x044
#define IHOST_L2C_AUX_CONTROL		0x104
#define IHOST_L2C_FILT_START		0xc00
#define IHOST_L2C_FILT_END		0xc04


static struct of_device_id of_coherency_table[] = {
	{.compatible = "brcm,iproc-coherency"},
	{ /* end of list */ },
};

struct coherent_field_info {
	u32 shift;
	u32 mask;
	u32 val;
};

static void __iomem *acpal_base;
bool _bcm_coherent_system_allow_cache_work = true;

static int brcm_iproc_platform_notifier(struct notifier_block *nb,
					unsigned long event, void *__dev)
{
	struct device *dev = __dev;
	struct device_node *np = dev->of_node;

	if (event != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	if (of_property_read_bool(dev->of_node, "dma-coherent")) {
		set_dma_ops(dev, &arm_coherent_dma_ops);
		pr_debug("Coherent dma set for %s %s %s\n", np->name,
						np->full_name, np->type);
	}

	return NOTIFY_OK;
}

static struct notifier_block brcm_iproc_platform_nb = {
	.notifier_call = brcm_iproc_platform_notifier,
};

static irqreturn_t acpal_fault_handler(int val, void *ptr)
{
	u32 errStat;
	static int acpal_init = 1;
	static u32 errCount;

	/* emit error header and count */
	if (acpal_init)
		pr_info("ACPAL: preliminary scan on init\n");
	else {
		errCount++;
		pr_err("ACPAL: fault detected %u (%u)\n", errCount, val);
	}

	errStat = readl_relaxed(acpal_base + ACPAL_ERR_RESP_INTR_STATUS);
	if (errStat) {
		pr_crit("ACPAL: Intr Status   = %08X\n", errStat);
		errStat = readl_relaxed(acpal_base + ACPAL_CONTROL_REG);
		pr_crit("ACPAL: configuration = %08X\n", errStat);
		errStat = readl_relaxed(acpal_base + ACPAL_ERR_RESP_INTR_MASK);
		pr_crit("ACPAL: Intr nask     = %08X\n", errStat);
		errStat = readl_relaxed(acpal_base + ACPAL_ERR_RESP_ADDRESS);
		pr_crit("ACPAL: Failt address = %08X\n", errStat);
		errStat = readl_relaxed(acpal_base + ACPAL_ERR_RESP_AWID);
		pr_crit("ACPAL: AWID, TYPE    = %08X\n", errStat);
		pr_crit("ACPAL:     AWID = %04X, type = %01X\n",
			errStat & 0x3FFF, (errStat >> 16) & 0x3);
		errStat = readl_relaxed(acpal_base + ACPAL_ERR_RESP_OTHER_ATTR);
		pr_crit("ACPAL: Other attribs = %08X\n", errStat);
		pr_crit("ACPAL:     SIZE = %01X;   LOCK = %01X; PROT = %01X\n",
			(errStat >> 20) & 0x7, (errStat >> 18) & 0x3,
			(errStat >> 15) & 0x7);
		pr_crit("ACPAL:     LEN = %01X;  CACHE = %01X; BURST = %01X\n",
			(errStat >> 11) & 0xF, (errStat >> 7) & 0xF,
			(errStat >> 5) & 0x3);
		pr_crit("ACPAL:     USER = %02X\n", errStat & 0x1F);
		errStat =
			readl_relaxed(acpal_base + ACPAL_ERR_RESP_INTR_STATUS);
		writel_relaxed(errStat,
				(acpal_base + ACPAL_ERR_RESP_INTR_STATUS));
		errStat =
			readl_relaxed(acpal_base + ACPAL_ERR_RESP_INTR_STATUS);
		if (errStat)
			pr_crit
			("ACPAL: unable to clear interrupt status: %08X\n",
				errStat);
	}

	if (acpal_init) {
		pr_info("ACPAL: preliminary scan complete\n");
		acpal_init = 0;
	} else
		pr_crit("ACPAL: fault scan %u complete\n", errCount);
	return IRQ_HANDLED;
}

int validate_scu_l2c_filters(struct device_node *np)
{
	void __iomem *scu_base;
	void __iomem *l2c_base;
	u32 reg_data;
	u32 reg_extra;
	u32 ram_start;
	u32 ram_end;

	scu_base = of_iomap(np, 3);
	if (!scu_base)
		return -ENODEV;

	l2c_base = of_iomap(np, 4);
	if (!l2c_base)
		return -ENODEV;

	ram_start = 0x60000000;
	ram_end = 0xdff00000;
	reg_data = readl_relaxed(scu_base + IHOST_SCU_CONTROL);
	if (3 != (reg_data & 3)) {
		pr_err("COHR: SCU must be enabled with addr filter enabled\n");
		return -EFAULT;
	}
	reg_data = readl_relaxed(scu_base + IHOST_SCU_FILTER_START);
	if (ram_start != (reg_data & 0xFFF00000)) {
		pr_err("COHR: SCU filter start must be at base of DRAM\n");
		return -EFAULT;
	}
	reg_extra = readl_relaxed(scu_base + IHOST_SCU_FILTER_END);
	if (ram_end < (reg_extra & 0xFFF00000)) {
		pr_err("COHR: SCU filter end must be at end of DRAM\n");
		return -EFAULT;
	}
	if (ram_start > (reg_extra & 0xFFF00000)) {
		pr_err("COHR: SCU filter end must be >= SCU filter start\n");
		return -EFAULT;
	}
	if (ram_end > (reg_extra & 0xFFF00000))
		pr_warn("COHR: SCU filter end does not cover DRAM range\n");
	reg_data = readl_relaxed(l2c_base + IHOST_L2C_FILT_START);
	if (0 == (reg_data & 1)) {
		pr_err("COHR: L2C filter must be enabled for coherent mode\n");
		return -EFAULT;
	}
	if (ram_start != (reg_data & 0xFFF00000)) {
		pr_err("COHR: SCU filter start must be at base of DRAM\n");
		return -EFAULT;
	}
	reg_data = readl_relaxed(l2c_base + IHOST_L2C_FILT_END);
	if ((reg_data & 0xFFF00000) != (reg_extra & 0xFFF00000)) {
		pr_err("COHR: L2C and SCU filter ends must match\n");
		return -EFAULT;
	}
	if (ram_end < (reg_data & 0xFFF00000)) {
		pr_err("COHR: L2C filter end must be at end of DRAM\n");
		return -EFAULT;
	}
	if (ram_start > (reg_data & 0xFFF00000)) {
		pr_err("COHR: L2C filter end must be >= SCU filter start\n");
		return -EFAULT;
	}
	if (ram_end > (reg_data & 0xFFF00000))
		pr_warn("COHR: L2C filter end does not cover all of DRAM\n");
	reg_data = readl_relaxed(l2c_base + IHOST_L2C_AUX_CONTROL);
	if (0 == (reg_data & 0x00000800))
		pr_warn("COHR: Recommend running with L2C device store buf\n");
	return 0;
}

int __init coherency_init(void)
{
	struct device_node *np;
	struct device_node *child_np;
	void __iomem *soc_info_base;
	void __iomem *idm_ctrl_base;
	void __iomem *iospace;
	struct coherent_field_info info[6];
	u32 no_val;
	u32 cnt;
	u32 reg_data;
	u32 acpal_intr;
	int rv;

	np = of_find_matching_node(NULL, of_coherency_table);
	if (np) {
		soc_info_base = of_iomap(np, 1);
		if (!soc_info_base)
			return -ENODEV;

		acpal_base = of_iomap(np, 0);
		if (!acpal_base)
			return -ENODEV;

		iospace = of_iomap(np, 2);
		if (!iospace)
			return -ENODEV;

		acpal_intr = irq_of_parse_and_map(np, 0);
		if (((*(u32 *)soc_info_base >> REV_ID_SHIFT) & REV_ID_MASK)
					 == REV_ID_A0) {
			pr_debug("SOC revision is A0\n");
			return 0;
		}

		rv = validate_scu_l2c_filters(np);
		if (rv)
			return rv;

		for_each_child_of_node(np, child_np) {
			pr_debug("Node: %s\n", child_np->name);
			idm_ctrl_base = of_iomap(child_np, 0);
			if (of_property_read_u32(child_np, "val-length",
						 &no_val))
				continue;
			if (of_property_read_u32_array(child_np,
				       "coherency-val", (u32 *)&info[0],
						       (no_val * 3)))
				continue;

			reg_data = readl_relaxed(idm_ctrl_base);
#ifndef CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED
			pr_debug("Pre data %x\n", reg_data);
#else /* ndef CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED */
			pr_debug("Current  %08X\n", reg_data);
#endif /* ndef CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED */
			for (cnt = 0; cnt < no_val; cnt++) {
				reg_data &=
					~(info[cnt].mask << info[cnt].shift);
			}
			for (cnt = 0; cnt < no_val; cnt++)
				reg_data |= info[cnt].val << info[cnt].shift;
#ifndef CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED
			writel_relaxed(reg_data, idm_ctrl_base);
			pr_debug("Post data %x\n", reg_data);
#else /* ndef CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED */
			pr_debug("Proposed %08X\n", reg_data);
#endif /* ndef CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED */
			iounmap(idm_ctrl_base);
			idm_ctrl_base = NULL;
		}

		acpal_fault_handler(acpal_intr, NULL);
		reg_data = request_irq(acpal_intr,
					(irq_handler_t) acpal_fault_handler,
					IRQF_PERCPU,
					"ACPAL_ERR", NULL);
		if (0 == reg_data) {
			reg_data = readl_relaxed(acpal_base +
					ACPAL_ERR_RESP_INTR_MASK);
			reg_data |= 0x00000001;
			writel_relaxed(reg_data,
				(acpal_base + ACPAL_ERR_RESP_INTR_MASK));
			pr_info
			    ("ACPAL: fault handler is registered & enabled\n");
		} else
			pr_err("ACPAL: unable to register fault handler\n");

		if (iospace) {
#ifndef CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED
			reg_data = 0x1;
			writel_relaxed(reg_data, iospace);
			pr_info("AXIIC_REMAP %p = %08X\n", iospace, reg_data);
#else /* ndef CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED */
			pr_info("AXIIC_REMAP must be set by SSB to 1\n");
#endif /* ndef CONFIG_MACH_IPROC_NSP_B0_BOOT_SIGNED */
			iounmap(iospace);
			pr_info
				("NorthStar+ entered coherent mode\n");
			_bcm_coherent_system_allow_cache_work = false;
		} else
			pr_err
			("unable to get I/O space to set coherent mode\n");

		bus_register_notifier(&platform_bus_type,
					&brcm_iproc_platform_nb);
	}
	return 0;
}

postcore_initcall(coherency_init);
