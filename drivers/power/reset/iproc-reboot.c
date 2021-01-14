/*
 * Copyright (C) 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/system_misc.h>

#define IPROC_REBOOT_METHOD_CRU 0
#define IPROC_REBOOT_METHOD_PCU 1

#define PCU_AOPC_DEEP_SLEEP 0x80400021

struct iproc_reboot {
	struct platform_device *pdev;
	void *__iomem reg_addr;
	int method;
};

struct iproc_reboot *ir;

static void iproc_restart(enum reboot_mode reboot_mode, const char *cmd)
{
	u32 val;

	switch (ir->method) {
	case IPROC_REBOOT_METHOD_CRU:
		/* Set CRU iproc_reset_n to 0 */
		dev_info(&ir->pdev->dev, "CRU reboot method\n");
	val = readl(ir->reg_addr);
	val &= ~((u32) 1 << 1);
	writel(val, ir->reg_addr);
		break;
	case IPROC_REBOOT_METHOD_PCU:
		/* Set PCU_AOPC "deep sleep" to 60 seconds (NS & NS+) */
		dev_info(&ir->pdev->dev, "PCU_AOPC reboot method\n");
		writel(PCU_AOPC_DEEP_SLEEP, ir->reg_addr);
		break;
	default:
		dev_err(&ir->pdev->dev, "Unsupported reboot method\n");
		BUG();
	}

	while (time_is_before_jiffies(msecs_to_jiffies(1000)))
		cpu_relax();

	dev_emerg(&ir->pdev->dev, "Unable to restart system\n");
}

static int iproc_reboot_probe(struct platform_device *pdev)
{
	ir = devm_kzalloc(&pdev->dev, sizeof(*ir), GFP_KERNEL);
	if (!ir) {
		dev_err(&pdev->dev, "Out of memory for iProc Reboot\n");
		return -ENOMEM;
	}

	ir->pdev = pdev;

	if (of_property_read_u32(pdev->dev.of_node,
		"method", &(ir->method))) {
		/* if reboot method is not in the device tree,
		 * then set it to zero.  Zero is CRU which is
		 * the default reboot method.  NS & NS+ use PCU_AOPC.
		 */
		ir->method = 0;
	}

	dev_dbg(&pdev->dev, "Reboot method %d\n", ir->method);

	/* Each method will list its register addr in order:
	 * method 0:  CRU 0x1803f184/0x1800c184
	 * method 1:  PCU 0x1803f020/0x1800c020
	 */
	ir->reg_addr = of_iomap(pdev->dev.of_node, ir->method);
	if (!ir->reg_addr) {
		devm_kfree(&pdev->dev, ir);
		dev_err(&pdev->dev, "Error mapping iProc Reboot\n");
		return -ENODEV;
	}

	dev_dbg(&pdev->dev, "Reboot reg addr %p\n", ir->reg_addr);

	arm_pm_restart = iproc_restart;

	dev_info(&pdev->dev, "iProc Reboot registered\n");

	return 0;
}

static struct of_device_id iproc_reboot_of_match[] = {
	{ .compatible = "brcm,iproc-reboot" },
	{}
};

static struct platform_driver iproc_reboot_driver = {
	.probe = iproc_reboot_probe,
	.driver = {
		.name = "iproc-reboot",
		.of_match_table = iproc_reboot_of_match,
	},
};
module_platform_driver(iproc_reboot_driver);
