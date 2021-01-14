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

#include <stdarg.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/ioport.h>

#include <asm/cacheflush.h>
#include <linux/of_address.h>

#include <asm/mach/bcm_iproc_smc.h>

struct secure_bridge_data {
	void __iomem *bounce;		/* virtual address */
	u32 buffer_addr;				/* physical address */
	int initialized;
} bridge_data;

struct bcm_iproc_smc_data {
	unsigned service_id;
	unsigned arg0;
	unsigned arg1;
	unsigned arg2;
	unsigned arg3;
	int ret_val;
};

static const struct of_device_id bcm_iproc_smc_ids[] __initconst = {
	{.compatible = "brcm,iproc-smc"},
	{},
};

int __init bcm_iproc_smc_init(void)
{
	struct device_node *node;
	__be32 of_address;

	/* Read buffer addr and size from the device tree node */
	node = of_find_matching_node(NULL, bcm_iproc_smc_ids);
	if (!node)
		return -ENODEV;

	/* Don't care about size or flags of the DT node */
	of_address = *of_get_address(node, 0, NULL, NULL);
	BUG_ON(!of_address);

	bridge_data.buffer_addr = be32_to_cpu(of_address);
	BUG_ON(!bridge_data.buffer_addr);

	/* obtain ptr to mapped memory for the node */
	bridge_data.bounce = of_iomap(node, 0);
	BUG_ON(!bridge_data.bounce);

	bridge_data.initialized = 1;

	return 0;
}

static u32 bcm_iproc_smc_asm(unsigned service_id)
{
	/* Set Up Registers to pass data to Secure Monitor */
	register u32 r4 asm("r4") = service_id;
	register u32 r5 asm("r5") = 0x3;	/* Keep IRQ and FIQ off in SM */
	register u32 r6 asm("r6") = bridge_data.buffer_addr;
	register u32 r12 asm("r12");
	register u32 r0 asm("r0");

	do {
		/* Secure Monitor Call */
		asm volatile (__asmeq("%0", "ip")
	      __asmeq("%1", "r4")
	      __asmeq("%2", "r5") __asmeq("%3", "r6")
	      ".arch_extension sec\n"
	      "smc	#0		@ switch to secure world\n" :
	      "+r"(r12), "+r"(r4), "+r"(r5), "+r"(r6), "+r"(r0)
	       : : "r1", "r2", "r3", "r7", "r8", "r14");

		/* Setup registers in case we need to re-enter secure mode */
		r4 = SSAPI_RET_FROM_INT_SERV;
		r5 = 0x3;
		r6 = bridge_data.buffer_addr;

	} while (r12 != SEC_EXIT_NORMAL);

	return r0;
}

/* __bcm_iproc_smc() should only run on CPU 0, with pre-emption disabled */
static void __bcm_iproc_smc(void *info)
{
	struct bcm_iproc_smc_data *data = info;
	u32 *args = bridge_data.bounce;
	int rc = 0;

	/* Must run on CPU 0 */
	BUG_ON(smp_processor_id() != 0);

	/* Check map in the bounce area */
	BUG_ON(!bridge_data.initialized);

	/* Copy one 32 bit word into the bounce area */
	args[0] = data->arg0;
	args[1] = data->arg1;
	args[2] = data->arg2;
	args[3] = data->arg3;

	/* Prior to switching to secure mode, it is essential
	 * to flush the cache and update all data into DDR
	 * memory
	 */
	flush_cache_all();

	/* Trap into Secure Monitor */
	pr_debug("%s:cmd:%x buffer:%x\n",
				__func__,
			  data->service_id,
			  bridge_data.buffer_addr);

	rc = bcm_iproc_smc_asm(data->service_id);
	if (rc != SEC_ROM_RET_OK)
		pr_err("Secure Monitor call failed (0x%x)!\n", rc);

	/* forward the smc return value to calling function */
	data->ret_val = rc;
}

unsigned bcm_iproc_smc(unsigned service_id, unsigned arg0, unsigned arg1,
		  unsigned arg2, unsigned arg3)
{
	struct bcm_iproc_smc_data data;

	data.service_id = service_id;
	data.arg0 = arg0;
	data.arg1 = arg1;
	data.arg2 = arg2;
	data.arg3 = arg3;


	/*
	 * In the case of SMP environment, due to a limitation of the
	 * secure monitor design, we must use the SMP
	 * infrastructure to forward all secure monitor calls to Core 0.
	 */
	smp_call_function_single(0, __bcm_iproc_smc, (void *)&data, 1);

	return data.ret_val;
}
