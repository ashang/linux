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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/completion.h>
#include <asm/cacheflush.h>
#include <asm/smp_plat.h>

static DECLARE_COMPLETION(cpu_killed);

static inline void cpu_enter_lowpower(void)
{
	unsigned int v;

	flush_cache_all();
	asm volatile("mcr p15, 0, %1, c7, c5, 0\n"
		     "mcr p15, 0, %1, c7, c10, 4\n"
		     /* Turn off coherency */
		     "mrc p15, 0, %0, c1, c0, 1\n"
		     "bic %0, %0, #0x20\n"
		     "mcr p15, 0, %0, c1, c0, 1\n"
		     "mrc p15, 0, %0, c1, c0, 0\n"
		     "bic %0, %0, #0x04\n"
		     "mcr p15, 0, %0, c1, c0, 0\n"
		     : "=&r" (v) : "r" (0) : "cc");
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;

	asm volatile("mrc p15, 0, %0, c1, c0, 0\n"
		     "orr %0, %0, #0x04\n"
		     "mcr p15, 0, %0, c1, c0, 0\n"
		     "mrc p15, 0, %0, c1, c0, 1\n"
		     "orr %0, %0, #0x20\n"
		     "mcr p15, 0, %0, c1, c0, 1\n"
		     : "=&r" (v) : : "cc");
}

static inline void iproc_do_lowpower(unsigned int cpu, int *spurious)
{
	/*
	 * there is no power-control hardware on this platform, so all
	 * we can do is put the core into WFI; this is safe as the calling
	 * code will have already disabled interrupts
	 */
	for (;;) {
		/*
		 * here's the WFI
		 */
		asm(".inst 0xe320f003\n" : : : "memory", "cc");

		if (pen_release == cpu_logical_map(cpu)) {
			/*
			 * OK, proper wakeup, we're done
			 */
			break;
		}

		/*
		 * getting here, means that we have come out of WFI without
		 * having been woken up - this shouldn't happen
		 *
		 * The trouble is, letting people know about this is not really
		 * possible, since we are currently running incoherently, and
		 * therefore cannot safely call printk() or anything else
		 */
		(*spurious)++;
		pr_debug("CPU%u: spurious wakeup call\n", cpu);
	}
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void iproc_cpu_die(unsigned int cpu)
{
	int spurious = 0;

	pr_notice("CPU%u: shutdown\n", cpu);
	complete(&cpu_killed);

	/*
	 * we're ready for shutdown now, so do it
	 */
	cpu_enter_lowpower();
	iproc_do_lowpower(cpu, &spurious);

	/*
	 * bring this CPU back into the world of cache
	 * coherency, and then restore interrupts
	 */
	cpu_leave_lowpower();

	if (spurious)
		pr_warn("CPU%u: %u spurious wakeup calls\n", cpu, spurious);
}
