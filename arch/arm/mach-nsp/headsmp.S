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
/* Based on arch/arm/mach-realview/headsmp.S */
/*
 *  linux/arch/arm/mach-realview/headsmp.S
 *
 *  Copyright (c) 2003 ARM Limited
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/linkage.h>
#include "memory.h"
#include <asm/assembler.h>

/*
 * iProc specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until we're
 * ready for them to initialise.
 */
#ifdef CONFIG_SMP
ENTRY(iproc_secondary_startup)
ARM_BE8(setend be)                      @ ensure we are in BE mode
	bl	v7_invalidate_l1
	bl	v7_flush_dcache_all
	mrc     p15, 0, r0, c0, c0, 5
	and     r0, r0, #15
	adr     r4, 1f
	ldmia   r4, {r5, r6}
	sub     r4, r4, r5
	add     r6, r6, r4
pen:	ldr     r7, [r6]
	cmp     r7, r0
	bne     pen

	/*
	 * we've been released from the holding pen: secondary_stack
	 * should now contain the SVC stack for this core
	 */
	b    secondary_startup

1:	.long   .
	.long   pen_release

ENDPROC(iproc_secondary_startup)

ENTRY(__iproc_vector_fixup)
	mov     r12, lr                 @ Save the return address
	/*
	 * fixup the vector table so that the secondary CPU does
	 * not start executing kernel instructions until we've
	 * patched its jump address during wakeup_secondary()
	 */
	ldr     r3,=0xffff002c
	ldr     r4,=SOC_ROM_BASE_PA
	str     r3, [r4, #SOC_ROM_LUT_OFF_NS]
	mov     pc, r12                 @ Return for regular boot
	nop
ENDPROC(__iproc_vector_fixup)
#else

ENTRY(__iproc_vector_fixup)
ENDPROC(__iproc_vector_fixup)
#endif
