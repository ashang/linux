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
#ifndef    __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <linux/io.h>
#include <mach/iproc_regs.h>

#if defined(CONFIG_DEBUG_UNCOMPRESS)
#  if defined(CONFIG_DEBUG_UART_PHYS)
#    define IPROC_UART_PA           CONFIG_DEBUG_UART_PHYS
#  else
#    warning CONFIG_DEBUG_UNCOMPRESS enabled, but CONFIG_DEBUG_UART_PHYS is undefined.
#    warning For iProc good values for CONFIG_DEBUG_UART_PHYS are:
#    warning UART0 -- 0x18000300
#    warning UART1 -- 0x18000400
#    error CONFIG_DEBUG_UNCOMPRESS enabled, but CONFIG_DEBUG_UART_PHYS is undefined.
#  endif

#define UART_LSR_OFFSET		0x05
#define UART_RBR_THR_DLL_OFFSET 0x00
#define UART_LSR_THRE_MASK	0x20
#define UART_LSR_TEMT_MASK	0x40

static inline void putc(int c)
{
    /*
     * data should be written to THR register only if THRE (LSR bit5)
     * is set)
     */
    while (0 == (__raw_readb((void *)(IPROC_UART_PA + UART_LSR_OFFSET)) &
                                                        UART_LSR_THRE_MASK))
    {
    }

    __raw_writeb((u8)c, (void *)(IPROC_UART_PA +
				 UART_RBR_THR_DLL_OFFSET));
}

static inline void flush(void)
{
    /* Wait for the tx fifo to be empty and last char to be sent */
    while (0 == (__raw_readb((void *)(IPROC_UART_PA + UART_LSR_OFFSET)) &
                                                        UART_LSR_TEMT_MASK ))
    {
    }
}

#else

#define putc(c)
#define flush()

#endif

#define arch_decomp_setup()
#define arch_decomp_wdog()

#endif /* __ASM_ARCH_UNCOMPRESS_H */
