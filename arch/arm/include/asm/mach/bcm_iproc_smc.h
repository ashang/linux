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

#ifndef BCM_IPROC_SMC_H
#define BCM_IPROC_SMC_H

#include <linux/types.h>

#define FLAGS	(SEC_ROM_ICACHE_ENABLE_MASK | SEC_ROM_DCACHE_ENABLE_MASK | \
			SEC_ROM_IRQ_ENABLE_MASK | SEC_ROM_FIQ_ENABLE_MASK)

/*!
 * Definitions for IRQ & FIQ Mask for ARM
 */

#define FIQ_IRQ_MASK					0xC0
#define FIQ_MASK						0x40
#define IRQ_MASK						0x80

/*!
 * Secure Mode FLAGs
 */

/* When set, enables ICache within the secure mode */
#define SEC_ROM_ICACHE_ENABLE_MASK                        0x00000001

/* When set, enables DCache within the secure mode */
#define SEC_ROM_DCACHE_ENABLE_MASK                        0x00000002

/* When set, enables IRQ within the secure mode */
#define SEC_ROM_IRQ_ENABLE_MASK                           0x00000004

/* When set, enables FIQ within the secure mode */
#define SEC_ROM_FIQ_ENABLE_MASK                           0x00000008

/* When set, enables Unified L2 cache within the secure mode */
#define SEC_ROM_UL2_CACHE_ENABLE_MASK                     0x00000010

/* Broadcom Secure Service API Service IDs */
#define SSAPI_PUBLIC_OTP_SERV                             0x01000001
#define SSAPI_ENABLE_L2_CACHE                             0x01000002
#define SSAPI_DISABLE_L2_CACHE                            0x01000003

#define SSAPI_SEND_SYS_CONFIG                             0x01001000
#define SSAPI_SEC_STOR_COMMAND                            0x01002000
#define SSAPI_AUTHENTICATE_COMMAND                        0x01003000
#define SSAPI_CASTAUDIO_COMMAND                           0x02001000

#define SSAPI_SOTP_SER_COMMAND                            0x04001000

#define SSAPI_CRYPTO_SER_COMMAND                          0x05001000

#define SSAPI_SLEEP_DORMANT                               0x01004000
#define SSAPI_SLEEP_DEEP                                  0x01004001

/* Broadcom Secure Service API Return Codes */
#define SEC_ROM_RET_OK                                    0x00000001
#define SEC_ROM_RET_FAIL                                  0x0E000001

#define SSAPI_RET_FROM_INT_SERV                           0x4
#define SEC_EXIT_NORMAL                                   0x1

extern int bcm_iproc_smc_init(void);

extern unsigned bcm_iproc_smc(unsigned service_id,
			     unsigned arg0,
			     unsigned arg1,
			     unsigned arg2,
			     unsigned arg3);


#endif /* BCM_IPROC_SMC_H */
