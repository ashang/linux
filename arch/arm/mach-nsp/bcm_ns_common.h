/*
 * Copyright (C) 2013 Broadcom Corporation
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

#ifndef BCM_NS_COMMON_H
#define BCM_NS_COMMON_H

#include <linux/init.h>

extern void __init bcm_ns_init_early(void);
extern void __init bcm_ns_init(void);
extern void __init bcm_ns_map_io(void);
extern void __init bcm_ns_timer_init(void);

#endif /* BCM_NS_COMMON_H */
