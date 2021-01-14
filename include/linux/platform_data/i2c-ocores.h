/* SPDX-License-Identifier: GPL-2.0 */
/*
 * i2c-ocores.h - definitions for the i2c-ocores interface
 * Copyright (C) 2018 Cumulus Networks, Inc.  All rights reserved
 *
 * Peter Korsgaard <peter@korsgaard.com>
 */

#ifndef _LINUX_I2C_OCORES_H
#define _LINUX_I2C_OCORES_H

enum {
	OCI2C_INTERRUPT,	/* specify traditional interrupt operation */
	OCI2C_POLL,		/* specify polled operation */
};

struct ocores_i2c_platform_data {
	u32 reg_shift; /* register offset shift value */
	u32 reg_io_width; /* register io read/write width */
	u32 clock_khz; /* input clock in kHz */
	u32 bus_khz; /* bus clock in kHz */
	bool big_endian; /* registers are big endian */
	u8 num_devices; /* number of devices in the devices list */
	int interrupt_mode; /* OCI2C_POLL or OCI2C_INTERRUPT */
	struct i2c_board_info const *devices; /* devices connected to the bus */
};

#endif /* _LINUX_I2C_OCORES_H */
