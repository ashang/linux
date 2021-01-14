/*
 * I2C client/driver for the cy14bxxxi family of i2c rtc chips.
 *
 * Author: Alexander Bigga <ab@mycable.de>
 *
 * Based on m41t00.c by Mark A. Greer <mgreer@mvista.com>
 *
 * 2006 (c) mycable GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bcd.h>
#include <linux/clk-provider.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#ifdef CONFIG_RTC_DRV_CY14B_WDT
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>
#endif

#define CY14B_REG_FLAGS	        0x00
#define CY14B_REG_ALARM_SEC	0x02
#define CY14B_REG_ALARM_MIN	0x03
#define CY14B_REG_ALARM_HOUR	0x04
#define CY14B_REG_ALARM_DAY	0x05

#define CY14B_REG_SEC		0x09
#define CY14B_REG_MIN		0x0a
#define CY14B_REG_HOUR		0x0b
#define CY14B_REG_WDAY		0x0c
#define CY14B_REG_DAY		0x0d
#define CY14B_REG_MON		0x0e
#define CY14B_REG_YEAR		0x0f
/*
#define CY14B_REG_SQW		0x13
*/

#define CY14B_DATETIME_REG_SIZE	(CY14B_REG_YEAR - CY14B_REG_SEC + 1)

//#define CY14B_ALARM_REG_SIZE  (CY14B_REG_ALARM_SEC + 1 - CY14B_REG_ALARM_MON)

#define CY14B_SQW_MAX_FREQ	32768

#define CY14B_SEC_ST		BIT(7)	/* ST: Stop Bit */
#define CY14B_ALMON_AFE	BIT(7)	/* AFE: AF Enable Bit */
#define CY14B_ALMON_SQWE	BIT(6)	/* SQWE: SQW Enable Bit */
#define CY14B_ALHOUR_HT	BIT(6)	/* HT: Halt Update Bit */

#define CY14B_FLAGS_BATT_LOW	BIT(5)	/* BL: Battery Low Bit */
#define CY14B_FLAGS_OF		BIT(4)	/* OF: Oscillator Failure Bit */
#define CY14B_FLAGS_W		BIT(1)	/* W: write enable Bit */
#define CY14B_FLAGS_R		BIT(0)	/* R: read enable Bit */
#define CY14B_FLAGS_AF		BIT(6)	/* AF: Alarm Flag Bit */
#define CY14B_WATCHDOG_RB2	BIT(7)	/* RB: Watchdog resolution */
#define CY14B_WATCHDOG_RB1	BIT(1)	/* RB: Watchdog resolution */
#define CY14B_WATCHDOG_RB0	BIT(0)	/* RB: Watchdog resolution */

#define CY14B_FEATURE_HT	BIT(0)	/* Halt feature */
#define CY14B_FEATURE_BL	BIT(1)	/* Battery low indicator */
#define CY14B_FEATURE_SQ	BIT(2)	/* Squarewave feature */
#define CY14B_FEATURE_WD	BIT(3)	/* Extra watchdog resolution */
#define CY14B_FEATURE_SQ_ALT	BIT(4)	/* RSx bits are in reg 4 */

static DEFINE_MUTEX(cy14b_rtc_mutex);
static const struct i2c_device_id cy14b_id[] = {
	{"cy14b", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cy14b_id);

static const struct of_device_id cy14b_of_match[] = {
	{
	 .compatible = "cs,cy14b",
	 .data = (void *)(CY14B_FEATURE_BL)
	 },
	{}
};

MODULE_DEVICE_TABLE(of, cy14b_of_match);

struct cy14b_data {
	unsigned long features;
	struct i2c_client *client;
	struct rtc_device *rtc;
#ifdef CONFIG_COMMON_CLK
	struct clk_hw sqw;
	unsigned long freq;
	unsigned int sqwe;
#endif
};

#define BLOCK_DATA_MAX_TRIES 10

static s32 cy14b_read_block_data_once(const struct i2c_client *client,
				      u8 command, u8 length, u8 * values)
{
	s32 i, data;

	for (i = 0; i < length; i++) {
		data = i2c_smbus_read_byte_data(client, command + i);
		if (data < 0)
			return data;
		values[i] = data;
	}

	return i;
}

static s32 cy14b_read_block_data(const struct i2c_client *client, u8 command,
				 u8 length, u8 * values)
{
	u8 oldvalues[I2C_SMBUS_BLOCK_MAX];
	s32 ret;
	int tries = 0;

	dev_dbg(&client->dev, "cy14b_read_block_data (length=%d)\n", length);
	ret = cy14b_read_block_data_once(client, command, length, values);
	if (ret < 0)
		return ret;

	do {
		if (++tries > BLOCK_DATA_MAX_TRIES) {
			dev_err(&client->dev, "cy14b_read_block_data failed\n");
			return -EIO;
		}
		memcpy(oldvalues, values, length);
		ret = cy14b_read_block_data_once(client, command, length,
						 values);
		if (ret < 0)
			return ret;
	} while (memcmp(oldvalues, values, length));

	return length;
}

static s32 cy14b_write_block_data(const struct i2c_client *client, u8 command,
				  u8 length, const u8 * values)
{
	u8 currvalues[I2C_SMBUS_BLOCK_MAX];
	int tries = 0;
	int ret;

	ret = i2c_smbus_read_byte_data(client, CY14B_REG_FLAGS);
	if (ret < 0)
		return ret;

	ret =
	    i2c_smbus_write_byte_data(client, CY14B_REG_FLAGS,
				      ret | CY14B_FLAGS_W);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to set write enable bit\n");
		return ret;
	}

	dev_dbg(&client->dev, "cy14b_write_block_data (length=%d)\n", length);
	do {
		s32 i, ret;

		if (++tries > BLOCK_DATA_MAX_TRIES) {
			dev_err(&client->dev,
				"cy14b_write_block_data failed\n");
			return -EIO;
		}
		for (i = 0; i < length; i++) {
			ret = i2c_smbus_write_byte_data(client, command + i,
							values[i]);
			if (ret < 0) {
				return ret;
			}
		}
		ret = cy14b_read_block_data_once(client, command, length,
						 currvalues);
		if (ret < 0)
			return ret;
	} while (memcmp(currvalues, values, length));

	ret = i2c_smbus_read_byte_data(client, CY14B_REG_FLAGS);
	if (ret < 0)
		return ret;

	ret =
	    i2c_smbus_write_byte_data(client, CY14B_REG_FLAGS,
				      ret & (~CY14B_FLAGS_W));
	if (ret < 0) {
		dev_err(&client->dev, "Unable to set write protect bit\n");
		return ret;
	}

	return length;
}

static int cy14b_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	u8 buf[CY14B_DATETIME_REG_SIZE];
	int flags;

	flags = i2c_smbus_read_byte_data(client, CY14B_REG_FLAGS);
	if (flags < 0)
		return flags;

	if (i2c_smbus_write_byte_data(client, CY14B_REG_FLAGS,
				      flags | CY14B_FLAGS_R)) {
		dev_err(&client->dev, "Unable to set read enable flags\n");
		return -EIO;
	}

	if (cy14b_read_block_data(client, CY14B_REG_SEC,
				  CY14B_DATETIME_REG_SIZE, buf) < 0) {
		dev_err(&client->dev, "Unable to read date\n");
		return -EIO;
	}

	if (i2c_smbus_write_byte_data(client, 0, flags & ~CY14B_FLAGS_R)) {
		dev_err(&client->dev, "Unable to disbale read enable flags\n");
		return -EIO;
	}

	tm->tm_sec = bcd2bin(buf[CY14B_REG_SEC - CY14B_REG_SEC] & 0x7f);
	tm->tm_min = bcd2bin(buf[CY14B_REG_MIN - CY14B_REG_SEC] & 0x7f);
	tm->tm_hour = bcd2bin(buf[CY14B_REG_HOUR - CY14B_REG_SEC] & 0x3f);
	tm->tm_mday = bcd2bin(buf[CY14B_REG_DAY - CY14B_REG_SEC] & 0x3f);
	tm->tm_wday = buf[CY14B_REG_WDAY - CY14B_REG_SEC] & 0x07;
	tm->tm_mon = bcd2bin(buf[CY14B_REG_MON - CY14B_REG_SEC] & 0x1f) - 1;

	/* assume 20YY not 19YY, and ignore the Century Bit */
	tm->tm_year = bcd2bin(buf[CY14B_REG_YEAR - CY14B_REG_SEC]) + 100;

	return rtc_valid_tm(tm);
}

/* Sets the given date and time to the real time clock. */
static int cy14b_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	unsigned char buf[8];
	int err, flags;

	if (tm->tm_year < 100 || tm->tm_year > 199)
		return -EINVAL;

	buf[CY14B_REG_SEC - CY14B_REG_SEC] = bin2bcd(tm->tm_sec);
	buf[CY14B_REG_MIN - CY14B_REG_SEC] = bin2bcd(tm->tm_min);
	buf[CY14B_REG_HOUR - CY14B_REG_SEC] = bin2bcd(tm->tm_hour);
	buf[CY14B_REG_DAY - CY14B_REG_SEC] = bin2bcd(tm->tm_mday);
	buf[CY14B_REG_MON - CY14B_REG_SEC] = bin2bcd(tm->tm_mon + 1);
	buf[CY14B_REG_YEAR - CY14B_REG_SEC] = bin2bcd(tm->tm_year - 100);
	buf[CY14B_REG_WDAY - CY14B_REG_SEC] = tm->tm_wday;

	if (cy14b_write_block_data
	    (client, CY14B_REG_SEC, CY14B_DATETIME_REG_SIZE,
	     buf) != CY14B_DATETIME_REG_SIZE) {
		dev_err(&client->dev, "Unable to write to date registers\n");
		return err = -EIO;
	}

	/* Clear the OSCF bit of Flags Register */
	flags = i2c_smbus_read_byte_data(client, CY14B_REG_FLAGS);
	if (flags < 0)
		return flags;

	if (i2c_smbus_write_byte_data(client, CY14B_REG_FLAGS,
				      flags & ~CY14B_FLAGS_OF)) {
		dev_err(&client->dev, "Unable to clear OSCF bit!\n");
		return -EIO;
	}

	return err = 0;
}

static int cy14b_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cy14b_data *clientdata = i2c_get_clientdata(client);
	u8 reg;

	if (clientdata->features & CY14B_FEATURE_BL) {
		reg = i2c_smbus_read_byte_data(client, CY14B_REG_FLAGS);
		seq_printf(seq, "battery\t\t: %s\n",
			   (reg & CY14B_FLAGS_BATT_LOW) ? "exhausted" : "ok");
	}
	return 0;
}

static int cy14b_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return cy14b_get_datetime(to_i2c_client(dev), tm);
}

static int cy14b_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return cy14b_set_datetime(to_i2c_client(dev), tm);
}

#if 0
static int cy14b_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	int flags, retval;

	flags = i2c_smbus_read_byte_data(client, CY14B_REG_ALARM_MON);
	if (flags < 0)
		return flags;

	if (enabled)
		flags |= CY14B_ALMON_AFE;
	else
		flags &= ~CY14B_ALMON_AFE;

	retval = i2c_smbus_write_byte_data(client, CY14B_REG_ALARM_MON, flags);
	if (retval < 0) {
		dev_err(dev, "Unable to enable alarm IRQ %d\n", retval);
		return retval;
	}
	return 0;
}

static int cy14b_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 alarmvals[5];
	int ret, err;

	alarmvals[0] = bin2bcd(alrm->time.tm_mon + 1);
	alarmvals[1] = bin2bcd(alrm->time.tm_mday);
	alarmvals[2] = bin2bcd(alrm->time.tm_hour);
	alarmvals[3] = bin2bcd(alrm->time.tm_min);
	alarmvals[4] = bin2bcd(alrm->time.tm_sec);

	/* Clear AF and AFE flags */
	ret = i2c_smbus_read_byte_data(client, CY14B_REG_ALARM_MON);
	if (ret < 0)
		return ret;
	err = i2c_smbus_write_byte_data(client, CY14B_REG_ALARM_MON,
					ret & ~(CY14B_ALMON_AFE));
	if (err < 0) {
		dev_err(dev, "Unable to clear AFE bit\n");
		return err;
	}

	/* Keep SQWE bit value */
	alarmvals[0] |= (ret & CY14B_ALMON_SQWE);

	ret = i2c_smbus_read_byte_data(client, CY14B_REG_FLAGS);
	if (ret < 0)
		return ret;

	err = i2c_smbus_write_byte_data(client, CY14B_REG_FLAGS,
					ret & ~(CY14B_FLAGS_AF));
	if (err < 0) {
		dev_err(dev, "Unable to clear AF bit\n");
		return err;
	}

	/* Write the alarm */
	if (cy14b_write_block_data
	    (client, CY14B_REG_ALARM_MON, CY14B_ALARM_REG_SIZE,
	     alarmvals) != 1) {
		dev_err(dev, "write error\n");
		return -EIO;
	}

	/* Enable the alarm interrupt */
	if (alrm->enabled) {
		alarmvals[0] |= CY14B_ALMON_AFE;
		err = i2c_smbus_write_byte_data(client, CY14B_REG_ALARM_MON,
						alarmvals[0]);
		if (err)
			return err;
	}

	return 0;
}

static int cy14b_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 alarmvals[5];
	int flags;

	if (cy14b_read_block_data
	    (client, CY14B_REG_ALARM_MON, CY14B_ALARM_REG_SIZE,
	     alarmvals) < 0) {
		dev_err(&client->dev, "read error\n");
		return -EIO;
	}

	flags = i2c_smbus_read_byte_data(client, CY14B_REG_FLAGS);
	if (flags < 0)
		return flags;

	alrm->time.tm_sec = bcd2bin(alarmvals[4] & 0x7f);
	alrm->time.tm_min = bcd2bin(alarmvals[3] & 0x7f);
	alrm->time.tm_hour = bcd2bin(alarmvals[2] & 0x3f);
	alrm->time.tm_mday = bcd2bin(alarmvals[1] & 0x3f);
	alrm->time.tm_mon = bcd2bin(alarmvals[0] & 0x3f);

	alrm->enabled = ! !(alarmvals[0] & CY14B_ALMON_AFE);
	alrm->pending = (flags & CY14B_FLAGS_AF) && alrm->enabled;

	return 0;
}
#endif

static struct rtc_class_ops cy14b_rtc_ops = {
	.read_time = cy14b_rtc_read_time,
	.set_time = cy14b_rtc_set_time,
	.proc = cy14b_rtc_proc,
};

/*
 *****************************************************************************
 *
 *	Driver Interface
 *
 *****************************************************************************
 */

static int cy14b_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct rtc_device *rtc = NULL;
	struct cy14b_data *cy14b_data = NULL;

	cy14b_data = devm_kzalloc(&client->dev, sizeof(*cy14b_data),
				  GFP_KERNEL);
	if (!cy14b_data)
		return -ENOMEM;

	cy14b_data->client = client;
	if (client->dev.of_node)
		cy14b_data->features = (unsigned long)
		    of_device_get_match_data(&client->dev);
	else
		cy14b_data->features = id->driver_data;
	i2c_set_clientdata(client, cy14b_data);

	rtc = devm_rtc_device_register(&client->dev, client->name,
				       &cy14b_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	cy14b_data->rtc = rtc;

	return 0;
}

static int cy14b_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver cy14b_driver = {
	.driver = {
		   .name = "rtc-cy14b",
		   .of_match_table = of_match_ptr(cy14b_of_match),
		   },
	.probe = cy14b_probe,
	.remove = cy14b_remove,
	.id_table = cy14b_id,
};

module_i2c_driver(cy14b_driver);

MODULE_AUTHOR("george <george.guo@pica8.com>");
MODULE_DESCRIPTION("cy14c06 series RTC I2C Client Driver");
MODULE_LICENSE("GPL");
