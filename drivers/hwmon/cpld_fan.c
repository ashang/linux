/*
 * cpld-fan.c - Support for CPLD fan
 *
 * Copyright (C) Vic Lan <vic.lan@pica8.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/cpld_fan.h>

#define FANR (1<<4)

static struct fan_info *fi;
static struct device *hwmon_dev;

static void fan_update_device(struct device *dev)
{
	int i;

	mutex_lock(&fi->update_lock);
	if (time_after(jiffies, fi->last_updated + 2 * HZ) || !fi->valid) {
		fi->last_updated = jiffies;

		for (i = 0; i < fi->fan_num; i++) {
			fi->fan_input[i] = fi->cpld_get_fan_input(i);
			if (!fi->cpld_get_fan_fault) {
				fi->fan_fault[i] = !(fi->fan_input[i]);
			} else {
				/*
				 * if no fault register, then
				 * use fan_input to fake it.
				 * */
				if (fi->cpld_get_fan_fault(i) ||
					    !fi->cpld_get_fan_input(i)) {
					fi->fan_fault[i] = true;
				} else {
					fi->fan_fault[i] = false;
				}
			}

			/* fan and fanr use same */
			if (fi->cpld_get_pwm) {
				/* maybe NO pwm: z9100 */
				fi->pwm[i] = fi->cpld_get_pwm(i);
			}
		}

		for (i = 0; i < fi->fanr_num; i++) {
			fi->fanr_input[i] = fi->cpld_get_fanr_input(i);
			if (!fi->cpld_get_fanr_fault) {
				fi->fanr_fault[i] = !(fi->fanr_input[i]);
			} else {
				/*
				 * if no fault register, then
				 * use fan_input to fake it.
				 * */
				if (fi->cpld_get_fanr_fault(i) ||
					    !fi->cpld_get_fanr_input(i)) {
					fi->fanr_fault[i] = true;
				} else {
					fi->fanr_fault[i] = false;
				}
			}
		}

		for (i = 0; i < fi->temp_num; i++) {
			fi->temp_input[i] = fi->cpld_get_temp_input(i);
		}

		fi->valid = 1;
	}
	mutex_unlock(&fi->update_lock);
}
static ssize_t show_name(struct device *dev,
			       struct device_attribute *dev_attr, char *buf)
{
	return sprintf(buf, "%s\n", DRV_NAME);
}
static ssize_t show_temp_input(struct device *dev,
			       struct device_attribute *dev_attr, char *buf)
{
	long temp;
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);

	fan_update_device(dev);

	if (!fi->temp_input[attr->index]) {
		return sprintf(buf, "N/A\n");
	}

	temp = fi->temp_input[attr->index];
	return sprintf(buf, "%ld\n", temp);
}
static ssize_t show_pwm(struct device *dev,
			struct device_attribute *dev_attr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);

	if (!fi->cpld_set_pwm) {
		return -EINVAL;
	}

	fan_update_device(dev);

	if (fi->pwm[attr->index] < 0)
		return sprintf(buf, "N/A\n");
	return sprintf(buf, "%d\n", fi->pwm[attr->index]);
}

static ssize_t set_pwm(struct device *dev,
		       struct device_attribute *dev_attr,
		       const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	unsigned long val;
	int res;

	if (!fi->cpld_set_pwm) {
		return -EINVAL;
	}

	res = kstrtoul(buf, 10, &val);

	if (res)
		return res;

	val = clamp_val(val, 0, 255);
	mutex_lock(&fi->update_lock);
	fi->cpld_set_pwm(attr->index, val);
	mutex_unlock(&fi->update_lock);
	return count;
}

static ssize_t show_fan_input(struct device *dev,
			      struct device_attribute *dev_attr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	int index = (attr->index & (~FANR));

	fan_update_device(dev);
	if (attr->index & FANR) {
		if (fi->fanr_input[index] < 0 )
			return sprintf(buf, "N/A\n");
		return sprintf(buf, "%d\n", (fi->fanr_input[index]));
	} else {

		if (fi->fan_input[index] < 0 )
			return sprintf(buf, "N/A\n");
		return sprintf(buf, "%d\n", (fi->fan_input[index]));
	}
}
static ssize_t show_fan_fault(struct device *dev,
			      struct device_attribute *dev_attr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	int index = (attr->index & (~FANR));

	fan_update_device(dev);
	if (attr->index & FANR) {
		if (fi->fanr_fault[index] < 0)
			return sprintf(buf, "N/A\n");
		return sprintf(buf, "%d\n", (fi->fanr_fault[index]));
	} else {
		if (fi->fan_fault[index] < 0)
			return sprintf(buf, "N/A\n");
		return sprintf(buf, "%d\n", (fi->fan_fault[index]));
	}
}

static SENSOR_DEVICE_ATTR(name, S_IRUGO, show_name, NULL, 0);
static SENSOR_DEVICE_ATTR(pwm1, S_IWUSR | S_IRUGO, show_pwm, set_pwm, 0);
static SENSOR_DEVICE_ATTR(pwm2, S_IWUSR | S_IRUGO, show_pwm, set_pwm, 1);
static SENSOR_DEVICE_ATTR(pwm3, S_IWUSR | S_IRUGO, show_pwm, set_pwm, 2);
static SENSOR_DEVICE_ATTR(pwm4, S_IWUSR | S_IRUGO, show_pwm, set_pwm, 3);
static SENSOR_DEVICE_ATTR(pwm5, S_IWUSR | S_IRUGO, show_pwm, set_pwm, 4);
static SENSOR_DEVICE_ATTR(pwm6, S_IWUSR | S_IRUGO, show_pwm, set_pwm, 5);
static SENSOR_DEVICE_ATTR(fan1_input, S_IRUGO, show_fan_input, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, S_IRUGO, show_fan_input, NULL, 1);
static SENSOR_DEVICE_ATTR(fan3_input, S_IRUGO, show_fan_input, NULL, 2);
static SENSOR_DEVICE_ATTR(fan4_input, S_IRUGO, show_fan_input, NULL, 3);
static SENSOR_DEVICE_ATTR(fan5_input, S_IRUGO, show_fan_input, NULL, 4);
static SENSOR_DEVICE_ATTR(fan6_input, S_IRUGO, show_fan_input, NULL, 5);
static SENSOR_DEVICE_ATTR(fan1_fault, S_IRUGO, show_fan_fault, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_fault, S_IRUGO, show_fan_fault, NULL, 1);
static SENSOR_DEVICE_ATTR(fan3_fault, S_IRUGO, show_fan_fault, NULL, 2);
static SENSOR_DEVICE_ATTR(fan4_fault, S_IRUGO, show_fan_fault, NULL, 3);
static SENSOR_DEVICE_ATTR(fan5_fault, S_IRUGO, show_fan_fault, NULL, 4);
static SENSOR_DEVICE_ATTR(fan6_fault, S_IRUGO, show_fan_fault, NULL, 5);
static SENSOR_DEVICE_ATTR(fanr1_input, S_IRUGO, show_fan_input, NULL, 0|FANR);
static SENSOR_DEVICE_ATTR(fanr2_input, S_IRUGO, show_fan_input, NULL, 1|FANR);
static SENSOR_DEVICE_ATTR(fanr3_input, S_IRUGO, show_fan_input, NULL, 2|FANR);
static SENSOR_DEVICE_ATTR(fanr4_input, S_IRUGO, show_fan_input, NULL, 3|FANR);
static SENSOR_DEVICE_ATTR(fanr5_input, S_IRUGO, show_fan_input, NULL, 4|FANR);
static SENSOR_DEVICE_ATTR(fanr6_input, S_IRUGO, show_fan_input, NULL, 5|FANR);
static SENSOR_DEVICE_ATTR(fanr1_fault, S_IRUGO, show_fan_fault, NULL, 0|FANR);
static SENSOR_DEVICE_ATTR(fanr2_fault, S_IRUGO, show_fan_fault, NULL, 1|FANR);
static SENSOR_DEVICE_ATTR(fanr3_fault, S_IRUGO, show_fan_fault, NULL, 2|FANR);
static SENSOR_DEVICE_ATTR(fanr4_fault, S_IRUGO, show_fan_fault, NULL, 3|FANR);
static SENSOR_DEVICE_ATTR(fanr5_fault, S_IRUGO, show_fan_fault, NULL, 4|FANR);
static SENSOR_DEVICE_ATTR(fanr6_fault, S_IRUGO, show_fan_fault, NULL, 5|FANR);
static SENSOR_DEVICE_ATTR(temp0_input, S_IRUGO, show_temp_input, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp_input, NULL, 1);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, show_temp_input, NULL, 2);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO, show_temp_input, NULL, 3);
static SENSOR_DEVICE_ATTR(temp4_input, S_IRUGO, show_temp_input, NULL, 4);
static SENSOR_DEVICE_ATTR(temp5_input, S_IRUGO, show_temp_input, NULL, 5);

static struct attribute *pwm_attrs[] = {
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm3.dev_attr.attr,
	&sensor_dev_attr_pwm4.dev_attr.attr,
	&sensor_dev_attr_pwm5.dev_attr.attr,
	&sensor_dev_attr_pwm6.dev_attr.attr,
	NULL
};
static struct attribute *fan_input_attrs[] = {
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan3_input.dev_attr.attr,
	&sensor_dev_attr_fan4_input.dev_attr.attr,
	&sensor_dev_attr_fan5_input.dev_attr.attr,
	&sensor_dev_attr_fan6_input.dev_attr.attr,
	NULL
};
static struct attribute *fanr_input_attrs[] = {
	&sensor_dev_attr_fanr1_input.dev_attr.attr,
	&sensor_dev_attr_fanr2_input.dev_attr.attr,
	&sensor_dev_attr_fanr3_input.dev_attr.attr,
	&sensor_dev_attr_fanr4_input.dev_attr.attr,
	&sensor_dev_attr_fanr5_input.dev_attr.attr,
	&sensor_dev_attr_fanr6_input.dev_attr.attr,
	NULL
};
static struct attribute *fan_fault_attrs[] = {
	&sensor_dev_attr_fan1_fault.dev_attr.attr,
	&sensor_dev_attr_fan2_fault.dev_attr.attr,
	&sensor_dev_attr_fan3_fault.dev_attr.attr,
	&sensor_dev_attr_fan4_fault.dev_attr.attr,
	&sensor_dev_attr_fan5_fault.dev_attr.attr,
	&sensor_dev_attr_fan6_fault.dev_attr.attr,
	NULL
};
static struct attribute *fanr_fault_attrs[] = {
	&sensor_dev_attr_fanr1_fault.dev_attr.attr,
	&sensor_dev_attr_fanr2_fault.dev_attr.attr,
	&sensor_dev_attr_fanr3_fault.dev_attr.attr,
	&sensor_dev_attr_fanr4_fault.dev_attr.attr,
	&sensor_dev_attr_fanr5_fault.dev_attr.attr,
	&sensor_dev_attr_fanr6_fault.dev_attr.attr,
	NULL
};
static struct attribute *temp_input_attrs[] = {
	&sensor_dev_attr_temp0_input.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp4_input.dev_attr.attr,
	&sensor_dev_attr_temp5_input.dev_attr.attr,
	NULL
};

void fan_prepare_probe(int fan_num, int fanr_num, int temp_num,
	      struct fan_info *f)
{
	fi = f;
	fi->fan_num = fan_num;
	fi->fanr_num = fanr_num;
	fi->temp_num = temp_num;
	fi->last_updated = jiffies;
}
EXPORT_SYMBOL_GPL(fan_prepare_probe);

static int fan_remove(struct platform_device *pdev)
{
	hwmon_device_unregister(hwmon_dev);
	return 0;
}
static int fan_probe(struct platform_device *pdev)
{
	int err = 0;
	int i;

	mutex_init(&fi->update_lock);

	err += sysfs_create_file(&pdev->dev.kobj,
			 &sensor_dev_attr_name.dev_attr.attr);

	for (i = 0; i < fi->fan_num; i++) {
		err += sysfs_create_file(&pdev->dev.kobj, fan_input_attrs[i]);
		err += sysfs_create_file(&pdev->dev.kobj, fan_fault_attrs[i]);
		err += sysfs_create_file(&pdev->dev.kobj, pwm_attrs[i]);
	}
	for (i = 0; i < fi->fanr_num; i++) {
		err += sysfs_create_file(&pdev->dev.kobj, fanr_input_attrs[i]);
		err += sysfs_create_file(&pdev->dev.kobj, fanr_fault_attrs[i]);
	}

	for (i = 0; i < fi->temp_num; i++) {
		err += sysfs_create_file(&pdev->dev.kobj, temp_input_attrs[i]);
	}

	if (err < 0) {
		return -1;
	}

	// register
	hwmon_dev = hwmon_device_register(&pdev->dev);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static struct platform_driver cpld_fan_driver = {
	.probe = fan_probe,
	.remove = fan_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(cpld_fan_driver);

MODULE_AUTHOR("Vic Lan <vic.lan@pica8.com");
MODULE_DESCRIPTION("cpld fan driver");
MODULE_LICENSE("GPL");
