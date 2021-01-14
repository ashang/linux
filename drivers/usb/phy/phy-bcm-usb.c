/*
 * Copyright (C) 2014, Broadcom Corporation. All Rights Reserved.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>

#define CDRU_USBPHY_CLK_RST_SEL_OFFSET 0x11b4
#define CDRU_USBPHY2_HOST_DEV_SEL_OFFSET 0x11b8
#define CDRU_SPARE_REG_0_OFFSET 0x1238
#define CRMU_USB_PHY_AON_CTRL_OFFSET 0x00028

#define USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET 0x0408
#define USB2D_IDM_IDM_IO_CONTROL_DIRECT_OFFSET 0x1408
#define USB2_IDM_IDM_RESET_CONTROL_OFFSET 0x0800
#define USB2D_IDM_IDM_RESET_CONTROL_OFFSET 0x1800

#define CRMU_USBPHY_P0_AFE_CORERDY_VDDC 1
#define CRMU_USBPHY_P1_AFE_CORERDY_VDDC 9
#define CRMU_USBPHY_P2_AFE_CORERDY_VDDC 17

struct bcm_phy {
	struct usb_phy phy;
	struct clk *clk;
};

static int bcm_phy_init(struct usb_phy *phy)
{
	return 0;
}

static void bcm_phy_shutdown(struct usb_phy *phy)
{
}

static int inith(struct device *dev, struct resource *res0,
	struct resource *res1)
{
	void *__iomem reg_addr;
	u32 reg_val;
	u32 p0host = 1;
	u32 p1host = 0;
	u32 p2host = 0;
	const void *ptr;

	ptr = of_get_property(dev->of_node, "p0-host", 0);
	if (ptr)
		p0host = be32_to_cpup(ptr) & 0x1;

	ptr = of_get_property(dev->of_node, "p1-host", 0);
	if (ptr)
		p1host = be32_to_cpup(ptr) & 0x1;

	ptr = of_get_property(dev->of_node, "p2-host", 0);
	if (ptr)
		p2host = be32_to_cpup(ptr) & 0x1;

	reg_addr = devm_ioremap_nocache(dev, res1->start, resource_size(res1));
	if (!reg_addr)
		return -ENOMEM;

	/*phy0 is connected to host*/
	reg_val = 0;
	writel(reg_val, reg_addr + CDRU_USBPHY_CLK_RST_SEL_OFFSET);

	reg_val = 7;
	writel(reg_val, reg_addr + CDRU_SPARE_REG_0_OFFSET);

	reg_val = p2host;
	writel(reg_val, reg_addr + CDRU_USBPHY2_HOST_DEV_SEL_OFFSET);

	/*Bring the AFE block out of reset to start powering up the PHY*/
	reg_val = readl(reg_addr + CRMU_USB_PHY_AON_CTRL_OFFSET);
	reg_val |= (p0host << CRMU_USBPHY_P0_AFE_CORERDY_VDDC);
	reg_val |= (p1host << CRMU_USBPHY_P1_AFE_CORERDY_VDDC);
	reg_val |= (p2host << CRMU_USBPHY_P2_AFE_CORERDY_VDDC);
	writel(reg_val, reg_addr + CRMU_USB_PHY_AON_CTRL_OFFSET);

	reg_addr = devm_ioremap_nocache(dev, res0->start, resource_size(res0));
	if (!reg_addr)
		return -ENOMEM;

	/*awuser + aruser values + clock enable*/
	reg_val = 0x03de0001;/*2^25+2^24+2^23+2^22 + 2^20+2^19+2^18+2^17 + 2^0*/
	writel(reg_val, reg_addr + USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

	/*exit reset*/
	reg_val = 0;
	writel(reg_val, reg_addr + USB2_IDM_IDM_RESET_CONTROL_OFFSET);

	return 0;
}

static int cygnus_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource res0, res1;
	struct bcm_phy *phy;
	int ret;

	ret = of_address_to_resource(dev->of_node, 0, &res0);
	if (ret) {
		dev_warn(dev, "Failed to obtain device tree resource\n");
		return ret;
	}

	ret = of_address_to_resource(dev->of_node, 1, &res1);
	if (ret) {
		dev_warn(dev, "Failed to obtain device tree resource\n");
		return ret;
	}

	ret = inith(dev, &res0, &res1);
	if (ret)
		return ret;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy) {
		dev_warn(dev, "Failed to allocate USB PHY structure!\n");
		return -ENOMEM;
	}

	phy->phy.dev = dev;
	phy->phy.init = bcm_phy_init;
	phy->phy.shutdown = bcm_phy_shutdown;
	phy->phy.type = USB_PHY_TYPE_USB2;

	platform_set_drvdata(pdev, phy);

	ret = usb_add_phy_dev(&phy->phy);
	if (ret)
		return ret;

	return 0;
}
static int iproc_phy_probe(struct platform_device *pdev)
{
	void *__iomem base_addr;
	struct bcm_phy *phy;
	u32 io_control_val;
	u32 io_control_mask;
	u32 reg_val;
	int ret;

	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	base_addr = of_iomap(node, 0);
	if (!base_addr) {
		dev_err(&pdev->dev, "can't iomap USB PHY base address\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(node, "io_control_val", &io_control_val);
	if (!ret) {
		ret = of_property_read_u32(node, "io_control_mask",
							&io_control_mask);
		if (!ret) {
			reg_val = readl(base_addr +
				 USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
			reg_val = reg_val & (~io_control_mask);
			reg_val = reg_val | io_control_val;
			writel(reg_val, base_addr +
					USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		}
	}

	/*exit reset*/
	reg_val = 0;
	writel(reg_val, base_addr + USB2_IDM_IDM_RESET_CONTROL_OFFSET);
	/* check for cache USER settings */
	base_addr = of_iomap(node, 1);
	if (base_addr) {
		ret = of_property_read_u32(node, "coherent_val",
							 &io_control_val);
		if (!ret) {
			ret = of_property_read_u32(node, "coherent_mask",
							&io_control_mask);
			if (!ret) {
				reg_val = readl(base_addr);
				reg_val = reg_val & (~io_control_mask);
				reg_val = reg_val | io_control_val;
				writel(reg_val, base_addr);
			}
		}
	}

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy) {
		dev_warn(dev, "Failed to allocate USB PHY structure!\n");
		return -ENOMEM;
	}

	phy->phy.dev = dev;
	phy->phy.init = bcm_phy_init;
	phy->phy.shutdown = bcm_phy_shutdown;
	phy->phy.type = USB_PHY_TYPE_USB2;

	of_phy_provider_register(dev, of_phy_simple_xlate);

	platform_set_drvdata(pdev, phy);

	ret = usb_add_phy_dev(&phy->phy);
	if (ret)
		return ret;

	return 0;
}
static int bcm_phy_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	if (of_device_is_compatible(node, "brcm,cygnus"))
		return cygnus_phy_probe(pdev);
	else
		return iproc_phy_probe(pdev);
}
static int bcm_phy_remove(struct platform_device *pdev)
{
	struct bcm_phy *phy = platform_get_drvdata(pdev);
	usb_remove_phy(&phy->phy);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id bcm_phy_dt_ids[] = {
	{ .compatible = "brcm,usb-phy", },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm_phy_dt_ids);

static struct platform_driver bcm_phy_driver = {
	.probe = bcm_phy_probe,
	.remove = bcm_phy_remove,
	.driver = {
		.name = "bcm-usbphy",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bcm_phy_dt_ids),
	},
};

static int __init bcm_usb_phy_init(void)
{
	return platform_driver_register(&bcm_phy_driver);
}
subsys_initcall(bcm_usb_phy_init);

static void __exit bcm_usb_phy_exit(void)
{
	platform_driver_unregister(&bcm_phy_driver);
}
module_exit(bcm_usb_phy_exit);

MODULE_ALIAS("platform:bcm-usbphy");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom USB PHY driver");
MODULE_LICENSE("GPL");

