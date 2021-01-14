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
#include <linux/kernel.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/of_device.h>

#define MII_CTRL_INIT_VAL	0x9a
#define MII_MGMT_CMD_DATA_VAL	0x587e8000
#define MII_MGMT_CMD_DATA_VAL1	0x582a6400
#define MII_MGMT_CMD_DATA_VAL2	0x58061000
#define MII_MGMT_CMD_DATA_VAL3	0x582EC000
#define MII_MGMT_CMD_DATA_VAL4	0x582E8000
#define MII_MGMT_CMD_DATA_VAL5	0x58069000
#define MII_MGMT_CMD_DATA_VAL6	0x587e80e0
#define MII_MGMT_CMD_DATA_VAL7	0x580a009c
#define MII_MGMT_CMD_DATA_VAL8	0x587e8040
#define MII_MGMT_CMD_DATA_VAL9	0x580a21d3
#define MII_MGMT_CMD_DATA_VAL10	0x58061003
#define MII_MGMT_CMD_DATA_VAL11	0x587e8060
#define MII_MGMT_CMD_DATA_VAL12	0x580af30d
#define MII_MGMT_CMD_DATA_VAL13	0x580e6302

#define USB3_IDM_CONTROL_OFFSET	0x8
#define USB3_IDM_RESET_OFFSET	0x400
#define CCB_MII_CMD_OFFSET	0x4

struct iproc_phy {
	struct usb_phy phy;

	void __iomem *cca_chip_id;

	/* USB 3.0 IDM (clock, reset) registers */
	void __iomem *usb3_idm_ctl;
	u32 usb_io_ctl_val;
	u32 usb_io_ctl_mask;
	void __iomem *usb3_idm_rst;

	/* ChipcommonB MDC/MDIO MII Management registers */
	void __iomem *ccb_mii_ctl;
	void __iomem *ccb_mii_cmd;
};

static int iproc_phy_init(struct usb_phy *usb)
{
	return 0;
}

void config_iproc_mdio(struct iproc_phy *phy)
{
	unsigned int chip_id;
	unsigned int rev_id;
	u32 reg_val;

	/*
	 * MDC/MDIO of ChipcommonB connects to internal PHYs
	 * including USB3/PCIe Combo PHY Port.
	 * ChipcommonB_MII_Mgmt_Control register Bit[9] selects
	 * if MDC/MDIO connects to external MDC/MDIO pins or
	 * internal MDC/MDIO slave devices (PHY Ports). Default
	 * value is '0' which selects connection to internal slave
	 * devices. Bit[10] should also be '0' for MDC/MDIO master.
	 *
	 * CRU_MDIO_Control register Bit[1:0] should be set to
	 * 2'b00 to select internal PHYs connected to
	 * ChipcommonB MDC/MDIO Master among internal bus mux.
	 */

	chip_id = readl_relaxed(phy->cca_chip_id);
	rev_id = ((chip_id & 0x000F0000) >> 16);
	chip_id &= 0x0000FFFF;

	/*
	 * Enable USB3 clock:
	 * Configure USB3_IDM_IDM_IO_CONTROL_DIRECT register
	 */
	reg_val = readl_relaxed(phy->usb3_idm_ctl);
	reg_val = reg_val & (~phy->usb_io_ctl_mask);
	reg_val = reg_val | phy->usb_io_ctl_val;
	writel_relaxed(reg_val, phy->usb3_idm_ctl);

	/* USB3_IDM_IDM_RESET_CONTROL, write 1 to enter reset */
	writel_relaxed(0x1, phy->usb3_idm_rst);
	mdelay(10);

	if ((chip_id >= 0xCF17) || ((chip_id == 0xCF12) && (rev_id >= 0x4))) {
		/*
		 * Configure the ChipcommonB MII Management Control register to
		 * value 0x9A for internal PHY registers access (one time),
		 * according to data sheet.
		 *
		 * Configure ChipcommonB MII Management Control register bit[10]
		 * (BYP) to '0'to select ChipcommonB MDC/MDIO as the MDC/MDIO
		 * master among internal MDC/MDIO bus mux
		 *
		 * Configure ChipcommonB MII Management Control register bit[9]
		 * (EXT) to '0' to select MDC/MDIO connecting to internal PHYs
		 *
		 * Configure ChipcommonB MII Management Control register
		 * bits[6:0](MDCDIV) with required divisor to set MDC clock
		 * (typical value 0x1A, 2.5 MHz)
		 *
		 * If preamble of 32 1's prior to send command is required: set
		 * ChipcommonB MII Management Control register bit[7](PRE) to 1
		 */

		writel_relaxed(MII_CTRL_INIT_VAL, phy->ccb_mii_ctl);
		mdelay(10);

		/*
		 * Write ChipcommonB MII Management Command Data register with
		 * values to start MDC/MDIO read/write operation for PHY
		 */
		writel_relaxed(MII_MGMT_CMD_DATA_VAL, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL2, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL1, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL3, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL4, phy->ccb_mii_cmd);
		mdelay(10);

		/* USB3_IDM_IDM_RESET_CONTROL, write 0 to exit reset */
		writel_relaxed(0x0, phy->usb3_idm_rst);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL5, phy->ccb_mii_cmd);
		mdelay(10);

		if (chip_id >=  0xCF17) {
			writel_relaxed(MII_MGMT_CMD_DATA_VAL11,
						phy->ccb_mii_cmd);
			mdelay(10);

			writel_relaxed(MII_MGMT_CMD_DATA_VAL12,
						phy->ccb_mii_cmd);
			mdelay(10);

			writel_relaxed(MII_MGMT_CMD_DATA_VAL13,
						phy->ccb_mii_cmd);
			mdelay(10);
		}
		writel_relaxed(MII_MGMT_CMD_DATA_VAL8, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL10, phy->ccb_mii_cmd);
		mdelay(10);
	} else if (chip_id < 0xCF17) {
		writel_relaxed(MII_CTRL_INIT_VAL, phy->ccb_mii_ctl);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL1, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL6, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL7, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL8, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL9, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL10, phy->ccb_mii_cmd);
		mdelay(10);

		/* USB3_IDM_IDM_RESET_CONTROL, write 0 to exit reset */
		writel_relaxed(0x0, phy->usb3_idm_rst);
		mdelay(10);
	} else {
		writel_relaxed(MII_CTRL_INIT_VAL, phy->ccb_mii_ctl);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL2, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL1, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL3, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL4, phy->ccb_mii_cmd);
		mdelay(10);

		/* USB3_IDM_IDM_RESET_CONTROL, write 0 to exit reset */
		writel_relaxed(0x0, phy->usb3_idm_rst);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL5, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL11, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL12, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL13, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL8, phy->ccb_mii_cmd);
		mdelay(10);

		writel_relaxed(MII_MGMT_CMD_DATA_VAL10, phy->ccb_mii_cmd);
		mdelay(10);
	}
}

static void iproc_phy_shutdown(struct usb_phy *phy)
{
}

static const struct of_device_id iproc_phy_dt_ids[] = {
	{ .compatible = "brcm,iproc-usb3-phy", },
	{ }
};
MODULE_DEVICE_TABLE(of, iproc_phy_dt_ids);

static int iproc_phy_probe(struct platform_device *pdev)
{
	struct device_node *phy_node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct iproc_phy *phy;
	const struct of_device_id *match;
	void __iomem *base_addr;
	int ret = -ENODEV;

	match = of_match_device(iproc_phy_dt_ids, &pdev->dev);
	if (!match) {
		dev_err(dev, "can't find DT configuration\n");
		ret = -ENODEV;
		goto error1;
	}

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy) {
		dev_err(dev, "Failed to allocate USB PHY structure!\n");
		ret = -ENOMEM;
		goto error1;
	}

	base_addr = of_iomap(phy_node, 0);
	if (!base_addr) {
		dev_err(dev, "can't iomap USB3 IDM Control reg\n");
		ret = -EIO;
		goto error2;
	}

	phy->usb3_idm_ctl = base_addr + USB3_IDM_CONTROL_OFFSET;

	ret = of_property_read_u32(phy_node, "io_control_val",
					&phy->usb_io_ctl_val);
	if (!ret) {
		ret = of_property_read_u32(phy_node, "io_control_mask",
						&phy->usb_io_ctl_mask);
		if (ret) {
			dev_err(dev, "can't read DT entry IO_CONTROL_MASK\n");
			goto error3;
		}
	} else {
		dev_err(dev, "can't read DT entry IO_CONTROL_VAL\n");
		goto error3;
	}

	phy->usb3_idm_rst = base_addr + USB3_IDM_RESET_OFFSET;

	phy->cca_chip_id = of_iomap(phy_node, 1);
	if (!phy->cca_chip_id) {
		dev_err(dev, "can't iomap CCA Chip ID reg\n");
		ret = -EIO;
		goto error3;
	}

	phy->ccb_mii_ctl = of_iomap(phy_node, 2);
	if (!phy->ccb_mii_ctl) {
		dev_err(dev, "can't iomap CCB MII Mgmt reg\n");
		ret = -EIO;
		goto error4;
	}

	phy->ccb_mii_cmd = phy->ccb_mii_ctl + CCB_MII_CMD_OFFSET;

	config_iproc_mdio(phy);

	phy->phy.dev = dev;
	phy->phy.init = iproc_phy_init;
	phy->phy.shutdown = iproc_phy_shutdown;
	phy->phy.type = USB_PHY_TYPE_USB3;

	platform_set_drvdata(pdev, phy);

	ret = usb_add_phy_dev(&phy->phy);
	if (ret)
		goto error5;

	return 0;

error5:
	iounmap(phy->ccb_mii_ctl);
	phy->ccb_mii_ctl = NULL;
error4:
	iounmap(phy->cca_chip_id);
	phy->cca_chip_id = NULL;
error3:
	iounmap(base_addr);
	phy->usb3_idm_ctl = NULL;
	phy->usb3_idm_rst = NULL;
error2:
	kfree(phy);
error1:
	return ret;
}

static int iproc_phy_remove(struct platform_device *pdev)
{
	struct iproc_phy *phy = platform_get_drvdata(pdev);
	usb_remove_phy(&phy->phy);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver iproc_usb_phy_driver = {
	.probe = iproc_phy_probe,
	.remove = iproc_phy_remove,
	.driver = {
		.name = "iproc-usb-phy",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(iproc_phy_dt_ids),
	},
};

static int __init iproc_usb_phy_init(void)
{
	return platform_driver_register(&iproc_usb_phy_driver);
}
subsys_initcall(iproc_usb_phy_init);

static void __exit iproc_usb_phy_exit(void)
{
	platform_driver_unregister(&iproc_usb_phy_driver);
}
module_exit(iproc_usb_phy_exit);

MODULE_ALIAS("platform:iproc-usb-phy");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom IPROC USB 3.0 PHY driver");
MODULE_LICENSE("GPL");
