#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include "../usb2jtag_v1.h"
#include <mach/sync_write.h>
void __iomem *INFRA_AO_BASE;
void __iomem *USB_SIF_BASE;
static int mt_usb2jtag_hw_init(void)
{
	struct device_node *node = NULL;
	int ret;

	node = of_find_compatible_node(NULL, NULL, "mediatek,usb2jtag_v1");
	if (!node) {
		pr_err("[USB2JTAG] map node @ mediatek,usb2jtag_v1 failed\n");
		return -1;
	}
	INFRA_AO_BASE = of_iomap(node, 0);
	if (!INFRA_AO_BASE) {
		pr_err("[USB2JTAG] iomap INFRA_AO_BASE failed\n");
		return -1;
	}
	USB_SIF_BASE = of_iomap(node, 1);
	if (!USB_SIF_BASE) {
		pr_err("[USB2JTAG] iomap USB_SIF_BASE failed\n");
		return -1;
	}

	/* set ap_usb2jtag_en: 0x1000_0700 bit[13] = 1 */
	
	//enable clock

	usb_enable_clock(1);

	/* delay for enabling usb clock */
	udelay(50);
	writel(readl(USB_SIF_BASE + 0x0800) | 0x21, USB_SIF_BASE + 0x0800);
	writel((readl(USB_SIF_BASE + 0x0818) & ~(0xff << 16)) | (0x7f << 16) , USB_SIF_BASE + 0x0818);
	writel(readl(USB_SIF_BASE + 0x0820) | (0x1 << 9), USB_SIF_BASE + 0x0820);
	writel(readl(INFRA_AO_BASE + 0xF00) | (0x1 << 14), INFRA_AO_BASE + 0xF00);
	pr_err("[USB2JTAG] 0x11210800 = 0x%x\n", readl(USB_SIF_BASE + 0x0800));
	pr_err("[USB2JTAG] 0x11210808 = 0x%x\n", readl(USB_SIF_BASE + 0x0808));
	pr_err("[USB2JTAG] 0x11210820 = 0x%x\n", readl(USB_SIF_BASE + 0x0820));
	pr_err("[USB2JTAG] 0x10001F00 = 0x%x\n", readl(INFRA_AO_BASE + 0xF00));
	pr_err("[USB2JTAG] setting done\n");
	return 0;
}


static int __init mt_usb2jtag_platform_init(void)
{
	struct mt_usb2jtag_driver *mt_usb2jtag_drv;

	mt_usb2jtag_drv = get_mt_usb2jtag_drv();
	mt_usb2jtag_drv->usb2jtag_init = mt_usb2jtag_hw_init;
	mt_usb2jtag_drv->usb2jtag_suspend = NULL;
	mt_usb2jtag_drv->usb2jtag_resume = NULL;

	return 0;
}

arch_initcall(mt_usb2jtag_platform_init);
