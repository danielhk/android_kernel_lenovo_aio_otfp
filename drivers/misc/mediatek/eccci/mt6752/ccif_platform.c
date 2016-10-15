#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <mach/mt_spm_sleep.h>
#include "ccci_config.h"
#include "ccci_core.h"
#include "ccci_platform.h"
#include "ccif_platform.h"
#include "modem_ccif.h"
#include "modem_reg_base.h"
#include <mach/mt_clkmgr.h>
#include <mach/mt6605.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#define TAG "cif"

int md_ccif_get_modem_hw_info(struct platform_device *dev_ptr, struct ccci_dev_cfg *dev_cfg, struct md_hw_info *hw_info)
{
	struct device_node *node = NULL;

	memset(dev_cfg, 0, sizeof(struct ccci_dev_cfg));
	memset(hw_info, 0, sizeof(struct md_hw_info));

#ifdef CONFIG_OF
	if (dev_ptr->dev.of_node == NULL) {
		CCCI_ERR_MSG(dev_cfg->index, TAG, "modem OF node NULL\n");
		return -1;
	}

	of_property_read_u32(dev_ptr->dev.of_node, "cell-index", &dev_cfg->index);
	CCCI_INF_MSG(dev_cfg->index, TAG, "modem hw info get idx:%d\n", dev_cfg->index);
	if (!get_modem_is_enabled(dev_cfg->index)) {
		CCCI_ERR_MSG(dev_cfg->index, TAG, "modem %d not enable, exit\n", dev_cfg->index + 1);
		return -1;
	}
#else
	struct ccci_dev_cfg *dev_cfg_ptr = (struct ccci_dev_cfg *)dev->dev.platform_data;

	dev_cfg->index = dev_cfg_ptr->index;

	CCCI_INF_MSG(dev_cfg->index, TAG, "modem hw info get idx:%d\n", dev_cfg->index);
	if (!get_modem_is_enabled(dev_cfg->index)) {
		CCCI_ERR_MSG(dev_cfg->index, TAG, "modem %d not enable, exit\n", dev_cfg->index + 1);
		return -1;
	}
#endif

	switch (dev_cfg->index) {
	case 1:		/* MD_SYS2 */
#ifdef CONFIG_OF
		of_property_read_u32(dev_ptr->dev.of_node, "ccif,major", &dev_cfg->major);
		of_property_read_u32(dev_ptr->dev.of_node, "ccif,minor_base", &dev_cfg->minor_base);
		of_property_read_u32(dev_ptr->dev.of_node, "ccif,capability", &dev_cfg->capability);

		hw_info->ap_ccif_base = of_iomap(dev_ptr->dev.of_node, 0);
		/* hw_info->md_ccif_base = hw_info->ap_ccif_base+0x1000; */
		node = of_find_compatible_node(NULL, NULL, "mediatek,MD_CCIF1");
		hw_info->md_ccif_base = of_iomap(node, 0);

		hw_info->ap_ccif_irq_id = irq_of_parse_and_map(dev_ptr->dev.of_node, 0);
		hw_info->md_wdt_irq_id = irq_of_parse_and_map(dev_ptr->dev.of_node, 1);

		/* Device tree using none flag to register irq, sensitivity has set at "irq_of_parse_and_map" */
		hw_info->ap_ccif_irq_flags = IRQF_TRIGGER_NONE;
		hw_info->md_wdt_irq_flags = IRQF_TRIGGER_NONE;
#endif

		hw_info->sram_size = CCIF_SRAM_SIZE;
		hw_info->md_rgu_base = MD2_RGU_BASE;
		hw_info->md_boot_slave_Vector = MD2_BOOT_VECTOR;
		hw_info->md_boot_slave_Key = MD2_BOOT_VECTOR_KEY;
		hw_info->md_boot_slave_En = MD2_BOOT_VECTOR_EN;

		break;
	default:
		return -1;
	}

	CCCI_INF_MSG(dev_cfg->index, TAG, "modem ccif of node get dev_major:%d\n", dev_cfg->major);
	CCCI_INF_MSG(dev_cfg->index, TAG, "modem ccif of node get minor_base:%d\n", dev_cfg->minor_base);
	CCCI_INF_MSG(dev_cfg->index, TAG, "modem ccif of node get capability:%d\n", dev_cfg->capability);

	CCCI_INF_MSG(dev_cfg->index, TAG, "ap_ccif_base:0x%p\n", (void *)hw_info->ap_ccif_base);
	CCCI_INF_MSG(dev_cfg->index, TAG, "ccif_irq_id:%d\n", hw_info->ap_ccif_irq_id);
	CCCI_INF_MSG(dev_cfg->index, TAG, "md_wdt_irq_id:%d\n", hw_info->md_wdt_irq_id);

	return 0;
}

int md_ccif_io_remap_md_side_register(struct ccci_modem *md)
{
	struct md_ccif_ctrl *md_ctrl = (struct md_ccif_ctrl *)md->private_data;

	md_ctrl->md_boot_slave_Vector = ioremap_nocache(md_ctrl->hw_info->md_boot_slave_Vector, 0x4);
	md_ctrl->md_boot_slave_Key = ioremap_nocache(md_ctrl->hw_info->md_boot_slave_Key, 0x4);
	md_ctrl->md_boot_slave_En = ioremap_nocache(md_ctrl->hw_info->md_boot_slave_En, 0x4);
	md_ctrl->md_rgu_base = ioremap_nocache(md_ctrl->hw_info->md_rgu_base, 0x40);

	return 0;
}

int md_ccif_let_md_go(struct ccci_modem *md)
{
	struct md_ccif_ctrl *md_ctrl = (struct md_ccif_ctrl *)md->private_data;

	if (MD_IN_DEBUG(md)) {
		CCCI_INF_MSG(md->index, TAG, "DBG_FLAG_JTAG is set\n");
		return -1;
	}
	CCCI_INF_MSG(md->index, TAG, "md_ccif_let_md_go\n");
	/* set the start address to let modem to run */
	ccif_write32(md_ctrl->md_boot_slave_Key, 0, MD2_BOOT_VECTOR_KEY_VALUE);	/* make boot vector programmable */
	ccif_write32(md_ctrl->md_boot_slave_Vector, 0, MD2_BOOT_VECTOR_VALUE);
		/* after remap, MD ROM address is 0 from MD's view */
	ccif_write32(md_ctrl->md_boot_slave_En, 0, MD2_BOOT_VECTOR_EN_VALUE);	/* make boot vector take effect */
	return 0;
}

int md_ccif_power_on(struct ccci_modem *md)
{
	int ret = 0;
	struct md_ccif_ctrl *md_ctrl = (struct md_ccif_ctrl *)md->private_data;

	switch (md->index) {
	case MD_SYS2:
		ret = md_power_on(SYS_MD2);

#ifdef FEATURE_INFORM_NFC_VSIM_CHANGE
		inform_nfc_vsim_change(md->index, 1, 0);
#endif
		break;
	}
	CCCI_INF_MSG(md->index, TAG, "md_ccif_power_on:ret=%d\n", ret);
	if (ret == 0) {
		/* disable MD WDT */
		ccif_write32(md_ctrl->md_rgu_base, WDT_MD_MODE, WDT_MD_MODE_KEY);
	}
	return ret;
}

int md_ccif_power_off(struct ccci_modem *md, unsigned int timeout)
{
	int ret = 0;

	switch (md->index) {
	case MD_SYS2:
#ifdef FEATURE_INFORM_NFC_VSIM_CHANGE
		inform_nfc_vsim_change(md->index, 0, 0);
#endif
		ret = md_power_off(SYS_MD2, timeout);
		break;
	}
	CCCI_INF_MSG(md->index, TAG, "md_ccif_power_off:ret=%d\n", ret);
	return ret;
}
