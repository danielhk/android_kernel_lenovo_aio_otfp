#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/xlog.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#else
#include <mach/mt_reg_base.h>
#endif
#include "ddr_info.h"

static struct ddr_info_driver ddr_info_driver = {
	.driver = {
        .name = "ddr_info",
        .bus = &platform_bus_type,
        .owner = THIS_MODULE,
	},
	.id_table = NULL,
};

#if 0
static unsigned int get_mempll_freq(struct ddr_info_driver *ddr_driver)
{
    unsigned int u4value1, u4value2;
    unsigned int MPLL_POSDIV, MPLL_PCW, MPLL_FOUT, MEMPLL_FBKDIV, MEMPLL_FOUT;
    
    u4value1 = readl(IOMEM(ddr_driver->reg_base.apmixed_base + 0x280)); 
    u4value2 = GET_FIELD(u4value1, 0x00000070, 4);
    MPLL_POSDIV = 1<<u4value2;

    u4value1 = readl(IOMEM(ddr_driver->reg_base.apmixed_base + 0x284)); 
    MPLL_PCW = GET_FIELD(u4value1, 0x001fffff, 0);

    MPLL_FOUT = 26/1*MPLL_PCW;
    MPLL_FOUT = DIV_ROUND_UP(MPLL_FOUT, MPLL_POSDIV*28); // freq*16384

    u4value1 = readl(ddr_driver->reg_base.ddrphy_base + 0x614); 

    MEMPLL_FBKDIV = GET_FIELD(u4value1, 0x007f0000, 16);
    MEMPLL_FOUT = MPLL_FOUT*1*4*(MEMPLL_FBKDIV+1);
    MEMPLL_FOUT = DIV_ROUND_UP(MEMPLL_FOUT, 16384);

    ddr_info_warn("MPLL_POSDIV=%d, MPLL_PCW=0x%x, MPLL_FOUT=%d, MEMPLL_FBKDIV=%d, MEMPLL_FOUT=%d\n",\
                   MPLL_POSDIV, MPLL_PCW, MPLL_FOUT, MEMPLL_FBKDIV, MEMPLL_FOUT);

    return MEMPLL_FOUT;
}
#else
struct mempll_tbl{
    unsigned int MEMPLL_N_INFO;
    unsigned int freq;
};

static struct mempll_tbl mempll_tbl[]= {
    { 0x6fa4158d, 1866 },
    { 0x7163b13b, 1780 },
    { 0x72492492, 1600 },
    { 0x71383483, 1333 },
    { 0x70c00000, 1066 },
    { 0x71c4ec4e, 1160 },
    { 0x6f89d89d, 800 },
    { 0x714dee6b, 667 }
};

static unsigned int get_mempll_freq(struct ddr_info_driver *ddr_driver)
{
    unsigned int i;
    unsigned int u4value;
    unsigned int freq = 0;

    u4value = readl(ddr_driver->reg_base.ddrphy_base + 0x600);
    for(i=0; i<ARRAY_SIZE(mempll_tbl); i++)
    {
        if(u4value == mempll_tbl[i].MEMPLL_N_INFO)
        {
            freq = mempll_tbl[i].freq;
            ddr_info_warn("MEMPLL_N_INFO: 0x%x, Feq:%d\n", u4value, freq);
            break;
        }
    }

    return freq;
}
#endif


static ssize_t mempll_show(struct device_driver *driver, char *buf)
{
	unsigned int mempll_freq;
    struct ddr_info_driver * ddr_driver;

    ddr_driver = container_of(driver, struct ddr_info_driver, driver);
    mempll_freq = get_mempll_freq(ddr_driver);

    sprintf(buf, "%d\n", mempll_freq);
      
    return strlen(buf);
}

DRIVER_ATTR(mempll_info, 0444, mempll_show, NULL);

/**************************************************************************
*STATIC FUNCTION
**************************************************************************/
#ifdef CONFIG_OF
static int ddr_reg_of_iomap(struct ddr_reg_base *reg_base)
{
    struct device_node *node = NULL;
    /*IO remap*/
    node = of_find_compatible_node(NULL, NULL, "mediatek,DDRPHY");
    if (node) {
       reg_base->ddrphy_base = of_iomap(node, 0);
       
       ddr_info_warn("DDRPHY ADDRESS %p\n", reg_base->ddrphy_base);
    } else {
       ddr_info_warn("can't find DDRPHY compatible node \n");
       return -1;
    }

#if 0
    node = of_find_compatible_node(NULL, NULL, "mediatek,DRAMC0");
    if (node) {
       reg_base->dramc0_base = of_iomap(node, 0);
       ddr_info_warn("DRAMC0 ADDRESS %p,\n", reg_base->dramc0_base);
    } else {
       ddr_info_warn("can't find DRAMC0 compatible node \n");
       return -1;
    }

    node = of_find_compatible_node(NULL, NULL, "mediatek,DRAMC_NAO");
    if (node) {
       reg_base->dramc_nao_base = of_iomap(node, 0);
       ddr_info_warn("DRAMC_NAO ADDRESS %p,\n", reg_base->dramc_nao_base);
    } else {
       ddr_info_warn("can't find DRAMC_NAO compatible node \n");
       return -1;
    }

    node = of_find_compatible_node(NULL, NULL, "mediatek,APMIXED");
    if (node) {
       reg_base->apmixed_base= of_iomap(node, 0);
       ddr_info_warn("APMIXED ADDRESS %p,\n", reg_base->apmixed_base);
    } else {
       ddr_info_warn("can't find APMIXED compatible node \n");
       return -1;
    }
#endif

    return 0;
}
#endif

static int __init ddr_info_init(void)
{
	int ret = 0;
    ddr_info_dbg("module init\n");

#ifdef CONFIG_OF
    ddr_reg_of_iomap(&ddr_info_driver.reg_base);
#else
    ddr_info_driver.reg_base.ddrphy_base = DDRPHY_BASE;
    ddr_info_driver.reg_base.dramc0_base = DRAMC0_BASE;
    ddr_info_driver.reg_base.dramc_nao_base = DRAMC_NAO_BASE;
    ddr_info_driver.reg_base.apmixed_base = APMIXEDSYS_BASE;
#endif

	/* register driver and create sysfs files */
	ret = driver_register(&ddr_info_driver.driver);
	if (ret) {
		ddr_info_warn("fail to register ddr_info driver\n");
        return -1;
	}

	ret = driver_create_file(&ddr_info_driver.driver, &driver_attr_mempll_info);
    if (ret) {
		ddr_info_warn("fail to create mempll_info sysfs file\n");
        driver_unregister(&ddr_info_driver.driver);
        return -1;
	}

	return 0;
}

static void __exit ddr_info_exit(void)
{
    ddr_info_dbg("module exit\n");
    driver_unregister(&ddr_info_driver.driver);
}

module_init(ddr_info_init);
module_exit(ddr_info_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Mediatek DDR info");
MODULE_AUTHOR("Quanwen Tan <quanwen.tan@mediatek.com>");

