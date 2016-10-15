#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <mach/emi_mpu.h>
#include <mach/sync_write.h>
#include <mach/memory.h>
#include <mach/upmu_sw.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mach/mt_ccci_common.h>
#include "ccci_config.h"
#include "ccci_core.h"
#include "ccci_debug.h"
#include "ccci_bm.h"
#include "ccci_platform.h"

#define TAG "plat"

static int is_4g_memory_size_support(void)
{
#ifdef FEATURE_USING_4G_MEMORY_API
	return enable_4G();
#else
	return 0;
#endif
}

/* =================================================== */
/* MPU Region defination */
/* =================================================== */
#define MPU_REGION_ID_SEC_OS        0
#define MPU_REGION_ID_MD32          1
#define MPU_REGION_ID_MD32_SMEM     2
#define MPU_REGION_ID_MD1_SEC_SMEM  3
#define MPU_REGION_ID_MD2_SEC_SMEM  4
#define MPU_REGION_ID_MD1_ROM       5
#define MPU_REGION_ID_MD1_DSP       6
#define MPU_REGION_ID_MD2_ROM       7
#define MPU_REGION_ID_MD2_RW        8
#define MPU_REGION_ID_MD1_SMEM      9
#define MPU_REGION_ID_MD2_SMEM      10
#define MPU_REGION_ID_MD1_RW        14
#define MPU_REGION_ID_AP            15

unsigned long infra_ao_base;
unsigned long dbgapb_base;
/* -- MD1 Bank 0 */
#define MD1_BANK0_MAP0 ((unsigned int *)(infra_ao_base+0x300))
#define MD1_BANK0_MAP1 ((unsigned int *)(infra_ao_base+0x304))
/* -- MD1 Bank 4 */
#define MD1_BANK4_MAP0 ((unsigned int *)(infra_ao_base+0x308))
#define MD1_BANK4_MAP1 ((unsigned int *)(infra_ao_base+0x30C))

/* -- MD2 Bank 0 */
#define MD2_BANK0_MAP0 ((unsigned int *)(infra_ao_base+0x310))
#define MD2_BANK0_MAP1 ((unsigned int *)(infra_ao_base+0x314))
/* -- MD2 Bank 4 */
#define MD2_BANK4_MAP0 ((unsigned int *)(infra_ao_base+0x318))
#define MD2_BANK4_MAP1 ((unsigned int *)(infra_ao_base+0x31C))

void ccci_clear_md_region_protection(struct ccci_modem *md)
{
#ifdef ENABLE_EMI_PROTECTION
	unsigned int rom_mem_mpu_id, rw_mem_mpu_id;

	CCCI_INF_MSG(md->index, TAG, "Clear MD region protect...\n");
	switch (md->index) {
	case MD_SYS1:
		rom_mem_mpu_id = MPU_REGION_ID_MD1_ROM;
		rw_mem_mpu_id = MPU_REGION_ID_MD1_RW;
		break;
	case MD_SYS2:
		rom_mem_mpu_id = MPU_REGION_ID_MD2_ROM;
		rw_mem_mpu_id = MPU_REGION_ID_MD2_RW;
		break;
	default:
		CCCI_INF_MSG(md->index, TAG, "[error]MD ID invalid when clear MPU protect\n");
		return;
	}

	CCCI_INF_MSG(md->index, TAG, "Clear MPU protect MD ROM region<%d>\n", rom_mem_mpu_id);
	emi_mpu_set_region_protection(0,	/*START_ADDR */
				      0,	/*END_ADDR */
				      rom_mem_mpu_id,	/*region */
				      SET_ACCESS_PERMISSON(NO_PROTECTION, NO_PROTECTION, NO_PROTECTION, NO_PROTECTION,
							   NO_PROTECTION, NO_PROTECTION, NO_PROTECTION, NO_PROTECTION));

	CCCI_INF_MSG(md->index, TAG, "Clear MPU protect MD R/W region<%d>\n", rw_mem_mpu_id);
	emi_mpu_set_region_protection(0,	/*START_ADDR */
				      0,	/*END_ADDR */
				      rw_mem_mpu_id,	/*region */
				      SET_ACCESS_PERMISSON(NO_PROTECTION, NO_PROTECTION, NO_PROTECTION, NO_PROTECTION,
							   NO_PROTECTION, NO_PROTECTION, NO_PROTECTION, NO_PROTECTION));
#endif
}

void ccci_clear_dsp_region_protection(struct ccci_modem *md)
{
#ifdef ENABLE_EMI_PROTECTION
	unsigned int dsp_mem_mpu_id;

	CCCI_INF_MSG(md->index, TAG, "Clear DSP region protect...\n");
	switch (md->index) {
	case MD_SYS1:
		dsp_mem_mpu_id = MPU_REGION_ID_MD1_DSP;
		break;
	default:
		CCCI_INF_MSG(md->index, TAG, "[error]MD ID invalid when clear MPU protect\n");
		return;
	}

	CCCI_INF_MSG(md->index, TAG, "Clear MPU protect DSP ROM region<%d>\n", dsp_mem_mpu_id);
	emi_mpu_set_region_protection(0,	/*START_ADDR */
				      0,	/*END_ADDR */
				      dsp_mem_mpu_id,	/*region */
				      SET_ACCESS_PERMISSON(NO_PROTECTION, NO_PROTECTION, NO_PROTECTION, NO_PROTECTION,
							   NO_PROTECTION, NO_PROTECTION, NO_PROTECTION, NO_PROTECTION));
#endif
}

/*
 * for some unkonw reason on 6582 and 6572, MD will read AP's memory during boot up, so we
 * set AP region as MD read-only at first, and re-set it to portected after MD boot up.
 * this function should be called right before sending runtime data.
 */
void ccci_set_ap_region_protection(struct ccci_modem *md)
{
#ifdef ENABLE_EMI_PROTECTION
	unsigned int ap_mem_mpu_id, ap_mem_mpu_attr;
	unsigned int kernel_base;
	unsigned int dram_size;

	if (is_4g_memory_size_support())
		kernel_base = 0;
	else
		kernel_base = get_phys_offset();
#ifdef ENABLE_DRAM_API
	dram_size = get_max_DRAM_size();
#else
	dram_size = 256 * 1024 * 1024;
#endif
	ap_mem_mpu_id = MPU_REGION_ID_AP;
	ap_mem_mpu_attr =
	    SET_ACCESS_PERMISSON(FORBIDDEN, NO_PROTECTION, FORBIDDEN, NO_PROTECTION, FORBIDDEN, FORBIDDEN, FORBIDDEN,
				 NO_PROTECTION);
	CCCI_INF_MSG(md->index, TAG, "MPU Start protect AP region<%d:%08x:%08x> %x\n", ap_mem_mpu_id, kernel_base,
		     (kernel_base + dram_size - 1), ap_mem_mpu_attr);
	emi_mpu_set_region_protection(kernel_base, (kernel_base + dram_size - 1), ap_mem_mpu_id, ap_mem_mpu_attr);

#endif
}
EXPORT_SYMBOL(ccci_set_ap_region_protection);

void ccci_set_dsp_region_protection(struct ccci_modem *md, int loaded)
{
#ifdef ENABLE_EMI_PROTECTION
	unsigned int dsp_mem_mpu_id, dsp_mem_mpu_attr;
	unsigned int dsp_mem_phy_start, dsp_mem_phy_end;
	struct ccci_image_info *img_info;

	switch (md->index) {
	case MD_SYS1:
		dsp_mem_mpu_id = MPU_REGION_ID_MD1_DSP;
		if (!loaded)
			dsp_mem_mpu_attr =
			    SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
						 SEC_R_NSEC_R, SEC_R_NSEC_R);
		else
			dsp_mem_mpu_attr =
			    SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
						 NO_PROTECTION, FORBIDDEN);
		break;

	default:
		CCCI_ERR_MSG(md->index, CORE, "[error]invalid when MPU protect\n");
		return;
	}

	dsp_mem_phy_start = (unsigned int)md->mem_layout.dsp_region_phy;
	dsp_mem_phy_end = ((dsp_mem_phy_start + md->mem_layout.dsp_region_size + 0xFFFF) & (~0xFFFF)) - 0x1;

	CCCI_INF_MSG(md->index, TAG, "MPU Start protect DSP region<%d:%08x:%08x> %x\n",
		     dsp_mem_mpu_id, dsp_mem_phy_start, dsp_mem_phy_end, dsp_mem_mpu_attr);
	emi_mpu_set_region_protection(dsp_mem_phy_start, dsp_mem_phy_end, dsp_mem_mpu_id, dsp_mem_mpu_attr);
#endif
}
EXPORT_SYMBOL(ccci_set_dsp_region_protection);

void ccci_set_mem_access_protection(struct ccci_modem *md)
{
#ifdef ENABLE_EMI_PROTECTION
	unsigned int shr_mem_phy_start, shr_mem_phy_end, shr_mem_mpu_id, shr_mem_mpu_attr;
	unsigned int rom_mem_phy_start, rom_mem_phy_end, rom_mem_mpu_id, rom_mem_mpu_attr;
	unsigned int rw_mem_phy_start, rw_mem_phy_end, rw_mem_mpu_id, rw_mem_mpu_attr;
	unsigned int ap_mem_mpu_id, ap_mem_mpu_attr;
	struct ccci_image_info *img_info;
	struct ccci_mem_layout *md_layout;
	unsigned int kernel_base;
	unsigned int dram_size;

	/* For MT6752 */
	/* Forbidden=>Fbd, No protect =>NP, Secure Read_none scrure read=>(S) */
	/*========================================================================================================== */
	/*           |  Region  |  D0(AP)  | D1(MD1) | D2(CONN) | D3(MD32) |   D4(MM) |  D5(MD2) | D6(MFG)  |  D7  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* Secure OS |     0    |  RW(S)   |   Fbd   |    Fbd   |   Fbd    |    Fbd   |    Fbd   |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD32 Code |     1    |  Fbd     |   Fbd   |    Fbd   |   RW(S)  |    Fbd   |    Fbd   |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD32 SMEM |     2    |  NP      |   Fbd   |    Fbd   |   NP     |    Fbd   |    Fbd   |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD1SecSMEM|     3    |  R/W(S)  |   NP    |    Fbd   |   Fbd    |    Fbd   |    Fbd   |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD2SecSMEM|     4    |  RW(S)   |   Fbd   |    Fbd   |   Fbd    |    Fbd   |    NP    |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD1 ROM   |     5    |  R(S)    |   R(S)  |    Fbd   |   Fbd    |    Fbd   |   Fbd    |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD1 DSP   |     6    |  Fbd     |   NP    |    Fbd   |   Fbd    |    Fbd   |   Fbd    |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD2 ROM   |     7    |  R(S)    |   Fbd   |    Fbd   |   Fbd    |    Fbd   |   R(S)   |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD2 R/W+  |     8    |  Fbd     |   Fbd   |    Fbd   |   Fbd    |    Fbd   |   NP     |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD1 SMEM  |     9    |  NP      |   NP    |    Fbd   |   Fbd    |    Fbd   |   NP     |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD2 SMEM  |    10    |  NP      |   NP    |    Fbd   |   Fbd    |    Fbd   |   NP     |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* MD1 R/W+  |    14    |  Fbd     |   NP    |    Fbd   |   Fbd    |    Fbd   |   Fbd    |   Fbd    | Fbd  | */
	/* ----------+---------------------------------------------------------------------------------------------- */
	/* AP        |    15    |  NP      |   Fbd   |    Fbd   |   NP     |    NP    |   Fbd    |   NP     | Fbd  | */
	/* ========================================================================================================= */
	switch (md->index) {
	case MD_SYS1:
		img_info = &md->img_info[IMG_MD];
		md_layout = &md->mem_layout;
		rom_mem_mpu_id = MPU_REGION_ID_MD1_ROM;
		rw_mem_mpu_id = MPU_REGION_ID_MD1_RW;
		shr_mem_mpu_id = MPU_REGION_ID_MD1_SMEM;
		rom_mem_mpu_attr =
		    SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, SEC_R_NSEC_R,
					 SEC_R_NSEC_R);
		rw_mem_mpu_attr =
		    SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
					 NO_PROTECTION, FORBIDDEN);
		shr_mem_mpu_attr =
		    SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, NO_PROTECTION, FORBIDDEN, FORBIDDEN, FORBIDDEN,
					 NO_PROTECTION, NO_PROTECTION);
		break;
	case MD_SYS2:
		img_info = &md->img_info[IMG_MD];
		md_layout = &md->mem_layout;
		rom_mem_mpu_id = MPU_REGION_ID_MD2_ROM;
		rw_mem_mpu_id = MPU_REGION_ID_MD2_RW;
		shr_mem_mpu_id = MPU_REGION_ID_MD2_SMEM;
		rom_mem_mpu_attr =
		    SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, SEC_R_NSEC_R, FORBIDDEN, FORBIDDEN, FORBIDDEN, FORBIDDEN,
					 SEC_R_NSEC_R);
		rw_mem_mpu_attr =
		    SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, NO_PROTECTION, FORBIDDEN, FORBIDDEN, FORBIDDEN,
					 FORBIDDEN, FORBIDDEN);
		shr_mem_mpu_attr =
		    SET_ACCESS_PERMISSON(FORBIDDEN, FORBIDDEN, NO_PROTECTION, FORBIDDEN, FORBIDDEN, FORBIDDEN,
					 NO_PROTECTION, NO_PROTECTION);
		break;

	default:
		CCCI_ERR_MSG(md->index, CORE, "[error]invalid when MPU protect\n");
		return;
	}

	if (is_4g_memory_size_support())
		kernel_base = 0;
	else
		kernel_base = get_phys_offset();
#ifdef ENABLE_DRAM_API
	dram_size = get_max_DRAM_size();
#else
	dram_size = 256 * 1024 * 1024;
#endif
	ap_mem_mpu_id = MPU_REGION_ID_AP;
	ap_mem_mpu_attr =
	    SET_ACCESS_PERMISSON(NO_PROTECTION, NO_PROTECTION, SEC_R_NSEC_R, NO_PROTECTION, NO_PROTECTION,
				 NO_PROTECTION, SEC_R_NSEC_R, NO_PROTECTION);

	/*
	 * if set start=0x0, end=0x10000, the actural protected area will be 0x0-0x1FFFF,
	 * here we use 64KB align, MPU actually request 32KB align since MT6582, but this works...
	 * we assume emi_mpu_set_region_protection will round end address down to 64KB align.
	 */
	rom_mem_phy_start = (unsigned int)md_layout->md_region_phy;
	rom_mem_phy_end = ((rom_mem_phy_start + img_info->size + 0xFFFF) & (~0xFFFF)) - 0x1;
	rw_mem_phy_start = rom_mem_phy_end + 0x1;
	rw_mem_phy_end = rom_mem_phy_start + md_layout->md_region_size - 0x1;
	shr_mem_phy_start = (unsigned int)md_layout->smem_region_phy;
	shr_mem_phy_end = ((shr_mem_phy_start + md_layout->smem_region_size + 0xFFFF) & (~0xFFFF)) - 0x1;

	CCCI_INF_MSG(md->index, TAG, "MPU Start protect MD ROM region<%d:%08x:%08x> %x\n",
		     rom_mem_mpu_id, rom_mem_phy_start, rom_mem_phy_end, rom_mem_mpu_attr);
	emi_mpu_set_region_protection(rom_mem_phy_start,	/*START_ADDR */
				      rom_mem_phy_end,	/*END_ADDR */
				      rom_mem_mpu_id,	/*region */
				      rom_mem_mpu_attr);

	CCCI_INF_MSG(md->index, TAG, "MPU Start protect MD R/W region<%d:%08x:%08x> %x\n",
		     rw_mem_mpu_id, rw_mem_phy_start, rw_mem_phy_end, rw_mem_mpu_attr);
	emi_mpu_set_region_protection(rw_mem_phy_start,	/*START_ADDR */
				      rw_mem_phy_end,	/*END_ADDR */
				      rw_mem_mpu_id,	/*region */
				      rw_mem_mpu_attr);

	CCCI_INF_MSG(md->index, TAG, "MPU Start protect MD Share region<%d:%08x:%08x> %x\n",
		     shr_mem_mpu_id, shr_mem_phy_start, shr_mem_phy_end, shr_mem_mpu_attr);
	emi_mpu_set_region_protection(shr_mem_phy_start,	/*START_ADDR */
				      shr_mem_phy_end,	/*END_ADDR */
				      shr_mem_mpu_id,	/*region */
				      shr_mem_mpu_attr);

/* This part need to move common part */
#if 1
	CCCI_INF_MSG(md->index, TAG, "MPU Start protect AP region<%d:%08x:%08x> %x\n",
		     ap_mem_mpu_id, kernel_base, (kernel_base + dram_size - 1), ap_mem_mpu_attr);
	emi_mpu_set_region_protection(kernel_base, (kernel_base + dram_size - 1), ap_mem_mpu_id, ap_mem_mpu_attr);
#endif
#endif
}
EXPORT_SYMBOL(ccci_set_mem_access_protection);

/* This function has phase out!!! */
int set_ap_smem_remap(struct ccci_modem *md, phys_addr_t src, phys_addr_t des)
{
	unsigned int remap1_val = 0;
	unsigned int remap2_val = 0;
	static int smem_remapped;

	if (!smem_remapped) {
		smem_remapped = 1;
		remap1_val = (((des >> 24) | 0x1) & 0xFF)
		    + (((INVALID_ADDR >> 16) | 1 << 8) & 0xFF00)
		    + (((INVALID_ADDR >> 8) | 1 << 16) & 0xFF0000)
		    + (((INVALID_ADDR >> 0) | 1 << 24) & 0xFF000000);

		remap2_val = (((INVALID_ADDR >> 24) | 0x1) & 0xFF)
		    + (((INVALID_ADDR >> 16) | 1 << 8) & 0xFF00)
		    + (((INVALID_ADDR >> 8) | 1 << 16) & 0xFF0000)
		    + (((INVALID_ADDR >> 0) | 1 << 24) & 0xFF000000);

		CCCI_INF_MSG(md->index, TAG, "AP Smem remap: [%llx]->[%llx](%08x:%08x)\n", (unsigned long long)des,
			     (unsigned long long)src, remap1_val, remap2_val);

#ifdef ENABLE_MEM_REMAP_HW
		mt_reg_sync_writel(remap1_val, AP_BANK4_MAP0);
		mt_reg_sync_writel(remap2_val, AP_BANK4_MAP1);
		mt_reg_sync_writel(remap2_val, AP_BANK4_MAP1);	/* HW bug, write twice to activate setting */
#endif
	}
	return 0;
}

int set_md_smem_remap(struct ccci_modem *md, phys_addr_t src, phys_addr_t des, phys_addr_t invalid)
{
	unsigned int remap1_val = 0;
	unsigned int remap2_val = 0;

	if (is_4g_memory_size_support())
		des &= 0xFFFFFFFF;
	else
		des -= KERN_EMI_BASE;

	switch (md->index) {
	case MD_SYS1:
		remap1_val = (((des >> 24) | 0x1) & 0xFF)
		    + ((((des + 0x2000000 * 1) >> 16) | 1 << 8) & 0xFF00)
		    + ((((des + 0x2000000 * 2) >> 8) | 1 << 16) & 0xFF0000)
		    + ((((des + 0x2000000 * 3) >> 0) | 1 << 24) & 0xFF000000);
		remap2_val = ((((des + 0x2000000 * 4) >> 24) | 0x1) & 0xFF)
		    + ((((des + 0x2000000 * 5) >> 16) | 1 << 8) & 0xFF00)
		    + ((((des + 0x2000000 * 6) >> 8) | 1 << 16) & 0xFF0000)
		    + ((((des + 0x2000000 * 7) >> 0) | 1 << 24) & 0xFF000000);

#ifdef ENABLE_MEM_REMAP_HW
		mt_reg_sync_writel(remap1_val, MD1_BANK4_MAP0);
		mt_reg_sync_writel(remap2_val, MD1_BANK4_MAP1);
#endif
		break;
	case MD_SYS2:
		remap1_val = (((des >> 24) | 0x1) & 0xFF)
		    + ((((des + 0x2000000 * 1) >> 16) | 1 << 8) & 0xFF00)
		    + ((((des + 0x2000000 * 2) >> 8) | 1 << 16) & 0xFF0000)
		    + ((((des + 0x2000000 * 3) >> 0) | 1 << 24) & 0xFF000000);
		remap2_val = ((((des + 0x2000000 * 4) >> 24) | 0x1) & 0xFF)
		    + ((((des + 0x2000000 * 5) >> 16) | 1 << 8) & 0xFF00)
		    + ((((des + 0x2000000 * 6) >> 8) | 1 << 16) & 0xFF0000)
		    + ((((des + 0x2000000 * 7) >> 0) | 1 << 24) & 0xFF000000);

#ifdef ENABLE_MEM_REMAP_HW
		mt_reg_sync_writel(remap1_val, MD2_BANK4_MAP0);
		mt_reg_sync_writel(remap2_val, MD2_BANK4_MAP1);
#endif
		break;
	default:
		break;
	}

	CCCI_INF_MSG(md->index, TAG, "MD Smem remap:[%llx]->[%llx](%08x:%08x)\n", (unsigned long long)des,
		     (unsigned long long)src, remap1_val, remap2_val);
	return 0;
}

int set_md_rom_rw_mem_remap(struct ccci_modem *md, phys_addr_t src, phys_addr_t des, phys_addr_t invalid)
{
	unsigned int remap1_val = 0;
	unsigned int remap2_val = 0;

	if (is_4g_memory_size_support())
		des &= 0xFFFFFFFF;
	else
		des -= KERN_EMI_BASE;

	switch (md->index) {
	case MD_SYS1:
		remap1_val = (((des >> 24) | 0x1) & 0xFF)
		    + ((((des + 0x2000000 * 1) >> 16) | 1 << 8) & 0xFF00)
		    + ((((des + 0x2000000 * 2) >> 8) | 1 << 16) & 0xFF0000)
		    + ((((des + 0x2000000 * 3) >> 0) | 1 << 24) & 0xFF000000);
		remap2_val = ((((des + 0x2000000 * 4) >> 24) | 0x1) & 0xFF)
		    + ((((des + 0x2000000 * 5) >> 16) | 1 << 8) & 0xFF00)
		    + ((((des + 0x2000000 * 6) >> 8) | 1 << 16) & 0xFF0000)
		    + ((((des + 0x2000000 * 7) >> 0) | 1 << 24) & 0xFF000000);

#ifdef ENABLE_MEM_REMAP_HW
		mt_reg_sync_writel(remap1_val, MD1_BANK0_MAP0);
		mt_reg_sync_writel(remap2_val, MD1_BANK0_MAP1);
#endif
		break;
	case MD_SYS2:
		remap1_val = (((des >> 24) | 0x1) & 0xFF)
		    + ((((des + 0x2000000 * 1) >> 16) | 1 << 8) & 0xFF00)
		    + ((((des + 0x2000000 * 2) >> 8) | 1 << 16) & 0xFF0000)
		    + ((((des + 0x2000000 * 3) >> 0) | 1 << 24) & 0xFF000000);
		remap2_val = ((((des + 0x2000000 * 4) >> 24) | 0x1) & 0xFF)
		    + ((((des + 0x2000000 * 5) >> 16) | 1 << 8) & 0xFF00)
		    + ((((des + 0x2000000 * 6) >> 8) | 1 << 16) & 0xFF0000)
		    + ((((des + 0x2000000 * 7) >> 0) | 1 << 24) & 0xFF000000);

#ifdef ENABLE_MEM_REMAP_HW
		mt_reg_sync_writel(remap1_val, MD2_BANK0_MAP0);
		mt_reg_sync_writel(remap2_val, MD2_BANK0_MAP1);
#endif
		break;
	default:
		break;
	}

	CCCI_INF_MSG(md->index, TAG, "MD ROM mem remap:[%llx]->[%llx](%08x:%08x)\n", (unsigned long long)des,
		     (unsigned long long)src, remap1_val, remap2_val);
	return 0;
}

void ccci_set_mem_remap(struct ccci_modem *md, unsigned long smem_offset, phys_addr_t invalid)
{
	unsigned long remainder;

	if (is_4g_memory_size_support()) {
		invalid &= 0xFFFFFFFF;
		CCCI_INF_MSG(md->index, TAG, "4GB mode enabled, invalid_map=%llx\n", (unsigned long long)invalid);
	} else {
		invalid -= KERN_EMI_BASE;
		CCCI_INF_MSG(md->index, TAG, "4GB mode disabled, invalid_map=%llx\n", (unsigned long long)invalid);
	}

	/* Set share memory remapping */
#if 0				/* no hardware AP remap after MT6592 */
	set_ap_smem_remap(md, 0x40000000, md->mem_layout.smem_region_phy_before_map);
	md->mem_layout.smem_region_phy = smem_offset + 0x40000000;
#endif
	/*
	 * always remap only the 1 slot where share memory locates. smem_offset is the offset between
	 * ROM start address(32M align) and share memory start address.
	 * (AP view smem address) - [(smem_region_phy) - (bank4 start address) - (un-32M-align space)]
	 * = (MD view smem address)
	 */
	remainder = smem_offset % 0x02000000;
	md->mem_layout.smem_offset_AP_to_MD = md->mem_layout.smem_region_phy - (remainder + 0x40000000);
	set_md_smem_remap(md, 0x40000000, md->mem_layout.md_region_phy + (smem_offset - remainder), invalid);
	CCCI_INF_MSG(md->index, TAG, "AP to MD share memory offset 0x%X", md->mem_layout.smem_offset_AP_to_MD);

	/* Set md image and rw runtime memory remapping */
	set_md_rom_rw_mem_remap(md, 0x00000000, md->mem_layout.md_region_phy, invalid);
}

/*
 * when MD attached its codeviser for debuging, this bit will be set. so CCCI should disable some
 * checkings and operations as MD may not respond to us.
 */
unsigned int ccci_get_md_debug_mode(struct ccci_modem *md)
{
	unsigned int dbg_spare;
	static unsigned int debug_setting_flag;
	/* this function does NOT distinguish modem ID, may be a risk point */
	if ((debug_setting_flag & DBG_FLAG_JTAG) == 0) {
		dbg_spare = ioread32((void __iomem *)(dbgapb_base + 0x10));
		CCCI_INF_MSG(md->index, TAG, "dbgapb_base value=0x%x\n", dbg_spare);
		if (dbg_spare & MD_DBG_JTAG_BIT) {
			CCCI_INF_MSG(md->index, TAG, "Jtag Debug mode(%08x)\n", dbg_spare);
			debug_setting_flag |= DBG_FLAG_JTAG;
			mt_reg_sync_writel(dbg_spare & (~MD_DBG_JTAG_BIT), (dbgapb_base + 0x10));
		}
	}
	return debug_setting_flag;
}
EXPORT_SYMBOL(ccci_get_md_debug_mode);

void ccci_get_platform_version(char *ver)
{
#ifdef ENABLE_CHIP_VER_CHECK
	sprintf(ver, "MT%04x_S%02x", get_chip_hw_ver_code(), (get_chip_hw_subcode() & 0xFF));
#else
	sprintf(ver, "MT6595_S00");
#endif
}

static int ccci_md_low_power_notify(struct ccci_modem *md, LOW_POEWR_NOTIFY_TYPE type, int level)
{
#ifdef FEATURE_LOW_BATTERY_SUPPORT
	unsigned int reserve = 0xFFFFFFFF;
	int ret = 0;

	CCCI_INF_MSG(md->index, TAG, "low power notification type=%d, level=%d\n", type, level);
	/*
	 * byte3 byte2 byte1 byte0
	 *    0   4G   3G   2G
	 */
	switch (type) {
	case LOW_BATTERY:
		if (level == LOW_BATTERY_LEVEL_0)
			reserve = 0;	/* 0 */
		else if (level == LOW_BATTERY_LEVEL_1 || level == LOW_BATTERY_LEVEL_2)
			reserve = (1 << 6);	/* 64 */
		ret = ccci_send_msg_to_md(md, CCCI_SYSTEM_TX, MD_LOW_BATTERY_LEVEL, reserve, 1);
		if (ret)
			CCCI_ERR_MSG(md->index, TAG, "send low battery notification fail, ret=%d\n", ret);
		break;
	case BATTERY_PERCENT:
		if (level == BATTERY_PERCENT_LEVEL_0)
			reserve = 0;	/* 0 */
		else if (level == BATTERY_PERCENT_LEVEL_1)
			reserve = (1 << 6);	/* 64 */
		ret = ccci_send_msg_to_md(md, CCCI_SYSTEM_TX, MD_LOW_BATTERY_LEVEL, reserve, 1);
		if (ret)
			CCCI_ERR_MSG(md->index, TAG, "send battery percent notification fail, ret=%d\n", ret);
		break;
	default:
		break;
	};

	return ret;
#endif
	return 0;
}

#ifdef FEATURE_LOW_BATTERY_SUPPORT
static void ccci_md_low_battery_cb(LOW_BATTERY_LEVEL level)
{
	int idx = 0;
	struct ccci_modem *md;

	for (idx = 0; idx < MAX_MD_NUM; idx++) {
		md = ccci_get_modem_by_id(idx);
		if (md != NULL)
			ccci_md_low_power_notify(md, LOW_BATTERY, level);
	}
}

static void ccci_md_battery_percent_cb(BATTERY_PERCENT_LEVEL level)
{
	int idx = 0;
	struct ccci_modem *md;

	for (idx = 0; idx < MAX_MD_NUM; idx++) {
		md = ccci_get_modem_by_id(idx);
		if (md != NULL)
			ccci_md_low_power_notify(md, BATTERY_PERCENT, level);
	}
}
#endif

int ccci_platform_init(struct ccci_modem *md)
{
	static int init;
	unsigned int reg_value;
	struct device_node *node;

	if (init == 0) {
		init = 1;
#ifdef FEATURE_VLTE_SUPPORT
		reg_value = get_devinfo_with_index(5);	/* 0x10206048 */
		CCCI_INF_MSG(md->index, TAG, "ccci_platform_init:0x10206048=0x%x, bit(31,30)=%d\n", reg_value,
			     ((reg_value >> 30) & 0x3));
		reg_value = ((reg_value >> 30) & 0x3);	/* b'[31:30] */
		switch (reg_value) {
		case 0:
			CCCI_INF_MSG(md->index, TAG, "ccci_platform_init:set VLTE=1.0v\n");
			pmic_config_interface(0x63C, 0x0040, 0x7F, 0);
			pmic_config_interface(0x63E, 0x0040, 0x7F, 0);
			break;
		case 1:
			CCCI_INF_MSG(md->index, TAG, "ccci_platform_init:set VLTE=0.9625v\n");
			pmic_config_interface(0x63C, 0x003A, 0x7F, 0);
			pmic_config_interface(0x63E, 0x003A, 0x7F, 0);
			break;
		case 2:
			CCCI_INF_MSG(md->index, TAG, "ccci_platform_init:set VLTE=0.9375v\n");
			pmic_config_interface(0x63C, 0x0036, 0x7F, 0);
			pmic_config_interface(0x63E, 0x0036, 0x7F, 0);

			break;
		case 3:
			CCCI_INF_MSG(md->index, TAG, "ccci_platform_init:set VLTE=0.9125v\n");
			pmic_config_interface(0x63C, 0x0032, 0x7F, 0);
			pmic_config_interface(0x63E, 0x0032, 0x7F, 0);
			break;
		default:
			CCCI_INF_MSG(md->index, TAG, "ccci_platform_init: not set VLTE, reg_value=%d\n", reg_value);
			break;
		}

#endif

#if (defined(CCCI_SMT_SETTING) || defined(CONFIG_MTK_ENABLE_MD2))
		CCCI_INF_MSG(md->index, TAG, "ccci_platform_init:set VTCXO_1 on,bit3=1\n");
		pmic_config_interface(0x0A02, 0x1, 0x1, 3);	/* bit:3: =>b1 */
		CCCI_INF_MSG(md->index, TAG, "ccci_platform_init:set SRCLK_EN_SEL on,bit(13,12)=(0,1)\n");
		pmic_config_interface(0x0A02, 0x1, 0x3, 12);	/* bit:13:12=>b01 */
#endif
#ifdef CONFIG_MTK_ENABLE_MD2
		/* reg_value = *((volatile unsigned int*)0x10001F00); */
		reg_value = ccci_read32(infra_ao_base, 0xF00);
		reg_value &= ~(0x1E000000);
		reg_value |= 0x12000000;
		/* *((volatile unsigned int*)0x10001F00) = reg_value; */
		ccci_write32(infra_ao_base, 0xF00, reg_value);
		CCCI_INF_MSG(md->index, TAG,
			     "ccci_platform_init: md2 enable, set SRCLKEN infra_misc(0x1000_1F00), bit(28,27,26,25)=0x%x\n",
			     (ccci_read32(infra_ao_base, 0xF00) & 0x1E000000));

		/* reg_value = *((volatile unsigned int*)0x10001F08); */
		reg_value = ccci_read32(infra_ao_base, 0xF08);
		reg_value &= ~(0x001F0078);
		reg_value |= 0x001B0048;
		/* *((volatile unsigned int*)0x10001F00) = reg_value; */
		ccci_write32(infra_ao_base, 0xF08, reg_value);
		CCCI_INF_MSG(md->index, TAG,
			     "ccci_platform_init:set PLL misc_config(0x1000_1F08), bit(20,19,18,17,16,6,5,4,3)=0x%x\n",
			     (ccci_read32(infra_ao_base, 0xF08) & 0x001F0078));
#else
		/* reg_value = *((volatile unsigned int*)0x10001F00); */
		reg_value = ccci_read32(infra_ao_base, 0xF00);
		reg_value &= ~(0x1E000000);
		reg_value |= 0x0A000000;
		/* *((volatile unsigned int*)0x10001F00) = reg_value; */
		ccci_write32(infra_ao_base, 0xF00, reg_value);
		CCCI_INF_MSG(md->index, TAG,
			     "ccci_platform_init: md2 disable, set SRCLKEN infra_misc(0x1000_1F00), bit(28,27,26,25)=0x%x\n",
			     (ccci_read32(infra_ao_base, 0xF00) & 0x1E000000));
		/* reg_value = *((volatile unsigned int*)0x10001F08); */
		reg_value = ccci_read32(infra_ao_base, 0xF08);
		reg_value &= ~(0x001F0078);
		reg_value |= 0x00000000;
		/* *((volatile unsigned int*)0x10001F00) = reg_value; */
		ccci_write32(infra_ao_base, 0xF08, reg_value);
		CCCI_INF_MSG(md->index, TAG,
			     "ccci_platform_init: set PLL misc_config(0x1000_1F08), bit(20,19,18,17,16,6,5,4,3)=0x%x\n",
			     (ccci_read32(infra_ao_base, 0xF08) & 0x001F0078));
#endif
	}

	return 0;
}

int ccci_plat_common_init(void)
{
	struct device_node *node;
	/* Get infra cfg ao base */
	node = of_find_compatible_node(NULL, NULL, "mediatek,INFRACFG_AO");
	infra_ao_base = (unsigned long)of_iomap(node, 0);
	CCCI_INF_MSG(-1, TAG, "infra_ao_base:0x%p\n", (void *)infra_ao_base);
	node = of_find_compatible_node(NULL, NULL, "mediatek,DBGAPB_BASE");
	dbgapb_base = of_iomap(node, 0);
	CCCI_INF_MSG(-1, TAG, "dbgapb_base:%pa\n", &dbgapb_base);
#ifdef FEATURE_LOW_BATTERY_SUPPORT
	register_low_battery_notify(&ccci_md_low_battery_cb, LOW_BATTERY_PRIO_MD);
	register_battery_percent_notify(&ccci_md_battery_percent_cb, BATTERY_PERCENT_PRIO_MD);
#endif
	return 0;
}
