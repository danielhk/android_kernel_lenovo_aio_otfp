#define LOG_TAG "DEBUG"

#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/aee.h>
#include <linux/disp_assert_layer.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/time.h>

#include <mach/mt_clkmgr.h>
#include <mach/m4u.h>
#include <mach/m4u_port.h>

#include "cmdq_def.h"
#include "cmdq_record.h"
#include "cmdq_reg.h"
#include "cmdq_core.h"

#include "disp_drv_ddp.h"

#include "ddp_debug.h"
#include "ddp_reg.h"
#include "ddp_drv.h"
#include "ddp_wdma.h"
#include "ddp_hal.h"
#include "ddp_path.h"
#include "ddp_aal.h"
#include "ddp_pwm.h"
#include "ddp_info.h"
#include "ddp_dsi.h"
#include "ddp_ovl.h"

#include "ddp_manager.h"
#include "ddp_log.h"
#include "ddp_met.h"
#include "display_recorder.h"
#include "disp_session.h"
#include "primary_display.h"

#pragma GCC optimize("O0")

#define ddp_aee_print(string, args...) do {\
	char ddp_name[100];\
	snprintf(ddp_name, 100, "[DDP]"string, ##args); \
	aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_MMPROFILE_BUFFER, ddp_name, "[DDP] error"string, ##args);  \
	pr_err("DDP""error: "string, ##args);  \
} while (0)

/* --------------------------------------------------------------------------- */
/* External variable declarations */
/* --------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------- */
/* Debug Options */
/* --------------------------------------------------------------------------- */

/*extern void disp_set_sodi(unsigned int enable, void *cmdq_handle);
extern void DSI_LFR_UPDATE(DISP_MODULE_ENUM module, cmdqRecHandle cmdq);
extern void DSI_Set_LFR(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int mode,
			unsigned int type, unsigned int enable, unsigned int skip_num);*/
static struct dentry *debugfs;
static struct dentry *debugDir;

static struct dentry *debugfs_dump;

static const long int DEFAULT_LOG_FPS_WND_SIZE = 30;
static int debug_init;

unsigned char pq_debug_flag = 0;
unsigned char aal_debug_flag = 0;

static unsigned int dbg_log_level;
static unsigned int irq_log_level;
static unsigned int dump_to_buffer;

unsigned int gOVLBackground = 0xFF000000;
unsigned int gUltraEnable = 1;
unsigned int gDumpMemoutCmdq = 0;
unsigned int gEnableUnderflowAEE = 0;

unsigned int disp_low_power_enlarge_blanking = 0;
unsigned int disp_low_power_disable_ddp_clock = 0;
unsigned int disp_low_power_disable_fence_thread = 0;
unsigned int disp_low_power_remove_ovl = 1;
unsigned int disp_low_power_lfr = 0;
unsigned int gSkipIdleDetect = 0;
unsigned int gDumpClockStatus = 1;
#ifdef DISP_ENABLE_SODI_FOR_VIDEO_MODE
unsigned int gEnableSODIControl = 1;
  /* workaround for SVP IT, todo: please K fix it */
#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT) && defined(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
unsigned int gPrefetchControl = 0;
#else
unsigned int gPrefetchControl = 1;
#endif
#else
unsigned int gEnableSODIControl = 0;
unsigned int gPrefetchControl = 0;
#endif

/* enable it when use UART to grab log */
unsigned int gEnableUartLog = 0;
/* mutex SOF at raing edge of vsync, can save more time for cmdq config */
unsigned int gEnableMutexRisingEdge = 0;
/* only write dirty register, reduce register number write by cmdq */
unsigned int gEnableReduceRegWrite = 0;

unsigned int gDumpConfigCMD = 0;
unsigned int gDumpESDCMD = 0;

unsigned int gESDEnableSODI = 1;
unsigned int gEnableOVLStatusCheck = 1;

unsigned int gResetRDMAEnable = 1;
unsigned int gEnableSWTrigger = 0;

unsigned int gResetOVLInAALTrigger = 0;
unsigned int gDisableOVLTF = 0;

unsigned int gRDMAUltraSetting = 0x1b013bea;	/* so we can modify RDMA ultra at run-time */
unsigned int gRDMAUltraSetting_Directlink = 0x1b013bea;
unsigned int gRDMAUltraSetting_Decouple = 0x0e014570;
unsigned int gRDMAFIFOLen = 32;
unsigned int gEnableSODIWhenIdle = 1;
unsigned int gDisableIRQWhenIdle = 1;

#ifdef _MTK_USER_
unsigned int gEnableIRQ = 0;
/* #error eng_error */
#else
unsigned int gEnableIRQ = 1;
/* #error user_error */
#endif
unsigned int gDisableSODIForTriggerLoop = 1;
unsigned int gEnableCMDQProfile = 0;

unsigned int gChangeRDMAThreshold = 0;
unsigned int gIssueRequestThreshold = 1;

unsigned int gChangeMMClock = 0;
unsigned int gDecouplePQWithRDMA = 1;

unsigned int gRDMARequestThreshold = 0x10;
unsigned int gCMDQDump = 0;

#ifdef DISP_ENABLE_LAYER_FRAME
unsigned int gAddFrame = 1;
#else
unsigned int gAddFrame = 0;
#endif

static char STR_HELP[] =
	"USAGE:\n"
	"       echo [ACTION]>/d/dispsys\n"
	"ACTION:\n"
	"       regr:addr\n              :regr:0xf400c000\n"
	"       regw:addr,value          :regw:0xf400c000,0x1\n"
	"       dbg_log:0|1|2            :0 off, 1 dbg, 2 all\n"
	"       irq_log:0|1              :0 off, !0 on\n"
	"       met_on:[0|1],[0|1],[0|1] :fist[0|1]on|off,other [0|1]direct|decouple\n"
	"       backlight:level\n"
	"       dump_aal:arg\n"
	"       mmp\n"
	"       dump_reg:moduleID\n" "       dump_path:mutexID\n"
	"       dpfd_ut1:channel\n";
/* --------------------------------------------------------------------------- */
/* Command Processor */
/* --------------------------------------------------------------------------- */
static char dbg_buf[2048];
static unsigned int is_reg_addr_valid(unsigned int isVa, unsigned long addr)
{
	unsigned int i = 0;

	for (i = 0; i < DISP_REG_NUM; i++) {
		if ((isVa == 1) && (addr >= dispsys_reg[i])
		    && (addr <= dispsys_reg[i] + 0x1000))
			break;
		if ((isVa == 0) && (addr >= ddp_reg_pa_base[i])
		    && (addr <= ddp_reg_pa_base[i] + 0x1000))
			break;
	}

	if (i < DISP_REG_NUM) {
		DDPMSG("addr valid, isVa=0x%x, addr=0x%lx, module=%s!\n", isVa,
		       addr, ddp_get_reg_module_name(i));
		return 1;
	} else {
		DDPERR
		    ("is_reg_addr_valid return fail, isVa=0x%x, addr=0x%lx!\n",
		     isVa, addr);
		return 0;
	}

}

/* default MM_Clock is 364MHz */
#define REG_PA_VENC_PLL_CON0 0x1000c260
#define REG_PA_VENC_PLL_CON1 0x1000c264
#define REG_PA_VENC_PLL_PWR_CON0 0x1000c26c

#define DISP_MMCLK_156MHZ_CON0 0x131
#define DISP_MMCLK_156MHZ_CON1 0x800C0000
#define DISP_MMCLK_182MHZ_CON0 0x131
#define DISP_MMCLK_182MHZ_CON1 0x800E0000
#define DISP_MMCLK_364MHZ_CON0 0x121
#define DISP_MMCLK_364MHZ_CON1 0x800E0000
#define DISP_CLOCK_USER_NAME "display"
void disp_set_pll(unsigned int freq)
{
	unsigned long reg_va_con0 = 0;
	unsigned long reg_va_con1 = 0;
	static unsigned int freq_last = 364;
	static unsigned int pll_cnt;

	if (freq == freq_last)
		return;

	freq_last = freq;

	reg_va_con0 =
	    (unsigned long)ioremap_nocache(REG_PA_VENC_PLL_CON0,
					   sizeof(unsigned long));
	reg_va_con1 =
	    (unsigned long)ioremap_nocache(REG_PA_VENC_PLL_CON1,
					   sizeof(unsigned long));

	pr_debug
	    ("disp_set_pll(%d), before set, con0=0x%x, con1=0x%x, 0x%lx, 0x%lx\n",
	     freq, DISP_REG_GET(reg_va_con0), DISP_REG_GET(reg_va_con1),
	     reg_va_con0, reg_va_con1);

	if (freq == 156) {
		enable_pll(VENCPLL, DISP_CLOCK_USER_NAME);
		DISP_CPU_REG_SET(reg_va_con0, DISP_MMCLK_156MHZ_CON0);
		DISP_CPU_REG_SET(reg_va_con1, DISP_MMCLK_156MHZ_CON1);
		clkmux_sel(MT_MUX_MM, 3, DISP_CLOCK_USER_NAME);
		pll_cnt++;
	} else if (freq == 182) {
		enable_pll(VENCPLL, DISP_CLOCK_USER_NAME);
		DISP_CPU_REG_SET(reg_va_con0, DISP_MMCLK_182MHZ_CON0);
		DISP_CPU_REG_SET(reg_va_con1, DISP_MMCLK_182MHZ_CON1);
		clkmux_sel(MT_MUX_MM, 3, DISP_CLOCK_USER_NAME);
		pll_cnt++;
	} else if (freq == 364) {
		clkmux_sel(MT_MUX_MM, 1, DISP_CLOCK_USER_NAME);

		if (pll_cnt != 0) {
			disable_pll(VENCPLL, DISP_CLOCK_USER_NAME);
			pll_cnt--;
		}
	} else {
		pr_debug("disp_set_pll, error, invalid freq=%d\n", freq);
	}

	pr_debug("disp_set_pll(%d), after set, con0=0x%x, con1=0x%x\n",
		 freq, DISP_REG_GET(reg_va_con0), DISP_REG_GET(reg_va_con1));

	iounmap((void *)reg_va_con0);
	iounmap((void *)reg_va_con1);

}

unsigned int disp_set_pll_by_cmdq(unsigned int freq, void *cmdq_handle)
{
	unsigned long reg_va_con0 = 0;
	unsigned long reg_va_con1 = 0;
	static unsigned int freq_last = 364;
	static unsigned int pll_cnt;
	unsigned int i = 0;

	if (freq == freq_last)
		/* return; */

	freq_last = freq;

	reg_va_con0 =
	    (unsigned long)ioremap_nocache(REG_PA_VENC_PLL_CON0,
					   sizeof(unsigned long));
	reg_va_con1 =
	    (unsigned long)ioremap_nocache(REG_PA_VENC_PLL_CON1,
					   sizeof(unsigned long));

	pr_debug
	    ("disp_set_pll(%d), before set, con0=0x%x, con1=0x%x, 0x%lx, 0x%lx\n",
	     freq, DISP_REG_GET(reg_va_con0), DISP_REG_GET(reg_va_con1),
	     reg_va_con0, reg_va_con1);

	cmdqRecWaitNoClear(cmdq_handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);

	if (freq == 156) {
		enable_pll(VENCPLL, DISP_CLOCK_USER_NAME);
		DISP_CPU_REG_SET(reg_va_con0, DISP_MMCLK_156MHZ_CON0);
		DISP_CPU_REG_SET(reg_va_con1, DISP_MMCLK_156MHZ_CON1);
		pll_cnt++;

		/* set mux to 3 by CMDQ */
		DISP_REG_SET_PA(cmdq_handle, 0x10000048, 0x3000000);
		DISP_REG_SET_PA(cmdq_handle, 0x10000044, 0x3000000);
		DISP_REG_SET_PA(cmdq_handle, 0x10000004, 8);
	} else if (freq == 182) {
		/*NOT USE*/
	} else if (freq == 364) {
		/* set mux to 1 by CMDQ */
		DISP_REG_SET_PA(cmdq_handle, 0x10000048, 0x3000000);
		DISP_REG_SET_PA(cmdq_handle, 0x10000044, 0x1000000);
		DISP_REG_SET_PA(cmdq_handle, 0x10000004, 8);
	} else {
		pr_debug("disp_set_pll, error, invalid freq=%d\n", freq);
	}
	/* cmdqRecDumpCommand(cmdq_handle); */
	cmdqRecFlush(cmdq_handle);

	if (freq == 364) {
		if (pll_cnt != 0) {
			disable_pll(VENCPLL, DISP_CLOCK_USER_NAME);
			pll_cnt--;
		}
	}

	pr_debug("disp_set_pll(%d), after set, con0=0x%x, con1=0x%x\n",
		 freq, DISP_REG_GET(reg_va_con0), DISP_REG_GET(reg_va_con1));

	iounmap((void *)reg_va_con0);
	iounmap((void *)reg_va_con1);

	return 0;		/* cmdqRecEstimateEommandExecTime(cmdq_handle); */

}

static void disp_debug_api(unsigned int enable, char *buf)
{
	if (enable == 1) {
		DDPMSG("[DDP] debug=1, trigger AEE\n");
		/* aee_kernel_exception("DDP-TEST-ASSERT", "[DDP] DDP-TEST-ASSERT"); */
	} else if (enable == 2) {
		ddp_mem_test();
	} else if (enable == 3) {
		ddp_lcd_test();
	} else if (enable == 4) {
		DDPAEE("test enable=%d\n", enable);
		sprintf(buf, "test enable=%d\n", enable);
	} else if (enable == 5) {
		if (gDDPError == 0)
			gDDPError = 1;
		else
			gDDPError = 0;

		sprintf(buf, "bypass PQ: %d\n", gDDPError);
		DDPMSG("bypass PQ: %d\n", gDDPError);
	} else if (enable == 6) {
		/*ddp_dump_analysis(DISP_MODULE_DSI0);*/
		ddp_dump_reg(DISP_MODULE_DSI0);
	} else if (enable == 7) {
		if (dbg_log_level < 3)
			dbg_log_level++;
		else
			dbg_log_level = 0;

		pr_debug("DDP: dbg_log_level=%d\n", dbg_log_level);
		sprintf(buf, "dbg_log_level: %d\n", dbg_log_level);
	} else if (enable == 8) {
		DDPDUMP("clock_mm setting:%u\n",
			DISP_REG_GET(DISP_REG_CONFIG_C11));
		if (DISP_REG_GET(DISP_REG_CONFIG_C11) & 0xff000000 !=
		    0xff000000) {
			DDPDUMP
			    ("error, MM clock bit 24~bit31 should be 1, but real value=0x%x",
			     DISP_REG_GET(DISP_REG_CONFIG_C11));
		}
	} else if (enable == 9) {
		gOVLBackground = 0xFF0000FF;
		pr_debug("DDP: gOVLBackground=%d\n", gOVLBackground);
		sprintf(buf, "gOVLBackground: %d\n", gOVLBackground);
	} else if (enable == 10) {
		gOVLBackground = 0xFF000000;
		pr_debug("DDP: gOVLBackground=%d\n", gOVLBackground);
		sprintf(buf, "gOVLBackground: %d\n", gOVLBackground);
	} else if (enable == 11) {
		dispsys_irq[DISP_REG_NUM];
		ddp_irq_num[DISP_REG_NUM];
		unsigned int i = 0;
		char *buf_temp = buf;

		for (i = 0; i < DISP_REG_NUM; i++) {
			DDPDUMP
			    ("i=%d, module=%s, va=0x%lx, pa=0x%x, irq(%d,%d)\n",
			     i, ddp_get_reg_module_name(i), dispsys_reg[i],
			     ddp_reg_pa_base[i], dispsys_irq[i],
			     ddp_irq_num[i]);
			sprintf(buf_temp,
				"i=%d, module=%s, va=0x%lx, pa=0x%x, irq(%d,%d)\n",
				i, ddp_get_reg_module_name(i), dispsys_reg[i],
				ddp_reg_pa_base[i], dispsys_irq[i],
				ddp_irq_num[i]);
			buf_temp += strlen(buf_temp);
		}
	} else if (enable == 12) {
		if (gUltraEnable == 0)
			gUltraEnable = 1;
		else
			gUltraEnable = 0;

		pr_debug("DDP : gUltraEnable = %d\n", gUltraEnable);
		sprintf(buf, "gUltraEnable: %d\n", gUltraEnable);
	} else if (enable == 13) {
		/*int ovl_status = ovl_get_status();
		   config.type = DISP_SESSION_MEMORY;
		   config.device_id = 0;
		   disp_create_session(&config);
		   pr_debug("old status=%d, ovl1 status=%d\n", ovl_status, ovl_get_status());
		   sprintf(buf, "old status=%d, ovl1 status=%d\n", ovl_status, ovl_get_status()); */
	} else if (enable == 14) {
		/*int ovl_status = ovl_get_status();
		   disp_destroy_session(&config);
		   pr_debug("old status=%d, ovl1 status=%d\n", ovl_status, ovl_get_status());
		   sprintf(buf, "old status=%d, ovl1 status=%d\n", ovl_status, ovl_get_status()); */
	} else if (enable == 15) {
		/* extern smi_dumpDebugMsg(void); */
		ddp_dump_analysis(DISP_MODULE_CONFIG);
		ddp_dump_analysis(DISP_MODULE_RDMA0);
		ddp_dump_analysis(DISP_MODULE_OVL0);
		ddp_dump_analysis(DISP_MODULE_OVL1);

		/* dump ultra/preultra related regs */
	DDPMSG("wdma_con1(2c) = 0x%x, wdma_con2(0x38) = 0x%x, rdma_gmc0(30) = 0x%x",
			DISP_REG_GET(DISP_REG_WDMA_BUF_CON1), DISP_REG_GET(DISP_REG_WDMA_BUF_CON2),
			DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_0));
	DDPMSG(" rdma_gmc1(38) = 0x%x, fifo_con(40) = 0x%x\n ",
		     DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_1),
		     DISP_REG_GET(DISP_REG_RDMA_FIFO_CON));
		DDPMSG(" ovl0_gmc : 0x%x, 0x%x, 0x%x, 0x%x, ovl1_gmc : 0x%x, 0x%x, 0x%x, 0x%x\n ",
		       DISP_REG_GET(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING),
		       DISP_REG_GET(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING),
		       DISP_REG_GET(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING),
		       DISP_REG_GET(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING),
		       DISP_REG_GET(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING + DISP_OVL_INDEX_OFFSET),
		       DISP_REG_GET(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING + DISP_OVL_INDEX_OFFSET),
		       DISP_REG_GET(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING + DISP_OVL_INDEX_OFFSET),
		       DISP_REG_GET(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING + DISP_OVL_INDEX_OFFSET));

		/* dump smi regs */
		/* smi_dumpDebugMsg(); */

	} else if (enable == 16) {
		if (gDumpMemoutCmdq == 0)
			gDumpMemoutCmdq = 1;
		else
			gDumpMemoutCmdq = 0;

		pr_debug(" DDP : gDumpMemoutCmdq = %d\n ", gDumpMemoutCmdq);
		sprintf(buf, " gDumpMemoutCmdq :  %d\n ", gDumpMemoutCmdq);
	} else if (enable == 21) {
		if (gEnableSODIControl == 0)
			gEnableSODIControl = 1;
		else
			gEnableSODIControl = 0;

		pr_debug(" DDP : gEnableSODIControl = %d\n ", gEnableSODIControl);
		sprintf(buf, " gEnableSODIControl :  %d\n ", gEnableSODIControl);
	} else if (enable == 22) {
		if (gPrefetchControl == 0)
			gPrefetchControl = 1;
		else
			gPrefetchControl = 0;

		pr_debug(" DDP : gPrefetchControl = %d\n ", gPrefetchControl);
		sprintf(buf, " gPrefetchControl :  %d\n ", gPrefetchControl);
	} else if (enable == 23) {
		if (disp_low_power_enlarge_blanking == 0)
			disp_low_power_enlarge_blanking = 1;
		else
			disp_low_power_enlarge_blanking = 0;

		pr_debug(" DDP : disp_low_power_enlarge_blanking = %d\n ",
		       disp_low_power_enlarge_blanking);
		sprintf(buf, " disp_low_power_enlarge_blanking :  %d\n ",
			disp_low_power_enlarge_blanking);

	} else if (enable == 24) {
		if (disp_low_power_disable_ddp_clock == 0)
			disp_low_power_disable_ddp_clock = 1;
		else
			disp_low_power_disable_ddp_clock = 0;

		pr_debug(" DDP : disp_low_power_disable_ddp_clock = %d\n ",
		       disp_low_power_disable_ddp_clock);
		sprintf(buf, " disp_low_power_disable_ddp_clock :  %d\n ",
			disp_low_power_disable_ddp_clock);

	} else if (enable == 25) {
		if (disp_low_power_disable_fence_thread == 0)
			disp_low_power_disable_fence_thread = 1;
		else
			disp_low_power_disable_fence_thread = 0;

		pr_debug(" DDP : disp_low_power_disable_fence_thread = %d\n ",
		       disp_low_power_disable_fence_thread);
		sprintf(buf, " disp_low_power_disable_fence_thread :  %d\n ",
			disp_low_power_disable_fence_thread);

	} else if (enable == 26) {
		if (disp_low_power_remove_ovl == 0)
			disp_low_power_remove_ovl = 1;
		else
			disp_low_power_remove_ovl = 0;

		pr_debug(" DDP : disp_low_power_remove_ovl = %d\n ", disp_low_power_remove_ovl);
		sprintf(buf, " disp_low_power_remove_ovl :  %d\n ", disp_low_power_remove_ovl);

	} else if (enable == 27) {
		if (gSkipIdleDetect == 0)
			gSkipIdleDetect = 1;
		else
			gSkipIdleDetect = 0;

		pr_debug(" DDP : gSkipIdleDetect = %d\n ", gSkipIdleDetect);
		sprintf(buf, " gSkipIdleDetect :  %d\n ", gSkipIdleDetect);

	} else if (enable == 28) {
		if (gDumpClockStatus == 0)
			gDumpClockStatus = 1;
		else
			gDumpClockStatus = 0;

		pr_debug(" DDP : gDumpClockStatus = %d\n ", gDumpClockStatus);
		sprintf(buf, " gDumpClockStatus :  %d\n ", gDumpClockStatus);

	} else if (enable == 29) {
		if (gEnableUartLog == 0)
			gEnableUartLog = 1;
		else
			gEnableUartLog = 0;

		pr_debug(" DDP : gEnableUartLog = %d\n ", gEnableUartLog);
		sprintf(buf, " gEnableUartLog :  %d\n ", gEnableUartLog);

	} else if (enable == 30) {
		if (gEnableMutexRisingEdge == 0) {
			gEnableMutexRisingEdge = 1;
			DISP_REG_SET_FIELD(0, SOF_FLD_MUTEX0_SOF_TIMING, DISP_REG_CONFIG_MUTEX0_SOF,
					   1);
		} else {
			gEnableMutexRisingEdge = 0;
			DISP_REG_SET_FIELD(0, SOF_FLD_MUTEX0_SOF_TIMING, DISP_REG_CONFIG_MUTEX0_SOF,
					   0);
		}

		pr_debug(" DDP : gEnableMutexRisingEdge = %d\n ", gEnableMutexRisingEdge);
		sprintf(buf, " gEnableMutexRisingEdge :  %d\n ", gEnableMutexRisingEdge);

	} else if (enable == 31) {
		if (gEnableReduceRegWrite == 0)
			gEnableReduceRegWrite = 1;
		else
			gEnableReduceRegWrite = 0;

		pr_debug(" DDP : gEnableReduceRegWrite = %d\n ", gEnableReduceRegWrite);
		sprintf(buf, " gEnableReduceRegWrite :  %d\n ", gEnableReduceRegWrite);

	} else if (enable == 32) {
		/* DDPAEE(" DDP : (32) gEnableReduceRegWrite = %d\n ", gEnableReduceRegWrite); */
	} else if (enable == 33) {
		if (gDumpConfigCMD == 0)
			gDumpConfigCMD = 1;
		else
			gDumpConfigCMD = 0;

		pr_debug(" DDP : gDumpConfigCMD = %d\n ", gDumpConfigCMD);
		sprintf(buf, " gDumpConfigCMD :  %d\n ", gDumpConfigCMD);

	} else if (enable == 34) {
		if (gESDEnableSODI == 0)
			gESDEnableSODI = 1;
		else
			gESDEnableSODI = 0;

		pr_debug(" DDP : gESDEnableSODI = %d\n ", gESDEnableSODI);
		sprintf(buf, " gESDEnableSODI :  %d\n ", gESDEnableSODI);

	} else if (enable == 35) {
		if (gEnableOVLStatusCheck == 0)
			gEnableOVLStatusCheck = 1;
		else
			gEnableOVLStatusCheck = 0;

		pr_debug(" DDP : gEnableOVLStatusCheck = %d\n ", gEnableOVLStatusCheck);
		sprintf(buf, " gEnableOVLStatusCheck :  %d\n ", gEnableOVLStatusCheck);

	} else if (enable == 36) {
		if (gResetRDMAEnable == 0)
			gResetRDMAEnable = 1;
		else
			gResetRDMAEnable = 0;

		pr_debug(" DDP : gResetRDMAEnable = %d\n ", gResetRDMAEnable);
		sprintf(buf, " gResetRDMAEnable :  %d\n ", gResetRDMAEnable);
	} else if (enable == 37) {
		unsigned int reg_value = 0;

		if (gEnableIRQ == 0) {
			gEnableIRQ = 1;

			/* OVL0/OVL1 */
			DISP_CPU_REG_SET(DISP_REG_OVL_INTEN, 0x1e2);
			DISP_CPU_REG_SET(DISP_REG_OVL_INTEN + DISP_OVL_INDEX_OFFSET, 0x1e2);

			/* Mutex0 */
			reg_value = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTEN);
			DISP_CPU_REG_SET(DISP_REG_CONFIG_MUTEX_INTEN,
					 reg_value | (1 << 0) | (1 << DISP_MUTEX_TOTAL));

			/* RDMA0 */
			DISP_CPU_REG_SET(DISP_REG_RDMA_INT_ENABLE, 0x3E);
		} else {
			gEnableIRQ = 0;

			/* OVL0/OVL1 */
			DISP_CPU_REG_SET(DISP_REG_OVL_INTEN, 0x1e0);
			DISP_CPU_REG_SET(DISP_REG_OVL_INTEN + DISP_OVL_INDEX_OFFSET, 0x1e0);

			/* Mutex0 */
			reg_value = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTEN);
			DISP_CPU_REG_SET(DISP_REG_CONFIG_MUTEX_INTEN,
					 reg_value & (~(1 << 0)) & (~(1 << DISP_MUTEX_TOTAL)));

			/* RDMA0 */
			DISP_CPU_REG_SET(DISP_REG_RDMA_INT_ENABLE, 0x18);

		}

		pr_debug(" DDP : gEnableIRQ = %d\n ", gEnableIRQ);
		sprintf(buf, " gEnableIRQ :  %d\n ", gEnableIRQ);

	} else if (enable == 38) {
		if (gDisableSODIForTriggerLoop == 0)
			gDisableSODIForTriggerLoop = 1;
		else
			gDisableSODIForTriggerLoop = 0;

		pr_debug(" DDP : gDisableSODIForTriggerLoop = %d\n ", gDisableSODIForTriggerLoop);
		sprintf(buf, " gDisableSODIForTriggerLoop :  %d\n ", gDisableSODIForTriggerLoop);

	} else if (enable == 39) {
		cmdqCoreSetEvent(CMDQ_SYNC_TOKEN_STREAM_EOF);
		cmdqCoreSetEvent(CMDQ_EVENT_DISP_RDMA0_EOF);
		sprintf(buf, " enable = %d\n ", enable);
	} else if (enable == 41) {
		if (gResetOVLInAALTrigger == 0)
			gResetOVLInAALTrigger = 1;
		else
			gResetOVLInAALTrigger = 0;

		pr_debug(" DDP : gResetOVLInAALTrigger = %d\n ", gResetOVLInAALTrigger);
		sprintf(buf, " gResetOVLInAALTrigger :  %d\n ", gResetOVLInAALTrigger);

	} else if (enable == 42) {
		if (gDisableOVLTF == 0)
			gDisableOVLTF = 1;
		else
			gDisableOVLTF = 0;

		pr_debug(" DDP : gDisableOVLTF = %d\n ", gDisableOVLTF);
		sprintf(buf, " gDisableOVLTF :  %d\n ", gDisableOVLTF);

	} else if (enable == 43) {
		if (gDumpESDCMD == 0)
			gDumpESDCMD = 1;
		else
			gDumpESDCMD = 0;

		pr_debug(" DDP : gDumpESDCMD = %d\n ", gDumpESDCMD);
		sprintf(buf, " gDumpESDCMD :  %d\n ", gDumpESDCMD);

	} else if (enable == 44) {
		disp_dump_emi_status();
		disp_dump_emi_status();
		sprintf(buf, " dump emi status !\n ");
	} else if (enable == 45) {
		if (gEnableCMDQProfile == 0)
			gEnableCMDQProfile = 1;
		else
			gEnableCMDQProfile = 0;

		pr_debug(" DDP : gEnableCMDQProfile = %d\n ", gEnableCMDQProfile);
		sprintf(buf, " gEnableCMDQProfile :  %d\n ", gEnableCMDQProfile);

	} else if (enable == 46) {
		disp_set_pll(156);
		pr_debug(" DDP : disp_set_pll = 156.\n ");
		sprintf(buf, " disp_set_pll = 156.\n ");
	} else if (enable == 47) {
		disp_set_pll(182);
		pr_debug(" DDP : disp_set_pll = 182.\n ");
		sprintf(buf, " disp_set_pll = 182.\n ");
	} else if (enable == 48) {
		disp_set_pll(364);
		pr_debug(" DDP : disp_set_pll = 364\n ");
		sprintf(buf, " disp_set_pll = 364.\n ");
	} else if (enable == 49) {
		if (gChangeRDMAThreshold == 0)
			gChangeRDMAThreshold = 1;
		else
			gChangeRDMAThreshold = 0;

		pr_debug(" DDP : gChangeRDMAThreshold = %d\n ", gChangeRDMAThreshold);
		sprintf(buf, " gChangeRDMAThreshold :  %d\n ", gChangeRDMAThreshold);

	} else if (enable == 50) {
		if (gChangeMMClock == 0)
			gChangeMMClock = 1;
		else
			gChangeMMClock = 0;

		pr_debug(" DDP : gChangeMMClock = %d\n ", gChangeMMClock);
		sprintf(buf, " gChangeMMClock :  %d\n ", gChangeMMClock);

	} else if (enable == 51) {
		if (gEnableUnderflowAEE == 0)
			gEnableUnderflowAEE = 1;
		else
			gEnableUnderflowAEE = 0;

		pr_debug(" DDP : gEnableUnderflowAEE = %d\n ", gEnableUnderflowAEE);
		sprintf(buf, " gEnableUnderflowAEE :  %d\n ", gEnableUnderflowAEE);

	} else if (enable == 52) {
		unsigned int time;
		cmdqRecHandle handle = NULL;
		cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);

		time = disp_set_pll_by_cmdq(156, handle);
		pr_debug(" DDP : disp_set_pll_by_cmdq = 156. estimate execute time = %d\n ", time);
		sprintf(buf, " disp_set_pll_by_cmdq = 156. estimate execute time = %d\n ", time);
		cmdqRecDestroy(handle);
	} else if (enable == 53) {
		unsigned int time;
		cmdqRecHandle handle = NULL;
		cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);

		time = disp_set_pll_by_cmdq(182, handle);
		pr_debug(" DDP : disp_set_pll_by_cmdq = 182. estimate execute time = %d\n ", time);
		sprintf(buf, " disp_set_pll_by_cmdq = 182. estimate execute time = %d\n ", time);
		cmdqRecDestroy(handle);
	} else if (enable == 54) {
		unsigned int time;
		cmdqRecHandle handle = NULL;
		cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);

		time = disp_set_pll_by_cmdq(364, handle);
		pr_debug(" DDP : disp_set_pll_by_cmdq = 364. estimate execute time = %d\n ", time);
		sprintf(buf, " disp_set_pll_by_cmdq = 364. estimate execute time = %d\n ", time);
		cmdqRecDestroy(handle);
	} else if (enable == 55) {
		if (gIssueRequestThreshold == 0)
			gIssueRequestThreshold = 1;
		else
			gIssueRequestThreshold = 0;

		pr_debug(" DDP : gIssueRequestThreshold = %d\n ", gIssueRequestThreshold);
		sprintf(buf, " gIssueRequestThreshold :  %d\n ", gIssueRequestThreshold);

	} else if (enable == 56) {
		if (gDisableIRQWhenIdle == 0)
			gDisableIRQWhenIdle = 1;
		else
			gDisableIRQWhenIdle = 0;

		pr_debug(" DDP : gDisableIRQWhenIdle = %d\n ", gDisableIRQWhenIdle);
		sprintf(buf, " gDisableIRQWhenIdle :  %d\n ", gDisableIRQWhenIdle);
	} else if (enable == 57) {
		if (gEnableSODIWhenIdle == 0)
			gEnableSODIWhenIdle = 1;
		else
			gEnableSODIWhenIdle = 0;

		pr_debug(" DDP : gEnableSODIWhenIdle = %d\n ", gEnableSODIWhenIdle);
		sprintf(buf, " gEnableSODIWhenIdle :  %d\n ", gEnableSODIWhenIdle);

	} else if (enable == 58) {
#ifdef DISP_ENABLE_LAYER_FRAME
		if (gAddFrame == 0)
			gAddFrame = 1;
		else
			gAddFrame = 0;

		pr_debug(" DDP : gAddFrame = %d\n ", gAddFrame);
		sprintf(buf, " gAddFrame :  %d\n ", gAddFrame);
#else
		pr_debug(" Please enable DISP_ENABLE_LAYER_FRAME in ddp_debug.h first !\n ");
		sprintf(buf, " Please enable DISP_ENABLE_LAYER_FRAME in ddp_debug.h first !\n ");
#endif
	} else if (enable == 40) {
		sprintf(buf, " version :  %d, %s\n ", 12, __TIME__);
	} else if (enable==59) {
        extern void ddp_reset_test(void);
        ddp_reset_test();
        sprintf(buf, " dp_reset_test called. \n ");
	} else if (enable == 60) {
		unsigned int i = 0;
		int *modules = ddp_get_scenario_list(DDP_SCENARIO_PRIMARY_DISP);
		int module_num = ddp_get_module_num(DDP_SCENARIO_PRIMARY_DISP);

		pr_debug("dump path status:");
		for (i = 0; i < module_num; i++)
			pr_debug("%s-", ddp_get_module_name(modules[i]));

		pr_debug("\n");

		ddp_dump_analysis(DISP_MODULE_CONFIG);
		ddp_dump_analysis(DISP_MODULE_MUTEX);
		for (i = 0; i < module_num; i++)
			ddp_dump_analysis(modules[i]);

		if (primary_display_is_decouple_mode()) {
			ddp_dump_analysis(DISP_MODULE_OVL0);
			ddp_dump_analysis(DISP_MODULE_OVL1);
			ddp_dump_analysis(DISP_MODULE_WDMA0);
		}

		ddp_dump_reg(DISP_MODULE_CONFIG);
		ddp_dump_reg(DISP_MODULE_MUTEX);

		if (primary_display_is_decouple_mode()) {
			ddp_dump_reg(DISP_MODULE_OVL0);
			ddp_dump_reg(DISP_MODULE_OVL1);
			ddp_dump_reg(DISP_MODULE_WDMA0);
		}

		for (i = 0; i < module_num; i++)
			ddp_dump_reg(modules[i]);
	}
}
static void process_dbg_opt(const char *opt)
{
	char *buf = dbg_buf + strlen(dbg_buf);

	if (0 == strncmp(opt, "regr:", 5)) {
		unsigned long addr;
        int ret;
        
        ret = sscanf(opt, "regr: 0x%lx\n", &addr);
        if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
        
		if (is_reg_addr_valid(1, addr) == 1) {
			unsigned int regVal = DISP_REG_GET(addr);
			DDPMSG(" regr : 0x%lx = 0x%08x\n ", addr, regVal);
			sprintf(buf, " regr : 0x%lx = 0x%08x\n ", addr, regVal);
		} else {
			sprintf(buf, " regr, invalid address 0x%lx\n ", addr);
			goto Error;
		}
	} else if (0 == strncmp(opt, "regw:", 5)) {
		unsigned long addr;
		unsigned int val;
		unsigned int ret;
        
		ret = sscanf(opt, "regw: 0x%lx,0x%08x\n", &addr, &val);
		if (ret != 2) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}

		if (is_reg_addr_valid(1, addr) == 1) {
			unsigned int regVal;
			DISP_CPU_REG_SET(addr, val);
			regVal = DISP_REG_GET(addr);
			DDPMSG(" regw : 0x%lx, 0x%08x = 0x%08x\n ", addr, val, regVal);
			sprintf(buf, " regw : 0x%lx, 0x%08x = 0x%08x\n ", addr, val, regVal);
		} else {
			sprintf(buf, " regw, invalid address 0x%lx\n ", addr);
			goto Error;
		}
	} else if (0 == strncmp(opt, "rdma_ultra:", 11)) {
		int ret;
        
        ret = sscanf(opt, "rdma_ultra: 0x%x\n", &gRDMAUltraSetting);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        } 
        
		DISP_CPU_REG_SET(DISP_REG_RDMA_MEM_GMC_SETTING_0, gRDMAUltraSetting);
		sprintf(buf, " rdma_ultra, gRDMAUltraSetting = 0x%x, reg = 0x%x\n ", gRDMAUltraSetting,
			DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_0));
	} else if (0 == strncmp(opt, "rdma_fifo:", 10)) {
		int ret;
        
        ret = sscanf(opt, "rdma_fifo: 0x%x\n", &gRDMAFIFOLen);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
        
		DISP_CPU_REG_SET_FIELD(FIFO_CON_FLD_OUTPUT_VALID_FIFO_THRESHOLD,
				       DISP_REG_RDMA_FIFO_CON, gRDMAFIFOLen);
		sprintf(buf, " rdma_fifo, gRDMAFIFOLen = 0x%x, reg = 0x%x\n ", gRDMAFIFOLen,
			DISP_REG_GET(DISP_REG_RDMA_FIFO_CON));
	} else if (0 == strncmp(opt, "g_regr:", 7)) {
		unsigned int reg_va_before;
		unsigned long reg_va;
		unsigned long reg_pa;
        int ret;
        
        ret = sscanf(opt, "g_regr: 0x%lx\n", &reg_pa);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
        
		if (reg_pa < 0x10000000 || reg_pa > 0x18000000) {
			sprintf(buf, " g_regr, invalid pa = 0x%lx\n ", reg_pa);
		} else {
			reg_va = (unsigned long)ioremap_nocache(reg_pa, sizeof(unsigned long));
			reg_va_before = DISP_REG_GET(reg_va);
			pr_debug(" g_regr, pa = 0x%lx, va = 0x%lx, reg_val = 0x%x\n ",
			       reg_pa, reg_va, reg_va_before);
			sprintf(buf, " g_regr, pa = 0x%lx, va = 0x%lx, reg_val = 0x%x\n ",
				reg_pa, reg_va, reg_va_before);

			iounmap((void *)reg_va);
		}
	} else if (0 == strncmp(opt, "g_regw:", 7)) {
		unsigned int reg_va_before;
		unsigned int reg_va_after;
		unsigned int val;
		unsigned long reg_va;
		unsigned long reg_pa;
		int ret;
        
        ret = sscanf(opt, "g_regw: 0x%lx\n", &reg_pa);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        } 
        
		if (reg_pa < 0x10000000 || reg_pa > 0x18000000) {
			sprintf(buf, " g_regw, invalid pa = 0x%lx\n ", reg_pa);
		} else {           
            ret = sscanf(opt, "g_regw,val: 0x%x\n", &val);
            if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
            }
            
			reg_va = (unsigned long)ioremap_nocache(reg_pa, sizeof(unsigned long));
			reg_va_before = DISP_REG_GET(reg_va);
			DISP_CPU_REG_SET(reg_va, val);
			reg_va_after = DISP_REG_GET(reg_va);

			pr_debug
			("g_regw, pa = 0x%lx, va = 0x%lx, value = 0x%x, reg_val_before = 0x%x, reg_val_after = 0x%x\n ",
			     reg_pa, reg_va, val, reg_va_before, reg_va_after);
			sprintf(buf,
			" g_regw, pa = 0x%lx, va = 0x%lx, value = 0x%x, reg_val_before = 0x%x, reg_val_after = 0x%x\n ",
				reg_pa, reg_va, val, reg_va_before, reg_va_after);

			iounmap((void *)reg_va);
		}
	} else if (0 == strncmp(opt, "dbg_log:", 8)) {
		unsigned int enable;
		int ret;
        
        ret = sscanf(opt, "dbg_log: %d\n", &enable);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
                
		if (enable)
			dbg_log_level = 1;
		else
			dbg_log_level = 0;

		sprintf(buf, " dbg_log :  %d\n ", dbg_log_level);
	} else if (0 == strncmp(opt, "irq_log:", 8)) {
		unsigned int enable;
		int ret;
        
        ret = sscanf(opt, "irq_log: %d\n", &enable);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
        
		if (enable)
			irq_log_level = 1;
		else
			irq_log_level = 0;

		sprintf(buf, " irq_log :  %d\n ", irq_log_level);
	} else if (0 == strncmp(opt, "met_on:", 7)) {
		int met_on;
		int rdma0_mode;
		int rdma1_mode;
		int ret;
        
		ret = sscanf(opt, "met_on : %d,%d,%d\n",
				&met_on, &rdma0_mode, &rdma1_mode);
		if (ret != 3) {
			pr_err("error to parse cmd %s\n", opt);
			return;
		}
		ddp_init_met_tag(met_on, rdma0_mode, rdma1_mode);
		DDPMSG(" process_dbg_opt, met_on = %d, rdma0_mode %d, rdma1 %d\n ", met_on, rdma0_mode,
		       rdma1_mode);
		sprintf(buf, " met_on :  %d, rdma0_mode :  %d, rdma1_mode :  %d\n ", met_on, rdma0_mode,
			rdma1_mode);
	} else if (0 == strncmp(opt, "backlight:", 10)) {
		unsigned int level;
		int ret;
        
        ret = sscanf(opt, "backlight: %d\n", &level);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
        
		if (level) {
			disp_bls_set_backlight(level);
			sprintf(buf, " backlight :  %d\n ", level);
		} else {
			goto Error;
		}
	} else if (0 == strncmp(opt, "pwm0:", 5) || 0 == strncmp(opt, "pwm1:", 5)) {
		unsigned int level;
		int ret;
        ret = sscanf(opt, "pwm: %d\n", &level);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
		if (level) {
			disp_pwm_id_t pwm_id = DISP_PWM0;
			if (opt[3] == '1')
				pwm_id = DISP_PWM1;

			disp_pwm_set_backlight(pwm_id, level);
			sprintf(buf, " PWM 0x%x :  %d\n ", pwm_id, level);
		} else {
			goto Error;
		}
	} else if (0 == strncmp(opt, "aal_dbg:", 8)) {
        int ret;
        
        ret = sscanf(opt, "aal_dbg: %d\n", &aal_dbg_en);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
		sprintf(buf, " aal_dbg_en = 0x%x\n ", aal_dbg_en);
	} else if (0 == strncmp(opt, "dump_reg:", 9)) {
		unsigned int module;
		int ret;
        
        ret = sscanf(opt, "dump_reg: %d\n", &module);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
		DDPMSG(" process_dbg_opt, module = %d\n ", module);
		if (module < DISP_MODULE_NUM) {
			ddp_dump_reg(module);
			sprintf(buf, " dump_reg :  %d\n ", module);
		} else {
			DDPMSG(" process_dbg_opt2, module = %d\n ", module);
			goto Error;
		}
	} else if (0 == strncmp(opt, "dump_path:", 10)) {
		unsigned int mutex_idx;
		int ret;
        
        ret = sscanf(opt, "dump_path: %d\n", &mutex_idx);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
		DDPMSG(" process_dbg_opt, path mutex = %d\n ", mutex_idx);
		dpmgr_debug_path_status(mutex_idx);
		sprintf(buf, " dump_path :  %d\n ", mutex_idx);
	} else if (0 == strncmp(opt, "debug:", 6)) {
		unsigned int enable;
		int ret;
        
        ret = sscanf(opt, "debug: %d\n", &enable);
		if (ret != 1) {
				pr_err("error to parse cmd %s\n", opt);
				return;
        }
		disp_debug_api(enable, buf);
	} else if (0 == strncmp(opt, "mmp", 3)) {
		init_ddp_mmp_events();
	} else {
		dbg_buf[0] = '\0';
		goto Error;
	}

	return;

Error:
	DDPERR(" parse command error !\n%s\n\n%s", opt, STR_HELP);
}


static void process_dbg_cmd(char *cmd)
{
	char *tok;

	DDPDBG(" cmd : %s\n ", cmd);
	memset(dbg_buf, 0, sizeof(dbg_buf));
	while ((tok = strsep(&cmd, " ")) != NULL)
		process_dbg_opt(tok);

}


/* --------------------------------------------------------------------------- */
/* Debug FileSystem Routines */
/* --------------------------------------------------------------------------- */

static ssize_t debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}


static char cmd_buf[512];

static ssize_t debug_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	if (strlen(dbg_buf))
		return simple_read_from_buffer(ubuf, count, ppos, dbg_buf, strlen(dbg_buf));
	else
		return simple_read_from_buffer(ubuf, count, ppos, STR_HELP, strlen(STR_HELP));

}


static ssize_t debug_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	const int debug_bufmax = sizeof(cmd_buf) - 1;
	size_t ret;

	ret = count;

	if (count > debug_bufmax)
		count = debug_bufmax;

	if (copy_from_user(&cmd_buf, ubuf, count))
		return -EFAULT;

	cmd_buf[count] = 0;

	process_dbg_cmd(cmd_buf);

	return ret;
}


static const struct file_operations debug_fops = {
	.read = debug_read,
	.write = debug_write,
	.open = debug_open,
};

static ssize_t debug_dump_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	//dprec_logger_dump_reset();
	//dump_to_buffer = 1;
	/* dump all */
	//dpmgr_debug_path_status(-1);
	//dump_to_buffer = 0;

	return simple_read_from_buffer(buf, size, ppos, dprec_logger_get_dump_addr(),
				       dprec_logger_get_dump_len());
}


static const struct file_operations debug_fops_dump = {
	.read = debug_dump_read,
};

void ddp_debug_init(void)
{
	if (!debug_init) {
		debug_init = 1;
		debugfs = debugfs_create_file("dispsys",
					      S_IFREG | S_IRUGO, NULL, (void *)0, &debug_fops);


		debugDir = debugfs_create_dir("disp", NULL);
		if (debugDir) {

			debugfs_dump = debugfs_create_file("dump",
							   S_IFREG | S_IRUGO, debugDir, NULL,
							   &debug_fops_dump);
		}
	}
}

unsigned int ddp_debug_analysis_to_buffer(void)
{
	return dump_to_buffer;
}

unsigned int ddp_debug_dbg_log_level(void)
{
	return dbg_log_level;
}

unsigned int ddp_debug_irq_log_level(void)
{
	return irq_log_level;
}


void ddp_debug_exit(void)
{
	debugfs_remove(debugfs);
	debugfs_remove(debugfs_dump);
	debug_init = 0;
}

int ddp_mem_test(void)
{
	return -1;
}

int ddp_lcd_test(void)
{
	return -1;
}

char *disp_get_fmt_name(DP_COLOR_ENUM color)
{
	switch (color) {
	case DP_COLOR_FULLG8:
		return " fullg8 ";
	case DP_COLOR_FULLG10:
		return " fullg10 ";
	case DP_COLOR_FULLG12:
		return " fullg12 ";
	case DP_COLOR_FULLG14:
		return " fullg14 ";
	case DP_COLOR_UFO10:
		return " ufo10 ";
	case DP_COLOR_BAYER8:
		return " bayer8 ";
	case DP_COLOR_BAYER10:
		return " bayer10 ";
	case DP_COLOR_BAYER12:
		return " bayer12 ";
	case DP_COLOR_RGB565:
		return " rgb565 ";
	case DP_COLOR_BGR565:
		return " bgr565 ";
	case DP_COLOR_RGB888:
		return " rgb888 ";
	case DP_COLOR_BGR888:
		return " bgr888 ";
	case DP_COLOR_RGBA8888:
		return " rgba ";
	case DP_COLOR_BGRA8888:
		return " bgra ";
	case DP_COLOR_ARGB8888:
		return " argb ";
	case DP_COLOR_ABGR8888:
		return " abgr ";
	case DP_COLOR_I420:
		return " i420 ";
	case DP_COLOR_YV12:
		return " yv12 ";
	case DP_COLOR_NV12:
		return " nv12 ";
	case DP_COLOR_NV21:
		return " nv21 ";
	case DP_COLOR_I422:
		return " i422 ";
	case DP_COLOR_YV16:
		return " yv16 ";
	case DP_COLOR_NV16:
		return " nv16 ";
	case DP_COLOR_NV61:
		return " nv61 ";
	case DP_COLOR_YUYV:
		return " yuyv ";
	case DP_COLOR_YVYU:
		return " yvyu ";
	case DP_COLOR_UYVY:
		return " uyvy ";
	case DP_COLOR_VYUY:
		return " vyuy ";
	case DP_COLOR_I444:
		return " i444 ";
	case DP_COLOR_YV24:
		return " yv24 ";
	case DP_COLOR_IYU2:
		return " iyu2 ";
	case DP_COLOR_NV24:
		return " nv24 ";
	case DP_COLOR_NV42:
		return " nv42 ";
	case DP_COLOR_GREY:
		return " grey ";
	default:
		return " undefined ";
	}

}

unsigned int ddp_dump_reg_to_buf(unsigned int start_module, unsigned long *addr)
{
	unsigned int cnt = 0;
	unsigned long reg_addr;

	switch (start_module) {
	case 0:		/* DISP_MODULE_WDMA0: */
		reg_addr = DISP_REG_WDMA_INTEN;

		while (reg_addr <= DISP_REG_WDMA_PRE_ADD2) {
			addr[cnt++] = DISP_REG_GET(reg_addr);
			reg_addr += 4;
		}
	case 1:		/* DISP_MODULE_OVL: */
		reg_addr = DISP_REG_OVL_STA;

		while (reg_addr <= DISP_REG_OVL_L3_PITCH) {
			addr[cnt++] = DISP_REG_GET(reg_addr);
			reg_addr += 4;
		}
	case 2:		/* DISP_MODULE_RDMA: */
		reg_addr = DISP_REG_RDMA_INT_ENABLE;

		while (reg_addr <= DISP_REG_RDMA_PRE_ADD_1) {
			addr[cnt++] = DISP_REG_GET(reg_addr);
			reg_addr += 4;
		}
		break;
	}
	return cnt * sizeof(unsigned long);
}

#ifdef DISP_ENABLE_LAYER_FRAME
/* red, orange, yellow, green, cyan, blue, violet */
unsigned int disp_color[8] = { 0xFF0000FF, 0xFF0DC9FF, 0xFF00FFFF, 0xFF00FF00, 0xFF1DE6B5, 0xFFFF0000, 0xFF577AB9,
0xFFFF0080 };
int disp_draw_num(unsigned long va,	/* dst buf va */
		  unsigned int x,	/* x offset (pixel) */
		  unsigned int y,	/* y offset */
		  unsigned int pitch,	/* dst pitch */
		  DpColorFormat fmt,	/* dst buf format */
		  unsigned int number,	/* data_digit number */
		  unsigned int alpha_en,	/* transparent */
		  unsigned int color_argb)	/* background oqupa or not */
{
	unsigned int h = 0;
	unsigned int w = 0;
	unsigned int bpp = 0;
	unsigned int num_offset = 0;
	unsigned int len = 0;
	unsigned int i = 0;
	unsigned int num[20] = { 0 };
	unsigned int value;

	if (DP_COLOR_BITS_PER_PIXEL(fmt) != 32 && DP_COLOR_BITS_PER_PIXEL(fmt) != 24) {
		DDPERR(" only support ARGB / RGB888 now !\n ");
		return -1;
	}
	bpp = DP_COLOR_BITS_PER_PIXEL(fmt) / 8;

	do {
		if (len >= 20 || DISP_DIGIT_W * (len + 1) > pitch / bpp - x) {	/* len to long or buf width too short */
			DDPERR(" number too big for dst buf !\n ");
			break;
		}
		num[len++] = number % 10;
		number = number / 10;
	} while (number > 0);

	for (i = 0; i < len; i++) {

		num_offset = DISP_DIGIT_W * DISP_DIGIT_H / 8 * num[len - i - 1];
		for (h = 0; h < DISP_DIGIT_H; h++) {
			/* DDPMSG(" h = %d, value = 0x%x.\n ", h, value); */
			for (w = 0; w < DISP_DIGIT_W; w++) {
				value =
				    (unsigned int)data_digit[num_offset +
							     (h * DISP_DIGIT_W + w) / 8];

				if (bpp == 4) {
					if ((value & 0xff & (1 << (7 - (w % 8)))) == 0)
						*(unsigned int *)(va + h * pitch + w * bpp +
								  y * pitch + x * bpp +
								  i * DISP_DIGIT_W * bpp) =
						    color_argb;
					else if (alpha_en == 0)
						*(unsigned int *)(va + h * pitch + w * bpp +
								  y * pitch + x * bpp +
								  i * DISP_DIGIT_W * bpp) =
						    0xffffffff;

					/* DDPMSG(" w = %d, of = %d.\n ",
					w, h*pitch+w*bpp+y*pitch+x*bpp+i*DISP_DIGIT_W*bpp); */
				} else if (bpp == 3) {
					if ((value & 0xff & (1 << (7 - (w % 8)))) == 0) {
						*(unsigned char *)(va + h * pitch + w * bpp +
								   y * pitch + x * bpp +
								   i * DISP_DIGIT_W * bpp) =
						    (unsigned char)(color_argb & 0xff);
						*(unsigned char *)(va + h * pitch + w * bpp +
								   y * pitch + x * bpp +
								   i * DISP_DIGIT_W * bpp + 1) =
						    (unsigned char)((color_argb >> 8) & 0xff);
						*(unsigned char *)(va + h * pitch + w * bpp +
								   y * pitch + x * bpp +
								   i * DISP_DIGIT_W * bpp + 2) =
						    (unsigned char)((color_argb >> 16) & 0xff);
					} else if (alpha_en == 0) {
						*(unsigned int *)(va + h * pitch + w * bpp +
								  y * pitch + x * bpp +
								  i * DISP_DIGIT_W * bpp) = 0xff;
						*(unsigned int *)(va + h * pitch + w * bpp +
								  y * pitch + x * bpp +
								  i * DISP_DIGIT_W * bpp + 1) =
						    0xff;
						*(unsigned int *)(va + h * pitch + w * bpp +
								  y * pitch + x * bpp +
								  i * DISP_DIGIT_W * bpp + 2) =
						    0xff;
					}
				}
			}
		}
	}

	return 0;
}
#endif



