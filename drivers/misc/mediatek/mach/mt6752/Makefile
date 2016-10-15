
# Makefile for the linux kernel.
#
# Only aarch32 need to build core/ (need machine descriptor)

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(MTK_TABLET_HARDWARE), mt8752)
ccflags-y += -DMTK_TABLET_TURBO
endif

ifeq ($(CONFIG_MTK_TC1_FM_AT_SUSPEND),y)
ifeq ($(CONFIG_MT_SND_SOC_V2),y)
EXTRA_CFLAGS += -I$(srctree)/sound/soc/mediatek/mt_soc_audio_v2
endif
endif

ifneq ($(CONFIG_ARM64), y)
# For arm32
#ifeq ($(CONFIG_MTK_INTERNAL), yes)
#ifeq ($(MTK_USE_RESERVED_EXT_MEM), yes)
#KBUILD_CFLAGS += -DMTK_USE_RESERVED_EXT_MEM
#endif
#endif
#
obj-y := core.o
#obj-y += timer.o
obj-y += mt_devs.o
#obj-y += mt_boot.o
#obj-y += board.o
obj-y += mt_cpuxgpt.o
obj-y += mt_gpt.o
obj-y += ca53_timer.o
##obj-y += mt_dma.o
##obj-y += mt_reg_dump.o
#obj-y += mt_device_apc.o
obj-y += mt_pm_init.o
obj-y += mt-smp.o
obj-y += hotplug.o
obj-y += mt-headsmp.o
obj-y += mt_hotplug.o
#obj-y += mt_hotplug_mechanism.o
obj-y += mt_hotplug_strategy_main.o mt_hotplug_strategy_core.o mt_hotplug_strategy_procfs.o mt_hotplug_strategy_cpu.o mt_hotplug_strategy_api.o mt_hotplug_strategy_algo.o
obj-y += innercache.o
#obj-y += mt_l2c.o
#obj-y += eint.o
#obj-$(CONFIG_MT65XX_TRACER) += pmu_v7.o mon_interface.o mt_mon.o mt_emi_bm.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq
obj-y += camera_isp.o
obj-y += camera_sysram.o
#obj-y += camera_eis.o
obj-y += camera_fdvt.o
obj-y += mt_pm_ldo.o
#obj-y += mt_l2c.o
#obj-$(CONFIG_MTK_CIRQ) += mt_cirq.o
obj-y += mt_spm_mtcmos.o
obj-y += mt_sleep.o
obj-y += mt_cpufreq.o mt_cpu_ss.o
obj-y += mt_gpufreq.o
##obj-y += mt_emifreq.o
#
MT_CPUIDLE:=y
ifneq ($(MT_CPUIDLE),y)
obj-y += mt_dbg.o
obj-y += mt_dormant.o
else
obj-y += mt_cpuidle.o
KBUILD_CFLAGS += -DMT_CPUIDLE
KBUILD_AFLAGS += -DMT_CPUIDLE
endif
#
obj-y += cpu_dormant.o
obj-y += mt_idle.o
obj-y += mt_dcm.o
obj-y += mt_clkmgr_common.o
obj-y += mt_clkmgr.o
obj-y += mt_clkbuf_ctl.o
obj-y += mt_golden_setting.o
#obj-y += sbchk_base.o
##obj-y += mtk_ccci_helper.o
#
#ifneq ($(CONFIG_MTK_CCCI_DEVICES),y)
#ccflags-y  += -D__USING_DUMMY_CCCI_API__
#endif
#
#obj-y += mt_static_power.o
#
obj-y += mt_ptp.o
obj-y += mt_ptp2.o
##obj-y += mssv.o
#
## Hibernation
#AFLAGS_swsusp.o	:= -DTEXT_OFFSET=$(TEXT_OFFSET)
obj-$(CONFIG_HIBERNATION)	+= cpu_hibernate.o swsusp.o
#
#KBUILD_AFLAGS	:=$(KBUILD_AFLAGS:-msoft-float=-Wa,-mfpu=neon)
#
##KBUILD_CFLAGS += -DMT_DORMANT_UT
##KBUILD_CFLAGS += -DMT_SMP_VIRTUAL_BOOT_ADDR

# Module project related part
obj-y += $(ARCH_MTK_PROJECT)/

# hooks
obj-y += mt_hooks.o

else
# For arm64
obj-y += irq_write_protect.o
#obj-y += emi_bwl.o emi_mpu.o
obj-y += mt_cpuxgpt.o
obj-y += mt_gpt.o
obj-y += ca53_timer.o
obj-y += mt_spm_mtcmos.o
obj-y += mt_clkmgr_common.o
obj-y += mt_clkmgr.o
obj-y += mt_pm_ldo.o
obj-y += camera_isp.o
obj-y += camera_fdvt.o
obj-y += mt_sleep.o
obj-y += mt_idle.o
obj-y += mt_clkbuf_ctl.o
obj-y += mt_golden_setting.o
obj-y += mt_cpufreq.o mt_cpu_ss.o
obj-y += mt_gpufreq.o
# Module project related part
obj-y += $(ARCH_MTK_PROJECT)/
obj-y += innercache.o
obj-y += mt_pm_init.o
# Mediatek psci wrapper
obj-$(CONFIG_MTK_PSCI)	+= mt_psci.o
obj-y += mt_hotplug_strategy_main.o mt_hotplug_strategy_core.o mt_hotplug_strategy_procfs.o mt_hotplug_strategy_cpu.o mt_hotplug_strategy_api.o mt_hotplug_strategy_algo.o
obj-y += mt_ptp2.o
obj-y += mt_ptp.o
obj-y += mt_hooks_64.o

MT_CPUIDLE:=y
ifneq ($(MT_CPUIDLE),y)
obj-y += mt_dbg.o
obj-y += mt_dormant.o
else
obj-y += mt_cpuidle64.o
KBUILD_CFLAGS += -DMT_CPUIDLE
KBUILD_AFLAGS += -DMT_CPUIDLE
##KBUILD_CFLAGS += -DMT_DORMANT_UT
endif

obj-y += mt_dcm.o
## Hibernation
obj-$(CONFIG_HIBERNATION)  += hibernate64.o

endif
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/cam_cal),)
obj-$(CONFIG_MTK_CAM_CAL)   +=  $(ARCH_MTK_PROJECT)/cam_cal/
endif

ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/imgsensor),)
obj-y   +=  $(ARCH_MTK_PROJECT)/imgsensor/
endif

