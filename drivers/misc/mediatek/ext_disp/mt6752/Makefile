#
# Makefile for external display driver.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/misc/mediatek/ext_disp/  \
             -I$(srctree)/drivers/misc/mediatek/video/common \
             -I$(srctree)/drivers/misc/mediatek/video/include \
             -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys \
             -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox  \
             -I$(srctree)/drivers/misc/mediatek/sync/ \
             -I$(srctree)/drivers/misc/mediatek/hdmi/$(CONFIG_CUSTOM_KERNEL_HDMI)/ \
             -I$(srctree)/drivers/misc/mediatek/cmdq/ \
             -I$(srctree)/drivers/misc/mediatek/hdmi/inc/ \
             -I$(srctree)/drivers/staging/android/ion


obj-y :=  \
	external_display.o      \
	./../mtk_extd_mgr.o     \
	./../extd_debug.o       \
	./../extd_utils.o       \
	./../extd_hdmi.o    \
	./../extd_epd.o         \
	./../extd_factory.o     \
	./../extd_multi_control.o 
