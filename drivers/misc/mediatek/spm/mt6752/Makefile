ifeq ($(MTK_TABLET_HARDWARE),mt8752)
ccflags-y += -DMTK_TABLET_TURBO
endif

include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y	+= mt_spm.o
obj-y	+= mt_spm_internal.o
obj-$(CONFIG_PM)	+= mt_spm_fs.o
obj-y	+= mt_spm_sleep.o
obj-y	+= mt_spm_dpidle.o
obj-y	+= mt_spm_sodi.o
obj-y	+= mt_spm_mcdi.o
#obj-y	+= mt_spm_talking.o
