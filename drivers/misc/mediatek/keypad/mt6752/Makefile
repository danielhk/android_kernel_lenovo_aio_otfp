include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_MTK_TC1_FM_AT_SUSPEND),y)
ifeq ($(CONFIG_MT_SND_SOC_V2),y)
EXTRA_CFLAGS += -I$(srctree)/sound/soc/mediatek/mt_soc_audio_v2
endif
endif

obj-$(CONFIG_MTK_KEYPAD) := hal_kpd.o
