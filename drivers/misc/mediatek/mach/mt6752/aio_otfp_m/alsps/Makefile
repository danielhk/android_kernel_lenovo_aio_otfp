
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  cm36652-new/
endif

obj-$(CONFIG_MTK_CM366XX)	+=  cm366xx/

ifeq ($(CONFIG_MTK_EPL21822_AIO),y)
obj-y	+=  EPL21822-aio/
endif
