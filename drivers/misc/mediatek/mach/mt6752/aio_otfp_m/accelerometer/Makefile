
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_MPU6515A),y)
obj-y	+=  mpu6515/
endif
#lenovo-sw chenlj2 add 20151013 begin
ifeq ($(CONFIG_MTK_BMA222E_SISLEY2),y)
obj-y	+=  bma222e-sisley2/
endif
#lenovo-sw chenlj2 add 20151013 end

ifeq ($(CONFIG_MTK_BMA222E_AIO),y)
obj-y	+=  bma222E-aio/
endif
