include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o pmic_auxadc.o pmic_chr_type_det.o tps6128x.o mt6311.o

ifeq ($(CONFIG_MTK_PMIC_DVT_SUPPORT),y)
     obj-$(CONFIG_MTK_PMIC)     += pmic_dvt.o
endif

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o 

ifeq ($(CONFIG_MTK_BQ24296_SUPPORT),y)
     obj-$(CONFIG_MTK_SMART_BATTERY) += bq24296.o charging_hw_bq24296.o 
else
  ifeq ($(CONFIG_MTK_BQ24261_SUPPORT),y)
     obj-$(CONFIG_MTK_SMART_BATTERY) += bq24261.o charging_hw_bq24261.o
  else
     ifeq ($(CONFIG_MTK_SWCHR_SUPPORT),y)
          obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_mt_sw_charger.o                
     else
          ifeq ($(CONFIG_MTK_NCP1854_SUPPORT),y)
               obj-$(CONFIG_MTK_SMART_BATTERY) += ncp1854.o charging_hw_ncp1854.o             
          else
               ifeq ($(CONFIG_MTK_RT9536_SUPPORT),y)
                   obj-$(CONFIG_MTK_SMART_BATTERY) += rt9536/
               else               
                    ifeq ($(CONFIG_MTK_BQ24196_SUPPORT),y)
                        obj-$(CONFIG_MTK_SMART_BATTERY) += bq24196.o charging_hw_bq24196.o
                    else
                         ifeq ($(CONFIG_MTK_BQ24157_SUPPORT),y)
    		             obj-$(CONFIG_MTK_SMART_BATTERY) += bq24157.o charging_hw_bq24157.o 
                         else
                            obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_pmic.o
                         endif
                    endif
               endif
          endif
      endif
  endif
endif

#Begin lenovo-sw chailu1 add for 3rd FG max17058
ifeq ($(CONFIG_BATTERY_MAX17058_SUPPORT),y)
     obj-$(CONFIG_BATTERY_MAX17058_SUPPORT)     += max17058_battery.o
endif
#End lenovo-sw chailu1 add for 3rd FG max17058
