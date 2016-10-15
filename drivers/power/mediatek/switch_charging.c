/*****************************************************************************
 *
 * Filename:
 * ---------
 *    linear_charging.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   11 Aug 2005 10:28:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *
 * 03 05 2015 wy.chuang
 * [ALPS01921641] [L1_merge] for PMIC and charging
 * .
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/kernel.h>
#include <mach/battery_common.h>
#include <mach/charging.h>
#include "cust_charging.h"
#include <mach/mt_boot.h>
#include <mach/battery_meter.h>
//modify by willcai 2014-5-29 begin
#ifdef CONFIG_LENOVO_CHARGING_STANDARD_SUPPORT
#include "lenovo_charging.h"
#endif
//end

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#if !defined(TA_AC_CHARGING_CURRENT)
#include "cust_pe.h"
#endif
#endif

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mach/diso.h>
#endif

 /* ============================================================ // */
 /* define */
 /* ============================================================ // */
 /* cut off to full */
#define POST_CHARGING_TIME		30 * 60		/* 30mins */
#define FULL_CHECK_TIMES		6

 /* ============================================================ // */
 /* global variable */
 /* ============================================================ // */
kal_uint32 g_bcct_flag = 0;
kal_uint32 g_bcct_value = 0;
kal_uint32 g_full_check_count = 0;
 /*Begin Lenovo-sw mahj2 add for thermal 2014-10-28*/
#ifdef CONFIG_LENOVO_THERMAL_SUPPORT
CHR_CURRENT_ENUM g_bcct_CC_value = CHARGE_CURRENT_0_00_MA;
#endif
/*Enable Lenovo-sw chailu1 add for thermal 2014-10-28*/
//lenovo-sw mahj2 modify for ncp1854 charging bug Begin
#ifdef CONFIG_MTK_NCP1854_SUPPORT
CHR_CURRENT_ENUM g_pre_CC_value = CHARGE_CURRENT_0_00_MA;
#endif
//lenovo-sw mahj2 modify for ncp1854 charging bug End
CHR_CURRENT_ENUM g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
kal_uint32 g_usb_state = USB_UNCONFIGURED;
static bool usb_unlimited=false;
/*Begin, lenovo-sw chailu1 add 45-50  CV limit fuction, use 3rd fg  20150318 */
#ifdef LENOVO_TEMP_POS_45_TO_POS_50_CV_LiMIT_SUPPORT
static int g_lenovo_cv_limit_stage =0;	
#endif
/*End, lelenovo-sw chailu1 add 45-50  CV limit fuction, use 3rd fg  20150318 */

/*Begin sw chailu1 add 20150403*/
#ifdef LENOVO_CHARGING_FULL_CHECK_AGAIN_SUPPORT
static bool  g_is_lenovo_charging_check_again_state = KAL_FALSE;
static UINT32      g_bat_charging_state_back = CHR_PRE;
extern  void lenovo_charging_again_enble(void);
#endif
/*End sw chailu1 add 20150403*/			  

/*Begin, lenovo-sw chailu1 add     20150730 */
#ifdef LENOVO_LIMIT_IN_OTA_UPGRADE_SUPPORT 
extern kal_bool g_ota_upgrade;
#endif
/*Begin, lenovo-sw chailu1 add     20150730 */

BATTERY_VOLTAGE_ENUM cv_voltage;
  /* ///////////////////////////////////////////////////////////////////////////////////////// */
  /* // PUMP EXPRESS */
  /* ///////////////////////////////////////////////////////////////////////////////////////// */
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
struct wake_lock TA_charger_suspend_lock;
kal_bool ta_check_chr_type = KAL_TRUE;
kal_bool ta_cable_out_occur = KAL_FALSE;
kal_bool is_ta_connect = KAL_FALSE;
kal_bool ta_vchr_tuning = KAL_TRUE;
int ta_v_chr_org = 0;
#endif

  /* ///////////////////////////////////////////////////////////////////////////////////////// */
  /* // JEITA */
  /* ///////////////////////////////////////////////////////////////////////////////////////// */
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
int g_temp_status = TEMP_POS_10_TO_POS_45;
kal_bool temp_error_recovery_chr_flag = KAL_TRUE;
#endif



 /* ============================================================ // */
 /* function prototype */
 /* ============================================================ // */


 /* ============================================================ // */
 /* extern variable */
 /* ============================================================ // */
extern int g_platform_boot_mode;

 /* ============================================================ // */
 /* extern function */
 /* ============================================================ // */


 /* ============================================================ // */
void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_log(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\r\n");
#else
	if ((usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))) {
		battery_log(BAT_LOG_CRTI,
				    "[BATTERY] BAT_SetUSBState Fail! Restore to default value\r\n");
		usb_state_value = USB_UNCONFIGURED;
	} else {
		battery_log(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Success! Set %d\r\n",
				    usb_state_value);
		g_usb_state = usb_state_value;
	}
#endif
}


kal_uint32 get_charging_setting_current(void)
{
	return g_temp_CC_value;
}

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
static DEFINE_MUTEX(ta_mutex);

static void set_ta_charging_current(void)
{
	int real_v_chrA = 0;
	
	real_v_chrA = battery_meter_get_charger_voltage();
	battery_log(BAT_LOG_CRTI, "set_ta_charging_current, chrA=%d, chrB=%d\n", 
		ta_v_chr_org, real_v_chrA);

	if((real_v_chrA - ta_v_chr_org) > 3000) {
		g_temp_input_CC_value = TA_AC_9V_INPUT_CURRENT;  //TA = 9V		
		g_temp_CC_value = TA_AC_CHARGING_CURRENT;
	} else if((real_v_chrA - ta_v_chr_org) > 1000) {
		g_temp_input_CC_value = TA_AC_7V_INPUT_CURRENT;  //TA = 7V
		g_temp_CC_value = TA_AC_CHARGING_CURRENT;
	}
}

static void mtk_ta_reset_vchr(void)
{
	CHR_CURRENT_ENUM chr_current = CHARGE_CURRENT_70_00_MA;

	battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,&chr_current);
	msleep(250);    // reset Vchr to 5V

	battery_log(BAT_LOG_CRTI, "mtk_ta_reset_vchr(): reset Vchr to 5V \n");
}

static void mtk_ta_increase(void)
{
	kal_bool ta_current_pattern = KAL_TRUE;  // TRUE = increase

	if(ta_cable_out_occur == KAL_FALSE) {
		battery_charging_control(CHARGING_CMD_SET_TA_CURRENT_PATTERN, &ta_current_pattern);
	} else {
		ta_check_chr_type = KAL_TRUE;
		battery_log(BAT_LOG_CRTI, "mtk_ta_increase() Cable out \n");
	}
}

static kal_bool mtk_ta_retry_increase(void)
{
	int real_v_chrA;
	int real_v_chrB;
	kal_bool retransmit = KAL_TRUE;
	kal_uint32 retransmit_count=0;
	
	do {
		real_v_chrA = battery_meter_get_charger_voltage();
		mtk_ta_increase();  //increase TA voltage to 7V
		real_v_chrB = battery_meter_get_charger_voltage();

		if(real_v_chrB - real_v_chrA >= 1000) {	/* 1.0V */
			retransmit = KAL_FALSE;
		} else {
			retransmit_count++;
			battery_log(BAT_LOG_CRTI, "mtk_ta_detector(): retransmit_count =%d, chrA=%d, chrB=%d\n", 
				retransmit_count, real_v_chrA, real_v_chrB);
		}

		if((retransmit_count == 3) || (BMT_status.charger_exist == KAL_FALSE)) {
			retransmit = KAL_FALSE;
		}

	} while((retransmit == KAL_TRUE) && (ta_cable_out_occur == KAL_FALSE));

	battery_log(BAT_LOG_CRTI, "mtk_ta_retry_increase() real_v_chrA=%d, real_v_chrB=%d, retry=%d\n", 
	real_v_chrA, real_v_chrB,retransmit_count);

	if(retransmit_count == 3)
		return KAL_FALSE;	
	else
		return KAL_TRUE;	
}

static void mtk_ta_detector(void)
{
	int real_v_chrB = 0;

	battery_log(BAT_LOG_CRTI, "mtk_ta_detector() start\n");

	ta_v_chr_org = battery_meter_get_charger_voltage();
	mtk_ta_retry_increase();
	real_v_chrB = battery_meter_get_charger_voltage();

	if(real_v_chrB - ta_v_chr_org >= 1000)
		is_ta_connect = KAL_TRUE;
	else
		is_ta_connect = KAL_FALSE;
		
	battery_log(BAT_LOG_CRTI, "mtk_ta_detector() end, is_ta_connect=%d\n",is_ta_connect);
}

static void mtk_ta_init(void)
{
	is_ta_connect = KAL_FALSE;
	ta_cable_out_occur = KAL_FALSE;

#ifdef TA_9V_SUPPORT
	ta_vchr_tuning = KAL_FALSE;
#endif

	battery_charging_control(CHARGING_CMD_INIT,NULL);
}

static void battery_pump_express_charger_check(void)
{
	if (KAL_TRUE == ta_check_chr_type &&
		STANDARD_CHARGER == BMT_status.charger_type &&
		BMT_status.SOC >= TA_START_BATTERY_SOC &&
		BMT_status.SOC < TA_STOP_BATTERY_SOC) {

		mutex_lock(&ta_mutex);
		wake_lock(&TA_charger_suspend_lock);

		mtk_ta_reset_vchr();

		mtk_ta_init();
		mtk_ta_detector();

		/* need to re-check if the charger plug out during ta detector */
		if(KAL_TRUE == ta_cable_out_occur)
			ta_check_chr_type = KAL_TRUE;
		else
			ta_check_chr_type = KAL_FALSE;

		wake_unlock(&TA_charger_suspend_lock);
		mutex_unlock(&ta_mutex);
	} else {
		battery_log(BAT_LOG_CRTI, 
		"Stop battery_pump_express_charger_check, SOC=%d, ta_check_chr_type = %d, charger_type = %d \n", 
		BMT_status.SOC, ta_check_chr_type, BMT_status.charger_type);
	}
}

static void battery_pump_express_algorithm_start(void)
{
	kal_int32 charger_vol;
	kal_uint32 charging_enable = KAL_FALSE;

	mutex_lock(&ta_mutex);
	wake_lock(&TA_charger_suspend_lock);

	if(KAL_TRUE == is_ta_connect) {
		/* check cable impedance */
		charger_vol = battery_meter_get_charger_voltage();
		if(KAL_FALSE == ta_vchr_tuning) {
			mtk_ta_retry_increase();	/* increase TA voltage to 9V */
			charger_vol = battery_meter_get_charger_voltage();
			ta_vchr_tuning = KAL_TRUE;
		} else if(BMT_status.SOC > TA_STOP_BATTERY_SOC) {
			/* disable charging, avoid Iterm issue */
			battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
			mtk_ta_reset_vchr();	//decrease TA voltage to 5V
			charger_vol = battery_meter_get_charger_voltage();
			if(abs(charger_vol - ta_v_chr_org) <= 1000)	/* 1.0V */
				is_ta_connect = KAL_FALSE;

			battery_log(BAT_LOG_CRTI, "Stop battery_pump_express_algorithm, SOC=%d is_ta_connect =%d, TA_STOP_BATTERY_SOC: %d\n",
				BMT_status.SOC, is_ta_connect, TA_STOP_BATTERY_SOC);
		}
		battery_log(BAT_LOG_CRTI, "[BATTERY] check cable impedance, VA(%d) VB(%d) delta(%d).\n", 
		ta_v_chr_org, charger_vol, charger_vol - ta_v_chr_org);

		battery_log(BAT_LOG_CRTI, "mtk_ta_algorithm() end\n");
	} else {
		battery_log(BAT_LOG_CRTI, "It's not a TA charger, bypass TA algorithm\n");
	}

	wake_unlock(&TA_charger_suspend_lock);
	mutex_unlock(&ta_mutex);
}
#endif

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)

static BATTERY_VOLTAGE_ENUM select_jeita_cv(void)
{
	if (g_temp_status == TEMP_ABOVE_POS_60) {
		cv_voltage = JEITA_TEMP_ABOVE_POS_60_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_45_TO_POS_60) {
		cv_voltage = JEITA_TEMP_POS_45_TO_POS_60_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_10_TO_POS_45) {
#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
		cv_voltage = BATTERY_VOLT_04_340000_V;
#else
		cv_voltage = JEITA_TEMP_POS_10_TO_POS_45_CV_VOLTAGE;
#endif
	} else if (g_temp_status == TEMP_POS_0_TO_POS_10) {
		cv_voltage = JEITA_TEMP_POS_0_TO_POS_10_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
		cv_voltage = JEITA_TEMP_NEG_10_TO_POS_0_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_BELOW_NEG_10) {
		cv_voltage = JEITA_TEMP_BELOW_NEG_10_CV_VOLTAGE;
	} else {
		cv_voltage = BATTERY_VOLT_04_200000_V;
	}

	return cv_voltage;
}

PMU_STATUS do_jeita_state_machine(void)
{
	/* JEITA battery temp Standard */

	if (BMT_status.temperature >= TEMP_POS_60_THRESHOLD) {
		battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery Over high Temperature(%d) !!\n\r",
				    TEMP_POS_60_THRESHOLD);

		g_temp_status = TEMP_ABOVE_POS_60;

		return PMU_STATUS_FAIL;
	} else if (BMT_status.temperature > TEMP_POS_45_THRESHOLD)	/* control 45c to normal behavior */
	{
		if ((g_temp_status == TEMP_ABOVE_POS_60)
		    && (BMT_status.temperature >= TEMP_POS_60_THRES_MINUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
					    TEMP_POS_60_THRES_MINUS_X_DEGREE,
					    TEMP_POS_60_THRESHOLD);

			return PMU_STATUS_FAIL;
		} else {
			battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
					    TEMP_POS_45_THRESHOLD, TEMP_POS_60_THRESHOLD);

			g_temp_status = TEMP_POS_45_TO_POS_60;
		}
	} else if (BMT_status.temperature >= TEMP_POS_10_THRESHOLD) {
		if (((g_temp_status == TEMP_POS_45_TO_POS_60)
		     && (BMT_status.temperature >= TEMP_POS_45_THRES_MINUS_X_DEGREE))
		    || ((g_temp_status == TEMP_POS_0_TO_POS_10)
			&& (BMT_status.temperature <= TEMP_POS_10_THRES_PLUS_X_DEGREE))) {
			battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature not recovery to normal temperature charging mode yet!!\n\r");
		} else {
			battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Normal Temperature between %d and %d !!\n\r",
					    TEMP_POS_10_THRESHOLD, TEMP_POS_45_THRESHOLD);
			g_temp_status = TEMP_POS_10_TO_POS_45;
		}
	} else if (BMT_status.temperature >= TEMP_POS_0_THRESHOLD) {
		if ((g_temp_status == TEMP_NEG_10_TO_POS_0 || g_temp_status == TEMP_BELOW_NEG_10)
		    && (BMT_status.temperature <= TEMP_POS_0_THRES_PLUS_X_DEGREE)) {
			if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
				battery_log(BAT_LOG_CRTI,
						    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
						    TEMP_POS_0_THRES_PLUS_X_DEGREE,
						    TEMP_POS_10_THRESHOLD);
			}
			if (g_temp_status == TEMP_BELOW_NEG_10) {
				battery_log(BAT_LOG_CRTI,
						    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
						    TEMP_POS_0_THRESHOLD,
						    TEMP_POS_0_THRES_PLUS_X_DEGREE);
				return PMU_STATUS_FAIL;
			}
		} else {
			battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
					    TEMP_POS_0_THRESHOLD, TEMP_POS_10_THRESHOLD);

			g_temp_status = TEMP_POS_0_TO_POS_10;
		}
	} else if (BMT_status.temperature >= TEMP_NEG_10_THRESHOLD) {
		if ((g_temp_status == TEMP_BELOW_NEG_10)
		    && (BMT_status.temperature <= TEMP_NEG_10_THRES_PLUS_X_DEGREE)) {
			battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
					    TEMP_NEG_10_THRESHOLD, TEMP_NEG_10_THRES_PLUS_X_DEGREE);

			return PMU_STATUS_FAIL;
		} else {
			battery_log(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
					    TEMP_NEG_10_THRESHOLD, TEMP_POS_0_THRESHOLD);

			g_temp_status = TEMP_NEG_10_TO_POS_0;
		}
	} else {
		battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Battery below low Temperature(%d) !!\n\r",
				    TEMP_NEG_10_THRESHOLD);
		g_temp_status = TEMP_BELOW_NEG_10;

		return PMU_STATUS_FAIL;
	}

	/* set CV after temperature changed */

	cv_voltage = select_jeita_cv();
	battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);

	return PMU_STATUS_OK;
}


static void set_jeita_charging_current(void)
{
#ifdef CONFIG_USB_IF
	if (BMT_status.charger_type == STANDARD_HOST)
		return;
#endif

	if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
		g_temp_CC_value = CHARGE_CURRENT_350_00_MA;
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		battery_log(BAT_LOG_CRTI, "[BATTERY] JEITA set charging current : %d\r\n",
				    g_temp_CC_value);
	}
}

#endif




bool get_usb_current_unlimited(void)
{
	if (BMT_status.charger_type == STANDARD_HOST || BMT_status.charger_type == CHARGING_HOST)
		return usb_unlimited;
	else
		return false;
}

void set_usb_current_unlimited(bool enable)
{
	usb_unlimited = enable;
}

void select_charging_curret_bcct(void)
{
	if ((BMT_status.charger_type == STANDARD_HOST) ||
	    (BMT_status.charger_type == NONSTANDARD_CHARGER)) {
/*Begin Lenovo-sw mahj2 add for thermal 2014-10-28*/
#ifdef CONFIG_LENOVO_THERMAL_SUPPORT
		if(g_bcct_value < 100)	
		{
			g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA ;
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA ;
		}
		else
		{
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
			g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
		}
#else
		if (g_bcct_value < 100)
			g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 500)
			g_temp_input_CC_value = CHARGE_CURRENT_100_00_MA;
		else if (g_bcct_value < 800)
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		else if (g_bcct_value == 800)
			g_temp_input_CC_value = CHARGE_CURRENT_800_00_MA;
		else
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
#endif
/*End Lenovo-sw mahj2 add for thermal 2014-10-28*/
	} else if ((BMT_status.charger_type == STANDARD_CHARGER) ||
		   (BMT_status.charger_type == CHARGING_HOST)) {
/*Begin Lenovo-sw mahj2 add for thermal 2014-10-28*/
#ifdef CONFIG_LENOVO_THERMAL_SUPPORT

/*Begin Lenovo-sw chailu1 add for bcct 2014-05-04*/
#ifdef LENOVO_BCCT_SUPPROT
	g_temp_input_CC_value = LENOVO_BCCT_IPNUT_CURRENT;
       battery_xlog_printk(BAT_LOG_CRTI, "lenovo bcct limit : g_temp_input_CC_value=%d \n",g_temp_input_CC_value);
	   
	if(g_bcct_value < 550)        g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
#else
        g_temp_input_CC_value = CHARGE_CURRENT_MAX;

	if(g_bcct_value < 550)        g_temp_CC_value = CHARGE_CURRENT_0_00_MA;

#endif
/*End Lenovo-sw chailu1 add for bcct 2014-05-04*/
        else if(g_bcct_value < 650)   g_temp_CC_value = CHARGE_CURRENT_550_00_MA;
        else if(g_bcct_value < 750)   g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
        else if(g_bcct_value < 850)   g_temp_CC_value = CHARGE_CURRENT_750_00_MA;
        else if(g_bcct_value < 950)   g_temp_CC_value = CHARGE_CURRENT_850_00_MA;
        else if(g_bcct_value < 1050)  g_temp_CC_value = CHARGE_CURRENT_950_00_MA;
        else if(g_bcct_value < 1150)  g_temp_CC_value = CHARGE_CURRENT_1050_00_MA;
        else if(g_bcct_value < 1250)  g_temp_CC_value = CHARGE_CURRENT_1150_00_MA;
        else if(g_bcct_value < 1350) g_temp_CC_value = CHARGE_CURRENT_1250_00_MA;
	else if(g_bcct_value < 1450) g_temp_CC_value = CHARGE_CURRENT_1350_00_MA;
	else if(g_bcct_value < 1550) g_temp_CC_value = CHARGE_CURRENT_1450_00_MA;
	else if(g_bcct_value < 1650) g_temp_CC_value = CHARGE_CURRENT_1575_00_MA;
	else if(g_bcct_value < 1750) g_temp_CC_value = CHARGE_CURRENT_1650_00_MA;
	else if(g_bcct_value < 1850) g_temp_CC_value = CHARGE_CURRENT_1750_00_MA;
	else if(g_bcct_value < 1950) g_temp_CC_value = CHARGE_CURRENT_1875_00_MA;
	else if(g_bcct_value < 2050) g_temp_CC_value = CHARGE_CURRENT_1950_00_MA;
	else if(g_bcct_value == 2050) g_temp_CC_value = CHARGE_CURRENT_2050_00_MA;
        else                          g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
#else
		g_temp_input_CC_value = CHARGE_CURRENT_MAX;

		/* --------------------------------------------------- */
		/* set IOCHARGE */
		if (g_bcct_value < 550)
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 650)
			g_temp_CC_value = CHARGE_CURRENT_550_00_MA;
		else if (g_bcct_value < 750)
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		else if (g_bcct_value < 850)
			g_temp_CC_value = CHARGE_CURRENT_750_00_MA;
		else if (g_bcct_value < 950)
			g_temp_CC_value = CHARGE_CURRENT_850_00_MA;
		else if (g_bcct_value < 1050)
			g_temp_CC_value = CHARGE_CURRENT_950_00_MA;
		else if (g_bcct_value < 1150)
			g_temp_CC_value = CHARGE_CURRENT_1050_00_MA;
		else if (g_bcct_value < 1250)
			g_temp_CC_value = CHARGE_CURRENT_1150_00_MA;
		else if (g_bcct_value == 1250)
			g_temp_CC_value = CHARGE_CURRENT_1250_00_MA;
		else
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		/* --------------------------------------------------- */
#endif
/*End Lenovo-sw mahj2 add for thermal 2014-10-28*/
	} else {
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	}
}

static void pchr_turn_on_charging(void);
kal_uint32 set_bat_charging_current_limit(int current_limit)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] set_bat_charging_current_limit (%d)\r\n",
			    current_limit);

	if (current_limit != -1) {
		g_bcct_flag = 1;
		g_bcct_value = current_limit;

		if (current_limit < 70)
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (current_limit < 200)
			g_temp_CC_value = CHARGE_CURRENT_70_00_MA;
		else if (current_limit < 300)
			g_temp_CC_value = CHARGE_CURRENT_200_00_MA;
		else if (current_limit < 400)
			g_temp_CC_value = CHARGE_CURRENT_300_00_MA;
		else if (current_limit < 450)
			g_temp_CC_value = CHARGE_CURRENT_400_00_MA;
		else if (current_limit < 550)
			g_temp_CC_value = CHARGE_CURRENT_450_00_MA;
		else if (current_limit < 650)
			g_temp_CC_value = CHARGE_CURRENT_550_00_MA;
		else if (current_limit < 700)
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		else if (current_limit < 800)
			g_temp_CC_value = CHARGE_CURRENT_700_00_MA;
		else if (current_limit < 900)
			g_temp_CC_value = CHARGE_CURRENT_800_00_MA;
		else if (current_limit < 1000)
			g_temp_CC_value = CHARGE_CURRENT_900_00_MA;
		else if (current_limit < 1100)
			g_temp_CC_value = CHARGE_CURRENT_1000_00_MA;
		else if (current_limit < 1200)
			g_temp_CC_value = CHARGE_CURRENT_1100_00_MA;
		else if (current_limit < 1300)
			g_temp_CC_value = CHARGE_CURRENT_1200_00_MA;
		else if (current_limit < 1400)
			g_temp_CC_value = CHARGE_CURRENT_1300_00_MA;
		else if (current_limit < 1500)
			g_temp_CC_value = CHARGE_CURRENT_1400_00_MA;
		else if (current_limit < 1600)
			g_temp_CC_value = CHARGE_CURRENT_1500_00_MA;
		else if (current_limit == 1600)
			g_temp_CC_value = CHARGE_CURRENT_1600_00_MA;
		else
			g_temp_CC_value = CHARGE_CURRENT_450_00_MA;
	} else {
		/* change to default current setting */
		g_bcct_flag = 0;
	}

	/* wake_up_bat(); */
	pchr_turn_on_charging();

	return g_bcct_flag;
}


void select_charging_curret(void)
{
	if (g_ftm_battery_flag) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] FTM charging : %d\r\n",
				    charging_level_data[0]);
		g_temp_CC_value = charging_level_data[0];

		if (g_temp_CC_value == CHARGE_CURRENT_450_00_MA) {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_MAX;
			g_temp_CC_value = AC_CHARGER_CURRENT;

			battery_log(BAT_LOG_CRTI, "[BATTERY] set_ac_current \r\n");
		}
	} else {
		if (BMT_status.charger_type == STANDARD_HOST) {
#ifdef CONFIG_USB_IF
			{
				g_temp_input_CC_value = CHARGE_CURRENT_MAX;
				if (g_usb_state == USB_SUSPEND) {
					g_temp_CC_value = USB_CHARGER_CURRENT_SUSPEND;
				} else if (g_usb_state == USB_UNCONFIGURED) {
					g_temp_CC_value = USB_CHARGER_CURRENT_UNCONFIGURED;
				} else if (g_usb_state == USB_CONFIGURED) {
					g_temp_CC_value = USB_CHARGER_CURRENT_CONFIGURED;
				} else {
					g_temp_CC_value = USB_CHARGER_CURRENT_UNCONFIGURED;
				}

				battery_log(BAT_LOG_CRTI,
						    "[BATTERY] STANDARD_HOST CC mode charging : %d on %d state\r\n",
						    g_temp_CC_value, g_usb_state);
			}
#else
			{
				g_temp_input_CC_value = USB_CHARGER_CURRENT;
				g_temp_CC_value = USB_CHARGER_CURRENT;
			}
#endif
		} else if (BMT_status.charger_type == NONSTANDARD_CHARGER) {
			g_temp_input_CC_value = NON_STD_AC_CHARGER_CURRENT;
			g_temp_CC_value = NON_STD_AC_CHARGER_CURRENT;

		} else if (BMT_status.charger_type == STANDARD_CHARGER) {
	/* Lenovo sw chailu1 add, charger ic hw decide input current  20150226 */			
#if defined(AC_CHARGER_INPUT_CURRENT_MAX_SUPPORT)	
                     g_temp_input_CC_value = AC_CHARGER_INPUT_CURRENT_MAX;
#else
			g_temp_input_CC_value = AC_CHARGER_CURRENT;
#endif
			g_temp_CC_value = AC_CHARGER_CURRENT;
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
        		if(is_ta_connect == KAL_TRUE)
        			set_ta_charging_current();
#endif
		} else if (BMT_status.charger_type == CHARGING_HOST) {
			g_temp_input_CC_value = CHARGING_HOST_CHARGER_CURRENT;
			g_temp_CC_value = CHARGING_HOST_CHARGER_CURRENT;
		} else if (BMT_status.charger_type == APPLE_2_1A_CHARGER) {
			g_temp_input_CC_value = APPLE_2_1A_CHARGER_CURRENT;
			g_temp_CC_value = APPLE_2_1A_CHARGER_CURRENT;
		} else if (BMT_status.charger_type == APPLE_1_0A_CHARGER) {
			g_temp_input_CC_value = APPLE_1_0A_CHARGER_CURRENT;
			g_temp_CC_value = APPLE_1_0A_CHARGER_CURRENT;
		} else if (BMT_status.charger_type == APPLE_0_5A_CHARGER) {
			g_temp_input_CC_value = APPLE_0_5A_CHARGER_CURRENT;
			g_temp_CC_value = APPLE_0_5A_CHARGER_CURRENT;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
			g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
		}


		#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE) {
			g_temp_input_CC_value = AC_CHARGER_CURRENT;
			g_temp_CC_value = AC_CHARGER_CURRENT;
		}
		#endif

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
		set_jeita_charging_current();
#endif
	}


}


static kal_uint32 charging_full_check(void)
{
	kal_uint32 status;

	battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &status);
	if (status == KAL_TRUE) {
		g_full_check_count++;
		if (g_full_check_count >= FULL_CHECK_TIMES) {
			return KAL_TRUE;
		} else
			return KAL_FALSE;
	} else {
		g_full_check_count = 0;
		return status;
	}
}

extern int g_thermal_limit_current_level;
void thermal_pchr_turn_on_charging(void)
{
	pchr_turn_on_charging();
}
/*Lenovo-sw: AIOROW-4398 thermal limit current*/

static void pchr_turn_on_charging(void)
{
	kal_uint32 charging_enable = KAL_TRUE;

	#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	if(KAL_TRUE == BMT_status.charger_exist)
		charging_enable = KAL_TRUE;
	else 
		charging_enable = KAL_FALSE;
	#endif

	if (BMT_status.bat_charging_state == CHR_ERROR) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Charger Error, turn OFF charging !\n");

		charging_enable = KAL_FALSE;

	} else if ((g_platform_boot_mode == META_BOOT) || (g_platform_boot_mode == ADVMETA_BOOT)) {
		battery_log(BAT_LOG_CRTI,
				    "[BATTERY] In meta or advanced meta mode, disable charging.\n");
		charging_enable = KAL_FALSE;
	} else {
		/*HW initialization */
		battery_charging_control(CHARGING_CMD_INIT, NULL);

		battery_log(BAT_LOG_FULL, "charging_hw_init\n");

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
        battery_pump_express_algorithm_start();
#endif

		/* Set Charging Current */
		if (get_usb_current_unlimited()) {
			g_temp_input_CC_value = AC_CHARGER_CURRENT;
			g_temp_CC_value = AC_CHARGER_CURRENT;
			battery_log(BAT_LOG_FULL,
					    "USB_CURRENT_UNLIMITED, use AC_CHARGER_CURRENT\n");
		} else if (g_bcct_flag == 1) {
			select_charging_curret_bcct();

			battery_log(BAT_LOG_FULL,
					    "[BATTERY] select_charging_curret_bcct !\n");
		} else {
			select_charging_curret();

			battery_log(BAT_LOG_FULL, "[BATTERY] select_charging_curret !\n");
		}
#ifdef CONFIG_LENOVO_THERMAL_SUPPORT
	if (g_bcct_flag == 1)
	{
		g_bcct_CC_value = g_temp_CC_value;
	}
#endif
/*End Lenovo-sw mahj2 add for thermal 2014-10-28*/

//modify by willcai 2014-5-29 begin
	 #ifdef CONFIG_LENOVO_CHARGING_STANDARD_SUPPORT
	    lenovo_get_charging_curret(&g_temp_CC_value);
	 #endif
//end
/*Begin Lenovo-sw mahj2 add for thermal 2014-10-28*/
#ifdef CONFIG_LENOVO_THERMAL_SUPPORT
	if (g_bcct_flag == 1)
	{
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] thermal bcct : g_temp_CC_value=%d, g_bcct_CC_value = %d\r\n", g_temp_CC_value,g_bcct_CC_value);
		if(g_bcct_CC_value < g_temp_CC_value)	
			g_temp_CC_value = g_bcct_CC_value;
	}
#endif
/*End Lenovo-sw mahj2 add for thermal 2014-10-28*/

/*Begin, lenovo-sw chailu1 add     20150714 */
#ifdef LENOVO_LIMIT_IN_TALKING_SUPPORT
    if(g_call_state  == CALL_ACTIVE)
    {
        if(g_temp_input_CC_value > LENOVO_LIMIT_IN_TALKING_INPUT )
        {
            g_temp_input_CC_value = LENOVO_LIMIT_IN_TALKING_INPUT;
	 }
           
	 if(g_temp_CC_value > LENOVO_LIMIT_IN_TALKING_CHARING )
        {
            g_temp_CC_value = LENOVO_LIMIT_IN_TALKING_CHARING;
	 }
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] limit charger curret in talking : g_temp_CC_value=%d, g_temp_input_CC_value = %d\r\n", g_temp_CC_value,g_temp_input_CC_value); 
    }
#endif
/*End, lenovo-sw chailu1 add     20150714 */
/*Begin, lenovo-sw chailu1 add     20150730 */
#ifdef LENOVO_LIMIT_IN_OTA_UPGRADE_SUPPORT 
     if(g_ota_upgrade  == 1)
     {
        if(g_temp_input_CC_value > LENOVO_LIMIT_IN_OTA_UPGRADE_INPUT )
        {
            g_temp_input_CC_value = LENOVO_LIMIT_IN_OTA_UPGRADE_INPUT;
	 }
           
	 if(g_temp_CC_value > LENOVO_LIMIT_IN_OTA_UPGRADE_CHARING )
        {
            g_temp_CC_value = LENOVO_LIMIT_IN_OTA_UPGRADE_CHARING;
	 }
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] g_ota_upgrade : g_temp_CC_value=%d, g_temp_input_CC_value = %d\r\n", g_temp_CC_value,g_temp_input_CC_value); 
    }
#endif
/*Begin, lenovo-sw chailu1 add     20150730 */
		battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Default CC mode charging : %d, input current = %d\r\n",
				    g_temp_CC_value, g_temp_input_CC_value);
		if (g_temp_CC_value == CHARGE_CURRENT_0_00_MA
		    || g_temp_input_CC_value == CHARGE_CURRENT_0_00_MA) {

			charging_enable = KAL_FALSE;

			battery_log(BAT_LOG_CRTI,
					    "[BATTERY] charging current is set 0mA, turn off charging !\r\n");
		} else {
			//lenovo-sw mahj2 modify for ncp1854 charging bug Begin
			#ifdef CONFIG_MTK_NCP1854_SUPPORT
			if(g_pre_CC_value != g_temp_CC_value){
				g_pre_CC_value = g_temp_CC_value;
				charging_enable = KAL_FALSE;
				battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
				charging_enable = KAL_TRUE;
			}
			#endif

	/*Lenovo-sw: AIOROW-4398 thermal limit current*/
	switch(g_thermal_limit_current_level){
	case 0:
		break;
	case 1:
	case 2:
	case 3:
		if(g_temp_CC_value > CHARGE_CURRENT_900_00_MA)
			g_temp_CC_value = CHARGE_CURRENT_900_00_MA ; 
		break;
	default:
		break;
	}
	printk(KERN_ERR "thermal limit level:%d, charger current:%d,input current:%d \n",g_thermal_limit_current_level,g_temp_CC_value,g_temp_input_CC_value);
/*Lenovo-sw: AIOROW-4398 thermal limit current*/
			//lenovo-sw mahj2 modify for ncp1854 charging bug End
			battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,
						 &g_temp_input_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CURRENT, &g_temp_CC_value);

			/*Set CV Voltage */
#if !defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)

/*Begin, lenovo-sw chailu1 add 45-50  CV limit fuction, use 3rd fg  20150318 */
#ifdef LENOVO_TEMP_POS_45_TO_POS_50_CV_LiMIT_SUPPORT
            if(lenovo_battery_is_temp_45_to_pos_50())
            {
                 battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] pchr_turn_on_charging: g_lenovo_cv_limit_stage = 1!\r\n");
		   g_lenovo_cv_limit_stage = 1;		 
	          cv_voltage = LENOVO_TEMP_POS_45_TO_POS_50_CV_VOLTAGE;	      		  
            }		  
	    else
	    {
	         battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] pchr_turn_on_charging: g_lenovo_cv_limit_stage = 0!\r\n");
	         g_lenovo_cv_limit_stage = 0;	
     #ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
                cv_voltage = BATTERY_VOLT_04_350000_V;
    #elif defined(HIGH_BATTERY_VOLTAGE_4400MV_SUPPORT)    
                cv_voltage = BATTERY_VOLT_04_400000_V; 
    #else
	        cv_voltage = BATTERY_VOLT_04_200000_V;
    #endif
	    }
#else    	
    #ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
		  /* lenovo-sw zhangrc2 change cv to 4.35V 2014-08-27 begin */		
            cv_voltage = BATTERY_VOLT_04_350000_V;
    #elif defined(HIGH_BATTERY_VOLTAGE_4400MV_SUPPORT)    
             cv_voltage = BATTERY_VOLT_04_400000_V; //lenovo sw chailu1 add 20150226
    #else
			cv_voltage = BATTERY_VOLT_04_200000_V;
    #endif
#endif
/*End, lenovo-sw chailu1 add 45-50  CV limit fuction, use 3rd fg  20150318*/
			battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);
#endif
		}
	}

	/* enable/disable charging */
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_log(BAT_LOG_FULL, "[BATTERY] pchr_turn_on_charging(), enable =%d !\r\n",
			    charging_enable);
}


PMU_STATUS BAT_PreChargeModeAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] Pre-CC mode charge, timer=%d on %d !!\n\r",
			    BMT_status.PRE_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	/*  Enable charger */
	pchr_turn_on_charging();

	if (BMT_status.UI_SOC == 100) {
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	} else if (BMT_status.bat_vol > V_PRE2CC_THRES) {
		BMT_status.bat_charging_state = CHR_CC;
	}



	return PMU_STATUS_OK;
}


PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
	battery_log(BAT_LOG_FULL, "[BATTERY] CC mode charge, timer=%d on %d !!\n\r",
			    BMT_status.CC_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time += BAT_TASK_PERIOD;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	/*  Enable charger */
	pchr_turn_on_charging();

	if (charging_full_check() == KAL_TRUE) {
	/*Begin sw chailu1 add 20150403*/	
	#ifdef LENOVO_CHARGING_FULL_CHECK_AGAIN_SUPPORT
              if ((g_is_lenovo_charging_check_again_state) &&(BMT_status.SOC <100))
              	{
              	     battery_xlog_printk(BAT_LOG_CRTI, "lenovo charging full check again in\n\r");

                   lenovo_charging_again_enble();
	            g_is_lenovo_charging_check_again_state = KAL_FALSE;			   
	       }
              else
              	{
		    BMT_status.bat_charging_state = CHR_BATFULL;
		    BMT_status.bat_full = KAL_TRUE;
		    g_charging_full_reset_bat_meter = KAL_TRUE;
	       }
	#else
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	#endif
	/*End sw chailu1 add 20150403*/
	}

	return PMU_STATUS_OK;
}

/*lenovo-sw weiweij added for charging terminate as 0.1c*/
#ifdef LENOVO_CHARGING_TERM
extern int lenovo_charging_stage;
//extern int lenovo_charging_charger_type;
extern void lenovo_charging_term_reset_to_stage_2(void);
extern void lenovo_charging_term_set_stage(int stage);
#endif
/*lenovo-sw weiweij added for charging terminate as 0.1c end*/

PMU_STATUS BAT_BatteryFullAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] Battery full !!\n\r");

	BMT_status.bat_full = KAL_TRUE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

/*lenovo-sw weiweij added for charging terminate as 0.1c*/
	#ifdef LENOVO_CHARGING_TERM
 	battery_xlog_printk(BAT_LOG_CRTI, "[LENOVO_CHARGING_TERM] ww_debug stage=%d!soc=%d\n\r", lenovo_charging_stage, BMT_status.UI_SOC);         
 
	if(lenovo_charging_stage==1)
    	{
    	/* lenovo-sw zhangrc2 added for when ui_soc is 100,plug in charger but charing is stop 2014-10-10 */ 
    		if(charging_full_check() == KAL_FALSE && BMT_status.UI_SOC==100)
    		{		
       	 battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] zhangrc2 Battery Re-charging !!\n\r");                
		BMT_status.bat_in_recharging_state = KAL_TRUE;
        	BMT_status.bat_charging_state = CHR_CC;
		battery_meter_reset();
		}
	/* lenovo-sw zhangrc2 added for when ui_soc is 100,plug in charger but charing is stop 2014-10-10 */ 		
			
    		if(BMT_status.UI_SOC==100)
    		{
	        	battery_xlog_printk(BAT_LOG_CRTI, "[LENOVO_CHARGING_TERM] Battery charging stage 2!!\n\r");                

	//		BMT_status.bat_charging_state = CHR_CC;

			lenovo_charging_term_reset_to_stage_2();
			//pchr_turn_on_charging();
			//battery_meter_reset();
    		}else
		{
			battery_xlog_printk(BAT_LOG_CRTI, "[LENOVO_CHARGING_TERM] Battery charging ui_soc(%d)!=100 wait stage 2!!\n\r", BMT_status.UI_SOC);  
		}
	}else if(lenovo_charging_stage==2)
	{
		if(charging_full_check() == KAL_TRUE)
		{
			battery_xlog_printk(BAT_LOG_CRTI, "[LENOVO_CHARGING_TERM] Battery charging stage 2 complete!\n\r");  
			lenovo_charging_term_set_stage(3);
			BMT_status.bat_charging_state = CHR_BATFULL;
		}
	}else
#endif
/*lenovo-sw weiweij added for charging terminate as 0.1c end*/
//lenovo-sw mahj2 modify Begin
	if (charging_full_check() == KAL_FALSE && BMT_status.UI_SOC==100) {
//lenovo-sw mahj2 modify End
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Re-charging !!\n\r");

		BMT_status.bat_in_recharging_state = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_CC;
		battery_meter_reset();
	}

/*Begin, lenovo-sw chailu1 add 45-50  CV limit fuction, use 3rd fg  20150318 */
#ifdef LENOVO_TEMP_POS_45_TO_POS_50_CV_LiMIT_SUPPORT
        if(lenovo_battery_is_temp_45_to_pos_50()) 
        {
            if (1 == g_lenovo_cv_limit_stage)
            {
                 battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] :BAT_BatteryFullAction g_lenovo_cv_limit_stage = 2!\r\n");
	          g_lenovo_cv_limit_stage = 2;
            }		  
        }
	else if (2 == g_lenovo_cv_limit_stage)	
	{
             battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] :BAT_BatteryFullAction g_lenovo_cv_limit_stage = 3!\r\n");
	       g_lenovo_cv_limit_stage = 3;	
		BMT_status.bat_charging_state = CHR_CC; 
		pchr_turn_on_charging();
		
	       if(BMT_status.UI_SOC==100)
	       	{
                  BMT_status.bat_in_recharging_state = KAL_TRUE;
		}	
		else
	       {
	           battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] :BAT_BatteryFullAction g_lenovo_cv_limit_stage = 3, ui_soc != 100!\r\n");
                  BMT_status.bat_full = KAL_FALSE;
	           BMT_status.bat_in_recharging_state = KAL_FALSE;			  
		}
		
       }	
#endif
/*End,lenovo-sw chailu1 add 45-50  CV limit fuction, use 3rd fg  20150318 */	
	
	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryHoldAction(void)
{
	kal_uint32 charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Hold mode !!\n\r");

	if (BMT_status.bat_vol < TALKING_RECHARGE_VOLTAGE || g_call_state == CALL_IDLE) {
		BMT_status.bat_charging_state = CHR_CC;
		battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Exit Hold mode and Enter CC mode !!\n\r");
	}

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryStatusFailAction(void)
{
	kal_uint32 charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] BAD Battery status... Charging Stop !!\n\r");

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	if ((g_temp_status == TEMP_ABOVE_POS_60) || (g_temp_status == TEMP_BELOW_NEG_10)) {
		temp_error_recovery_chr_flag = KAL_FALSE;
	}
	if ((temp_error_recovery_chr_flag == KAL_FALSE) && (g_temp_status != TEMP_ABOVE_POS_60)
	    && (g_temp_status != TEMP_BELOW_NEG_10)) {
		temp_error_recovery_chr_flag = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_PRE;
	}
#endif

	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}


void mt_battery_charging_algorithm(void)
{
	battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	battery_pump_express_charger_check();
#endif

/*Begin sw chailu1 add 20150403*/
#ifdef LENOVO_CHARGING_FULL_CHECK_AGAIN_SUPPORT
        if ((BMT_status.bat_charging_state == CHR_CC ) &&( (g_bat_charging_state_back == CHR_PRE ) ||(g_bat_charging_state_back == CHR_BATFULL )))
	 {
	     g_is_lenovo_charging_check_again_state = KAL_TRUE;
	     battery_xlog_printk(BAT_LOG_CRTI, "g_is_lenovo_charging_check_again_state = true \r\n");	 
        }	 
		
        g_bat_charging_state_back = BMT_status.bat_charging_state;        
#endif
/*End sw chailu1 add 20150403*/	
	switch (BMT_status.bat_charging_state) {
	case CHR_PRE:
		BAT_PreChargeModeAction();
		break;

	case CHR_CC:
		BAT_ConstantCurrentModeAction();
		break;

	case CHR_BATFULL:
		BAT_BatteryFullAction();
		break;

	case CHR_HOLD:
		BAT_BatteryHoldAction();
		break;

	case CHR_ERROR:
		BAT_BatteryStatusFailAction();
		break;
	}

	battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
}
