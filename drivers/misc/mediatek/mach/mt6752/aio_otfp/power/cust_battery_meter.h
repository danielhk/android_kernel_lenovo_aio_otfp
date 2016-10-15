#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
//#define SOC_BY_AUXADC
#define SOC_BY_HW_FG
//#define SOC_BY_SW_FG
//#define HW_FG_FORCE_USE_SW_OCV

//#define CONFIG_DIS_CHECK_BATTERY
//#define FIXED_TBAT_25

/* ADC Channel Number */
#if 0
#define CUST_TABT_NUMBER 17
#define VBAT_CHANNEL_NUMBER      7
#define ISENSE_CHANNEL_NUMBER	 6
#define VCHARGER_CHANNEL_NUMBER  4
#define VBATTEMP_CHANNEL_NUMBER  5
#endif
/* ADC resistor  */
#define R_BAT_SENSE 4					
#define R_I_SENSE 4						
#define R_CHARGER_1 330
#define R_CHARGER_2 39

#define TEMPERATURE_T0             110
#define TEMPERATURE_T1             0
#define TEMPERATURE_T2             25
#define TEMPERATURE_T3             50
#define TEMPERATURE_T              255 // This should be fixed, never change the value

//lenovo-sw mahj2 modify Begin
#define FG_METER_RESISTANCE 	50
//lenovo-sw mahj2 modify End
/* Begin, lenovo-sw mahj2 update battery param at 20141021*/
/* Qmax for battery  */
#define Q_MAX_POS_50	2798
#define Q_MAX_POS_25	2777
#define Q_MAX_POS_0		2518
#define Q_MAX_NEG_10	1698

#define Q_MAX_POS_50_H_CURRENT	2796
#define Q_MAX_POS_25_H_CURRENT	2761
#define Q_MAX_POS_0_H_CURRENT	2489
#define Q_MAX_NEG_10_H_CURRENT	1681
/* End, lenovo-sw mahj2 update battery param at 20141021*/

/* Discharge Percentage */
#define OAM_D5		 1		//  1 : D5,   0: D2


/* battery meter parameter */
#define CHANGE_TRACKING_POINT
/*Begin, lenovo-sw mahj2 modify at 20141105 */
//#define CUST_TRACKING_POINT  1
#define CUST_TRACKING_POINT  20
/*End, lenovo-sw mahj2 modify at 20141105 */
/*Begin, lenovo-sw mahj2 modify for cust r sense at 20141011 */
//#define CUST_R_SENSE         68
#ifdef CONFIG_MTK_BQ24157_SUPPORT
#define CUST_R_SENSE         56 //33mohm
#else
#define CUST_R_SENSE         33 //33mohm
#endif
/*end, lenovo-sw mahj2 modify for cust r sense at 20141011 */
#define CUST_HW_CC 		    0
#define AGING_TUNING_VALUE   103
#define CUST_R_FG_OFFSET    0

#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
/*Begin, lenovo-sw mahj2 modify at 20141105*/
//#define CAR_TUNE_VALUE		97 //1.00
#define CAR_TUNE_VALUE		103 //1.00
/*end, lenovo-sw mahj2 modify at 20141105 */


/* HW Fuel gague  */
#define CURRENT_DETECT_R_FG	10  //1mA
#define MinErrorOffset       1000
#define FG_VBAT_AVERAGE_SIZE 18
#define R_FG_VALUE 			10 // mOhm, base is 20
/* lenovo-sw zhangrc2 change tolrance from 25 to 40 2014-11-06 */
#define CUST_POWERON_DELTA_CAPACITY_TOLRANCE	40
#define CUST_POWERON_LOW_CAPACITY_TOLRANCE		2
/* lenovo-sw zhangrc2 change tolrance from 25 to 40 2014-11-06 */
#define CUST_POWERON_MAX_VBAT_TOLRANCE			90
#define CUST_POWERON_DELTA_VBAT_TOLRANCE		30

/* lenovo-sw zhangrc2 close FIXED_TBAT_25 2014-08-19 begin, mahj2 porting */
/* Disable Battery check for HQA */
#ifdef CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
//#define FIXED_TBAT_25
#endif
/* lenovo-sw zhangrc2 close FIXED_TBAT_25 2014-08-19 end, mahj2 porting */

/* Dynamic change wake up period of battery thread when suspend*/
/* lenovo-sw zhangrc2 change time 2014-10-10 Begin*/
#define VBAT_NORMAL_WAKEUP		3700		//3.7V
#define VBAT_LOW_POWER_WAKEUP		3550		//3.55v
#define NORMAL_WAKEUP_PERIOD		2700		//45 * 60 = 90 min
#define LOW_POWER_WAKEUP_PERIOD		240		//4* 60 = 4 min
#define CLOSE_POWEROFF_WAKEUP_PERIOD	30	//30 s
/* lenovo-sw zhangrc2 change time 2014-10-10 End*/
#define INIT_SOC_BY_SW_SOC
//#define SYNC_UI_SOC_IMM			//3. UI SOC sync to FG SOC immediately
#define MTK_ENABLE_AGING_ALGORITHM	//6. Q_MAX aging algorithm
#define MD_SLEEP_CURRENT_CHECK	//5. Gauge Adjust by OCV 9. MD sleep current check
#define Q_MAX_BY_CURRENT		//7. Qmax varient by current loading.

#define DISABLE_RFG_EXIST_CHECK
#endif	//#ifndef _CUST_BATTERY_METER_H
