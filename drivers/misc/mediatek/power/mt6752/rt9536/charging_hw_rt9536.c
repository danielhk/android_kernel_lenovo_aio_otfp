#include <mach/charging.h>
#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_gpio.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
#include <cust_gpio_usage.h>
#include <cust_charging.h>
#include "rt9536.h"

/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK    0
#define STATUS_UNSUPPORTED    -1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))
#define battery_log(num, fmt, args...)

/* ============================================================ // */
/* Global variable */
/* ============================================================ // */

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0

#if defined(GPIO_PWR_AVAIL_WLC)
kal_uint32 wireless_charger_gpio_number = GPIO_PWR_AVAIL_WLC;
#else
kal_uint32 wireless_charger_gpio_number = 0;
#endif

#endif

static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;

kal_bool charging_type_det_done = KAL_TRUE;

const kal_uint32 VBAT_CV_VTH[] = {
	BATTERY_VOLT_03_500000_V, BATTERY_VOLT_03_520000_V, BATTERY_VOLT_03_540000_V,
	    BATTERY_VOLT_03_560000_V, BATTERY_VOLT_03_580000_V,
	BATTERY_VOLT_03_600000_V, BATTERY_VOLT_03_620000_V, BATTERY_VOLT_03_640000_V,
	    BATTERY_VOLT_03_660000_V, BATTERY_VOLT_03_680000_V,
	BATTERY_VOLT_03_700000_V, BATTERY_VOLT_03_720000_V, BATTERY_VOLT_03_740000_V,
	    BATTERY_VOLT_03_760000_V, BATTERY_VOLT_03_780000_V,
	BATTERY_VOLT_03_800000_V, BATTERY_VOLT_03_820000_V, BATTERY_VOLT_03_840000_V,
	    BATTERY_VOLT_03_860000_V, BATTERY_VOLT_03_880000_V,
	BATTERY_VOLT_03_900000_V, BATTERY_VOLT_03_920000_V, BATTERY_VOLT_03_940000_V,
	    BATTERY_VOLT_03_960000_V, BATTERY_VOLT_03_980000_V,
	BATTERY_VOLT_04_000000_V, BATTERY_VOLT_04_020000_V, BATTERY_VOLT_04_040000_V,
	    BATTERY_VOLT_04_060000_V, BATTERY_VOLT_04_080000_V,
	BATTERY_VOLT_04_100000_V, BATTERY_VOLT_04_120000_V, BATTERY_VOLT_04_140000_V,
	    BATTERY_VOLT_04_160000_V, BATTERY_VOLT_04_180000_V,
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_220000_V, BATTERY_VOLT_04_240000_V,
	    BATTERY_VOLT_04_260000_V, BATTERY_VOLT_04_280000_V,
	BATTERY_VOLT_04_300000_V, BATTERY_VOLT_04_320000_V, BATTERY_VOLT_04_340000_V,
	    BATTERY_VOLT_04_360000_V, BATTERY_VOLT_04_380000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_420000_V, BATTERY_VOLT_04_440000_V,
	    BATTERY_VOLT_04_460000_V, BATTERY_VOLT_04_480000_V,
	BATTERY_VOLT_04_500000_V, BATTERY_VOLT_04_520000_V, BATTERY_VOLT_04_540000_V,
	    BATTERY_VOLT_04_560000_V, BATTERY_VOLT_04_580000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_04_620000_V, BATTERY_VOLT_04_640000_V,
	    BATTERY_VOLT_04_660000_V, BATTERY_VOLT_04_680000_V,
	BATTERY_VOLT_04_700000_V, BATTERY_VOLT_04_720000_V, BATTERY_VOLT_04_740000_V,
	    BATTERY_VOLT_04_760000_V
};

const kal_uint32 CS_VTH[] = {
	CHARGE_CURRENT_500_00_MA, CHARGE_CURRENT_600_00_MA, CHARGE_CURRENT_700_00_MA,
	    CHARGE_CURRENT_800_00_MA, CHARGE_CURRENT_900_00_MA,
	CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1100_00_MA, CHARGE_CURRENT_1200_00_MA,
	    CHARGE_CURRENT_1300_00_MA, CHARGE_CURRENT_1400_00_MA,
	CHARGE_CURRENT_1500_00_MA, CHARGE_CURRENT_1600_00_MA, CHARGE_CURRENT_1700_00_MA,
	    CHARGE_CURRENT_1800_00_MA, CHARGE_CURRENT_1900_00_MA,
	CHARGE_CURRENT_2000_00_MA, CHARGE_CURRENT_2100_00_MA, CHARGE_CURRENT_2200_00_MA,
	    CHARGE_CURRENT_2300_00_MA, CHARGE_CURRENT_2400_00_MA,
	CHARGE_CURRENT_2500_00_MA, CHARGE_CURRENT_2600_00_MA, CHARGE_CURRENT_2700_00_MA,
	    CHARGE_CURRENT_2800_00_MA, CHARGE_CURRENT_2900_00_MA,
	CHARGE_CURRENT_3000_00_MA, CHARGE_CURRENT_MAX
	    /* from datasheet : any setting programmed above 3A selects the 3A setting */
};

/* USB connector (USB or AC adaptor) */
const kal_uint32 INPUT_CS_VTH[] = {
	CHARGE_CURRENT_100_00_MA, CHARGE_CURRENT_150_00_MA, CHARGE_CURRENT_500_00_MA,
	    CHARGE_CURRENT_900_00_MA,
	CHARGE_CURRENT_1500_00_MA, CHARGE_CURRENT_1950_00_MA, CHARGE_CURRENT_2500_00_MA,
	    CHARGE_CURRENT_2000_00_MA,
	CHARGE_CURRENT_MAX
};

const kal_uint32 VCDT_HV_VTH[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};

kal_uint32 g_charging_current = 0;
/* ============================================================ // */
/* function prototype */
/* ============================================================ // */


/* ============================================================ // */
/* extern variable */
/* ============================================================ // */

/* ============================================================ // */
/* extern function */
/* ============================================================ // */


/* ============================================================ // */
kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size,
				       const kal_uint32 val)
{
	if (val < array_size) {
		return parameter[val];
	} else {
		battery_log(BAT_LOG_CRTI, "Can't find the parameter \r\n");
		return parameter[0];
	}
}


kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size,
				       const kal_uint32 val)
{
	kal_uint32 i;

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	battery_log(BAT_LOG_CRTI, "NO register value match \r\n");
	/* TODO: ASSERT(0);    // not find the value */
	return 0;
}


static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList, kal_uint32 number,
					 kal_uint32 level)
{
	kal_uint32 i;
	kal_uint32 max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--) {
			if (pList[i] <= level)
				return pList[i];
		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++) {
			if (pList[i] <= level)
				return pList[i];
		}

		battery_log(BAT_LOG_CRTI, "Can't find closest level \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}


static kal_uint32 is_chr_det(void)
{
	kal_uint32 val = 0;

	val = mt6325_upmu_get_rgs_chrdet();

	battery_log(BAT_LOG_CRTI, "[is_chr_det] %d\n", val);

	return val;
}


static kal_uint32 charging_hw_init(void *data)
{
	kal_uint32 status = STATUS_OK;

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	if (wireless_charger_gpio_number != 0) {
		mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
		mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
	}
#endif

	return status;
}


static kal_uint32 charging_dump_register(void *data)
{
	kal_uint32 status = STATUS_OK;

	return status;
}


static kal_uint32 charging_enable(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32 *) (data);

	if (KAL_TRUE == enable) {
		rt9536_charging_enable(g_charging_current, enable);
	} else {
		g_charging_current = 0;
		rt9536_charging_enable(g_charging_current, enable);
	}

	battery_log(BAT_LOG_CRTI,
		    "[%s][charger_rt9536] charger enable = %d, g_charging_current = %d \r\n",
		    __func__, enable, g_charging_current);

	return status;
}


static kal_uint32 charging_set_cv_voltage(void *data)
{
	kal_uint32 status = STATUS_OK;
	/* kal_uint32 cv_value = *(kal_uint32 *)(data); */

	/* TO DO */
	battery_log(BAT_LOG_FULL, "[%s][charger_rt9536] NO Action! \r\n", __func__);

	return status;
}


static kal_uint32 charging_get_current(void *data)
{
	kal_uint32 status = STATUS_OK;

	/* TO DO */
	battery_log(BAT_LOG_CRTI, "[%s][charger_rt9536] g_charging_current = %d! \r\n", __func__,
		    g_charging_current);

	return status;
}


static kal_uint32 charging_set_current(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 current_value = *(kal_uint32 *) data;

	if ((current_value == AC_CHARGER_CURRENT) || (current_value == USB_CHARGER_CURRENT))
		g_charging_current = current_value;
	else
		g_charging_current = USB_CHARGER_CURRENT;

	battery_log(BAT_LOG_CRTI,
		    "[%s][charger_rt9536] current_value = %d,     g_charging_current = %d \r\n",
		    __func__, current_value, g_charging_current);

	return status;
}


static kal_uint32 charging_set_input_current(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 current_value = *(kal_uint32 *) data;

	battery_log(BAT_LOG_CRTI, "[%s][charger_rt9536] input_current_value = %d \r\n", __func__,
		    current_value);

	return status;
}


static kal_uint32 charging_get_charging_status(void *data)
{
	kal_uint32 status = STATUS_OK;

	*(kal_uint32 *) data = rt9536_check_eoc_status();

	battery_log(BAT_LOG_CRTI, "[%s][charger_rt9536] EOC_status = %d \r\n", __func__,
		    *(kal_uint32 *) data);

	return status;
}


static kal_uint32 charging_reset_watch_dog_timer(void *data)
{
	kal_uint32 status = STATUS_OK;

	/* TO DO */
	battery_log(BAT_LOG_FULL, "[%s][charger_rt9536] NO Action! \r\n", __func__);

	return status;
}


static kal_uint32 charging_set_hv_threshold(void *data)
{
	kal_uint32 status = STATUS_OK;

	kal_uint32 set_hv_voltage;
	kal_uint32 array_size;
	kal_uint16 register_value;
	kal_uint32 voltage = *(kal_uint32 *) (data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size, set_hv_voltage);
	mt6325_upmu_set_rg_vcdt_hv_vth(register_value);

	return status;
}


static kal_uint32 charging_get_hv_status(void *data)
{
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;
	battery_log(BAT_LOG_CRTI, "[charging_get_hv_status] charger ok for bring up.\n");
#else
	*(kal_bool *) (data) = mt6325_upmu_get_rgs_vcdt_hv_det();
#endif

	return status;
}


static kal_uint32 charging_get_battery_status(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 val = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;	/* battery exist */
	battery_log(BAT_LOG_CRTI, "[charging_get_battery_status] battery exist for bring up.\n");
#else
	pmic_read_interface(MT6325_CHR_CON7, &val, MT6325_PMIC_BATON_TDET_EN_MASK,
			    MT6325_PMIC_BATON_TDET_EN_SHIFT);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		mt6325_upmu_set_baton_tdet_en(1);
		mt6325_upmu_set_rg_baton_en(1);
		*(kal_bool *) (data) = mt6325_upmu_get_rgs_baton_undet();
	} else {
		*(kal_bool *) (data) = KAL_FALSE;
	}
#endif

	return status;
}


static kal_uint32 charging_get_charger_det_status(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 val = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	val = 1;
	battery_log(BAT_LOG_CRTI,
		    "[charging_get_charger_det_status] charger exist for bring up.\n");
#else
	val = mt6325_upmu_get_rgs_chrdet();
#endif

	*(kal_bool *) (data) = val;
	if (val == 0)
		g_charger_type = CHARGER_UNKNOWN;

	return status;
}


kal_bool charging_type_detection_done(void)
{
	return charging_type_det_done;
}


static kal_uint32 charging_get_charger_type(void *data)
{
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	int wireless_state = 0;

	if (wireless_charger_gpio_number != 0) {
		wireless_state = mt_get_gpio_in(wireless_charger_gpio_number);
		if (wireless_state == WIRELESS_CHARGER_EXIST_STATE) {
			*(CHARGER_TYPE *) (data) = WIRELESS_CHARGER;
			battery_log(BAT_LOG_CRTI, "WIRELESS_CHARGER!\n");
			return status;
		}
	} else {
		battery_log(BAT_LOG_CRTI, "wireless_charger_gpio_number=%d\n",
			    wireless_charger_gpio_number);
	}

	if (g_charger_type != CHARGER_UNKNOWN && g_charger_type != WIRELESS_CHARGER) {
		*(CHARGER_TYPE *) (data) = g_charger_type;
		battery_log(BAT_LOG_CRTI, "return %d!\n", g_charger_type);
		return status;
	}
#endif

	if (is_chr_det() == 0) {
		g_charger_type = CHARGER_UNKNOWN;
		*(CHARGER_TYPE *) (data) = CHARGER_UNKNOWN;
		battery_log(BAT_LOG_CRTI, "[charging_get_charger_type] return CHARGER_UNKNOWN\n");
		return status;
	}

	charging_type_det_done = KAL_FALSE;

	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
	/* *(CHARGER_TYPE*)(data) = STANDARD_HOST; */
	/* *(CHARGER_TYPE*)(data) = STANDARD_CHARGER; */

	charging_type_det_done = KAL_TRUE;

	g_charger_type = *(CHARGER_TYPE *) (data);

#endif

	return status;
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = KAL_FALSE;
#else
	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;

	battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
#endif

	return status;
}

static kal_uint32 charging_set_platform_reset(void *data)
{
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");

	arch_reset(0, NULL);
#endif

	return status;
}

static kal_uint32 charging_get_platfrom_boot_mode(void *data)
{
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	*(kal_uint32 *) (data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif

	return status;
}

static kal_uint32 charging_set_power_off(void *data)
{
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_power_off\n");
	mt_power_off();
#endif

	return status;
}

static kal_uint32 charging_get_power_source(void *data)
{
	kal_uint32 status = STATUS_OK;

#if 0				/* #if defined(MTK_POWER_EXT_DETECT) */
	if (MT_BOARD_PHONE == mt_get_board_type())
		*(kal_bool *) data = KAL_FALSE;
	else
		*(kal_bool *) data = KAL_TRUE;
#else
	*(kal_bool *) data = KAL_FALSE;
#endif

	return status;
}

static kal_uint32 charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

static kal_uint32 charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;
}

static kal_uint32(*const charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
charging_hw_init, charging_dump_register, charging_enable, charging_set_cv_voltage,
	    charging_get_current, charging_set_current, charging_set_input_current,
	    charging_get_charging_status, charging_reset_watch_dog_timer,
	    charging_set_hv_threshold, charging_get_hv_status, charging_get_battery_status,
	    charging_get_charger_det_status, charging_get_charger_type,
	    charging_get_is_pcm_timer_trigger, charging_set_platform_reset,
	    charging_get_platfrom_boot_mode, charging_set_power_off,
	    charging_get_power_source, charging_get_csdac_full_flag,
	    charging_set_ta_current_pattern};


/*
* FUNCTION
*        Internal_chr_control_handler
*
* DESCRIPTION
*         This function is called to set the charger hw
*
* CALLS
*
* PARAMETERS
*        None
*
* RETURNS
*
*
* GLOBALS AFFECTED
*       None
*/
kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	kal_int32 status;

	if (cmd < CHARGING_CMD_NUMBER)
		status = charging_func[cmd] (data);
	else
		return STATUS_UNSUPPORTED;

	return status;
}
