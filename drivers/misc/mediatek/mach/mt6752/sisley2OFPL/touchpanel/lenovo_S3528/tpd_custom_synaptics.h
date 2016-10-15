#ifndef __S3528_TOUCHPANEL_CONFIG_H__
#define __S3528_TOUCHPANEL_CONFIG_H__
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>

#include <cust_eint.h>
#include <linux/jiffies.h>
#include <pmic_drv.h>
#include <cust_i2c.h>

#ifdef CONFIG_LENOVO_CTP_FEATURE

#define LENOVO_CTP_GLOVE_CONTROL
//#define LENOVO_CTP_GLOVE_CONTROL_EXTBTN
#define LENOVO_GESTURE_WAKEUP
#define LENOVO_AREA_TOUCH

#endif
#define LENOVO_CTP_ESD_CHECK
#define TPD_CLOSE_POWER_IN_SLEEP

#define S3203_SUPPORT
//#define SPECIAL_REPORT_LOG	//it is define fot Rex Chen log analyse

/* if 3203 need support, please set which family id is 3203 here
	ex. : family id 2 and 3 are 3203, now set S3203_BIT as:
			#define S3203_BIT ((1 << 2) | (1 << 3))
	family id refer to : tpd_synaptics_upgrade.h
*/
#ifdef S3203_SUPPORT

#define S3203_BIT	((1 << 2) | (1 << 3))

#else
#define S3203_BIT	0
#endif

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE

//#define TPD_PWRBY_PMIC
#define TPD_PWRBY_GPIO

#ifdef TPD_PWRBY_PMIC
#define TPD_POWER_SOURCE	MT6331_POWER_LDO_VGP1//MT6323_POWER_LDO_VGP1         
#endif

#ifdef TPD_PWRBY_GPIO
#define TPD_POWER_GPIO	GPIO_CTP_EN_PIN         
#endif

#define TPD_I2C_BUS		I2C_CAP_TOUCH_CHANNEL//2 // 1
#define TPD_I2C_ADDR		0x38 //0x22
#define TPD_WAKEUP_TRIAL	60
#define TPD_WAKEUP_DELAY	100


//#define TPD_HAVE_TREMBLE_ELIMINATION

/* Define the virtual button mapping */
//#define TPD_HAVE_BUTTON
#define TPD_BUTTON_HEIGH        (100)
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_MENU,KEY_HOMEPAGE,KEY_BACK}
#define TPD_KEYS_DIM            {{223,2000,80,TPD_BUTTON_HEIGH},{493,2000,80,TPD_BUTTON_HEIGH},{853,2000,80,TPD_BUTTON_HEIGH}}
// {{80,850,160,TPD_BUTTON_HEIGH},{240,850,160,TPD_BUTTON_HEIGH},{400,850,160,TPD_BUTTON_HEIGH}}

/* Define the touch dimension */
#ifdef TPD_HAVE_BUTTON
#define TPD_TOUCH_HEIGH_RATIO	80
#define TPD_DISPLAY_HEIGH_RATIO	73
#endif

/* Define the 0D button mapping */
#ifdef TPD_HAVE_BUTTON
#define TPD_0D_BUTTON		{NULL} //changed by xuwen1 for 3528
#else
	#ifdef LENOVO_CTP_GLOVE_CONTROL_EXTBTN
		#ifdef CONFIG_LENOVO_CTP_APPSELECT/*replace KEY_MENU with KEY_APPSELECT*/
		#define TPD_0D_BUTTON		{KEY_APPSELECT,KEY_HOMEPAGE,KEY_BACK,KEY_CAMERA}//changed by liuyw2 double click camera
		#else
		#define TPD_0D_BUTTON		{KEY_MENU,KEY_HOMEPAGE,KEY_BACK,KEY_CAMERA}//changed by liuyw2 double click camera
		#endif
	#else
		#ifdef CONFIG_LENOVO_CTP_APPSELECT/*replace KEY_MENU with KEY_APPSELECT*/
		#define TPD_0D_BUTTON		{KEY_APPSELECT,KEY_HOMEPAGE,KEY_BACK}//changed by liuyw2 double click camera
		#else
		#define TPD_0D_BUTTON		{KEY_MENU,KEY_HOMEPAGE,KEY_BACK}//changed by liuyw2 double click camera
		#endif
	#endif
#endif

#define	TOUCH_FILTER	0	//Touch filter algorithm
#if	TOUCH_FILTER
#define TPD_FILTER_PARA	{1, 174}	//{enable, pixel density}
#endif

#endif /* __S3528_TOUCHPANEL_CONFIG_H__ */

