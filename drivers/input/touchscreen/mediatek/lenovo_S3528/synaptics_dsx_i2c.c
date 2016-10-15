/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <cust_eint.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include "cust_gpio_usage.h"
#include "tpd.h"
#include <linux/sched.h>

#include "synaptics_dsx_i2c.h"
#include "synaptics_dsx.h"
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif
#ifdef CONFIG_LENOVO_POWEROFF_CHARGING_UI
#include <cust_charging.h>
#endif

#define TPD_DEBUG_MORE	0
#define TPD_DEBUG_CORE	1 

#if TPD_DEBUG_MORE
#define __dbg(X...)	 printk(KERN_INFO "[S3528][DRV]"X)
#else
#define __dbg(X...)
#endif

#if TPD_DEBUG_CORE
#define __dbg_core(X...)	 printk(KERN_INFO "[S3528][DRV]"X)
#else
#define __dbg_core(X...)
#endif

extern struct tpd_device *tpd;

static unsigned char bypass_suspend = 0;
extern unsigned char syna_fwu_upgrade_progress;
/*lenovo-xw xuwen1 add 20140506 for start work begin*/
/*lenovo-sw xuwen1 add for exeit begin*/
/*
#if !defined(__devinit)
#define __devinit
#endif
#if !defined(__devexit)
#define __devexit
#endif
#if !defined(__devexit_p)
#define __devexit_p(x) (&(x))
#endif*/
/*lenovo-sw xuwen1 add for exeit end*/
#define DRIVER_NAME "mtk-tpd"	//"synaptics_dsx_i2c", modify for sync the input device, or the factory mode is fail
#define INPUT_PHYS_NAME "mtk-tpd/input0"	//"synaptics_dsx_i2c/input0"

#ifdef LENOVO_GESTURE_WAKEUP

static struct tpd_gesture_t gesture_func;
static int lpwg_flag = 0;//use for lpwg func match suspend and resume
static int lpwg_int_flag = 0;//use for flag eint happened

#ifndef KEY_SLIDE
#define KEY_SLIDE 254
#endif

struct synaptics_rmi4_3203_gesture_reg {
	unsigned char ctrl;
	unsigned short data_0d;
	unsigned char data_2d;
} s3203_gesture_reg;

#endif

/*lenovo-sw xuwen1 add 20140625 for glove mode begin*/
#ifdef LENOVO_CTP_GLOVE_CONTROL
static struct  tpd_glove_t glove_func;

static int set_glove_mode_func (bool flag);
static int get_glove_mode_func(void);

enum e_finger_status {
	FS_FINGER = 0,
	FS_GLOVE,
	FS_SWITCH_FTOG,
	FS_SWITCH_GTOF,
	FS_INIT
};
static unsigned char gf_current = FS_INIT;
static int glove_debounce_cnt = 20;//60 ints
static int glove_debounce = 20;//60 ints

#endif
/*lenovo-sw xuwen1 add 20140625 for glove mode end*/
/*lenovo-sw liuyw2 20140729 add ctp esd check */

#ifdef LENOVO_CTP_ESD_CHECK
#define TPD_ESD_CHECK_DELAY       2000//2s
static struct delayed_work tpd_esd_check_dwork;
static struct workqueue_struct *tpd_esd_check_wq = NULL;
static unsigned char tpd_esd_doing_reset = 0;
static void tpd_esd_check_func(struct work_struct *);
static void tpd_esd_do_reset(struct synaptics_rmi4_data *rmi4_data);
static int tpd_esd_check_status(unsigned char status);
#endif

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL	//use the A protocol for mediatek factory test
#endif

#define NO_0D_WHILE_2D
/*
#define REPORT_2D_Z
*/
#define REPORT_2D_W

#define F12_DATA_15_WORKAROUND

/*
#define IGNORE_FN_INIT_FAILURE
*/

#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define EXP_FN_WORK_DELAY_MS 1000 /* ms */
#define SYN_I2C_RETRY_TIMES 10
#define MAX_F11_TOUCH_WIDTH 15

#define CHECK_STATUS_TIMEOUT_MS 100
#define DELAY_S7300_BOOT_READY  160
#define DELAY_S7300_RESET       20
#define DELAY_S7300_RESET_READY 90
#define I2C_DMA_LIMIT 252

#define F01_STD_QUERY_LEN 21
#define F01_BUID_ID_OFFSET 18
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12

#define STATUS_NO_ERROR 0x00
#define STATUS_RESET_OCCURRED 0x01
#define STATUS_INVALID_CONFIG 0x02
#define STATUS_DEVICE_FAILURE 0x03
#define STATUS_CONFIG_CRC_FAILURE 0x04
#define STATUS_FIRMWARE_CRC_FAILURE 0x05
#define STATUS_CRC_IN_PROGRESS 0x06

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)
#define CONFIGURED (1 << 7)

// for MTK

#ifdef CONFIG_LENOVO_POWEROFF_CHARGING_UI //lixh10 add
extern struct input_dev *kpd_input_dev;
extern  int ipo_flag;
int tp_button_flag = 0;
extern int g_tp_poweron;
#endif

static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_halt = 0; 
static int tpd_flag = 0;
#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
static u8 boot_mode;

#ifdef	TPD_HAVE_BUTTON
#define TPD_BUTTON_REPORT_XY		//report the virtual key x and y,mediatek
#endif

#if TOUCH_FILTER	//Touch filter algorithm,mediatek
extern struct tpd_filter_t tpd_filter;
static struct tpd_filter_t tpd_filter_local = TPD_FILTER_PARA;
#endif

//[mli,2014-02-19::exlusive isr report & suspend free fingers
DEFINE_MUTEX( rmi4_report_mutex );
//mli]
// for DMA accessing
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
struct i2c_msg *read_msg;
static struct device* g_dev = NULL; 

// for 0D button
static unsigned short cap_button_codes[] = TPD_0D_BUTTON;
static struct synaptics_dsx_cap_button_map cap_button_map = {
	.nbuttons = ARRAY_SIZE(cap_button_codes),
	.map = cap_button_codes,
};
#define MAX_KEY_NUM ( sizeof(cap_button_codes)/sizeof( cap_button_codes[0] ) )

// extern function
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flag,  void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);
static void tpd_eint_handler(void);
static int touch_event_handler(void *data);

static int  synaptics_rmi4_probe(struct i2c_client *client,const struct i2c_device_id *dev_id);
static int  synaptics_rmi4_remove(struct i2c_client *client);

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28);

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static void synaptics_rmi4_early_suspend(struct early_suspend *h);

static void synaptics_rmi4_late_resume(struct early_suspend *h);
#endif

static int synaptics_rmi4_suspend(struct device *dev);

static int synaptics_rmi4_resume(struct device *dev);

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#ifdef LENOVO_CTP_GLOVE_CONTROL
static ssize_t synaptics_rmi4_show_glove_debounce(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_glove_debounce_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#endif

static void tpd_power_ctrl(int en);
static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable);
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable);

extern unsigned int tpd_update_extra_param();
struct synaptics_rmi4_f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query6;
			struct {
				unsigned char ctrl0_is_present:1;
				unsigned char ctrl1_is_present:1;
				unsigned char ctrl2_is_present:1;
				unsigned char ctrl3_is_present:1;
				unsigned char ctrl4_is_present:1;
				unsigned char ctrl5_is_present:1;
				unsigned char ctrl6_is_present:1;
				unsigned char ctrl7_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl8_is_present:1;
				unsigned char ctrl9_is_present:1;
				unsigned char ctrl10_is_present:1;
				unsigned char ctrl11_is_present:1;
				unsigned char ctrl12_is_present:1;
				unsigned char ctrl13_is_present:1;
				unsigned char ctrl14_is_present:1;
				unsigned char ctrl15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl16_is_present:1;
				unsigned char ctrl17_is_present:1;
				unsigned char ctrl18_is_present:1;
				unsigned char ctrl19_is_present:1;
				unsigned char ctrl20_is_present:1;
				unsigned char ctrl21_is_present:1;
				unsigned char ctrl22_is_present:1;
				unsigned char ctrl23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl24_is_present:1;
				unsigned char ctrl25_is_present:1;
				unsigned char ctrl26_is_present:1;
				unsigned char ctrl27_is_present:1;
				unsigned char ctrl28_is_present:1;
				unsigned char ctrl29_is_present:1;
				unsigned char ctrl30_is_present:1;
				unsigned char ctrl31_is_present:1;
			} __packed;
		};
		unsigned char data[5];
	};
};

struct synaptics_rmi4_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query9;
			struct {
				unsigned char data0_is_present:1;
				unsigned char data1_is_present:1;
				unsigned char data2_is_present:1;
				unsigned char data3_is_present:1;
				unsigned char data4_is_present:1;
				unsigned char data5_is_present:1;
				unsigned char data6_is_present:1;
				unsigned char data7_is_present:1;
			} __packed;
			struct {
				unsigned char data8_is_present:1;
				unsigned char data9_is_present:1;
				unsigned char data10_is_present:1;
				unsigned char data11_is_present:1;
				unsigned char data12_is_present:1;
				unsigned char data13_is_present:1;
				unsigned char data14_is_present:1;
				unsigned char data15_is_present:1;
			} __packed;
		};
		unsigned char data[3];
	};
};

struct synaptics_rmi4_f12_ctrl_8 {
	union {
		struct {
			unsigned char max_x_coord_lsb;
			unsigned char max_x_coord_msb;
			unsigned char max_y_coord_lsb;
			unsigned char max_y_coord_msb;
			unsigned char rx_pitch_lsb;
			unsigned char rx_pitch_msb;
			unsigned char tx_pitch_lsb;
			unsigned char tx_pitch_msb;
			unsigned char low_rx_clip;
			unsigned char high_rx_clip;
			unsigned char low_tx_clip;
			unsigned char high_tx_clip;
			unsigned char num_of_rx;
			unsigned char num_of_tx;
		};
		unsigned char data[14];
	};
};
/*lenovo-sw xuwen1 add for slide gesture begin*/
struct synaptics_rmi4_f12_ctrl_20 {
	union {
		struct {
			unsigned char x_motion_sup;
			unsigned char y_motion_sup;
			unsigned char report_flags;
			unsigned char hover_report;
		};
		unsigned char data[4];
	};
	unsigned short addr;
	unsigned short offset;
};
struct synaptics_rmi4_f12_data_4 {
	union {
		struct {
			unsigned char gesture_type;
			unsigned char gesture_prop0;
			unsigned char gesture_prop1;
			unsigned char gesture_prop2;
			unsigned char gesture_prop3;
		};
		unsigned char data[5];
	};
	unsigned short addr;
};
/*lenovo-sw xuwen1 add 20140915 for FW enter doze from area begin*/
struct synaptics_rmi4_f12_ctrl_27 {
	union {
		struct {
			unsigned char wakeup_gesture_enable;
			unsigned char report_rate;
			unsigned char false_activation;
			unsigned char max_active_duration;
                        unsigned char timer_1;
                        unsigned char max_active_duration_timeout;
                        unsigned char allowed_swipes;
                        unsigned char swipe_tolerance;
                        unsigned char swipe_lift_control;
		};
		unsigned char data[9];
	};
	unsigned short addr;
	unsigned short offset;
};
/*lenovo-sw xuwen1 add 20140915 for FW enter doze from area end*/
struct low_power_wakeup_gesture {
	bool supported;
	bool lpwg_mode;
	bool gesture;
	struct synaptics_rmi4_f12_ctrl_20 control_20;
	struct synaptics_rmi4_f12_data_4 data_04;
//lenovo-sw xuwen1 add 201409215 for lwpw mode
        struct synaptics_rmi4_f12_ctrl_27 control_27;
};

struct low_power_wakeup_gesture lpwg_handler;
/*lenovo-sw xuwen1 add for slide gesture end*/
struct synaptics_rmi4_f12_ctrl_23 {
	union {
		struct {
			unsigned char obj_type_enable;
			unsigned char max_reported_objects;
		};
		unsigned char data[2];
	};
	//lenovo-sw xuwen1 add 20140625 for glove mode
	unsigned short addr; 
};

/*lenovo-sw xuwen1 add 20140625 for glove mode begin*/

struct glove_mode_ctrl {
          bool glove_mode;
          struct synaptics_rmi4_f12_ctrl_23 control_23;
	  
};

struct glove_mode_ctrl glove_handler;
/*lenovo-sw xuwen1 add 20140625 for glove mode end*/

struct synaptics_rmi4_f12_finger_data {
	unsigned char object_type_and_status;
	unsigned char x_lsb;
	unsigned char x_msb;
	unsigned char y_lsb;
	unsigned char y_msb;
#ifdef REPORT_2D_Z
	unsigned char z;
#endif
#ifdef REPORT_2D_W
	unsigned char wx;
	unsigned char wy;
#endif
};

struct synaptics_rmi4_f1a_query {
	union {
		struct {
			unsigned char max_button_count:3;
			unsigned char reserved:5;
			unsigned char has_general_control:1;
			unsigned char has_interrupt_enable:1;
			unsigned char has_multibutton_select:1;
			unsigned char has_tx_rx_map:1;
			unsigned char has_perbutton_threshold:1;
			unsigned char has_release_threshold:1;
			unsigned char has_strongestbtn_hysteresis:1;
			unsigned char has_filter_strength:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f1a_control_0 {
	union {
		struct {
			unsigned char multibutton_report:2;
			unsigned char filter_mode:2;
			unsigned char reserved:4;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f1a_control {
	struct synaptics_rmi4_f1a_control_0 general_control;
	unsigned char button_int_enable;
	unsigned char multi_button;
	unsigned char *txrx_map;
	unsigned char *button_threshold;
	unsigned char button_release_threshold;
	unsigned char strongest_button_hysteresis;
	unsigned char filter_strength;
};

struct synaptics_rmi4_f1a_handle {
	int button_bitmask_size;
	unsigned char max_count;
	unsigned char valid_button_count;
	unsigned char *button_data_buffer;
	unsigned short *button_map;
	struct synaptics_rmi4_f1a_query button_query;
	struct synaptics_rmi4_f1a_control button_control;
};

struct synaptics_rmi4_exp_fhandler {
	struct synaptics_rmi4_exp_fn *exp_fn;
	bool insert;
	bool remove;
	struct list_head link;
};

struct synaptics_rmi4_exp_fn_data {
	bool initialized;
	bool queue_work;
	struct mutex mutex;
	struct list_head list;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct synaptics_rmi4_data *rmi4_data;
};

static struct synaptics_rmi4_exp_fn_data exp_data;

static struct device_attribute attrs[] = {
#ifdef CONFIG_HAS_EARLYSUSPEND
	__ATTR(full_pm_cycle, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP)/*(S_IRUGO | S_IWUGO)*/, /*CTS test, no other permission, mediatek modify*/
			synaptics_rmi4_full_pm_cycle_show,
			synaptics_rmi4_full_pm_cycle_store),
#endif
	__ATTR(reset, (S_IWUSR | S_IWGRP)/*S_IWUGO*/,
			synaptics_rmi4_show_error,
			synaptics_rmi4_f01_reset_store),
	__ATTR(productinfo, (S_IRUSR | S_IRGRP)/*S_IRUGO*/,
			synaptics_rmi4_f01_productinfo_show,
			synaptics_rmi4_store_error),
	__ATTR(buildid, (S_IRUSR | S_IRGRP)/*S_IRUGO*/,
			synaptics_rmi4_f01_buildid_show,
			synaptics_rmi4_store_error),
	__ATTR(flashprog, (S_IRUSR | S_IRGRP)/*S_IRUGO*/,
			synaptics_rmi4_f01_flashprog_show,
			synaptics_rmi4_store_error),
	__ATTR(0dbutton, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP)/*(S_IRUGO | S_IWUGO)*/,
			synaptics_rmi4_0dbutton_show,
			synaptics_rmi4_0dbutton_store),
	__ATTR(suspend, (S_IWUSR | S_IWGRP)/*S_IWUGO*/,
			synaptics_rmi4_show_error,
			synaptics_rmi4_suspend_store),
#ifdef LENOVO_CTP_GLOVE_CONTROL
	__ATTR(glove_debounce, (S_IRUSR | S_IRGRP )/*| S_IWUSR | S_IWGRP | S_IRUGO | S_IWUGO)*/,
			synaptics_rmi4_show_glove_debounce,
			synaptics_rmi4_glove_debounce_store),
#endif
};

#ifdef LENOVO_CTP_GLOVE_CONTROL
static ssize_t synaptics_rmi4_show_glove_debounce(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			glove_debounce_cnt);
}

static ssize_t synaptics_rmi4_glove_debounce_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int input;

	sscanf(buf, "%d", &input);

	glove_debounce_cnt = input;

	return count;
}
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->full_pm_cycle);
}

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->full_pm_cycle = input > 0 ? 1 : 0;

	return count;
}
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int reset;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reset_device(rmi4_data);
	if (retval < 0) {
		dev_err(dev,
				"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
			(rmi4_data->rmi4_mod_info.product_info[0]),
			(rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->firmware_id);
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct synaptics_rmi4_f01_device_status device_status;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			device_status.data,
			sizeof(device_status.data));
	if (retval < 0) {
		dev_err(dev,
				"%s: Failed to read device status, error = %d\n",
				__func__, retval);
		return retval;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
			device_status.flash_prog);
}

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	unsigned char ii;
	unsigned char intr_enable;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->button_0d_enabled == input)
		return count;

	if (list_empty(&rmi->support_fn_list))
		return -ENODEV;

	list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
		if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
			ii = fhandler->intr_reg_num;

			retval = synaptics_rmi4_i2c_read(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;

			if (input == 1)
				intr_enable |= fhandler->intr_mask;
			else
				intr_enable &= ~fhandler->intr_mask;

			retval = synaptics_rmi4_i2c_write(rmi4_data,
					rmi4_data->f01_ctrl_base_addr + 1 + ii,
					&intr_enable,
					sizeof(intr_enable));
			if (retval < 0)
				return retval;
		}
	}

	rmi4_data->button_0d_enabled = input;

	return count;
}

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input == 1)
		synaptics_rmi4_suspend(dev);
	else if (input == 0)
		synaptics_rmi4_resume(dev);
	else
		return -EINVAL;

	return count;
}

 /**
 * synaptics_rmi4_set_page()
 *
 * Called by synaptics_rmi4_i2c_read() and synaptics_rmi4_i2c_write().
 *
 * This function writes to the page select register to switch to the
 * assigned page.
 */
static int synaptics_rmi4_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned int address)
{
	int retval = 0;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = rmi4_data->i2c_client;

	page = ((address >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				dev_err(&i2c->dev,
						"%s: I2C retry %d\n",
						__func__, retry + 1);
				msleep(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return (retval == PAGE_SELECT_LEN) ? retval : -EIO;
}

 /**
 * synaptics_rmi4_i2c_read()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function reads data of an arbitrary length from the sensor,
 * starting from an assigned register address of the sensor, via I2C
 * with a retry mechanism.
 */
static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;

	int full = length / I2C_DMA_LIMIT;
	int partial = length % I2C_DMA_LIMIT;
	int total;
	int last;
	int ii;
	static int msg_length;

	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

	if ((full + 2) > msg_length) {
		kfree(read_msg);
		msg_length = full + 2;
		read_msg = kcalloc(msg_length, sizeof(struct i2c_msg), GFP_KERNEL);
	}

	read_msg[0].addr = rmi4_data->i2c_client->addr;
	read_msg[0].flags = 0;
	read_msg[0].len = 1;
	read_msg[0].buf = &buf;
	read_msg[0].timing = 380;//lenovo liuyw2 scl speed 400->380 for tLow;SCL

	if (partial) {
		total = full + 1;
		last = partial;
	} else {
		total = full;
		last = I2C_DMA_LIMIT;
	}

	for (ii = 1; ii <= total; ii++) {
		read_msg[ii].addr = rmi4_data->i2c_client->addr;
		read_msg[ii].flags = I2C_M_RD;
		read_msg[ii].len = (ii == total) ? last : I2C_DMA_LIMIT;
		read_msg[ii].buf = (u8 *)gpDMABuf_pa + I2C_DMA_LIMIT * (ii - 1);
		read_msg[ii].ext_flag = (rmi4_data->i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
		read_msg[ii].timing = 380;//lenovo liuyw2 scl speed 400->380 for tLow;SCL
	}

	buf = addr & MASK_8BIT;

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, read_msg, (total + 1)) == (total + 1)) {

			retval = length;
			break;
		}
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C read over retry limit\n",
				__func__);
		retval = -EIO;
	}

	memcpy(data, gpDMABuf_va, length);

exit:

	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

 /**
 * synaptics_rmi4_i2c_write()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];

	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = (u8 *)gpDMABuf_pa,
			.ext_flag=(rmi4_data->i2c_client->ext_flag|I2C_ENEXT_FLAG|I2C_DMA_FLAG),
			.timing = 380,//lenovo liuyw2 scl speed 400->380 for tLow;SCL
		}
	};

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	gpDMABuf_va[0] = addr & MASK_8BIT;

	memcpy(&gpDMABuf_va[1],&data[0] , length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:

	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

#ifdef	SPECIAL_REPORT_LOG
#define	NS_TO_MS(_p1, _p2)	(((unsigned int)((_p1)-(_p2)))/(1000)/(1000))
#endif

#ifdef LENOVO_GESTURE_WAKEUP

static int synaptics_rmi4_gesture_report(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char found = 0;

	if (lpwg_handler.lpwg_mode) {
		__dbg("enter in gesture mode\n");
		__dbg_core("%s, lpwg_int_flag(%d.)\n", __func__, lpwg_int_flag);

		if (tpd_rmi4_s3203_det) {
			unsigned char dat;
			retval = synaptics_rmi4_i2c_read(rmi4_data,
								s3203_gesture_reg.data_2d,
								&dat,
								sizeof(dat));
			if (retval < 0) {
				dev_err(&rmi4_data->i2c_client->dev,
						"%s: Failed to read data2d gesture registers\n",
						__func__);
				return -1;

			}
			dat &= 0x3;
			if (dat && (lpwg_int_flag == 1)) {
				if (dat & 0x1)
					gesture_func.letter = 0x24;//double click
				else if (dat & 0x2)
					gesture_func.letter = 0x20;//slide
				else
					__dbg_core("gesture: not supported dat (0x%x.)\n", dat);

				found = 1;
			}

		} else {
			retval = synaptics_rmi4_i2c_read(rmi4_data,
					lpwg_handler.data_04.addr,
					lpwg_handler.data_04.data,
					sizeof(lpwg_handler.data_04.data));
			if (retval < 0)
				return;

			__dbg_core(" gesture_type(0x%x 0x%x 0x%x 0x%x 0x%x.)\n",
					lpwg_handler.data_04.gesture_type,
					lpwg_handler.data_04.gesture_prop0,
					lpwg_handler.data_04.gesture_prop1,
					lpwg_handler.data_04.gesture_prop2,
					lpwg_handler.data_04.gesture_prop3);
			if (((lpwg_handler.data_04.gesture_type ==0x03)
				||(lpwg_handler.data_04.gesture_type == 0x07))
				&& (lpwg_int_flag == 1)) {

					__dbg("enter in gesture mode report KEY_POWER after\n");
					if (lpwg_handler.data_04.gesture_type == 0x03)
						gesture_func.letter = 0x24;//double click
					else if(lpwg_handler.data_04.gesture_type == 0x07)
						gesture_func.letter = 0x20;//slide
					else
						__dbg_core("gesture: not supported type(0x%x.)\n",
								lpwg_handler.data_04.gesture_type);

					found = 1;
			}
		}

		if (found) {
			__dbg_core("3203-gesture report letter(0x%x.)\n", gesture_func.letter);
			input_report_key(rmi4_data->input_dev, KEY_SLIDE, 1);
			//input_report_key(rmi4_data->input_dev, KEY_POWER, 1);
			input_sync(rmi4_data->input_dev);
			input_report_key(rmi4_data->input_dev, KEY_SLIDE, 0);
			//input_report_key(rmi4_data->input_dev, KEY_POWER, 0);
			input_sync(rmi4_data->input_dev);

			lpwg_int_flag = 0;
			lpwg_handler.gesture = true;
		} else
			return -1;
	}
	return 0;
}

static int synaptics_rmi4_0d_gesture_report(struct synaptics_rmi4_data *rmi4_data,
				struct synaptics_rmi4_fn *fhandler)

{
	int retval = 0;
	unsigned char found = 0;
	u8 val;

	if (lpwg_handler.lpwg_mode) {
		__dbg_core("enter 3203 0d gesture mode\n");
		__dbg_core("%s, 3203-lpwg_int_flag = %d.\n", __func__,lpwg_int_flag);

		if (tpd_rmi4_s3203_det) {
			unsigned char dat;
			retval = synaptics_rmi4_i2c_read(rmi4_data,
								s3203_gesture_reg.data_0d,
								&dat,
								sizeof(dat));
			if (retval < 0) {
				dev_err(&rmi4_data->i2c_client->dev,
						"%s: Failed to read data2d gesture registers\n",
						__func__);
				return -1;

			}
			__dbg_core("%s, 0d gesture type = 0x%2x.\n",__func__, dat);
			if (lpwg_int_flag == 1) {
				/*1,2,4,8*/
				if (dat == 0x08) {
					if (dat == 0x08)
						gesture_func.letter = 0x34;
					found = 1;
				}
			}
		} else {
			retval = synaptics_rmi4_i2c_read(rmi4_data,
								fhandler->full_addr.data_base + 1,//0x0201
								&val,
								sizeof(val));
			if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to read 0d gesture registers\n",
					__func__);
				return retval;
			}
			__dbg_core("%s, 0d gesture type = 0x%2x.\n",__func__,val);
			if (lpwg_int_flag == 1) {
				/*1,2,4,8*/
				if (val == 0x08) {
					if (val == 0x08)
						gesture_func.letter = 0x34;
					found = 1;
				}
			}
		}
		if (found) {
			__dbg_core("%s, 0d gesture letter = 0x%x.\n",__func__,gesture_func.letter);
			input_report_key(rmi4_data->input_dev, KEY_SLIDE, 1);
			input_sync(rmi4_data->input_dev);
			input_report_key(rmi4_data->input_dev, KEY_SLIDE, 0);
			input_sync(rmi4_data->input_dev);

			lpwg_int_flag = 0;
			lpwg_handler.gesture = true;
		}
	}
	return retval;
}

static void synaptics_rmi4_gesture_suspend(struct synaptics_rmi4_data *rmi4_data)
{
	int retval = 0;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;

	if (gesture_func.wakeup_enable) {
		__dbg_core("%s, enter suspend lpwg with gesture enable. fullctrl %x fulldata %x\n", __func__, s3203_gesture_reg.ctrl, s3203_gesture_reg.data_2d);
		if (tpd_rmi4_s3203_det) {
			unsigned char dat;
			retval = synaptics_rmi4_i2c_read(rmi4_data,
								s3203_gesture_reg.ctrl,
								&dat,
								sizeof(dat));
			if (retval < 0) {
				dev_err(&rmi4_data->i2c_client->dev,
						"%s: Failed to read ctrl00 gesture registers\n",
						__func__);
			}
			dat |= (1 << 2);
			__dbg_core("lpwg ctrl00 %x", dat);
			retval = synaptics_rmi4_i2c_write(rmi4_data,
									s3203_gesture_reg.ctrl,
									&dat,
									sizeof(dat));
			if (retval < 0)
				dev_err(&rmi4_data->i2c_client->dev,
						"%s: Failed to write ctrl00 gesture registers\n",
						__func__);
		} else {
			lpwg_handler.control_20.report_flags = (lpwg_handler.control_20.report_flags | 0x02);
			retval = synaptics_rmi4_i2c_write(rmi4_data,
				 lpwg_handler.control_20.addr,
				lpwg_handler.control_20.data,
				sizeof(lpwg_handler.control_20.data));
			__dbg(":suspend_1/data is %d \n",lpwg_handler.control_20.report_flags);
			if (retval < 0) {
				dev_err(&(rmi4_data->input_dev->dev),
						"%s: Failed to enter doze mode\n",
						__func__);
				     lpwg_handler.control_20.report_flags = (lpwg_handler.control_20.report_flags)&0xFD;
			}
			/*lenovo-sw xuwen1 add 20140915 for FW enter doze from area begin*/
			lpwg_handler.control_27.max_active_duration = 0x64;
			lpwg_handler.control_27.timer_1 = 0x02;
			lpwg_handler.control_27.max_active_duration_timeout = 0x07;
			retval = synaptics_rmi4_i2c_write(rmi4_data,
				 lpwg_handler.control_27.addr,
				 lpwg_handler.control_27.data,
				sizeof(lpwg_handler.control_27.data));
			if (retval < 0) {
				dev_err(&(rmi4_data->input_dev->dev),
					"%s: Failed to set time control of lwpw  mode\n",
					__func__);
				/*add for back to original status begin*/
				lpwg_handler.control_27.max_active_duration = 0x0c;
				lpwg_handler.control_27.timer_1 = 0x0f;
				lpwg_handler.control_27.max_active_duration_timeout = 0x0a;
				/*add for back to original status begin*/
			}
			/*lenovo-sw xuwen1 add 20140915 for FW enter doze from area end*/
			/*lenovo-sw make sure of lwpw mode*/
			/*	retval = synaptics_rmi4_i2c_read(rmi4_data,
						 lpwg_handler.control_20.addr,
						lpwg_handler.control_20.data,
						sizeof(lpwg_handler.control_20.data));
				__dbg(":suspend_2/data is %d \n",lpwg_handler.control_20.report_flags);*/
			/*lenovo-sw make sure of lwpw mode*/
		}

		lpwg_handler.supported = true;
		lpwg_handler.lpwg_mode = true;
		lpwg_flag = 1;
		lpwg_int_flag = 1;

		synaptics_rmi4_sensor_sleep(rmi4_data);
		synaptics_rmi4_irq_enable(rmi4_data, true);
		rmi4_data->touch_stopped = false;
	} else {
		if (!rmi4_data->sensor_sleep) {

			rmi4_data->touch_stopped = true;
			synaptics_rmi4_irq_enable(rmi4_data, false);
			synaptics_rmi4_sensor_sleep(rmi4_data);
			synaptics_rmi4_free_fingers(rmi4_data);
			__dbg_core("%s, enter suspend lpwg with gesture disable.\n", __func__);
		}

		mutex_lock(&exp_data.mutex);
		if (!list_empty(&exp_data.list)) {
			list_for_each_entry(exp_fhandler, &exp_data.list, link)
				if (exp_fhandler->exp_fn->suspend != NULL)
					exp_fhandler->exp_fn->suspend(rmi4_data);
		}
		mutex_unlock(&exp_data.mutex);

		tpd_halt = 1;

		#ifdef TPD_CLOSE_POWER_IN_SLEEP
			// power down sequence
			tpd_power_ctrl(0);
			mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
			msleep(DELAY_S7300_RESET);
		#endif
	}
}

static void synaptics_rmi4_gesture_resume(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;

	if(gesture_func.wakeup_enable && (lpwg_flag == 1)) {
		__dbg_core("%s, enter resume lpwg.\n", __func__);
		if (tpd_rmi4_s3203_det) {
			unsigned char dat;
			retval = synaptics_rmi4_i2c_read(rmi4_data,
								s3203_gesture_reg.ctrl,
								&dat,
								sizeof(dat));
			if (retval < 0) {
				dev_err(&rmi4_data->i2c_client->dev,
						"%s: Failed to ctrl00 gesture registers\n",
						__func__);
			}
			dat &= ~(1 << 2);
			__dbg_core("lpwg ctrl00 %x", dat);
			retval = synaptics_rmi4_i2c_write(rmi4_data,
									s3203_gesture_reg.ctrl,
									&dat,
									sizeof(dat));
			if (retval < 0)
				dev_err(&rmi4_data->i2c_client->dev,
						"%s: Failed to write ctrl00 gesture registers\n",
						__func__);
		} else {
			lpwg_handler.control_20.report_flags = ( lpwg_handler.control_20.report_flags)&0xFD;
			retval = synaptics_rmi4_i2c_write(rmi4_data,
					 lpwg_handler.control_20.addr,
					lpwg_handler.control_20.data,
					sizeof(lpwg_handler.control_20.data));
			__dbg(" resume_1/lpwg_handler.control_20.report_flags is %d.\n",
					lpwg_handler.control_20.report_flags);
			if (retval < 0) {
				dev_err(&(rmi4_data->input_dev->dev),
						"%s: Failed to enter sleep mode\n",
						__func__);
				lpwg_handler.control_20.report_flags = 0x00;
				return 0;
			}
			__dbg("resume_2/ lpwg_handler.control_20.report_flags is %d.\n", lpwg_handler.control_20.report_flags);
		}
		lpwg_handler.supported = true;
		lpwg_handler.lpwg_mode = false;
		lpwg_flag = 0;
		lpwg_int_flag = 0;
	}
}

#endif

 /**
 * synaptics_rmi4_f11_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $11
 * finger data has been detected.
 *
 * This function reads the Function $11 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char reg_index;
	unsigned char finger;
	unsigned char fingers_supported;
	unsigned char num_of_finger_status_regs;
	unsigned char finger_shift;
	unsigned char finger_status;
	unsigned char data_reg_blk_size;
	unsigned char finger_status_reg[3];
	unsigned char data[F11_STD_DATA_LEN];
	unsigned short data_addr;
	unsigned short data_offset;
	int x;
	int y;
	int wx;
	int wy;
	int temp;
	
#ifdef	SPECIAL_REPORT_LOG
	static unsigned int x_s=0, y_s=0, wx_s=0, wy_s=0;
	unsigned char finger_c;
	unsigned long long ns_tmp;
	static unsigned long long ns_gap = 0;
	unsigned int ms_gap;
#endif
#if defined(LENOVO_AREA_TOUCH)//lenovo xuwen1 modify 20140620 begin area touch
	int xysqr = 0;
#endif
	/*
	 * The number of finger status registers is determined by the
	 * maximum number of fingers supported - 2 bits per finger. So
	 * the number of finger status registers to read is:
	 * register_count = ceil(max_num_of_fingers / 4)
	 */
	fingers_supported = fhandler->num_of_data_points;
	num_of_finger_status_regs = (fingers_supported + 3) / 4;
	data_addr = fhandler->full_addr.data_base;
	data_reg_blk_size = fhandler->size_of_data_register_block;

	__dbg_core("fn11_mask(%d)\n",rmi4_data->fn11_mask);
	#ifdef LENOVO_GESTURE_WAKEUP
		synaptics_rmi4_gesture_report(rmi4_data);
	#endif

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr,
			finger_status_reg,
			num_of_finger_status_regs);
	if (retval < 0)
		return 0;
	mutex_lock(&rmi4_report_mutex);
	__dbg("%s fingers_supported=%d\n", __func__, fingers_supported);

	for (finger = 0; finger < fingers_supported; finger++) {
		reg_index = finger / 4;
		finger_shift = (finger % 4) * 2;
		finger_status = (finger_status_reg[reg_index] >> finger_shift)
				& MASK_2BIT;

	#ifdef LENOVO_CTP_GLOVE_CONTROL
		if (glove_func.status == 1) {
			 __dbg("finger_status is 0x%2x\n",finger_status);
			switch (gf_current) {
				case FS_INIT:
				case FS_GLOVE:
				if (finger_status == 0x2)
					gf_current = FS_GLOVE;
				else if (finger_status == 0x1)
					gf_current = FS_FINGER;
				break;
			case FS_FINGER:
				if (finger_status == 0x2) {
					gf_current = FS_SWITCH_FTOG;
					glove_debounce = glove_debounce_cnt;
					__dbg_core("[f2g-sw] trigger begin: %d\n",glove_debounce);
				}
				break;
			default:
				break;
			}
			if (gf_current == FS_SWITCH_FTOG) {
				if (finger_status == 0x2) {
					if(glove_debounce-- <= 0) {
						__dbg_core(" [f2g-sw] trigger glove successful\n");
						gf_current = FS_GLOVE;
					} else {
						__dbg(" [f2g-sw] debounce left(%d)\n",glove_debounce);
						goto exit_f11_report;
					}
				} else if (finger_status == 0x1) {
					gf_current = FS_FINGER;
					__dbg_core("[[f2g-sw] failed to trigger glove, switch to finger mode\n");
				}
			}

		}
	#endif

		/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status);
#endif
		__dbg(" finger=%d, finger_status=%d\n", finger, finger_status);

		if (finger_status) {
			data_offset = data_addr +
					num_of_finger_status_regs +
					(finger * data_reg_blk_size);
			retval = synaptics_rmi4_i2c_read(rmi4_data,
					data_offset,
					data,
					data_reg_blk_size);
			if (retval < 0) {
				touch_count = 0;
				goto exit;
			}

			x = (data[0] << 4) | (data[2] & MASK_4BIT);
			y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
			wx = (data[3] & MASK_4BIT);
			wy = (data[3] >> 4) & MASK_4BIT;

		#ifdef CONFIG_LENOVO_POWEROFF_CHARGING_UI
			if (((x > LENOVO_CHARGING_DRAW_LEFT) && (x<LENOVO_CHARGING_DRAW_RIGHT))
					&&((y>LENOVO_CHARGING_DRAW_TOP)
					&&(y<LENOVO_CHARGING_DRAW_BOTTOM))
					&&(ipo_flag ==0x1)
					&&(g_tp_poweron !=0x1)) {

				g_tp_poweron = 0x1;
				tp_button_flag = 0x1;
				input_report_key(kpd_input_dev, KEY_HOME, 1);
				input_sync(kpd_input_dev);
				input_report_key(kpd_input_dev, KEY_HOME, 0);
				input_sync(kpd_input_dev);
			}
		#endif

		#if defined(LENOVO_AREA_TOUCH)
			xysqr = wx * wy;
			__dbg_core(" %s XYsqr=%d\n",__func__, xysqr);
		#endif

			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, 1);
#ifndef TYPE_B_PROTOCOL
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TRACKING_ID, finger);
#endif
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
		#if defined(LENOVO_AREA_TOUCH)
			input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_X_W, xysqr);
			input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_Y_W, xysqr);
		#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif

#ifdef TPD_HAVE_BUTTON
			if (NORMAL_BOOT != boot_mode)
			{   
				tpd_button(x, y, 1);
			}	
#endif
			__dbg_core(
					"f11_abs report Finger %d: st(0x%02x), x(%d), y(%d)\n",
					finger,
					finger_status,
					x, y);

#ifdef	SPECIAL_REPORT_LOG
			x_s = x; y_s = y; wx_s = wx; wy_s = wy;
			ns_tmp = sched_clock(); //ns
			ms_gap = NS_TO_MS(ns_tmp, ns_gap); //ms
			ns_gap = ns_tmp; //ns
			finger_c = 'd';
			__dbg("[PF] rx=%d,ry=%d,cx=%d,cy=%d,p=%d,%c(+%d ms)\n",
					x, y, wx, wy, finger, finger_c, ms_gap);
#endif

			touch_count++;
		}
	}

	if (touch_count == 0) {
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);
#ifdef	SPECIAL_REPORT_LOG
		input_report_abs(rmi4_data->input_dev,
				ABS_MT_POSITION_X, x);
		input_report_abs(rmi4_data->input_dev,
				ABS_MT_POSITION_Y, y);
	#if defined(LENOVO_AREA_TOUCH)
		input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_X_W, xysqr);
		input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_Y_W, xysqr);
	#endif
		ns_tmp = sched_clock(); //ns
		ms_gap = NS_TO_MS(ns_tmp, ns_gap); //ms
		ns_gap = ns_tmp; //ns
		finger_c = 'u';
		__dbg("DRV][PF] rx=%d,ry=%d,cx=%d,cy=%d,p=%d,%c(+%d ms)\n",
					x_s, y_s, wx_s, wy_s, finger, finger_c, ms_gap);
#endif
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
#ifdef TPD_HAVE_BUTTON
		if (NORMAL_BOOT != boot_mode)
		{   
			tpd_button(x, y, 0);
		}   
#endif
		__dbg_core("f11_abs up");
	}

	input_sync(rmi4_data->input_dev);
exit:
exit_f11_report:
	mutex_unlock(&rmi4_report_mutex);//mutex_unlock(&(rmi4_data->rmi4_report_mutex));
	return touch_count;
}

 /**
 * synaptics_rmi4_f12_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $12
 * finger data has been detected.
 *
 * This function reads the Function $12 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f12_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char finger;
	unsigned char fingers_to_process;
	unsigned char finger_status;
	unsigned char size_of_2d_data;
	unsigned short data_addr;
	int x;
	int y;
	int wx;
	int wy;
	int temp;
#if defined(LENOVO_AREA_TOUCH)//lenovo xuwen1 modify 20140620 begin area touch
	int Xw = 0,Yw = 0,XYsqr = 0;	
#endif
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_finger_data *data;
	struct synaptics_rmi4_f12_finger_data *finger_data;
	/* lenovo liuyw2 8-13 add. use safe buffer avoid NULL pointer.
	 * origin fhandler buffer could been kfree once reset_device called.
	*/
	struct synaptics_rmi4_f12_extra_data extra_data_safe;
	struct synaptics_rmi4_f12_finger_data data_safe[F12_FINGERS_TO_SUPPORT];
	struct synaptics_rmi4_f12_extra_data *extra_data_unsafe;
	struct synaptics_rmi4_f12_finger_data *data_unsafe;

#ifdef F12_DATA_15_WORKAROUND
	static unsigned char fingers_already_present;
#endif
#ifdef	SPECIAL_REPORT_LOG
	static unsigned int x_s=0, y_s=0, wx_s=0, wy_s=0;
	unsigned char finger_c;
	unsigned long long ns_tmp;
	static unsigned long long ns_gap= 0;
	unsigned int ms_gap;
#endif

	fingers_to_process = fhandler->num_of_data_points;
	data_addr = fhandler->full_addr.data_base;
	extra_data_unsafe = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	data_unsafe = (struct synaptics_rmi4_f12_finger_data *)fhandler->data;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	extra_data = &extra_data_safe;
	data = data_safe;
	/* if origin buffer is safe, copy data to safe buffer, else do nothing*/
	if (extra_data_unsafe)
		memcpy(extra_data, extra_data_unsafe, sizeof(*extra_data));
	else
		goto exit_f12_report;
	if (data_unsafe)
		memcpy(data, data_unsafe, fingers_to_process * size_of_2d_data);
	else
		goto exit_f12_report;
	
	__dbg_core("fn11_mask(%d)\n",rmi4_data->fn11_mask);
	/*lenovo-sw xuwen1 add for gesture begin*/
	#ifdef LENOVO_GESTURE_WAKEUP
	if ( rmi4_data->fn11_mask) {
		synaptics_rmi4_gesture_report(rmi4_data);
	}
	#endif
	/*lenovo-sw xuwen1 add for gesture end*/
	/* Determine the total number of fingers to process */
	if (extra_data->data15_size) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				data_addr + extra_data->data15_offset,
				extra_data->data15_data,
				extra_data->data15_size);
		if (retval < 0)
			return 0;

		/* Start checking from the highest bit */
		temp = extra_data->data15_size - 1; /* Highest byte */
		finger = (fingers_to_process - 1) % 8; /* Highest bit */
		do {
			if (extra_data->data15_data[temp] & (1 << finger))
				break;

			if (finger) {
				finger--;
			} else {
				temp--; /* Move to the next lower byte */
				finger = 7;
			}

			fingers_to_process--;
		} while (fingers_to_process);

		dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Number of fingers to process = %d\n",
			__func__, fingers_to_process);
	}

#ifdef F12_DATA_15_WORKAROUND
	fingers_to_process = max(fingers_to_process, fingers_already_present);
#endif

	if (!fingers_to_process) {
		synaptics_rmi4_free_fingers(rmi4_data);
		return 0;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr + extra_data->data1_offset,
			data,//(unsigned char *)fhandler->data,
			fingers_to_process * size_of_2d_data);
	if (retval < 0)
		return 0;

	//data = (struct synaptics_rmi4_f12_finger_data *)fhandler->data;
	mutex_lock(&rmi4_report_mutex);
	__dbg_core("f2p(%d)\n", fingers_to_process);
	for (finger = 0; finger < fingers_to_process; finger++) {
		finger_data = data + finger;
		/*lenovo-sw xuwen1  changed 20140825 for glove begin*/
	#ifndef LENOVO_CTP_GLOVE_CONTROL
		finger_status = finger_data->object_type_and_status & MASK_1BIT;//lenovo-sw xuwen1 
		__dbg("finger_status is 0x%2x,finger_data->object_type_and_status is 0x%2x\n",finger_status,finger_data->object_type_and_status);
 	#else
		if (glove_func.status == 0)  {
			finger_status = finger_data->object_type_and_status & MASK_1BIT;//lenovo-sw xuwen1 
			__dbg("finger_status is 0x%2x,finger_data->object_type_and_status is 0x%2x\n",finger_status,finger_data->object_type_and_status);
		} else if (glove_func.status == 1) {
			finger_status = finger_data->object_type_and_status & MASK_4BIT;
			 __dbg("finger_status is 0x%2x\n",finger_status);
			switch (gf_current) {
				case FS_INIT:
				case FS_GLOVE:
				if (finger_status == 0x6)
					gf_current = FS_GLOVE;
				else if (finger_status == 0x1)
					gf_current = FS_FINGER;
				break;
			case FS_FINGER:
				if (finger_status == 0x6) {
					gf_current = FS_SWITCH_FTOG;
					glove_debounce = glove_debounce_cnt;
					__dbg("[finger-glove-sw] trigger switch begin: %d\n",glove_debounce);
				}
				break;
			default:
				break;
			}
			if (gf_current == FS_SWITCH_FTOG) {
				if (finger_status == 0x6) {
					if(glove_debounce-- <= 0) {
						__dbg(" [finger-glove-sw] trigger glove successful\n");
						gf_current = FS_GLOVE;
					} else {
						__dbg(" [finger-glove-sw] debounce left(%d)\n",glove_debounce);
						mutex_unlock(&rmi4_report_mutex);
						goto exit_f12_report;
					}
				} else if (finger_status == 0x1) {
					gf_current = FS_FINGER;
					__dbg("[finger-glove-sw] failed to trigger glove, switch to finger mode\n");
				}
			}

		}
	#endif
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, finger_status);
#endif

		if (finger_status) {
#ifdef F12_DATA_15_WORKAROUND
			fingers_already_present = finger + 1;
#endif

			x = (finger_data->x_msb << 8) | (finger_data->x_lsb);
			y = (finger_data->y_msb << 8) | (finger_data->y_lsb);
#ifdef REPORT_2D_W
			wx = finger_data->wx;
			wy = finger_data->wy;
#endif

		/*lenovo xuwen1 modify 20140620 for area touch,begin*/
		#if defined(LENOVO_AREA_TOUCH)
			Xw =finger_data->wx;
			Yw = finger_data->wy;
			XYsqr = Xw*Yw;
			__dbg_core(" %s XYsqr=%d\n",__func__,XYsqr);
		#endif
		/*lenovo xuwen1 modify 20140620 for area touch,end*/
    
		/*lenovo-xw xuwen1 add 20140506 for start work begin*/
		#ifdef CONFIG_LENOVO_POWEROFF_CHARGING_UI	  
		   	if (((x > LENOVO_CHARGING_DRAW_LEFT) && (x<LENOVO_CHARGING_DRAW_RIGHT)) 
					&&((y>LENOVO_CHARGING_DRAW_TOP)
					&&(y<LENOVO_CHARGING_DRAW_BOTTOM))
					&&(ipo_flag ==0x1)
					&&(g_tp_poweron !=0x1)) {

				g_tp_poweron = 0x1;
				tp_button_flag = 0x1;   
				input_report_key(kpd_input_dev, KEY_HOME, 1);
				input_sync(kpd_input_dev);
				input_report_key(kpd_input_dev, KEY_HOME, 0);
				input_sync(kpd_input_dev);	                         
			}
		#endif
		/*lenovo-xw xuwen1 add 20140506 for start work begin*/

			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, 1);
		#ifndef TYPE_B_PROTOCOL
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TRACKING_ID, finger);
		#endif
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
		#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
		#endif
		/*lenovo xuwen1 modify 20140620 for area touch,begin*/
		#if defined(LENOVO_AREA_TOUCH)
			input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_X_W, XYsqr);
			input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_Y_W, XYsqr);
		#endif
		/*lenovo xuwen1 modify 20140620 for area touch,end*/
		#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
		#endif

			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: Finger %d:\n"
					"status = 0x%02x\n"
					"x = %d\n"
					"y = %d\n"
					"wx = %d\n"
					"wy = %d\n",
					__func__, finger,
					finger_status,
					x, y, wx, wy);
		#ifdef	SPECIAL_REPORT_LOG
			x_s = x; y_s = y; wx_s = wx; wy_s = wy;
			ns_tmp = sched_clock(); //ns
			ms_gap = NS_TO_MS(ns_tmp, ns_gap); //ms
			ns_gap = ns_tmp; //ns
			finger_c = 'd';
			__dbg_core("[PF] rx=%d,ry=%d,cx=%d,cy=%d,p=%d,%c(+%d ms)\n",
					x, y, wx, wy, finger, finger_c, ms_gap);
		#endif
			__dbg_core(" f12_abs down(%d,%d,%d,%d)\n",x, y, finger, finger_status);
			TPD_EM_PRINT(wx, wy, x, y, finger, 1);	//for mediatek engineer mode tpd log
			touch_count++;
		}
	}

	if (touch_count == 0) {
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);
	#ifdef	SPECIAL_REPORT_LOG
		input_report_abs(rmi4_data->input_dev,
				ABS_MT_POSITION_X, x);
		input_report_abs(rmi4_data->input_dev,
				ABS_MT_POSITION_Y, y);
		/*lenovo xuwen1 modify 20140620 for area touch,begin*/
		#if defined(LENOVO_AREA_TOUCH)
		input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_X_W, XYsqr);
		input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_Y_W, XYsqr);
		#endif
		/*lenovo xuwen1 modify 20140620 for area touch,end*/
		ns_tmp = sched_clock(); //ns
		ms_gap = NS_TO_MS(ns_tmp, ns_gap); //ms
		ns_gap = ns_tmp; //ns
		finger_c = 'u';
		__dbg("[PF] rx=%d,ry=%d,cx=%d,cy=%d,p=%d,%c(+%d ms)\n",
					x_s, y_s, wx_s, wy_s, finger, finger_c, ms_gap);
	#endif
	#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
	#endif
		__dbg_core(" f12_abs up\n");
		TPD_EM_PRINT(0, 0, 0, 0, 0, 0);	//for mediatek engineer mode tpd log
	}

	input_sync(rmi4_data->input_dev);
	mutex_unlock(&rmi4_report_mutex);
	/* lenovo liuyw2 8-13 add. use safe buffer avoid NULL pointer.
	 * origin fhandler buffer could been kfree once reset_device called.
	*/
	/*copy back to origin buffer, keep it max compatible with origin codes idea */
	if (extra_data_unsafe)
		memcpy(extra_data_unsafe, extra_data, sizeof(*extra_data));
	else
		goto exit_f12_report;/* if extra_data_unsafe is null , do nothing*/

	if (data_unsafe)
		memcpy(data_unsafe, data, fingers_to_process * size_of_2d_data);


exit_f12_report:
	return touch_count;
}
/*lenovo-sw xuwen1 add for double click home gesture begin*/
int synaptics_rmi4_f51_report(struct synaptics_rmi4_data *rmi4_data)

{

	int retval = 0;
	u8 val;
#ifdef LENOVO_GESTURE_WAKEUP
	if (lpwg_handler.lpwg_mode) {
		__dbg("enter in double gesture mode\n");
		retval = synaptics_rmi4_i2c_read(rmi4_data,0x0400,&val,sizeof(val));
		if (retval < 0)
			return;
		__dbg("%s, lpwg_int_flag_in_double = %d.\n", __func__,lpwg_int_flag);
		__dbg("%s, double_gesture_type = 0x%2x.\n",__func__,val);
		if ((val == 0x02) && (lpwg_int_flag == 1)) {
			__dbg("enter in double gesture mode \n");
			gesture_func.letter = 0x50;
			input_report_key(rmi4_data->input_dev, KEY_SLIDE, 1);
			input_sync(rmi4_data->input_dev);
			input_report_key(rmi4_data->input_dev, KEY_SLIDE, 0);
			input_sync(rmi4_data->input_dev);
			
			lpwg_int_flag = 0;
			lpwg_handler.gesture = true;
		}
	}
#endif
	return retval;
}
/*lenovo-sw xuwen1 add for double click home gesture end*/

//lenovo_sw liuyw2 3/20/15 add 0d gesture


static void synaptics_rmi4_f1a_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0;
	unsigned char button;
	unsigned char index;
	unsigned char shift;
	unsigned char status;
	unsigned char *data;
	unsigned short data_addr = fhandler->full_addr.data_base;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	static unsigned char do_once = 1;
	static bool current_status[MAX_NUMBER_OF_BUTTONS];
#ifdef NO_0D_WHILE_2D
	static bool before_2d_status[MAX_NUMBER_OF_BUTTONS];
	static bool while_2d_status[MAX_NUMBER_OF_BUTTONS];
#endif
#ifdef	TPD_BUTTON_REPORT_XY
	unsigned int touch_count_real = 0;
#endif

	if (do_once) {
		memset(current_status, 0, sizeof(current_status));
#ifdef NO_0D_WHILE_2D
		memset(before_2d_status, 0, sizeof(before_2d_status));
		memset(while_2d_status, 0, sizeof(while_2d_status));
#endif
		do_once = 0;
	}
	//lenovo_sw liuyw2 3/20/15 add 0d gesture
	#ifdef LENOVO_GESTURE_WAKEUP
	#ifdef LENOVO_CTP_GLOVE_CONTROL_EXTBTN
	synaptics_rmi4_0d_gesture_report(rmi4_data, fhandler);
	#endif
	#endif

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr,
			f1a->button_data_buffer,
			f1a->button_bitmask_size);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read button data registers\n",
				__func__);
		return;
	}

	data = f1a->button_data_buffer;
	mutex_lock(&rmi4_report_mutex);
	for (button = 0; button < f1a->valid_button_count; button++) {
		index = button / 8;
		shift = button % 8;
		status = ((data[index] >> shift) & MASK_1BIT);

#ifdef	TPD_BUTTON_REPORT_XY
		touch_count_real += status;	//record the total read touch finger
		__dbg("tpd button, button=%d, status=%d, oldstatus=%d, touch_count_real=%d\n", button, status, current_status[button], touch_count_real);
#endif

#ifndef	TPD_BUTTON_REPORT_XY
		if (current_status[button] == status)
			continue;
		else
			current_status[button] = status;
#else
		//report the xy every time and every finger
		if (current_status[button] == status) {
			touch_count = touch_count;
		}
		else {
			current_status[button] = status;
			touch_count++;	//touch_count is the number of touch status changed finger
		}
#endif

		dev_dbg(&rmi4_data->i2c_client->dev,
				"%s: Button %d (code %d) ->%d\n",
				__func__, button,
				f1a->button_map[button],
				status);

#ifdef	TPD_BUTTON_REPORT_XY
		if(1 == status) {	//only if button down, report the x and y
			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, status);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, status);
#ifndef TYPE_B_PROTOCOL
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TRACKING_ID, button);
#endif
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, tpd_keys_dim_local[button][0]);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, tpd_keys_dim_local[button][1]);
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif
			__dbg("tpd button down, button=%d, status=%d, x=%d, y=%d\n", button, status, tpd_keys_dim_local[button][0], tpd_keys_dim_local[button][1]);
		}
#else	/* TPD_BUTTON_REPORT_XY */
#ifdef NO_0D_WHILE_2D
		if (rmi4_data->fingers_on_2d == false) {
			if (status == 1) {
				before_2d_status[button] = 1;
			} else {
				if (while_2d_status[button] == 1) {
					while_2d_status[button] = 0;
					continue;
				} else {
					before_2d_status[button] = 0;
				}
			}
			touch_count++;
			input_report_key(rmi4_data->input_dev,
					f1a->button_map[button],
					status);
			__dbg_core(" %s key(%d) st(%d)\n", __func__, f1a->button_map[button], status);
		} else {
			if (before_2d_status[button] == 1) {
				before_2d_status[button] = 0;
				touch_count++;
				input_report_key(rmi4_data->input_dev,
						f1a->button_map[button],
						status);
				__dbg_core(" %s key(%d) st(%d)\n", __func__, f1a->button_map[button], status);
			} else {
				if (status == 1)
					while_2d_status[button] = 1;
				else
					while_2d_status[button] = 0;
			}
		}
#else
		touch_count++;
		input_report_key(rmi4_data->input_dev,
				f1a->button_map[button],
				status);
		__dbg_core(" %s key(%d) st(%d)\n", __func__, f1a->button_map[button], status);
#endif
#endif	/* TPD_BUTTON_REPORT_XY */
	}

#ifdef	TPD_BUTTON_REPORT_XY
	//touch_count is not zero, it means there have fingers down or up
	//touch_count_real is zero, it means all the finger up, report the up event
	if(touch_count && (0 == touch_count_real)) {
		__dbg("tpd button up, touch_count_real=%d\n", touch_count_real);
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
	}
#endif
	
	if (touch_count) {
		__dbg("tpd button input_sync, touch_count=%d\n", touch_count);
		input_sync(rmi4_data->input_dev);
	}
	mutex_unlock(&rmi4_report_mutex);
	return;
}

 /**
 * synaptics_rmi4_report_touch()
 *
 * Called by synaptics_rmi4_sensor_report().
 *
 * This function calls the appropriate finger data reporting function
 * based on the function handler it receives and returns the number of
 * fingers detected.
 */
static void synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	unsigned char touch_count_2d;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Function %02x reporting\n",
			__func__, fhandler->fn_number);

	switch (fhandler->fn_number) {
	case SYNAPTICS_RMI4_F11:
		__dbg("probe in F11\n");
		touch_count_2d = synaptics_rmi4_f11_abs_report(rmi4_data,
				fhandler);

		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F12:
		__dbg("probe in F12\n");
		touch_count_2d = synaptics_rmi4_f12_abs_report(rmi4_data,
				fhandler);
		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;
	case SYNAPTICS_RMI4_F1A:
		__dbg("probe in F1A\n");
		synaptics_rmi4_f1a_report(rmi4_data, fhandler);
		break;
	default:
		break;
	}

	return;
}

 /**
 * synaptics_rmi4_sensor_report()
 *
 * Called by synaptics_rmi4_irq().
 *
 * This function determines the interrupt source(s) from the sensor
 * and calls synaptics_rmi4_report_touch() with the appropriate
 * function handler for each function with valid data inputs.
 */
static void synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char data[MAX_INTR_REGISTERS + 1];
	unsigned char *intr = &data[1];
	struct synaptics_rmi4_f01_device_status status;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	/*
	 * Get interrupt status information from F01 Data1 register to
	 * determine the source(s) that are flagging the interrupt.
	 */
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			data,
			rmi4_data->num_of_intr_regs + 1);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read interrupt status\n",
				__func__);
		return;
	}
#if defined(LENOVO_CTP_ESD_CHECK)
	if (tpd_esd_check_status(data[0])) {
		__dbg_core("rmi4_esd [IRQ] status: 0x%x\n",data[0]);
		__dbg_core("rmi4_esd [IRQ] esd check trigger\n");
		tpd_esd_do_reset(rmi4_data);
	}
#endif
	status.data[0] = data[0];
	if (status.unconfigured && !status.flash_prog) {
		pr_notice("%s: spontaneous reset detected\n", __func__);
		retval = synaptics_rmi4_reinit_device(rmi4_data);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to reinit device\n",
					__func__);
		}
		return;
	}

	/*
	 * Traverse the function handler list and service the source(s)
	 * of the interrupt accordingly.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				if (fhandler->intr_mask &
						intr[fhandler->intr_reg_num]) {
					synaptics_rmi4_report_touch(rmi4_data,
							fhandler);
				}
			}
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (!exp_fhandler->insert &&
					!exp_fhandler->remove &&
					(exp_fhandler->exp_fn->attn != NULL))
				exp_fhandler->exp_fn->attn(rmi4_data, intr[0]);
		}
	}
	mutex_unlock(&exp_data.mutex);

	return;
}

 /**
 * synaptics_rmi4_irq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */

static void tpd_eint_handler(void)
{      
        __dbg("enter in tpd_eint_handler\n");
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}

static int touch_event_handler(void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };

	sched_setscheduler(current, SCHED_RR, &param);
	do{
		set_current_state(TASK_INTERRUPTIBLE);

		while (tpd_halt) {
			tpd_flag = 0;
			msleep(20);
		}

		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);

		if (!rmi4_data->touch_stopped)
			synaptics_rmi4_sensor_report(rmi4_data);
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	}while(1);

	return 0;
}

 /**
 * synaptics_rmi4_irq_enable()
 *
 * Called by synaptics_rmi4_probe() and the power management functions
 * in this driver and also exported to other expansion Function modules
 * such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int retval = 0;
	unsigned char intr_status[MAX_INTR_REGISTERS];

	if (enable) {

		/* Clear interrupts first */
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr + 1,
				intr_status,
				rmi4_data->num_of_intr_regs);
		if (retval < 0)
			return retval;

		// set up irq
		if (!rmi4_data->irq_enabled) {
			mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, MT_EDGE_SENSITIVE);
			mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING);
			mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING,tpd_eint_handler, 1); //0);
			}
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		rmi4_data->irq_enabled = true;
	} else {
		if (rmi4_data->irq_enabled) {
			mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
			rmi4_data->irq_enabled = false;
		}
	}

	return retval;
}

static void synaptics_rmi4_set_intr_mask(struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	unsigned char ii;
	unsigned char intr_offset;

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	return;
}

static int synaptics_rmi4_f01_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	rmi4_data->f01_query_base_addr = fd->query_base_addr;
	rmi4_data->f01_ctrl_base_addr = fd->ctrl_base_addr;
	rmi4_data->f01_data_base_addr = fd->data_base_addr;
	rmi4_data->f01_cmd_base_addr = fd->cmd_base_addr;

	return 0;
}

 /**
 * synaptics_rmi4_f11_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 11 registers
 * and determines the number of fingers supported, x and y data ranges,
 * offset to the associated interrupt status register, interrupt bit
 * mask, and gathers finger data acquisition capabilities from the query
 * registers.
 */
static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char abs_data_size;
	unsigned char abs_data_blk_size;
	unsigned char query[F11_STD_QUERY_LEN];
	unsigned char control[F11_STD_CTRL_LEN];

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	s3203_gesture_reg.ctrl = fhandler->full_addr.ctrl_base;
	s3203_gesture_reg.data_2d = fhandler->full_addr.data_base + 0x36;
	s3203_gesture_reg.data_0d = 0x419;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base,
			query,
			sizeof(query));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	if ((query[1] & MASK_3BIT) <= 4)
		fhandler->num_of_data_points = (query[1] & MASK_3BIT) + 1;
	else if ((query[1] & MASK_3BIT) == 5)
		fhandler->num_of_data_points = 10;

	rmi4_data->num_of_fingers = fhandler->num_of_data_points;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base,
			control,
			sizeof(control));
	if (retval < 0)
		return retval;

	/* Maximum x and y */
	rmi4_data->sensor_max_x = ((control[6] & MASK_8BIT) << 0) |
			((control[7] & MASK_4BIT) << 8);
	rmi4_data->sensor_max_y = ((control[8] & MASK_8BIT) << 0) |
			((control[9] & MASK_4BIT) << 8);
/*#ifdef TPD_HAVE_BUTTON
	rmi4_data->sensor_max_y = rmi4_data->sensor_max_y * TPD_DISPLAY_HEIGH_RATIO / TPD_TOUCH_HEIGH_RATIO;
#endif*/
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);
        __dbg("[DRV1] max x = %d max y = %d\n ",rmi4_data->sensor_max_x,rmi4_data->sensor_max_y);

	rmi4_data->max_touch_width = MAX_F11_TOUCH_WIDTH;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	abs_data_size = query[5] & MASK_2BIT;
	abs_data_blk_size = 3 + (2 * (abs_data_size == 0 ? 1 : 0));
	fhandler->size_of_data_register_block = abs_data_blk_size;
	fhandler->data = NULL;
	fhandler->extra = NULL;

	return retval;
}

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28)
{
	int retval;
	static unsigned short ctrl_28_address;

	if (ctrl28)
		ctrl_28_address = ctrl28;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			ctrl_28_address,
			&rmi4_data->report_enable,
			sizeof(rmi4_data->report_enable));
	if (retval < 0)
		return retval;

	return retval;
}

 /**
 * synaptics_rmi4_f12_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 12 registers and
 * determines the number of fingers supported, offset to the data1
 * register, x and y data ranges, offset to the associated interrupt
 * status register, interrupt bit mask, and allocates memory resources
 * for finger data acquisition.
 */
static int synaptics_rmi4_f12_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;
	unsigned char size_of_2d_data;
	unsigned char size_of_query8;
	unsigned char ctrl_8_offset;
	unsigned char ctrl_23_offset;
	unsigned char ctrl_28_offset;
	unsigned char num_of_fingers;
	unsigned char ctrl_0d_00;//add by xuwen1 for glove
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_query_5 query_5;
	struct synaptics_rmi4_f12_query_8 query_8;
	struct synaptics_rmi4_f12_ctrl_8 ctrl_8;
	struct synaptics_rmi4_f12_ctrl_23 ctrl_23;


	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 5,
			query_5.data,
			sizeof(query_5.data));
	if (retval < 0)
		return retval;

	ctrl_8_offset = query_5.ctrl0_is_present +
			query_5.ctrl1_is_present +
			query_5.ctrl2_is_present +
			query_5.ctrl3_is_present +
			query_5.ctrl4_is_present +
			query_5.ctrl5_is_present +
			query_5.ctrl6_is_present +
			query_5.ctrl7_is_present;
     /*lenovo-sw xuwen1 add for slide gesture begin*/
      lpwg_handler.control_20.offset = ctrl_8_offset+
	   	      query_5.ctrl8_is_present +
			query_5.ctrl9_is_present +
			query_5.ctrl10_is_present +
			query_5.ctrl11_is_present +
			query_5.ctrl12_is_present +
			query_5.ctrl13_is_present +
			query_5.ctrl14_is_present +
			query_5.ctrl15_is_present +
			query_5.ctrl16_is_present +
			query_5.ctrl17_is_present +
			query_5.ctrl18_is_present +
			query_5.ctrl19_is_present;
	 lpwg_handler.control_20.addr= fhandler->full_addr.ctrl_base + lpwg_handler.control_20.offset;
	 lpwg_handler.data_04.addr = fhandler->full_addr.data_base+0x02;
	 __dbg("lpwg_handler.control_20.addr:0x%04x, lpwg_handler.data_04.addr:0x%04x, fhandler.ctrl_base:0x%04x,fhandler.data_base:0x%04x\n",lpwg_handler.control_20.addr,lpwg_handler.data_04.addr,fhandler->full_addr.ctrl_base,fhandler->full_addr.data_base);
         retval = synaptics_rmi4_i2c_read(rmi4_data,
			 lpwg_handler.control_20.addr,
			lpwg_handler.control_20.data,
			sizeof(lpwg_handler.control_20.data));
	if (retval < 0)
		{
		__dbg(" read lpwg_handler failed\n");
		return retval;
		}
	__dbg(" lpwg_handler.control_20.report_flags is %d.\n", lpwg_handler.control_20.report_flags);
       if(lpwg_handler.control_20.report_flags){
	   	lpwg_handler.supported = false;
		lpwg_handler.lpwg_mode = false;
		
        }
	   
	 /*lenovo-sw xuwen1 add for slide gesture end*/   
	ctrl_23_offset = ctrl_8_offset +
			query_5.ctrl8_is_present +
			query_5.ctrl9_is_present +
			query_5.ctrl10_is_present +
			query_5.ctrl11_is_present +
			query_5.ctrl12_is_present +
			query_5.ctrl13_is_present +
			query_5.ctrl14_is_present +
			query_5.ctrl15_is_present +
			query_5.ctrl16_is_present +
			query_5.ctrl17_is_present +
			query_5.ctrl18_is_present +
			query_5.ctrl19_is_present +
			query_5.ctrl20_is_present +
			query_5.ctrl21_is_present +
			query_5.ctrl22_is_present;
/*lenovo-sw xuwen1 add 20140625 for glove mode begin*/
        glove_handler.control_23.addr= fhandler->full_addr.ctrl_base + ctrl_23_offset;
       __dbg("glove_handler.control_23.addr:0x%04x,fhandler.ctrl_base:0x%04x,ctrl_23_offset:0x%04x\n",glove_handler.control_23.addr,fhandler->full_addr.ctrl_base,ctrl_23_offset);

	 retval = synaptics_rmi4_i2c_read(rmi4_data,
			glove_handler.control_23.addr,
			glove_handler.control_23.data,
			sizeof(glove_handler.control_23.data));
	 if (retval < 0)
	 	{
	 	__dbg(" read glove_handler ctrl_23 failed\n");
		return retval;
	 	}
	 __dbg(" glove_handler.control_23.obj_type_enable is %d.\n", glove_handler.control_23.obj_type_enable);

     retval = synaptics_rmi4_i2c_read(rmi4_data,0x0201,&ctrl_0d_00,sizeof(ctrl_0d_00));
	 if (retval < 0)
	 	{
	 	__dbg(" read glove_handler 0D_ctrl_0 failed\n");
		return retval;
	 	}
	 __dbg(" glove_handler.virtual_control_0.glove_0D_value is %d.\n", ctrl_0d_00);
/*lenovo-sw xuwen1 add 20140625 for glove mode end*/
/*lenovo-sw xuwen1 add 20140915 for FW enter doze from area begin*/	   
lpwg_handler.control_27.offset = ctrl_23_offset +
			query_5.ctrl23_is_present +
			query_5.ctrl24_is_present +
			query_5.ctrl25_is_present +
			query_5.ctrl26_is_present;
lpwg_handler.control_27.addr= fhandler->full_addr.ctrl_base + lpwg_handler.control_27.offset;
__dbg("lpwg_handler.control_27.addr:0x%04x\n",lpwg_handler.control_27.addr);
         retval = synaptics_rmi4_i2c_read(rmi4_data,
			 lpwg_handler.control_27.addr,
			lpwg_handler.control_27.data,
			sizeof(lpwg_handler.control_27.data));
	if (retval < 0)
		{
		__dbg(" read timer_control_lpwg_handler failed\n");
		return retval;
		}
	// __dbg(" max_active_duration d%,timer_1 %d,max_active_duration_timeout %d.\n", lpwg_handler.control_27.max_active_duration,
  //lpwg_handler.control_27.timer_1,lpwg_handler.control_27.max_active_duration_timeout);
/*lenovo-sw xuwen1 add 20140915 for FW enter doze from area end*/	 	
	ctrl_28_offset = ctrl_23_offset +
			query_5.ctrl23_is_present +
			query_5.ctrl24_is_present +
			query_5.ctrl25_is_present +
			query_5.ctrl26_is_present +
			query_5.ctrl27_is_present;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_23_offset,
			ctrl_23.data,
			sizeof(ctrl_23.data));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	fhandler->num_of_data_points = min(ctrl_23.max_reported_objects,
			(unsigned char)F12_FINGERS_TO_SUPPORT);

	num_of_fingers = fhandler->num_of_data_points;
	rmi4_data->num_of_fingers = num_of_fingers;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 7,
			&size_of_query8,
			sizeof(size_of_query8));
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 8,
			query_8.data,
			size_of_query8);
	if (retval < 0)
		return retval;

	/* Determine the presence of the Data0 register */
	extra_data->data1_offset = query_8.data0_is_present;

	if ((size_of_query8 >= 3) && (query_8.data15_is_present)) {
		extra_data->data15_offset = query_8.data0_is_present +
				query_8.data1_is_present +
				query_8.data2_is_present +
				query_8.data3_is_present +
				query_8.data4_is_present +
				query_8.data5_is_present +
				query_8.data6_is_present +
				query_8.data7_is_present +
				query_8.data8_is_present +
				query_8.data9_is_present +
				query_8.data10_is_present +
				query_8.data11_is_present +
				query_8.data12_is_present +
				query_8.data13_is_present +
				query_8.data14_is_present;
		extra_data->data15_size = (num_of_fingers + 7) / 8;
	} else {
		extra_data->data15_size = 0;
	}

	rmi4_data->report_enable = RPT_DEFAULT;
#ifdef REPORT_2D_Z
	rmi4_data->report_enable |= RPT_Z;
#endif
#ifdef REPORT_2D_W
	rmi4_data->report_enable |= (RPT_WX | RPT_WY);
#endif

	retval = synaptics_rmi4_f12_set_enables(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_28_offset);
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_8_offset,
			ctrl_8.data,
			sizeof(ctrl_8.data));
	if (retval < 0)
		return retval;

	/* Maximum x and y */
	rmi4_data->sensor_max_x =
			((unsigned short)ctrl_8.max_x_coord_lsb << 0) |
			((unsigned short)ctrl_8.max_x_coord_msb << 8);
	rmi4_data->sensor_max_y =
			((unsigned short)ctrl_8.max_y_coord_lsb << 0) |
			((unsigned short)ctrl_8.max_y_coord_msb << 8);
/*#ifdef TPD_HAVE_BUTTON
	rmi4_data->sensor_max_y = rmi4_data->sensor_max_y * TPD_DISPLAY_HEIGH_RATIO / TPD_TOUCH_HEIGH_RATIO;
#endif*/
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);
      __dbg("[DRV2] max x = %d max y = %d\n ",rmi4_data->sensor_max_x,rmi4_data->sensor_max_y);
	rmi4_data->num_of_rx = ctrl_8.num_of_rx;
	rmi4_data->num_of_tx = ctrl_8.num_of_tx;
	rmi4_data->max_touch_width = max(rmi4_data->num_of_rx,
			rmi4_data->num_of_tx);

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	/* Allocate memory for finger data storage space */
	fhandler->data_size = num_of_fingers * size_of_2d_data;
	fhandler->data = kmalloc(fhandler->data_size, GFP_KERNEL);

	return retval;
}

static int synaptics_rmi4_f1a_alloc_mem(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	struct synaptics_rmi4_f1a_handle *f1a;

	f1a = kzalloc(sizeof(*f1a), GFP_KERNEL);
	if (!f1a) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for function handle\n",
				__func__);
		return -ENOMEM;
	}

	fhandler->data = (void *)f1a;
	fhandler->extra = NULL;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base,
			f1a->button_query.data,
			sizeof(f1a->button_query.data));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read query registers\n",
				__func__);
		return retval;
	}

	f1a->max_count = f1a->button_query.max_button_count + 1;

	f1a->button_control.txrx_map = kzalloc(f1a->max_count * 2, GFP_KERNEL);
	if (!f1a->button_control.txrx_map) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for tx rx mapping\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_bitmask_size = (f1a->max_count + 7) / 8;

	f1a->button_data_buffer = kcalloc(f1a->button_bitmask_size,
			sizeof(*(f1a->button_data_buffer)), GFP_KERNEL);
	if (!f1a->button_data_buffer) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for data buffer\n",
				__func__);
		return -ENOMEM;
	}

	f1a->button_map = kcalloc(f1a->max_count,
			sizeof(*(f1a->button_map)), GFP_KERNEL);
	if (!f1a->button_map) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for button map\n",
				__func__);
		return -ENOMEM;
	}

	return 0;
}

static int synaptics_rmi4_f1a_button_map(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char ii;
	unsigned char mapping_offset = 0;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	mapping_offset = f1a->button_query.has_general_control +
			f1a->button_query.has_interrupt_enable +
			f1a->button_query.has_multibutton_select;

	if (f1a->button_query.has_tx_rx_map) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				fhandler->full_addr.ctrl_base + mapping_offset,
				f1a->button_control.txrx_map,
				sizeof(f1a->button_control.txrx_map));
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to read tx rx mapping\n",
					__func__);
			return retval;
		}

		rmi4_data->button_txrx_mapping = f1a->button_control.txrx_map;
	}

	if (cap_button_map.map) {
		if (cap_button_map.nbuttons != f1a->max_count) {
			f1a->valid_button_count = min(f1a->max_count,
					cap_button_map.nbuttons);
		} else {
			f1a->valid_button_count = f1a->max_count;
		}

		for (ii = 0; ii < f1a->valid_button_count; ii++)
			f1a->button_map[ii] = cap_button_map.map[ii];
	}
	return 0;
}

static void synaptics_rmi4_f1a_kfree(struct synaptics_rmi4_fn *fhandler)
{
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	if (f1a) {
		kfree(f1a->button_control.txrx_map);
		kfree(f1a->button_data_buffer);
		kfree(f1a->button_map);
		kfree(f1a);
		fhandler->data = NULL;
	}

	return;
}

static int synaptics_rmi4_f1a_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int retval;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

	retval = synaptics_rmi4_f1a_alloc_mem(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	retval = synaptics_rmi4_f1a_button_map(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	rmi4_data->button_0d_enabled = 1;

	return 0;

error_exit:
	synaptics_rmi4_f1a_kfree(fhandler);

	return retval;
}
static void synaptics_rmi4_empty_fn_list(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_fn *fhandler_temp;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry_safe(fhandler,
				fhandler_temp,
				&rmi->support_fn_list,
				link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
				synaptics_rmi4_f1a_kfree(fhandler);
			} else {
				kfree(fhandler->extra);
				kfree(fhandler->data);
			}
			list_del(&fhandler->link);
			kfree(fhandler);
		}
	}
	INIT_LIST_HEAD(&rmi->support_fn_list);

	return;
}

static int synaptics_rmi4_check_status(struct synaptics_rmi4_data *rmi4_data,
		bool *was_in_bl_mode)
{
	int retval;
	int timeout = CHECK_STATUS_TIMEOUT_MS;
	unsigned char command = 0x01;
	unsigned char intr_status;
	struct synaptics_rmi4_f01_device_status status;

	/* Do a device reset first */
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0)
		return retval;

	msleep(DELAY_S7300_RESET_READY);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			status.data,
			sizeof(status.data));
	if (retval < 0)
		return retval;

	while (status.status_code == STATUS_CRC_IN_PROGRESS) {
		if (timeout > 0)
			msleep(20);
		else
			return -1;

		retval = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr,
				status.data,
				sizeof(status.data));
		if (retval < 0)
			return retval;

		timeout -= 20;
	}

	if (timeout != CHECK_STATUS_TIMEOUT_MS)
		*was_in_bl_mode = true;

	if (status.flash_prog == 1) {
		rmi4_data->flash_prog_mode = true;
		pr_notice("%s: In flash prog mode, status = 0x%02x\n",
				__func__,
				status.status_code);
	} else {
		rmi4_data->flash_prog_mode = false;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr + 1,
			&intr_status,
			sizeof(intr_status));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read interrupt status\n",
				__func__);
		return retval;
	}

	return 0;
}

static void synaptics_rmi4_set_configured(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to set configured\n",
				__func__);
		return;
	}

	rmi4_data->no_sleep_setting = device_ctrl & NO_SLEEP_ON;
	device_ctrl |= CONFIGURED;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to set configured\n",
				__func__);
	}

	return;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
		struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
	*fhandler = kmalloc(sizeof(**fhandler), GFP_KERNEL);
	if (!(*fhandler))
		return -ENOMEM;

	(*fhandler)->full_addr.data_base =
			(rmi_fd->data_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.ctrl_base =
			(rmi_fd->ctrl_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.cmd_base =
			(rmi_fd->cmd_base_addr |
			(page_number << 8));
	(*fhandler)->full_addr.query_base =
			(rmi_fd->query_base_addr |
			(page_number << 8));

	return 0;
}

 /**
 * synaptics_rmi4_query_device()
 *
 * Called by synaptics_rmi4_probe().
 *
 * This funtion scans the page description table, records the offsets
 * to the register types of Function $01, sets up the function handlers
 * for Function $11 and Function $12, determines the number of interrupt
 * sources from the sensor, adds valid Functions with data inputs to the
 * Function linked list, parses information from the query registers of
 * Function $01, and enables the interrupt sources from the valid Functions
 * with data inputs.
 */
static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned char offset=0;//lenovo-sw xuwen1
	unsigned char page_number;
	unsigned char intr_count;
	unsigned char f01_query[F01_STD_QUERY_LEN];
	unsigned short pdt_entry_addr;
	unsigned short intr_addr;
	bool was_in_bl_mode;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
	rmi = &(rmi4_data->rmi4_mod_info);
rescan_pdt:
	was_in_bl_mode = false;
	intr_count = 0;
	INIT_LIST_HEAD(&rmi->support_fn_list);

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);

			retval = synaptics_rmi4_i2c_read(rmi4_data,
					pdt_entry_addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			fhandler = NULL;

			if (rmi_fd.fn_number == 0) {
				dev_dbg(&rmi4_data->i2c_client->dev,
						"%s: Reached end of PDT\n",
						__func__);
				break;
			}

			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: F%02x found (page %d)\n",
					__func__, rmi_fd.fn_number,
					page_number);

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f01_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;

				retval = synaptics_rmi4_check_status(rmi4_data,
						&was_in_bl_mode);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"%s: Failed to check status\n",
							__func__);
					return retval;
				}

				if (was_in_bl_mode) {
					kfree(fhandler);
					fhandler = NULL;
					goto rescan_pdt;
				}

				if (rmi4_data->flash_prog_mode)
					goto flash_prog_mode;

				break;
			case SYNAPTICS_RMI4_F11:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f11_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;
			case SYNAPTICS_RMI4_F12:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f12_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				/*lenovo-sw xuwen1 add for gesture begin*/
				rmi4_data->fn11_mask = 0;
                          offset = intr_count%8;
                       for(ii=offset;ii<(rmi_fd.intr_src_count+offset);ii++)
                               rmi4_data->fn11_mask |= 1 <<ii;
				/*lenovo-sw xuwen1 add for gesture end*/
				break;
			case SYNAPTICS_RMI4_F1A:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
						&rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
							"%s: Failed to alloc for F%d\n",
							__func__,
							rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f1a_init(rmi4_data,
						fhandler, &rmi_fd, intr_count);
				if (retval < 0) {
#ifdef IGNORE_FN_INIT_FAILURE
					kfree(fhandler);
					fhandler = NULL;
#else
					return retval;
#endif
				}
				break;
				}
			/* Accumulate the interrupt count */
			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

			if (fhandler && rmi_fd.intr_src_count) {
				list_add_tail(&fhandler->link,
						&rmi->support_fn_list);
			}
		}
	}

flash_prog_mode:
	rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Number of interrupt registers = %d\n",
			__func__, rmi4_data->num_of_intr_regs);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr,
			f01_query,
			sizeof(f01_query));
	if (retval < 0)
		return retval;

	/* RMI Version 4.0 currently supported */
	rmi->version_major = 4;
	rmi->version_minor = 0;

	rmi->manufacturer_id = f01_query[0];
	rmi->product_props = f01_query[1];
	rmi->product_info[0] = f01_query[2] & MASK_7BIT;
	rmi->product_info[1] = f01_query[3] & MASK_7BIT;
	rmi->date_code[0] = f01_query[4] & MASK_5BIT;
	rmi->date_code[1] = f01_query[5] & MASK_4BIT;
	rmi->date_code[2] = f01_query[6] & MASK_5BIT;
	rmi->tester_id = ((f01_query[7] & MASK_7BIT) << 8) |
			(f01_query[8] & MASK_7BIT);
	rmi->serial_number = ((f01_query[9] & MASK_7BIT) << 8) |
			(f01_query[10] & MASK_7BIT);
	memcpy(rmi->product_id_string, &f01_query[11], 10);

	if (rmi->manufacturer_id != 1) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Non-Synaptics device found, manufacturer ID = %d\n",
				__func__, rmi->manufacturer_id);
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET,
			rmi->build_id,
			sizeof(rmi->build_id));
	if (retval < 0)
		return retval;

	rmi4_data->firmware_id = (unsigned int)rmi->build_id[0] +
			(unsigned int)rmi->build_id[1] * 0x100 +
			(unsigned int)rmi->build_id[2] * 0x10000;

	memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

	/*
	 * Map out the interrupt bit masks for the interrupt sources
	 * from the registered function handlers.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				rmi4_data->intr_mask[fhandler->intr_reg_num] |=
						fhandler->intr_mask;
			}
		}
	}

	/* Enable the interrupt sources */
	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (rmi4_data->intr_mask[ii] != 0x00) {
			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: Interrupt enable mask %d = 0x%02x\n",
					__func__, ii, rmi4_data->intr_mask[ii]);
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			retval = synaptics_rmi4_i2c_write(rmi4_data,
					intr_addr,
					&(rmi4_data->intr_mask[ii]),
					sizeof(rmi4_data->intr_mask[ii]));
			if (retval < 0)
				return retval;
		}
	}

	synaptics_rmi4_set_configured(rmi4_data);

	return 0;
}

static void synaptics_rmi4_set_params(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;
	struct synaptics_rmi4_f1a_handle *f1a;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, 0,
			rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, 0,
			rmi4_data->sensor_max_y, 0, 0);
#ifdef REPORT_2D_W
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			rmi4_data->max_touch_width, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MINOR, 0,
			rmi4_data->max_touch_width, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers,0);
#else
	input_mt_init_slots(rmi4_data->input_dev,
			rmi4_data->num_of_fingers);
#endif
#endif

#ifndef TYPE_B_PROTOCOL
/*
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TRACKING_ID, 0,
			rmi4_data->num_of_fingers, 0, 0);
*/
#endif

	f1a = NULL;
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				f1a = fhandler->data;
		}
	}

	if (f1a) {
		for (ii = 0; ii < f1a->valid_button_count; ii++) {
			set_bit(f1a->button_map[ii],
					rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
					EV_KEY, f1a->button_map[ii]);
		}
	}

	return;
}

static int synaptics_rmi4_set_input_dev(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int temp;
       int i;
	rmi4_data->input_dev = input_allocate_device();
	if (rmi4_data->input_dev == NULL) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to allocate input device\n",
				__func__);
		retval = -ENOMEM;
		goto err_input_device;
	}
	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to query device\n",
				__func__);
		goto err_query_device;
	}
	rmi4_data->input_dev->name = DRIVER_NAME;
	rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
	rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
	rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
	rmi4_data->input_dev->id.bustype = BUS_I2C;
	rmi4_data->input_dev->dev.parent = &rmi4_data->i2c_client->dev;
	input_set_drvdata(rmi4_data->input_dev, rmi4_data);

	set_bit(EV_SYN, rmi4_data->input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_dev->evbit);
	set_bit(BTN_TOUCH, rmi4_data->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, rmi4_data->input_dev->keybit);
	
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

#ifndef TYPE_B_PROTOCOL
	set_bit(ABS_MT_TRACKING_ID, rmi4_data->input_dev->absbit);
#endif
	set_bit(ABS_MT_TOUCH_MAJOR, rmi4_data->input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MINOR, rmi4_data->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, rmi4_data->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, rmi4_data->input_dev->absbit);
		
	synaptics_rmi4_set_params(rmi4_data);

	retval = input_register_device(rmi4_data->input_dev);
	if (retval) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to register input device\n",
				__func__);
		goto err_register_input;
	}

	return 0;

err_register_input:
err_query_device:
	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_free_device(rmi4_data->input_dev);

err_input_device:
	return retval;
}

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;
mutex_lock(&rmi4_report_mutex);
#ifdef TYPE_B_PROTOCOL
	for (ii = 0; ii < rmi4_data->num_of_fingers; ii++) {
		input_mt_slot(rmi4_data->input_dev, ii);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(rmi4_data->input_dev,
			BTN_TOUCH, 0);
	input_report_key(rmi4_data->input_dev,
			BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(rmi4_data->input_dev);
#endif
	input_sync(rmi4_data->input_dev);
mutex_unlock(&rmi4_report_mutex);
	rmi4_data->fingers_on_2d = false;

	return 0;
}

static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned short intr_addr;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;

	if (syna_fwu_upgrade_progress == FWU_PROGRESS_IN_PROGRESS)
		return;

	rmi = &(rmi4_data->rmi4_mod_info);

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	synaptics_rmi4_free_fingers(rmi4_data);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F12) {
				synaptics_rmi4_f12_set_enables(rmi4_data, 0);
				break;
			}
		}
	}

	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (rmi4_data->intr_mask[ii] != 0x00) {
			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: Interrupt enable mask %d = 0x%02x\n",
					__func__, ii, rmi4_data->intr_mask[ii]);
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			retval = synaptics_rmi4_i2c_write(rmi4_data,
					intr_addr,
					&(rmi4_data->intr_mask[ii]),
					sizeof(rmi4_data->intr_mask[ii]));
			if (retval < 0)
				goto exit;
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->reinit != NULL)
				exp_fhandler->exp_fn->reinit(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	synaptics_rmi4_set_configured(rmi4_data);

	retval = 0;
	tpd_update_extra_param();

exit:
	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
	return retval;
}

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int temp;
	unsigned char command = 0x01;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;

	if (syna_fwu_upgrade_progress == FWU_PROGRESS_IN_PROGRESS)
		return;

	mutex_lock(&(rmi4_data->rmi4_reset_mutex));

	rmi4_data->touch_stopped = true;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to issue reset command, error = %d\n",
				__func__, retval);
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
		return retval;
	}

	msleep(DELAY_S7300_RESET_READY);

	synaptics_rmi4_free_fingers(rmi4_data);

	synaptics_rmi4_empty_fn_list(rmi4_data);

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to query device\n",
				__func__);
		mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
		return retval;
	}

	synaptics_rmi4_set_params(rmi4_data);

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->reset != NULL)
				exp_fhandler->exp_fn->reset(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	rmi4_data->touch_stopped = false;

	mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
	tpd_update_extra_param();
	return 0;
}

/**
* synaptics_rmi4_exp_fn_work()
*
* Called by the kernel at the scheduled time.
*
* This function is a work thread that checks for the insertion and
* removal of other expansion Function modules such as rmi_dev and calls
* their initialization and removal callback functions accordingly.
*/
static void synaptics_rmi4_exp_fn_work(struct work_struct *work)
{
	int retval;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler_temp;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry_safe(exp_fhandler,
				exp_fhandler_temp,
				&exp_data.list,
				link) {
			if ((exp_fhandler->exp_fn->init != NULL) &&
					exp_fhandler->insert) {
				retval = exp_fhandler->exp_fn->init(rmi4_data);
				if (retval < 0) {
					list_del(&exp_fhandler->link);
					kfree(exp_fhandler);
				} else {
					exp_fhandler->insert = false;
				}
			} else if ((exp_fhandler->exp_fn->remove != NULL) &&
					exp_fhandler->remove) {
				exp_fhandler->exp_fn->remove(rmi4_data);
				list_del(&exp_fhandler->link);
				kfree(exp_fhandler);
			}
		}
	}
	mutex_unlock(&exp_data.mutex);

	return;
}

/**
* synaptics_rmi4_new_function()
*
* Called by other expansion Function modules in their module init and
* module exit functions.
*
* This function is used by other expansion Function modules such as
* rmi_dev to register themselves with the driver by providing their
* initialization and removal callback function pointers so that they
* can be inserted or removed dynamically at module init and exit times,
* respectively.
*/
void synaptics_rmi4_new_function(struct synaptics_rmi4_exp_fn *exp_fn,
		bool insert)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}

	mutex_lock(&exp_data.mutex);
	if (insert) {
		exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
		if (!exp_fhandler) {
			pr_err("%s: Failed to alloc mem for expansion function\n",
					__func__);
			goto exit;
		}
		exp_fhandler->exp_fn = exp_fn;
		exp_fhandler->insert = true;
		exp_fhandler->remove = false;
		list_add_tail(&exp_fhandler->link, &exp_data.list);
	} else if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (exp_fhandler->exp_fn->fn_type == exp_fn->fn_type) {
				exp_fhandler->insert = false;
				exp_fhandler->remove = true;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&exp_data.mutex);

	if (exp_data.queue_work) {
		queue_delayed_work(exp_data.workqueue,
				&exp_data.work,
				msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
	}

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);

static void tpd_power_ctrl(int en)
{
        if (en) {
	#ifdef TPD_PWRBY_PMIC
		hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP");
        	msleep(DELAY_S7300_BOOT_READY);
	#elif defined(TPD_PWRBY_GPIO)
		mt_set_gpio_mode(TPD_POWER_GPIO, GPIO_CTP_EN_PIN_M_GPIO);
		mt_set_gpio_dir(TPD_POWER_GPIO, GPIO_DIR_OUT);
		mt_set_gpio_out(TPD_POWER_GPIO, GPIO_OUT_ONE);
		msleep(DELAY_S7300_BOOT_READY);
	#else
		//no need ctrl
	#endif
	} else {
	#ifdef TPD_PWRBY_PMIC
        	hwPowerDown(TPD_POWER_SOURCE, "TP");
        	msleep(10);
	#elif defined(TPD_PWRBY_GPIO)
		mt_set_gpio_dir(TPD_POWER_GPIO, GPIO_DIR_IN);
		msleep(20);
	#else
		//no need ctrl
	#endif
	}
}

#ifdef LENOVO_CTP_ESD_CHECK
static void tpd_esd_do_reset(struct synaptics_rmi4_data *rmi4_data)
{

	if (syna_fwu_upgrade_progress == FWU_PROGRESS_IN_PROGRESS)
		return;


	tpd_esd_doing_reset = 1;
#if 0
	int retval;

	retval = synaptics_rmi4_reset_device(rmi4_data);
	if (retval < 0) {
		__dbg("tpd_esd_check_reset error\n");
		return;
	}
#endif
	#if 1
	tpd_power_ctrl(0);
	tpd_power_ctrl(1);
	#endif
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(10);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(60);
	tpd_esd_doing_reset = 0;
}

static int tpd_esd_check_status(unsigned char status)
{
	if ((status & 0xF) == 0x3 || (status & 0xF) > 0x7) {
		return 1;
	}

	return 0;
}

static int tpd_esd_check(struct synaptics_rmi4_data *rmi4_data)
{
	int i, retval;
	int do_reset = 0;
	unsigned char status = 0;

	if (tpd_esd_doing_reset)
		return 0;

	for (i = 0; i < 1; i++) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			&status,1);
		if (retval >= 0)
			break;

	}
	__dbg_core("rmi4_esd [TIMER] status: 0x%x\n",status);

	if (retval < 0) {
		__dbg_core("rmi4_esd [TIMER] i2c error trigger\n");
	    do_reset = 1;
	}

	if (tpd_esd_check_status(status)) {
		__dbg_core("rmi4_esd [TIMER] esd check trigger\n");
		do_reset = 1;
	}

	if (do_reset)
		return 1;
	else
		return 0;
}

static void tpd_esd_check_func(struct work_struct *work)
{
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

	if (syna_fwu_upgrade_progress != FWU_PROGRESS_IN_PROGRESS) {
		if (tpd_esd_check(rmi4_data))
		    tpd_esd_do_reset(rmi4_data);
	}
	queue_delayed_work(tpd_esd_check_wq,
			&tpd_esd_check_dwork,
			msecs_to_jiffies(TPD_ESD_CHECK_DELAY));

}
#endif

int rmi4_tpd_get_status(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char status = 0;
	int retval;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
		rmi4_data->f01_data_base_addr,
		&status,1);
	if (retval < 0)
		return -1;

	return status;

}

void rmi4_tpd_hw_reset(void)
{

	tpd_power_ctrl(0);
	tpd_power_ctrl(1);

	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(10);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(60);
}

/*lenovo liuyw2 add , called by fw upgrade, once fwu failed, reprobe some codes*/
void rmi4_tpd_reprobe(struct synaptics_rmi4_data *rmi4_data)
{
	synaptics_rmi4_set_input_dev(rmi4_data);
}

void rmi4_tpd_reinit(struct synaptics_rmi4_data *rmi4_data)
{
	synaptics_rmi4_reinit_device(rmi4_data);
}

 /*
 * synaptics_rmi4_probe()
 *
 * Called by the kernel when an association with an I2C device of the
 * same name is made (after doing i2c_add_driver).
 *
 * This funtion allocates and initializes the resources for the driver
 * as an input driver, turns on the power to the sensor, queries the
 * sensor for its supported Functions and characteristics, registers
 * the driver to the input subsystem, sets up the interrupt, handles
 * the registration of the early_suspend and late_resume functions,
 * and creates a work queue for detection of other expansion Function
 * modules.
 */

 
static int  synaptics_rmi4_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;
	unsigned char attr_count;
	struct synaptics_rmi4_data *rmi4_data;
	int i;

	/*if other tp ic already, then return, dont do reset&pwr on*/ 
	if (tpd_ic_ready_get())
		return 0;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data not supported\n",
				__func__);
		return -EIO;
	}

	// gpio setting
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
   	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
  	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

	// power up sequence
  	tpd_power_ctrl(1);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(DELAY_S7300_RESET);
  	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(DELAY_S7300_RESET_READY);

#if	TOUCH_FILTER
	memcpy(&tpd_filter, &tpd_filter_local, sizeof(struct tpd_filter_t));
#endif

	rmi4_data = kzalloc(sizeof(*rmi4_data), GFP_KERNEL);
	if (!rmi4_data) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for rmi4_data\n",
				__func__);
		return -ENOMEM;
	}

	rmi4_data->i2c_client = client;
	rmi4_data->current_page = MASK_8BIT;
	rmi4_data->touch_stopped = false;
	rmi4_data->sensor_sleep = false;
	rmi4_data->irq_enabled = false;
	rmi4_data->fingers_on_2d = false;

	rmi4_data->i2c_read = synaptics_rmi4_i2c_read;
	rmi4_data->i2c_write = synaptics_rmi4_i2c_write;
	rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
	rmi4_data->reset_device = synaptics_rmi4_reset_device;

	mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));
	mutex_init(&(rmi4_data->rmi4_reset_mutex));
	i2c_set_clientdata(client, rmi4_data);
	retval = synaptics_rmi4_set_input_dev(rmi4_data);
	if (retval < 0) {
		dev_err(&client->dev,
				"%s: Failed to set up input device\n",
				__func__);
		goto err_set_input_dev;
	}

/*lenovo-sw, xuwen1, 20140423, add for light up screen begin */
#ifdef LENOVO_GESTURE_WAKEUP
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_POWER);
	input_set_capability(rmi4_data->input_dev, EV_KEY, KEY_SLIDE);
#endif
/*lenovo-sw, xuwen1, 20140423, add for light up screen end*/

/*lenovo xuwen1 modify 20140620 for area touch,begin*/
 #if defined(LENOVO_AREA_TOUCH)
	set_bit(ABS_MT_POSITION_X_W,rmi4_data->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y_W, rmi4_data->input_dev->absbit);
#endif
/*lenovo xuwen1 modify 20140620 for area touch,end*/

#ifdef CONFIG_HAS_EARLYSUSPEND
	rmi4_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	rmi4_data->early_suspend.suspend = synaptics_rmi4_early_suspend;
	rmi4_data->early_suspend.resume = synaptics_rmi4_late_resume;
	register_early_suspend(&rmi4_data->early_suspend);
#endif

	thread = kthread_run(touch_event_handler, rmi4_data, "synaptics-tpd");
	if ( IS_ERR(thread) ) {
		retval = PTR_ERR(thread);
		pr_err(" %s: failed to create kernel thread: %d\n",__func__, retval);
	}

	retval = synaptics_rmi4_irq_enable(rmi4_data, true);
	if (retval < 0) {
		dev_err(&client->dev,
				"%s: Failed to enable attention interrupt\n",
				__func__);
		goto err_enable_irq;
	}

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}


	exp_data.workqueue = create_singlethread_workqueue("dsx_exp_workqueue");
	INIT_DELAYED_WORK(&exp_data.work, synaptics_rmi4_exp_fn_work);
	exp_data.rmi4_data = rmi4_data;
	exp_data.queue_work = true;
	queue_delayed_work(exp_data.workqueue,
			&exp_data.work,
			msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			goto err_sysfs;
		}
	}
#ifdef LENOVO_CTP_ESD_CHECK
	INIT_DELAYED_WORK(&tpd_esd_check_dwork, tpd_esd_check_func);
	tpd_esd_check_wq = create_workqueue("ctp_esd_check");
	if (!tpd_esd_check_wq) {
		dev_err(&client->dev,
			"%s: Failed to create esd check workqueue\n",
			__func__);
		goto err_esd_wq_creat_failed;
	}
	queue_delayed_work(tpd_esd_check_wq,
			&tpd_esd_check_dwork,
			msecs_to_jiffies(TPD_ESD_CHECK_DELAY));
#endif
	tpd_load_status = 1;
	g_dev = &rmi4_data->input_dev->dev;
#ifdef LENOVO_GESTURE_WAKEUP
	gesture_func.letter = gesture_func.wakeup_enable = 0;
	le_tpd_reg_feature_gesture_func(&gesture_func);
#endif
#ifdef LENOVO_CTP_GLOVE_CONTROL
	glove_func.pre_status = glove_func.status = 0;
	glove_func.usb_st = 0;
	glove_func.set_mode = set_glove_mode_func;
	glove_func.get_mode = get_glove_mode_func;
	le_tpd_reg_feature_glove_func(&glove_func);
#endif
	tpd_ic_ready_set(1);
	return retval;

#ifdef LENOVO_CTP_ESD_CHECK
err_esd_wq_creat_failed:
#endif
err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);

	synaptics_rmi4_irq_enable(rmi4_data, false);

err_enable_irq:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->early_suspend);
#endif

	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;

err_set_input_dev:
	kfree(rmi4_data);

	return retval;
}

/*lenovo-sw xuwen1 add 20140625 for glove mode begin*/
#ifdef LENOVO_CTP_GLOVE_CONTROL
static int set_glove_mode_func(bool en)
{
	u8 reg_val = 0;
	int retval = 0;

	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);

	if (!tpd_load_status) {
	    __dbg("[glove_mode]Do not use correct touchpanel.\n");
	    return -1;
	}

	if (tpd_halt) {
	    __dbg("[glove_mode]touchpanel status is suspend.\n");
	    return -1;
	}

	if (en) {
		if (tpd_rmi4_s3203_det) {
			reg_val = 0x00;
			retval = synaptics_rmi4_i2c_write(rmi4_data, 0x041A,
						&reg_val,sizeof(reg_val));
		} else {
			reg_val = 0x84;
			retval = synaptics_rmi4_i2c_write(rmi4_data, 0x0201,
						&reg_val,sizeof(reg_val));
		}
		glove_func.status = 1; //close button&USB
		__dbg_core("enable glove func  mode.\n");

	} else {
		if (tpd_rmi4_s3203_det) {
			reg_val = 0x02;
			retval = synaptics_rmi4_i2c_write(rmi4_data, 0x041A,
						&reg_val,sizeof(reg_val));
		} else {
			reg_val = 0x04;
			retval = synaptics_rmi4_i2c_write(rmi4_data, 0x0201,
						&reg_val,sizeof(reg_val));
		}
		glove_func.status = 0;
		__dbg_core("disable glove func mode.\n");

	}
	if (retval < 0) {
		__dbg("set  glove func error !\n");
	}
	return retval;
}

static int get_glove_mode_func(void)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
	u8 reg_val = 0;
	int retval = 0;
	u8 tmp;

	if (!tpd_load_status) {
		__dbg("[glove_mode]Do not use correct touchpanel.\n");
		return -1;
	}

	if (tpd_halt) {
		__dbg("[glove_mode]touchpanel status is suspend.\n");
		return -1;
	}

	if (tpd_rmi4_s3203_det) {
		retval = synaptics_rmi4_i2c_read(rmi4_data, 0x041A,
						&reg_val,sizeof(reg_val));
		if (retval< 0) {
			TPD_DMESG("set glove func error !\n");
		}
		/*00:enabled, 02:disabled, so !reg_val*/
		tmp = !reg_val;

	} else {
		retval = synaptics_rmi4_i2c_read(rmi4_data, 0x0201,
						&reg_val,sizeof(reg_val));
		if (retval< 0) {
			TPD_DMESG("set glove func error !\n");
		}
		tmp = (reg_val >> 7) & 0xff;

	}
	
	__dbg("the value of glove status is 0x%8x\n",tmp);
	return tmp;
}

#endif
/*lenovo-sw xuwen1 add 20140625 for glove mode end*/

 /**
 * synaptics_rmi4_remove()
 *
 * Called by the kernel when the association with an I2C device of the
 * same name is broken (when the driver is unloaded).
 *
 * This funtion terminates the work queue, stops sensor data acquisition,
 * frees the interrupt, unregisters the driver from the input subsystem,
 * turns off the power to the sensor, and frees other allocated resources.
 */
static int synaptics_rmi4_remove(struct i2c_client *client)
{
	unsigned char attr_count;
	struct synaptics_rmi4_data *rmi4_data = i2c_get_clientdata(client);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);
#ifdef LENOVO_CTP_ESD_CHECK
	cancel_delayed_work_sync(&tpd_esd_check_dwork);
	flush_workqueue(tpd_esd_check_wq);
	destroy_workqueue(tpd_esd_check_wq);
#endif
	synaptics_rmi4_irq_enable(rmi4_data, false);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->early_suspend);
#endif

	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;

	kfree(rmi4_data);

	return 0;
}

#ifdef CONFIG_PM
 /**
 * synaptics_rmi4_sensor_sleep()
 *
 * Called by synaptics_rmi4_early_suspend() and synaptics_rmi4_suspend().
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;
	unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;
	__dbg("enter in %s\n",__func__);
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return;
	}

#ifdef LENOVO_GESTURE_WAKEUP
	if (gesture_func.wakeup_enable) {
		device_ctrl = (device_ctrl & ~MASK_3BIT);
		device_ctrl = (device_ctrl |  NO_SLEEP_OFF );

		retval = synaptics_rmi4_i2c_write(rmi4_data,
				rmi4_data->f01_ctrl_base_addr,
				&device_ctrl,
				sizeof(device_ctrl));
		if (retval < 0) {
			dev_err(&(rmi4_data->input_dev->dev),
					"%s: Failed to wake from sleep mode\n",
					__func__);
			rmi4_data->sensor_sleep = true;
			return;
		} else {
			rmi4_data->sensor_sleep = false;
		}
		__dbg("1/the status of sensor_sleep is %d\n",rmi4_data->sensor_sleep);
	} else {
		device_ctrl = (device_ctrl & ~MASK_3BIT);
		device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);

		retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
		if (retval < 0) {
			dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return;
		} else {
			rmi4_data->sensor_sleep = true;
		}
     		__dbg("2/the status of sensor_sleep is %d\n",rmi4_data->sensor_sleep);
	}
#else
	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to enter sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = false;
		return;
	} else {
		rmi4_data->sensor_sleep = true;
	}
	__dbg("2/the status of sensor_sleep is %s\n",rmi4_data->sensor_sleep);
#endif
	return;
}

 /**
 * synaptics_rmi4_sensor_wake()
 *
 * Called by synaptics_rmi4_resume() and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */
static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;
	unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to wake from sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = true;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | no_sleep_setting | NORMAL_OPERATION);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
				"%s: Failed to wake from sleep mode\n",
				__func__);
		rmi4_data->sensor_sleep = true;
		return;
	} else {
		rmi4_data->sensor_sleep = false;
	}

	return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
 /**
 * synaptics_rmi4_early_suspend()
 *
 * Called by the kernel during the early suspend phase when the system
 * enters suspend.
 *
 * This function calls synaptics_rmi4_sensor_sleep() to stop finger
 * data acquisition and put the sensor to sleep.
 */
static void synaptics_rmi4_early_suspend(struct early_suspend *h)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data,
			early_suspend);

	if (rmi4_data->stay_awake) {
		rmi4_data->staying_awake = true;
		return;
	} else {
		rmi4_data->staying_awake = false;
	}

	rmi4_data->touch_stopped = true;
	synaptics_rmi4_irq_enable(rmi4_data, false);
	synaptics_rmi4_sensor_sleep(rmi4_data);
	synaptics_rmi4_free_fingers(rmi4_data);

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->early_suspend != NULL)
				exp_fhandler->exp_fn->early_suspend(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	if (rmi4_data->full_pm_cycle)
		synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));

	return;
}

 /**
 * synaptics_rmi4_late_resume()
 *
 * Called by the kernel during the late resume phase when the system
 * wakes up from suspend.
 *
 * This function goes through the sensor wake process if the system wakes
 * up from early suspend (without going into suspend).
 */
static void synaptics_rmi4_late_resume(struct early_suspend *h)
{
	int retval;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data =
			container_of(h, struct synaptics_rmi4_data,
			early_suspend);

	if (rmi4_data->staying_awake)
		return;

	if (rmi4_data->full_pm_cycle)
		synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));

	if (rmi4_data->sensor_sleep == true) {
		synaptics_rmi4_sensor_wake(rmi4_data);
		synaptics_rmi4_irq_enable(rmi4_data, true);
		retval = synaptics_rmi4_reinit_device(rmi4_data);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to reinit device\n",
					__func__);
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->late_resume != NULL)
				exp_fhandler->exp_fn->late_resume(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	rmi4_data->touch_stopped = false;

	return;
}
#endif

 /**
 * synaptics_rmi4_suspend()
 *
 * Called by the kernel during the suspend phase when the system
 * enters suspend.
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
static int synaptics_rmi4_suspend(struct device *dev)
{
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);
	int retval;      
	if (rmi4_data->staying_awake)
		return 0;

	if (syna_fwu_upgrade_progress == FWU_PROGRESS_IN_PROGRESS) {
		bypass_suspend = 1;
		return 0;
	}

#ifdef LENOVO_CTP_ESD_CHECK
	cancel_delayed_work_sync(&tpd_esd_check_dwork);
#endif

#ifdef LENOVO_GESTURE_WAKEUP
	synaptics_rmi4_gesture_suspend(rmi4_data);
#else
	if (!rmi4_data->sensor_sleep) {
			__dbg_core("%s, enter suspend no gesture.\n", __func__);
		rmi4_data->touch_stopped = true;
		synaptics_rmi4_irq_enable(rmi4_data, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
		synaptics_rmi4_free_fingers(rmi4_data);
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->suspend != NULL)
				exp_fhandler->exp_fn->suspend(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	tpd_halt = 1;
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

#endif
	return 0;
}

 /**
 * synaptics_rmi4_resume()
 *
 * Called by the kernel during the resume phase when the system
 * wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
static int synaptics_rmi4_resume(struct device *dev)
{
	int retval;
	struct synaptics_rmi4_exp_fhandler *exp_fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(g_dev);

	if (bypass_suspend) {
		bypass_suspend = 0;
		return 0;
	}

#ifdef TPD_CLOSE_POWER_IN_SLEEP
	// power up sequence
	#ifdef LENOVO_GESTURE_WAKEUP
	if (!gesture_func.wakeup_enable) {
		tpd_power_ctrl(1);
	}
	#endif
#endif
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(DELAY_S7300_RESET);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(DELAY_S7300_RESET_READY);

	if (rmi4_data->staying_awake)
		return 0;
#ifdef LENOVO_GESTURE_WAKEUP
	synaptics_rmi4_gesture_resume(rmi4_data);
#endif

	__dbg_core("%s \n", __func__);
	synaptics_rmi4_sensor_wake(rmi4_data);
	synaptics_rmi4_irq_enable(rmi4_data, true);

	retval = synaptics_rmi4_reinit_device(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to reinit device\n",
				__func__);
		return retval;
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link)
			if (exp_fhandler->exp_fn->resume != NULL)
				exp_fhandler->exp_fn->resume(rmi4_data);
	}
	mutex_unlock(&exp_data.mutex);

	rmi4_data->touch_stopped = false;
	tpd_halt = 0;
#ifdef LENOVO_CTP_GLOVE_CONTROL
	/*Lenovo-sw caoyi1 modify for glove mode  20140709 start*/
	if (glove_func.pre_status && glove_func.usb_st == 0) {
		//__dbg("recall glove function glove_value is %d\n");
		glove_func.set_mode(1);
	}
	/*Lenovo-sw caoyi1 modify for glove mode  20140709 end*/
#endif
#ifdef LENOVO_CTP_ESD_CHECK
	queue_delayed_work(tpd_esd_check_wq,
			&tpd_esd_check_dwork,
			msecs_to_jiffies(TPD_ESD_CHECK_DELAY));
#endif
	return 0;
}

static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
	.suspend = synaptics_rmi4_suspend,
	.resume  = synaptics_rmi4_resume,
};
#endif

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{"synaptics-tpd", 0},
	{},
};
unsigned short force[] = {0,TPD_I2C_ADDR,I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
//static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info);

MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name = "synaptics-tpd",
	},
	.probe = synaptics_rmi4_probe,
	.remove = synaptics_rmi4_remove,
	//.detect = tpd_detect,
	.id_table = synaptics_rmi4_id_table,
	.address_list = (const unsigned short*) forces,
};

static int tpd_local_init(void)
{
	TPD_DMESG("synaptics I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev,
							4096, &gpDMABuf_pa, GFP_KERNEL);
	if (!gpDMABuf_va) {
		__dbg_core("Allocate DMA I2C Buffer failed!");
		return -1;
	}
	memset(gpDMABuf_va, 0, 4096);

	if (i2c_add_driver(&tpd_i2c_driver)!=0) {
		TPD_DMESG("tangjie Error unable to add i2c driver.\n");
		goto exit_local_init;
	}
#ifdef TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif 
	boot_mode = get_boot_mode();
	if (boot_mode == 3) {
		boot_mode = NORMAL_BOOT;
	}
	tpd_type_cap = 1;


	return 0;

exit_local_init:
	if(gpDMABuf_va)
		dma_free_coherent(&tpd->dev->dev,
			4096, gpDMABuf_va, gpDMABuf_pa);
	return -1;
}

static struct tpd_driver_t synaptics_rmi4_driver = {
	.tpd_device_name = "synaptics_tpd",
	.tpd_local_init = tpd_local_init,
	.suspend = synaptics_rmi4_suspend,
	.resume = synaptics_rmi4_resume,
#ifndef TPD_HAVE_BUTTON //lenovo-sw xuwen1 add for read version
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif		
};

static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO("synaptics-tpd", (TPD_I2C_ADDR))};

 /**
 * synaptics_rmi4_init()
 *
 * Called by the kernel during do_initcalls (if built-in)
 * or when the driver is loaded (if a module).
 *
 * This function registers the driver to the I2C subsystem.
 *
 */
static int __init synaptics_rmi4_init(void)
{
	i2c_register_board_info(TPD_I2C_BUS, &i2c_tpd, 1);
	if(tpd_driver_add(&synaptics_rmi4_driver) < 0){
		pr_err("Fail to add tpd driver\n");
		return -1;
	}

	return 0;
}

 /**
 * synaptics_rmi4_exit()
 *
 * Called by the kernel when the driver is unloaded.
 *
 * This funtion unregisters the driver from the I2C subsystem.
 *
 */
static void __exit synaptics_rmi4_exit(void)
{
	 tpd_driver_remove(&synaptics_rmi4_driver); //i2c_del_driver(&synaptics_rmi4_driver);
	return;
}

module_init(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Touch Driver");
MODULE_LICENSE("GPL v2");
