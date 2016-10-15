/*
 * Charging IC driver (rt9536)
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/power_supply.h>		// might need to get fuel gauge info
#include <linux/spinlock.h>
#include <linux/kernel.h>

#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <cust_gpio_boot.h>

#include <linux/xlog.h>
#include <mach/charging.h>
#include <cust_charging.h>
#include "rt9536.h"

static DEFINE_MUTEX(charging_lock);

enum power_supply_type charging_ic_status;

kal_bool chargin_hw_init_done = KAL_FALSE;

/* Fuction Prototype */
static void charging_ic_initialize(void);
static irqreturn_t charging_ic_interrupt_handler(int irq, void *data);

struct timer_list charging_timer;

static DEFINE_SPINLOCK(rt9536_spin);

enum charging_ic_charging_mode{
    CHR_USB_500_MODE,
    CHR_ISET_MODE,
    CHR_USB_100_MODE,
    CHR_PTM_MODE,
    CHR_DENACTIVE_MODE,
    CHR_UNKNOW_MODE,
};

static DECLARE_WAIT_QUEUE_HEAD(charger_set_gpio_ctrl_waiter);

int charging_ic_set_chargingmode( enum charging_ic_charging_mode mode )
{
    int i= 0;
    int pulse_cnt = 0;

	battery_log(BAT_LOG_CRTI, "[charger_rt9536] :: %s\n", __func__);

    switch( mode )
    {
        case CHR_USB_500_MODE:
            pulse_cnt = 0;
            break;
        case CHR_ISET_MODE:
            pulse_cnt = 1;
            break;
        case CHR_USB_100_MODE:
            pulse_cnt = 2;
            break;
        case CHR_PTM_MODE:
            pulse_cnt = 3;
            break;
        case CHR_DENACTIVE_MODE:
            pulse_cnt = 0;
            break;
        case CHR_UNKNOW_MODE:
        default:
            break;
    }

    if( (mode != CHR_DENACTIVE_MODE) && ( mode != CHR_UNKNOW_MODE ) )
    {
        spin_lock(&rt9536_spin);

        mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);

        for( i=0; i<pulse_cnt ; i++)
        {
            udelay(110);
            mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);
            udelay(110);
            mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);
        }

        spin_unlock(&rt9536_spin);

        udelay(1500);

        spin_lock(&rt9536_spin);

        mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);
        udelay(770);  // about 770 us
        mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);

        spin_unlock(&rt9536_spin);
    }
    else
    {
        mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);
    }
    /*
    {
        GPIO_CFG cfg = {.no = CHG_EN_SET_N};
        cfg.mode   = mt_get_gpio_mode(cfg.no);
        cfg.pullsel= mt_get_gpio_pull_select(cfg.no);
        cfg.dout   = mt_get_gpio_out(cfg.no);
        cfg.pullen = mt_get_gpio_pull_enable(cfg.no);
        cfg.dir    = mt_get_gpio_dir(cfg.no);
		battery_log(BAT_LOG_CRTI,  "[charger_rt9536] ::idx = %3d: mode = %d, pullsel = %d, dout = %d, pullen = %d, dir = %d\n",
            cfg.no, cfg.mode, cfg.pullsel, cfg.dout, cfg.pullen, cfg.dir);
    }
    */
}

enum power_supply_type get_charging_ic_status()
{
    return charging_ic_status;
}

// USB500 mode charging
void charging_ic_active_default(void)
{
    u32 wait;

    if(charging_ic_status == POWER_SUPPLY_TYPE_USB)
    {
		battery_log(BAT_LOG_CRTI,  "[charger_rt9536] :: it's already %s mode!!\n", __func__);
        return;
    }

    if(charging_ic_status != POWER_SUPPLY_TYPE_BATTERY)
    {
        charging_ic_deactive();
    }

    mutex_lock(&charging_lock);

    // USB500 mode
#if (1)
    charging_ic_set_chargingmode(CHR_USB_500_MODE);
#else
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);
    udelay(1500);
#endif /* 1 */

    charging_ic_status = POWER_SUPPLY_TYPE_USB;

    mutex_unlock(&charging_lock);

	battery_log(BAT_LOG_CRTI,  "[charger_rt9536] :: %s :\n", __func__);

}
EXPORT_SYMBOL(charging_ic_active_default);
// TA connection, ISET mode
void charging_ic_set_ta_mode(void)
{
    u32 wait;

	battery_log(BAT_LOG_CRTI,  "[charger_rt9536] :: charging_ic_set_ta_mode\n");

    if(charging_ic_status == POWER_SUPPLY_TYPE_MAINS)
    {
		battery_log(BAT_LOG_CRTI,  "[charger_rt9536] :: it's already %s mode!! :\n", __func__);
        return;
    }

    if(charging_ic_status != POWER_SUPPLY_TYPE_BATTERY)
    {
        charging_ic_deactive();
    }

    mutex_lock(&charging_lock);

#if (1)
    charging_ic_set_chargingmode(CHR_ISET_MODE);
#else
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);
    udelay(400);
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);
    udelay(400);  // about 400 us
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);
    udelay(1500);
#endif /* 1 */

    charging_ic_status = POWER_SUPPLY_TYPE_MAINS;

    mutex_unlock(&charging_lock);

	battery_log(BAT_LOG_CRTI,  "[charger_rt9536] :: %s :\n", __func__);
}

void charging_ic_set_usb_mode(void)
{
    charging_ic_active_default();
}

void charging_ic_set_factory_mode(void)
{
    u32 wait;

#if 0
    if(charging_ic_status == POWER_SUPPLY_TYPE_FACTORY)
    {
		battery_log(BAT_LOG_CRTI,  "[charger_rt9536] :: it's already %s mode!! :\n", __func__);
        return;
    }
#endif /* 0 */

	battery_log(BAT_LOG_CRTI,  "[charger_rt9536] :: charging_ic_set_factory_mode\n");

    if(charging_ic_status != POWER_SUPPLY_TYPE_BATTERY)
    {
        charging_ic_deactive();
    }

    mutex_lock(&charging_lock);

#if (1)
    charging_ic_set_chargingmode(CHR_PTM_MODE);
#else
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);
    udelay(400);
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);
    udelay(400);
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);
    udelay(400);
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);
    udelay(400);
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);
    udelay(400);
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);
    udelay(400);
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);
    udelay(1500);
#endif /* 0 */

#if 0
    charging_ic_status = POWER_SUPPLY_TYPE_FACTORY;
#endif /* 0 */

    mutex_unlock(&charging_lock);

	battery_log(BAT_LOG_CRTI,  "[charger_rt9536] :: %s :\n", __func__);
}

void charging_ic_deactive(void)
{

    if(charging_ic_status == POWER_SUPPLY_TYPE_BATTERY)
    {
		battery_log(BAT_LOG_CRTI,  "[charger_rt9536] :: it's already %s mode!! :\n", __func__);
        return;
    }

    mutex_lock(&charging_lock);

#if 1
    charging_ic_set_chargingmode(CHR_DENACTIVE_MODE);
#else
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);

    udelay(2000);
#endif /* 0 */

    charging_ic_status = POWER_SUPPLY_TYPE_BATTERY;

    mutex_unlock(&charging_lock);

	battery_log(BAT_LOG_CRTI,  "[charger_rt9536] :: %s :\n", __func__);
}
EXPORT_SYMBOL(charging_ic_deactive);

void rt9536_charging_enable(unsigned int set_current, unsigned int enable)
{
    if (enable)
    {
        if ( set_current == AC_CHARGER_CURRENT )
            charging_ic_set_ta_mode();
        else if ( set_current == USB_CHARGER_CURRENT )
            charging_ic_set_usb_mode();
        else
            charging_ic_active_default();

		battery_log(BAT_LOG_CRTI, "[charger_rt9536] :: %s, current(%d), enable(%d)\n", __func__, set_current, enable);
    }
    else
    {
        charging_ic_deactive();
		battery_log(BAT_LOG_CRTI, "[charger_rt9536] :: %s, enable(%d)\n", __func__, enable);
    }

}

unsigned char rt9536_check_eoc_status(void)
{
    unsigned char reg_val = 0;
    unsigned char eoc_status = 0;

    // TO DO
    eoc_status = mt_get_gpio_in(CHG_EOC_N);;

    if( eoc_status == 1 )
    {
		battery_log(BAT_LOG_CRTI, "[charger_rt9536] :: (%s) eoc_status(%d)\n", __func__, eoc_status);
        return 1;
    }

	battery_log(BAT_LOG_CRTI, "[charger_rt9536] :: (%s) eoc_status(%d)\n", __func__, eoc_status);
    return 0;
}

static void charging_ic_initialize(void)
{
    charging_ic_status = POWER_SUPPLY_TYPE_BATTERY;
}

static irqreturn_t charging_ic_interrupt_handler(int irq, void *data)
{
    ;
}

static void charging_timer_work(struct work_struct *work)
{
    ;
}

static int charging_ic_probe(struct platform_device *dev)
{
    int ret = 0;

    mt_set_gpio_mode(CHG_EN_SET_N, CHG_EN_MODE);
    mt_set_gpio_dir(CHG_EN_SET_N, CHG_EN_DIR);
    mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);  // charging off ; HIGH
    //mt_set_gpio_out(CHG_EN_SET_N, CHG_EN_DATA_OUT);  // charging off ; HIGH

    mt_set_gpio_mode(CHG_EOC_N, CHG_EOC_MODE);
    mt_set_gpio_dir(CHG_EOC_N, CHG_EOC_DIR);
    mt_set_gpio_pull_enable(CHG_EOC_N, CHG_EOC_PULL_ENABLE);
    mt_set_gpio_pull_select(CHG_EOC_N, CHG_EOC_PULL_SELECT);

	battery_log(BAT_LOG_CRTI, "[charger_rt9536] :: charging IC Initialization is done\n");

    charging_ic_initialize();

    chargin_hw_init_done = KAL_TRUE;

    return 0;
}

static int charging_ic_remove(struct platform_device *dev)
{
    charging_ic_deactive();

    return 0;
}

static int charging_ic_suspend(struct platform_device *dev, pm_message_t state)
{
	battery_log(BAT_LOG_CRTI, "[charger_rt9536] :: charging_ic_suspend\n");
    dev->dev.power.power_state = state;
    return 0;
}

static int charging_ic_resume(struct platform_device *dev)
{
	battery_log(BAT_LOG_CRTI, "[charger_rt9536] :: charging_ic_resume\n");
    dev->dev.power.power_state = PMSG_ON;
    return 0;
}

static struct platform_driver charging_ic_driver = {
    .probe = charging_ic_probe,
    .remove = charging_ic_remove,
//    .suspend = charging_ic_suspend,
//    .resume = charging_ic_resume,
    .driver = {
        .name = "ext_charger",
        .owner = THIS_MODULE,
    },
};

static struct platform_device charger_ic_dev = {
	.name = "ext_charger",
	.id   = -1,
};


static int __init charging_ic_init(void)
{
    int ret=0;

	battery_log(BAT_LOG_CRTI, "[charger_rt9536] Charging IC Driver Init\n");

    ret = platform_device_register(&charger_ic_dev);
    if (ret) {
		battery_log(BAT_LOG_CRTI, "[charger_rt9536] Unable to device register(%d)\n", ret);
        return ret;
    }

    return platform_driver_register(&charging_ic_driver);
}

static void __exit charging_ic_exit(void)
{
	battery_log(BAT_LOG_CRTI, "[charger_rt9536] Charging IC Driver Exit\n");
    platform_driver_unregister(&charging_ic_driver);
}

module_init(charging_ic_init);
module_exit(charging_ic_exit);
