/* 
 * 
 * Author: yucong xiong <yucong.xion@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "cm366xx.h"
#include <linux/sched.h>
#include <alsps.h>
#include <linux/batch.h>

#ifdef CUSTOM_KERNEL_SENSORHUB //need modify PS data length for the custom sensor hub
#include <SCP_sensorHub.h>
#endif
/******************************************************************************
 * configuration
 *******************************************************************************/
/*----------------------------------------------------------------------------*/

#define CM36686_DEV_NAME     "cm36686"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO 	APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  	APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR	APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO 	APS_TAG fmt, ##args)    

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

/******************************************************************************
 * extern functions
 *******************************************************************************/
#ifdef CUST_EINT_ALS_TYPE
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
#else
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
/*----------------------------------------------------------------------------*/
static int cm36686_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int cm36686_i2c_remove(struct i2c_client *client);
static int cm36686_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int cm36686_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int cm36686_i2c_resume(struct i2c_client *client);
static int set_psensor_threshold(struct i2c_client *client);
static int cm36686_read_als(struct i2c_client *client, u16 *data);

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id cm36686_i2c_id[] = {{CM36686_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_cm36686={ I2C_BOARD_INFO(CM36686_DEV_NAME, 0x60)};
/*----------------------------------------------------------------------------*/
struct cm36686_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;

	/*misc*/
	u16 		als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on; 	/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end; 	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t 	trace;


	/*data*/
	u16	als;
	u16	ps;
	u8	_align;
	u16	als_level_num;
	u16 	als_value_num;
	u32	als_level[C_CUST_ALS_LEVEL-1];
	u32	als_value[C_CUST_ALS_LEVEL];
	int	ps_cali;

	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_eng_cali_high;  /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_eng_cali_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;
	atomic_t  als_algo_threadhold;
	atomic_t  als_algo_noise;
	ulong	enable; 		/*enable mask*/
	ulong	pending_intr;	/*pending interrupt*/
	atomic_t	ps_ftm_cali_high;  /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_ftm_cali_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cali_high;  /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cali_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_threshold_high;  /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_threshold_low;  /*the cmd value can't be read, stored in ram*/

	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/

static struct i2c_driver cm36686_i2c_driver = {	
	.probe      = cm36686_i2c_probe,
	.remove     = cm36686_i2c_remove,
	.detect     = cm36686_i2c_detect,
	.suspend    = cm36686_i2c_suspend,
	.resume     = cm36686_i2c_resume,
	.id_table   = cm36686_i2c_id,
	.driver = {
		.name = CM36686_DEV_NAME,
	},
};

/*----------------------------------------------------------------------------*/
struct PS_CALI_DATA_STRUCT
{
	int close;
	int far_away;
	int valid;
};
/*----------------------------------------------------------------------------*/
static struct i2c_client *cm36686_i2c_client = NULL;
static struct cm36686_priv *cm36686_obj = NULL;
static bool is_als_first_read_after_enable;
#define CM36686_REG_WHITE_DATA	0x0A

static int cm36686_local_init(void);
static int cm36686_remove(void);
static int cm36686_init_flag =-1; // 0<==>OK -1 <==> fail
static struct alsps_init_info cm36686_init_info = {
	.name = "cm36686",
	.init = cm36686_local_init,
	.uninit = cm36686_remove,

};
#define ALS_CALI_ALGO 0
#if ALS_CALI_ALGO
const u8 als_change_sensitivity = 20;
u16 als_last_value;
u16 als_thd_high;
u16 als_thd_low;
const u8 als_persistence = 2;
u8 out_of_window_count;
#endif
/*----------------------------------------------------------------------------*/

static DEFINE_MUTEX(cm36686_mutex);


/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS    = 1,
	CMC_BIT_PS	   = 2,
}CMC_BIT;
/*-----------------------------CMC for debugging-------------------------------*/
typedef enum {
	CMC_TRC_ALS_DATA= 0x0001,
	CMC_TRC_PS_DATA = 0x0002,
	CMC_TRC_EINT    = 0x0004,
	CMC_TRC_IOCTL   = 0x0008,
	CMC_TRC_I2C     = 0x0010,
	CMC_TRC_CVT_ALS = 0x0020,
	CMC_TRC_CVT_PS  = 0x0040,
	CMC_TRC_DEBUG   = 0x8000,
} CMC_TRC;
/*-----------------------------------------------------------------------------*/

int CM36686_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	int res = 0;
	mutex_lock(&cm36686_mutex);
	switch(i2c_flag){	
		case I2C_FLAG_WRITE:
			client->addr &=I2C_MASK_FLAG;
			res = i2c_master_send(client, buf, count);
			client->addr &=I2C_MASK_FLAG;
			break;

		case I2C_FLAG_READ:
			client->addr &=I2C_MASK_FLAG;
			client->addr |=I2C_WR_FLAG;
			client->addr |=I2C_RS_FLAG;
			res = i2c_master_send(client, buf, count);
			client->addr &=I2C_MASK_FLAG;
			break;
		default:
			APS_LOG("CM36686_i2c_master_operate i2c_flag command not support!\n");
			break;
	}
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	mutex_unlock(&cm36686_mutex);
	return res;
EXIT_ERR:
	mutex_unlock(&cm36686_mutex);
	APS_ERR("CM36686_i2c_master_operate fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static void cm36686_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "CM36686")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "CM36686")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}

static int cm36686_enable_ps(struct i2c_client *client, int enable)
{
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 databuf[3];
	APS_LOG("cm36686_enable_ps enable:%d\n", enable);
	if(enable == 1)
	{
		databuf[0]= CM36686_REG_PS_CONF3_MS;
		res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_PS_EXIT_ERR;
		}

		databuf[0]= CM36686_REG_PS_CONF1_2;
		res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_PS_EXIT_ERR;
		}
		databuf[2] = 0x03;
		databuf[1] = databuf[0]&0xFE;
		databuf[0]= CM36686_REG_PS_CONF1_2;
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_PS_EXIT_ERR;
		}
		//lenovo-sw molg1 add for auto calibration 20150408		
		msleep(5);//(3);min
		set_psensor_threshold(client);
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		set_bit(CMC_BIT_PS, &obj->enable);
	}
	else
	{
		databuf[0]= CM36686_REG_PS_CONF1_2;
		res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_PS_EXIT_ERR;
		}

		databuf[2] = databuf[1];
		databuf[1] = databuf[0]|0x01;	
		databuf[0]= CM36686_REG_PS_CONF1_2;
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_PS_EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		clear_bit(CMC_BIT_PS, &obj->enable);
	}
	APS_LOG("cm36686_enable_ps finished\n");
	return 0;
ENABLE_PS_EXIT_ERR:
	return res;
}
/********************************************************************/
static int cm36686_enable_als(struct i2c_client *client, int enable)
{
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 databuf[3];
	APS_LOG("cm36686_enable_als enable:%d\n", enable);

	if(enable == 1)
	{
		databuf[2] = 0x00;
		databuf[1] = 0x00;		
		databuf[0] = CM36686_REG_ALS_CONF;
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_ALS_EXIT_ERR;
		}
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		set_bit(CMC_BIT_ALS, &obj->enable);

		is_als_first_read_after_enable = true;
		msleep(80);
		cm36686_read_als(obj->client, &obj->als);
		als_report_interrupt_data(obj->als);
		APS_ERR("cm36686_enable_als first data report value:%d\n", obj->als);
	}
	else
	{
		databuf[2] = 0x00;
		databuf[1] = 0x01;//databuf[0]|0x01;
		databuf[0] = CM36686_REG_ALS_CONF;
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_ALS_EXIT_ERR;
		}
		atomic_set(&obj->als_deb_on, 0);
		clear_bit(CMC_BIT_ALS, &obj->enable);
	}
	return 0;
ENABLE_ALS_EXIT_ERR:
	return res;
}
/********************************************************************/
long cm36686_read_ps(struct i2c_client *client, u16 *data)
{
	long res;
	u8 databuf[2];
	struct cm36686_priv *obj = i2c_get_clientdata(client);

	databuf[0] = CM36686_REG_PS_DATA;
	res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_PS_EXIT_ERR;
	}
	if(atomic_read(&obj->trace) & CMC_TRC_DEBUG){	
		APS_LOG("CM36686_REG_PS_DATA value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
	}
	*data = ((databuf[1]<<8)|databuf[0]);

	return 0;
READ_PS_EXIT_ERR:
	return res;
}
/********************************************************************/
u16 cm36686_convert_als(struct i2c_client *client, u16 als_data, u16 white_data)
{
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	u32 i_cf;
	u32 temp;
	u16 converted_als;

	//APS_LOG("cm36686_convert_als ch0 = %d ch1 = %d\n", als_data, white_data);
	if (als_data <=  atomic_read(&obj->als_algo_threadhold))
	{
		if (white_data > atomic_read(&obj->als_algo_threadhold))
			white_data = atomic_read(&obj->als_algo_threadhold);
		return white_data;
	}
	else if(white_data == 0)
	{
		white_data = 1;
	}

	i_cf = als_data * 10 / white_data;
	if (i_cf > 11)
	{
		i_cf = 11;
	}

	if (i_cf >= 7)
	{
		temp = als_data * ((24038 * i_cf * i_cf / 100) - (45642 * i_cf / 10) + 31186) / 10000;
	}
	else
	{
		temp = als_data * ((3255 * i_cf * i_cf / 100) + (2846 * i_cf / 10) + 7438) / 10000;
	}
	converted_als = (u16)((temp > 65535) ? 65535 : temp);	

	return converted_als;
}
/********************************************************************/
#if ALS_CALI_ALGO
void cm36686_set_als_threshold(u16 als_data)
{
	int temp;

	// calculate als high threshold
	if (als_data == 65535)
	{
		als_thd_high = 65535;
	}
	else
	{
		temp = (als_data) * (100 + als_change_sensitivity) / 100;
		als_thd_high = (u16)((temp > 65534) ? 65534 : temp);
	}

	// calculate als low threshold			
	if (als_data == 0)
	{
		als_thd_low = 0;
	}
	else
	{
		temp = (als_data) * (100 - als_change_sensitivity) / 100;
		als_thd_low = (u16)((temp < 1) ? 1 : temp);
	}
}
/********************************************************************/
#endif
int cm36686_read_als(struct i2c_client *client, u16 *data)
{
	long res;
	u8 databuf[3];
	u16 als_data;
	u16 white_data;
	struct cm36686_priv *obj = i2c_get_clientdata(client);

	databuf[0] = CM36686_REG_ALS_DATA;
	res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_ALS_EXIT_ERR;
	}
	als_data = ((databuf[1]<<8)|databuf[0]);	

	databuf[0] = CM36686_REG_WHITE_DATA;
	res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_ALS_EXIT_ERR;
	}
	white_data = ((databuf[1]<<8)|databuf[0]);	


	*data = cm36686_convert_als(client, als_data, white_data);	
	if(atomic_read(&obj->trace) & CMC_TRC_DEBUG){
		APS_LOG("CM36686_REG_ALS_DATA value: %d\n", *data);
	}
	if (is_als_first_read_after_enable == true)
	{
		databuf[2] = 0x00;
		databuf[1] = 0x81;		
		databuf[0] = CM36686_REG_ALS_CONF;
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto READ_ALS_EXIT_ERR;
		}

		databuf[2] = 0x00;
		databuf[1] = 0x80;		
		databuf[0] = CM36686_REG_ALS_CONF;
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto READ_ALS_EXIT_ERR;
		}
		*data = (*data)*4;	
#if ALS_CALI_ALGO		
		cm36686_set_als_threshold(*data);
		als_last_value = *data;
		out_of_window_count = 0;
#endif		

		is_als_first_read_after_enable = false;
	}
#if ALS_CALI_ALGO
	else
	{
		if ((*data > als_thd_high) || (*data < als_thd_low))
		{
			out_of_window_count++;
			if (out_of_window_count == als_persistence)
			{
				cm36686_set_als_threshold(*data);				
				als_last_value = *data;
				out_of_window_count = 0;
			}
			else
			{
				*data = als_last_value;
			}
		}
		else
		{
			*data = als_last_value;
			out_of_window_count = 0;
		}
	}
#endif	
	return 0;
READ_ALS_EXIT_ERR:
	return res;
}
/********************************************************************/
static int cm36686_get_ps_value(struct cm36686_priv *obj, u8 ps)
{
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	val = 0;

	if(ps > atomic_read(&obj->ps_eng_cali_high))
	{
		val = 0;  /*close*/
	}
	else if(ps < atomic_read(&obj->ps_eng_cali_low))
	{
		val = 1;  /*far away*/
	}

	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}

		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			if(mask)
			{
				APS_DBG("PS:  %05d => %05d [M] \n", ps, val);
			}
			else
			{
				APS_DBG("PS:  %05d => %05d\n", ps, val);
			}
		}
		if(0 == test_bit(CMC_BIT_PS,  &obj->enable))
		{
			//if ps is disable do not report value
			APS_DBG("PS: not enable and do not report this value\n");
			return -1;
		}
		else
		{
			return val;
		}

	}	
	else
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}
/********************************************************************/
static int cm36686_get_als_value(struct cm36686_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}

	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}

		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
#if defined(MTK_AAL_SUPPORT)
		int level_high = obj->hw->als_level[idx];
		int level_low = (idx > 0) ? obj->hw->als_level[idx-1] : 0;
		int level_diff = level_high - level_low;
		int value_high = obj->hw->als_value[idx];
		int value_low = (idx > 0) ? obj->hw->als_value[idx-1] : 0;
		int value_diff = value_high - value_low;
		int value = 0;

		if ((level_low >= level_high) || (value_low >= value_high))
			value = value_low;
		else
			value = (level_diff * value_low + (als - level_low) * value_diff + ((level_diff + 1) >> 1)) / level_diff;

		APS_DBG("ALS: %d [%d, %d] => %d [%d, %d] \n", als, level_low, level_high, value, value_low, value_high);
		return value;
#endif
		if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		}

		return obj->hw->als_value[idx];
	}
	else
	{
		if(atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);	  
		}
		return -1;
	}

}


/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
 *******************************************************************************/
static ssize_t cm36686_show_reg(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct i2c_client *client = cm36686_obj->client;
	u8 databuf[2];
	int i,res;	

	for(i=0; i <= 0x0C; ++i)
	{
		databuf[0] = i;
		res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
		if(res<0)
		{
			APS_ERR("chip id REG 0x%x read error\n", i);
		}
		len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x%x value = 0x%x\n", i, ((databuf[1]<<8)|databuf[0]));
	}
	return len;
}

static ssize_t cm36686_show_ps_enable(struct device_driver *ddri, char *buf)
{
	struct cm36686_priv *obj = cm36686_obj;
	ssize_t len = 0;
	int enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
	APS_FUN();
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", enable_ps);
	return len;

}static ssize_t cm36686_store_ps_enable(struct device_driver *ddri, const char *buf, size_t count)
{
	struct cm36686_priv *obj = cm36686_obj;
	uint16_t enable=0;
	int res;

	sscanf(buf, "%hu", &enable);
	APS_LOG("cm36686_store_ps_enable entry enable:%d !!! \n", enable);
	res=	cm36686_enable_ps(obj->client, enable);
	if(res)
	{
		APS_ERR("als_enable_nodata is failed!!\n");
	}

	APS_LOG("cm36686_store_ps_enable finished !!! \n");
	return count;
}

static ssize_t cm36686_show_als_enable(struct device_driver *ddri, char *buf)
{
	struct cm36686_priv *obj = cm36686_obj;
	ssize_t len = 0;
	bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);
	APS_FUN();
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", enable_als);
	return len;

}

static ssize_t cm36686_store_als_enable(struct device_driver *ddri, const char *buf, size_t count)
{
	struct cm36686_priv *obj = cm36686_obj;
	uint16_t enable=0;
	int res;
	sscanf(buf, "%hu", &enable);
	APS_LOG("cm36686_store_als_enable entry enable:%d !!! \n", enable);
	res = cm36686_enable_als(obj->client, enable);
	if(res)
	{
		APS_ERR("als_enable_nodata is failed!!\n");
	}

	APS_LOG("cm36686_store_als_enable finished !!! \n");
	return count;
}

static ssize_t cm36686_show_pdata(struct device_driver *ddri, char *buf)
{
	struct cm36686_priv *obj = cm36686_obj;
	ssize_t len = 0;
	cm36686_read_ps(obj->client,  &obj->ps);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", obj->ps);
	return len;
}

static ssize_t cm36686_show_als_data(struct device_driver *ddri, char *buf)
{
	struct cm36686_priv *obj = cm36686_obj;
	ssize_t len = 0;
	cm36686_read_als(obj->client, &obj->als);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",  obj->als);
	return len;
}

static ssize_t cm36686_show_ps_threshold_high(struct device_driver *ddri, char *buf)
{
	struct cm36686_priv *obj = cm36686_obj;
	ssize_t len = 0;
	int temp = atomic_read(&obj->ps_eng_cali_high);
	int data =atomic_read(&obj->ps_cali_high);
	len += snprintf(buf + len, PAGE_SIZE - len, "threshold high:%d %d\n", temp, data);
	return len;
}

static ssize_t cm36686_store_ps_threshold_high(struct device_driver *ddri, const char *buf, size_t count)
{
	struct cm36686_priv *obj = cm36686_obj;
	int temp=0;
	sscanf(buf, "%d", &temp);
	atomic_set(&obj->ps_cali_high, temp);
	return count;
}

static ssize_t cm36686_show_ps_threshold_low(struct device_driver *ddri, char *buf)
{
	struct cm36686_priv *obj = cm36686_obj;
	ssize_t len = 0;
	int temp = atomic_read(&obj->ps_eng_cali_low);
	int data =atomic_read(&obj->ps_cali_low);
	len += snprintf(buf + len, PAGE_SIZE - len, "threshold low:%d %d\n", temp, data);
	return len;
}

static ssize_t cm36686_store_ps_threshold_low(struct device_driver *ddri, const char *buf, size_t count)
{
	struct cm36686_priv *obj = cm36686_obj;
	int temp=0;

	sscanf(buf, "%d", &temp);
	atomic_set(&obj->ps_cali_low, temp);
	return count;
}

static ssize_t cm36686_show_ps_threshold_offset_high(struct device_driver *ddri, char *buf)
{
	struct cm36686_priv *obj = cm36686_obj;
	ssize_t len = 0;
	int tmp_hi = atomic_read(&obj->ps_threshold_high);
	int tmp_lo = atomic_read(&obj->ps_threshold_low);
	len += snprintf(buf + len, PAGE_SIZE - len, "high=%d low=%d\n", tmp_hi, tmp_lo);
	return len;
}

static ssize_t cm36686_store_ps_threshold_offset_high(struct device_driver *ddri, const char *buf, size_t count)
{
	struct cm36686_priv *obj = cm36686_obj;
	int temp=0;

	sscanf(buf, "%d", &temp);
	atomic_set(&obj->ps_threshold_high, temp);
	return count;
}

static ssize_t cm36686_show_ps_threshold_offset_low(struct device_driver *ddri, char *buf)
{
	struct cm36686_priv *obj = cm36686_obj;
	ssize_t len = 0;
	int tmp_hi = atomic_read(&obj->ps_threshold_high);
	int tmp_lo = atomic_read(&obj->ps_threshold_low);
	len += snprintf(buf + len, PAGE_SIZE - len, "high=%d low=%d\n", tmp_hi, tmp_lo);
	return len;
}

static ssize_t cm36686_store_ps_threshold_offset_low(struct device_driver *ddri, const char *buf, size_t count)
{
	struct cm36686_priv *obj = cm36686_obj;
	int temp=0;

	sscanf(buf, "%d", &temp);
	atomic_set(&obj->ps_threshold_low, temp);
	return count;
}

static ssize_t cm36686_show_als_algo_threadhold(struct device_driver *ddri, char *buf)
{
	struct cm36686_priv *obj = cm36686_obj;
	ssize_t len = 0;
	int temp = atomic_read(&obj->als_algo_threadhold);
	len += snprintf(buf + len, PAGE_SIZE - len, "als_algo_threadhold:%d\n", temp);
	return len;
}

static ssize_t cm36686_store_als_algo_threadhold(struct device_driver *ddri, const char *buf, size_t count)
{
	struct cm36686_priv *obj = cm36686_obj;
	int temp=0;

	sscanf(buf, "%d", &temp);
	atomic_set(&obj->als_algo_threadhold, temp);
	return count;
}
static ssize_t cm36686_store_reg_write(struct device_driver *ddri, const char *buf, size_t count)
{
	struct cm36686_priv *obj = cm36686_obj;
	int reg;
	int data;
	u8 databuf[3];
	int res;
	APS_FUN();

	sscanf(buf, "%x,%x",&reg, &data);

	APS_LOG("[%s]: reg=0x%x, data=0x%x", __func__, reg, data);
	databuf[2] = data >> 8;
	databuf[1] = data & 0x00FF;
	databuf[0] = reg;
	res = CM36686_i2c_master_operate(obj->client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
	}
	return count;
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, cm36686_show_reg, NULL);
static DRIVER_ATTR(ps_enable,	S_IROTH  | S_IWOTH, 	cm36686_show_ps_enable, cm36686_store_ps_enable);
static DRIVER_ATTR(als_enable,	S_IROTH  | S_IWOTH,	 cm36686_show_als_enable, cm36686_store_als_enable);
static DRIVER_ATTR(pdata,	S_IROTH  | S_IWOTH, 	cm36686_show_pdata, NULL);
static DRIVER_ATTR(als_data,	S_IROTH  | S_IWOTH, 	cm36686_show_als_data, NULL);
static DRIVER_ATTR(ps_threshold_high,	S_IROTH  | S_IWOTH, 	cm36686_show_ps_threshold_high, cm36686_store_ps_threshold_high);
static DRIVER_ATTR(ps_threshold_low,	S_IROTH  | S_IWOTH, 	cm36686_show_ps_threshold_low, cm36686_store_ps_threshold_low);
static DRIVER_ATTR(ps_threshold_offset_high,	S_IROTH  | S_IWOTH, 	cm36686_show_ps_threshold_offset_high, cm36686_store_ps_threshold_offset_high);
static DRIVER_ATTR(ps_threshold_offset_low,	S_IROTH  | S_IWOTH, 	cm36686_show_ps_threshold_offset_low, cm36686_store_ps_threshold_offset_low);
static DRIVER_ATTR(als_algo_threadhold,	S_IROTH  | S_IWOTH, 	cm36686_show_als_algo_threadhold, cm36686_store_als_algo_threadhold);
static DRIVER_ATTR(i2c_w,	S_IROTH  | S_IWOTH, 	NULL, cm36686_store_reg_write );
/*----------------------------------------------------------------------------*/
static struct driver_attribute *cm36686_attr_list[] = {
	&driver_attr_reg,
	&driver_attr_ps_enable,
	&driver_attr_als_enable,
	&driver_attr_pdata,
	&driver_attr_als_data,
	&driver_attr_ps_threshold_high,
	&driver_attr_ps_threshold_low,
	&driver_attr_ps_threshold_offset_high,
	&driver_attr_ps_threshold_offset_low,
	&driver_attr_als_algo_threadhold,
	&driver_attr_i2c_w,
};

/*----------------------------------------------------------------------------*/
static int cm36686_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(cm36686_attr_list)/sizeof(cm36686_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, cm36686_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", cm36686_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int cm36686_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(cm36686_attr_list)/sizeof(cm36686_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, cm36686_attr_list[idx]);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static void cm36686_eint_work(struct work_struct *work)
{
	struct cm36686_priv *obj = (struct cm36686_priv *)container_of(work, struct cm36686_priv, eint_work);
	int res = 0;
	u8 databuf[2];
	int intr_flag;

	APS_LOG("cm36686_eint_work entry !!!\n");
	databuf[0] = CM36686_REG_PS_DATA;
	res = CM36686_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
	if(res<0)
	{
		APS_ERR("i2c_master_send function err res = %d\n",res);
		goto EXIT_INTR_ERR;
	}
	APS_LOG("cm36686_eint_work ps value:%d\n",((databuf[1]<<8)|databuf[0]));

	databuf[0] = CM36686_REG_INT_FLAG;
	res = CM36686_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
	if(res<0)
	{
		APS_ERR("i2c_master_send function err res = %d\n",res);
		goto EXIT_INTR_ERR;
	}
	APS_LOG("cm36686_REG_INT_FLAG value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

	if(databuf[1]&0x02)
	{
		intr_flag = 0;
	}
	else if(databuf[1]&0x01)
	{
		intr_flag = 1;
	}
	else
	{
		APS_ERR("cm36686_check_intr fail databuf[1]&0x01: %d\n", res);
		goto EXIT_INTR_ERR;
	}
	APS_LOG("cm36686 interrupt value = %d\n", intr_flag);
	res = ps_report_interrupt_data(intr_flag);

#ifdef CUST_EINT_ALS_TYPE
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
	return;
EXIT_INTR_ERR:
#ifdef CUST_EINT_ALS_TYPE
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
	APS_ERR("cm36686_eint_work err: %d\n", res);
}
/*----------------------------------------------------------------------------*/
static void cm36686_eint_func(void)
{
	struct cm36686_priv *obj = cm36686_obj;
	if(!obj)
	{
		return;
	}
#ifdef CUST_EINT_ALS_TYPE
	mt_eint_mask(CUST_EINT_ALS_NUM);
#else
	mt65xx_eint_mask(CUST_EINT_ALS_NUM);
#endif
	schedule_work(&obj->eint_work);
}
/*----------------------------------------------------------------------------*/
int cm36686_setup_eint(struct i2c_client *client)
{
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

#ifdef CUST_EINT_ALS_TYPE
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, cm36686_eint_func, 0);
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, cm36686_eint_func, 0);
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
#endif
	return 0;
}
/************************************************************/
static int set_psensor_threshold(struct i2c_client *client)
{
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	int res = 0;
	u8 databuf[3];
	int ps_value = -1;
	cm36686_read_ps(obj->client, &obj->ps);
	APS_LOG("set_psensor_threshold cm36686_read_ps value:%d\n", obj->ps);
	if((obj->ps > 0)&& (obj->ps < 1000))
	{
		atomic_set(&obj->ps_eng_cali_high, obj->ps + atomic_read(&obj->ps_threshold_high));
		atomic_set(&obj->ps_eng_cali_low,  obj->ps + atomic_read(&obj->ps_threshold_low));
	}
	else
	{
		atomic_set(&obj->ps_eng_cali_high, atomic_read(&obj->ps_cali_high)+100);
		atomic_set(&obj->ps_eng_cali_low,  atomic_read(&obj->ps_cali_low)+100);
	}   

	databuf[0] = CM36686_REG_PS_THDL;
	databuf[1] = atomic_read(&obj->ps_eng_cali_low )& 0x00ff;
	databuf[2] = atomic_read(&obj->ps_eng_cali_low) >> 8;//threshold value need to confirm
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		return -1;
	}

	databuf[0] = CM36686_REG_PS_THDH;
	databuf[1] = atomic_read(&obj->ps_eng_cali_high )& 0x00ff;
	databuf[2] = atomic_read(&obj->ps_eng_cali_high )>> 8;//threshold value need to confirm
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		return -1;
	}
	APS_ERR("set_psensor_threshold function high: %d, low:%d\n",atomic_read(&obj->ps_eng_cali_high),atomic_read(&obj->ps_eng_cali_low));
	cm36686_read_ps(obj->client, &obj->ps);
	if (obj->ps < atomic_read(&obj->ps_eng_cali_low))
	{
		ps_value = 1;
	}
	else if(obj->ps >  atomic_read(&obj->ps_eng_cali_high))
	{
		ps_value = 0;
	}
	ps_report_interrupt_data(ps_value);

	return 0;
}
/************************************************PS CALI*****************************************************************************/
static void cm36686_early_suspend(struct early_suspend *h)
{
	struct cm36686_priv *obj = container_of(h, struct cm36686_priv, early_drv);	
	int err;
	APS_LOG("cm36686_early_suspend entry!!!\n"); 
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	if (1 == test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = cm36686_enable_als(obj->client, 0))
		{
			APS_ERR("cm36686_early_suspend disable als fail: %d\n", err); 
		}
		atomic_set(&obj->als_suspend, 1);
	}
	APS_LOG("cm36686_early_suspend finished!!!\n"); 
}

static void cm36686_late_resume(struct early_suspend *h) 
{
	struct cm36686_priv *obj = container_of(h, struct cm36686_priv, early_drv);		  
	int err;
	APS_LOG("cm36686_late_resume entry!!!\n"); 
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	if(1 == atomic_read(&obj->als_suspend))
	{
		if((err = cm36686_enable_als(obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);		  

		}
		atomic_set(&obj->als_suspend, 0);
	}
	APS_LOG("cm36686_late_resume finished!!!\n"); 
}
/*--------------------------------------------------------------------------------*/
static int cm36686_init_client(struct i2c_client *client)
{
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3];    
	int res = 0;
	APS_FUN();
	databuf[0] = CM36686_REG_ALS_CONF;
	if(1 == obj->hw->polling_mode_als)
		databuf[1] = 0x01;//0x81;
	else
		databuf[1] = 0x03;//0x83;	
	databuf[2] = 0x00;
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	APS_LOG("cm36686 ps CM36686_REG_ALS_CONF command!\n");

	is_als_first_read_after_enable = false;	

	databuf[0] = CM36686_REG_PS_CONF1_2;
	databuf[1] = 0x2c;//0x37;
	databuf[2] = 0x00;
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	APS_LOG("cm36686 ps CM36686_REG_PS_CONF1_2 command!\n");

	databuf[0] = CM36686_REG_PS_CONF3_MS;
	databuf[1] = 0x10;
	if(1 == obj->hw->polling_mode_ps)
		databuf[2] = 0x45;//LED_I=160mA
	else
		databuf[2] =0x05;// 0x05;//LED_I=160mA
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	APS_LOG("cm36686 ps CM36686_REG_PS_CONF3_MS command!\n");

	databuf[0] = CM36686_REG_PS_CANC;//value need to confirm
	databuf[1] = 0x00;
	databuf[2] = 0x00;
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}

	APS_LOG("cm36686 ps CM36686_REG_PS_CANC command!\n");

	if(0 == obj->hw->polling_mode_als){
		databuf[0] = CM36686_REG_ALS_THDH;
		databuf[1] = 0x00;
		databuf[2] = atomic_read(&obj->als_thd_val_high);
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto EXIT_ERR;
		}
		databuf[0] = CM36686_REG_ALS_THDL;
		databuf[1] = 0x00;
		databuf[2] = atomic_read(&obj->als_thd_val_low);//threshold value need to confirm
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto EXIT_ERR;
		}
	}
	if(0 == obj->hw->polling_mode_ps)
	{
		databuf[0] = CM36686_REG_PS_THDL;
		databuf[1] = atomic_read(&obj->ps_eng_cali_low )& 0x00ff;
		databuf[2] = atomic_read(&obj->ps_eng_cali_low) >> 8;//threshold value need to confirm
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);

		databuf[0] = CM36686_REG_PS_THDH;
		databuf[1] = atomic_read(&obj->ps_eng_cali_high )& 0x00ff;
		databuf[2] = atomic_read(&obj->ps_eng_cali_high) >> 8;//threshold value need to confirm
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto EXIT_ERR;
		}
	}
	res = cm36686_setup_eint(client);
	if(res!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}

	return CM36686_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}
/*--------------------------------------------------------------------------------*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int als_enable_nodata(int en)
{
	struct cm36686_priv *obj = cm36686_obj;
	int res = 0;
	APS_LOG("cm36686_obj als enable value = %d\n", en);

	if(!obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return -1;
	}
	res=	cm36686_enable_als(obj->client, en);
	if(res)
	{
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_get_data(int* value, int* status)
{
	struct cm36686_priv *obj = cm36686_obj;
	int err = 0;

	if(!obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return -1;
	}
	if((err = cm36686_read_als(obj->client, &obj->als)))
	{
		err = -1;
	}
	else
	{
		//APS_LOG("als_get_data value = %d\n", obj->als);
		//if (atomic_read(&obj->als_algo_threadhold) == obj->als)
		if (0 != obj->als)
		{
			if(atomic_read(&obj->als_algo_noise) == obj->als)
			{
 				atomic_set(&obj->als_algo_noise, 0);
				obj->als += 1;
			}
			else
			{
				atomic_set(&obj->als_algo_noise, obj->als);
			}
		}
		else
		{
			if (0 != atomic_read(&obj->als_algo_noise))
			{
				atomic_set(&obj->als_algo_noise, 0);
			}
		}
		*value = obj->als;//cm36686_get_als_value(obj, obj->als);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
	return err;
}

// if use  this type of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int ps_enable_nodata(int en)
{
	struct cm36686_priv *obj = cm36686_obj;
	int res = 0;
	APS_LOG("cm36686_obj ps enable value = %d\n", en);

	if(!obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return -1;
	}
	res=	cm36686_enable_ps(obj->client, en);
	if(res)
	{
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	cm36686_read_ps(obj->client, &obj->ps);
	APS_LOG("ps_enable_nodata cm36686_read_ps value:%d\n", obj->ps);
	return 0;
}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int* value, int* status)
{
	struct cm36686_priv *obj = cm36686_obj;
	int err = 0;
	if(!obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return -1;
	}

	if((err = cm36686_read_ps(obj->client, &obj->ps)))
	{
		err = -1;;
	}
	else
	{
		*value = cm36686_get_ps_value(obj, obj->ps);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
	return 0;
}
/*-----------------------------------i2c operations----------------------------------*/
static int cm36686_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cm36686_priv *obj;
	int err = 0;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
	int databuf[2];
	APS_LOG("cm36686_i2c_probe entry!!!\n");
	databuf[0] = 0x00;
	err = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(err<0)
	{
		APS_ERR("probe REG 0x00 read error\n");
		goto exit;
	}
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(*obj));
	cm36686_obj = obj;

	obj->hw = get_cust_alsps_hw();//get custom file data struct

	INIT_WORK(&obj->eint_work, cm36686_eint_work);

	obj->client = client;
	i2c_set_clientdata(client, obj);
	/*-----------------------------value need to be confirmed-----------------------------------------*/
	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 200);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_eng_cali_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_eng_cali_low,  obj->hw->ps_threshold_low);
	atomic_set(&obj->als_thd_val_high,  obj->hw->als_threshold_high);
	atomic_set(&obj->als_thd_val_low,  obj->hw->als_threshold_low);
	atomic_set(&obj->ps_ftm_cali_high,  1000);
	atomic_set(&obj->ps_ftm_cali_low,  900);
	atomic_set(&obj->ps_cali_high,  1000);
	atomic_set(&obj->ps_cali_low,  900);
	atomic_set(&obj->ps_threshold_high,  350);//used for set high threshold
	atomic_set(&obj->ps_threshold_low,  250);//used for set low threshold
	atomic_set(&obj->als_algo_threadhold,  52);
	atomic_set(&obj->als_algo_noise, 0);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	/*-----------------------------value need to be confirmed-----------------------------------------*/

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	cm36686_i2c_client = client;

	if((err = cm36686_init_client(client)))
	{
		goto exit_init_failed;
	}
	APS_LOG("cm36686_init_client() OK!\n");
	/*------------------------cm36686 attribute file for debug--------------------------------------*/
	if((err = cm36686_create_attr(&(cm36686_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------cm36686 attribute file for debug--------------------------------------*/
	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = obj->hw->is_batch_supported_als;
#else
	als_ctl.is_support_batch = false;
#endif

	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = true;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = obj->hw->is_batch_supported_ps;
#else
	ps_ctl.is_support_batch = false;
#endif

	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_EARLYSUSPEND)
	obj->early_drv.level  = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
		obj->early_drv.suspend  = cm36686_early_suspend,
		obj->early_drv.resume  = cm36686_late_resume,    
		register_early_suspend(&obj->early_drv);
#endif

	cm36686_init_flag =0;
	APS_LOG("%s: OK  gpio:%d \n", __func__, mt_get_gpio_in(GPIO_ALS_EINT_PIN));
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_init_failed:
	kfree(obj);
exit:
	cm36686_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
	cm36686_init_flag = -1;
	return err;
}

static int cm36686_i2c_remove(struct i2c_client *client)
{
	int err;	
	/*------------------------cm36686 attribute file for debug--------------------------------------*/	
	if((err = cm36686_delete_attr(&(cm36686_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("cm36686_delete_attr fail: %d\n", err);
	} 
	/*----------------------------------------------------------------------------------------*/

	cm36686_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;

}

static int cm36686_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, CM36686_DEV_NAME);
	return 0;

}

static int cm36686_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}

static int cm36686_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
static int cm36686_remove(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	cm36686_power(hw, 0);
	i2c_del_driver(&cm36686_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int  cm36686_local_init(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_LOG("cm36686_local_init entry!!!\n");
	cm36686_power(hw, 1);
	if(i2c_add_driver(&cm36686_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}
	if(-1 == cm36686_init_flag)
	{
		APS_ERR("cm36686_local_init failed!!!\n");
		return -1;
	}
	APS_LOG("cm36686_local_init finished!!!\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init cm36686_init(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_LOG("%s: i2c_number=%d\n", __func__, hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &i2c_cm36686, 1);
	alsps_driver_add(&cm36686_init_info);
	APS_LOG("%s finished!!!\n", __func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit cm36686_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(cm36686_init);
module_exit(cm36686_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("yucong xiong");
MODULE_DESCRIPTION("cm36686 driver");
MODULE_LICENSE("GPL");
