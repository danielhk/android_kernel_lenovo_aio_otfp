/* drivers/hwmon/mt6516/amit/epl2182.c - EPL2182 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <linux/hwmsen_helper.h>
#include "epl21822.h"

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/sched.h>

#include <alsps.h>
#undef CUSTOM_KERNEL_SENSORHUB
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif
/******************************************************************************
 * extern functions
 *******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);

/******************************************************************************
 * configuration
 *******************************************************************************/
#define MED1    1
#define MED2    0
//#if MED2
bool ps_suspend_flag = false;
//#endif
/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 start*/
#define DYNAMIC_INTT    1
/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 end*/
#define PS_DYN_K           1
#define PS_THD_PATCH       1
// TODO: change ps/als integrationtime
int PS_INTT = 4;
int ALS_INTT = 7;

#define TXBYTES 				2
#define RXBYTES 				2
#define PACKAGE_SIZE 			2
#define I2C_RETRY_COUNT 		3

/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 start*/
#if DYNAMIC_INTT
// TODO: change delay time
//modify ELAN Rober at 2014/11/24
//#define PS_DELAY 			20
#define PS_DELAY 			55
#define ALS_DELAY 			165
/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 start*/
#define ALS_INIT_DELAY 		15

int dynamic_intt_init_flag = 1;
int dynamic_intt_init_count = 0;
/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 end*/

int dynamic_intt_idx;
int dynamic_intt_init_idx = 1; //2;	//initial dynamic_intt_idx
/*Lenovo-sw caoyi1 modify for als real lux 2014-04-15 start*/
int c_gain = 280; // 10.0/1000 *10000 //0.66
/*Lenovo-sw caoyi1 modify for als real lux 2014-04-15 end*/

uint8_t dynamic_intt_intt;
uint8_t dynamic_intt_gain;
/*Lenovo-sw caoyi1 modify for cycle als lux 2014-04-16 start*/
uint8_t dynamic_intt_cycle;
/*Lenovo-sw caoyi1 modify for cycle als lux 2014-04-16 end*/
uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;
uint32_t dynamic_intt_max_lux = 87000;  //8700
uint32_t dynamic_intt_min_lux = 5;      //0.5
uint32_t dynamic_intt_min_unit = 1000;  //1000: float, 1000:integer

static int als_dynamic_intt_intt[] = {EPL_ALS_INTT_4096, EPL_ALS_INTT_256, EPL_ALS_INTT_16, EPL_ALS_INTT_16};
static int als_dynamic_intt_intt_value[] = {4096, 256, 16, 16};
static int als_dynamic_intt_gain[] = {EPL_M_GAIN, EPL_M_GAIN, EPL_M_GAIN, EPL_L_GAIN};
/*Lenovo-sw caoyi1 modify for light sensor low-lux value 2014-08-11 start*/
//static int als_dynamic_intt_high_thr[] = {900, 800, 400, 3200};
//static int als_dynamic_intt_low_thr[] = {32, 32, 32, 256};
static int als_dynamic_intt_high_thr[] = {7200, 6400, 3200, 3200*8};
static int als_dynamic_intt_low_thr[] = {32, 320, 320, 320*8};
/*Lenovo-sw caoyi1 modify for light sensor low-lux value 2014-08-11 end*/

static int als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_intt_value)/sizeof(int);

int als_report_idx = 0;

#else

// TODO: change delay time
/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
#define PS_DELAY 			20
/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/
#define ALS_DELAY 			40

#endif
/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 end*/
// TODO: parameters for lux equation y = ax + b
/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
#define LUX_PER_COUNT		100              // 1100 = 1.1 * 1000
/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/

#define IPI_WAIT_RSP_TIMEOUT    (HZ/10)     //100ms

static DEFINE_MUTEX(epl2182_mutex);
static DEFINE_MUTEX(epl2182_ps_mutex);

typedef struct _epl_raw_data
{
	u8 raw_bytes[PACKAGE_SIZE];
	u16 ps_raw;
	u16 ps_state;
	u16 ps_int_state;
	u16 als_ch0_raw;
	u16 als_ch1_raw;
	u16 ps_thd_fac_val_high;
	u16 ps_thd_fac_val_low;
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
	u16 als_lux;
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/
} epl_raw_data;


#if PS_DYN_K
bool ps_dynk_flag = true;
int ps_dynk_count;
int ps_dynk_avg=1;
u32 ps_dynk_sum;
u16 ps_sta;
int ps_dynk_h_offset=460;
int ps_dynk_l_offset=200;
int ps_dynk_max_ch0=4500;
int ps_dynk_max_ch1=4800;

int ps_dynk_lux = 3000;
#endif

#define EPL2182_DEV_NAME     "EPL2182"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_DEBUG APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_DEBUG fmt, ##args)
#define FTM_CUST_ALSPS "/data/epl2182"

#define POWER_NONE_MACRO MT65XX_POWER_NONE

static struct i2c_client *epl2182_i2c_client = NULL;


/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl2182_i2c_id[] = {{"EPL2182",0},{}};
static struct i2c_board_info __initdata i2c_EPL2182= { I2C_BOARD_INFO("EPL2182", (0X92>>1))};

/*----------------------------------------------------------------------------*/
static int epl2182_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl2182_i2c_remove(struct i2c_client *client);
static int epl2182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);

static int alsps_local_init(void);
static int alsps_remove(void);
/*----------------------------------------------------------------------------*/
static int epl2182_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl2182_i2c_resume(struct i2c_client *client);
#ifndef CUSTOM_KERNEL_SENSORHUB
static void epl2182_eint_func(void);
#endif
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
static int set_psensor_threshold(struct i2c_client *client);

static struct epl2182_priv *g_epl2182_ptr = NULL;
static bool isInterrupt = false;

/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
static void polling_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);
/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/

#ifndef CUSTOM_KERNEL_SENSORHUB
static long long int_top_time = 0;
static int int_flag = 0;
#endif


/*----------------------------------------------------------------------------*/
typedef enum
{
	CMC_TRC_ALS_DATA = 0x0001,
	CMC_TRC_PS_DATA = 0X0002,
	CMC_TRC_EINT    = 0x0004,
	CMC_TRC_IOCTL   = 0x0008,
	CMC_TRC_I2C     = 0x0010,
	CMC_TRC_CVT_ALS = 0x0020,
	CMC_TRC_CVT_PS  = 0x0040,
	CMC_TRC_DEBUG   = 0x0800,
} CMC_TRC;

/*----------------------------------------------------------------------------*/
typedef enum
{
	CMC_BIT_ALS    = 1,
	CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct epl2182_i2c_addr      /*define a series of i2c slave address*/
{
	u8  write_addr;
	u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct epl2182_priv
{
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct  eint_work;
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
	struct workqueue_struct *epl_wq;
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/
#ifdef CUSTOM_KERNEL_SENSORHUB
	struct work_struct init_done_work;
#endif
	/*i2c address group*/
	struct epl2182_i2c_addr  addr;

	int enable_pflag;
	int enable_lflag;

	/*misc*/
	atomic_t    trace;
	atomic_t    i2c_retry;
	atomic_t    als_suspend;
	atomic_t    als_debounce;   /*debounce time after enabling als*/
	atomic_t    als_deb_on;     /*indicates if the debounce is on*/
	atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
	atomic_t    ps_mask;        /*mask ps: always return far away*/
	atomic_t    ps_debounce;    /*debounce time after enabling ps*/
	atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
	atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
	atomic_t    ps_suspend;

	/*data*/
	u16         als;
	u16         ps;
	u16		lux_per_count;
	bool   		als_enable;    /*record current als status*/
	bool    	ps_enable;     /*record current ps status*/
	ulong       enable;         /*record HAL enalbe status*/
	ulong       pending_intr;   /*pending interrupt*/
	//ulong        first_read;   // record first read ps and als

	/*data*/
	u16         als_level_num;
	u16         als_value_num;
	u32         als_level[C_CUST_ALS_LEVEL-1];
	u32         als_value[C_CUST_ALS_LEVEL];
	int			ps_cali;

	atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend    early_drv;
#endif
};



/*----------------------------------------------------------------------------*/
static struct i2c_driver epl2182_i2c_driver =
{
	.probe      = epl2182_i2c_probe,
	.remove     = epl2182_i2c_remove,
	.detect     = epl2182_i2c_detect,
	.suspend    = epl2182_i2c_suspend,
	.resume     = epl2182_i2c_resume,
	.id_table   = epl2182_i2c_id,
	.driver = {
		.name           = EPL2182_DEV_NAME,
	},
};


static struct epl2182_priv *epl2182_obj = NULL;
static epl_raw_data	gRawData;

static int alsps_init_flag =-1; // 0<==>OK -1 <==> fail

static struct alsps_init_info epl2182_init_info = {
	.name = EPL2182_DEV_NAME,
	.init = alsps_local_init,
	.uninit = alsps_remove,

};

static DECLARE_WAIT_QUEUE_HEAD(wait_rsp_wq);

static atomic_t wait_rsp_flag = ATOMIC_INIT(0);

//static struct wake_lock als_lock; /* Bob.chen add for if ps run, the system forbid to goto sleep mode. */
static struct wake_lock ps_lock;
/*
//====================I2C write operation===============//
//regaddr: ELAN epl2182 Register Address.
//bytecount: How many bytes to be written to epl2182 register via i2c bus.
//txbyte: I2C bus transmit byte(s). Single byte(0X01) transmit only slave address.
//data: setting value.
//
// Example: If you want to write single byte to 0x1D register address, show below
//	      elan_epl2182_I2C_Write(client,0x1D,0x01,0X02,0xff);
//
 */
static int elan_epl2182_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
	uint8_t buffer[2];
	int ret = 0;
	int retry;

	//APS_DBG("[ELAN epl2182] %s\n", __func__);
	mutex_lock(&epl2182_mutex);
	buffer[0] = (regaddr<<3) | bytecount ;
	buffer[1] = data;


	//APS_DBG("---elan_epl2182_I2C_Write register (0x%x) buffer data (%x) (%x)---\n",regaddr,buffer[0],buffer[1]);

	for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
	{
		ret = i2c_master_send(client, buffer, txbyte);
		if (ret >= 0)
		{
			break;
		}

		APS_DBG("epl2182 i2c write error,TXBYTES %d\r\n",ret);
		mdelay(10);
	}

	if(retry>=I2C_RETRY_COUNT)
	{
		mutex_unlock(&epl2182_mutex);
		APS_DBG(KERN_ERR "[ELAN epl2182 error] %s i2c write retry over %d\n",__func__, I2C_RETRY_COUNT);
		return -EINVAL;
	}
	mutex_unlock(&epl2182_mutex);
	return ret;
}




/*
//====================I2C read operation===============//
 */
static int elan_epl2182_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t rxbyte, uint8_t *data)
{
	uint8_t buffer[RXBYTES];
	int ret = 0, i =0;
	int retry;

	//APS_DBG("[ELAN epl2182] %s\n", __func__);
	mutex_lock(&epl2182_mutex);
	buffer[0] = (regaddr<<3) | bytecount ;

	for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
	{
		ret = hwmsen_read_block(client, buffer[0], buffer, rxbyte);
		if (ret >= 0)
			break;

		APS_ERR("epl2182 i2c read error,RXBYTES %d\r\n",ret);
		mdelay(10);
	}

	if(retry>=I2C_RETRY_COUNT)
	{
		APS_ERR(KERN_ERR "[ELAN epl2182 error] %s i2c read retry over %d\n",__func__, I2C_RETRY_COUNT);
		mutex_unlock(&epl2182_mutex);
		return -EINVAL;
	}

	for(i=0; i<PACKAGE_SIZE; i++)
		*data++ = buffer[i];
	mutex_unlock(&epl2182_mutex);
	//APS_DBG("----elan_epl2182_I2C_Read Receive data from (0x%x):byte1 (%x) byte2 (%x)-----\n",regaddr, buffer[0], buffer[1]);

	return ret;
}

#if PS_DYN_K
static int epl2182_read_data_for_cali(struct i2c_client *client, HWMON_PS_STRUCT *ps_data_cali);
static void epl2182_WriteCalibration(struct epl2182_priv *obj, HWMON_PS_STRUCT *data_cali);

static void epl2182_do_ps_dynk(struct epl2182_priv *epld, u16 sta, u16 ch0, u16 ch1)
{
	HWMON_PS_STRUCT ps_cali_temp;

	APS_LOG("[%s]: sta=%d, ch0=%d, ch1=%d, gRawData.als_lux=%d, ps_dynk_max_ch0=%d, ps_dynk_max_ch1=%d \n", __func__, sta, ch0, ch1, gRawData.als_lux, ps_dynk_max_ch0, ps_dynk_max_ch1);
	ps_dynk_count++;
	if((sta == 0) && (ch0 < ps_dynk_max_ch0) && (ch1 < ps_dynk_max_ch1) && gRawData.als_lux < ps_dynk_lux)
	{
		ps_dynk_sum += ch1;
		if(ps_dynk_count >= ps_dynk_avg)
		{
			u16 ps_dynk_avg_ch1;
			ps_dynk_avg_ch1 = ps_dynk_sum/ps_dynk_avg;
			epld->hw->ps_threshold_high = ps_dynk_avg_ch1+ps_dynk_h_offset;       //CT+delta
			epld->hw->ps_threshold_low = ps_dynk_avg_ch1+ps_dynk_l_offset; //CT+0.6*delta
			atomic_set(&epld->ps_thd_val_high,  epld->hw->ps_threshold_high);
			atomic_set(&epld->ps_thd_val_low,  epld->hw->ps_threshold_low);//need to confirm
			APS_LOG("[%s]:PS_DYNK sueecss! ps_dynk_avg_ch1=%d, L=%d, H=%d \n", __func__, ps_dynk_avg_ch1, epld->hw->ps_threshold_low, epld->hw->ps_threshold_high);
			ps_dynk_flag = false;
		}
	}
	else
	{
		/* lenovo-sw youwc1 20141225: use default value but factory cali value when dynk cali fail start */
		atomic_set(&epld->ps_thd_val_high, gRawData.ps_thd_fac_val_high);
		atomic_set(&epld->ps_thd_val_low,  gRawData.ps_thd_fac_val_low);
		/* lenovo-sw youwc1 20141225: use default value but factory cali value when dynk cali fail end */

		//update threshold , elan Robert
		//epld->ps_thd_val_high = gRawData.ps_thd_fac_val_high;
		//epld->ps_thd_val_low = gRawData.ps_thd_fac_val_low;
		//update threshold , elan Robert

		APS_LOG("[%s]:PS_DYNK sueecss! , L=%d, H=%d \n", __func__ , gRawData.ps_thd_fac_val_low, gRawData.ps_thd_fac_val_high);
		ps_dynk_flag = false;
	}
}
#endif

bool ps_resume_flag = true;
static int elan_epl2182_psensor_enable(struct epl2182_priv *epl_data, int enable)
{
	int ret = 0;

	int ps_threshold_high_value;
	int ps_threshold_low_value;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else
	uint8_t regdata;
	uint8_t read_data[2];
	int ps_state;
	struct i2c_client *client = epl_data->client;
#endif
#if PS_DYN_K
	u16 ps_ch0;
#endif
#ifdef CUSTOM_KERNEL_SENSORHUB
	req.activate_req.sensorType = ID_PROXIMITY;
	req.activate_req.action = SENSOR_HUB_ACTIVATE;
	req.activate_req.enable = enable;
	len = sizeof(req.activate_req);
	ret = SCP_sensorHub_req_send(&req, &len, 1);
	//Handle delay control in SCP side.
	atomic_set(&wait_rsp_flag, 1);
	if (0 == wait_event_interruptible_timeout(wait_rsp_wq, atomic_read(&wait_rsp_flag) == 0, IPI_WAIT_RSP_TIMEOUT))
	{
		APS_ERR("Wait IPI response timeout!\n");
	}
	else
	{
		//gRawData.ps_state has been updated in eint_work.
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	epl_data->enable_pflag = enable;
	ret = elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE | EPL_DRIVE_120MA);

	if(enable)
	{
		// modify lenovo-sw youwc1 at 2014/12/16
		regdata = EPL_SENSING_1_TIME | EPL_PS_MODE | EPL_M_GAIN ;
		regdata = regdata | (isInterrupt ? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE);
		ret = elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

		//modify ELAN Rober at 2014/11/24
		//regdata = PS_INTT<<4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
		regdata = PS_INTT<<4 | EPL_PST_1_TIME | EPL_14BIT_ADC;
		ret = elan_epl2182_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);
		//set_psensor_intr_threshold(epl_data->hw ->ps_threshold_low,epl_data->hw ->ps_threshold_high);
		set_psensor_threshold(client);

		ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
		ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
		//ret = elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW | EPL_DRIVE_120MA);//yucong add
		msleep(PS_DELAY);

#if PS_DYN_K
		elan_epl2182_I2C_Read(client,REG_14,R_TWO_BYTE,0x02,read_data);
		ps_ch0 = (read_data[1]<<8) | read_data[0];
#endif
		elan_epl2182_I2C_Read(client,REG_16,R_TWO_BYTE,0x02,read_data);
		gRawData.ps_raw = (read_data[1]<<8) | read_data[0];

		/*
		   ret = elan_epl2182_I2C_Read(client,REG_13,R_SINGLE_BYTE,0x01,read_data);
		   ps_state= !((read_data[0]&0x04)>>2);
		   int_flag = ps_state;
		 */
		ps_state = gRawData.ps_state;

		ps_threshold_high_value = atomic_read(&epl_data->ps_thd_val_high);
		ps_threshold_low_value = atomic_read(&epl_data->ps_thd_val_low);

#if PS_DYN_K
		if(ps_dynk_flag == true)
		{
			elan_epl2182_I2C_Read(client, REG_13, R_SINGLE_BYTE, 0x01, gRawData.raw_bytes);
			ps_sta = ((gRawData.raw_bytes[0]&0x02)>>1);
			epl2182_do_ps_dynk(epl_data, ps_sta, ps_ch0, gRawData.ps_raw);

			if(gRawData.ps_raw > ps_threshold_high_value)
				ps_state = 0;
			else
				ps_state = 1;
		}
#endif
		if(isInterrupt)
		{
#if PS_THD_PATCH
			//modify by elan Robert, using ps raw data compare dynamic threshold to report ps event.
			if((gRawData.ps_raw > ps_threshold_high_value)&& gRawData.ps_state ==1)
				ps_state = 0;
			else if((gRawData.ps_raw < ps_threshold_low_value)&& gRawData.ps_state ==0)
				ps_state = 1;
			/*
			   else
			   ps_state = gRawData.ps_state ;
			 */
			// APS_LOG("real state=%d, raw=%d, h=%d, l=%d\n", ps_state,gRawData.ps_raw,ps_threshold_high_value, ps_threshold_low_value);
			/*
			   u8 ps_state_tmp;
			   ps_state_tmp = ps_state;
			   APS_LOG("[%s]:real ps_state = %d\n", __func__, ps_state_tmp);

			   if((ps_state_tmp==0 && gRawData.ps_raw > &epl_data->ps_thd_val_high) ||
			   (ps_state_tmp==1 && gRawData.ps_raw < &epl_data->ps_thd_val_low))
			   {
			   APS_LOG("change ps_state(ps_state_tmp=%d, gRawData.ps_state=%d) \r\n", ps_state_tmp, gRawData.ps_state);
			   ps_state = ps_state_tmp;
			   }
			   else
			   {
			   ps_state = gRawData.ps_state;
			   APS_LOG("change ps_state(ps_state_tmp=%d, gRawData.ps_state=%d) \r\n", ps_state_tmp, gRawData.ps_state);
			   }
			 */
#endif
			// ps_state = 1- gRawData.ps_state; //modify by elan  Rober
			if(ps_state != gRawData.ps_state)
			{
				APS_LOG("[%s]: ps_suspend_flag = %d \r\n", __func__, ps_suspend_flag);
				//wake_lock_timeout(&ps_lock, 2*HZ); //20 ms??
				gRawData.ps_state = ps_state;//update ps state
				elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW | EPL_DRIVE_120MA);
				if(ps_suspend_flag == false)
				{
					ps_report_interrupt_data(gRawData.ps_state);
					APS_LOG("ps_report_interrupt_data gRawData.ps_state:%d, gRawData.ps_raw=%d \n ", gRawData.ps_state, gRawData.ps_raw);
				}
				ps_resume_flag = false;
			}
			else
			{
				//elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW);
				elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW | EPL_DRIVE_120MA);
			}
		}
		/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
	}
	else
	{
		regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_GAIN ;
		regdata = regdata | EPL_S_SENSING_MODE;
		ret = elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);
		ret = elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE | EPL_DRIVE_120MA);//yucong add
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	if(ret<0)
	{
		APS_ERR("[ELAN epl2182 error]%s: ps enable %d fail\n",__func__,ret);
	}
	else
	{
		ret = 0;
	}
	return ret;
}


bool als_first_enable_report = false;
static int elan_epl2182_lsensor_enable(struct epl2182_priv *epl_data, int enable)
{
	int ret = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	uint8_t regdata;

	struct i2c_client *client = epl_data->client;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	//APS_LOG("[ELAN epl2182] %s enable = %d\n", __func__, enable);

	epl_data->enable_lflag = enable;

	if(enable)
	{
#ifdef CUSTOM_KERNEL_SENSORHUB
		req.activate_req.sensorType = ID_LIGHT;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = enable;
		len = sizeof(req.activate_req);
		ret = SCP_sensorHub_req_send(&req, &len, 1);
		if (ret)
		{
			APS_ERR("SCP_sensorHub_req_send!\n");
		}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
		regdata = EPL_INT_DISABLE;
		ret = elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02, regdata);
		/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 start*/
#if DYNAMIC_INTT
		/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 start*/
		if(dynamic_intt_init_flag == 0)
		{
			dynamic_intt_cycle = EPL_SENSING_1_TIME;
		}
		else
		{
			dynamic_intt_cycle = EPL_SENSING_16_TIME;
		}
		//APS_LOG("[%s]: dynamic_intt_cycle=%d \r\n", __func__, dynamic_intt_cycle>>5);
		/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 end*/
		regdata = EPL_S_SENSING_MODE | dynamic_intt_cycle | EPL_ALS_MODE | dynamic_intt_gain;
		ret = elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

		/*Lenovo-sw caoyi1 modify DYNAMIC_INTT function 2014-07-18 start*/
		regdata = dynamic_intt_intt<<4 | EPL_PST_1_TIME | EPL_8BIT_ADC;
		ret = elan_epl2182_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);
		/*Lenovo-sw caoyi1 modify DYNAMIC_INTT function 2014-07-18 end*/
#else
		/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
		regdata = EPL_S_SENSING_MODE | EPL_SENSING_4_TIME | EPL_ALS_MODE | EPL_M_GAIN;
		ret = elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);
		/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/

		regdata = ALS_INTT<<4 | EPL_PST_1_TIME | EPL_10BIT_ADC;
		ret = elan_epl2182_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);
#endif
		/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 end*/
		ret = elan_epl2182_I2C_Write(client,REG_10,W_SINGLE_BYTE,0X02,0x3e);
		ret = elan_epl2182_I2C_Write(client,REG_11,W_SINGLE_BYTE,0x02,0x3e);

		ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
		ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
		/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 start*/
		if(dynamic_intt_init_flag == 0)
		{
			msleep(ALS_INIT_DELAY);
		}
		else
		{
			msleep(ALS_DELAY);
		}
		/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 end*/
	}
	if(ret<0)
	{
		APS_ERR("[ELAN epl2182 error]%s: als_enable %d fail\n",__func__,ret);
	}
	else
	{
		ret = 0;
	}
	return ret;
}

/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 start*/
#if DYNAMIC_INTT
long raw_convert_to_lux(u16 raw_data)
{
	long lux = 0;

	/* lux = C*count*(4096/intt)*(64/Gain), Gain:H=64, M=8, L=1 */
	lux = c_gain * raw_data * (4096 / als_dynamic_intt_intt_value[dynamic_intt_idx]);
	if(lux >= (dynamic_intt_max_lux*1000)){
		APS_LOG("raw_convert_to_lux: change max lux\r\n");
		lux = dynamic_intt_max_lux * 1000;
	}
	else if(lux <= (dynamic_intt_min_lux*1000)){
		APS_LOG("raw_convert_to_lux: change min lux\r\n");
		lux = dynamic_intt_min_lux * 1000;
	}
	return lux;
}
#endif
/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 end*/

//convert raw to lux
static int epl2182_get_als_value(struct epl2182_priv *obj, u16 als)
{
	/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 start*/
#if DYNAMIC_INTT
	uint32_t now_lux, lux_tmp;
	int als_com = 0;
	elan_epl2182_I2C_Read(obj->client,REG_13,R_TWO_BYTE,0x02,gRawData.raw_bytes);

	als_com = (gRawData.raw_bytes[0]&0x04)>>2;
	if(als_com == 1)
	{
		if(dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)){
			lux_tmp = dynamic_intt_max_lux * 1000;
		}
		else
		{
			als  = dynamic_intt_high_thr;
			obj->als  = dynamic_intt_high_thr;
			gRawData.als_ch1_raw = dynamic_intt_high_thr;
			lux_tmp = raw_convert_to_lux(als);
			dynamic_intt_idx++;
		}
		//APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>> channel output has saturated!\n");
	}
	else
	{
		if(als > dynamic_intt_high_thr)
		{
			if(dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)){
				lux_tmp = dynamic_intt_max_lux * 1000;
				//APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MAX_LUX\r\n");
			}
			else
			{
				als  = dynamic_intt_high_thr;
				obj->als  = dynamic_intt_high_thr;
				gRawData.als_ch1_raw = dynamic_intt_high_thr;
				lux_tmp = raw_convert_to_lux(als);
				dynamic_intt_idx++;
				//APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>>change INTT high: %d, raw: %d \r\n", dynamic_intt_idx, als);
			}
		}
		else if(als < dynamic_intt_low_thr)
		{
			if(dynamic_intt_idx == 0){
				lux_tmp = dynamic_intt_min_lux * 1000;
				//APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MIN_LUX\r\n");
			}
			else
			{
				als  = dynamic_intt_low_thr;
				obj->als = dynamic_intt_low_thr;
				gRawData.als_ch1_raw = dynamic_intt_low_thr;
				lux_tmp = raw_convert_to_lux(als);
				dynamic_intt_idx--;
				//APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>>change INTT low: %d, raw: %d \r\n", dynamic_intt_idx, als);
			}
		}
		else
		{
			lux_tmp = raw_convert_to_lux(als);
		}
	}

	now_lux = lux_tmp / dynamic_intt_min_unit;
	//APS_LOG("-------------------  ALS raw = %d, now_lux = %d   \r\n",  als, now_lux);

	dynamic_intt_intt = als_dynamic_intt_intt[dynamic_intt_idx];
	dynamic_intt_gain = als_dynamic_intt_gain[dynamic_intt_idx];
	dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
	dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
	return now_lux/10;
#else

	int idx;
	int invalid = 0;
	int lux = 0;

	if(als < 15)
	{
		//APS_DBG("epl2182 ALS: %05d => 0\n", als);
		return 0;
	}

	lux = (als * obj->lux_per_count)/1000;

	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(lux < obj->hw->als_level[idx])
		{
			break;
		}
	}

	if(idx >= obj->als_value_num)
	{
		APS_ERR("epl2182 exceed range\n");
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

		//APS_DBG("ALS: %d [%d, %d] => %d [%d, %d] \n", als, level_low, level_high, value, value_low, value_high);
		return value;
#endif
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
		return -1;
	}
#endif
	/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 end*/
}


static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA data;
	EPL2182_CUST_DATA *pCustData;
	int len;

	//ps_cali would be add back in SCP side.
	low_thd -= epl2182_obj->ps_cali;
	high_thd -= epl2182_obj->ps_cali;

	data.set_cust_req.sensorType = ID_PROXIMITY;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	pCustData = (EPL2182_CUST_DATA *)(&data.set_cust_req.custData);

	pCustData->setPSThreshold.action = EPL2182_CUST_ACTION_SET_PS_THRESHODL;
	pCustData->setPSThreshold.threshold[0] = low_thd;
	pCustData->setPSThreshold.threshold[1] = high_thd;
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setPSThreshold);

	ret = SCP_sensorHub_req_send(&data, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	struct epl2182_priv *epld = epl2182_obj;
	struct i2c_client *client = epld->client;
	uint8_t high_msb ,high_lsb, low_msb, low_lsb;

	//APS_LOG("epl2182 %s: low_thd = 0x%X, high_thd = 0x%x \n",__func__, low_thd, high_thd);

	high_msb = (uint8_t) (high_thd >> 8);
	high_lsb = (uint8_t) (high_thd & 0x00ff);
	low_msb  = (uint8_t) (low_thd >> 8);
	low_lsb  = (uint8_t) (low_thd & 0x00ff);

	elan_epl2182_I2C_Write(client,REG_2,W_SINGLE_BYTE,0x02,high_lsb);
	elan_epl2182_I2C_Write(client,REG_3,W_SINGLE_BYTE,0x02,high_msb);
	elan_epl2182_I2C_Write(client,REG_4,W_SINGLE_BYTE,0x02,low_lsb);
	elan_epl2182_I2C_Write(client,REG_5,W_SINGLE_BYTE,0x02,low_msb);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	return ret;
}
/*----------------------------------------------------------------------------*/
int hw8k_init_device(struct i2c_client *client)
{
	APS_LOG("hw8k_init_device.........\r\n");

	epl2182_i2c_client=client;

	APS_LOG("epl2182 I2C Addr==[0x%x],line=%d\n",epl2182_i2c_client->addr,__LINE__);

	return 0;
}

/*----------------------------------------------------------------------------*/
int epl2182_get_addr(struct alsps_hw *hw, struct epl2182_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}


/*----------------------------------------------------------------------------*/
static void epl2182_power(struct alsps_hw *hw, unsigned int on)
{
#ifndef FPGA_EARLY_PORTING
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "EPL2182"))
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "EPL2182"))
			{
				APS_ERR("power off fail!!\n");
			}
		}
	}
	power_on = on;
#endif //#ifndef FPGA_EARLY_PORTING
}

/*----------------------------------------------------------------------------*/

int epl2182_read_als(struct i2c_client *client, u16 *data)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA reqData;
	EPL2182_CUST_DATA *pCustData;
	int len;

	reqData.set_cust_req.sensorType = ID_LIGHT;
	reqData.set_cust_req.action = SENSOR_HUB_SET_CUST;
	pCustData = (EPL2182_CUST_DATA *)(&reqData.set_cust_req.custData);

	pCustData->getALSRawData.action = EPL2182_CUST_ACTION_GET_ALS_RAW_DATA;
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getALSRawData);

	SCP_sensorHub_req_send(&reqData, &len, 1);

	//gRawData.als_ch0_raw and gRawData.als_ch1_raw would be updated in eint_work.
	atomic_set(&wait_rsp_flag, 1);
	if (0 == wait_event_interruptible_timeout(wait_rsp_wq, atomic_read(&wait_rsp_flag) == 0, IPI_WAIT_RSP_TIMEOUT))
	{
		APS_ERR("Wait IPI response timeout!\n");
	}
	else
	{
		*data = gRawData.als_ch1_raw;
	}

#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	uint8_t read_data[2];
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	elan_epl2182_I2C_Read(obj->client,REG_14,R_TWO_BYTE,0x02,read_data);
	gRawData.als_ch0_raw = (read_data[1]<<8) | read_data[0];

	elan_epl2182_I2C_Read(obj->client,REG_16,R_TWO_BYTE,0x02,read_data);
	gRawData.als_ch1_raw = (read_data[1]<<8) | read_data[0];
	*data =  gRawData.als_ch1_raw;

	//APS_LOG("epl2182 read als raw data = %d\n", gRawData.als_ch1_raw);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	return 0;
}


/*----------------------------------------------------------------------------*/
long epl2182_read_ps(struct i2c_client *client, u16 *data)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA reqData;
	EPL2182_CUST_DATA *pCustData;
	int len;

	reqData.set_cust_req.sensorType = ID_PROXIMITY;
	reqData.set_cust_req.action = SENSOR_HUB_SET_CUST;
	pCustData = (EPL2182_CUST_DATA *)(&reqData.set_cust_req.custData);

	pCustData->getPSRawData.action = EPL2182_CUST_ACTION_GET_PS_RAW_DATA;
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getPSRawData);

	SCP_sensorHub_req_send(&reqData, &len, 1);

	atomic_set(&wait_rsp_flag, 1);
	if (0 == wait_event_interruptible_timeout(wait_rsp_wq, atomic_read(&wait_rsp_flag) == 0, IPI_WAIT_RSP_TIMEOUT))
	{
		APS_ERR("Wait IPI response timeout!\n");
	}
	else
	{
		//gRawData.ps_raw and gRawData.ps_state would be updated in eint_work.
	}

#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	uint8_t read_data[2];
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	elan_epl2182_I2C_Read(obj->client,REG_16,R_TWO_BYTE,0x02,read_data);
	APS_DBG("epl2182_read_ps read REG_16 raw_bytes_high: 0x%x, raw_bytes_low: 0x%x\n",read_data[1],read_data[0]);
	gRawData.ps_raw = (read_data[1]<<8) | read_data[0];

#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
	*data = gRawData.ps_raw;
	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
	APS_LOG("epl2182 read ps raw data = %d\n", gRawData.ps_raw);
	APS_LOG("epl2182 read ps binary data = %d\n", gRawData.ps_state);

	return 0;
}


/*----------------------------------------------------------------------------*/
#ifdef CUSTOM_KERNEL_SENSORHUB
static void alsps_init_done_work(struct work_struct *work)
{
	struct epl2182_priv *obj = g_epl2182_ptr;
	EPL2182_CUST_DATA *p_cust_data;
	SCP_SENSOR_HUB_DATA data;
	int max_cust_data_size_per_packet;
	int i;
	uint sizeOfCustData;
	uint len;
	char *p = (char *)obj->hw;

	p_cust_data = (EPL2182_CUST_DATA *)data.set_cust_req.custData;

	data.set_cust_req.sensorType = ID_LIGHT;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	sizeOfCustData = sizeof(*(obj->hw));
	p_cust_data->setCust.action = EPL2182_CUST_ACTION_SET_CUST;
	max_cust_data_size_per_packet = sizeof(data.set_cust_req.custData) - offsetof(EPL2182_SET_CUST, data);

	for (i=0;sizeOfCustData>0;i++)
	{
		p_cust_data->setCust.part = i;
		if (sizeOfCustData > max_cust_data_size_per_packet)
		{
			len = max_cust_data_size_per_packet;
		}
		else
		{
			len = sizeOfCustData;
		}

		memcpy(p_cust_data->setCust.data, p, len);
		sizeOfCustData -= len;
		p += len;

		len += offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + offsetof(EPL2182_SET_CUST, data);
		SCP_sensorHub_req_send(&data, &len, 1);
	}
}
#endif
/*----------------------------------------------------------------------------*/
#ifndef CUSTOM_KERNEL_SENSORHUB
void epl2182_eint_func(void)
{
	struct epl2182_priv *obj = g_epl2182_ptr;

	int_top_time = sched_clock();

	if(!obj)
	{
		return;
	}

#ifndef FPGA_EARLY_PORTING
	mt_eint_mask(CUST_EINT_ALS_NUM);
#endif //#ifndef FPGA_EARLY_PORTING
	schedule_work(&obj->eint_work);
}
#else
static int alsps_irq_handler(void* data, uint len)
{
	struct epl2182_priv *obj = g_epl2182_ptr;
	SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P)data;

	if(!obj)
	{
		return -1;
	}

	switch(rsp->rsp.action)
	{
		case SENSOR_HUB_NOTIFY:
			switch(rsp->notify_rsp.event)
			{
				case SCP_INIT_DONE:
					schedule_work(&obj->init_done_work);
					break;
				case SCP_NOTIFY:
					if (EPL2182_NOTIFY_PROXIMITY_CHANGE == rsp->notify_rsp.event)
					{
						gRawData.ps_state = rsp->notify_rsp.data[0];
						schedule_work(&obj->eint_work);
					}
					else
					{
						APS_ERR("Unknow notify");
					}
					break;
				default:
					APS_ERR("Error sensor hub notify");
					break;
			}
			break;
		default:
			APS_ERR("Error sensor hub action");
			break;
	}

	return 0;
}
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

/*----------------------------------------------------------------------------*/
static void epl2182_eint_work(struct work_struct *work)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
	int res = 0;

	res = ps_report_interrupt_data(gRawData.ps_state);
	if(res != 0)
	{
		APS_ERR("epl2182_eint_work err: %d\n", res);
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	struct epl2182_priv *epld = g_epl2182_ptr;
	int err;
	uint8_t read_data[2];
	int flag;

	mutex_lock(&epl2182_ps_mutex);
	if(epld->enable_pflag==0)
	{
		goto exit;
	}

	APS_ERR("epl2182 int top half time = %lld\n", int_top_time);
	elan_epl2182_I2C_Read(epld->client,REG_16,R_TWO_BYTE,0x02,read_data);
	gRawData.ps_raw = (read_data[1]<<8) | read_data[0];
	APS_LOG("epl2182 ps raw_data = %d\n", gRawData.ps_raw);

	elan_epl2182_I2C_Read(epld->client,REG_13,R_SINGLE_BYTE,0x01,read_data);
	flag = !((read_data[0]&0x04)>>2);

#if PS_THD_PATCH
	APS_LOG("[%s]:real ps_state = %d\n", __func__, flag);
	if(gRawData.ps_raw > &epld->ps_thd_val_high && gRawData.ps_state ==1)
		flag = 0;
	else if(gRawData.ps_raw < &epld->ps_thd_val_low && gRawData.ps_state ==0)
		flag = 1;	
	/*	
		u8 ps_state;
		ps_state = flag;
		if((gRawData.ps_raw > &epld->ps_thd_val_high && ps_state == 0) ||
		(gRawData.ps_raw < &epld->ps_thd_val_low && ps_state == 1))
		{
		flag = ps_state;
		APS_LOG("[%s]:change ps_state(ps_state=%d, gRawData.ps_state=%d) \r\n", __func__ ,ps_state, gRawData.ps_state);
		}
		else
		{
		flag = gRawData.ps_state;
		APS_LOG("[%s]:change ps_state(ps_state=%d, gRawData.ps_state=%d) \r\n", __func__,ps_state, gRawData.ps_state);
		}
	 */	
#else

	if(gRawData.ps_raw > &epld->ps_thd_val_high && flag == 0)
		flag = 0;
	else if(gRawData.ps_raw < &epld->ps_thd_val_low && flag == 1)
		flag = 1;
#endif

	//if(flag != gRawData.ps_state)
	//{
	APS_LOG("epl2182 eint work gRawData.ps_state = %d, flag = %d, %s\n", gRawData.ps_state, flag, __func__);

	gRawData.ps_state = flag;//update ps state

	wake_lock_timeout(&ps_lock, 1*HZ);
	//let up layer to know
	if((err = ps_report_interrupt_data(gRawData.ps_state)))
	{
		APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
	}
	//}
#if 0
	else
	{
		APS_LOG("epl2182 eint data won't update");
		//APS_LOG("epl2182 eint work gRawData.ps_state = %d, flag = %d, %s\n", gRawData.ps_state, flag, __func__);
	}
#endif
exit:
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
	elan_epl2182_I2C_Write(epld->client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW);
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/
	elan_epl2182_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);

	mutex_unlock(&epl2182_ps_mutex);
	mt_eint_unmask(CUST_EINT_ALS_NUM);

	/*
	   if(test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0)
	   {
	   APS_LOG("als enable eint mask ps!\n");
#ifndef FPGA_EARLY_PORTING
mt_eint_mask(CUST_EINT_ALS_NUM);
#endif //#ifndef FPGA_EARLY_PORTING
}
else{
APS_LOG("als disable eint unmask ps!\n");
#ifndef FPGA_EARLY_PORTING
mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif //#ifndef FPGA_EARLY_PORTING
}
	 */
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	}

/*----------------------------------------------------------------------------*/
int epl2182_setup_eint(struct i2c_client *client)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
	int err = 0;

	err = SCP_sensorHub_rsp_registration(ID_PROXIMITY, alsps_irq_handler);

	return err;
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	struct epl2182_priv *obj = i2c_get_clientdata(client);

	APS_LOG("epl2182_setup_eint\n");


	g_epl2182_ptr = obj;

	/*configure to GPIO function, external interrupt*/

#ifndef FPGA_EARLY_PORTING
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, epl2182_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif //#ifndef FPGA_EARLY_PORTING

	return 0;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
}




/*----------------------------------------------------------------------------*/
static int epl2182_init_client(struct i2c_client *client)
{
#ifndef CUSTOM_KERNEL_SENSORHUB
	struct epl2182_priv *obj = i2c_get_clientdata(client);
#endif
	int err=0;

	APS_LOG("epl2182 [Agold spl] I2C Addr==[0x%x],line=%d\n",epl2182_i2c_client->addr,__LINE__);

	/*  interrupt mode */


	APS_FUN();

#ifdef CUSTOM_KERNEL_SENSORHUB
	epl2182_setup_eint(client);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(obj->hw->polling_mode_ps == 0)
	{
#ifndef FPGA_EARLY_PORTING
		mt_eint_mask(CUST_EINT_ALS_NUM);
#endif //#ifndef FPGA_EARLY_PORTING

		if((err = epl2182_setup_eint(client)))
		{
			APS_ERR("setup eint: %d\n", err);
			return err;
		}
		APS_LOG("epl2182 interrupt setup\n");
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB


	if((err = hw8k_init_device(client)) != 0)
	{
		APS_ERR("init dev: %d\n", err);
		return err;
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl2182_show_reg(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = epl2182_obj->client;
	ssize_t len = 0;

	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return 0;
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

	return len;

}

/*----------------------------------------------------------------------------*/
static ssize_t epl2182_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct epl2182_priv *epld = epl2182_obj;
	uint8_t read_data[2];
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return 0;
	}
	elan_epl2182_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_LOCK);

	elan_epl2182_I2C_Read(epld->client,REG_16,R_TWO_BYTE,0x02,read_data);
	gRawData.ps_raw = (read_data[1]<<8) | read_data[0];
	APS_LOG("ch1 raw_data = %d\n", gRawData.ps_raw);

	elan_epl2182_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);

	len += snprintf(buf+len, PAGE_SIZE-len, "ch1 raw is %d\n",gRawData.ps_raw);
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl2182_store_als_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return 0;
	}

	sscanf(buf, "%d", &ALS_INTT);
	APS_LOG("als int time is %d\n", ALS_INTT);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl2182_store_ps_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return 0;
	}
	sscanf(buf, "%d", &PS_INTT);
	APS_LOG("ps int time is %d\n", PS_INTT);
	return count;
}

/*Lenovo-sw caoyi1 add for p-sensor cali 2014-07-17 start*/
#if 1 //ELAN_WRITE_CALI lenovo-sw molg1 add for PS cali 20141031
static ssize_t epl2182_show_ps_cali_close(struct device_driver *ddri, char *buf)
{
	struct epl2182_priv *obj = epl2182_obj;
	ssize_t len = 0;

	APS_FUN();
	len += snprintf(buf+len, PAGE_SIZE-len, "ret = %d\n", atomic_read(&obj->ps_thd_val_high));

	return len;
}

static ssize_t epl2182_store_ps_cali_close(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl2182_priv *obj = epl2182_obj;
	int cali_high=0;
	APS_FUN();

	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return 0;
	}
	sscanf(buf, "%d", &cali_high);

	atomic_set(&obj->ps_thd_val_high, cali_high);
	return count;
}

static ssize_t epl2182_show_ps_cali_far(struct device_driver *ddri, char *buf)
{
	struct epl2182_priv *obj = epl2182_obj;
	ssize_t len = 0;

	APS_FUN();
	len += snprintf(buf+len, PAGE_SIZE-len, "ret = %d\n", atomic_read(&obj->ps_thd_val_low));

	return len;
}
static ssize_t epl2182_store_ps_cali_far(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl2182_priv *obj = epl2182_obj;
	int cali_low=0;
	APS_FUN();

	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return 0;
	}
	sscanf(buf, "%d", &cali_low);

	atomic_set(&obj->ps_thd_val_low, cali_low);
	return count;
}

#endif

#if PS_DYN_K
static ssize_t epl2182_store_ps_dynk_average(struct device_driver *ddri, const char *buf, size_t count)
{
	APS_FUN();
	sscanf(buf, "%d", &ps_dynk_avg);
	return count;
}

static ssize_t epl2182_store_ps_dynk_max_ch0_ch1(struct device_driver *ddri, const char *buf, size_t count)
{
	int mode=0;

	APS_FUN();
	sscanf(buf, "%d,%d", &ps_dynk_max_ch0, &ps_dynk_max_ch1);

	return count;
}

static ssize_t epl2182_store_ps_dynk_offset(struct device_driver *ddri, const char *buf, size_t count)
{
	int mode=0;

	APS_FUN();
	sscanf(buf, "%d,%d", &ps_dynk_l_offset, &ps_dynk_h_offset);

	return count;
}
#endif

/*Lenovo-sw caoyi1 add for p-sensor cali 2014-07-17 end*/
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, epl2182_show_status,  NULL);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, epl2182_show_reg,   NULL);
static DRIVER_ATTR(als_int_time,     S_IWUSR | S_IRUGO, NULL,   epl2182_store_als_int_time);
static DRIVER_ATTR(ps_int_time,     S_IWUSR | S_IRUGO, NULL,   epl2182_store_ps_int_time);
/*Lenovo-sw caoyi1 add for p-sensor cali 2014-07-17 start*/
#if  1//ELAN_WRITE_CALI lenovo-sw molg1 add for PS cali 20141031
static DRIVER_ATTR(ps_cali_close,				S_IROTH  | S_IWOTH, epl2182_show_ps_cali_close, epl2182_store_ps_cali_close);
static DRIVER_ATTR(ps_cali_far,				S_IROTH  | S_IWOTH, epl2182_show_ps_cali_far, epl2182_store_ps_cali_far);
#endif
/*Lenovo-sw caoyi1 add for p-sensor cali 2014-07-17 end*/
#if PS_DYN_K
static DRIVER_ATTR(ps_dyn_avg,				S_IROTH  | S_IWOTH, NULL,			epl2182_store_ps_dynk_average);
static DRIVER_ATTR(ps_dyn_max_ch0_ch1,		S_IROTH  | S_IWOTH, NULL,			epl2182_store_ps_dynk_max_ch0_ch1);
static DRIVER_ATTR(ps_dyn_l_h_offset,				S_IROTH  | S_IWOTH, NULL,			epl2182_store_ps_dynk_offset);
#endif

/*----------------------------------------------------------------------------*/
static struct driver_attribute * epl2182_attr_list[] =
{
	&driver_attr_status,
	&driver_attr_reg,
	&driver_attr_als_int_time,
	&driver_attr_ps_int_time,
	/*Lenovo-sw caoyi1 add for p-sensor cali 2014-07-17 start*/
#if 1//ELAN_WRITE_CALI lenovo-sw molg1 add for PS cali 20141031
	&driver_attr_ps_cali_close,
	&driver_attr_ps_cali_far,
#endif
	/*Lenovo-sw caoyi1 add for p-sensor cali 2014-07-17 end*/
#if PS_DYN_K
	&driver_attr_ps_dyn_avg,
	&driver_attr_ps_dyn_max_ch0_ch1,
	&driver_attr_ps_dyn_l_h_offset,
#endif
};

/*----------------------------------------------------------------------------*/
static int epl2182_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(epl2182_attr_list)/sizeof(epl2182_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, epl2182_attr_list[idx])))
		{
			APS_ERR("driver_create_file (%s) = %d\n", epl2182_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}



/*----------------------------------------------------------------------------*/
static int epl2182_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(epl2182_attr_list)/sizeof(epl2182_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, epl2182_attr_list[idx]);
	}

	return err;
}



/******************************************************************************
 * Function Configuration
 ******************************************************************************/
static int epl2182_open(struct inode *inode, struct file *file)
{
	file->private_data = epl2182_i2c_client;

	APS_FUN();

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int epl2182_release(struct inode *inode, struct file *file)
{
	APS_FUN();
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/

/*Lenovo-sw caoyi1 for proximity sensor cali 20140711 start*/
static int epl2182_ps_average_val = 0;

static void epl2182_WriteCalibration(struct epl2182_priv *obj, HWMON_PS_STRUCT *data_cali)
{
	// struct PS_CALI_DATA_STRUCT *ps_data_cali;
	APS_LOG("le_WriteCalibration  1 %d," ,data_cali->close);
	APS_LOG("le_WriteCalibration  2 %d," ,data_cali->far_away);
	APS_LOG("le_WriteCalibration  3 %d,", data_cali->valid);
	//APS_LOG("le_WriteCalibration  4 %d,", data_cali->pulse);

	if(data_cali->valid == 1)
	{
		atomic_set(&obj->ps_thd_val_high, data_cali->close);
		atomic_set(&obj->ps_thd_val_low, data_cali->far_away);
		gRawData.ps_thd_fac_val_high = data_cali->close;
		gRawData.ps_thd_fac_val_low = data_cali->far_away;
	}else{
		/*lenovo-sw molg1 add default value for uncalibration 20141222 begin*/
		gRawData.ps_thd_fac_val_high = 4000;
		gRawData.ps_thd_fac_val_low = 3000;
		/*lenovo-sw molg1 add default value for uncalibration 20141222 end*/
	}
}
static int epl2182_read_data_for_cali(struct i2c_client *client, HWMON_PS_STRUCT *ps_data_cali)
{
	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
	u8 databuf[2];
	u8 buffer[2];
	int i=0 ,res = 0;
	/*Lenovo huangdra 20130713 modify u16 will overflow,change to u32*/
	u16 data[32];
	u32 sum=0, data_cali=0;
	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
	bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);

	if(enable_ps == 0)
	{
		set_bit(CMC_BIT_PS, &obj->enable);
		queue_delayed_work(obj->epl_wq, &polling_work,msecs_to_jiffies(5));
	}
	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/

	for(i = 0; i < 10; i++)
	{
		mdelay(40);//50
		/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
		//res = epl2182_read_ps(client,&data[i]);
		data[i] = gRawData.ps_raw;
		/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
		if(res != 0)
		{
			APS_ERR("epl2182_read_data_for_cali fail: %d\n", i);
			break;
		}
		else
		{
			APS_LOG("[%d]sample = %d\n", i, data[i]);
			sum += data[i];
		}
	}


	if(i < 10)
	{
		res=  -1;

		return res;
	}
	else
	{
		data_cali = sum / 10;
		epl2182_ps_average_val = data_cali;
		APS_LOG("epl2182_read_data_for_cali data = %d",data_cali);
		/*Lenovo-sw caoyi1 modify for Aubest p-sensor calibration 2014-08-06 start*/
		if( data_cali>6000)
		{
			APS_ERR("epl2182_read_data_for_cali fail value to high: %d\n", data_cali);
			return -2;
		}
#if PS_DYN_K
		if((data_cali>0)&&(data_cali <=10))
		{
			ps_data_cali->close =data_cali + ps_dynk_h_offset;
			ps_data_cali->far_away =data_cali + ps_dynk_l_offset;
		}
		else if(data_cali <=100)
		{
			ps_data_cali->close =data_cali + ps_dynk_h_offset;
			ps_data_cali->far_away =data_cali + ps_dynk_l_offset;
		}
		else if(data_cali <=200)
		{
			ps_data_cali->close =data_cali + ps_dynk_h_offset;
			ps_data_cali->far_away =data_cali + ps_dynk_l_offset;
		}
		else if(data_cali<6000)
		{
			ps_data_cali->close =data_cali + ps_dynk_h_offset;
			ps_data_cali->far_away =data_cali + ps_dynk_l_offset;
		}
		else
		{
			APS_ERR("epl2182_read_data_for_cali IR current 0\n");
			return -2;
		}
#else
		if((data_cali>0)&&(data_cali <=10))
		{
			ps_data_cali->close =data_cali + 460;
			ps_data_cali->far_away =data_cali + 200;
		}
		else if(data_cali <=100)
		{
			ps_data_cali->close =data_cali + 460;
			ps_data_cali->far_away =data_cali + 200;
		}
		else if(data_cali <=200)
		{
			ps_data_cali->close =data_cali + 460;
			ps_data_cali->far_away =data_cali + 200;
		}
		else if(data_cali<6000)
		{
			ps_data_cali->close =data_cali + 460;
			ps_data_cali->far_away =data_cali + 200;
		}
		else
		{
			APS_ERR("epl2182_read_data_for_cali IR current 0\n");
			return -2;
		}
#endif
		ps_data_cali->valid = 1;
		/*Lenovo-sw caoyi1 modify for Aubest p-sensor calibration 2014-08-06 end*/
		APS_LOG("rpr400_read_data_for_cali close = %d,far_away = %d,valid = %d\n",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);
		//APS_LOG("rpr400_read_data_for_cali avg=%d max=%d min=%d\n",data_cali, max, min);
	}

	return res;
}
/*Lenovo-sw caoyi1 for proximity sensor cali 20140711 end*/

static int set_psensor_threshold(struct i2c_client *client)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	int databuf[2];
	int res = 0;
	databuf[0] = atomic_read(&obj->ps_thd_val_low);
	databuf[1] = atomic_read(&obj->ps_thd_val_high);//threshold value need to confirm

	res = set_psensor_intr_threshold(databuf[0],databuf[1]);
	return res;
}

/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
static void polling_do_work(struct work_struct *work)
{
	struct epl2182_priv *epld = epl2182_obj;
	struct i2c_client *client = epld->client;

	bool enable_als=test_bit(CMC_BIT_ALS, &epld->enable);
	bool enable_ps=test_bit(CMC_BIT_PS, &epld->enable);
	int err;

	cancel_delayed_work(&polling_work);
	/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 start*/
	if(dynamic_intt_init_flag == 0)
	{
		queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(PS_DELAY * 2 + ALS_INIT_DELAY + 30));
		dynamic_intt_init_count++;
		if(dynamic_intt_init_count > (als_dynamic_intt_intt_num - 1))  //1
		{
			dynamic_intt_init_flag = 1;
			APS_LOG("[%s]: ALS first report: gRawData.als_lux = %d \r\n", __func__, gRawData.als_lux);
			if((err = als_report_interrupt_data(gRawData.als_lux)))
			{
				APS_ERR("epl2182 call als_report_interrupt_data fail = %d\n", err);
			}
			cancel_delayed_work(&polling_work);
			queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(PS_DELAY * 2 + ALS_DELAY + 30));
		}
		APS_LOG("[%s]: dynamic_intt_init_count=%d \r\n", __func__, dynamic_intt_init_count);
	}
	else
	{
		queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(PS_DELAY * 2 + ALS_DELAY + 30));
	}
	/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 end*/
	//if(enable_als && atomic_read(&epld->als_suspend)==0 && test_bit(CMC_BIT_PS, &epld->pending_intr)==0)
	if(enable_als)//molg1 add for ALS first value in dark env 20150513 
	{
		mutex_lock(&epl2182_ps_mutex);
		elan_epl2182_lsensor_enable(epld, 1);
		epl2182_read_als(client, &gRawData.als_ch1_raw);
		mutex_unlock(&epl2182_ps_mutex);
		/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 start*/
#if DYNAMIC_INTT
		gRawData.als_lux=epl2182_get_als_value(epld,gRawData.als_ch1_raw);
#else
		gRawData.als_lux=gRawData.als_ch1_raw*LUX_PER_COUNT/1000;
#endif
		/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 end*/
	}
	if(enable_ps && atomic_read(&epld->ps_suspend)==0  && test_bit(CMC_BIT_PS, &epld->pending_intr)==0)
	{
		mutex_lock(&epl2182_ps_mutex);
		elan_epl2182_psensor_enable(epld, 1);
		mutex_unlock(&epl2182_ps_mutex);
		if(isInterrupt==0)
		{
			epl2182_read_ps(epl2182_obj->client, &epl2182_obj->ps);
		}
	}
#if MED2
	if(ps_suspend_flag == true)
	{
		APS_LOG("[%s]: ps_suspend_flag=%d \r\n", __func__, ps_suspend_flag);
		elan_epl2182_psensor_enable(epld, 1);
		cancel_delayed_work(&polling_work);
	}
#endif
	if(!enable_als && !enable_ps)
	{
		cancel_delayed_work(&polling_work);
		elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
		elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,EPL_S_SENSING_MODE);
		APS_LOG("polling_do_work disable sensor!\n");
	}

}
/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/

/*----------------------------------------------------------------------------*/
static long epl2182_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	/*Lenovo-sw caoyi1 for proximity sensor cali 20140711 start*/
	HWMON_PS_STRUCT ps_cali_temp;
	/*Lenovo-sw caoyi1 for proximity sensor cali 20140711 end*/
	int ps_result;
	int ps_cali;
	int threshold[2];
	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
	bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
	bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);
	/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/

#ifdef CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA data;
	EPL2182_CUST_DATA *pCustData;
	int len;

	data.set_cust_req.sensorType = ID_PROXIMITY;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	pCustData = (EPL2182_CUST_DATA *)(&data.set_cust_req.custData);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	//APS_LOG("---epl2182_ioctll- ALSPS_SET_PS_CALIBRATION  = %x, cmd = %x........\r\n", ALSPS_SET_PS_CALIBRATION, cmd);

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
			if(enable)
			{
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			queue_delayed_work(obj->epl_wq, &polling_work,msecs_to_jiffies(5));
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
			break;


		case ALSPS_GET_PS_MODE:
			enable=test_bit(CMC_BIT_PS, &obj->enable);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;


		case ALSPS_GET_PS_DATA:
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
			if(enable_ps == 0)
			{
				set_bit(CMC_BIT_PS, &obj->enable);
				queue_delayed_work(obj->epl_wq, &polling_work,msecs_to_jiffies(5));
			}
			dat = gRawData.ps_state;
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
			APS_LOG("ioctl ps state value = %d \n", dat);

			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;


		case ALSPS_GET_PS_RAW_DATA:
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
			if(enable_ps == 0)
			{
				set_bit(CMC_BIT_PS, &obj->enable);
				queue_delayed_work(obj->epl_wq, &polling_work,msecs_to_jiffies(5));
			}
			dat = gRawData.ps_raw;
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
			APS_LOG("ioctl ps raw value = %d \n", dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;


		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 start*/
#if DYNAMIC_INTT
				dynamic_intt_idx = dynamic_intt_init_idx;
				dynamic_intt_intt = als_dynamic_intt_intt[dynamic_intt_idx];
				dynamic_intt_gain = als_dynamic_intt_gain[dynamic_intt_idx];
				dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
				dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
				als_report_idx = 0;
				/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 start*/
				dynamic_intt_init_flag = 1;
				dynamic_intt_init_count = 0;
				/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 end*/
#endif
				/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 end*/
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
			queue_delayed_work(obj->epl_wq, &polling_work,msecs_to_jiffies(5));
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
			break;



		case ALSPS_GET_ALS_MODE:
			enable=test_bit(CMC_BIT_ALS, &obj->enable);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;



		case ALSPS_GET_ALS_DATA:
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
			if(enable_als == 0)
			{
				set_bit(CMC_BIT_ALS, &obj->enable);
				queue_delayed_work(obj->epl_wq, &polling_work,msecs_to_jiffies(5));
			}
			dat = gRawData.als_lux;
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
			APS_LOG("ioctl get als data = %d\n", dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;


		case ALSPS_GET_ALS_RAW_DATA:
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 begin*/
			if(enable_als == 0)
			{
				set_bit(CMC_BIT_ALS, &obj->enable);
				queue_delayed_work(obj->epl_wq, &polling_work,msecs_to_jiffies(5));
			}
			dat = gRawData.als_ch1_raw;
			/*Lenovo-sw caoyi1 modify for proximity sensor cali raw data 20140717 end*/
			APS_DBG("ioctl get als raw data = %d\n", dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
			/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			if((err = epl2182_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			if(obj->ps > atomic_read(&obj->ps_thd_val_high))
			{
				ps_result = 0;
			}
			else	ps_result = 1;

			if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;


		case ALSPS_IOCTL_CLR_CALI:
			if(copy_from_user(&dat, ptr, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(dat == 0)
				obj->ps_cali = 0;

#ifdef CUSTOM_KERNEL_SENSORHUB
			pCustData->clearCali.action = EPL2182_CUST_ACTION_CLR_CALI;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->clearCali);

			err = SCP_sensorHub_req_send(&data, &len, 1);
#endif

			break;

		case ALSPS_IOCTL_GET_CALI:
			ps_cali = obj->ps_cali ;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_IOCTL_SET_CALI:
			if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}

			obj->ps_cali = ps_cali;

#ifdef CUSTOM_KERNEL_SENSORHUB
			pCustData->setCali.action = EPL2182_CUST_ACTION_SET_CALI;
			pCustData->setCali.cali = ps_cali;
			len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setCali);

			err = SCP_sensorHub_req_send(&data, &len, 1);
#endif

			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			break;

		case ALSPS_SET_PS_THRESHOLD:
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
			APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]);
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm
			set_psensor_threshold(obj->client);
			break;

		case ALSPS_GET_PS_THRESHOLD_HIGH:
			APS_ERR("%s get threshold high before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_high));
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_THRESHOLD_LOW:
			APS_ERR("%s get threshold low before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_low));
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
			/*------------------------------------------------------------------------------------------*/

			/*Lenovo-sw caoyi1 for proximity sensor cali 20140711 start*/
		case ALSPS_SET_PS_CALI:
			if(copy_from_user(&ps_cali_temp, ptr, sizeof(ps_cali_temp)))
			{
				APS_LOG("copy_from_user\n");
				err = -EFAULT;
				break;
			}
			epl2182_WriteCalibration(obj, &ps_cali_temp);
			APS_LOG(" ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid);
			break;

			/*for ps cali work mode support -- by liaoxl.lenovo 2.08.2011 start*/
		case ALSPS_GET_PS_RAW_DATA_FOR_CALI:
			{

				cancel_work_sync(&obj->eint_work);
				mt_eint_mask(CUST_EINT_ALS_NUM);
				err = epl2182_read_data_for_cali(obj->client,&ps_cali_temp);
				if(err < 0 ){
					goto err_out;
				}
				epl2182_WriteCalibration(obj, &ps_cali_temp);
				if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
				{
					err = -EFAULT;
					goto err_out;
				}
			}
			break;
			/*for ps cali work mode support -- by liaoxl.lenovo 2.08.2011 end*/
			//lenovo-sw, shanghai, add by chenlj2, for geting ps average val, 2012-05-14 begin
		case ALSPS_GET_PS_AVERAGE:
			enable = epl2182_ps_average_val;
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
			//lenovo-sw, shanghai, add by chenlj2, for geting ps average val, 2012-05-14 end
			//lenovo-sw, shanghai, add by chenlj2, for AVAGO project, 2012-10-09 start
		case ALSPS_GET_PS_FAR_THRESHOLD:
			enable = atomic_read(&obj->ps_thd_val_low);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_PS_CLOSE_THRESHOLD:
			enable = atomic_read(&obj->ps_thd_val_high);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
			//lenovo-sw, shanghai, add by chenlj2, for AVAGO project, 2012-10-09 end
			/*Lenovo-sw caoyi1 for proximity sensor cali 20140711 end*/

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

err_out:
	return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations epl2182_fops =
{
	.owner = THIS_MODULE,
	.open = epl2182_open,
	.release = epl2182_release,
	.unlocked_ioctl = epl2182_unlocked_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice epl2182_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &epl2182_fops,
};

void epl2182_restart_polling(void)
{
	struct epl2182_priv *obj = epl2182_obj;
	cancel_delayed_work(&polling_work);
	queue_delayed_work(obj->epl_wq, &polling_work,msecs_to_jiffies(PS_DELAY * 2 + ALS_DELAY + 50));
	//queue_delayed_work(obj->epl_wq, &polling_work,msecs_to_jiffies(PS_DELAY * 2));
}
int suspend_idx = 0;
/*----------------------------------------------------------------------------*/
static int epl2182_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	int err = 0;
	uint8_t regdata;
	APS_LOG("[%s]: entry suspend_idx=%d!! \r\n", __func__, suspend_idx);
	if(suspend_idx == 0)
	{
		APS_LOG("[%s]: wake_lock! \r\n", __func__);
		wake_lock(&ps_lock);
	}
#if MED1
	if(test_bit(CMC_BIT_PS, &obj->enable) && suspend_idx == 0)
	{
		ps_suspend_flag = true;
		cancel_delayed_work(&polling_work);
		msleep(PS_DELAY * 2 + ALS_DELAY + 50);
		APS_LOG("[%s]MED1: cancel_delayed_work\n", __func__);
		//elan_epl2182_psensor_enable(obj, 1);
		//elan_epl2182_I2C_Write(obj->client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW | EPL_DRIVE_120MA);

	}
	else
	{
		APS_LOG("[%s]MED1: nothing \n", __func__);
	}
#elif MED2
	if(test_bit(CMC_BIT_PS, &obj->enable))
	{
		APS_LOG("[%s]MED2: PS enable\n", __func__);
		ps_suspend_flag = true;
		epl2182_restart_polling();
	}
#endif
	if(suspend_idx == 0)
	{
		APS_LOG("[%s]: wake_unlock! \r\n", __func__);
		wake_unlock(&ps_lock);
	}
	suspend_idx++;
	APS_LOG("[%s]:finished!!\n", __func__);
	return err;
}



/*----------------------------------------------------------------------------*/
static int epl2182_i2c_resume(struct i2c_client *client)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	int err = 0;
	APS_LOG("[%s]:entry!!\n", __func__);
#if MED1 || MED2
	ps_suspend_flag = false;
	APS_LOG("[%s]: ps_state now(%d)\n", __func__, gRawData.ps_state);
	if (gRawData.ps_state)
	{
		wake_lock_timeout(&ps_lock, 1*HZ);
		ps_report_interrupt_data(gRawData.ps_state);
	} 
#if MED2
	ps_suspend_flag = false;
#endif
	//epl2182_restart_polling();
#endif
	APS_LOG("[%s]:finished!!\n", __func__);
	return err;
}



/*----------------------------------------------------------------------------*/
static void epl2182_early_suspend(struct early_suspend *h)
{
	/*early_suspend is only applied for ALS*/
	struct epl2182_priv *obj = container_of(h, struct epl2182_priv, early_drv);
	int err;
	APS_LOG("[%s]:entry!!\n", __func__);
	suspend_idx = 0;
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 start*/
	atomic_set(&obj->als_suspend, 1);

	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		APS_LOG("[%s]: ALS enable\n", __func__);
		/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 end*/
	}
	APS_LOG("[%s]:finished!!\n", __func__);
}



/*----------------------------------------------------------------------------*/
static void epl2182_late_resume(struct early_suspend *h)
{
	/*late_resume is only applied for ALS*/
	struct epl2182_priv *obj = container_of(h, struct epl2182_priv, early_drv);
	int err;
	APS_LOG("[%s]:entry!!\n", __func__);

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 start*/


	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		//cancel_delayed_work(&polling_work);
		//queue_delayed_work(obj->epl_wq, &polling_work,msecs_to_jiffies(5));
		set_bit(CMC_BIT_ALS, &obj->enable);

		APS_LOG("[%s]: nothing polling_work \r\n", __func__);
	}

	if(test_bit(CMC_BIT_PS, &obj->enable))
	{
		APS_LOG("[%s]: PS enable \r\n", __func__);
	}

	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_suspend, 0);
#if MED1
	ps_suspend_flag = false;
	if(suspend_idx != 0)
	{
		APS_LOG("[%s]: restart polling ............\r\n", __func__);
		epl2182_restart_polling();
	}

#endif
	/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 end*/
	APS_LOG("[%s]:finished!!\n", __func__);
}

/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
	int res = 0;
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}
	APS_LOG("als_enable_nodata value = %d\n", en);

	if(en)
	{
		/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 start*/
#if DYNAMIC_INTT
		als_first_enable_report = true;
		dynamic_intt_idx = dynamic_intt_init_idx;
		dynamic_intt_intt = als_dynamic_intt_intt[dynamic_intt_idx];
		dynamic_intt_gain = als_dynamic_intt_gain[dynamic_intt_idx];
		dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
		dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
		als_report_idx = 0;
		/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 start*/
		dynamic_intt_init_flag = 0;
		dynamic_intt_init_count = 0;
		/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 end*/
#endif
		/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 end*/
		queue_delayed_work(epl2182_obj->epl_wq, &polling_work,msecs_to_jiffies(5));
		set_bit(CMC_BIT_ALS, &epl2182_obj->enable);
	}
	else
	{
		clear_bit(CMC_BIT_ALS, &epl2182_obj->enable);
	}
	APS_LOG("als_enable_nodata finished!!!\n");
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_get_data(int* value, int* status)
{
	int err = 0;

	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}
#ifdef CUSTOM_KERNEL_SENSORHUB
	if((err = epl2182_read_als(epl2182_obj->client, &epl2182_obj->als)))
	{
		err = -1;
	}
	else
	{
		*value = epl2182_get_als_value(epl2182_obj, epl2182_obj->als);
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB

	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
	/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 start*/
#if DYNAMIC_INTT
	/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 start*/
	if(dynamic_intt_init_flag == 1)
	{
		//APS_LOG("[%s]: gRawData.als_lux = %d \n", __func__, gRawData.als_lux);
	}
	else
	{
		msleep(((PS_DELAY * 2 + ALS_INIT_DELAY) * (als_dynamic_intt_intt_num -1)));
		APS_LOG("[%s]: init>>> gRawData.als_lux=%d \n", __func__, gRawData.als_lux);
	}
	*value =gRawData.als_lux;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
#else
	*value =gRawData.als_lux;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
#endif
	/*Lenovo-sw caoyi1 modify for light sensor resume value 2014-08-14 end*/
	/*Lenovo-sw caoyi1 add DYNAMIC_INTT function 2014-07-18 end*/
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/
	return err;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	int res = 0;
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}
	APS_LOG("ps_enable_nodata value = %d\n", en);
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
	if(en)
	{
		ps_resume_flag = true;
		// wake_lock(&ps_lock); //20141029 flank add
		if(isInterrupt)
		{
			//modify by chenlj2 at 2013/7/10
			//modify by ELAN Robert at 2014/11/13, ps_int_state is never used, change ps_int_state to ps_state and set ps_state in unknow.
			//elan_epl2182_psensor_enable function will check ps_state and sent currect event to up layer.
			//gRawData.ps_int_state = 2;
			gRawData.ps_state = 2;
		}
#if PS_DYN_K
		ps_dynk_flag = true;
		ps_dynk_count = 0;
		ps_dynk_sum = 0;
#endif
#if MED1
		ps_suspend_flag = false;
#endif
		queue_delayed_work(epl2182_obj->epl_wq, &polling_work,msecs_to_jiffies(5));
		set_bit(CMC_BIT_PS, &epl2182_obj->enable);
	}
	else
	{
		clear_bit(CMC_BIT_PS, &epl2182_obj->enable);
		// wake_unlock(&ps_lock); //20141029 flank add
	}
	APS_LOG("ps_enable_nodata finished!!!\n");
	return 0;
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/
}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
	*value = gRawData.ps_state;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return 0;
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/
}
/*----------------------------------------------------------------------------*/

static int epl2182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, EPL2182_DEV_NAME);
	return 0;
}
/*----------------------------------------------------------------------------*/

/* lenovo-sw youwc1 20150104: adapter psensor for different project start */
static void epl2182_set_ps_threshold_offset(struct epl2182_priv *obj)
{
	if (obj->hw->ps_threshold_high_offset != 0)
	{
#if PS_DYN_K
		ps_dynk_h_offset = obj->hw->ps_threshold_high_offset;
#endif
	}
	if (obj->hw->ps_threshold_low_offset != 0)
	{
#if PS_DYN_K
		ps_dynk_l_offset = obj->hw->ps_threshold_low_offset;
#endif
	}
}
/* lenovo-sw youwc1 20150104: adapter psensor for different project end */

static int epl2182_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct epl2182_priv *obj;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
	int err = 0;
	APS_ERR("epl2182_i2c_probe entry!!!\n");	
 	if(i2c_smbus_read_byte_data(client, 0x00) < 0)
	{
		APS_ERR("epl2182_sensor_device failed addr:%d\n", client->addr);
		err = -ENOMEM;
		goto exit;
	}	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	epl2182_obj = obj;
	obj->hw = get_cust_alsps_hw();
	epl2182_get_addr(obj->hw, &obj->addr);

	/* lenovo-sw youwc1 20150104: adapter psensor for different project start */
	epl2182_set_ps_threshold_offset(epl2182_obj);
	/* lenovo-sw youwc1 20150104: adapter psensor for different project end */

	epl2182_obj->als_level_num = sizeof(epl2182_obj->hw->als_level)/sizeof(epl2182_obj->hw->als_level[0]);
	epl2182_obj->als_value_num = sizeof(epl2182_obj->hw->als_value)/sizeof(epl2182_obj->hw->als_value[0]);
	BUG_ON(sizeof(epl2182_obj->als_level) != sizeof(epl2182_obj->hw->als_level));
	memcpy(epl2182_obj->als_level, epl2182_obj->hw->als_level, sizeof(epl2182_obj->als_level));
	BUG_ON(sizeof(epl2182_obj->als_value) != sizeof(epl2182_obj->hw->als_value));
	memcpy(epl2182_obj->als_value, epl2182_obj->hw->als_value, sizeof(epl2182_obj->als_value));

	INIT_WORK(&obj->eint_work, epl2182_eint_work);
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 start*/
	obj->epl_wq = create_singlethread_workqueue("elan_sensor_wq");
	/*Lenovo-sw caoyi1 modify for p-sensor and light sensor data 2014-07-12 end*/

#ifdef CUSTOM_KERNEL_SENSORHUB
	INIT_WORK(&obj->init_done_work, alsps_init_done_work);
#endif

	init_waitqueue_head(&wait_rsp_wq);
	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

	obj->client = client;
#ifdef FPGA_EARLY_PORTING
	obj->client->timing = 100;
#else
	obj->client->timing = 400;
#endif

	i2c_set_clientdata(client, obj);

	atomic_set(&obj->als_debounce, 2000);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 1000);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->trace, 0x00);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_thd_val_high, obj->hw ->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low, obj->hw ->ps_threshold_low);

	obj->ps_cali = 0;
	obj->ps_enable = 0;
	obj->als_enable = 0;
	obj->lux_per_count = LUX_PER_COUNT;
	obj->enable = 0;
	obj->pending_intr = 0;

	gRawData.ps_state = -1;
	atomic_set(&obj->i2c_retry, 3);
	epl2182_i2c_client = client;
	elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0x02, EPL_S_SENSING_MODE);
	elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
	if((err = epl2182_init_client(client)))
	{
		goto exit_init_failed;
	}

	if((err = misc_register(&epl2182_device)))
	{
		APS_ERR("epl2182_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = epl2182_create_attr(&epl2182_init_info.platform_diver_addr->driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	if( obj->hw->polling_mode_ps == 0)
	{
		isInterrupt=true;
	}
	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = true;
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
	/*Lenovo-sw caoyi1 modify for p-sensor  2014-07-23 start*/
	/*lenovo-sw molg1 modify for input direct20150107*/
	ps_ctl.is_report_input_direct = true;
	/*Lenovo-sw caoyi1 modify for p-sensor  2014-07-23 end*/
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = true;
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

#ifndef FPGA_EARLY_PORTING
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
		obj->early_drv.suspend  = epl2182_early_suspend,
		obj->early_drv.resume   = epl2182_late_resume,
		register_early_suspend(&obj->early_drv);
#endif
#endif

	if(isInterrupt)
		epl2182_setup_eint(client);

	alsps_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
	misc_deregister(&epl2182_device);
exit_misc_device_register_failed:
exit_init_failed:
	kfree(obj);
exit:
	epl2182_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	alsps_init_flag = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int epl2182_i2c_remove(struct i2c_client *client)
{
	int err;

	if((err = epl2182_delete_attr(&epl2182_init_info.platform_diver_addr->driver)))
	{
		APS_ERR("epl2182_delete_attr fail: %d\n", err);
	}

	if((err = misc_deregister(&epl2182_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);
	}

	epl2182_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}



/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_ERR("alsps_local_init entry\n");
	epl2182_power(hw, 1);
	if(i2c_add_driver(&epl2182_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}
	if(-1 == alsps_init_flag)
	{
		APS_ERR("alsps_local_init failed\n");	
		return -1;
	}
	APS_ERR("alsps_local_init finished!!!\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove()
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();
	epl2182_power(hw, 0);

	APS_ERR("EPL2182 remove \n");
	i2c_del_driver(&epl2182_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init epl2182_init(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &i2c_EPL2182, 1);
	alsps_driver_add(&epl2182_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit epl2182_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(epl2182_init);
module_exit(epl2182_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("yucong.xiong@mediatek.com");
MODULE_DESCRIPTION("EPL2182 ALSPS driver");
MODULE_LICENSE("GPL");
