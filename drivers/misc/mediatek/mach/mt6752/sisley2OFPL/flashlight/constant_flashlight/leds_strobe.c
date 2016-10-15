#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG printk
	#define PK_VER printk
	#define PK_ERR printk
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;


static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0x63


static struct work_struct workTimeOut;

//#define FLASH_GPIO_ENF GPIO12
//#define FLASH_GPIO_ENT GPIO13

#define LM3643_REG_ENABLE       	0x01
#define LM3643_REG_FLASH_LED1  0x03
#define LM3643_REG_FLASH_LED2  0x04
#define LM3643_REG_TORCH_LED1  0x05
#define LM3643_REG_TORCH_LED2  0x06
#define LM3643_REG_TIMING           0x08

#define FLASH_ENABLE  (GPIO136 | 0x80000000)
#define FLASH_TORCH_ENABLE  (GPIO137| 0x80000000)
#define FLASH_STROBE_ENABLE  (GPIO113 | 0x80000000)


#define FLASH_GPIO_ENT GPIO13

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);
static struct i2c_client *LM3643_i2c_client = NULL;


struct LM3643_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct LM3643_chip_data {
	struct i2c_client *client;

	struct LM3643_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};


static int LM3643_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	#if 1
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);
	#else
	u8 buffer[2];
	buffer[0] = reg;
	buffer[1] = val;
	ret = i2c_master_send(client, (char *)buffer, 2);
	#endif
	if (ret < 0)
		PK_ERR("failed writting at 0x%02x\n", reg);
	return ret;
}

static int LM3643_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0,ret=0;
	#if 1
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);
	if (val < 0) {
		PK_ERR("failed reading at 0x%02x error %d\n",reg, val);
		return val;
	}
	#else
	ret = i2c_master_send(client, (char *)&reg, 1);
	if (ret < 0)
	{
		PK_ERR("send dummy is %d\n", ret);
		return -1;
	}
	
	ret = i2c_master_recv(client, val, 1);
	if (ret < 0)
	{
		PK_ERR("recv dummy is %d\n", ret);
		return -1;
	}
	#endif

	return val;
}

int readReg(int reg)
{
    char buf[2];
	int val=0;
    buf[0]=reg;
   // iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
	val=LM3643_read_reg(LM3643_i2c_client,buf[0]);
    PK_DBG("qq reg=%x val=%x qq\n", buf[0],val);
    return val;
}

int writeReg(int reg, int data)
{
    char buf[2];
    buf[0]=reg;
    buf[1]=data;
	
   // iWriteRegI2C(buf, 2, STROBE_DEVICE_ID);
	LM3643_write_reg(LM3643_i2c_client,buf[0],buf[1]);

   return 0;
}

enum
{
	e_DutyNum = 23,
};
static int isMovieMode[e_DutyNum]={1,1,1,1,0,0,0,0,0,0,0,0,0,0,0};
static int torchDuty[e_DutyNum]=    {35,81,106,127,0,0,0,0,0,0,0,0,0,0,0};
//52,105,156,179ma
static int flashDuty[e_DutyNum]=     {4,8,12,15,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84,93,101};//lenovo-sw sunliang modfiy 2015_4_14
static int flashDutylt[e_DutyNum]=     {8,12,15,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84,93,101,110};//lenovo-sw sunliang modfiy 2015_4_14
//200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500ma
static int m_duty1=0;
static int m_duty2=0;
static int torch_flag=0;
int flashEnable_LM3643_1(void)
{
	int temp;
	temp=readReg(LM3643_REG_ENABLE);
	PK_DBG(" flashEnable_LM3643_1 line=%d,torch_flag=%d\n",__LINE__,torch_flag);
	if(torch_flag)
	{
		
		//mt_set_gpio_mode(FLASH_TORCH_ENABLE, 0);
		//mt_set_gpio_dir(FLASH_TORCH_ENABLE, GPIO_DIR_OUT);
		//mt_set_gpio_out(FLASH_TORCH_ENABLE, GPIO_OUT_ONE);		
		writeReg(LM3643_REG_TORCH_LED1, torchDuty[m_duty1]);
		 writeReg(LM3643_REG_ENABLE, temp|0x09);
	}
	else
	{
		//mt_set_gpio_mode(FLASH_STROBE_ENABLE, 0);
		//mt_set_gpio_dir(FLASH_STROBE_ENABLE, GPIO_DIR_OUT);
		//mt_set_gpio_out(FLASH_STROBE_ENABLE, GPIO_OUT_ONE);
		writeReg(LM3643_REG_FLASH_LED1, flashDuty[m_duty1]);
              	writeReg(LM3643_REG_ENABLE, temp|0x0D);
	}
	return 0;
}
int flashDisable_LM3643_1(void)
{
	int temp;
	PK_DBG(" flashDisable_LM3643_1 line=%d\n",__LINE__);
	temp=readReg(LM3643_REG_ENABLE);
       writeReg(LM3643_REG_ENABLE, temp&0xFE);
      return 0;
}


int setDuty_LM3643_1(int duty)
{

	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1=duty;
	PK_DBG(" setDuty_LM3643_1 line=%d,torch_flag=%d\n",__LINE__,torch_flag);
	return 0;
}



int flashEnable_LM3643_2(void)
{
	int temp;
	PK_DBG(" flashEnable_LM3643_2 line=%d,torch_flag=%d\n",__LINE__,torch_flag);
	temp=readReg(LM3643_REG_ENABLE);
	
	if(torch_flag)
	{
		m_duty2=0;
		writeReg(LM3643_REG_TORCH_LED2, torchDuty[m_duty2]);
		 writeReg(LM3643_REG_ENABLE, temp|0x0A);
	}
	else
	{
		writeReg(LM3643_REG_FLASH_LED2, flashDutylt[m_duty2]);
		writeReg(LM3643_REG_ENABLE, temp|0x0E);
	}
	return 0;
}
int flashDisable_LM3643_2(void)
{
	int temp;
	temp=readReg(LM3643_REG_ENABLE);
       writeReg(LM3643_REG_ENABLE, temp&0xFD);
	   PK_DBG(" flashDisable_LM3643_2 line=%d\n",__LINE__);
	return 0;
}


int setDuty_LM3643_2(int duty)
{

	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty2=duty;
	PK_DBG(" setDuty_LM3643_2 line=%d,torch_flag=%d\n",__LINE__,torch_flag);

	return 0;

}

static int LM3643_chip_init(struct LM3643_chip_data *chip)
{

	return 0;
}

static int LM3643_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct LM3643_chip_data *chip;
	struct LM3643_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3643_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "LM3643 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3643_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		PK_ERR("LM3643 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3643_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(LM3643_chip_init(chip)<0)
		goto err_chip_init;

	LM3643_i2c_client = client;
	PK_DBG("LM3643 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_ERR("LM3643 probe is failed \n");
	return -ENODEV;
}

static int LM3643_remove(struct i2c_client *client)
{
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define LM3643_NAME "leds-LM3643"
static const struct i2c_device_id LM3643_id[] = {
	{LM3643_NAME, 0},
	{}
};

static struct i2c_driver LM3643_i2c_driver = {
	.driver = {
		.name  = LM3643_NAME,
	},
	.probe	= LM3643_probe,
	.remove   = LM3643_remove,
	.id_table = LM3643_id,
};

struct LM3643_platform_data LM3643_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_LM3643={ I2C_BOARD_INFO(LM3643_NAME, 0x63), \
													.platform_data = &LM3643_pdata,};

static int __init LM3643_init(void)
{
	int ret;
	printk("LM3643_init\n");
	//i2c_register_board_info(2, &i2c_LM3643, 1);
	i2c_register_board_info(1, &i2c_LM3643, 1);
	ret=i2c_add_driver(&LM3643_i2c_driver);
	printk("LM3643_init ret=%d\n",ret);
	return ret;
}

static void __exit LM3643_exit(void)
{
	i2c_del_driver(&LM3643_i2c_driver);
}
module_init(LM3643_init);
module_exit(LM3643_exit);

int init_LM3643()
{
	int err;
	err =  writeReg(LM3643_REG_ENABLE,0x00);
      err =  writeReg(LM3643_REG_TIMING, 0x1F);
	return err;
}

int FL_Enable(void)
{
	//mt_set_gpio_mode(FLASH_ENABLE, 0);
	//mt_set_gpio_dir(FLASH_ENABLE, GPIO_DIR_OUT);
	//mt_set_gpio_out(FLASH_ENABLE, GPIO_OUT_ONE);	
	if(torch_flag)
	{
		
		mt_set_gpio_mode(FLASH_TORCH_ENABLE, 0);
		mt_set_gpio_dir(FLASH_TORCH_ENABLE, GPIO_DIR_OUT);
		mt_set_gpio_out(FLASH_TORCH_ENABLE, GPIO_OUT_ONE);				
	}
	else
	{
		mt_set_gpio_mode(FLASH_STROBE_ENABLE, 0);
		mt_set_gpio_dir(FLASH_STROBE_ENABLE, GPIO_DIR_OUT);
		mt_set_gpio_out(FLASH_STROBE_ENABLE, GPIO_OUT_ONE);
	}

     flashEnable_LM3643_1();
	PK_DBG(" FL_Enable line=%d\n",__LINE__);


    return 0;
}



int FL_Disable(void)
{
    flashDisable_LM3643_1();
	if(torch_flag)
	{
		mt_set_gpio_mode(FLASH_TORCH_ENABLE, 0);
		mt_set_gpio_dir(FLASH_TORCH_ENABLE, GPIO_DIR_OUT);
		mt_set_gpio_out(FLASH_TORCH_ENABLE, GPIO_OUT_ZERO);		
	}
	else
	{
		mt_set_gpio_mode(FLASH_STROBE_ENABLE, 0);
		mt_set_gpio_dir(FLASH_STROBE_ENABLE, GPIO_DIR_OUT);
		mt_set_gpio_out(FLASH_STROBE_ENABLE, GPIO_OUT_ZERO);		
	}
	
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
    setDuty_LM3643_1(duty);
    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}




int FL_Init(void)
{
	mt_set_gpio_mode(FLASH_ENABLE, 0);
	mt_set_gpio_dir(FLASH_ENABLE, GPIO_DIR_OUT);
	mt_set_gpio_out(FLASH_ENABLE, GPIO_OUT_ONE);	
	
    init_LM3643();

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
    PK_DBG(" FL_Uninit line=%d\n",__LINE__);
	FL_Disable();
	mt_set_gpio_mode(FLASH_ENABLE, 0);
	mt_set_gpio_dir(FLASH_ENABLE, GPIO_DIR_OUT);
	//mt_set_gpio_out(FLASH_ENABLE, GPIO_OUT_ZERO);	
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("LM3643 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			if(arg == 20000 || arg == 0)
				torch_flag = 1;
			else 
				torch_flag = 0;
			PK_DBG("FLASHLIGHT_TORCH_FLAG: %d\n",torch_flag);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
    	case FLASH_IOC_SET_REG_ADR:
    	    break;
    	case FLASH_IOC_SET_REG_VAL:
    	    break;
    	case FLASH_IOC_SET_REG:
    	    break;
    	case FLASH_IOC_GET_REG:
		i4RetValue=readReg(arg);
		PK_DBG("  arg=%d,i4RetValue=%d\n",arg,i4RetValue);
    	    break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


/***************                   *******************/


