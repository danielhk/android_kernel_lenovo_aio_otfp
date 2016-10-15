/*
 *  max17058_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include  "max17058_battery.h"
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include <linux/time.h>

#include <linux/regulator/consumer.h>


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

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#include <linux/xlog.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>


#include "cust_max17058_bat_para.h"
#include <mach/charging.h>

#include <linux/vmalloc.h>
#include <asm/unaligned.h>

#include <linux/dma-mapping.h>

#include <mach/battery_common.h>
#include <linux/wakelock.h>

static DEFINE_MUTEX(max17058_i2c_access);
static DEFINE_MUTEX(max17058_dl_mode);
struct wake_lock  max17058_download_fw_wakelock;

static  int mode_load_flag = 0;

#define max17058_VCELL_REG/*max17058_VCELL_MSB*/	0x02

#define max17058_SOC_REG/*max17058_SOC_MSB*/		0x04

#define max17058_MODE_REG/*max17058_MODE_MSB*/		0x06

#define max17058_VER_REG/*max17058_VER_MSB*/		0x08

#define max17058_RCOMP_REG/*max17058_RCOMP_MSB*/	0x0C



#define max17058_VRESET_REG                       0x18

#define max17058_STATUS_REG                      0x1A

#define max17058_CMD_REG/*max17058_CMD_MSB*/		0xFE

#define max17058_MODEL_ACCESS_REG			0x3E
#define max17058_MODEL_ACCESS_UNLOCK			0x4A57
#define max17058_MODEL_ACCESS_LOCK			0x0000

//#define max17058_POR_REG			0xfe

#define max17058_DELAY		10*HZ //1000->10*HZ
#define max17058_BATTERY_FULL	95

#define DEF_R_BAT		180



#define VERIFY_AND_FIX 1
#define LOAD_MODEL !(VERIFY_AND_FIX)

static void max17058_get_soc(struct i2c_client *client);
static int max17058_check_por(struct i2c_client *client);
void prepare_to_load_model(struct i2c_client *client);
void load_model(struct i2c_client *client);
bool verify_model_is_correct(struct i2c_client *client);
void cleanup_model_load(struct i2c_client *client);
//extern u8 get_temp(void);//get external temperature
static int max17058_write_reg(struct i2c_client *client, u8 reg, u16 value);
static int max17058_read_reg(struct i2c_client *client, u8 reg);
static u8 original_OCV_1=0, original_OCV_2=0;
static int bat_temp=0;

/***************************************
APIs for MTK upper layer invoking
***************************************/
unsigned char max17048_get_capacity(void);
void max17048_set_temperature(int tmp);
static struct i2c_client *new_client;
/**************************************/



struct max17058_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct delayed_work		hand_work;
	struct power_supply		*bms_psy;	
	struct power_supply		*charger_psy;		
	struct max17058_platform_data	*pdata;
	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery cal_soc*/
	int cal_soc;	
	int cal_soc_integer;	
	int cal_soc_decimal;	
	/* battery cal_soc pre*/
	int cal_soc_pre;	
	/* battery capacity */
	int soc;
	/*soc for ui display*/
	int ui_soc;	
	/* State Of Charge */	
	int status;
	int current_now;
	unsigned int version;
	struct regulator *vcc_i2c;
#ifdef CONFIG_MULTI_BATTERY
	int bat_id;
#endif	
};

static int max17058_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
	int ret;

	mutex_lock(&max17058_i2c_access);   
	ret = i2c_smbus_write_word_data(client, reg, swab16(value));

	if (ret < 0)
	{
		//dump_stack();
		dev_err(&client->dev, "%s: err %d.reg=0x%x, val=0x%x\n", __func__, ret, reg, value);
	}
	
       mutex_unlock(&max17058_i2c_access);  
	return ret;
}

#ifdef CONFIG_MULTI_BATTERY
static int max17058_get_bat_id(struct i2c_client *client)
{
	union power_supply_propval val;
	struct power_supply* psy;
	int ret;
	
	psy = power_supply_get_by_name("bms");

	if(!psy)
	{
		pr_err("%s get bms psy error!\n", __func__);
		return -1;
	}

	psy->get_property(psy, POWER_SUPPLY_PROP_SERIAL_NUMBER, &val);

	if(strcmp(val.strval, MAX17058_BAT_STR_ERR)==0)
		return -1;
		
	if(strcmp(val.strval, MAX17058_BAT_STR_2)==0)
		ret =  MAX17058_BAT_2;
	else
		ret = MAX17058_BAT_1;

	pr_err("%s, bat id = %s, ret %d\n", __func__, val.strval, ret);

	return ret;
}

static int max17058_set_module_data_as_bat_id(struct i2c_client *client)
{
	struct max17058_chip *chip = i2c_get_clientdata(client);
	
	if(chip->bat_id==MAX17058_BAT_1)
	{
		INI_RCOMP = chip->pdata->ini_rcomp;
		TempCoHot = chip->pdata->ini_temp_co_hot;
		TempCoCold = chip->pdata->ini_temp_co_cold;
		INI_SOCCHECKA = chip->pdata->ini_soccheck_a;
		INI_SOCCHECKB = chip->pdata->ini_soccheck_b;
		INI_OCVTEST = chip->pdata->ini_ocv_test;
		INI_BITS = chip->pdata->ini_bits;
	}else if(chip->bat_id==MAX17058_BAT_2)
	{
		INI_RCOMP = chip->pdata->ini_rcomp_2;
		TempCoHot = chip->pdata->ini_temp_co_hot_2;
		TempCoCold = chip->pdata->ini_temp_co_cold_2;
		INI_SOCCHECKA = chip->pdata->ini_soccheck_a_2;
		INI_SOCCHECKB = chip->pdata->ini_soccheck_b_2;
		INI_OCVTEST = chip->pdata->ini_ocv_test_2;
		INI_BITS = chip->pdata->ini_bits_2;	
	}else
	{
		INI_RCOMP = chip->pdata->ini_rcomp;
		TempCoHot = chip->pdata->ini_temp_co_hot;
		TempCoCold = chip->pdata->ini_temp_co_cold;
		INI_SOCCHECKA = chip->pdata->ini_soccheck_a;
		INI_SOCCHECKB = chip->pdata->ini_soccheck_b;
		INI_OCVTEST = chip->pdata->ini_ocv_test;
		INI_BITS = chip->pdata->ini_bits;		
	}

	return 0;
}
#endif

static int max17058_read_reg(struct i2c_client *client, u8 reg)
{


   char     cmd_buf[2]={0x00};
    int     readData = 0;
    int      ret=0;

    mutex_lock(&max17058_i2c_access);
    
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
    new_client->timing = 100;
	
    cmd_buf[0] = reg;

    ret = i2c_master_send(new_client, &cmd_buf[0], (2<<8 | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        new_client->ext_flag=0;

        mutex_unlock(&max17058_i2c_access);
        return 0;
    }
    
    readData = (kal_uint16) (((cmd_buf[1]<<8)&0xff00) | (cmd_buf[0]&0xff));
    //*returnData = readData;

    // new_client->addr = new_client->addr & I2C_MASK_FLAG;
    new_client->ext_flag=0;
    
    mutex_unlock(&max17058_i2c_access);    
	
    return readData;

}
//check POR
static int max17058_check_por(struct i2c_client *client)
{
	int val = 0;

	val = max17058_read_reg(client, max17058_STATUS_REG);
	printk("ww_Debug, check por val = %d(0x%x)\n", val, val);
	if(val>=0)
		val = val&0x01;
  
	return val;
}
//get chip version
static u16 max17058_get_version(struct i2c_client *client)
{
    u16 fg_version = 0;
    u16 chip_version = 0;

    fg_version = max17058_read_reg(client, max17058_VER_REG);
    chip_version = ((u16)(fg_version & 0xFF)<<8) + ((u16)(fg_version & 0xFF00)>>8);
    chip_version = swab16(fg_version);
  
    dev_err(&client->dev, "max17058 Fuel-Gauge Ver 0x%04x\n", chip_version);
 
    return chip_version;
}


static u16 max17058_get_rcomp(struct i2c_client *client)
{
    u16 tmp = 0;

    tmp = max17058_read_reg(client, max17058_RCOMP_REG);
    dev_err(&client->dev, "max17058_get_rcomp   0x%04x\n", tmp);
 
    return tmp;
}	

static void max17058_set_vreset(struct i2c_client *client, u8 vreset )
{
    u16 tmp = 0;

    tmp = max17058_read_reg(client, max17058_VRESET_REG);
    printk( "max17058_set_vreset1   0x%04x\n", tmp);	
    tmp = tmp&0xff00 |vreset;
    tmp = ((u16)(tmp & 0xFF)<<8) + ((u16)(tmp & 0xFF00)>>8);
    max17058_write_reg(client, max17058_VRESET_REG, tmp);	

}

static u16 max17058_get_vreset(struct i2c_client *client )
{
    u16 tmp = 0;

    tmp = max17058_read_reg(client, max17058_VRESET_REG);
    tmp = (u16)(tmp & 0xFF)/2;
    
     return tmp*40;
}
	
void prepare_to_load_model(struct i2c_client *client) {
    u16 msb;
    u16 check_times = 0;
    u8 unlock_test_OCV_1/*MSB*/, unlock_test_OCV_2/*LSB*/;
	  u16 chip_version;
	  
  
    /******************************************************************************
		Step 2.5.1 MAX17058/59 Only
		To ensure good RCOMP_Seg values in MAX17058/59, a POR signal has to be sent to
		MAX17058. The OCV must be saved before the POR signal is sent for best Fuel Gauge
		performance.for chip version 0x0011 only */
	chip_version = max17058_get_version(client);

    if(chip_version == 0x0011){
	    do {
		//max17058_write_reg(client, 0xFE, 0x5400);
		 max17058_write_reg(client, max17058_MODEL_ACCESS_REG, max17058_MODEL_ACCESS_UNLOCK);
		 if (check_times > 0)
		 {
                     msleep(100);
		 }
	       
	        msb = max17058_read_reg(client, 0x0E);
					
	        unlock_test_OCV_1 = (msb)&(0x00FF);//"big endian":low byte save MSB
	        unlock_test_OCV_2 = ((msb)&(0xFF00))>>8;
	
					if(check_times++ >= 4) {//avoid of while(1)
					    check_times = 0;
					    printk("max17058:time out3...");
					    break;
					}
	    }while ((unlock_test_OCV_1==0xFF)&&(unlock_test_OCV_2==0xFF));
  }
	//printk("max17058:test ocv1=0x%x, ocv2=0x%x,check_times=%d\n", unlock_test_OCV_1, unlock_test_OCV_2,check_times);
    //Step3: Write OCV
    //only for max17058/1/3/4, 
    //do nothing for MAX17058

    //Step4: Write RCOMP to its Maximum Value
    // only for max17058/1/3/4
    // max17058_write_reg(client,0x0C, 0xFF00);
    //do nothing for MAX17058
}


void load_model(struct i2c_client *client) {	

   /******************************************************************************
	Step 5. Write the Model
	Once the model is unlocked, the host software must write the 64 byte model
	to the device. The model is located between memory 0x40 and 0x7F.
	The model is available in the INI file provided with your performance
	report. See the end of this document for an explanation of the INI file.
	Note that the table registers are write-only and will always read
	0xFF. Step 9 will confirm the values were written correctly.
	*/
	int k=0;
	u16 value = 0;

#ifdef CONFIG_MULTI_BATTERY
	if(chip->bat_id==MAX17058_BAT_2)
		model_data =  chip->pdata->ini_model_data_2;
	else
		model_data = chip->pdata->ini_model_data;
#endif

	//Once the model is unlocked, the host software must write the 64 bytes model to the device
	for(k=0;k<0x40;k+=2)
	{
		value = (model_data[k]<<8)+model_data[k+1];
		//The model is located between memory 0x40 and 0x7F
		max17058_write_reg(client, 0x40+k, value);
	}
	
	//Write RCOMPSeg (for MAX17048/MAX17049 only)
	/*for(k=0;k<0x10;k++){
	    max17058_write_reg(client,0x80, 0x0080);
	}*/
}

bool verify_model_is_correct(struct i2c_client *client) {
    u8 SOC_1, SOC_2;
    u16 msb;
	
    msleep(200);//Delay at least 150ms(max17058/1/3/4 only)

    //Step 7. Write OCV:write(reg[0x0E], INI_OCVTest_High_Byte, INI_OCVTest_Low_Byte)
printk("!!!!max17058: INI_OCVTEST=0x%x\n", INI_OCVTEST);
    
    max17058_write_reg(client,0x0E, INI_OCVTEST);

    //Step 7.1 Disable Hibernate (MAX17048/49 only)
    //max17058_write_reg(client,0x0A,0x0);

    //Step 7.2. Lock Model Access (MAX17048/49/58/59 only)
    max17058_write_reg(client, max17058_MODEL_ACCESS_REG, max17058_MODEL_ACCESS_LOCK);

    //Step 8: Delay between 150ms and 600ms, delaying beyond 600ms could cause the verification to fail
    msleep(200);
 
    //Step 9. Read SOC register and compare to expected result
    msb = max17058_read_reg(client, max17058_SOC_REG);

    SOC_1 = (msb)&(0x00FF);//"big endian":low byte save MSB
    SOC_2 = ((msb)&(0xFF00))>>8;

printk("!!!!max17058: msb=0x%x, SOC_1=0x%x SOC_2=0x%x\n", msb, SOC_1, SOC_2);

    if(SOC_1 >= INI_SOCCHECKA && SOC_1 <= INI_SOCCHECKB) {
				pr_err("####max17058:model was loaded successfully####\n");
        return true;
    }
    else {		
				pr_err("!!!!max17058:model was NOT loaded successfully!!!! SOC_1=0x%x SOC_2=0x%x\n", SOC_1, SOC_2);
        return false; 
    }   
}

void cleanup_model_load(struct i2c_client *client) {	
    u16 original_ocv=0;
	int val;
	
    original_ocv = ((u16)((original_OCV_1)<<8)+(u16)original_OCV_2);
    //step9.1, Unlock Model Access (MAX17048/49/58/59 only): To write OCV, requires model access to be unlocked
    max17058_write_reg(client,max17058_MODEL_ACCESS_REG, max17058_MODEL_ACCESS_UNLOCK);

    //step 10 Restore CONFIG and OCV: write(reg[0x0C], INI_RCOMP, Your_Desired_Alert_Configuration
    max17058_write_reg(client,0x0C, ((INI_RCOMP<<8)|0x1c));//RCOMP0=127 , battery empty Alert threshold = 4% -> 0x1C
    max17058_write_reg(client,0x0E, original_ocv); 

    //step 10.1 Restore Hibernate (MAX17048/49 only)
    //do nothing for MAX17058

    //step 11 Lock Model Access
    max17058_write_reg(client,max17058_MODEL_ACCESS_REG, max17058_MODEL_ACCESS_LOCK);
    //step 12,//delay at least 150ms before reading SOC register
    msleep(200); 

	//clear por bit
	val = max17058_read_reg(client, max17058_STATUS_REG);
	val = val&0xfffe;
	max17058_write_reg(client, max17058_STATUS_REG, swab16(val));

	//check
	val = max17058_read_reg(client, max17058_STATUS_REG);
	pr_err("!!!!max17058!!!!val=0x%x\n", val);	
}

void max17058_unlock_model(struct i2c_client *client)
{
	  u16 msb;
    u16 check_times = 0;
    u8 unlock_test_OCV_1/*MSB*/, unlock_test_OCV_2/*LSB*/;

    do {
				
				max17058_write_reg(client, max17058_MODEL_ACCESS_REG, max17058_MODEL_ACCESS_UNLOCK);
        msleep(100);
        msb = max17058_read_reg(client, 0x0E);
				
        unlock_test_OCV_1 = (msb)&(0x00FF);//"big endian":low byte save MSB
        unlock_test_OCV_2 = ((msb)&(0xFF00))>>8;

				if(check_times++ >= 3) {//avoid of while(1)
				    check_times = 0;
				    printk("max17058:failded to unlock the model...");
				    break;
				}
    }while ((unlock_test_OCV_1==0xFF)&&(unlock_test_OCV_2==0xFF));

}

void max17058_get_init_ocv(struct i2c_client *client)
{
  	u16 check_times = 0;
    //u16 msb;  
	u16 OCV;  
	//u16 VCell1, VCell2, OCV;//, Desired_OCV;
  
	  //msleep(200);
	  /*
	  The OCV Register will be modified during the process of loading the custom
    model.  Read and store this value so that it can be written back to the 
    device after the model has been loaded. do it for only the first time after power up or chip reset
	  */
	  do {
    //Step1:unlock model access, enable access to OCV and table registers
    		max17058_write_reg(client, max17058_MODEL_ACCESS_REG, max17058_MODEL_ACCESS_UNLOCK);
    //Step2:Read OCV, verify Model Access Unlocked
         if (check_times > 0)
         {
              msleep(100);  
         }
    	      
         OCV = max17058_read_reg(client, 0x0E);//read OCV     
                 
        original_OCV_1 = (OCV)&(0x00FF);//"big endian":low byte save to MSB
        original_OCV_2 = ((OCV)&(0xFF00))>>8;

				if(check_times++ >= 4) {//avoid of while(1)
				    check_times = 0;
				    printk("max17058:failed to ulock the model...");
				    break;
				}
    }while ((original_OCV_1==0xFF)&&(original_OCV_2==0xFF));//verify Model Access Unlocked

	printk("max17058:init ocv1=0x%x, ocv2=0x%x, check_times=%d \n", original_OCV_1, original_OCV_2,check_times);
  
}


void handle_model(struct i2c_client *client, int load) {
	bool model_load_ok = false;
	int status;
	int check_times;
	static int load_or_verify = LOAD_MODEL;
	struct max17058_chip *chip = i2c_get_clientdata(client);

     wake_lock(&max17058_download_fw_wakelock);	
	 
    //check POR firstly
    if (mode_load_flag == 0)
    {
         // first time must load model again
         mode_load_flag = 1;
	pr_err("shone check load model\n");	 
     } else {
    	status = max17058_check_por(client);
         if(status==1) {
    	 pr_err("max17058 POR happens(0x%x),need to load model data\n", status);
         }	 
    	else
   	 {
    		pr_err("max17058 POR does not happen,don't need to load model data\n");
		wake_unlock(&max17058_download_fw_wakelock);		
  		return;
  	  }
    }
 
	
#ifdef CONFIG_MULTI_BATTERY
	{
		const int COUNT_TIME = 6;
		static int bat_id_check_cnt = COUNT_TIME;
		struct max17058_chip *chip = i2c_get_clientdata(client);	
		
		chip->bat_id = max17058_get_bat_id(client);
		if(chip->bat_id==-1)
		{
			bat_id_check_cnt--;
			if(bat_id_check_cnt>0)
			{
				pr_err("max17058 get bat id fail(%d)\n", bat_id_check_cnt);
				wake_unlock(&max17058_download_fw_wakelock);	
				return;
			}else
			{
				bat_id_check_cnt = COUNT_TIME;
				chip->bat_id = MAX17058_BAT_DEFAULT;
				pr_err("max17058 get bat id fail timeout, use default id\n");
			}
		}

		max17058_set_module_data_as_bat_id(client);
	}
#endif

    mutex_lock(&max17058_dl_mode);
    //read ocv and remeber it,later it will be written back   	
    max17058_get_init_ocv(client);

    	pr_err("max17058 load_or_verify=%d\n", load_or_verify);
    
	check_times = 0;
    do {
       
      if(load_or_verify == LOAD_MODEL) {		
      	// Steps 1-4		
	    	prepare_to_load_model(client);
		    // Step 5
		    load_model(client);
        }
        // Steps 6-9
        model_load_ok = verify_model_is_correct(client);
        if(!model_load_ok) {
            load_or_verify = LOAD_MODEL;
            max17058_unlock_model(client);
        }else
        	load_or_verify = !LOAD_MODEL;
		
	if(check_times++ >= 3) {
	    check_times = 0;
	    printk("max17058 handle model :time out1...");
	    break;
	}
    } while (!model_load_ok);

    // Steps 10-12
    cleanup_model_load(client);
    mutex_unlock(&max17058_dl_mode);
   
	if(model_load_ok==true)
	{
		max17058_get_soc(client);
	    	printk("max17058 module re-loaded, sync ui_soc(%d) to soc(%d)\n", chip->ui_soc, chip->soc);		
		//chip->ui_soc = chip->soc;
	}
       
      max17058_set_vreset(client,MAX17058_VRESER)	; 	
      wake_unlock(&max17058_download_fw_wakelock);	 	  
}

static int max17058_get_charger_state(struct i2c_client *client)
{
	struct max17058_chip *chip = i2c_get_clientdata(client);
	union power_supply_propval val;
	struct power_supply *phy = 0;

	if (!chip->bms_psy)
		chip->bms_psy = power_supply_get_by_name("battery");

	phy = chip->bms_psy;


	if(!phy)
	{
		pr_err("%s get battery power_supply error!\n", __func__);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;//defult 25C
	}
	
	phy->get_property(phy, POWER_SUPPLY_PROP_STATUS, &val);

	pr_err("%s, state = %d\n", __func__, val.intval);
	
	return val.intval;			
}


static void max17058_reset(struct i2c_client *client)
{
	max17058_write_reg(client, max17058_CMD_REG, 0x5400);//
}

static void max17058_get_vcell(struct i2c_client *client)
{
	struct max17058_chip *chip = i2c_get_clientdata(client);
	u16 fg_vcell = 0;
	u32 vcell_mV = 0;

	fg_vcell = max17058_read_reg(client, max17058_VCELL_REG);
	vcell_mV = (u32)(((fg_vcell & 0xFF)<<8) + ((fg_vcell & 0xFF00)>>8))*5/64;//78125uV/(1000*1000) = 5/64 mV/cell
	chip->vcell = vcell_mV;

	pr_err("max17058:chip->vcell = \t%d\t mV\n", chip->vcell);

}

static int  max17058_get_ocv(struct i2c_client *client)
{
	u16 ocv = 0;

	max17058_write_reg(client, max17058_MODEL_ACCESS_REG, max17058_MODEL_ACCESS_UNLOCK);

       ocv = max17058_read_reg(client, 0x0E);//read OCV     
	max17058_write_reg(client, max17058_MODEL_ACCESS_REG, max17058_MODEL_ACCESS_LOCK);       
	
  	ocv = (u32)(((ocv & 0xFF)<<8) + ((ocv & 0xFF00)>>8))*5/64;//78125uV/(1000*1000) = 5/64 mV/cell

	return ocv;
}

static void max17058_get_soc(struct i2c_client *client)
{
	struct max17058_chip *chip = i2c_get_clientdata(client);
	u16 fg_soc = 0;
	u16 tmp = 0;
	
	fg_soc = max17058_read_reg(client, max17058_SOC_REG);

	tmp = ((u16)(fg_soc & 0xFF)<<8) + ((u16)(fg_soc & 0xFF00)>>8);

	if(INI_BITS == 19) {
	    fg_soc = tmp/512;
	    tmp =  tmp%512;
	    
	}else if(INI_BITS == 18){
	    fg_soc = tmp/256;
	     tmp = tmp%256;	
	}

	if (tmp > 100)
		chip->soc = ++fg_soc;
	else
              chip->soc = fg_soc;

	
	if(chip->soc < 0)	chip->soc = 0;
	
}

static int max17058_cal_cur(struct i2c_client *client)
{
	struct max17058_chip *chip = i2c_get_clientdata(client);
	static struct timespec last_cal_time;
	const int cal_timer = 1;
	static int cal_cnt = cal_timer ;
	static int cal_soc_bak = -1;
	struct timespec now_time;
	int delta_time = 0;
	int de_soc = 0;
	int tmp;
	int state;
	
	if(cal_cnt>0)
	{
		cal_cnt--;
		return -1;
	}

	cal_cnt = cal_timer;

	getrawmonotonic(&now_time);

	delta_time = now_time.tv_sec - last_cal_time.tv_sec;
	if(delta_time==0)
	{
		pr_err("%s delta_time is 0 continue \n", __func__);
		return -2;
	}
	
	tmp = max17058_read_reg(client, max17058_SOC_REG);

	chip->cal_soc = ((u16)(tmp & 0xFF)<<8) + ((u16)(tmp & 0xFF00)>>8);
	
	if(INI_BITS == 19) {
	    chip->cal_soc_integer = chip->cal_soc/512;
	    chip->cal_soc_decimal	= chip->cal_soc & 0x1FF;
	}else if(INI_BITS == 18){
	    chip->cal_soc_integer = chip->cal_soc/256;
	     chip->cal_soc_decimal	= chip->cal_soc & 0xFF;	
	}
	
	if(chip->cal_soc_pre==-1)
		chip->cal_soc_pre = chip->cal_soc;

	if(cal_soc_bak==-1)
		cal_soc_bak = chip->cal_soc_pre;
	
	de_soc = chip->cal_soc- chip->cal_soc_pre;

	if(de_soc==0)
	{
		if((cal_soc_bak!=-1)&&(chip->cal_soc>cal_soc_bak))
		{
			state = max17058_get_charger_state(chip->client);	
			
			if(state==POWER_SUPPLY_STATUS_CHARGING)
			{
				de_soc = chip->cal_soc - cal_soc_bak;
			}
		}

		if(chip->cal_soc<cal_soc_bak)
			cal_soc_bak = chip->cal_soc;
	}else
	{
		cal_soc_bak = chip->cal_soc_pre;
		chip->cal_soc_pre = chip->cal_soc;
	}

	if(INI_BITS == 19) {
	    chip->current_now = (100*3600*de_soc/512/100/delta_time)*BATTERY_CAPACITY/100;// 2 is  a compensite value
	}else if(INI_BITS == 18){
	    chip->current_now = (100*3600*de_soc/256/100/delta_time)*BATTERY_CAPACITY/100;
	}	

	pr_debug("%s %d, %d, %d, %d, %d, %d\n", __func__, delta_time,  de_soc, chip->cal_soc,  chip->cal_soc_pre, cal_soc_bak, chip->current_now);
	
	last_cal_time = now_time;

	return 0;	
}


//=======================================end=========================================


static void max17058_handle_work(struct work_struct *work)
{
	struct max17058_chip *chip;
		
	chip = container_of(work, struct max17058_chip, hand_work.work);

	handle_model(chip->client, LOAD_MODEL);

	schedule_delayed_work(&chip->hand_work,HZ*1200);
	
}

/**********************************
MTK API:set temperature
***********************************/
void max17048_set_temperature(int tmp)
{
	int NewRCOMP;
	u16 cfg=0;
	
	bat_temp = tmp;
  if(new_client == NULL)
		return;

	mutex_lock(&max17058_dl_mode);
	if(tmp > 20) 
	{
		NewRCOMP = INI_RCOMP + ((tmp - 20) * TempCoHot)/INI_RCOMP_FACTOR;
	}else if(tmp <20) 
	{
		NewRCOMP = INI_RCOMP +  ((tmp - 20) * TempCoCold)/INI_RCOMP_FACTOR;
	}else 
	{
		NewRCOMP = INI_RCOMP;
	}
	
	if(NewRCOMP > 0xFF)
	{
		NewRCOMP = 0xFF;
	}else if(NewRCOMP <0) 
		NewRCOMP = 0;
	
	cfg=(NewRCOMP<<8)|0x1c;//soc alert:4%   

	printk("%s temp=%d,cfg=0x%x \n", __func__,tmp,cfg);
	max17058_write_reg(new_client, 0x0c, cfg);
       mutex_unlock(&max17058_dl_mode);
	
	msleep(150);
	
}

static  void printk_max17058_info(void)
{
	struct max17058_chip *chip = i2c_get_clientdata(new_client);
       unsigned int  alert_reg;
	unsigned int  rcomp_reg;   
	u16 tmp;
       int ocv;
	u16 vreset;   

       max17058_get_vcell(chip->client);
	max17058_cal_cur(chip->client);

#ifdef PRINT_OCV_DEBUG
       ocv = max17058_get_ocv(chip->client); 
#else
        ocv = 0;
#endif
	   
	tmp = max17058_get_rcomp(chip->client);
	rcomp_reg = (u16)(tmp & 0xFF);
       alert_reg = (u16)(tmp & 0xFF00)>>8;
       vreset = max17058_get_vreset(chip->client);

	   
      
	pr_err("%s: %d,   %d,    %d,   %d,   %d,   %d.%d,   0x%x,   0x%x,   %d,   0x%x  \n",
__func__, chip->vcell,  ocv,chip->current_now, bat_temp,chip->ui_soc, chip->cal_soc_integer,chip->cal_soc_decimal,rcomp_reg,alert_reg,vreset,chip->version);
}

/**********************************
MTK API: get current capacity
***********************************/
unsigned char max17058_get_capacity(void)
{
       int temp;
	struct max17058_chip *chip = i2c_get_clientdata(new_client);

       mutex_lock(&max17058_dl_mode);
	max17058_get_soc(chip->client);
	printk_max17058_info();
       mutex_unlock(&max17058_dl_mode);
#ifdef BATTERY_SOC_ADJUST_SUPPORT	
   #if 0
       if(BMT_status.charger_exist==KAL_TRUE)
       	{
            chip->ui_soc = chip->soc*100/BATTERY_SOC_ADJUST;
       	}		
	else
   #endif		
	{
           temp = chip->soc*10000/BATTERY_SOC_ADJUST;
           chip->ui_soc = temp/100;
           if(temp%100 >= 50)
           {
               chip->ui_soc++;
           }	   	
	}
#else
        chip->ui_soc = chip->soc;
#endif

	if(chip->ui_soc>100)
	{
	    chip->ui_soc = 100;
	}
	return chip->ui_soc;
}

static int max17058_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);			
	struct max17058_chip *chip;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	
	chip->client = client;
	chip->pdata = dev_get_platdata(&client->dev);

	i2c_set_clientdata(client, chip);

	chip->cal_soc_pre = -1;

        new_client = client;
		
        wake_lock_init(&max17058_download_fw_wakelock, WAKE_LOCK_SUSPEND, "max17058 download fw  wakelock");
		
	chip->version = max17058_get_version(client);
  
	max17058_write_reg(client, 0x0c, 0x7f1C);
	handle_model(client,LOAD_MODEL);

	INIT_DEFERRABLE_WORK(&chip->hand_work, max17058_handle_work);
	schedule_delayed_work(&chip->hand_work,0);
	
	return 0;
}

static int max17058_remove(struct i2c_client *client)
{
	struct max17058_chip *chip = i2c_get_clientdata(client);

	//cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM
#if 0
static int max17058_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17058_chip *chip = i2c_get_clientdata(client);
	return 0;
}

static int max17058_resume(struct i2c_client *client)
{
	struct max17058_chip *chip = i2c_get_clientdata(client);

	return 0;
}
#endif
#else

#define max17058_suspend NULL
#define max17058_resume NULL

#endif /* CONFIG_PM */

static struct of_device_id max17058_match_table[] = {
	{ .compatible = "max,max17058-fg"},
	{ },
};

static const struct i2c_device_id max17058_id[] = {
	{ "max17058", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17058_id);

static struct i2c_driver max17058_i2c_driver = {
	.driver	= {
		.name	= "max17058",
		.of_match_table	= max17058_match_table,
	},
	.probe		= max17058_probe,
	.remove		= max17058_remove,
	.suspend	= NULL,//max17058_suspend,
	.resume		= NULL,// max17058_resume,
	.id_table	= max17058_id,
};

/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/
static ssize_t show_max17058_id(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct max17058_chip *chip = i2c_get_clientdata(new_client);

    printk("show_max17058_access: %x \n",chip->version);		
    if (chip->version == 0x0012)
        return snprintf(buf, 10, "%s\n", "MAX17058");
    else
	 return snprintf(buf, 10, "%s\n", "ERROR");	
}
static ssize_t store_max17058_id(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{  
    return size;
}
static DEVICE_ATTR(max17058_id, 0664, show_max17058_id, store_max17058_id); //664

static int max17058_user_space_probe(struct platform_device *dev)    
{    
    int ret_device_file = 0;

    printk("******** max17058_user_space_probe!! ********\n" );
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_max17058_id);
    
    return 0;
}

struct platform_device max17058_user_space_device = {
    .name   = "max17058-user",
    .id     = -1,
};

static struct platform_driver max17058_user_space_driver = {
    .probe      = max17058_user_space_probe,
    .driver     = {
        .name = "max17058-user",
    },
};
//static struct i2c_board_info __initdata i2c_max17058 = { I2C_BOARD_INFO("MAX77819_FG", 0x6C >> 1)};
//static struct i2c_board_info __initdata i2c_max17058 = { I2C_BOARD_INFO("max17058", (0x56))};
static struct i2c_board_info __initdata i2c_max17058 = { I2C_BOARD_INFO("max17058", 0x6C >> 1)};
static int __init max17058_init(void)
{    
    int ret=0;
    
    printk("[max17058_init] init start\n");
    
    i2c_register_board_info(0, &i2c_max17058, 1);

    if(i2c_add_driver(&max17058_i2c_driver)!=0)
    {
        printk("[max17058_init] failed to register max17058 i2c driver.\n");
    }
    else
    {
        printk("[max17058_init] Success to register max17058 i2c driver.\n");
    }

   // max17058  user space access interface
    ret = platform_device_register(&max17058_user_space_device);
    if (ret) {
        printk("****[max17058_init] Unable to device register(%d)\n", ret);
        return ret;
    }    
    ret = platform_driver_register(&max17058_user_space_driver);
    if (ret) {
        printk("****[max17058_init] Unable to register driver (%d)\n", ret);
        return ret;
    }
    return 0;        
}

static void __exit max17058_exit(void)
{
    i2c_del_driver(&max17058_i2c_driver);
}

module_init(max17058_init);
module_exit(max17058_exit);


MODULE_AUTHOR("maxim integrated");
MODULE_DESCRIPTION("max17058 Fuel Gauge");
MODULE_LICENSE("GPL");
