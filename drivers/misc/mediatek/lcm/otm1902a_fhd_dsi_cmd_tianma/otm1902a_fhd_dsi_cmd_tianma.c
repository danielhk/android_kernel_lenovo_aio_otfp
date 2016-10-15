/* BEGIN PN:DTS2013053103858 , Added by d00238048, 2013.05.31*/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h>
	#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
    //#include <linux/delay.h>
    #include <mach/mt_gpio.h>
#endif
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif
//Lenovo-sw wuwl10 add 20150128 for esd recover backlight
#ifndef BUILD_LK
extern unsigned int cmd_esd_last_backlight_level ;
#endif
//extern int isAAL;
//#define GPIO_AAL_ID		 (GPIO174 | 0x80000000)
static unsigned int orise_cabc = 0x2;//default cabc open
static unsigned int orise_sre = 0x0;//default sre close

//add for sre issue wangyq13 20150727
#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
unsigned int sre_init = 0x0; //0 stand for first start,1 stand for system has started already
unsigned int sre_suspend = 0x0; //0 stand for not suspend,1 stand for system just suspend
#endif

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmd_by_cmdq(handle,cmd,count,ppara,force_update)    lcm_util.dsi_set_cmdq_V22(handle,cmd,count,ppara,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


//static unsigned char lcd_id_pins_value = 0xFF;
static const unsigned char LCD_MODULE_ID = 0x01; //  haobing modified 2013.07.11
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									1
#define FRAME_WIDTH  										(1080)
#define FRAME_HEIGHT 										(1920)
#define PHYSICAL_WIDTH										(64)
#define PHYSICAL_HEIGHT										(112)

#define REGFLAG_DELAY             								0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/***************************************************************************** 
 * Define
 *****************************************************************************/
#ifndef FPGA_EARLY_PORTING
#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL//for I2C channel 0
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E
/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata tps65132_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
static struct i2c_client *tps65132_i2c_client = NULL;


/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

 struct tps65132_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id tps65132_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

static struct i2c_driver tps65132_iic_driver = {
	.id_table	= tps65132_id,
	.probe		= tps65132_probe,
	.remove		= tps65132_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tps65132",
	},
 
};


/***************************************************************************** 
 * Function
 *****************************************************************************/ 

static  int vpos_set(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{	
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2]={0};	
	write_data[0]= 0x00;
	write_data[1] = 0x0A;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
		printk("tps65132 vpos_set data fail !!\n");	
	else 
		printk("tps65132 vpos_set data success !!\n");
	write_data[0]= 0x01;
	write_data[1] = 0x0A;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
		printk("tps65132 vpos_set data fail !!\n");	
	else 
		printk("tps65132 vpos_set data success !!\n");
	return ret ;
}

static  int vpos_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char data = 0x0;
    ret=i2c_master_send(client, &data, 1);
	if(ret<0)
		printk("tps65132 write data fail !!\n");	
	else
		printk("tps65132 write data success !!\n");	
	ret=i2c_master_recv(client, &data, 1);
	if(ret<0)
		{
		    printk("tps65132 read data fail !!\n");
	            return snprintf(buf, 15, "%s\n", "ERROR");
		}
	else
		{
		    printk("tps65132 read data success,data =%d  !!\n",data);
		    return snprintf(buf, 15, "%s\n", "TPS65132L0");
		}
}

static DEVICE_ATTR(vpos, 0666, vpos_show, vpos_set);

static struct attribute *sysfs_attributes[] = {
	&dev_attr_vpos.attr,
	NULL
};

static struct attribute_group sysfs_attribute_group = {
	.attrs = sysfs_attributes
};

static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
        int error;
	tps65132_i2c_client  = client;	
	error = sysfs_create_group(&tps65132_i2c_client->dev.kobj,
					&sysfs_attribute_group);
	if (error)
		printk("tps65132 sysfs_create_group fail !!\n");

	return 0;      
}


static int tps65132_remove(struct i2c_client *client)
{  	
  tps65132_i2c_client = NULL;
   i2c_unregister_device(client);
  return 0;
}


static int __init tps65132_iic_init(void)
{
  // int error;
   i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
   i2c_add_driver(&tps65132_iic_driver);
   /*error = sysfs_create_group(&tps65132_i2c_client->dev.kobj,
					&sysfs_attribute_group);
	if (error)
		printk("tps65132 sysfs_create_group fail !!\n");	*/

   return 0;
}

static void __exit tps65132_iic_exit(void)
{
  i2c_del_driver(&tps65132_iic_driver);  
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL"); 
#endif
#endif


static struct LCM_setting_table lcm_cabc_level_setting[] = {
{0x55, 1, {0x00}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
static struct LCM_setting_table lcm_sre_level_setting[] = {
{0x55, 1, {0x40}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_initialization_setting[] = {
         //enable orise command
         {0x00,1,{0x00}},
         {0xFF,3,{0x19,0x02,0x01}},
         {0x00,1,{0x80}},
         {0xFF,2,{0x19,0x02}},
		 
         //select LCD work mode. 00(Cmd mode)/13(Video mode)
         {0x00,1,{0x00}},
         {0x1C,1,{0x00}},

         {0x00,1,{0x8b}},
         {0xb0,1,{0x00}},
		 
         //turn off SRAM power when reset keeps low or Ultra low power mode in sleep in
         {0x00,1,{0x90}},
         {0xB4,1,{0x09}},

		 //OSC freq 73.96MHZ,
         {0x00,1,{0x80}},
         {0xC1,2,{0x11,0x11}},

         {0x00,1,{0x90}},
         {0xC1,2,{0x44,0x00}},

		 //RTN Setting
         {0x00,1,{0x80}},
         {0xC0,14,{0x00,0x77,0x00,0x42,0x43,0x00,0x77,0x42,0x43,0x00,0x77,0x00,0x42,0x43}},
 
         {0x00,1,{0x80}},
         {0xA5,3,{0x0C,0x00,0x0F}},

		 //LTPS CKHSEQ
         {0x00,1,{0xF8}},
         {0xC2,6,{0x03,0x00,0x00,0x0C,0x01,0x01}},

         {0x00,1,{0xA0}},
         { 0xC0,13,{0x00,0x00,0x00,0x15,0x02,0x15,0x05,0x00,0x00,0xFF,0xFF,0x00,0x00}},

         //column  inversion		 
         {0x00,1,{0xB3}},
         {0xC0,1,{0xCC}},
         //Up todown scan
         {0x00,1,{0xB4}},
         {0xC0,1,{0x40}},
		 
         //specify start point of fp_dummy active line
         {0x00,1,{0xEA}},
         {0xC2,2,{0x22,0x00}},
		 
         //LTPS PCG&SELR/G/B setting for video mode 
         {0x00,1,{0xD0}},
         {0xC0,13,{0x00,0x00,0x00,0x12,0x01,0x1A,0x05,0x00,0x00,0xFF,0xFF,0x00,0x00}},
        //STV setting
         {0x00,1,{0x80}},
         {0xC2,8,{0x82,0x01,0x00,0x0E,0x83,0x00,0x1D,0x00}},


		 
        //CKV setting
         {0x00,1,{0x90}},
         {0xC2,15,{0x82,0x80,0x01,0x0D,0x00,0x81,0x00,0x01,0x0D,0x00,0x80,0x00,0x01,0x0D,0x00}},

         {0x00,1,{0xA0}},
         {0xC2,15,{0x01,0x01,0x01,0x0D,0x00,0x82,0x00,0x00,0x00,0x8E,0x81,0x02,0x00,0x00,0x8E}},

         {0x00,1,{0xB0}},
         {0xC2,10,{0x82,0x03,0x01,0x00,0x8E,0x82,0x03,0x01,0x00,0x8E}},

         {0x00,1,{0xC0}},
         {0xC1,13,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x11,0x11,0x11,0x11}},

         {0x00,1,{0xC0}},
         {0xC2,10,{0x00,0x01,0x03,0x00,0x00,0x01,0x02,0x03,0x00,0x00}},

         {0x00,1,{0xD0}},
         {0xC2,10,{0x82,0x81,0x03,0x00,0x00,0x81,0x80,0x03,0x00,0x00}},

         {0x00,1,{0xE0}},
         {0xC2,10,{0x81,0x5F,0x01,0x00,0x1F,0x81,0x5F,0x01,0x00,0x1F}},

         {0x00,1,{0xEC}},
         {0xC2,2,{0x10,0x00}},

         {0x00,1,{0xE0}},
         {0xC1,12,{0x80,0x74,0x01,0x01,0x09,0x81,0x74,0x01,0x01,0x09,0x01,0x01}},

         {0x00,1,{0xF0}},
         {0xC2,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

         {0x00,1,{0x80}},
         {0xCB,13,{0xF0,0x0F,0x00,0xFF,0x00,0x33,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

         {0x00,1,{0x90}},
         {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

         {0x00,1,{0xA0}},
         {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x00}},

         {0x00,1,{0xB0}},
         {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x50,0x40,0x00,0x00}},

         {0x00,1,{0xEB}},
         {0xCC,2,{0xF0,0x00}},

         {0x00,1,{0xC0}},
         {0xCB,15,{0x00,0x00,0xD4,0xD4,0xD4,0xD4,0x00,0x00,0x00,0x00,0x00,0x00,0xD4,0xD4,0xD4}},

         {0x00,1,{0xD0}},
         {0xCB,15,{0xD4,0x28,0x28,0x28,0x28,0xD4,0x14,0xE8,0x28,0x37,0x04,0x04,0x04,0x04,0x04}},

         {0x00,1,{0xE0}},
         {0xCB,15,{0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x05,0x05,0x05,0x04,0x00}},

         {0x00,1,{0xED}},
         {0xCC,2,{0xFF,0x00}},

         {0x00,1,{0xF0}},
         {0xCB,12,{0xF0,0x0F,0x00,0xFF,0x00,0xF3,0x03,0x00,0x00,0x00,0x00,0x00}},

         {0x00,1,{0xE0}},
         {0xCC,6,{0x00,0x00,0x00,0x00,0x00,0x40}},

         {0x00,1,{0x80}},
         {0xCC,12,{0x2E,0x0F,0x03,0x04,0x05,0x06,0x07,0x08,0x05,0x0A,0x10,0x2F}},

         {0x00,1,{0xB0}},
         {0xCC,12,{0x2F,0x10,0x06,0x05,0x04,0x03,0x07,0x08,0x09,0x0A,0x0F,0x2E}},

         {0x00,1,{0x80}},
         {0xCD,15,{0x13,0x14,0x15,0x12,0x13,0x14,0x15,0x12,0x01,0x2D,0x2D,0x02,0x17,0x26,0x18}},
         
		 //CGOUT_SEL
         {0x00,1,{0x90}},
         {0xCD,15,{0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x27,0x28}},

         {0x00,1,{0xE6}},
         {0xCC,5,{0x29,0x2A,0x11,0x11,0x11}},

         {0x00,1,{0xA0}},
         {0xCD,15,{0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x03}},

         {0x00,1,{0xB0}},
         {0xCD,15,{0x05,0x2F,0x18,0x16,0x17,0x15,0x2F,0x12,0x11,0x14,0x0E,0x0D,0x10,0x2F,0x2F}},

         {0x00,1,{0xC0}},
         {0xCD,14,{0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F}},

         {0x00,1,{0xD0}},
         {0xCD,15,{0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x04}},
         //
         {0x00,1,{0xE0}},
         {0xCD,15,{0x06,0x2F,0x18,0x16,0x17,0x15,0x2F,0x12,0x11,0x14,0x0E,0x0D,0x10,0x2F,0x2F}},

         {0x00,1,{0xF0}},
         {0xCD,14,{0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F,0x2F}},

		 //STV setting for video mode
         {0x00,1,{0x80}},
         {0xC3,8,{0x82,0x01,0x00,0x0E,0x83,0x00,0x1D,0x00}},

		 //ckv setting for video mode
         {0x00,1,{0x90}},
         {0xC3,15,{0x82,0x80,0x01,0x0D,0x00,0x81,0x00,0x01,0x0D,0x00,0x80,0x00,0x01,0x0D,0x00}},

		 //ckv setting for video mode
         {0x00,1,{0xA0}},
         {0xC3,15,{0x01,0x01,0x01,0x0D,0x00,0x82,0x00,0x00,0x00,0x8E,0x81,0x02,0x00,0x00,0x8E}},

         {0x00,1,{0xB0}},
         {0xC3,10,{0x82,0x03,0x01,0x00,0x8E,0x82,0x03,0x01,0x00,0x8E}},

         {0x00,1,{0xD0}},
         {0xC1,13,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x11,0x11,0x11,0x11}},

         {0x00,1,{0xC0}},
         {0xC3,10,{0x00,0x01,0x03,0x00,0x00,0x01,0x02,0x03,0x00,0x00}},

         {0x00,1,{0xD0}},
         {0xC3,10,{0x82,0x81,0x03,0x00,0x00,0x81,0x80,0x03,0x00,0x00}},

         {0x00,1,{0xE0}},
         {0xC3,10,{0x81,0x5F,0x01,0x00,0x1A,0x81,0x5F,0x01,0x00,0x1A}},

         {0x00,1,{0xEC}},
         {0xC3,2,{0x10,0x00}},

         {0x00,1,{0xF0}},
         {0xC1,10,{0x80,0x74,0x01,0x01,0x09,0x81,0x74,0x01,0x01,0x09}},

         //source 在 V-blanking推最小压差
         {0x00,1,{0x80}},
         {0xC4,5,{0x18,0x2C,0x00,0x20,0x00}},

		 //power control for DC2DC circuit
         {0x00,1,{0xA0}},
         {0xC4,9,{0x30,0x06,0x04,0x3A,0x30,0x26,0x84,0x3A,0x01}},

		 //power control setting  
         {0x00,1,{0x90}},
         {0xC5,14,{0x92,0x16,0xA0,0x14,0xA0,0x1E,0x80,0x14,0x80,0x32,0x44,0x44,0x40,0x88}},
		 
         {0x00,1,{0xA0}},
         {0xC5,14,{0x92,0x16,0xA0,0x14,0xA0,0x1E,0x80,0x14,0x80,0x32,0x44,0x44,0x40,0x88}},

         {0x00,1,{0xC0}},
         {0xC5,3,{0x12,0xFA,0x00}},

         {0x00,1,{0x00}},
         {0xC6,2,{0x00,0x00}},

        //PWM		 
         {0x00,1,{0xB0}},
         {0xCA,4,{0x02,0x20,0x5F,0x40}}, //PWM 22.8KHz,256 step

        //GVDD/NGVDD setting
         {0x00,1,{0x00}},
         {0xD8,2,{0x28,0x28}},



        //////////FIX_SETTING

         {0x00,1,{0x83}},
         {0xF3,1,{0xC8}},

         {0x00,1,{0x90}},
         {0xC4,1,{0x00}},

         {0x00,1,{0xC4}},
         {0xC5,1,{0x04}},

         {0x00,1,{0xB6}},
         {0xC0,1,{0x7F}},

         {0x00,1,{0xA0}},
         {0xC1,1,{0x00}},

         {0x00,1,{0xA1}},
         {0xC1,1,{0xED}},

         {0x00,1,{0xA2}},
         {0xC1,1,{0x00}},

         {0x00,1,{0xA3}},
         {0xC1,1,{0x00}},

         {0x00,1,{0xA4}},
         {0xC1,1,{0x00}},

         {0x00,1,{0xA5}},
         {0xC1,1,{0x00}},

         {0x00,1,{0xA6}},
         {0xC1,1,{0x00}},

         {0x00,1,{0xA7}},
         {0xC1,1,{0x02}},

         {0x00,1,{0x91}},
         {0xE9,1,{0x00}},
         //AVDD SHORT TO AVDDR,AVEE SHORT TO AVEER
         {0x00,1,{0xBA}},
         {0xF5,2,{0x80,0x80}},

         {0x00,1,{0xA5}},
         {0xB3,1,{0x80}},

         //source Pre-charge enable
         {0x00,1,{0xF8}},
         {0xC2,1,{0x82}},


         //
         {0x00,1,{0xFB}},
         {0xC2,1,{0x00}},
		 
		 
		 //default gamma 
		 {0x00,1,{0x00}},
         {0xE1,24,{0x05,0x12,0x1b,0x26,0x30,0x37,0x43,0x55,0x61,0x73,0x7f,0x89,0x6f,0x68,0x60,0x52,0x40,0x30,0x25,0x20,0x17,0x0c,0x05,0x03}},

         {0x00,1,{0x00}},
         {0xE2,24,{0x05,0x12,0x1b,0x26,0x30,0x37,0x43,0x55,0x61,0x73,0x7f,0x89,0x6f,0x68,0x60,0x52,0x40,0x30,0x25,0x20,0x17,0x0c,0x05,0x03}},
       
        {0x00,1,{0x00}},
        {0xE3,24,{0x05,0x12,0x1b,0x26,0x30,0x37,0x43,0x55,0x61,0x73,0x7f,0x89,0x6f,0x68,0x60,0x52,0x40,0x30,0x25,0x20,0x17,0x0c,0x05,0x03}},
 
       {0x00,1,{0x00}},
       {0xE4,24,{0x05,0x12,0x1b,0x26,0x30,0x37,0x43,0x55,0x61,0x73,0x7f,0x89,0x6f,0x68,0x60,0x52,0x40,0x30,0x25,0x20,0x17,0x0c,0x05,0x03}},
       
 	   {0x00,1,{0x00}},
 	   {0xE5,24,{0x05,0x12,0x1b,0x26,0x30,0x37,0x43,0x55,0x61,0x73,0x7f,0x89,0x6f,0x68,0x60,0x52,0x40,0x30,0x25,0x20,0x17,0x0c,0x05,0x03}},

       {0x00,1,{0x00}},
       {0xE6,24,{0x05,0x12,0x1b,0x26,0x30,0x37,0x43,0x55,0x61,0x73,0x7f,0x89,0x6f,0x68,0x60,0x52,0x40,0x30,0x25,0x20,0x17,0x0c,0x05,0x03}},


        //SRE CE
        {0x00,1,{0x00}},
	{0x59,1,{0x03}},

        /*{0x00,1,{0xA0}},
        {0xD6,12,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
        {0x00,1,{0xB0}},
        {0xD6,12,{0x00,0x00,0x7f,0xfd,0xfd,0xfd,0xfd,0xfd,0xfd,0xfd,0xfd,0xfd}},
        {0x00,1,{0xC0}},
        {0xD6,12,{0x00,0x00,0xa9,0xa9,0xa9,0xa9,0xa9,0xa9,0xa9,0xa9,0xa9,0xa9}},
        {0x00,1,{0xD0}},
        {0xD6,12,{0x00,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54}},*/
        //AIE gamma
	/*{0x00,1,{0x01}},
        {0xC6,1,{0x10}},//AIE G 1
        {0x00,1,{0x00}},
        {0xC8,18,{0x80,0x37,0x25,0x25,0x24,0x14,0x24,0x44,0x34,0x43,0x44,0x44,0x45,0x54,0x54,0x45,0x54,0x44}},

	{0x00,1,{0x01}},
        {0xC6,1,{0x11}},//AIE G 2
        {0x00,1,{0x00}},
        {0xC8,18,{0x80,0x37,0x25,0x25,0x24,0x14,0x24,0x44,0x34,0x43,0x44,0x44,0x45,0x54,0x54,0x45,0x54,0x44}},*/

	{0x00,1,{0x01}},
        {0xC6,1,{0x10}},//AIE G 3 0x12
        {0x00,1,{0x00}},
        {0xC8,18,{0xff,0x6d,0x67,0x66,0x55,0x04,0x00,0x11,0x11,0x21,0x21,0x13,0x23,0x42,0x43,0x44,0x53,0xf4}},

	{0x00,1,{0x00}},
	{0x59,1,{0x00}},


	//{0x00,1,{0x80}},//lcm ic self refresh
	//{0xA6,3,{0xA5,0x9F,01}},
		 
         //enable TE, just for cmd mode of mipi,		 
         {0x35,1,{0x00}}, 
		 {0x44,2,{0x01,0x00}},

		 //enable CABC function.
         {0x53,1,{0x24}},  //Dimming off  
         {0x55,1,{0x02}},
         {0x5E,1,{0x29}},	
		 
         //sleep out
         {0x11,0,{}},
         {REGFLAG_DELAY, 120, {}},
         //display on
         {0x29,0,{}},
         //{REGFLAG_DELAY, 20, {}},
         {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    {0x53,1,{0x00}},  //Dimming off 
    // Display off sequence
    {0x28, 1, {0x00}},
    //{REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    //Deep sleep in
    {0x00, 1, {0x91}},
    {0xF7, 4, {0x5A,0xA5,0x19,0x02}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    //Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
   // {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table2(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}


static void push_table(void* handle,struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmd_by_cmdq(handle,cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = PHYSICAL_WIDTH;
	params->physical_height = PHYSICAL_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      		= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;
	//video mode timing

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 8;
	params->dsi.vertical_frontporch					= 10;
	params->dsi.vertical_active_line					= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 20;
	params->dsi.horizontal_frontporch				= 40;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	//begin:haobing modified
	/*BEGIN PN:DTS2013013101431 modified by s00179437 , 2013-01-31*/
	//improve clk quality
	params->dsi.PLL_CLOCK = 440; //this value must be in MTK suggested table
	//params->dsi.CLK_HS_POST = 300; //add for screen mess
	//params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	//params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4
	//params->dsi.fbk_div =21;    	// fref=26MHz, fvco=fref*(fbk_div)*2/(div1_real*div2_real)
	/*END PN:DTS2013013101431 modified by s00179437 , 2013-01-31*/
	//end:haobing modified

//Lenovo-sw wuwl10 add 20150128 enable esd 
#if 1
	//params->dsi.cont_clock = 0;
	params->dsi.clk_lp_per_line_enable = 1;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
#endif
}

static void lcm_init(void)
{

	//int ret=0;
#if 0
	if(isAAL == 3)
	{
		mt_set_gpio_mode(GPIO_AAL_ID, GPIO_MODE_00);
    		mt_set_gpio_dir(GPIO_AAL_ID, GPIO_DIR_IN);
		isAAL = mt_get_gpio_in(GPIO_AAL_ID);
		dprintf(0, "[lenovo] %s isAAL is %d \n",__func__,isAAL);
	}
#endif
#ifndef BUILD_LK
	printk("[wuwl10] %s \n",__func__);
#endif
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
    	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
    	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	MDELAY(6);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
    	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
    	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);

	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(20);

	// when phone initial , config output high, enable backlight drv chip
	push_table2(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
#ifndef BUILD_LK
	printk("[wuwl10] %s \n",__func__);
#endif
#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
        sre_suspend = 1; //system suspend
#endif
	push_table2(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

	mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ZERO);
	//SET_RESET_PIN(1);
	//MDELAY(2);
	SET_RESET_PIN(0);
	MDELAY(2);
	SET_RESET_PIN(1);
	//MDELAY(2);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
    	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
    	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
	MDELAY(2);	
 	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
 	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
 	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
	MDELAY(6);	
}

static void lcm_resume(void)
{
	lcm_init();
#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
	if(orise_sre == 1)//add by wangyq13,if sre is on,set sre register when system resume
	{
	        lcm_sre_level_setting[0].para_list[0] = 0x40;
		push_table2(lcm_sre_level_setting, sizeof(lcm_sre_level_setting) / sizeof(struct LCM_setting_table), 1);
	}
#endif
#ifndef BUILD_LK
	printk("[wuwl10] %s \n",__func__);
#endif	
#if 0

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(50);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
 	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
 	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	MDELAY(6);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
    	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
    	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	MDELAY(15);

	push_table2(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1); 
#endif
	mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ONE);
}

static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
/*BEGIN PN:DTS2013013101431 modified by s00179437 , 2013-01-31*/
	//delete high speed packet
	//data_array[0]=0x00290508;
	//dsi_set_cmdq(data_array, 1, 1);
/*END PN:DTS2013013101431 modified by s00179437 , 2013-01-31*/
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
extern int super_backlight;
#endif
static void lcm_setbacklight(unsigned int level)
{
	unsigned char data_array[16];
	unsigned long real_level;

	if(level > 255)
		level = 255;
     
     //copy from K7-QC platform
if(level >=0 && level <=10)//(0,0),(10,10) 
real_level = (unsigned long)level;

else if(level <=128 &&  level>10)//(10,10),(128,75)
real_level = ((unsigned long)level*65 + 530)/118;

else if(level <=205 && level>128)//(128,75),(205,155)
real_level = ((unsigned long)level* 80 - 4465)/77;

else if(level <=255 && level>205) //(205,155),(255,255) 
real_level = ((unsigned long)level*2) - 255;

#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
	if(super_backlight != 1)
#endif
	level = ((unsigned int)real_level*81)/100; 

if((real_level == 1)||(real_level == 2)) //if real_level is one,level is zero,set it to one.
		level = 2;

#ifndef BUILD_LK
	printk("[wuwl10] %s level is %d \n",__func__,level);
//Lenovo-sw wuwl10 add 20150128 for esd recover backlight
	cmd_esd_last_backlight_level = level;
#endif
	data_array[0] = level;
	dsi_set_cmdq_V2(0x51, 1, data_array, 1);
#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
       if(sre_init ==0)
       	{
#endif
	        data_array[0] = 0x24;
	        dsi_set_cmdq_V2(0x53, 1, data_array, 1);
#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
	        sre_init = 1;
       	}
#endif
}

//Lenovo-sw wuwl10 add 20150128 for esd recover backlight begin
#ifndef BUILD_LK
static void lcm_esd_recover_backlight(void)
{
	unsigned char data_array[16];
	printk("%s, kernel otm1902a tm recover backlight: level = %d\n", __func__, cmd_esd_last_backlight_level);

	// recovder last backlight level.
	data_array[0] = cmd_esd_last_backlight_level;
	dsi_set_cmdq_V2(0x51, 1, data_array, 1);
	mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ONE);
}
#endif
//Lenovo-sw wuwl10 add 20150128 for esd recover backlight end

#ifdef CONFIG_LENOVO_CUSTOM_LCM_FEATURE
static void lcm_set_cabcmode(unsigned int mode)
{
	#ifdef BUILD_LK
		dprintf(0,"%s, mode = %d\n", __func__, mode);
	#else
		printk("%s, mode = %d\n", __func__, mode);
	#endif

	lcm_cabc_level_setting[0].para_list[0] = mode;
	
	push_table2(lcm_cabc_level_setting, sizeof(lcm_cabc_level_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_set_cabcmode_cmd(void* handle,unsigned int mode)
{
	#ifdef BUILD_LK
		dprintf(0,"%s, mode = %d\n", __func__, mode);
	#else
		printk("%s, mode = %d\n", __func__, mode);
	#endif
        orise_cabc = mode;
        if(!orise_sre)
        {
	      lcm_cabc_level_setting[0].para_list[0] = mode;	
	      push_table(handle,lcm_cabc_level_setting, sizeof(lcm_cabc_level_setting) / sizeof(struct LCM_setting_table), 1);
        }
}
#endif

#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
static void lcm_set_sremode(void* handle,unsigned int mode)
{
	#ifdef BUILD_LK
		dprintf(0,"%s, mode = %d\n", __func__, mode);
	#else
		printk("%s, mode = %d\n", __func__, mode);
	#endif
	orise_sre = mode;
	if(mode == 0)
	{
	        lcm_sre_level_setting[0].para_list[0] = orise_cabc;
	        push_table(handle,lcm_sre_level_setting, sizeof(lcm_sre_level_setting) / sizeof(struct LCM_setting_table), 1);
	}
	else
	{
	        lcm_sre_level_setting[0].para_list[0] = 0x40;
		push_table(handle,lcm_sre_level_setting, sizeof(lcm_sre_level_setting) / sizeof(struct LCM_setting_table), 1);
	}
}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned  int ret = 1;
	ret = mt_get_gpio_in(GPIO_DISP_ID0_PIN);
#ifdef BUILD_LK
	if(0 == ret)
		dprintf(0, "[LK]tianma_otm1902a found\n");
#else
	if(0 == ret)
		printk("[KERNEL]tianma_otm1902a found\n");
#endif
	return (0 == ret) ? 1: 0;

}

LCM_DRIVER otm1902a_fhd_dsi_cmd_tianma_lcm_drv=
{
    .name           = "otm1902a_fhd_dsi_cmd_tianma",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,/*tianma init fun.*/
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
//#ifdef LENOVO_LCD_BACKLIGHT_CONTROL_BY_LCM
	.set_backlight		= lcm_setbacklight,	
//#endif

#ifndef BUILD_LK
//Lenovo-sw wuwl10 add 20150128 for esd recover backlight begin
	.esd_recover_backlight = lcm_esd_recover_backlight,
//Lenovo-sw wuwl10 add 20150128 for esd recover backlight end
#ifdef CONFIG_LENOVO_CUSTOM_LCM_FEATURE
	.set_cabcmode = lcm_set_cabcmode,
	.set_cabcmode_cmd = lcm_set_cabcmode_cmd,
#endif
#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
        .set_sremode = lcm_set_sremode,
#endif
#endif
};
/* END PN:DTS2013053103858 , Added by d00238048, 2013.05.31*/
