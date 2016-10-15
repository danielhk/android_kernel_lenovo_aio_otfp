
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
	#include <mach/mt_pm_ldo.h>
    #include <mach/mt_gpio.h>
#endif
#include <cust_gpio_usage.h>
#ifndef FPGA_EARLY_PORTING
#include <cust_i2c.h>
#endif
//Lenovo-sw wuwl10 add 20141224 for esd recover backlight
#ifndef BUILD_LK
static unsigned int esd_last_backlight_level = 255;
#endif

static const unsigned int BL_MIN_LEVEL =20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    


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
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	tps65132_i2c_client  = client;		
	return 0;      
}


static int tps65132_remove(struct i2c_client *client)
{  	
  tps65132_i2c_client = NULL;
   i2c_unregister_device(client);
  return 0;
}


 int tps65132_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
		printk("tps65132 write data fail !!\n");	
	return ret ;
}
EXPORT_SYMBOL_GPL(tps65132_write_bytes);

static int __init tps65132_iic_init(void)
{
   i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
   i2c_add_driver(&tps65132_iic_driver);
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
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(1080)//(1080)
#define FRAME_HEIGHT 										(1920)//(1920)
#define PHYSICAL_WIDTH										(68)
#define PHYSICAL_HEIGHT										(121)
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#ifndef GPIO_LCD_BIAS_ENP_PIN
#define GPIO_LCD_BIAS_ENP_PIN GPIO122
#endif
#ifndef GPIO_LCD_BIAS_ENN_PIN
#define GPIO_LCD_BIAS_ENN_PIN GPIO95
#endif
#ifndef GPIO_LCM_BL_EN
#define GPIO_LCM_BL_EN  (GPIO113 | 0x80000000)  //wuwl10 modify for gpio warning
#endif
#ifndef GPIO_LCM_LED_EN
#define GPIO_LCM_LED_EN GPIO94
#endif
//wuwl10 20150411 add for pabel auto detect
#ifndef GPIO_DISP_ID0_PIN
#define GPIO_DISP_ID0_PIN (GPIO114 | 0x80000000)
#endif

#define LCM_DSI_CMD_MODE									0
#ifndef FPGA_EARLY_PORTING
#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
#endif
#define LCM_ID_OTM1284 0x40

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
       

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
{0x51, 1, {0xFF}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_cabc_level_setting[] = {
{0x55, 1, {0x00}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_inverse_off_setting[] = {
{0x20, 1, {0x00}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_inverse_on_setting[] = {
{0x21, 1, {0x00}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#ifdef BUILD_LK
#ifndef FPGA_EARLY_PORTING
#define TPS65132_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t TPS65132_i2c;

int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    TPS65132_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL;//I2C2;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
    TPS65132_i2c.mode = ST_MODE;
    TPS65132_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&TPS65132_i2c, write_data, len);
    //printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}
#endif
#endif
static void lcm_init_power(void)
{
#ifndef FPGA_EARLY_PORTING
#ifdef BUILD_LK
	mt6325_upmu_set_rg_vgp1_en(1);
#else
	hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	
#endif
#endif
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
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
		params->dsi.mode   = BURST_VDO_MODE;
#endif
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 18;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 42;//2;
	params->dsi.horizontal_backporch				= 64;
	params->dsi.horizontal_frontporch				= 13;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

#ifndef FPGA_EARLY_PORTING
	params->dsi.PLL_CLOCK = 480;//220; //this value must be in MTK suggested table

#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
	params->dsi.LPX = 10;
	//lenovo_sw wuwl10 20141221 modify for enable esd check
    #if 0
    params->dsi.esd_check_enable =1;
    params->dsi.customization_esd_check_enable =1;

    params->dsi.lcm_esd_check_table[2].cmd =0x54;
    params->dsi.lcm_esd_check_table[2].count =1;
    params->dsi.lcm_esd_check_table[2].para_list[0] =0x24;


    params->dsi.lcm_esd_check_table[1].cmd =0xEF;
    params->dsi.lcm_esd_check_table[1].count =4;
    params->dsi.lcm_esd_check_table[1].para_list[0] =0x01;
    params->dsi.lcm_esd_check_table[1].para_list[1] =0x31;
    params->dsi.lcm_esd_check_table[1].para_list[2] =0x0a;
    params->dsi.lcm_esd_check_table[1].para_list[3] =0xff;

    params->dsi.lcm_esd_check_table[0].cmd =0x09;
    params->dsi.lcm_esd_check_table[0].count =4;
    params->dsi.lcm_esd_check_table[0].para_list[0] =0x80;
    params->dsi.lcm_esd_check_table[0].para_list[1] =0x73;
    params->dsi.lcm_esd_check_table[0].para_list[2] =0x06;
    params->dsi.lcm_esd_check_table[0].para_list[3] =0x00;
#endif
}
//lenovo-sw wuwl10 20150411 add for highlight tm panel begin
const static unsigned char LCD_MODULE_ID = 0x00;
static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char  id_pin_read = 0;


#ifdef GPIO_DISP_ID0_PIN
	id_pin_read = mt_get_gpio_in(GPIO_DISP_ID0_PIN);
#endif
#ifdef BUILD_LK
	dprintf(0, "%s,  id hx8395a tm= 0x%x LCD_ID_value=%d \n", __func__, id,id_pin_read);
#endif
	if(LCD_MODULE_ID == id_pin_read)
		return 1;
	else
		return 0;
}
//lenovo-sw wuwl10 20150411 add for highlight tm panel end

static void lcm_init(void)
{
	unsigned int data_array[16];

	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;

#ifndef FPGA_EARLY_PORTING
	MDELAY(1);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	MDELAY(2);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	MDELAY(1);
#endif	

	cmd=0x00;
	data=0x0F;
#ifdef BUILD_LK
	ret=TPS65132_write_byte(cmd,data);
    if(ret)    	
    dprintf(0, "[LK]tps65132----cmd=%0x--i2c write error----\n",cmd);    	
	else
	dprintf(0, "[LK]tps65132----cmd=%0x--i2c write success----\n",cmd);    		
#else
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]tps65132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]tps65132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
	cmd=0x01;
	data=0x0F;
#ifdef BUILD_LK
	ret=TPS65132_write_byte(cmd,data);
    if(ret)    	
	    dprintf(0, "[LK]tps65132----cmd=%0x--i2c write error----\n",cmd);    	
	else
		dprintf(0, "[LK]tps65132----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]tps65132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]tps65132---cmd=%0x-- i2c write success-----\n",cmd);
#endif	
	
	SET_RESET_PIN(1);
	MDELAY(1);//5
    SET_RESET_PIN(0);
	MDELAY(1);//50
    SET_RESET_PIN(1);
	MDELAY(50);//100

	data_array[0]= 0x00043902;
	data_array[1]= 0x9583FFB9;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0]= 0x00053902;//10-20 ver.C update
	data_array[1]= 0xA80373BA;
	data_array[2]= 0x0000007B;
	dsi_set_cmdq(&data_array, 3, 1);

	data_array[0]= 0x000B3902;//10-20 ver.C update
	data_array[1]= 0x52126CB1;
	data_array[2]= 0xF1110424;//10-20 ver.C update
	data_array[3]= 0x00E9E380;
	dsi_set_cmdq(&data_array, 4, 1);


  	data_array[0]= 0x00023902;//10-20 ver.C update
	data_array[1]= 0x000099D2;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0]= 0x00063902;//10-20 ver.C update
	data_array[1]= 0x0CB400B2;
	data_array[2]= 0x00002A0A;
	dsi_set_cmdq(&data_array, 3, 1);

	data_array[0]= 0x00383902;	//GIP;//10-20 ver.C update
	data_array[1]= 0x000000D3; 
	data_array[2]= 0x00100000;
	data_array[3]= 0x00011032;
	data_array[4]= 0xC0133201;
	data_array[5]= 0x10320000;
	data_array[6]= 0x37000008;
	data_array[7]= 0x37030304;
	data_array[8]= 0x0C470004;//10-20 ver.C update
	data_array[9]= 0x0A000008;//10-20 ver.C update
	data_array[10]= 0x15010300;//10-20 ver.C update
	data_array[11]= 0x00000000;
	data_array[12]= 0xC0030000;
	data_array[13]= 0x04020800;
	data_array[14]= 0x15010000;
	dsi_set_cmdq(&data_array, 15, 1);


	data_array[0]= 0x00173902;//9.22 Update
	data_array[1]= 0x0EFF00B4;
	data_array[2]= 0x0E4A0E4A;
	data_array[3]= 0x014C014A;
	data_array[4]= 0x013A014C; //10-24 ver.C update soff
	data_array[5]= 0x013A033A;
	data_array[6]= 0x004A014A;
	dsi_set_cmdq(&data_array, 7, 1);


	data_array[0]= 0x002D3902;  // Forward
	data_array[1]= 0x183939D5;
	data_array[2]= 0x03000118;
	data_array[3]= 0x07040502;
	data_array[4]= 0x18181806;
	data_array[5]= 0x38181818;
	data_array[6]= 0x21191938;
	data_array[7]= 0x18222320;
	data_array[8]= 0x18181818;
	data_array[9]= 0x18181818;
	data_array[10]= 0x18181818;
	data_array[11]= 0x18181818;
	data_array[12]= 0x00000018;
	dsi_set_cmdq(&data_array, 13, 1);

	data_array[0]= 0x002D3902;  // Backward
	data_array[1]= 0x193939D6;
	data_array[2]= 0x04070619;
	data_array[3]= 0x00030205;
	data_array[4]= 0x18181801;
	data_array[5]= 0x38181818;
	data_array[6]= 0x22181838;
	data_array[7]= 0x58212023;
	data_array[8]= 0x58585858;
	data_array[9]= 0x58585858;
	data_array[10]= 0x58585858;
	data_array[11]= 0x58585858;
	data_array[12]= 0x00000058;
	dsi_set_cmdq(&data_array, 13, 1);
	
  	data_array[0]= 0x00023902;
	data_array[1]= 0x000008CC;
	dsi_set_cmdq(&data_array, 2, 1);

  	data_array[0]= 0x00033902;
	data_array[1]= 0x001530C0;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0]= 0x00063902;
	data_array[1]= 0x0C0800C7;
	data_array[2]= 0x0000D000;
	dsi_set_cmdq(&data_array, 3, 1);

	data_array[0]= 0x002B3902;
	data_array[1]= 0x110E00E0;
	data_array[2]= 0x213F342E;
	data_array[3]= 0x0C0B0741;
	data_array[4]= 0x13120F17;
	data_array[5]= 0x10071411;
	data_array[6]= 0x0D001511;
	data_array[7]= 0x3F352E12;
	data_array[8]= 0x0B074122;
	data_array[9]= 0x120F170D;
	data_array[10]= 0x07131112;
	data_array[11]= 0x00151111;
	dsi_set_cmdq(&data_array, 12, 1); 

  //********************YYG*******************//
  	data_array[0]= 0x00023902;
	data_array[1]= 0x000001BD;
	dsi_set_cmdq(&data_array, 2, 1);
	
	// Himax EF
  	data_array[0]= 0x00043902;
	data_array[1]= 0x010002EF;
	dsi_set_cmdq(&data_array, 2, 1);

  	data_array[0]= 0x00023902;
	data_array[1]= 0x000000BD;
	dsi_set_cmdq(&data_array, 2, 1);

  	data_array[0]= 0x00023902;
	data_array[1]= 0x000000BD;
	dsi_set_cmdq(&data_array, 2, 1);

  	data_array[0]= 0x00073902;
	data_array[1]= 0xF700B1EF;
	data_array[2]= 0x00022F37;
	dsi_set_cmdq(&data_array, 3, 1);

  	data_array[0]= 0x00033902;
	data_array[1]= 0x0060B2EF;
	dsi_set_cmdq(&data_array, 2, 1);

  	data_array[0]= 0x000E3902;
	data_array[1]= 0x0000B5EF;
  	data_array[2]= 0xFF000000;
	data_array[3]= 0x00000003;
	data_array[4]= 0x00000000;
	dsi_set_cmdq(&data_array, 5, 1);

  	data_array[0]= 0x00163902;
	data_array[1]= 0x0040B6EF;
  	data_array[2]= 0x01000080;
	data_array[3]= 0x38040080;
	data_array[4]= 0x20200004;
	data_array[5]= 0x00043804;
	data_array[6]= 0x00002020;
	dsi_set_cmdq(&data_array, 7, 1);

 	data_array[0]= 0x00093902;
	data_array[1]= 0xA000B7EF;
 	data_array[2]= 0x00000400;
	data_array[3]= 0x00000800;
	dsi_set_cmdq(&data_array, 4, 1);

 	data_array[0]= 0x00303902;
	data_array[1]= 0x0000C0EF;
 	data_array[2]= 0x60026002;
	data_array[3]= 0xC005C004;
	data_array[4]= 0x800B8008;
	data_array[5]= 0x00160010;
	data_array[6]= 0x002D0020;
 	data_array[7]= 0x005A0040;
	data_array[8]= 0x00B40080;
 	data_array[9]= 0x01680100;
	data_array[10]= 0x02D10201;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0000C1EF;
  	data_array[2]= 0x60026002;
	data_array[3]= 0xC005C004;
	data_array[4]= 0x800B8008;
	data_array[5]= 0x00160010;
	data_array[6]= 0x002D0020;
  	data_array[7]= 0x005A0040;
	data_array[8]= 0x00B40080;
  	data_array[9]= 0x01680100;
	data_array[10]= 0x02D10201;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0001C2EF;
  	data_array[2]= 0x60996098;
	data_array[3]= 0xC132C031;
	data_array[4]= 0x82648062;
	data_array[5]= 0x04C807C5;
	data_array[6]= 0x09910D8B;
  	data_array[7]= 0x13231A16;
	data_array[8]= 0x2647282C;
  	data_array[9]= 0x4C8E5F58;
	data_array[10]= 0x991C80B1;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 13, 1);

  	data_array[0]= 0x00243902;
	data_array[1]= 0x0002C4EF;
  	data_array[2]= 0x8E888E89;
	data_array[3]= 0x1D101D12;
	data_array[4]= 0x3A213A25;
	data_array[5]= 0x7443744B;
	data_array[6]= 0xE886E897;
  	data_array[7]= 0xD10DD12E;
	data_array[8]= 0x88888888;
  	data_array[9]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 10, 1);

  	data_array[0]= 0x00283902;
	data_array[1]= 0x0002C5EF;
  	data_array[2]= 0x8E888E89;
	data_array[3]= 0x1D101D12;
	data_array[4]= 0x3A213A25;
	data_array[5]= 0x7443744B;
	data_array[6]= 0xE886E897;
  	data_array[7]= 0xD10DD12E;
	data_array[8]= 0xA21BA25C;
  	data_array[9]= 0x88888888;
	data_array[10]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 11, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0005C7EF;
  	data_array[2]= 0x5C315C31;
	data_array[3]= 0xB863B862;
	data_array[4]= 0x70C670C5;
	data_array[5]= 0xE18CE18B;
	data_array[6]= 0xC319C317;
  	data_array[7]= 0x8633862F;
	data_array[8]= 0x4C764C5F;
  	data_array[9]= 0x58AD588E;
	data_array[10]= 0x7266703C;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0000C8EF;
  	data_array[2]= 0x00000000;
	data_array[3]= 0x00000000;
	data_array[4]= 0x00000000;
	data_array[5]= 0x00000000;
	data_array[6]= 0x00000000;
  	data_array[7]= 0x00000000;
	data_array[8]= 0x00000000;
  	data_array[9]= 0x00000000;
	data_array[10]= 0x00000000;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0000C9EF;
  	data_array[2]= 0x00000000;
	data_array[3]= 0x00000000;
	data_array[4]= 0x00000000;
	data_array[5]= 0x00000000;
	data_array[6]= 0x00000000;
  	data_array[7]= 0x00000000;
	data_array[8]= 0x00000000;
  	data_array[9]= 0x00000000;
	data_array[10]= 0x00000000;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0000CAEF;
  	data_array[2]= 0x00000000;
	data_array[3]= 0x00000000;
	data_array[4]= 0x00000000;
	data_array[5]= 0x00000000;
	data_array[6]= 0x00000000;
  	data_array[7]= 0x00000000;
	data_array[8]= 0x00000000;
  	data_array[9]= 0x00000000;
	data_array[10]= 0x00000000;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0000CBEF;
  	data_array[2]= 0x00000000;
	data_array[3]= 0x00000000;
	data_array[4]= 0x00000000;
	data_array[5]= 0x00000000;
	data_array[6]= 0x00000000;
  	data_array[7]= 0x00000000;
	data_array[8]= 0x00000000;
  	data_array[9]= 0x00000000;
	data_array[10]= 0x00000000;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 13, 1);

  	data_array[0]= 0x00183902;
	data_array[1]= 0x0004CCEF;
  	data_array[2]= 0x300D300C;
	data_array[3]= 0x601A6018;
	data_array[4]= 0xC035C031;
	data_array[5]= 0x88888888;
	data_array[6]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 7, 1);

  	data_array[0]= 0x00073902;
	data_array[1]= 0x3210D3EF;
  	data_array[2]= 0x00107654;
	dsi_set_cmdq(&data_array, 3, 1);

  	data_array[0]= 0x00063902;
	data_array[1]= 0x3210D4EF;
  	data_array[2]= 0x00007654;
	dsi_set_cmdq(&data_array, 3, 1);

  	data_array[0]= 0x00053902;
	data_array[1]= 0x0101D5EF;
  	data_array[2]= 0x00000001;
	dsi_set_cmdq(&data_array, 3, 1);

  	data_array[0]= 0x00033902;
	data_array[1]= 0x0012D6EF;
	dsi_set_cmdq(&data_array, 2, 1);

  	data_array[0]= 0x00063902;
	data_array[1]= 0xC14CDEEF;
  	data_array[2]= 0x00008D52;
	dsi_set_cmdq(&data_array, 3, 1);

  	data_array[0]= 0x00143902;
	data_array[1]= 0xAB00DFEF;
  	data_array[2]= 0x0A0B0C0D;
	data_array[3]= 0x0A0B0C0D;
	data_array[4]= 0x00FF0000;
	data_array[5]= 0x0A0B0C0D;
	dsi_set_cmdq(&data_array, 6, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0001C3EF;
  	data_array[2]= 0x60996098;
	data_array[3]= 0xC132C331;
	data_array[4]= 0x82648662;
	data_array[5]= 0x04C802C5;
	data_array[6]= 0x0991018B;
  	data_array[7]= 0x13231F16;
	data_array[8]= 0x2647362C;
  	data_array[9]= 0x4C8E5858;
	data_array[10]= 0x991C80B1;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 13, 1);

  	data_array[0]= 0x00343902;
	data_array[1]= 0x0003C6EF;
  	data_array[2]= 0x62DB62DB;
	data_array[3]= 0xC5B6C5B7;
	data_array[4]= 0x8B6C8B6F;
	data_array[5]= 0x16D916DE;
	data_array[6]= 0x2DB32DBC;
  	data_array[7]= 0x5B665B79;
	data_array[8]= 0xB6CDB6F2;
  	data_array[9]= 0x6D9A6DE5;
	data_array[10]= 0xDB34DBCA;
	data_array[11]= 0xB668B795;
	data_array[12]= 0x88888888;
	data_array[13]= 0xFFFFFFFF;
	dsi_set_cmdq(&data_array, 14, 1);

	data_array[0]= 0x00063902;
	data_array[1]= 0x3101ddef;
	data_array[2]= 0x0000ff0A; 
	
	dsi_set_cmdq(&data_array, 3, 1);


 	data_array[0]= 0x00033902;
	data_array[1]= 0x00DDbfef;
	dsi_set_cmdq(&data_array, 2, 1);


  //********************YYG*******************//
 
        data_array[0]= 0x000A3902; 
        data_array[1]= 0x14001fC9;// PWM 20K 
        data_array[2]= 0x001E001E; 
        data_array[3]= 0x00004480; 
        dsi_set_cmdq(&data_array, 4, 1); 

	data_array[0] = 0x00352300;//te on
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x00512300;//bl mode
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x24532300;//bl mode
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x02552300;//cabc 03
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0] = 0x325E2300;//cabc min limit
	dsi_set_cmdq(&data_array, 1, 1);

	//CE
 	data_array[0]= 0x00033902;
	data_array[1]= 0x0000C3E4;
	dsi_set_cmdq(&data_array, 2, 1);

	MDELAY(5);
 	data_array[0]= 0x000B3902;
	data_array[1]= 0x0E0800E5;
	data_array[2]= 0x2000080A;
	data_array[3]= 0x00000020;
	dsi_set_cmdq(&data_array, 4, 1);
 	
        data_array[0]= 0x00123902;
	data_array[1]= 0x000000E6;
	data_array[2]= 0x00000505;
	data_array[3]= 0x10202003;
	data_array[4]= 0x03030305;
	data_array[5]= 0x00000305;
	dsi_set_cmdq(&data_array, 6, 1);
	//ce end
	data_array[0] = 0x00110500;	
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;	
	dsi_set_cmdq(&data_array, 1, 1);

#ifdef GPIO_LCM_BL_EN
	mt_set_gpio_mode(GPIO_LCM_BL_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_BL_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ONE);
#endif
#ifdef GPIO_LCM_LED_EN
	mt_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ONE);
#endif
}

static void lcm_suspend(void)
{
#ifdef GPIO_LCM_BL_EN
	mt_set_gpio_mode(GPIO_LCM_BL_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_BL_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ZERO);
#endif
#ifdef GPIO_LCM_LED_EN
	mt_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ZERO);
#endif
	SET_RESET_PIN(1);	
	MDELAY(10);	
	SET_RESET_PIN(0);
	MDELAY(10);	
	SET_RESET_PIN(1);
	MDELAY(120);

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
//lenovo-sw wuwl10 20141230  modify for new timming
	MDELAY(5);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
	MDELAY(50);
}



static void lcm_resume(void)
{
	lcm_init();
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
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);	
}

static void lcm_setbacklight(unsigned int level)
{
	#ifdef BUILD_LK
		dprintf(0,"%s, level = %d\n", __func__, level);
	#else
		printk("%s tm, level = %d\n", __func__, level);
//Lenovo-sw wuwl10 add 20141224 for esd recover backlight
	esd_last_backlight_level = level;
#endif
//Lenovo-sw wuwl10 add 20150109 for min backlight control
	if((0 < level) && (level < 5))
	{
		level = 5;
	}
	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = level;
	
	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);

}
#ifndef BUILD_LK
//Lenovo-sw wuwl10 add 20141224 for esd recover backlight begin
static void lcm_esd_recover_backlight(void)
{
	printk("%s, kernel hx8395 recover backlight: level = %d\n", __func__, esd_last_backlight_level);

	// recovder last backlight level.
	lcm_backlight_level_setting[0].para_list[0] = esd_last_backlight_level;
	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);

}
#endif
//Lenovo-sw wuwl10 add 20141224 for esd recover backlight end

#ifdef CONFIG_LENOVO_CUSTOM_LCM_FEATURE
static void lcm_set_cabcmode(unsigned int mode)
{
	#ifdef BUILD_LK
		dprintf(0,"%s, mode = %d\n", __func__, mode);
	#else
		printk("%s, mode = %d\n", __func__, mode);
	#endif

	lcm_cabc_level_setting[0].para_list[0] = mode;
	
	push_table(lcm_cabc_level_setting, sizeof(lcm_cabc_level_setting) / sizeof(struct LCM_setting_table), 1);
}
static void lcm_set_inversemode(unsigned int mode)
{
	#ifdef BUILD_LK
		dprintf(0,"%s, mode = %d\n", __func__, mode);
	#else
		printk("%s, mode = %d\n", __func__, mode);
	#endif

if(mode)	
	push_table(lcm_inverse_on_setting, sizeof(lcm_inverse_on_setting) / sizeof(struct LCM_setting_table), 1);
else
	push_table(lcm_inverse_off_setting, sizeof(lcm_inverse_off_setting) / sizeof(struct LCM_setting_table), 1);
}
#endif
LCM_DRIVER hx8395a_hd720_dsi_vdo_tm_lcm_drv = 
{
    .name			= "hx8395a_dsi_vdo_tm",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.init_power		= lcm_init_power,
	.set_backlight	= lcm_setbacklight,
//Lenovo-sw wuwl10 add 20141224 for esd recover backlight begin
#ifndef BUILD_LK
	//.esd_recover_backlight = lcm_esd_recover_backlight,
#endif
//Lenovo-sw wuwl10 add 20141224 for esd recover backlight end
//lenovo-sw wuwl10 add 20150411 for highlight boe panel
	.compare_id     = lcm_compare_id,
#ifdef CONFIG_LENOVO_CUSTOM_LCM_FEATURE
	.set_cabcmode = lcm_set_cabcmode,
	.set_inversemode = lcm_set_inversemode,
#endif
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
