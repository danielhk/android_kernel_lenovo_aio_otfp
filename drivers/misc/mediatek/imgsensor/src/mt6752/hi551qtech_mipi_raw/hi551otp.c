
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/semaphore.h>


//#include <asm/system.h>
#include <linux/xlog.h>

#ifdef CONFIG_COMPAT
//64 bit 
#include <linux/compat.h>
#endif

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi551mipi_Sensor.h"

#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include "hi551mipi_Sensor.h"

static DEFINE_SEMAPHORE(otp_sem);

#define  	otp_i2c_write_id  0x40

#define PFX "HI551OTP"

#define LOG_INF(format, args...)	 pr_info("[%s][%s] " format, PFX, __FUNCTION__, ##args)
static hi551Otp_struct hi551Otp = {0};
static kal_uint16 ReadSensorReg(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)((addr >> 8) & 0xFF), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, otp_i2c_write_id);
	return get_byte;
	
}

static void WriteSensorReg(kal_uint32 addr, kal_uint16 para)
{
	char pu_send_cmd[3] = {(char)((addr>> 8) & 0xFF), (char)(addr & 0xFF), (para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, otp_i2c_write_id);
}

static void HI551OTP_InitialOTPSetting() { 
	//sleep on 
	WriteSensorReg( 0x0118, 0x00);
	mdelay(100); 
	//pll disable 
	WriteSensorReg( 0x0f02, 0x00); 
	//CP TRI_H 
	WriteSensorReg( 0x011a, 0x01);
	//IPGM TRIM_H 
	WriteSensorReg( 0x011b, 0x09); 
	//Fsync Output enable 
	WriteSensorReg( 0x0d04, 0x01);
	//Fsync Output Drivability 
	WriteSensorReg( 0x0d00, 0x07); 
	//TG MCU enable 
	WriteSensorReg( 0x004c, 0x01); 
	//OTP R/W 
	WriteSensorReg( 0x003e, 0x01);
	//sleep off 
	WriteSensorReg( 0x0118, 0x01);
	
 } 
static int HI551OTP_ReadAll(kal_uint16 startAddr , kal_uint16 endAddr, kal_uint8 * buf) { 
	kal_uint16 value, i; 
	HI551OTP_InitialOTPSetting();
	 //read mode 
	for (i = startAddr; i <= endAddr; i++)	{ 
		//start address H 
		WriteSensorReg( 0x10a, ((i>>8)&0xff)); 
		//start address L 
		WriteSensorReg( 0x10b, (i&0xff)); 
		// single read 
		WriteSensorReg( 0x0102, 0x00); 
		udelay(10); 
		value =(kal_uint8)  ReadSensorReg( 0x0108 );
		udelay(10); 
		buf[i-startAddr] = value;
	} 
	return 0;
}
static kal_uint16 HI551OTPSensorReg(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	WriteSensorReg( 0x10a, ((addr>>8)&0xff)); 
	WriteSensorReg( 0x10b, (addr&0xff)); 
	WriteSensorReg( 0x0102, 0x00); 
	udelay(10); 
	get_byte =(kal_uint8)  ReadSensorReg( 0x0108 );
	return get_byte;
}

static void  HI551OTP_DUMP( kal_uint8 *databuf,kal_uint32 data_size)
{
	kal_uint16 i,j;
	struct file *fp=NULL;
	mm_segment_t fs;
	loff_t pos;
	int tail, line = data_size;

	line = data_size/16;
 	for (i = 0; i <line ;i++)  {
		printk("HI551OTP_DUMP line %d  ",i);
		for (j = 0; j < 16; j++)  {
			printk("%x  ",databuf[ j + i*16]);

		}
		printk("\n ");
 	}
	tail = data_size%16;
	printk("HI551OTP_DUMP line%d  ", line );
	for (j = 0; j< tail; j++)  {
		printk("%x   ", databuf[line * 16+j ]);
	}
	printk("\n ");	
}

static void HI551OTP_Printf()
{

    //OTP Module info
    LOG_INF("checksum =%x module_flag=%x module_info_valid =%x group=%x\n",hi551Otp.module_i.checksum,hi551Otp.module_i.module_flag,
    		     hi551Otp.module_i.module_info_valid, hi551Otp.module_i.group);
    LOG_INF("mid =%x calibration_version=%x\n",hi551Otp.module_i.data.info.mid,    hi551Otp.module_i.data.info.otp_calibration_version);
    LOG_INF("year =%x month=%x day=%x\n",hi551Otp.module_i.data.info. product_year,    hi551Otp.module_i.data.info. product_month,    hi551Otp.module_i.data.info. product_day);
    LOG_INF("sensor_id =%x lens_id=%x\n",hi551Otp.module_i.data.info. sensor_id,    hi551Otp.module_i.data.info. lens_id);
    LOG_INF("vcm_id =%x driver_ic_id=%x\n", hi551Otp.module_i.data.info. vcm_id,    hi551Otp.module_i.data.info. driver_ic_id);
    LOG_INF("ir_bg_id =%x color_temperature=%x\n", hi551Otp.module_i.data.info. ir_bg_id,    hi551Otp.module_i.data.info. color_temperature);
    LOG_INF("af_ff_flag =%x light_source_flag=%x\n",  hi551Otp.module_i.data.info. af_ff_flag,    hi551Otp.module_i.data.info. light_source_flag);
    LOG_INF(" position =%x reserved=%x\n",hi551Otp.module_i.data.info. cam_position,    hi551Otp.module_i.data.info. reserved);
	
     //OTP AWB info
     LOG_INF("AWB checksum =%x awb_flag=%x awb_info_valid =%x group=%x\n",
     			hi551Otp.awb_i.checksum,hi551Otp.awb_i.awb_flag,
    		    	 hi551Otp.awb_i.awb_info_valid, hi551Otp.awb_i.group);
	 
     LOG_INF("rgr= %x %x bgr=%x %x gbgr=%x %x\n",
	 		hi551Otp.awb_i.data.info.rgr_value_H, hi551Otp.awb_i.data.info.rgr_value_L,
	 		hi551Otp.awb_i.data.info.bgr_value_H,    hi551Otp.awb_i.data.info.bgr_value_L, 
	 		hi551Otp.awb_i.data.info.gbgr_value_H, hi551Otp.awb_i.data.info.gbgr_value_L);
    LOG_INF("goldrgr= %x %x goldbgr=%x %x goldgbgr=%x %x\n",
			hi551Otp.awb_i.data.info.golden_rgr_value_H,  hi551Otp.awb_i.data.info.golden_rgr_value_L,  
			hi551Otp.awb_i.data.info.golden_bgr_value_H,    hi551Otp.awb_i.data.info.golden_bgr_value_L,  
			hi551Otp.awb_i.data.info.golden_gbgr_value_H, hi551Otp.awb_i.data.info.golden_gbgr_value_L);
    LOG_INF("R_value= %x %x G_value=%x %x \n ",hi551Otp.awb_i.data.info.R_value_H,    hi551Otp.awb_i.data.info.R_value_L,  
			hi551Otp.awb_i.data.info.B_value_H,    hi551Otp.awb_i.data.info.B_value_L);
	
    LOG_INF("gr_value= %x %x Gb_value=%x %x\n ",hi551Otp.awb_i.data.info.Gr_value_H,  hi551Otp.awb_i.data.info.Gr_value_L,  
	 		hi551Otp.awb_i.data.info.Gb_value_H,    hi551Otp.awb_i.data.info.Gb_value_L);
    LOG_INF("goldR_value= %x %x goldB_value=%x %x\n ",hi551Otp.awb_i.data.info.golden_R_value_H,    hi551Otp.awb_i.data.info.golden_R_value_L,   
			hi551Otp.awb_i.data.info.golden_B_value_H,    hi551Otp.awb_i.data.info.golden_B_value_L);
    LOG_INF("gold_grvalue= %x %x gold_gbvalue=%x %x\n ",hi551Otp.awb_i.data.info.golden_Gr_value_H,    hi551Otp.awb_i.data.info.golden_Gr_value_L,  
			hi551Otp.awb_i.data.info.golden_Gb_value_H,   hi551Otp.awb_i.data.info.golden_Gb_value_L);

} 

static int HI551OTP_Check(void) {
	kal_uint32 addr,offset, checksum,i ,j;

	kal_uint8 otp_buffer[OTPBUFFER_SIZE];
	kal_uint8 value = 0;
		
	memset(&otp_buffer, 0, sizeof(otp_buffer));
	memset(&hi551Otp,0,sizeof(hi551Otp));
	HI551OTP_InitialOTPSetting();
	
	//HI551OTP_DUMP(otp_buffer,OTPBUFFER_SIZE);

	//module info
	hi551Otp.module_i.module_flag = HI551OTPSensorReg(OTP_MODULE_ADDR); 
	LOG_INF("module info flag = %x\n ",hi551Otp.module_i.module_flag);
	if(!(hi551Otp.module_i.module_flag & (1 <<2 |1 << 4|1<<6)))  {
		LOG_INF("no valid module info is found\n ");
		goto otp_error;
	}
		
	//module group
	for (j =0; j<3; j++) {
		if(((hi551Otp.module_i.module_flag >> (3-j)*2) & 0x3) == 1) { //group is valid
			offset = 1;
			addr = OTPV1_MODULE_GROUP_ONE;
			if (j  == 1) {
				addr = OTPV1_MODULE_GROUP_TWO;
			}else if( j == 2) {
				addr = OTPV1_MODULE_GROUP_THREE;
			}
			hi551Otp.module_i.group = j;
			hi551Otp.otp_version = HI551OTPSensorReg(addr + offset );
			break;
	   	 }
	}
    	LOG_INF("otp_version = %x\n ",hi551Otp.otp_version);
	if( hi551Otp.otp_version == 1 )   {
		if (HI551OTP_ReadAll(addr, addr + MODULE_V1_LENGTH -1, hi551Otp.module_i.data.module_data) < 0)
			goto otp_error;
			checksum = 0;
		for(i=0 ; i < MODULE_V1_LENGTH-1; i++) {
			value =  hi551Otp.module_i.data.module_data[i];
				checksum += value;
			}
		hi551Otp.module_i.checksum = hi551Otp.module_i.data.module_data[MODULE_V1_LENGTH -1];
		LOG_INF("module  group %d checksum =%x %x\n",hi551Otp.module_i.group,checksum,(kal_uint32)hi551Otp.module_i.checksum);

			checksum = checksum%0xFF +1;
			if(hi551Otp.module_i.checksum  !=  checksum) {
			LOG_INF("module  group %d checksum error\n",hi551Otp.module_i.group);
				goto otp_error;
			}
			LOG_INF("module  group %d checksum valid\n",j);
			hi551Otp.module_i.module_info_valid = 1;
		//READ AWB OTP INFO
	//AWB table 
		hi551Otp.awb_i.awb_flag = HI551OTPSensorReg(OTPV1_AWB_FLAG_ADDR);
	LOG_INF("AWB info flag= %x ",hi551Otp.awb_i.awb_flag);
	if(!(hi551Otp.awb_i.awb_flag & (1 <<2 |1 << 4|1<<6)) )  {
		LOG_INF("no valid awb  info is found ");
		goto otp_error;
	}
	for (j =0; j<3; j++) {
		if(((hi551Otp.awb_i.awb_flag >> (3-j)*2) & 0x3) == 1) { //group is valid
				addr = OTPV1_AWB_GROUP_ONE;          
				if (j ==1) {
					addr = OTPV1_AWB_GROUP_TWO;
				}else if (j==2) {
					addr = OTPV1_AWB_GROUP_THREE;
				}
				hi551Otp.awb_i.group = j;
				break;
			}
		}
		if (HI551OTP_ReadAll(addr, addr + AWB_LENGTH -1, hi551Otp.awb_i.data.awb_data) < 0)
			goto otp_error;
			checksum = 0;
			for(i=0 ; i<AWB_LENGTH -1; i++) {
			value =  hi551Otp.awb_i.data.awb_data[i];
				checksum += value;
			}
		hi551Otp.awb_i.checksum = hi551Otp.awb_i.data.awb_data[AWB_LENGTH -1];
			LOG_INF("awb  group %d checksum =%x, %x\n",j,checksum,(kal_uint32)hi551Otp.awb_i.checksum);
			checksum = checksum%0xFF +1;
			if(hi551Otp.awb_i.checksum != checksum) {
			LOG_INF("awb  group %d checksum error\n",hi551Otp.awb_i.group);
				goto otp_error;
			}
			hi551Otp.awb_i.awb_info_valid = 1;
			hi551Otp.awb_i.group = j;
		LOG_INF("awb  group %d checksum valid\n",hi551Otp.awb_i.group);
		
		
	
	if (hi551Otp.module_i.module_info_valid ==1 && hi551Otp.awb_i.awb_info_valid ==1) 
		return 0;
       } // end of  if cal_ver ==1 
otp_error:
	return -1;
}
void HI551OTP_ApplyWB(bool apply)
{
	kal_uint32 rgr,bgr,gbgr,golden_rgr,golden_bgr,golden_gbgr;
	kal_uint32 R_gain , B_gain ;
   	if(apply ) {
		if(hi551Otp.otp_valid_flag == 1) {
			rgr=(hi551Otp.awb_i.data.info.rgr_value_H<<2)+( hi551Otp.awb_i.data.info.rgr_value_L>>6);
			bgr=(hi551Otp.awb_i.data.info.bgr_value_H<<2)+(hi551Otp.awb_i.data.info.bgr_value_L>>6);

			golden_rgr = (hi551Otp.awb_i.data.info.golden_rgr_value_H<<2) + (hi551Otp.awb_i.data.info.golden_rgr_value_L>>6);
			golden_bgr = (hi551Otp.awb_i.data.info.golden_bgr_value_H<<2) + (hi551Otp.awb_i.data.info.golden_bgr_value_L>>6);

			R_gain = (kal_uint32)(golden_rgr *0x0100/ rgr); 
			B_gain =( kal_uint32)(golden_bgr *0x0100/bgr);

			WriteSensorReg(0x050c, (R_gain >> 8)); 
			WriteSensorReg(0x050d, (R_gain & 0xff)); 
			WriteSensorReg(0x050e, (B_gain >> 8)); 
			WriteSensorReg(0x050f, (B_gain & 0xff)); 
			LOG_INF( "HI551OTP_ApplyWB, r/gr:0x%x, b/gr:0x%x\n", rgr, bgr);
			LOG_INF( "HI551OTP_ApplyWB, r/golden_gr:0x%x, golden_b/gr:0x%x\n", golden_rgr, golden_bgr);
			LOG_INF( "HI551OTP_ApplyWB, R_gain0x%x,B_gain:0x%x\n",R_gain, B_gain);
			
		}else {
			LOG_INF( "HI551OTP_ApplyWB, No  valid data is found \n");
		}
   	}else
	{
   		kal_uint8  R_gain_H,R_gain_L,B_gain_H,B_gain_L;
		R_gain_H = ReadSensorReg(0x050c); 
		R_gain_L = ReadSensorReg(0x050d); 
		B_gain_H = ReadSensorReg(0x050e); 
		B_gain_L  =  ReadSensorReg(0x050f); 
		LOG_INF( "HI551OTP_ApplyWB, R_gain_H=0x%x,R_gain_L:0x%x\n",R_gain_H, R_gain_L);
		LOG_INF( "HI551OTP_ApplyWB, B_gain_H=0x%x,B_gain_L:0x%x\n",B_gain_H, B_gain_L);	
	}	     
} 
bool isHI551haveOTP(void) {
	return (hi551Otp.otp_valid_flag == 1);
}
int HI551OTP_OTP_Check(void)  {
	int i,ret;

	if(hi551Otp.otp_valid_flag != 0) {
		LOG_INF("HI551OTP_OTP_Check already checked otp_valid_flag=%d\n ",hi551Otp.otp_valid_flag);
		return hi551Otp.otp_valid_flag;
	}

	down_interruptible(&otp_sem);

	for(i=0;i<3;i++) 	{
		memset(&hi551Otp, 0, sizeof(hi551Otp));
		ret = HI551OTP_Check();
		if(ret != 0) {
			LOG_INF("HI551OTP_Check  failed %d!\n",i);
			hi551Otp.otp_valid_flag  = -1;
		}
		else	{
			hi551Otp.otp_valid_flag  = 1;
			LOG_INF("HI551OTP_Check  succeed %d !\n",i);
			HI551OTP_Printf();
			break;
		}
		HI551OTP_Printf();
	}

	up(&otp_sem);
	return hi551Otp.otp_valid_flag;
}

int HI551_OTP_Ver_Check(void)
{
	printk("%s,otp_valid_flag=%d,cal_version=%d\n",__func__,
		hi551Otp.otp_valid_flag,
		hi551Otp.module_i.data.info.otp_calibration_version); 

	if (hi551Otp.otp_valid_flag <= 0){
		return 0;
	} else {
		return hi551Otp.module_i.data.info.otp_calibration_version;
	}

}
