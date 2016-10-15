#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
//#include <asm/system.h>

#include <linux/proc_fs.h> 

#include <linux/dma-mapping.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov13850ofilmmipiraw_Sensor.h"
#include "ov13850ofilm_otp.h"

#define PFX "OV13850OFILM_R2A_OTP"
#define Ov13850_OTP_DEBUG
#define TRACE(format, args...)  xlog_printk(ANDROID_LOG_INFO   , PFX,"  [%s] " format, __FUNCTION__, ##args)


#define OTP_WRITE_ID   0x6c  //0x20
#define OTP_READ_ID    0xB0
//************************shenan   OTP ****************************************
static int RGr_Ratio_Typical_EEPROM =299;   //Sunny add
static int BGr_Ratio_Typical_EEPROM=296;    //Sunny add
static int GbGr_Ratio_Typical_EEPROM=511;    //Sunny add
static void ov13850r2a_write_cmos_sensor(kal_uint8 slaveID,kal_uint32 addr,kal_uint32 value)
{
  char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(value & 0xFF)};
  //TRACE("shenan, slaveID = 0x%x addr = 0x%x value = 0x%x \n", slaveID,addr,value);
  //TRACE("shenan, pu_send_cmd[0] = 0x%x pu_send_cmd[1] =0x%x pu_send_cmd[2] = 0x%x \n", pu_send_cmd[0],pu_send_cmd[1],pu_send_cmd[2]);
  iWriteRegI2C(pu_send_cmd, 3, slaveID);
}
static  kal_uint16 ov13850r2a_read_cmos_sensor(kal_uint8 slaveID,kal_uint32 addr)
{
  kal_uint16 get_byte=0;

  char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
  iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, slaveID);
  
  return get_byte;
}
//*******************************OTP END***************************************
//lenovo.sw  START wangsx3 add for AF otp
#define OV13850_AF_DATA_CNT	7
struct ov13850_otp_af_info{ 
	kal_uint8	af_flag;
	/*
	kal_uint8	af_cal_dir;
	kal_uint8	af_inf_high;
	kal_uint8	af_inf_low;
	kal_uint8	af_mac_high;
	kal_uint8	af_mac_low;	
	kal_uint8	af_start_high;
	kal_uint8	af_start_low;	
	kal_uint8	rsv_2;*/
	kal_uint8	af_data[OV13850_AF_DATA_CNT];
	kal_uint8	af_checksum;
	kal_uint8	af_valid;
};
static struct ov13850_otp_af_info ov13850_otp_af;
//lenovo.sw  END wangsx3 add for AF otp

static bool ov13850r2a_check_eeprom_flag(kal_uint8 slaveID,kal_uint32 addr)
{
    kal_uint32 value=0;
    value = ov13850r2a_read_cmos_sensor(slaveID,addr);
    if((value&0x01)==0)//judge Module information/AWB/LSC Flag
    {   
        return false;
    }
    else
    {   
        return true;
    }   
}
    
static bool OV13850r2a_read_eeprom(eeprom_struct* pCurrent_eeprom)
{
    bool result = false;
    result = ov13850r2a_check_eeprom_flag(OTP_READ_ID,0x0400);
    if(result)
    {
      kal_uint8 slaveID = 0;
      kal_uint32 addr = 0;
      unsigned  short /*int*/ eeprom_value[414];//415
      int i=0;
      int j=0;
      //calculate check sum
      unsigned int A_L_M_checksum_Read=0;
      unsigned int A_L_M_checksum_Cal=0;
      unsigned int A_L_M_checksum_Total=0;        
      slaveID = 0xB0;
      addr = 0x0400;
      for(addr = 0x0400; addr <= 0x059E; addr++)
      {       
          eeprom_value[i] = (unsigned short)ov13850r2a_read_cmos_sensor(OTP_READ_ID,addr);
          //TRACE(" slaveID : 0x%x, add:0x%x, value[%d] :0x%x \n",slaveID,addr,i,eeprom_value[i]);
          i++;
      }
  
      A_L_M_checksum_Read = eeprom_value[414];//checksum
      for(i=1;i<=413;i++)
      {
          A_L_M_checksum_Total += eeprom_value[i];
      }
      A_L_M_checksum_Cal = A_L_M_checksum_Total%255 + 1;
      //TRACE("ov13850r2a_read_cmos_sensor, A_L_M_checksum_Read = 0x%x, A_L_M_checksum_Cal:0x%x, A_L_M_checksum_Total:0x%x \n",A_L_M_checksum_Read,A_L_M_checksum_Cal,A_L_M_checksum_Total);  
      if(A_L_M_checksum_Read==A_L_M_checksum_Cal)
      {
        pCurrent_eeprom->module_info.mid= eeprom_value[1];
        pCurrent_eeprom->module_info.sensor_verson= eeprom_value[2];
        pCurrent_eeprom->module_info.otp_calibration_version= eeprom_value[3];
        pCurrent_eeprom->module_info.dll_version= eeprom_value[4];
        TRACE("ov13850r2a_read_cmos_sensor, mid = 0x%x, sensor_verson:0x%x, otp_calibration_version:0x%x, dll_version:0x%x \n",pCurrent_eeprom->module_info.mid,pCurrent_eeprom->module_info.sensor_verson,pCurrent_eeprom->module_info.otp_calibration_version,pCurrent_eeprom->module_info.dll_version); 
        pCurrent_eeprom->module_info.product_year = eeprom_value[5];
        pCurrent_eeprom->module_info.product_month=eeprom_value[6];
        pCurrent_eeprom->module_info.product_day=eeprom_value[7];
        //TRACE("ov13850r2a_read_cmos_sensor, product_year = 0x%x, product_month:0x%x, product_day:0x%x \n",pCurrent_eeprom->module_info.product_year,pCurrent_eeprom->module_info.product_month,pCurrent_eeprom->module_info.product_day); 
        pCurrent_eeprom->module_info.sensor_id=eeprom_value[8];
        pCurrent_eeprom->module_info.lens_id=eeprom_value[9];
        pCurrent_eeprom->module_info.vcm_id=eeprom_value[10];
        pCurrent_eeprom->module_info.driver_ic_id=eeprom_value[11];
        pCurrent_eeprom->module_info.ir_bg_id=eeprom_value[12];
        //TRACE("ov13850r2a_read_cmos_sensor, sensor_id = 0x%x, lens_id = 0x%x, vcm_id:0x%x, driver_ic_id:0x%x, ir_bg_id:0x%x\n",pCurrent_eeprom->module_info.sensor_id,pCurrent_eeprom->module_info.lens_id,pCurrent_eeprom->module_info.vcm_id,pCurrent_eeprom->module_info.driver_ic_id,pCurrent_eeprom->module_info.ir_bg_id); 
        pCurrent_eeprom->module_info.color_temperature=eeprom_value[13];
        pCurrent_eeprom->module_info.af_ff_flag=eeprom_value[14];
        pCurrent_eeprom->module_info.light_source_flag=eeprom_value[15];
        //TRACE("ov13850r2a_read_cmos_sensor, color_temperature = 0x%x, af_ff_flag = 0x%x, light_source_flag:0x%x, driver_ic_id:0x%x, ir_bg_id:0x%x\n",pCurrent_eeprom->module_info.color_temperature,pCurrent_eeprom->module_info.af_ff_flag,pCurrent_eeprom->module_info.light_source_flag); 

        pCurrent_eeprom->awb_info.rgr_value_H=eeprom_value[19];
        pCurrent_eeprom->awb_info.rgr_value_L=eeprom_value[20];
        pCurrent_eeprom->awb_info.bgr_value_H=eeprom_value[21];
        pCurrent_eeprom->awb_info.bgr_value_L=eeprom_value[22];
        pCurrent_eeprom->awb_info.gbgr_value_H=eeprom_value[23];
        pCurrent_eeprom->awb_info.gbgr_value_L=eeprom_value[24];
        //TRACE("ov13850r2a_read_cmos_sensor, rgr_value_H = 0x%x, rgr_value_L:0x%x, bgr_value_H:0x%x, bgr_value_L:0x%x, gbgr_value_H:0x%x, gbgr_value_L:0x%x,\n",pCurrent_eeprom->awb_info.rgr_value_H,pCurrent_eeprom->awb_info.rgr_value_L,pCurrent_eeprom->awb_info.bgr_value_H,pCurrent_eeprom->awb_info.bgr_value_L,pCurrent_eeprom->awb_info.gbgr_value_H,pCurrent_eeprom->awb_info.gbgr_value_L); 
        pCurrent_eeprom->awb_info.golden_rgr_value_H=eeprom_value[25];
        pCurrent_eeprom->awb_info.golden_rgr_value_L=eeprom_value[26];
        pCurrent_eeprom->awb_info.golden_bgr_value_H=eeprom_value[27];
        pCurrent_eeprom->awb_info.golden_bgr_value_L=eeprom_value[28];
        pCurrent_eeprom->awb_info.golden_gbgr_value_H=eeprom_value[29];
        pCurrent_eeprom->awb_info.golden_gbgr_value_L=eeprom_value[30];
        //TRACE("ov13850r2a_read_cmos_sensor, golden_rgr_value_H = 0x%x, golden_rgr_value_L:0x%x, golden_bgr_value_H:0x%x, golden_bgr_value_L:0x%x, golden_gbgr_value_H:0x%x, golden_gbgr_value_L:0x%x,\n",pCurrent_eeprom->awb_info.golden_rgr_value_H,pCurrent_eeprom->awb_info.golden_rgr_value_L,pCurrent_eeprom->awb_info.golden_bgr_value_H,pCurrent_eeprom->awb_info.golden_bgr_value_L,pCurrent_eeprom->awb_info.golden_gbgr_value_H,pCurrent_eeprom->awb_info.golden_gbgr_value_L); 

        pCurrent_eeprom->awb_info.R_value_H=eeprom_value[31];
        pCurrent_eeprom->awb_info.R_value_L=eeprom_value[32];
        pCurrent_eeprom->awb_info.B_value_H=eeprom_value[33];
        pCurrent_eeprom->awb_info.B_value_L=eeprom_value[34];
        pCurrent_eeprom->awb_info.Gr_value_H=eeprom_value[35];
        pCurrent_eeprom->awb_info.Gr_value_L=eeprom_value[36];
        pCurrent_eeprom->awb_info.Gb_value_H=eeprom_value[37];
        pCurrent_eeprom->awb_info.Gb_value_L=eeprom_value[38];
        //TRACE("ov13850r2a_read_cmos_sensor, R_value_H = 0x%x, R_value_L:0x%x, B_value_H:0x%x, B_value_L:0x%x, Gb_value_H:0x%x, Gb_value_L:0x%x, Gr_value_H:0x%x, Gr_value_L:0x%x,\n",pCurrent_eeprom->awb_info.R_value_H,pCurrent_eeprom->awb_info.R_value_L,pCurrent_eeprom->awb_info.B_value_H,pCurrent_eeprom->awb_info.B_value_L,pCurrent_eeprom->awb_info.Gb_value_H,pCurrent_eeprom->awb_info.Gb_value_L,pCurrent_eeprom->awb_info.Gr_value_H,pCurrent_eeprom->awb_info.Gr_value_L); 
        pCurrent_eeprom->awb_info.golden_R_value_H=eeprom_value[39];
        pCurrent_eeprom->awb_info.golden_R_value_L=eeprom_value[40];
        pCurrent_eeprom->awb_info.golden_B_value_H=eeprom_value[41];
        pCurrent_eeprom->awb_info.golden_B_value_L=eeprom_value[42];
        pCurrent_eeprom->awb_info.golden_Gr_value_H=eeprom_value[43];
        pCurrent_eeprom->awb_info.golden_Gr_value_L=eeprom_value[44];
        pCurrent_eeprom->awb_info.golden_Gb_value_H=eeprom_value[45];
        pCurrent_eeprom->awb_info.golden_Gb_value_L=eeprom_value[46];
        //TRACE("ov13850r2a_read_cmos_sensor, golden_R_value_H = 0x%x, golden_R_value_L:0x%x, golden_B_value_H:0x%x, golden_B_value_L:0x%x, golden_Gr_value_H:0x%x, golden_Gr_value_L:0x%x, golden_Gb_value_H:0x%x, golden_Gb_value_L:0x%x,\n",pCurrent_eeprom->awb_info.golden_R_value_H,pCurrent_eeprom->awb_info.golden_R_value_L,pCurrent_eeprom->awb_info.golden_B_value_H,pCurrent_eeprom->awb_info.golden_B_value_L,pCurrent_eeprom->awb_info.golden_Gr_value_H,pCurrent_eeprom->awb_info.golden_Gr_value_L,pCurrent_eeprom->awb_info.golden_Gb_value_H,pCurrent_eeprom->awb_info.golden_Gb_value_L); 

        for(j = 0;j<360;j++)
        {
            pCurrent_eeprom->lenc[j] = eeprom_value[47+j];
            
        }  
        pCurrent_eeprom->AF_calibration_direction=eeprom_value[407];
        pCurrent_eeprom->AF_Inf_H=eeprom_value[408];
        pCurrent_eeprom->AF_Inf_L=eeprom_value[409];
        pCurrent_eeprom->AF_Macro_H=eeprom_value[410];
        pCurrent_eeprom->AF_Macro_L=eeprom_value[411];
        pCurrent_eeprom->AF_start_H=eeprom_value[412];
        pCurrent_eeprom->AF_start_L=eeprom_value[413];
        //lenovo.sw wangsx3 camera_calibration_cam_cal_ov13850mipiraw:
        //AF_Inf = (OTPBuf[1] << 2) | (OTPBuf[2] >> 6);
        //AF_Macro = (OTPBuf[3] << 2) | (OTPBuf[4] >> 6);
        ov13850_otp_af.af_data[0]=pCurrent_eeprom->AF_calibration_direction;
        ov13850_otp_af.af_data[1]=pCurrent_eeprom->AF_Inf_H;
        ov13850_otp_af.af_data[2]=pCurrent_eeprom->AF_Inf_L;
        ov13850_otp_af.af_data[3]=pCurrent_eeprom->AF_Macro_H;
        ov13850_otp_af.af_data[4]=pCurrent_eeprom->AF_Macro_L;
        ov13850_otp_af.af_data[5]=pCurrent_eeprom->AF_start_H;
        ov13850_otp_af.af_data[6]=pCurrent_eeprom->AF_start_L;
        ov13850_otp_af.af_valid = 1; //AF OTP DATA could be un-available for some module
        TRACE("ov13850r2a_read_cmos_sensor,AF_calibration_direction = 0x%x, AF_Inf_H = 0x%x, AF_Inf_L:0x%x, AF_Macro_H:0x%x, AF_Macro_L:0x%x, AF_start_H:0x%x, AF_start_L:0x%x \n",pCurrent_eeprom->AF_calibration_direction,pCurrent_eeprom->AF_Inf_H,pCurrent_eeprom->AF_Inf_L,pCurrent_eeprom->AF_Macro_H,pCurrent_eeprom->AF_Macro_L,pCurrent_eeprom->AF_start_H,pCurrent_eeprom->AF_start_L); 
        
        TRACE("ov13850r2a_read_cmos_sensor success \n");
        return true;
      }
      else
      {
        TRACE("ov13850r2a_read_cmos_sensor fail:check sum error \n");
        return false;
      }       
    }      
    else
    {
      TRACE("ov13850r2a_read_cmos_sensor fail:flag error \n");
      return false;
    }
       
}

static void ov13850r2a_update_awb_gain(kal_uint32 R_gain, kal_uint32 G_gain, kal_uint32 B_gain)
{
  //lenovo.sw wangsx3 enable MWB,BLC 
  ov13850r2a_write_cmos_sensor(OTP_WRITE_ID,0x5001, 0x03);
  if (R_gain>=0x400) 
  {
    ov13850r2a_write_cmos_sensor(OTP_WRITE_ID,0x5056, (R_gain>>8)&0x0F);
    ov13850r2a_write_cmos_sensor(OTP_WRITE_ID,0x5057, R_gain & 0x00ff);
  }

  if (G_gain>=0x400)
  {
    ov13850r2a_write_cmos_sensor(OTP_WRITE_ID,0x5058, (G_gain>>8)&0x0F);
    ov13850r2a_write_cmos_sensor(OTP_WRITE_ID,0x5059, G_gain & 0x00ff);
  }

  if (B_gain>=0x400) 
  {
    ov13850r2a_write_cmos_sensor(OTP_WRITE_ID,0x505A, (B_gain>>8)&0x0F);
    ov13850r2a_write_cmos_sensor(OTP_WRITE_ID,0x505B, B_gain & 0x00ff);
  }
  #ifdef Ov13850_OTP_DEBUG  //lenovo.sw wangsx3 remove OTP debug log
  TRACE("update_awb_gain, 0x5056 = %x\n", ov13850r2a_read_cmos_sensor(OTP_WRITE_ID,0x5056));
  TRACE("update_awb_gain, 0x5057 = %x\n", ov13850r2a_read_cmos_sensor(OTP_WRITE_ID,0x5057));
  TRACE("update_awb_gain, 0x5058 = %x\n", ov13850r2a_read_cmos_sensor(OTP_WRITE_ID,0x5058));
  TRACE("update_awb_gain, 0x5059 = %x\n", ov13850r2a_read_cmos_sensor(OTP_WRITE_ID,0x5059));
  TRACE("update_awb_gain, 0x505A = %x\n", ov13850r2a_read_cmos_sensor(OTP_WRITE_ID,0x505A));
  TRACE("update_awb_gain, 0x505B = %x\n", ov13850r2a_read_cmos_sensor(OTP_WRITE_ID,0x505B));
  TRACE("update_awb_gain, 0x5001 = %x\n", ov13850r2a_read_cmos_sensor(OTP_WRITE_ID,0x5001));
  #endif
}

static int ov13850r2a_update_lenc(eeprom_struct*eeprom_ptr)
{
  kal_uint8 nSlaveId = 0x20;
  int i;
  kal_uint16 temp;
  temp =ov13850r2a_read_cmos_sensor(OTP_WRITE_ID,0x5000);
  temp= 0x01|temp;
  ov13850r2a_write_cmos_sensor(OTP_WRITE_ID,0x5000,temp);
  for(i=0;i<360;i++)
  {
      ov13850r2a_write_cmos_sensor(OTP_WRITE_ID,0x5200+i,(*eeprom_ptr).lenc[i]);
  }
  #ifdef Ov13850_OTP_DEBUG
  //dump lenc log
  for(i=0;i<360;i++)
  {
     TRACE("ov13850r2a_update_lenc read lens data, address 0x%x = %x\n",(0x5200+i), ov13850r2a_read_cmos_sensor(OTP_WRITE_ID,0x5200+i));
    //  ov13850r2a_read_cmos_sensor(OTP_WRITE_ID,0x5200+i,(*eeprom_ptr).lenc[i]);
  }
  #endif
  return 0;
}
int ov13850ofilmr2a_update_eeprom()
{
    kal_uint32 rgr,bgr,gbgr;
    kal_uint32 R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
    
    eeprom_struct current_eeprom;
    if(!OV13850r2a_read_eeprom(&current_eeprom))
    {
      TRACE("ov13850r2a_read_cmos_sensor error \n");
      return 0;
    }
    //TRACE("ov13850r2a_update_eeprom, current_eeprom.awb_info.rgr_value_H<<2 : 0x%x,\n", current_eeprom.awb_info.rgr_value_H<<2);
    //TRACE("ov13850r2a_update_eeprom, current_eeprom.awb_info.rgr_value_L>>6 : 0x%x,\n", current_eeprom.awb_info.rgr_value_L>>6);
    //TRACE("ov13850r2a_update_eeprom, current_eeprom.awb_info.bgr_value_H<<2 : 0x%x,\n", current_eeprom.awb_info.bgr_value_H<<2);
    //TRACE("ov13850r2a_update_eeprom, current_eeprom.awb_info.bgr_value_L>>6 : 0x%x,\n", current_eeprom.awb_info.bgr_value_L>>6);
    //TRACE("ov13850r2a_update_eeprom, current_eeprom.awb_info.gbgr_value_H<<2 : 0x%x,\n", current_eeprom.awb_info.gbgr_value_H<<2);
    //TRACE("ov13850r2a_update_eeprom, current_eeprom.awb_info.gbgr_value_L>>6 : 0x%x,\n", current_eeprom.awb_info.gbgr_value_L>>6);

    rgr=(current_eeprom.awb_info.rgr_value_H<<2)+(current_eeprom.awb_info.rgr_value_L>>6);
    bgr=(current_eeprom.awb_info.bgr_value_H<<2)+(current_eeprom.awb_info.bgr_value_L>>6);
    gbgr=(current_eeprom.awb_info.gbgr_value_H<<2)+(current_eeprom.awb_info.gbgr_value_L>>6);

    TRACE("ov13850r2a_update_eeprom_wb, r/gr:0x%x, b/gr:0x%x, gb/gr:0x%x\n", rgr, bgr,gbgr);
    
    //calculate G gain
    //0x400 = 1x gain
  //if(( bgr != 0 )&& ( rgr != 0 ))
  {
    if(bgr< BGr_Ratio_Typical_EEPROM) 
    {
        if (rgr< RGr_Ratio_Typical_EEPROM )
        {
            G_gain = 0x400;
            B_gain = 0x400 * BGr_Ratio_Typical_EEPROM/ bgr;
            R_gain = 0x400 * RGr_Ratio_Typical_EEPROM/ rgr; 
        }
        else 
        {
            R_gain = 0x400;
            G_gain = 0x400 * rgr/ RGr_Ratio_Typical_EEPROM;
            B_gain = G_gain * BGr_Ratio_Typical_EEPROM/bgr;
        }
    }
    else 
    {
        if (rgr< RGr_Ratio_Typical_EEPROM)
        {
            B_gain = 0x400;
            G_gain = 0x400 * bgr/ BGr_Ratio_Typical_EEPROM;
            R_gain = G_gain * RGr_Ratio_Typical_EEPROM/ rgr;
        }
        else 
        {
            G_gain_B = 0x400 * bgr/ BGr_Ratio_Typical_EEPROM;
            G_gain_R = 0x400 * rgr/ RGr_Ratio_Typical_EEPROM;
            if(G_gain_B > G_gain_R ) 
            {
                B_gain = 0x400;
                G_gain = G_gain_B;
                R_gain = G_gain * RGr_Ratio_Typical_EEPROM/rgr;
            }
            else 
            {
                R_gain = 0x400;
                G_gain = G_gain_R;
                B_gain = G_gain * BGr_Ratio_Typical_EEPROM/ bgr;
            }
        }    
    }
 } //end if(( bgr != 0 )&& ( rgr != 0 ))
    TRACE("OV16825_Upate_Otp_WB, R_gain:0x%x, G_gain:0x%x, B_gain:0x%x\n", R_gain, G_gain, B_gain);

    ov13850r2a_update_awb_gain(R_gain, G_gain, B_gain);

    ov13850r2a_update_lenc(&current_eeprom);

// ov13850r2a_dump_reg();
    return 1;

}
/*lenovo.sw START wangsx3 add for mid check */
bool ov13850ofilm_otp_check_mid(void)
{
  u32 mid = 0x00;
  u32 OTP_SENSOR_MID=0,VCM_ID=0;
  TRACE("ov13850ofilm_otp_check_mid start\n");    
  OTP_SENSOR_MID = (u32)ov13850r2a_read_cmos_sensor(OTP_READ_ID,0x0401);
  VCM_ID= (u32)ov13850r2a_read_cmos_sensor(OTP_READ_ID,0x040A);
  TRACE("%s OFILM MID=0x%02x VCM_ID=%d \r\n",__func__,OTP_SENSOR_MID,VCM_ID);

  if ((OTP_SENSOR_MID==0x07)  && (VCM_ID==0x08))//sunnny =0x01,,OFLIM=0x07,VCM_ID :OFILM A40=0x08,OFILM A50=0x04,
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}
/*lenovo.sw START wangsx3 add for AF OTP */
kal_uint8 ov13850ofilm_cam_cal_read_data(kal_uint32 offset,
	kal_uint32 lenth, kal_uint8 *pBuf)
{
	//return accual read lenth
	kal_uint32 i;

	TRACE("Enter %s,offset=0x%x,len=%d,pBuf=0x%x\n",
		__func__,offset,lenth,pBuf);



	//down_interruptible(&imx219_otp_sem);
	#if 0
	if(imx219_otp_module.otp_valid != 1){
		goto cal_err;
	}
  #endif
#if 0
	if (offset == 0x00){
		//read module id
		if (lenth < 2){
			goto cal_err;
		} else {
			pBuf[0]= 0x02;
			pBuf[1]= 0x19;
			//up(&imx219_otp_sem);
			return 2;
		}
	}

	if (offset == 0x5a10){
		//read af
		if ( (lenth < OV13850_AF_DATA_CNT) || (ov13850_otp_af.af_valid != 1)){
			goto cal_err;
		} else {
			for (i=0; i< OV13850_AF_DATA_CNT; i++){
				pBuf[i] = ov13850_otp_af.af_data[i];
			}
			//up(&imx219_otp_sem);
			return OV13850_AF_DATA_CNT;
		}
	}
#endif
			for (i=0; i< OV13850_AF_DATA_CNT; i++){
				pBuf[i] = ov13850_otp_af.af_data[i];
			}
cal_err:
	//up(&imx219_otp_sem);
	return OV13850_AF_DATA_CNT;
}
