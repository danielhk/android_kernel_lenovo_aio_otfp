#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <mach/mt_typedefs.h>


#include "kd_camera_hw.h"


#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"



/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(PFX fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   pr_debug(PFX fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif


#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4


extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);


u32 pinSetIdx = 0;//default main sensor
u32 pinSet[3][8] = {
                        //for main sensor
                     {  CAMERA_CMRST_PIN,
                        CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,              /* ON state */
                        GPIO_OUT_ZERO,             /* OFF state */
                        CAMERA_CMPDN_PIN,
                        CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for sub sensor
                     {  CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                   };

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

    int pwListIdx,pwIdx;
    BOOL sensorInPowerList = KAL_FALSE;

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }

    //power ON
    if (On) 
    {
        PK_DBG("kdCISModulePowerOn -on:currSensorName=%s\n",currSensorName);
        PK_DBG("kdCISModulePowerOn -on:pinSetIdx=%d\n",pinSetIdx);
		//mt_set_gpio_mode(GPIO_CAMERA_SUB_DVDD, 0);
		//mt_set_gpio_dir(GPIO_CAMERA_SUB_DVDD, GPIO_DIR_OUT);
		//mt_set_gpio_out(GPIO_CAMERA_SUB_DVDD, GPIO_OUT_ZERO);
		//mt_set_gpio_mode(CAMERA_MCLK1,0);
		//mt_set_gpio_dir(CAMERA_MCLK1, GPIO_DIR_OUT);
		//mt_set_gpio_out(CAMERA_MCLK1, GPIO_OUT_ZERO);	
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to DOVDD power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}	
		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K3M2_MIPI_RAW,currSensorName)))
		{
			PK_DBG("kdCISModulePowerOn get in ---  SENSOR_DRVNAME_S5K3M2_MIPI_RAW \n");
			
	

		    // enable VMC for MIPI switch power domain
			if(TRUE != hwPowerOn(MT6325_POWER_LDO_VMC, VOL_1800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable vmc power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
			mt_set_gpio_mode(CAMERA_MCLK1,1);
			mdelay(3);
			ISP_MCLK1_EN(TRUE);
			mdelay(1);

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);


			// VCAMD use external buck 
		    mt_set_gpio_mode(GPIO_CAMERA_MAIN_DVDD, 0);
		    mt_set_gpio_dir(GPIO_CAMERA_MAIN_DVDD, GPIO_DIR_OUT);
		    mt_set_gpio_out(GPIO_CAMERA_MAIN_DVDD, GPIO_OUT_ONE);			
			mdelay(2);
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to DOVDD power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
			// PWDN use GPIO33 
			if(pinSetIdx == 0)
			{
				mt_set_gpio_mode(GPIO_CAMERA_MAIN_PDN, 0);
				mt_set_gpio_dir(GPIO_CAMERA_MAIN_PDN, GPIO_DIR_OUT);
				mt_set_gpio_out(GPIO_CAMERA_MAIN_PDN, GPIO_OUT_ONE);				    
				mdelay(2);	
			}
			else
			{
				mt_set_gpio_mode(GPIO_CAMERA_MAIN_PDN, 0);
				mt_set_gpio_dir(GPIO_CAMERA_MAIN_PDN, GPIO_DIR_OUT);
				mt_set_gpio_out(GPIO_CAMERA_MAIN_PDN, GPIO_OUT_ZERO);				    
				mdelay(2);	
			}
			// MIPI switch
			mt_set_gpio_mode(GPIO_MIPI_SWITCH_EN, 0);
			mt_set_gpio_dir(GPIO_MIPI_SWITCH_EN, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_MIPI_SWITCH_EN, GPIO_OUT_ZERO); 	
			
			mt_set_gpio_mode(GPIO_MIPI_SWITCH_SEL, 0);
			mt_set_gpio_dir(GPIO_MIPI_SWITCH_SEL, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_MIPI_SWITCH_SEL, GPIO_OUT_ZERO);

			
		}    		

		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8865_MIPI_RAW,currSensorName)))
		{
			PK_DBG("kdCISModulePowerOn get in ---  SENSOR_DRVNAME_OV8865_MIPI_RAW pinSetIdx=%d\n",pinSetIdx);
			
	
						
			PK_DBG("kdCISModulePowerOn get in ---  zhangjiano 1111111111111 \n");
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);
			/*if(TRUE != hwPowerOn(MT6325_POWER_LDO_VCAMD, VOL_1200,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to DOVDD power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}*/
			// DVDD enable
		    mt_set_gpio_mode(GPIO_CAMERA_SUB_DVDD, 0);
		    mt_set_gpio_dir(GPIO_CAMERA_SUB_DVDD, GPIO_DIR_OUT);
		    mt_set_gpio_out(GPIO_CAMERA_SUB_DVDD, GPIO_OUT_ONE);				
				mdelay(2);		
			// PWDN use GPIO34 
			if(pinSetIdx == 1)
			{
				mt_set_gpio_mode(GPIO_CAMERA_SUB_PDN, 0);
				mt_set_gpio_dir(GPIO_CAMERA_SUB_PDN, GPIO_DIR_OUT);
				mt_set_gpio_out(GPIO_CAMERA_SUB_PDN, GPIO_OUT_ONE);				    
				mdelay(2);		
			}
			else
			{
				mt_set_gpio_mode(GPIO_CAMERA_SUB_PDN, 0);
				mt_set_gpio_dir(GPIO_CAMERA_SUB_PDN, GPIO_DIR_OUT);
				mt_set_gpio_out(GPIO_CAMERA_SUB_PDN, GPIO_OUT_ZERO);				    
				mdelay(2);		
			}
			mt_set_gpio_mode(CAMERA_MCLK1,1);
			mdelay(2);		
			ISP_MCLK1_EN(TRUE);
			mdelay(5);			
			
		}   		

		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2680_MIPI_RAW,currSensorName)))
		{
			PK_DBG("kdCISModulePowerOn get in ---  SENSOR_DRVNAME_OV2680_MIPI_RAW \n");
			
		#if 1
		    // enable VMC for MIPI switch power domain
			if(TRUE != hwPowerOn(MT6325_POWER_LDO_VMC, VOL_1800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable vmc power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}				

			if(TRUE != hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_2800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerOn(MT6325_POWER_LDO_VGP3, VOL_1800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to DOVDD power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(2);

			// MCLK mode wrong
			PK_DBG("mode1\n");			
			mt_set_gpio_mode(CAMERA_MCLK2, 1);

			// DVDD use GPIO115 
		   /* mt_set_gpio_mode(GPIO_CAMERA_MAIN2_DVDD, 0);
		    mt_set_gpio_dir(GPIO_CAMERA_MAIN2_DVDD, GPIO_DIR_OUT);
		   mt_set_gpio_out(GPIO_CAMERA_MAIN2_DVDD, GPIO_OUT_ONE);	*/			
			
			// PWDN use GPIO32 
		    mt_set_gpio_mode(GPIO_CAMERA_MAIN2_PDN, 0);
		    mt_set_gpio_dir(GPIO_CAMERA_MAIN2_PDN, GPIO_DIR_OUT);
		    mt_set_gpio_out(GPIO_CAMERA_MAIN2_PDN, GPIO_OUT_ONE);				    
			mdelay(2);		
			//ISP_MCLK1_EN(TRUE);
			ISP_MCLK2_EN(TRUE);
			//ISP_MCLK3_EN(TRUE);
			mdelay(2);


			// MIPI switch
			mt_set_gpio_mode(GPIO_MIPI_SWITCH_EN, 0);
			mt_set_gpio_dir(GPIO_MIPI_SWITCH_EN, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_MIPI_SWITCH_EN, GPIO_OUT_ZERO); 	

			
			mt_set_gpio_mode(GPIO_MIPI_SWITCH_SEL, 0);
			mt_set_gpio_dir(GPIO_MIPI_SWITCH_SEL, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_MIPI_SWITCH_SEL, GPIO_OUT_ZERO);
			mdelay(10);
			mt_set_gpio_out(GPIO_MIPI_SWITCH_SEL, GPIO_OUT_ONE);
			#endif

			//while(1);			
		}    		

    }
    else {//power OFF

	    PK_DBG("kdCISModulePowerOn -off:currSensorName=%s\n",currSensorName);
	    PK_DBG("kdCISModulePowerOn -off:pinSetIdx=%d\n",pinSetIdx);
		//mt_set_gpio_mode(CAMERA_MCLK1,0);
		//mt_set_gpio_dir(CAMERA_MCLK1, GPIO_DIR_OUT);
		//mt_set_gpio_out(CAMERA_MCLK1, GPIO_OUT_ZERO);	
		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K3M2_MIPI_RAW,currSensorName)))
		{
	
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_D2);
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}	
			ISP_MCLK1_EN(FALSE);
			mdelay(5);

			// PWDN use GPIO33 
		    mt_set_gpio_mode(GPIO_CAMERA_MAIN_PDN, 0);
		    mt_set_gpio_dir(GPIO_CAMERA_MAIN_PDN, GPIO_DIR_OUT);
		    mt_set_gpio_out(GPIO_CAMERA_MAIN_PDN, GPIO_OUT_ZERO);	

			// MIPI switch
			mt_set_gpio_mode(GPIO_MIPI_SWITCH_EN, 0);
			mt_set_gpio_dir(GPIO_MIPI_SWITCH_EN, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_MIPI_SWITCH_EN, GPIO_OUT_ONE); 		    

			if(TRUE != hwPowerDown(MT6325_POWER_LDO_VMC,mode_name))
			{
				 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VMC), power id = %d \n",MT6325_POWER_LDO_VMC);
				 goto _kdCISModulePowerOn_exit_;
			}

			
			// VCAMD use external buck 
			mt_set_gpio_mode(GPIO_CAMERA_MAIN_DVDD, 0);
			mt_set_gpio_dir(GPIO_CAMERA_MAIN_DVDD, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_CAMERA_MAIN_DVDD, GPIO_OUT_ZERO); 
			mdelay(2);
	            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2, mode_name)) {
	                PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_A2);
	                //return -EIO;
	                goto _kdCISModulePowerOn_exit_;
	            }	
#if 0
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name))
			{
				 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);
				 goto _kdCISModulePowerOn_exit_;
			}
#endif

            //VCAM_A
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

                        
		}


		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8865_MIPI_RAW,currSensorName)))
		{
			PK_DBG("kdCISModulePowerOn get in ---  SENSOR_DRVNAME_OV8865_MIPI_RAW \n");
			
					
			ISP_MCLK1_EN(FALSE);
			mdelay(5);		

			// PWDN use GPIO34 
		    mt_set_gpio_mode(GPIO_CAMERA_SUB_PDN, 0);
		    mt_set_gpio_dir(GPIO_CAMERA_SUB_PDN, GPIO_DIR_OUT);
		    mt_set_gpio_out(GPIO_CAMERA_SUB_PDN, GPIO_OUT_ZERO);				    
			mdelay(2);				
		
			// DVDD enable
		  mt_set_gpio_mode(GPIO_CAMERA_SUB_DVDD, 0);
		    mt_set_gpio_dir(GPIO_CAMERA_SUB_DVDD, GPIO_DIR_OUT);
		    mt_set_gpio_out(GPIO_CAMERA_SUB_DVDD, GPIO_OUT_ZERO);
			mdelay(2);				
            /*if(TRUE != hwPowerDown(MT6325_POWER_LDO_VCAMD,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }*/
	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name)) {
		PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_D2);
		//return -EIO;
		goto _kdCISModulePowerOn_exit_;
	}	
            //VCAM_A
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_IO
			
			mdelay(2);
			
		}   	

	    if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2680_MIPI_RAW,currSensorName)))
     	{
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_D2);
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}	
			#if 1
			if(TRUE != hwPowerDown(MT6325_POWER_LDO_VMC,mode_name))
			{
				 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VMC), power id = %d \n",MT6325_POWER_LDO_VMC);
				 goto _kdCISModulePowerOn_exit_;
			}

     		
			ISP_MCLK2_EN(FALSE);
			mdelay(2);
			
			// PWDN use GPIO32 
		    mt_set_gpio_mode(GPIO_CAMERA_MAIN2_PDN, 0);
		    mt_set_gpio_dir(GPIO_CAMERA_MAIN2_PDN, GPIO_DIR_OUT);
		    mt_set_gpio_out(GPIO_CAMERA_MAIN2_PDN, GPIO_OUT_ZERO);				    
			mdelay(2);	
			#endif
     	}	
	    
    }//

	return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);


//!--
//




