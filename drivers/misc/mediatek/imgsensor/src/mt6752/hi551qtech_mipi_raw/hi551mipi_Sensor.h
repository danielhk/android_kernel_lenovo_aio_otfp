/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 HI551mipi_Sensor.h
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _HI551MIPI_SENSOR_H
#define _HI551MIPI_SENSOR_H


typedef enum{
    IMGSENSOR_MODE_INIT,
    IMGSENSOR_MODE_PREVIEW,
    IMGSENSOR_MODE_CAPTURE,
    IMGSENSOR_MODE_VIDEO,
    IMGSENSOR_MODE_HIGH_SPEED_VIDEO,
    IMGSENSOR_MODE_SLIM_VIDEO,
} IMGSENSOR_MODE;

typedef struct imgsensor_mode_struct {
	kal_uint32 pclk;				//record different mode's pclk
	kal_uint32 linelength;			//record different mode's linelength
	kal_uint32 framelength;			//record different mode's framelength

	kal_uint8 startx;				//record different mode's startx of grabwindow
	kal_uint8 starty;				//record different mode's startx of grabwindow

	kal_uint16 grabwindow_width;	//record different mode's width of grabwindow
	kal_uint16 grabwindow_height;	//record different mode's height of grabwindow

	/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
	kal_uint8 mipi_data_lp2hs_settle_dc;

	/*	 following for GetDefaultFramerateByScenario()	*/
	kal_uint16 max_framerate;
	
} imgsensor_mode_struct;

/* SENSOR PRIVATE STRUCT FOR VARIABLES*/
typedef struct imgsensor_struct {
	kal_uint8 mirror;				//mirrorflip information

	kal_uint8 sensor_mode;			//record IMGSENSOR_MODE enum value

	kal_uint32 shutter;				//current shutter
	kal_uint16 gain;				//current gain
	
	kal_uint32 pclk;				//current pclk

	kal_uint32 frame_length;		//current framelength
	kal_uint32 line_length;			//current linelength

	kal_uint32 min_frame_length;	//current min  framelength to max framerate
	kal_uint16 dummy_pixel;			//current dummypixel
	kal_uint16 dummy_line;			//current dummline
	
	kal_uint16 current_fps;			//current max fps
	kal_bool   autoflicker_en;		//record autoflicker enable or disable
    kal_bool test_pattern;            //record test pattern mode or not
	MSDK_SCENARIO_ID_ENUM current_scenario_id;//current scenario id
	kal_uint8  ihdr_en;				//ihdr enable or disable
	
	kal_uint8 i2c_write_id;			//record current sensor's i2c write id
} imgsensor_struct;

/* SENSOR PRIVATE STRUCT FOR CONSTANT*/
typedef struct imgsensor_info_struct {
    kal_uint32 sensor_id;            //record sensor id defined in Kd_imgsensor.h
    kal_uint32 checksum_value;        //checksum value for Camera Auto Test
    imgsensor_mode_struct pre;        //preview scenario relative information
    imgsensor_mode_struct cap;        //capture scenario relative information
    imgsensor_mode_struct cap1;        //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
    imgsensor_mode_struct normal_video;//normal video  scenario relative information
    imgsensor_mode_struct hs_video;    //high speed video scenario relative information
    imgsensor_mode_struct slim_video;    //slim video for VT scenario relative information
    kal_uint8  ae_shut_delay_frame;    //shutter delay frame for AE cycle
    kal_uint8  ae_sensor_gain_delay_frame;    //sensor gain delay frame for AE cycle
    kal_uint8  ae_ispGain_delay_frame;    //isp gain delay frame for AE cycle
    kal_uint8  ihdr_support;        //1, support; 0,not support
    kal_uint8  ihdr_le_firstline;    //1,le first ; 0, se first
    kal_uint8  sensor_mode_num;        //support sensor mode num

    kal_uint8  cap_delay_frame;        //enter capture delay frame num
    kal_uint8  pre_delay_frame;        //enter preview delay frame num
    kal_uint8  video_delay_frame;    //enter video delay frame num
    kal_uint8  hs_video_delay_frame;    //enter high speed video  delay frame num
    kal_uint8  slim_video_delay_frame;    //enter slim video delay frame num
    kal_uint8  margin;                //sensor framelength & shutter margin
    kal_uint32 min_shutter;            //min shutter
    kal_uint32 max_frame_length;    //max framelength by sensor register's limitation

	kal_uint8  isp_driving_current;	//mclk driving current
	kal_uint8  sensor_interface_type;//sensor_interface_type
    kal_uint8  mipi_sensor_type; //0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2, default is NCSI2, don't modify this para
    kal_uint8  mipi_settle_delay_mode; //0, high speed signal auto detect; 1, use settle delay,unit is ns, default is auto detect, don't modify this para
	kal_uint8  sensor_output_dataformat;
	kal_uint8  mclk;				//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	
	kal_uint8  mipi_lane_num;		//mipi lane num
	kal_uint8  i2c_addr_table[5];	//record sensor support all write id addr, only supprt 4must end with 0xff
	kal_uint32  i2c_speed;     //i2c speed
} imgsensor_info_struct;

#define OTPBUFFER_SIZE 138
#define MODULE_LENGTH 16
#define AWB_LENGTH 29
#define AF_LENGTH 9

#define OTP_START_ADDR  0x1801
#define OTP_END_ADDR  0x1889

#define OTP_MODULE_ADDR  OTP_START_ADDR
#define OTP_AWB_ADDR 0x1832
#define MODULE_V1_LENGTH 16 
#define OTPV1_MODULE_GROUP_ONE       0x1802
#define OTPV1_MODULE_GROUP_TWO      0x1812
#define OTPV1_MODULE_GROUP_THREE   0x1822
#define OTPV1_AWB_FLAG_ADDR       0x1832
#define OTPV1_AWB_GROUP_ONE      0x1833
#define OTPV1_AWB_GROUP_TWO     0x1850
#define OTPV1_AWB_GROUP_THREE  0x186d

typedef struct 
{
    kal_uint8 mid;
    kal_uint8 otp_calibration_version;
    kal_uint8 product_year;
    kal_uint8 product_month;
    kal_uint8 product_day;
    kal_uint8 sensor_id;
    kal_uint8 lens_id;
    kal_uint8 vcm_id;
    kal_uint8 driver_ic_id;
    kal_uint8 ir_bg_id;
    kal_uint8 color_temperature;
    kal_uint8 af_ff_flag;
    kal_uint8 light_source_flag;
    kal_uint8 cam_position;
    kal_uint8 reserved;
} module_data_info;

typedef  struct 
{
    kal_uint8 rgr_value_H;
    kal_uint8 rgr_value_L;
    kal_uint8 bgr_value_H;
    kal_uint8 bgr_value_L;
    kal_uint8 gbgr_value_H;
    kal_uint8 gbgr_value_L;
    kal_uint8 golden_rgr_value_H;
    kal_uint8 golden_rgr_value_L;
    kal_uint8 golden_bgr_value_H;
    kal_uint8 golden_bgr_value_L;
    kal_uint8 golden_gbgr_value_H;
    kal_uint8 golden_gbgr_value_L;
    kal_uint8 R_value_H;
    kal_uint8 R_value_L;
    kal_uint8 B_value_H;
    kal_uint8 B_value_L;
    kal_uint8 Gr_value_H;
    kal_uint8 Gr_value_L;
    kal_uint8 Gb_value_H;
    kal_uint8 Gb_value_L;
    kal_uint8 golden_R_value_H;
    kal_uint8 golden_R_value_L;
    kal_uint8 golden_B_value_H;
    kal_uint8 golden_B_value_L;
    kal_uint8 golden_Gr_value_H;
    kal_uint8 golden_Gr_value_L;
    kal_uint8 golden_Gb_value_H;
    kal_uint8 golden_Gb_value_L;
} awb_data_info;

typedef struct 
{
	int module_flag;
	int checksum;
	int module_info_valid;
	int group;
	union {
		kal_uint8 module_data[MODULE_V1_LENGTH];
		module_data_info info;
	} data;
} module_info;

typedef  struct 
{
	int awb_flag;
	int checksum;
	int awb_info_valid;
	int group;
	union {
		kal_uint8 awb_data[AWB_LENGTH];
		awb_data_info info;
	} data;
} awb_info;

typedef struct 
{
    int     otp_valid_flag;
    int                  otp_version;
    module_info    module_i;
    awb_info    awb_i;
} hi551Otp_struct;



/* SENSOR READ/WRITE ID */
//#define IMGSENSOR_WRITE_ID_1 (0x6c)
//#define IMGSENSOR_READ_ID_1  (0x6d)
//#define IMGSENSOR_WRITE_ID_2 (0x20)
//#define IMGSENSOR_READ_ID_2  (0x21)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
extern void HI551OTP_ApplyWB(bool apply);
extern int HI551OTP_OTP_Check(void);
extern bool isHI551haveOTP(void);
extern int HI551_OTP_Ver_Check(void);
#endif 
