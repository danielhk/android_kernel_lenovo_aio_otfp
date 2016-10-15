
typedef struct 
{
    int mid;
    int sensor_verson;
    int otp_calibration_version;
    int dll_version;
    int product_year;
    int product_month;
    int product_day;
    int sensor_id;
    int lens_id;
    int vcm_id;
    int driver_ic_id;
    int ir_bg_id;
    int color_temperature;
    int af_ff_flag;
    int light_source_flag;
} module_info;

typedef  struct 
{
    int rgr_value_H;
    int rgr_value_L;
    int bgr_value_H;
    int bgr_value_L;
    int gbgr_value_H;
    int gbgr_value_L;
    int golden_rgr_value_H;
    int golden_rgr_value_L;
    int golden_bgr_value_H;
    int golden_bgr_value_L;
    int golden_gbgr_value_H;
    int golden_gbgr_value_L;
    int R_value_H;
    int R_value_L;
    int B_value_H;
    int B_value_L;
    int Gr_value_H;
    int Gr_value_L;
    int Gb_value_H;
    int Gb_value_L;
    int golden_R_value_H;
    int golden_R_value_L;
    int golden_B_value_H;
    int golden_B_value_L;
    int golden_Gr_value_H;
    int golden_Gr_value_L;
    int golden_Gb_value_H;
    int golden_Gb_value_L;
} awb_info;

typedef struct 
{
    module_info module_info;
    awb_info awb_info;
    char lenc[360];
    int AF_calibration_direction;
    int AF_Inf_H;
    int AF_Inf_L;
    int AF_Macro_H;
    int AF_Macro_L;
    int AF_start_H;
    int AF_start_L;
	
} eeprom_struct;
