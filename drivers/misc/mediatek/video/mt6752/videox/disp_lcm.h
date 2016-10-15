#ifndef _DISP_LCM_H_
#define _DISP_LCM_H_
#include "lcm_drv.h"

#define MAX_LCM_NUMBER	2

typedef struct {
	LCM_PARAMS *params;
	LCM_DRIVER *drv;
	LCM_INTERFACE_ID lcm_if_id;
	int module;
	int is_inited;
	unsigned int lcm_original_width;
	unsigned int lcm_original_height;
	int index;
	} disp_lcm_handle, *pdisp_lcm_handle;
disp_lcm_handle *disp_lcm_probe(char *plcm_name, LCM_INTERFACE_ID lcm_id);
int disp_lcm_init(disp_lcm_handle *plcm, int force);
LCM_PARAMS *disp_lcm_get_params(disp_lcm_handle *plcm);
LCM_INTERFACE_ID disp_lcm_get_interface_id(disp_lcm_handle *plcm);
int disp_lcm_update(disp_lcm_handle *plcm, int x, int y, int w, int h, int force);
int disp_lcm_esd_check(disp_lcm_handle *plcm);
int disp_lcm_esd_recover(disp_lcm_handle *plcm);
int disp_lcm_suspend(disp_lcm_handle *plcm);
int disp_lcm_resume(disp_lcm_handle *plcm);
int disp_lcm_set_backlight(disp_lcm_handle *plcm, int level);
int disp_lcm_read_fb(disp_lcm_handle *plcm);
int disp_lcm_ioctl(disp_lcm_handle *plcm, LCM_IOCTL ioctl, unsigned int arg);
int disp_lcm_is_video_mode(disp_lcm_handle *plcm);
int disp_lcm_is_inited(disp_lcm_handle *plcm);
unsigned int disp_lcm_ATA(disp_lcm_handle *plcm);
void *disp_lcm_switch_mode(disp_lcm_handle *plcm, int mode);
int disp_lcm_set_cmd(disp_lcm_handle *plcm, void *handle, int *lcm_cmd, unsigned int cmd_num);
/* these 2 variables are defined in mt65xx_lcm_list.c */
extern LCM_DRIVER *lcm_driver_list[];
extern unsigned int lcm_count;
//lenovo jixu add begin
#ifdef CONFIG_LENOVO_CUSTOM_LCM_FEATURE
int disp_lcm_set_cabc(disp_lcm_handle *plcm, unsigned int mode);
int disp_lcm_set_cabc_cmd(disp_lcm_handle *plcm,void* handle, unsigned int mode);
int disp_lcm_set_inverse(disp_lcm_handle *plcm, unsigned int mode);
#endif
//lenovo jixu add end
//lenovo wangyq13 add for sre 20150402
#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
int disp_lcm_set_sre(disp_lcm_handle *plcm,void* handle, unsigned int mode);
int disp_lcm_set_sre_video(disp_lcm_handle *plcm, unsigned int mode);
#endif

#endif
