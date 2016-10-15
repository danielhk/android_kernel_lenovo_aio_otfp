#ifndef __S3528_TOUCHPANEL_UPG_H__
#define __S3528_TOUCHPANEL_UPG_H__

/*lenovo liuyw2 2014-8-9. add AUTO_UPDATE*/
/*
 *	AUTO_UPDATE 0 //disable phone startup autoupdate
 *	AUTO_UPDATE 1 //enable phone startup autoupdate
*/
#define AUTO_UPDATE 1 // 0 close update-function

#define SYNA_SUPPORT_VENDOR2
#define SYNA_SUPPORT_VENDOR3
#define SYNA_SUPPORT_VENDOR4

#if AUTO_UPDATE
	//#define FW_IMAGE_NAME "synaptics/startup_fw_update.img"//if you need to auto_update,please close this
	#define FW_IMAGE_NAME "SynaImage.h"//lenovo-sw xuwen1 changed for update,if you need to auto_update,please open this 	
	#define FORCE_UPDATE  true  //if you need to auto_update,please choose true
	#define DO_STARTUP_FW_UPDATE
	#include "image/s3528_v1_014D0014.h"
	#define FW_ID_CURR_VENDOR1	FW_ID_VENDOR1
	#define FAMILY_QUERY_ID_VENDOR1	(FW_ID_CURR_VENDOR1 >> 24) 

	#ifdef SYNA_SUPPORT_VENDOR2
	#include "image/s3528_v2_04520005.h"//
	#define FW_ID_CURR_VENDOR2	FW_ID_VENDOR2
	#define FAMILY_QUERY_ID_VENDOR2 (FW_ID_CURR_VENDOR2 >> 24)
	#endif

	#ifdef SYNA_SUPPORT_VENDOR3
	#include "image/s3203_v3_02520111.h"//
	#define FW_ID_CURR_VENDOR3	FW_ID_VENDOR3
	#define FAMILY_QUERY_ID_VENDOR3 (FW_ID_CURR_VENDOR3 >> 24)
	#endif

	#ifdef SYNA_SUPPORT_VENDOR4
	#include "image/s3203_v4_03520022.h"//
	#define FW_ID_CURR_VENDOR4	FW_ID_VENDOR4
	#define FAMILY_QUERY_ID_VENDOR4 (FW_ID_CURR_VENDOR4 >> 24)
	#endif

	#define FWU_AUTO_UPDATE_CTRL_USERSPACE
	#define PATH_USERSPACE_CTRL	"/data/NeedUpgradeTPFW"
#else
	#define FW_IMAGE_NAME "synaptics/startup_fw_update.img"//if you need to auto_update,please close this
	//#define FW_IMAGE_NAME "SynaImage.h"//lenovo-sw xuwen1 changed for update,if you need to auto_update,please open this 
	#define FORCE_UPDATE  false  //if you need to auto_update,please choose true
#endif

#endif /* __S3528_TOUCHPANEL_UPG_H__ */

