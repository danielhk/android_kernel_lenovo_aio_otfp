#ifndef _PRIMARY_DISPLAY_H_
#define _PRIMARY_DISPLAY_H_

/***************************/
#include "ddp_hal.h"
#include "ddp_manager.h"
#include <mach/mt_typedefs.h>
#include <linux/types.h>
#include "disp_lcm.h"
#include "disp_session.h"
#include "fbconfig_kdebug.h"
#include "disp_helper.h"
typedef enum {
	DIRECT_LINK_MODE,
	DECOUPLE_MODE,
	SINGLE_LAYER_MODE,
	DEBUG_RDMA1_DSI0_MODE
} DISP_PRIMARY_PATH_MODE;


/* --------------------------------------------------------------------------- */

#define DISP_RET_0_IF_NULL(x)			\
	do {									\
		if ((x) == NULL) {  \
			DISPERR("%s is NULL, return 0\n", #x);  \
			return 0; }  \
	} while (0)


#define DISP_RET_VOID_IF_NULL(x)			\
	do {								\
		if ((x) == NULL) {   \
			DISPERR("%s is NULL, return 0\n", #x);  \
			return; }  \
	} while (0)

#define DISP_RET_VOID_IF_0(x)			\
	do {								\
		if ((x) == 0) {  \
			DISPERR("%s is NULL, return 0\n", #x);  \
			return; } \
	} while (0)

#define DISP_RET_0_IF_0(x)			\
		do {								\
			if ((x) == 0) {  \
				DISPERR("%s is NULL, return 0\n", #x);  \
				return 0; }  \
		} while (0)

#define DISP_RET_NULL_IF_0(x)			\
	do {								\
		if ((x) == 0) {   \
			DISPERR("%s is NULL, return 0\n", #x);  \
			return NULL; }  \
	} while (0)  \


#define DISP_CHECK_RET(expr)                                                \
	do {                                                                    \
		DISP_STATUS ret = (expr);                                           \
		if (DISP_STATUS_OK != ret) {                                        \
			DISP_LOG_PRINT(ANDROID_LOG_ERROR, "COMMON",   \
			"[ERROR][mtkfb] DISP API return error code: 0x%x\n"      \
		   "  file : %s, line : %d\n"                               \
		   "  expr : %s\n", ret, __FILE__, __LINE__, #expr);        \
	}                                                                   \
	} while (0)


/* --------------------------------------------------------------------------- */

#define ASSERT_LAYER    (DDP_OVL_LAYER_MUN-1)
extern unsigned int FB_LAYER;	/* default LCD layer */
#define DISP_DEFAULT_UI_LAYER_ID (DDP_OVL_LAYER_MUN-1)
#define DISP_CHANGED_UI_LAYER_ID (DDP_OVL_LAYER_MUN-2)

typedef struct {
	unsigned int id;
	unsigned int curr_en;
	unsigned int next_en;
	unsigned int hw_en;
	int curr_idx;
	int next_idx;
	int hw_idx;
	int curr_identity;
	int next_identity;
	int hw_identity;
	int curr_conn_type;
	int next_conn_type;
	int hw_conn_type;
} DISP_LAYER_INFO;

typedef enum {
	DISP_STATUS_OK = 0,

	DISP_STATUS_NOT_IMPLEMENTED,
	DISP_STATUS_ALREADY_SET,
	DISP_STATUS_ERROR,
} DISP_STATUS;


typedef enum {
	DISP_STATE_IDLE = 0,
	DISP_STATE_BUSY,
} DISP_STATE;

typedef enum {
	DISP_OP_PRE = 0,
	DISP_OP_NORMAL,
	DISP_OP_POST,
} DISP_OP_STATE;

typedef enum {
	DISP_ALIVE = 0xf0,
	DISP_SLEEPED
} DISP_POWER_STATE;

typedef enum {
	FRM_CONFIG = 0,
	FRM_TRIGGER,
	FRM_START,
	FRM_END
} DISP_FRM_SEQ_STATE;


typedef enum {
	DISPLAY_HAL_IOCTL_SET_CMDQ = 0xff00,
	DISPLAY_HAL_IOCTL_ENABLE_CMDQ,
	DISPLAY_HAL_IOCTL_DUMP,
	DISPLAY_HAL_IOCTL_PATTERN,
} DISPLAY_HAL_IOCTL;


typedef struct {
	unsigned int layer;
	unsigned int layer_en;
	unsigned int buff_source;
	unsigned int fmt;
	unsigned long addr;
	unsigned long addr_sub_u;
	unsigned long addr_sub_v;
	unsigned long vaddr;
	unsigned int src_x;
	unsigned int src_y;
	unsigned int src_w;
	unsigned int src_h;
	unsigned int src_pitch;
	unsigned int dst_x;
	unsigned int dst_y;
	unsigned int dst_w;
	unsigned int dst_h;	/* clip region */
	unsigned int keyEn;
	unsigned int key;
	unsigned int aen;
	unsigned char alpha;

	unsigned int sur_aen;
	unsigned int src_alpha;
	unsigned int dst_alpha;


	unsigned int isTdshp;
	unsigned int isDirty;

	unsigned int buff_idx;
	unsigned int identity;
	unsigned int connected_type;
	DISP_BUFFER_TYPE security;
	unsigned int dirty;
	unsigned int yuv_range;
} primary_disp_input_config;

typedef struct {
	unsigned int fmt;
	unsigned long addr;
	unsigned long addr_sub_u;
	unsigned long addr_sub_v;
	unsigned long vaddr;
	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;
	unsigned int pitch;
	unsigned int pitchUV;

	unsigned int buff_idx;
	unsigned int interface_idx;
	DISP_BUFFER_TYPE security;
	unsigned int dirty;
	int mode;
} disp_mem_output_config;

#define DISP_INTERNAL_BUFFER_COUNT 3

typedef struct {
	struct list_head list;
	struct ion_handle *handle;
	struct sync_fence *pfence;
	void *va;
	uint32_t fence_id;
	uint32_t mva;
	uint32_t size;
	uint32_t output_fence_id;
	uint32_t interface_fence_id;
	unsigned long long timestamp;
} disp_internal_buffer_info;

typedef struct {
	unsigned int mva;
	unsigned int max_offset;
	unsigned int seq;
	DISP_FRM_SEQ_STATE state;
} disp_frm_seq_info;


typedef struct {
	DISP_POWER_STATE state;
	unsigned int lcm_fps;
	int max_layer;
	int need_trigger_overlay;
	int need_trigger_ovl1to2;
	int need_trigger_dcMirror_out;
	DISP_PRIMARY_PATH_MODE mode;
	int session_mode;
	unsigned int session_id;
	unsigned int last_vsync_tick;
	unsigned long framebuffer_mva;
	unsigned long framebuffer_va;
	struct mutex lock;
	struct mutex capture_lock;
	struct mutex vsync_lock;
#ifdef DISP_SWITCH_DST_MODE
	struct mutex switch_dst_lock;
#endif
	struct mutex cmd_lock;
	disp_lcm_handle *plcm;
	cmdqRecHandle cmdq_handle_config_esd;

	cmdqRecHandle cmdq_handle_trigger;

	cmdqRecHandle cmdq_handle_config;
	disp_path_handle dpmgr_handle;

	cmdqRecHandle cmdq_dummy_trigger;

	cmdqRecHandle cmdq_handle_ovl1to2_config;
	disp_path_handle ovl2mem_path_handle;

	char *mutex_locker;
	int vsync_drop;
	struct mutex dc_lock;
	struct list_head dc_free_list;
	struct list_head dc_reading_list;
	struct list_head dc_writing_list;
	unsigned int dc_buf_id;
	unsigned int dc_buf[DISP_INTERNAL_BUFFER_COUNT];
	cmdqBackupSlotHandle cur_config_fence;
	cmdqBackupSlotHandle subtractor_when_free;

	cmdqBackupSlotHandle rdma_buff_info;
	cmdqBackupSlotHandle ovl_status_info;
	int is_primary_sec;

#ifdef DISP_DUMP_EVENT_STATUS
	cmdqBackupSlotHandle event_status;
#endif
} display_primary_path_context;

struct sec_session_node {
	struct list_head link;
	unsigned int tgid;
};

typedef int (*PRIMARY_DISPLAY_CALLBACK) (unsigned int user_data);

int primary_display_init(char *lcm_name, unsigned int lcm_fps);
int primary_display_config(unsigned long pa, unsigned long mva);

int primary_display_set_frame_buffer_address(unsigned long va, unsigned long mva);
unsigned long primary_display_get_frame_buffer_mva_address(void);
unsigned long primary_display_get_frame_buffer_va_address(void);
unsigned long get_dim_layer_mva_addr(void);
int is_dim_layer(unsigned int long mva);

int primary_display_suspend(void);
int primary_display_resume(void);
int primary_display_ipoh_restore(void);
int primary_display_ipoh_recover(void);
int primary_display_get_width(void);
int primary_display_get_height(void);
int primary_display_get_bpp(void);
int primary_display_get_pages(void);

int primary_display_set_overlay_layer(primary_disp_input_config *input);
int primary_display_is_alive(void);
int primary_display_is_sleepd(void);
int primary_display_wait_for_vsync(void *config);
unsigned int primary_display_get_ticket(void);
int primary_display_config_input(primary_disp_input_config *input);
int primary_display_user_cmd(unsigned int cmd, unsigned long arg);
int primary_display_trigger(int blocking, void *callback, unsigned int userdata);
int primary_display_config_output(disp_mem_output_config *output);
int primary_display_mem_out_trigger(int blocking, void *callback, unsigned int userdata);
int primary_display_switch_mode(int sess_mode, unsigned int session, int force);
int primary_display_diagnose(void);

int primary_display_get_info(void *info);
int primary_display_capture_framebuffer(unsigned long pbuf);
int primary_display_capture_framebuffer_ovl(unsigned long pbuf, unsigned int format);
UINT32 DISP_GetVRamSizeBoot(char *cmdline);
UINT32 DISP_GetVRamSize(void);
UINT32 DISP_GetFBRamSize(void);
UINT32 DISP_GetPages(void);
UINT32 DISP_GetScreenBpp(void);
UINT32 DISP_GetScreenWidth(void);
UINT32 DISP_GetScreenHeight(void);
UINT32 DISP_GetActiveHeight(void);
UINT32 DISP_GetActiveWidth(void);
int disp_hal_allocate_framebuffer(phys_addr_t pa_start, phys_addr_t pa_end, unsigned long *va, unsigned long *mva);
int primary_display_is_video_mode(void);
int primary_display_is_decouple_mode(void);
int primary_display_is_mirror_mode(void);
unsigned int primary_display_get_option(const char *option);
CMDQ_SWITCH primary_display_cmdq_enabled(void);
int primary_display_switch_cmdq_cpu(CMDQ_SWITCH use_cmdq);
int primary_display_check_path(char *stringbuf, int buf_len);
int primary_display_manual_lock(void);
int primary_display_manual_unlock(void);
int primary_display_start(void);
int primary_display_stop(void);
int primary_display_esd_recovery(void);
int primary_display_get_debug_state(char *stringbuf, int buf_len);
void primary_display_set_max_layer(int maxlayer);
void primary_display_reset(void);
void primary_display_esd_check_enable(int enable);
LCM_PARAMS *DISP_GetLcmPara(void);
LCM_DRIVER *DISP_GetLcmDrv(void);
int Panel_Master_dsi_config_entry(const char *name, void *config_value);
int primary_display_config_input_multiple(disp_session_input_config *session_input);
int primary_display_config_interface_input(primary_disp_input_config *input);
int primary_display_force_set_vsync_fps(unsigned int fps);
unsigned int primary_display_get_fps(void);
int primary_display_get_original_width(void);
int primary_display_get_original_height(void);
int primary_display_enable_path_cg(int enable);
int primary_display_lcm_ATA(void);
int primary_display_setbacklight(unsigned int level);
//lenovo jixu add for custom cabc
#ifdef CONFIG_LENOVO_CUSTOM_LCM_FEATURE
int primary_display_setcabc(unsigned int mode);
int primary_display_setinverse(unsigned int mode);
#endif
//lenovo jixu add end
//lenovo wangyq13 add for sre 20150402
#ifdef CONFIG_LENOVO_SUPER_BACKLIGHT
int primary_display_setsre(unsigned int mode);
#endif
int fbconfig_get_esd_check_test(UINT32 dsi_id, UINT32 cmd, UINT8 *buffer, UINT32 num);
int primary_display_pause(PRIMARY_DISPLAY_CALLBACK callback, unsigned int user_data);
int primary_display_switch_dst_mode(int mode);
int primary_display_get_lcm_index(void);
int primary_display_set_cmd(int *lcm_cmd, unsigned int cmd_num);
int disp_fmt_to_hw_fmt(DISP_FORMAT src_fmt, unsigned int *hw_fmt, unsigned int *Bpp, unsigned int *bpp);

display_primary_path_context *primary_display_path_lock(const char *caller);
void primary_display_path_unlock(const char *caller);
int primary_display_switch_wdma_dump(int on);
extern void _cmdq_insert_wait_frame_done_token_mira(void *handle);
int primary_display_release_fence_fake(void);
#ifdef CONFIG_MTK_SEGMENT_TEST
int primary_display_check_test(void);
#endif
int primary_display_cmdq_set_reg(unsigned int addr, unsigned int val);
void primary_display_update_present_fence(unsigned int fence_idx);
void disp_set_sodi(unsigned int enable, void *cmdq_handle);
int primary_display_esd_cust_get(void);
int primary_display_diagnose(void);
int primary_display_is_video_mode(void);
void primary_display_reset_ovl_by_cmdq(unsigned int force);
/************************************/

extern int rdma_resolution_test(DISP_MODULE_ENUM module, unsigned int fw, unsigned fh);
extern unsigned int DSI_WaitForNotBusyTimeout(DISP_MODULE_ENUM module, cmdqRecHandle cmdq);
extern void DSI_ForceConfig(int forceconfig);
extern void disp_record_rdma_end_interval(unsigned int enable);
extern unsigned long long disp_get_rdma_max_interval(void);
extern unsigned long long disp_get_rdma_min_interval(void);
extern int DSI_test_force_te(DISP_MODULE_ENUM module);
extern int DSI_test_roi(int x, int y);
extern unsigned int ddp_ovl_get_cur_addr(bool rdma_mode, int layerid);
extern unsigned int ddp_wdma_get_cur_addr(void);

extern bool is_ipoh_bootup;

extern unsigned int gEnableMutexRisingEdge;
extern unsigned int _need_wait_esd_eof(void);
extern unsigned int _need_register_eint(void);
extern unsigned int _need_do_esd_check(void);
extern unsigned int disp_low_power_enlarge_blanking;
extern unsigned int disp_low_power_disable_ddp_clock;
extern unsigned int disp_low_power_disable_fence_thread;
extern unsigned int disp_low_power_remove_ovl;
extern unsigned int disp_low_power_lfr;
extern unsigned int gDumpClockStatus;
extern unsigned int gEnableSODIWhenIdle;
extern unsigned int gDisableIRQWhenIdle;
extern unsigned int gEnableIRQ;
extern void DSI_Set_LFR(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, unsigned int mode,
	unsigned int type, unsigned int enable, unsigned int skip_num);
extern unsigned int gEnableSWTrigger;
extern unsigned int gEnableMutexRisingEdge;
extern unsigned int gDisableSODIForTriggerLoop;
extern unsigned int gDumpConfigCMD;
extern unsigned int gEnableOVLStatusCheck;
extern int DSI_LFR_Status_Check(void);
extern void DSI_LFR_UPDATE(DISP_MODULE_ENUM module, cmdqRecHandle cmdq);
extern unsigned int gEnableCMDQProfile;
extern unsigned int gESDEnableSODI;
extern unsigned int gDumpESDCMD;
#ifdef DISP_ENABLE_LAYER_FRAME
extern unsigned int disp_color[8];
extern unsigned int gAddFrame;
extern int disp_draw_num(unsigned long va,	/* dst buf va */
			 unsigned int x,	/* x offset (pixel) */
			 unsigned int y,	/* y offset */
			 unsigned int pitch,	/* dst pitch */
			 DpColorFormat fmt,	/* dst buf format */
			 unsigned int number,	/* digit number */
			 unsigned int alpha_en, unsigned int color_argb);	/* background oqupa or not */
extern int m4u_dma_cache_flush_all(void);
#endif
extern unsigned int disp_set_pll_by_cmdq(unsigned int freq,
								 void *cmdq_handle);
extern unsigned int gChangeMMClock;
extern unsigned int gChangeRDMAThreshold;
extern unsigned int gRDMAUltraSetting_Decouple;
extern unsigned int gRDMAUltraSetting_Directlink;
extern unsigned int gRDMAUltraSetting;
extern unsigned int gRDMARequestThreshold;
extern unsigned int gIssueRequestThreshold;
extern int ddp_dsi_disable_irq_in_idle(unsigned int isIdle, void *cmdq_handle);

#ifdef CONFIG_OF
extern unsigned int vramsize;
/*unsigned int vramsize       = 0;*/
extern int _parse_tag_videolfb(void);
#endif
extern char *saved_command_line;
extern UINT32 DSI_dcs_read_lcm_reg_v2(DISP_MODULE_ENUM module, cmdqRecHandle cmdq,
						      UINT8 cmd, UINT8 *buffer, UINT8 buffer_size);
extern unsigned int gDumpMemoutCmdq;
extern int disp_get_session_number(void);
extern unsigned int gEnableIRQ;
extern unsigned int gDisableOVLTF;

extern unsigned int ddp_ovl_get_cur_addr(bool rdma_mode, int layerid);
extern int dfo_query(const char *s, unsigned long *v);
extern unsigned int gCMDQDump;
extern unsigned int gResetOVLInAALTrigger;
extern unsigned int gDecouplePQWithRDMA;
extern unsigned int gEnableSODIControl;
extern unsigned int gPrefetchControl;
extern unsigned int gSkipIdleDetect;
int primary_display_switch_mode_nolock(int sess_mode, unsigned int session, int force);
void _cmdq_insert_wait_frame_done_token_mira(void *handle);
static void _cmdq_flush_config_handle_mira(void *handle, int blocking);
#ifdef CONFIG_OF
extern unsigned int islcmconnected;
#endif
int _trigger_display_interface(int blocking, void *callback, unsigned int userdata);
extern void disp_m4u_tf_disable(void);
void disp_update_trigger_time(void);
/********mtkfb.c*************/

extern unsigned int vramsize;

extern unsigned int disp_running;
extern wait_queue_head_t disp_done_wq;
extern BOOL is_engine_in_suspend_mode;
extern BOOL is_lcm_in_suspend_mode;
#ifdef CONFIG_OF
extern int _parse_tag_videolfb(void);
#endif

extern char *saved_command_line;

int ddp_lcd_test_dpi(void);
extern DISP_HELPER_STAGE disp_helper_get_stage(void);
unsigned int mtkfb_fm_auto_test(void);
/*************disp_assert_layer.c****************/
extern struct mutex OverlaySettingMutex;
extern atomic_t OverlaySettingDirtyFlag;
extern atomic_t OverlaySettingApplied;
extern OVL_CONFIG_STRUCT cached_layer_config[DDP_OVL_LAYER_MUN];
extern struct semaphore sem_early_suspend;
extern unsigned int dprec_error_log_len;
/***********************mtkfb_vsync.c*************************/
#if defined(CONFIG_MTK_HDMI_SUPPORT)
extern void hdmi_waitVsync(void);
#endif
extern void mtkfb_waitVsync(void);
extern void mtkfb_disable_non_fb_layer(void);
#endif





