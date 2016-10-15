#ifndef __DDP_DEBUG_H__
#define __DDP_DEBUG_H__

#include <linux/kernel.h>
#include "ddp_mmp.h"
#include "ddp_dump.h"

#define DISP_ENABLE_SODI_FOR_VIDEO_MODE
void ddp_debug_init(void);
void ddp_debug_exit(void);

unsigned int  ddp_debug_analysis_to_buffer(void);
unsigned int  ddp_debug_dbg_log_level(void);
unsigned int  ddp_debug_irq_log_level(void);
unsigned int ddp_dump_reg_to_buf(unsigned int start_module, unsigned long *addr);
extern unsigned int g_mobilelog;
extern unsigned int gDumpConfigCMD;

int ddp_mem_test(void);
int ddp_lcd_test(void);


extern int disp_create_session(disp_session_config *config);
extern int disp_destroy_session(disp_session_config *config);

extern void disp_dump_emi_status(void);

extern unsigned int gDDPError;
#ifdef DISP_ENABLE_LAYER_FRAME
/* draw data_digit number to a buffer */
extern unsigned char data_digit[816];
#endif

/* #define DISP_ENABLE_LAYER_FRAME */
#define DISP_DIGIT_W 16
#define DISP_DIGIT_H 24

#endif /* __DDP_DEBUG_H__ */
