#ifndef __DISP_ASSERT_LAYER_PRIV__
#define __DISP_ASSERT_LAYER_PRIV__

#include <linux/types.h>

unsigned long get_Assert_Layer_PA(void);
int is_DAL_Enabled(void);

extern unsigned int isAEEEnabled;
extern int DAL_Clean(void);
extern int DAL_Printf(const char *fmt, ...);
extern struct semaphore dal_sem;
extern bool dal_shown;

#endif

