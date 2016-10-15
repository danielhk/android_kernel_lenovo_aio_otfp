#ifndef _H_DDP_IRQ_H
#define _H_DDP_IRQ_H

#include "ddp_info.h"
#include "ddp_log.h"
#include "ddp_debug.h"

#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/timer.h>

#include <mach/mt_irq.h>
typedef void (*DDP_IRQ_CALLBACK)(DISP_MODULE_ENUM module, unsigned int reg_value);

int disp_register_module_irq_callback(DISP_MODULE_ENUM module, DDP_IRQ_CALLBACK cb);
int disp_unregister_module_irq_callback(DISP_MODULE_ENUM module, DDP_IRQ_CALLBACK cb);

int disp_register_irq_callback(DDP_IRQ_CALLBACK cb);
int disp_unregister_irq_callback(DDP_IRQ_CALLBACK cb);

void disp_register_irq(unsigned int irq_num, char *device_name);
int disp_init_irq(void);
irqreturn_t disp_irq_handler(int irq, void *dev_id);

/* extern smi_dumpDebugMsg(void); */

extern unsigned int gResetRDMAEnable;
extern unsigned int is_hwc_enabled;
extern unsigned int gEnableUnderflowAEE;

#endif

