

#include "sec_osal_light.h"

#ifdef CONFIG_ARM64
extern unsigned long long es_base;
#else
extern unsigned int es_base;
#endif

#define HRID0   ((volatile unsigned int*)(es_base + 0x140))
#define HRID1   ((volatile unsigned int*)(es_base + 0x144))

int masp_hal_get_uuid(unsigned int *uuid)
{
    uuid[0] = (unsigned int)__raw_readl(HRID0);
    uuid[1] = (unsigned int)__raw_readl(HRID1);
    uuid[2] = (unsigned int)__raw_readl(HRID0);
    uuid[3] = (unsigned int)__raw_readl(HRID1);

    return 0;
}

