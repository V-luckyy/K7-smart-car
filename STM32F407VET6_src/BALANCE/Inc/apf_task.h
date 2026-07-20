#ifndef __APF_TASK_H
#define __APF_TASK_H

#include "system.h"

#define APF_TASK_PRIO      5
#define APF_STK_SIZE       512
#define APF_TASK_RATE      RATE_50_HZ   /* 50Hz, 20ms per cycle */

void APF_task(void *pvParameters);

#endif
