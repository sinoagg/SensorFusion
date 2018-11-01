#ifndef __THREAD_H
#define __THREAD_H

#include "cmsis_os.h"

extern osSemaphoreId bSemRadarCANRxSigHandle;
extern osSemaphoreId bSemADASRxSigHandle;
extern osSemaphoreId bSemPrepareCanDataSigHandle;
extern osSemaphoreId bSemRadarCalcSigHandle;

#endif
