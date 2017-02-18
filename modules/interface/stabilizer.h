#ifndef STABILIZER_H
#define STABILIZER_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"

extern int16_t M_Thr;
extern xTaskHandle stabilizerHandle;

void stabilizerInit(void);

#endif


