#ifndef __COM_H
#define __COM_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"

extern uint8_t comm_pid;

extern xTaskHandle comSendHandle;
extern xTaskHandle comRxHandle;

void Send_Messge(void);
void commander(void);
void comm_process(void);

void comTaskInit(void);

#endif

