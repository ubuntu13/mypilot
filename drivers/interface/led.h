#ifndef __LED_H
#define __LED_H	 

#include "stm32f10x.h"
#include "sys.h"

#define LED3 PCout(13)
#define LED2 PCout(14)
#define LED1 PCout(15)

void LED_Init(void);

#endif
