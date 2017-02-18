#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"

#define MOTOR1 TIM4->CCR1
#define MOTOR2 TIM4->CCR2
#define MOTOR3 TIM4->CCR3
#define MOTOR4 TIM4->CCR4

void TIM1_Init(void);
void Cap_Init(uint16_t arr, uint16_t psc);
void timer2_getCap(uint16_t *cap);

#endif























