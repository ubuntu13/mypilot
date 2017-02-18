#include "motors.h"

void TIM4_PWM_Init(uint16_t arr,uint16_t psc)
{
	RCC->APB1ENR|=1<<2;
	RCC->APB2ENR|=1<<3;

	GPIOB->CRL&=0X00FFFFFF;
	GPIOB->CRL|=0XBB000000;
	GPIOB->CRH&=0XFFFFFF00;
	GPIOB->CRH|=0X000000BB;

	TIM4->ARR=arr;
	TIM4->PSC=psc;

	TIM4->CCMR1|=6<<4;
	TIM4->CCMR1|=1<<3;
	TIM4->CCER|=1<<0;

	TIM4->CCMR1|=6<<12;
	TIM4->CCMR1|=1<<11;
	TIM4->CCER|=1<<4;

	TIM4->CCMR2|=6<<4;
	TIM4->CCMR2|=1<<3;
	TIM4->CCER|=1<<8;

	TIM4->CCMR2|=6<<12;
	TIM4->CCMR2|=1<<11;
	TIM4->CCER|=1<<12;

	TIM4->CR1 = 1<<7;
	TIM4->CR1|=0x01;
}

void motors_set_frequency(uint16_t arr)
{
	if(arr < 490 && arr > 50)
	{
		TIM4->ARR=arr;
	}
}

void motors_init(void)
{
	TIM4_PWM_Init(2040,72-1); //490hz
	
	MOTOR1 = 1000;
	MOTOR2 = 1000;
	MOTOR3 = 1000;
	MOTOR4 = 1000;
}

void motors_stop(void)
{
	MOTOR1 = 1000;
	MOTOR2 = 1000;
	MOTOR3 = 1000;
	MOTOR4 = 1000;
}

void motors_limit(int value, int low, int high)
{
	if(value > high)
		value = high;
	if(value < low)
		value = low;
}

void motors_update(int thr, int pitch, int roll, int yaw)
{
	motors_limit(thr, 1000, 1800);
	motors_limit(pitch, 0, 200);
	motors_limit(roll, 0, 200);
	motors_limit(yaw, 0, 200);

	MOTOR1 = thr + pitch + roll + yaw;
	MOTOR2 = thr - pitch + roll - yaw;
	MOTOR3 = thr - pitch - roll + yaw;
	MOTOR4 = thr + pitch - roll - yaw;
}






