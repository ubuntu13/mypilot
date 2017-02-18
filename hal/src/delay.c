#include "delay.h"

void delay_ms(u16 num)
{
	u16 i = 7200;

	while(num--)
	{
		i = 7200;
		while(i--);
	}
}

void delay_us(u16 num)
{
	u16 i = 7;

	while(num--)
	{
		i = 7;
		while(i--);
	}
}




















