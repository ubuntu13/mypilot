#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "com.h"
#include "pid.h"
#include "mpu6500.h"
#include "sensfusion6.h"
#include "controller.h"

uint16_t CAP_Ch1 = 0, CAP_Ch2 = 0, CAP_Ch3 = 0, CAP_Ch4 = 0,
		CAP_Ch5 = 0, CAP_Ch6 = 0, CAP_Ch7 = 0, CAP_Ch8 = 0;

void Cap_Init(uint16_t arr,uint16_t psc)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC->APB1ENR |= 1<<0;//TIM1
	RCC->APB2ENR |= 1<<2;//PA
	 
	GPIOA->CRL &= 0XFFFF0000;	//PA0~3
	GPIOA->CRL |= 0X00008888;
	GPIOA->ODR |= 15<<0;
	  
	TIM2->ARR=arr;  		//设定计数器自动重装值
	TIM2->PSC=psc;  		//预分频器

	TIM2->CCMR1|=1<<0;		//CC1S=01 	选择输入端 IC1映射到TI1上
	TIM2->CCMR1|=0<<4; 		//IC1F=0000 配置输入滤波器 不滤波
	TIM2->CCMR1|=0<<2; 		//IC1PS=00 	配置输入分频,不分频
	TIM2->CCER|=0<<1; 		//CC1P=0	上升沿捕获
	TIM2->CCER|=1<<0; 		//CC1E=1 	允许捕获计数器的值到捕获寄存器中
	
	TIM2->CCMR1|=1<<8;		//CC2S=01 	选择输入端 IC1映射到TI1上
	TIM2->CCMR1|=0<<12; 	//IC2F=0000 配置输入滤波器 不滤波
	TIM2->CCMR1|=0<<10; 	//IC2PS=00 	配置输入分频,不分频
	TIM2->CCER|=0<<5; 		//CC2P=0	上升沿捕获
	TIM2->CCER|=1<<4; 		//CC2E=1 	允许捕获计数器的值到捕获寄存器中
	
	TIM2->CCMR2|=1<<0;		//CC3S=01 	选择输入端 IC3映射到TI3上
	TIM2->CCMR2|=0<<4; 		//IC3F=0000 配置输入滤波器 不滤波
	TIM2->CCMR2|=0<<2; 		//IC3PS=00 	配置输入分频,不分频
	TIM2->CCER|=0<<9; 		//CC3P=0	上升沿捕获
	TIM2->CCER|=1<<8; 		//CC3E=1 	允许捕获计数器的值到捕获寄存器中
	
	TIM2->CCMR2|=1<<8;		//CC4S=01 	选择输入端 IC4映射到TI4上
	TIM2->CCMR2|=0<<12; 	//IC4F=0000 配置输入滤波器 不滤波
	TIM2->CCMR2|=0<<10; 	//IC4PS=00 	配置输入分频,不分频
	TIM2->CCER|=0<<13; 		//CC4P=0	上升沿捕获
	TIM2->CCER|=1<<12; 		//CC4E=1 	允许捕获计数器的值到捕获寄存器中

	TIM2->DIER|=31<<0;
	TIM2->CR1|=0x01;
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3??
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //?????0?
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //????3?
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ?????
	NVIC_Init(&NVIC_InitStructure);
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到高电平;1,已经捕获到高电平了.
//[5:0]:捕获高电平后溢出的次数
uint8_t  CAP_Ch1_CAPTURE_STA=0;	//输入捕获状态
uint16_t	CAP_Ch1_CAPTURE_VAL;	//输入捕获值
uint8_t  CAP_Ch2_CAPTURE_STA=0;	//输入捕获状态
uint16_t	CAP_Ch2_CAPTURE_VAL;	//输入捕获值
uint8_t  CAP_Ch3_CAPTURE_STA=0;	//输入捕获状态
uint16_t	CAP_Ch3_CAPTURE_VAL;	//输入捕获值
uint8_t  CAP_Ch4_CAPTURE_STA=0;	//输入捕获状态
uint16_t	CAP_Ch4_CAPTURE_VAL;	//输入捕获值
//定时器5中断服务程序	 
void TIM2_IRQHandler(void)
{ 		    
	uint16_t tsr;
	
	tsr=TIM2->SR;
	
	/*****************CAP_Ch1******************/
 	if((CAP_Ch1_CAPTURE_STA&0X80)==0)//还未成功捕获
	{
		if(tsr&0X01)//溢出
		{	    
			if(CAP_Ch1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((CAP_Ch1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					CAP_Ch1_CAPTURE_STA|=0X80;//标记成功捕获了一次
					CAP_Ch1_CAPTURE_VAL=0XFFFF;
				}else CAP_Ch1_CAPTURE_STA++;
			}	 
		}
		if(tsr&0x02)//捕获1发生捕获事件
		{	
			if(CAP_Ch1_CAPTURE_STA&0X40)		//捕获到一个下降沿
			{	  			
				CAP_Ch1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
				CAP_Ch1_CAPTURE_VAL=TIM2->CCR1;	//获取当前的捕获值.
				TIM2->CCER&=~(1<<1);			//CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				CAP_Ch1_CAPTURE_STA=0;			//清空
				CAP_Ch1_CAPTURE_VAL=0;
				CAP_Ch1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM2->CNT=0;					//计数器清空
				TIM2->CCER|=1<<1; 				//CC1P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}
	else
	{
		CAP_Ch1=CAP_Ch1_CAPTURE_STA&0X3F;
		CAP_Ch1*=65536;					//溢出时间总和
		CAP_Ch1+=CAP_Ch1_CAPTURE_VAL;
		CAP_Ch1_CAPTURE_STA=0;
	}
	
	/*****************CAP_Ch2******************/
	if((CAP_Ch2_CAPTURE_STA&0X80)==0)//还未成功捕获
	{
		if(tsr&0X01)//溢出
		{	    
			if(CAP_Ch2_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((CAP_Ch2_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					CAP_Ch2_CAPTURE_STA|=0X80;//标记成功捕获了一次
					CAP_Ch2_CAPTURE_VAL=0XFFFF;
				}
				else
				{
					CAP_Ch2_CAPTURE_STA++;
				}
			}	 
		}
		if(tsr&0x04)//捕获2发生捕获事件
		{	
			if(CAP_Ch2_CAPTURE_STA&0X40)		//捕获到一个下降沿
			{	  			
				CAP_Ch2_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
				CAP_Ch2_CAPTURE_VAL=TIM2->CCR2;	//获取当前的捕获值.
				TIM2->CCER&=~(1<<5);			//CC2P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				CAP_Ch2_CAPTURE_STA=0;			//清空
				CAP_Ch2_CAPTURE_VAL=0;
				CAP_Ch2_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM2->CNT=0;					//计数器清空
				TIM2->CCER|=1<<5; 				//CC2P=1 设置为下降沿捕获
			}		    
		}
 	}
	else
	{
		CAP_Ch2=CAP_Ch2_CAPTURE_STA&0X3F;
		CAP_Ch2*=65536;					//溢出时间总和
		CAP_Ch2+=CAP_Ch2_CAPTURE_VAL;
		CAP_Ch2_CAPTURE_STA=0;
	}
	
	/*****************CAP_Ch3******************/
	if((CAP_Ch3_CAPTURE_STA&0X80)==0)//还未成功捕获
	{
		if(tsr&0X01)//溢出
		{	    
			if(CAP_Ch3_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((CAP_Ch3_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					CAP_Ch3_CAPTURE_STA|=0X80;//标记成功捕获了一次
					CAP_Ch3_CAPTURE_VAL=0XFFFF;
				}else CAP_Ch3_CAPTURE_STA++;
			}	 
		}
		if(tsr&0x08)//捕获3发生捕获事件
		{	
			if(CAP_Ch3_CAPTURE_STA&0X40)		//捕获到一个下降沿
			{	  			
				CAP_Ch3_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  	CAP_Ch3_CAPTURE_VAL=TIM2->CCR3;	//获取当前的捕获值.
	 			TIM2->CCER&=~(1<<9);			//CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				CAP_Ch3_CAPTURE_STA=0;			//清空
				CAP_Ch3_CAPTURE_VAL=0;
				CAP_Ch3_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM2->CNT=0;					//计数器清空
				TIM2->CCER|=1<<9; 				//CC1P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}
	else
	{
		CAP_Ch3=CAP_Ch3_CAPTURE_STA&0X3F;
		CAP_Ch3*=65536;					//溢出时间总和
		CAP_Ch3+=CAP_Ch3_CAPTURE_VAL;
		CAP_Ch3_CAPTURE_STA=0;
	}
	
	/*****************CAP_Ch4******************/
	if((CAP_Ch4_CAPTURE_STA&0X80)==0)//还未成功捕获
	{
		if(tsr&0X01)//溢出
		{	    
			if(CAP_Ch4_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((CAP_Ch4_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					CAP_Ch4_CAPTURE_STA|=0X80;//标记成功捕获了一次
					CAP_Ch4_CAPTURE_VAL=0XFFFF;
				}
				else
				{
					CAP_Ch4_CAPTURE_STA++;
				}
			}	 
		}
		if(tsr&0x10)//捕获4发生捕获事件
		{	
			if(CAP_Ch4_CAPTURE_STA&0X40)		//捕获到一个下降沿
			{	  			
				CAP_Ch4_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
				CAP_Ch4_CAPTURE_VAL=TIM2->CCR4;	//获取当前的捕获值.
				TIM2->CCER&=~(1<<13);			//CC4P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				CAP_Ch4_CAPTURE_STA=0;			//清空
				CAP_Ch4_CAPTURE_VAL=0;
				CAP_Ch4_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM2->CNT=0;					//计数器清空
				TIM2->CCER|=1<<13; 				//CC4P=1 设置为下降沿捕获
			}		    
		}
 	}
	else
	{
		CAP_Ch4=CAP_Ch4_CAPTURE_STA&0X3F;
		CAP_Ch4*=65536;					//溢出时间总和
		CAP_Ch4+=CAP_Ch4_CAPTURE_VAL;
		CAP_Ch4_CAPTURE_STA=0;
	}
	
	TIM2->SR=0;//清除中断标志位
}

void timer2_getCap(uint16_t *cap)
{
	cap[0] = CAP_Ch1;
	cap[1] = CAP_Ch2;
	cap[2] = CAP_Ch3;
	cap[3] = CAP_Ch4;
	cap[4] = CAP_Ch5;
	cap[5] = CAP_Ch6;
	cap[6] = CAP_Ch7;
	cap[7] = CAP_Ch8;
}



