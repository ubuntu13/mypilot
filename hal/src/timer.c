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
	  
	TIM2->ARR=arr;  		//�趨�������Զ���װֵ
	TIM2->PSC=psc;  		//Ԥ��Ƶ��

	TIM2->CCMR1|=1<<0;		//CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM2->CCMR1|=0<<4; 		//IC1F=0000 ���������˲��� ���˲�
	TIM2->CCMR1|=0<<2; 		//IC1PS=00 	���������Ƶ,����Ƶ
	TIM2->CCER|=0<<1; 		//CC1P=0	�����ز���
	TIM2->CCER|=1<<0; 		//CC1E=1 	�������������ֵ������Ĵ�����
	
	TIM2->CCMR1|=1<<8;		//CC2S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM2->CCMR1|=0<<12; 	//IC2F=0000 ���������˲��� ���˲�
	TIM2->CCMR1|=0<<10; 	//IC2PS=00 	���������Ƶ,����Ƶ
	TIM2->CCER|=0<<5; 		//CC2P=0	�����ز���
	TIM2->CCER|=1<<4; 		//CC2E=1 	�������������ֵ������Ĵ�����
	
	TIM2->CCMR2|=1<<0;		//CC3S=01 	ѡ������� IC3ӳ�䵽TI3��
	TIM2->CCMR2|=0<<4; 		//IC3F=0000 ���������˲��� ���˲�
	TIM2->CCMR2|=0<<2; 		//IC3PS=00 	���������Ƶ,����Ƶ
	TIM2->CCER|=0<<9; 		//CC3P=0	�����ز���
	TIM2->CCER|=1<<8; 		//CC3E=1 	�������������ֵ������Ĵ�����
	
	TIM2->CCMR2|=1<<8;		//CC4S=01 	ѡ������� IC4ӳ�䵽TI4��
	TIM2->CCMR2|=0<<12; 	//IC4F=0000 ���������˲��� ���˲�
	TIM2->CCMR2|=0<<10; 	//IC4PS=00 	���������Ƶ,����Ƶ
	TIM2->CCER|=0<<13; 		//CC4P=0	�����ز���
	TIM2->CCER|=1<<12; 		//CC4E=1 	�������������ֵ������Ĵ�����

	TIM2->DIER|=31<<0;
	TIM2->CR1|=0x01;
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3??
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //?????0?
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //????3?
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ?????
	NVIC_Init(&NVIC_InitStructure);
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽸ߵ�ƽ;1,�Ѿ����񵽸ߵ�ƽ��.
//[5:0]:����ߵ�ƽ������Ĵ���
uint8_t  CAP_Ch1_CAPTURE_STA=0;	//���벶��״̬
uint16_t	CAP_Ch1_CAPTURE_VAL;	//���벶��ֵ
uint8_t  CAP_Ch2_CAPTURE_STA=0;	//���벶��״̬
uint16_t	CAP_Ch2_CAPTURE_VAL;	//���벶��ֵ
uint8_t  CAP_Ch3_CAPTURE_STA=0;	//���벶��״̬
uint16_t	CAP_Ch3_CAPTURE_VAL;	//���벶��ֵ
uint8_t  CAP_Ch4_CAPTURE_STA=0;	//���벶��״̬
uint16_t	CAP_Ch4_CAPTURE_VAL;	//���벶��ֵ
//��ʱ��5�жϷ������	 
void TIM2_IRQHandler(void)
{ 		    
	uint16_t tsr;
	
	tsr=TIM2->SR;
	
	/*****************CAP_Ch1******************/
 	if((CAP_Ch1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
	{
		if(tsr&0X01)//���
		{	    
			if(CAP_Ch1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((CAP_Ch1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					CAP_Ch1_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					CAP_Ch1_CAPTURE_VAL=0XFFFF;
				}else CAP_Ch1_CAPTURE_STA++;
			}	 
		}
		if(tsr&0x02)//����1���������¼�
		{	
			if(CAP_Ch1_CAPTURE_STA&0X40)		//����һ���½���
			{	  			
				CAP_Ch1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				CAP_Ch1_CAPTURE_VAL=TIM2->CCR1;	//��ȡ��ǰ�Ĳ���ֵ.
				TIM2->CCER&=~(1<<1);			//CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				CAP_Ch1_CAPTURE_STA=0;			//���
				CAP_Ch1_CAPTURE_VAL=0;
				CAP_Ch1_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM2->CNT=0;					//���������
				TIM2->CCER|=1<<1; 				//CC1P=1 ����Ϊ�½��ز���
			}		    
		}			     	    					   
 	}
	else
	{
		CAP_Ch1=CAP_Ch1_CAPTURE_STA&0X3F;
		CAP_Ch1*=65536;					//���ʱ���ܺ�
		CAP_Ch1+=CAP_Ch1_CAPTURE_VAL;
		CAP_Ch1_CAPTURE_STA=0;
	}
	
	/*****************CAP_Ch2******************/
	if((CAP_Ch2_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
	{
		if(tsr&0X01)//���
		{	    
			if(CAP_Ch2_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((CAP_Ch2_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					CAP_Ch2_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					CAP_Ch2_CAPTURE_VAL=0XFFFF;
				}
				else
				{
					CAP_Ch2_CAPTURE_STA++;
				}
			}	 
		}
		if(tsr&0x04)//����2���������¼�
		{	
			if(CAP_Ch2_CAPTURE_STA&0X40)		//����һ���½���
			{	  			
				CAP_Ch2_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				CAP_Ch2_CAPTURE_VAL=TIM2->CCR2;	//��ȡ��ǰ�Ĳ���ֵ.
				TIM2->CCER&=~(1<<5);			//CC2P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				CAP_Ch2_CAPTURE_STA=0;			//���
				CAP_Ch2_CAPTURE_VAL=0;
				CAP_Ch2_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM2->CNT=0;					//���������
				TIM2->CCER|=1<<5; 				//CC2P=1 ����Ϊ�½��ز���
			}		    
		}
 	}
	else
	{
		CAP_Ch2=CAP_Ch2_CAPTURE_STA&0X3F;
		CAP_Ch2*=65536;					//���ʱ���ܺ�
		CAP_Ch2+=CAP_Ch2_CAPTURE_VAL;
		CAP_Ch2_CAPTURE_STA=0;
	}
	
	/*****************CAP_Ch3******************/
	if((CAP_Ch3_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
	{
		if(tsr&0X01)//���
		{	    
			if(CAP_Ch3_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((CAP_Ch3_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					CAP_Ch3_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					CAP_Ch3_CAPTURE_VAL=0XFFFF;
				}else CAP_Ch3_CAPTURE_STA++;
			}	 
		}
		if(tsr&0x08)//����3���������¼�
		{	
			if(CAP_Ch3_CAPTURE_STA&0X40)		//����һ���½���
			{	  			
				CAP_Ch3_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  	CAP_Ch3_CAPTURE_VAL=TIM2->CCR3;	//��ȡ��ǰ�Ĳ���ֵ.
	 			TIM2->CCER&=~(1<<9);			//CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				CAP_Ch3_CAPTURE_STA=0;			//���
				CAP_Ch3_CAPTURE_VAL=0;
				CAP_Ch3_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM2->CNT=0;					//���������
				TIM2->CCER|=1<<9; 				//CC1P=1 ����Ϊ�½��ز���
			}		    
		}			     	    					   
 	}
	else
	{
		CAP_Ch3=CAP_Ch3_CAPTURE_STA&0X3F;
		CAP_Ch3*=65536;					//���ʱ���ܺ�
		CAP_Ch3+=CAP_Ch3_CAPTURE_VAL;
		CAP_Ch3_CAPTURE_STA=0;
	}
	
	/*****************CAP_Ch4******************/
	if((CAP_Ch4_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
	{
		if(tsr&0X01)//���
		{	    
			if(CAP_Ch4_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((CAP_Ch4_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					CAP_Ch4_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					CAP_Ch4_CAPTURE_VAL=0XFFFF;
				}
				else
				{
					CAP_Ch4_CAPTURE_STA++;
				}
			}	 
		}
		if(tsr&0x10)//����4���������¼�
		{	
			if(CAP_Ch4_CAPTURE_STA&0X40)		//����һ���½���
			{	  			
				CAP_Ch4_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				CAP_Ch4_CAPTURE_VAL=TIM2->CCR4;	//��ȡ��ǰ�Ĳ���ֵ.
				TIM2->CCER&=~(1<<13);			//CC4P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				CAP_Ch4_CAPTURE_STA=0;			//���
				CAP_Ch4_CAPTURE_VAL=0;
				CAP_Ch4_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM2->CNT=0;					//���������
				TIM2->CCER|=1<<13; 				//CC4P=1 ����Ϊ�½��ز���
			}		    
		}
 	}
	else
	{
		CAP_Ch4=CAP_Ch4_CAPTURE_STA&0X3F;
		CAP_Ch4*=65536;					//���ʱ���ܺ�
		CAP_Ch4+=CAP_Ch4_CAPTURE_VAL;
		CAP_Ch4_CAPTURE_STA=0;
	}
	
	TIM2->SR=0;//����жϱ�־λ
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



