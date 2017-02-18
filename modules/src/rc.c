#include "rc.h"
#include "led.h"
#include "timer.h"
#include "flash.h"

#include "controller.h"
#include "stabilizer.h"
#include "com.h"

#include "usb_com.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#define 	rc_filter_alpha		0.90f
#define 	rc_filter_beta		(1 - rc_filter_alpha)

uint8_t unlock = 0;

uint16_t chBufList[8] = {0,0,0,0,0,0,0,0};
uint16_t chList[8] = {0,0,0,0,0,0,0,0};
uint16_t maxchList[8] = {2000,2000,2000,2000,2000,2000,2000,2000};
uint16_t minchList[8] = {1000,1000,1000,1000,1000,1000,1000,1000};
uint16_t midchList[8];
float yaw_rate;

xTaskHandle rcSaveHandle;
static void rcSaveTask(void *pvParameters);

void rcSaveTaskInit(void)
{
    xTaskCreate(rcSaveTask, (const signed char * const)"rcSave",
              2*configMINIMAL_STACK_SIZE, NULL, 4, &rcSaveHandle);
}

void rcSaveTask(void *pvParameters)
{
	uint8_t *write_buffer = pvPortMalloc(32);

	memcpy(write_buffer,	  maxchList,  16);
	memcpy(write_buffer + 16, minchList,  16);

	SPI_Flash_Write(write_buffer, flash_address_rc, 32);

	vPortFree(write_buffer);

	LED1 = 1;
	vTaskDelay(30/portTICK_RATE_MS);
	LED1 = 0;
	vTaskDelay(30/portTICK_RATE_MS);
	LED1 = 1;
	vTaskDelay(30/portTICK_RATE_MS);
	LED1 = 0;

	comTaskInit();
	stabilizerInit();
	vTaskDelete(rcSaveHandle);
}

void rc_init(void)
{
	uint8_t *read_buffer = pvPortMalloc(32);

	SPI_Flash_Read(read_buffer, flash_address_rc, 32);

	if(read_buffer[0] == 255 && read_buffer[1] == 255)
	{
		vPortFree(read_buffer);
		return;
	}
	else
	{
		memcpy(maxchList, read_buffer, 	    16);
		memcpy(minchList, read_buffer + 16, 16);

		vPortFree(read_buffer);
	}

	uint8_t i=0;
	for(i=0; i<8; i++)
	{
		midchList[i] = (minchList[i] + maxchList[i]) / 2;
	}
}

///2ms loop
void rc_locked_check(void)
{
	static uint16_t cnt1 = 0, cnt2 = 0;

	if(bDeviceState == UNCONNECTED)
	{
		if( ((minchList[2]-100) < chList[2]) && (chList[2] < (minchList[2]+100)) && ((maxchList[3]-100) < chList[3]))
		{
			cnt1++;
			if(cnt1 > 500)
			{
				cnt1 = 0;
				unlock = 1;
			}
		}
		else
		{
			cnt1 = 0;
		}

		if( ((minchList[2]-100) < chList[2]) && (chList[2] < (minchList[2]+100)) && ((minchList[3]-100) < chList[3]) && (chList[3] < (minchList[3]+100)))
		{
			cnt2++;
			if(cnt2 > 100)
			{
				cnt2 = 0;
				unlock = 0;
			}
		}
		else
		{
			cnt2 = 0;
		}
	}
}

void rc_get_and_filter(void)
{
	u8 i=0;
	static u8 cnt = 0;

	timer2_getCap(chBufList);
	if(cnt == 0)
	{
		memcpy(chList, chBufList, 16);
		cnt++;
	}

	for(i=0; i<8; i++)
	{
		chList[i] = chList[i] * rc_filter_alpha + chBufList[i] * rc_filter_beta;
	}

	rc_locked_check();
}

void rc_get_desired(void)
{
	M_Thr = chList[2];

	pidPitch.desired = -(chList[1] - midchList[1]) * 0.05;
	pidRoll.desired = -(chList[3] - midchList[3]) * 0.05;
	yaw_rate =  -(chList[0] - midchList[0]);
	if( (yaw_rate < 10) && (yaw_rate > -10) )
		yaw_rate = 0;
	pidYaw.desired += yaw_rate * 0.0005;

	if(pidRoll.desired > 20)
	{
		pidRoll.desired = 20;
	}
	else if(pidRoll.desired < -20)
	{
		pidRoll.desired = -20;
	}

	if(pidPitch.desired > 20)
	{
		pidPitch.desired = 20;
	}
	else if(pidPitch.desired < -20)
	{
		pidPitch.desired = -20;
	}
	
	if(pidYaw.desired == 180)
	{
		pidYaw.desired = 179;
	}
	else if(pidYaw.desired == -180)
	{
		pidYaw.desired = -179;
	}
	else if(pidYaw.desired > 180)
	{
		pidYaw.desired -= 360;
	}
	else if(pidYaw.desired < -180)
	{
		pidYaw.desired += 360;
	}
}



