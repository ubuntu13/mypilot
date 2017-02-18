#include "com.h"
#include "usb_com.h"
#include "led.h"
#include "timer.h"
#include "motors.h"

#include "rc.h"
#include "controller.h"
#include "stabilizer.h"
#include "mpu6500.h"
#include "sensor.h"
#include "ms5611.h"
#include "usb_desc.h"
#include "gcs_mavlink.h"

#include "usb_com.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>

#define comTx_UPDATE_DT  20/portTICK_RATE_MS

extern uint8_t buffer_out[VIRTUAL_COM_PORT_DATA_SIZE];
extern uint32_t count_out;

uint8_t comm_pid = 0;
uint8_t comm_process_en = 0;

xTaskHandle comSendHandle;
static void comSendTask(void *pvParameters);

xTaskHandle comRxHandle;
static void comRxTask(void *pvParameters);

void comTaskInit(void)
{
    xTaskCreate(comSendTask, (const signed char * const)"comSend",
              2*configMINIMAL_STACK_SIZE, NULL, 2, &comSendHandle);

    xTaskCreate(comRxTask, (const signed char * const)"comRx",
              2*configMINIMAL_STACK_SIZE, NULL, 2, &comRxHandle);
}

static void comSendTask(void *pvParameters)
{
	uint32_t lastWakeTime;

	lastWakeTime = xTaskGetTickCount();

	while(true)
	{
		vTaskDelayUntil(&lastWakeTime, comTx_UPDATE_DT); // 50Hz

		if(bDeviceState != UNCONNECTED)
		{
			LED2 = 0;

			motors_stop();

			commander();

			static uint8_t i=0;
			if(++i >= 5)
			{
				i=0;
				LED3 = ~LED3;
			}
		}
		else
		{
			LED2 = 1;
			if(unlock) //unlock
			{
				static uint8_t j=0;
				if(++j >= 5)
				{
					j=0;
					LED3 = ~LED3;
				}
			}
			else
			{
				static uint8_t k=0;
				if(++k >= 20)
				{
					k=0;
					LED3 = ~LED3;
				}

				motors_stop();
			}
		}
	}
}

static void comRxTask(void *pvParameters)
{
	while(true)
	{
		if(comm_process_en)
		{
			comm_process_en = 0;
			comm_process();
		}
	}
}

void commander(void)
{
	static uint8_t i = 0;

	if(comm_pid)
	{
		comm_pid = 0;

		gcs_mavlink_sendmsg_RollPid();
		gcs_mavlink_sendmsg_PitchPid();
		gcs_mavlink_sendmsg_YawPid();
	}
	else
	{
		i++;

		gcs_mavlink_sendmsg_attitude();
		if(i >= 4)
		{
			i = 0;

			gcs_mavlink_sendmsg_altitude();
			gcs_mavlink_sendmsg_rc_channels_scaled();
		}
	}
}

void comm_process(void)
{
	uint8_t temp = 0;

	if((uint8_t)(buffer_out[1] + 8) == count_out)
	{
		if(buffer_out[5] == 60)
		{
			if(bDeviceState != UNCONNECTED)
			{
				mavlink_quad_motors_decode(buffer_out);
			}
		}
		else if(buffer_out[5] == 148)
		{
			temp = mavlink_8dof_decode(buffer_out);
			switch(temp)
			{
				case 1:
					pidSaveTaskInit();
					vTaskDelete(stabilizerHandle);
					vTaskDelete(comSendHandle);
					vTaskDelete(comRxHandle);
					break;

				case 2:
					comm_pid = 1;
					break;

				case 3:
					sensorTaskInit();
					vTaskDelete(stabilizerHandle);
					vTaskDelete(comSendHandle);
					vTaskDelete(comRxHandle);
					break;

				case 4:
					rcSaveTaskInit();
					vTaskDelete(stabilizerHandle);
					vTaskDelete(comSendHandle);
					vTaskDelete(comRxHandle);
					break;

				default:
					break;
			}
		}
	}
}



