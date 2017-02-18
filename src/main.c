//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include "delay.h"
#include "sys.h"
#include "usb_com.h"
#include "led.h"
#include "timer.h"
#include "spi.h"

#include "rc.h"
#include "flash.h"
#include "mpu6500.h"
#include "motors.h"
#include "sensfusion6.h"
#include "stabilizer.h"
#include "pid.h"
#include "com.h"
#include "sensor.h"
#include "ms5611.h"

#include "usb_com.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "platform_config.h"

#include "gcs_mavlink.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//////////////////////////////////////////////

void hardware_init(void) //system init and self test
{
	SystemInit();

	//delay_ms(1000);

	usbcom_init();
	delay_ms(10);

	LED_Init();

	SPI2_Init();

	Cap_Init(0XFFFF,72-1);

	motors_init();
}

uint8_t system_post(void)
{
	if(mpu6500_check())
		return 1;

	if(SPI_Flash_ReadID() != FLASH_ID)
		return 2;

	sensor_init();

	MS561101BA_Init();

	FilterWithBuffer_reset();

	return 0;
}

int main(void)
{
	hardware_init();

	//system power on test
	if( system_post() )
	{
		while(true)
		{
			vTaskDelay(100/portTICK_RATE_MS); //100ms
			LED1= ~LED1;
		}
	}
	else
	{
		rc_init();
		comTaskInit();
		stabilizerInit();
	}

	//Start Scheduler
	vTaskStartScheduler();

	//Should never reach this point!
	while(true);

	return 0;
}



// ----------------------------------------------------------------------------
