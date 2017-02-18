#include "stm32f10x.h"
#include "led.h"
#include "delay.h"
#include "usb_com.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "platform_config.h"
#include "gcs_mavlink.h"
#include "com.h"

extern uint32_t count_in;
extern uint8_t comm_process_en;

void usbcom_init(void)
{
	usb_hal_config();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
}

void usbcom_send(uint8_t* data_buffer, u8 len)
{
	uint16_t i = 50000;

	while(i--)
	{
		if(count_in == 0)
			break;
	}

	count_in = len;

	UserToPMABufferCopy(data_buffer, ENDP1_TXADDR, count_in);
	SetEPTxCount(ENDP1, count_in);
	SetEPTxValid(ENDP1);
}

void usbcom_receive(void)
{
	comm_process_en = 1;
}








