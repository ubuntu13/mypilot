#ifndef __USB_COM_H
#define __USB_COM_H

#include "stm32f10x.h"

void usbcom_init(void);
void usbcom_send(u8* data_buffer, u8 len);
void usbcom_receive(void);

#endif
