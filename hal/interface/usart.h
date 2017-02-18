#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"

extern uint8_t usart_rx_buf[64];
extern uint8_t usart_rx_sta;

void usart2_init(u32 bound);
void usart2_sendchar(uint8_t data);
void usart2_senddata(uint8_t* buffer, uint8_t length);

#endif
