#ifndef __BUFFER_H
#define __BUFFER_H

#include "sys.h"
#include "usart.h"
#include "math.h"

#define SIZE 15

uint8_t     _num_items;             // number of items in the buffer
uint8_t     _head;                  // first item in the buffer (will be returned with the next pop_front call)
float       _buff[SIZE];            // x values of each point on the curve

#define Buffer_is_full  _num_items >= SIZE

void Buffer_clear(void);
void Buffer_push_back( const float item );
float Buffer_pop_front(void);
float Buffer_front(void);
float Buffer_peek(uint8_t position);

#endif

