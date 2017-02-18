#ifndef __FILTER_H
#define __FILTER_H

#include "sys.h"
#include "usart.h"
#include "math.h"

#define FILTER_SIZE  11

void FilterWithBuffer_reset(void);
void FilterWithBuffer_apply(float sample);
float DerivativeFilter_slope(void);

#endif

