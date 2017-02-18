#ifndef __motors_h
#define __motors_h

#include "sys.h"
#include "timer.h"

void motors_init(void);
void motors_stop(void);
void motors_set_frequency(uint16_t arr);
void motors_update(int thr, int pitch, int roll, int yaw);

#endif

