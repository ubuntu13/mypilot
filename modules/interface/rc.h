#ifndef __RC_H
#define __RC_H

#include "sys.h"

extern uint16_t chList[8];
extern uint16_t maxchList[8];
extern uint16_t minchList[8];
extern float yaw_rate;
extern uint8_t unlock;

void rc_init(void);
void rcSaveTaskInit(void);
void rc_get_and_filter(void);
void rc_get_desired(void);

#endif

