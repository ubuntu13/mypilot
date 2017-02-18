#ifndef __INAV_H
#define __INAV_H

#include "sys.h"
#include "usart.h"

extern float accel_ef_z, awg;
extern float _velocity_z;
extern float altitude;
extern float _position_z, _position_base_z, _position_correction_z, _position_error_z;

void InertialNav_update(float dt);
void InertialNav_correct_with_baro(float baro_alt);
void InertialNav_set_altitude( float new_altitude);

#endif
