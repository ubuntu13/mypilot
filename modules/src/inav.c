#include "inav.h"
#include "ms5611.h"
#include "sensfusion6.h"
#include "buffer.h"

#define 	_time_constant_z	5.0f
#define 	_k1_z 				3 / _time_constant_z
#define 	_k2_z 				3 / (_time_constant_z*_time_constant_z)
#define 	_k3_z 				1 / (_time_constant_z*_time_constant_z*_time_constant_z)

float awg = 0;  //accel without gravity
float accel_ef_z = 0;
float accel_correction_ef_z = 0;
float _position_z = 0;
float _velocity_z, velocity_increase_z = 0;
float _position_base_z = 0, _position_error_z = 0, _position_correction_z = 0;

extern float altitude;

void InertialNav_update(float dt)
{
	InertialNav_correct_with_baro(altitude * 100);

    // remove influence of gravity
    accel_ef_z = awg * 100; //cm/s2

    accel_correction_ef_z += _position_error_z * _k3_z  * dt;

    _velocity_z += _position_error_z * _k2_z  * dt;

    _position_correction_z += _position_error_z * _k1_z  * dt;

    // calculate velocity increase adding new acceleration from accelerometers
    velocity_increase_z = (accel_ef_z + accel_correction_ef_z) * dt;

    // calculate new estimate of position
    _position_base_z += (_velocity_z + velocity_increase_z * 0.5f) * dt;

    _position_z = _position_base_z + _position_correction_z;

    // calculate new velocity
    _velocity_z += velocity_increase_z;

    // store 3rd order estimate (i.e. estimated vertical position) for future use
    Buffer_push_back(_position_base_z);
}

// correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
void InertialNav_correct_with_baro(float baro_alt)
{
    static uint8_t first_reads = 0;

    // discard first 10 reads but perform some initialisation
    if( first_reads <= 10 )
    {
    	InertialNav_set_altitude(baro_alt);
        first_reads++;
    }

    // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
    // so we should calculate error using historical estimates
    float hist_position_base_z = 0;
    if( Buffer_is_full )
    {
        hist_position_base_z = Buffer_peek(14);
    }
    else
    {
        hist_position_base_z = _position_base_z;
    }

    // calculate error in position from baro with our estimate
    _position_error_z = baro_alt - (hist_position_base_z + _position_correction_z);
}

// set_altitude - set base altitude estimate in cm
void InertialNav_set_altitude( float new_altitude)
{
    _position_base_z = new_altitude;
    _position_correction_z = 0;
}













