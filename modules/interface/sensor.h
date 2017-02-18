#ifndef _sensor_H
#define _sensor_H

#include "sys.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "task.h"

struct Vector3f
{
	float x;
	float y;
	float z;
};

struct sensor
{
	float x;
	float y;
	float z;
	bool calibrated;
};

extern struct sensor accel;
extern struct sensor gyro;
extern xTaskHandle sensorCaliHandle;

void sensorTaskInit(void);

void sensor_init(void);
void sensor_update(void);
void sensor_calibrate_gyro(void);
bool sensor_calibrate_accel(void);
bool sensor_calibrate_accel_compute(void);
void sensor_calibrate_update_matrices(float dS[6], float JS[6][6],
                                    float beta[6], float data[3]);
void sensor_calibrate_reset_matrices(float dS[6], float JS[6][6]);
void sensor_calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);

#endif

