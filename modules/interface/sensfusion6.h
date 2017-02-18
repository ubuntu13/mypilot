#ifndef _sensfusion6_H
#define _sensfusion6_H

#include "sys.h"

struct Angle
{
	float Pitch;
	float Roll;
	float Yaw;
};
extern struct Angle Attitude;
extern struct Angle Expect;
extern struct F_XYZ Fil_Accel, Fil_Gyro;

void IMU_Prepare(void);
void sensfusion6_init(void);
void sensfusion6_update(float gx, float gy, float gz, float ax, float ay, float az);
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az);

#endif
