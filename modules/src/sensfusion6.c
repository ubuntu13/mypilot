#include "sensfusion6.h"
#include "mpu6500.h"
#include "sensor.h"
#include <math.h>

#define 	Buf_Num  	19
#define     RtoA        57.2957795f
#define     AtoR        0.01745329f
#define     Gyrot       0.00106422f

struct Angle Attitude, Expect;
struct F_XYZ Fil_Accel, Fil_Gyro;

float Buf_AX[Buf_Num], Buf_AY[Buf_Num], Buf_AZ[Buf_Num];
float vspeedAccel = 0;

float Gravity_mss = 9.78f;

uint8_t buffer[26] = 
{
	'G', 'Y'
};

void Send_mpu_data(void)
{
 	
}

void sensfusion6_Init(void)
{
	Fil_Gyro.X = 0;
	Fil_Gyro.Y = 0;
	Fil_Gyro.Z = 0;
	
	Fil_Accel.X = 0;
	Fil_Accel.Y = 0;
	Fil_Accel.Z = 0;
}

void IMU_Prepare(void)
{
    uint8_t i = 0;
	float temp_X = 0, temp_Y = 0, temp_Z = 0;
	
	float Hn[Buf_Num] =
	{
		-0.0048,  0.0000,  0.0155,  0.0186, -0.0152,
 		-0.0593, -0.0345,  0.1045,  0.2881,  0.3739,
 		 0.2881,  0.1045, -0.0345, -0.0593, -0.0152,
		 0.0186,  0.0155,  0.0000, -0.0048
	};
	
	Fil_Gyro.X = gyro.x * Gyrot;
	Fil_Gyro.Y = gyro.y * Gyrot;
	Fil_Gyro.Z = gyro.z * Gyrot;
	
	Buf_AX[0] = accel.x;
	Buf_AY[0] = accel.y;
	Buf_AZ[0] = accel.z;
	
	for(i = 0; i < Buf_Num; i++)
	{
		temp_X += Buf_AX[i] * Hn[i];
		temp_Y += Buf_AY[i] * Hn[i];
		temp_Z += Buf_AZ[i] * Hn[i];
	}
	
	Fil_Accel.X = temp_X;
	Fil_Accel.Y = temp_Y;
	Fil_Accel.Z = temp_Z;
	
	for(i = 0; i < Buf_Num -1; i++)
	{
		Buf_AX[Buf_Num-1-i] = Buf_AX[Buf_Num-2-i];
		Buf_AY[Buf_Num-1-i] = Buf_AY[Buf_Num-2-i];
		Buf_AZ[Buf_Num-1-i] = Buf_AZ[Buf_Num-2-i];
	}
}

float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

#define twoKp 		0.8f    //0.1f 		//0.8f		//2.0f
#define twoKi 		0.002f  //0.001f 	//0.002f	//0.008
#define dt 	      	0.004f

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float integralFBx = 0, integralFBy = 0, integralFBz = 0;

void sensfusion6_update(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  recipNorm = invSqrt(ax*ax + ay*ay + az*az);
  ax = ax * recipNorm;
  ay = ay * recipNorm;
  az = az * recipNorm;
           
  halfvx = q1 * q3 - q0 * q2;
  halfvy = q0 * q1 + q2 * q3;
  halfvz = q0 * q0 - 0.5f + q3 * q3;

  halfex = (ay * halfvz - az * halfvy);
  halfey = (az * halfvx - ax * halfvz);
  halfez = (ax * halfvy - ay * halfvx);

  integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
  integralFBy += twoKi * halfey * dt;
  integralFBz += twoKi * halfez * dt;
  gx += integralFBx;  // apply integral feedback
  gy += integralFBy;
  gz += integralFBz;

  gx += twoKp * halfex;
  gy += twoKp * halfey;
  gz += twoKp * halfez;
					   
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  sensfusion6GetEulerRPY(&Attitude.Pitch, &Attitude.Roll, &Attitude.Yaw);
}

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx, gy, gz; // estimated gravity direction

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  if (gx>1) gx=1;
  if (gx<-1) gx=-1;

  *yaw = atan2(2*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) * RtoA;
  *pitch = asin(gx) * RtoA; //Pitch seems to be inverted
  *roll = atan2(gy, gz) * RtoA;
}

float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
  float gx, gy, gz; // estimated gravity direction

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // return vertical acceleration without gravity
  // (A dot G) / |G| - 1G (|G| = 1) -> (A dot G) - 1G
  return ((ax*gx + ay*gy + az*gz) - Gravity_mss);
}




