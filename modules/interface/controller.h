#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "sys.h"
#include "pid.h"

#define controller3AxisUpdate_dt 0.004
#define controllerAlt_Update_dt 0.001

#define AttOutputLimit  300 //attitude output limit at 30% power
#define AttPitErrLimit  450 //attitude pitch error limit at 45degree * 10 = 450
#define AttRollErrLimit 450 //attitude roll error limit at 45degree * 10 = 450
#define AltRateLimit    200 //altitude speed limit at 200cm/s
#define AltAccelLimit   490 //altitude accel limit at 0.5g to 1,5g
#define AltOutputLimit  300 //altitude output limit at 30% power

extern PidObject pidRollRate;
extern PidObject pidPitchRate;
extern PidObject pidYawRate;
extern PidObject pidRoll;
extern PidObject pidPitch;
extern PidObject pidYaw;
extern PidObject pidAltRate;
extern PidObject pidAlt;

void pidSaveTaskInit(void);

void controller_init(void);

void controller_3Axis_AttitudePID(
	float eulerRollActual, float eulerPitchActual, float eulerYawActual,
	float rollRateActual, float pitchRateActual, float yawRateActual);
	
int16_t controllerAltHoldPID(float altitudeActual, float altitudeRateActual, float accelActual);
	
void controllerResetAllPID(void);
void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);

#endif

