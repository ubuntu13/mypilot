#include "controller.h"
#include "stabilizer.h"
#include "com.h"
#include "led.h"
#include "sensfusion6.h"
#include "flash.h"
#include "delay.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

PidObject pidRollRate;
PidObject pidRoll;
PidObject pidPitchRate;
PidObject pidPitch;
PidObject pidYawRate;
PidObject pidYaw;
PidObject pidAltAccel;
PidObject pidAltRate;
PidObject pidAlt;

float rollOutput = 0;
float pitchOutput = 0;
float yawOutput = 0;

xTaskHandle pidSaveHandle;
static void pidSaveTask(void *pvParameters);

void pidSaveTaskInit(void)
{
    xTaskCreate(pidSaveTask, (const signed char * const)"pidSave",
              2*configMINIMAL_STACK_SIZE, NULL, 4, &pidSaveHandle);
}

void pidSaveTask(void *pvParameters)
{
	uint8_t *write_buffer = pvPortMalloc(120);

	while(1)
	{
		memcpy(write_buffer      , &pidRoll,  20);
		memcpy(write_buffer + 20 , &pidPitch, 20);
		memcpy(write_buffer + 40 , &pidYaw,   20);
		memcpy(write_buffer + 60 , &pidRollRate,  20);
		memcpy(write_buffer + 80 , &pidPitchRate, 20);
		memcpy(write_buffer + 100, &pidYawRate,   20);

		SPI_Flash_Write(write_buffer, flash_address_pid, 120);

		vPortFree(write_buffer);

		LED1 = 1;
		vTaskDelay(30/portTICK_RATE_MS);
		LED1 = 0;
		vTaskDelay(30/portTICK_RATE_MS);
		LED1 = 1;
		vTaskDelay(30/portTICK_RATE_MS);
		LED1 = 0;

		comTaskInit();
		stabilizerInit();
		vTaskDelete(pidSaveHandle);
	}
}

void controller_init(void)
{
	uint8_t *read_buffer = pvPortMalloc(120);

	pidInit(&pidRollRate, 0, 0.16, 0.13, 0.009, 50, controller3AxisUpdate_dt);
	pidInit(&pidRoll, 0, 5, 0, 0, 0, controller3AxisUpdate_dt);
	
	pidInit(&pidPitchRate, 0, 0.16, 0.13, 0.009, 50, controller3AxisUpdate_dt);
	pidInit(&pidPitch, 0, 5, 0, 0, 0, controller3AxisUpdate_dt);
	
	pidInit(&pidYawRate, 0, 0.4, 0.02, 0, 1, controller3AxisUpdate_dt);
	pidInit(&pidYaw, 0, 5, 0, 0, 0, controller3AxisUpdate_dt);
	
	pidInit(&pidAltAccel, 0, 0.5, 1, 0, 10, controllerAlt_Update_dt);
  	pidInit(&pidAltRate, 0, 6, 0, 0, 0, controllerAlt_Update_dt);
    pidInit(&pidAlt, 0, 1, 0, 0, 0, controllerAlt_Update_dt);

    SPI_Flash_Read(read_buffer, 70, 120);

	if(read_buffer[0] == 255 && read_buffer[1] == 255)
    {
		vPortFree(read_buffer);
		return;
    }
	else
	{
		memcpy(&pidRoll,      read_buffer      , 20);
		memcpy(&pidPitch,     read_buffer + 20 , 20);
		memcpy(&pidYaw,       read_buffer + 40 , 20);
		memcpy(&pidRollRate,  read_buffer + 60 , 20);
		memcpy(&pidPitchRate, read_buffer + 80 , 20);
		memcpy(&pidYawRate,   read_buffer + 100, 20);

		vPortFree(read_buffer);
	}
}

void controller_3Axis_AttitudePID(
	float eulerRollActual, float eulerPitchActual, float eulerYawActual,
	float rollRateActual, float pitchRateActual, float yawRateActual)
{
	// Update PID for roll axis attitude
	pidRoll.error = (pidRoll.desired - eulerRollActual) * 10;
	if(pidRoll.error > AttPitErrLimit)
		pidRoll.error = AttPitErrLimit;
	else if(pidRoll.error < -AttPitErrLimit)
		pidRoll.error = -AttPitErrLimit;
	pidRollRate.desired = get_p(&pidRoll);
	
	// Update PID for pitch axis attitude
	pidPitch.error = (pidPitch.desired - eulerPitchActual) * 10;
	if(pidPitch.error > AttPitErrLimit)
		pidPitch.error = AttPitErrLimit;
	else if(pidPitch.error < -AttPitErrLimit)
		pidPitch.error = -AttPitErrLimit;
	pidPitchRate.desired = get_p(&pidPitch);
	
	// Update PID for yaw axis attitude
	pidYaw.error = pidYaw.desired - eulerYawActual;
	if (pidYaw.error > 180.0)
		pidYaw.error -= 360.0;
 	else if (pidYaw.error < -180.0)
 		pidYaw.error += 360.0;
	pidYaw.error *= 10;
	pidYawRate.desired = pidYaw.kp * pidYaw.error;
	
	// Update PID for 3 axis Rate
	pidRollRate.error = pidRollRate.desired - rollRateActual * 10;
	pidPitchRate.error = pidPitchRate.desired - pitchRateActual * 10;
	pidYawRate.error = pidYawRate.desired - yawRateActual * 10;
	rollOutput = get_pid(&pidRollRate);
	pitchOutput = get_pid(&pidPitchRate);
	yawOutput = get_pid(&pidYawRate);
	
	if(rollOutput > AttOutputLimit)
		rollOutput = AttOutputLimit;
	else if(rollOutput < -AttOutputLimit)
		rollOutput = -AttOutputLimit;
	if(pitchOutput > AttOutputLimit)
		pitchOutput = AttOutputLimit;
	else if(pitchOutput < -AttOutputLimit)
		pitchOutput = -AttOutputLimit;
}

int16_t controllerAltHoldPID(float altitudeActual, float altitudeRateActual, float accelActual)
{
	int16_t output;
	
 	pidAlt.error = altitudeActual - pidAlt.desired;
 	pidAltRate.desired = -get_p(&pidAlt); //get altitude desired speed

 	if(pidAltRate.desired > AltRateLimit)
 		pidAltRate.desired = AltRateLimit;
 	else if(pidAltRate.desired < -AltRateLimit)
  		pidAltRate.desired = -AltRateLimit;

	pidAltRate.error = altitudeRateActual - pidAltRate.desired;
	pidAltAccel.desired = -get_pid(&pidAltRate);
	if(pidAltAccel.desired > AltAccelLimit)
		pidAltAccel.desired = AltAccelLimit;
	else if(pidAltAccel.desired < -AltAccelLimit)
		pidAltAccel.desired = -AltAccelLimit;
	
	pidAltAccel.error = accelActual - pidAltAccel.desired;
	output = -(int16_t)get_pid(&pidAltAccel) * 0.1;
	if(output > AltOutputLimit)
		output = AltOutputLimit;
	else if(output < -AltOutputLimit)
		output = -AltOutputLimit;
	
	return output;
}

void controllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void controllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = (int16_t)rollOutput;
  *pitch = (int16_t)pitchOutput;
  *yaw = (int16_t)yawOutput;
}












