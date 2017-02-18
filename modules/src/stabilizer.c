#include "stabilizer.h"
#include "controller.h"
#include "sensfusion6.h"
#include "inav.h"
#include "filter.h"
#include "sensor.h"
#include "ms5611.h"
#include "motors.h"
#include "mpu6500.h"
#include "sensor.h"
#include "rc.h"
#include "timer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define 	ATTITUDE_UPDATE_DT 	2
#define 	ALTHOLD_UPDATE_DT   10
#define 	IMU_UPDATE_DT 		2/portTICK_RATE_MS

extern uint8_t unlock;
extern float awg;
extern float aslZero, asl;
extern float Gravity_mss;

int16_t M_Thr = 1000;
int16_t M_Roll = 0, M_Pitch = 0, M_Yaw = 0;

static void stabilizerTask(void *pvParameters);
xTaskHandle stabilizerHandle;

void stabilizerInit(void)
{
	controller_init();

    xTaskCreate(stabilizerTask, (const signed char * const)"STABILIZER",
              2*configMINIMAL_STACK_SIZE, NULL, 2, &stabilizerHandle);
}

static void stabilizerTask(void *pvParameters)
{
	uint32_t attitudeCounter = 0;
	uint32_t altHoldCounter = 0;
	uint32_t lastWakeTime;

	lastWakeTime = xTaskGetTickCount();

	while(true)
	{
		vTaskDelayUntil(&lastWakeTime, IMU_UPDATE_DT); // 500Hz

		rc_get_and_filter();

		sensor_update();
		IMU_Prepare();

		if(accel.calibrated && gyro.calibrated)
		{
			//250HZ
			if(++attitudeCounter >= ATTITUDE_UPDATE_DT)
			{
				attitudeCounter = 0;

				sensfusion6_update(Fil_Gyro.X, Fil_Gyro.Y, Fil_Gyro.Z, Fil_Accel.X, Fil_Accel.Y, Fil_Accel.Z);

				if(unlock)
				{
					rc_get_desired();

					if(M_Thr > (minchList[2]+100))
					{
						controller_3Axis_AttitudePID(Attitude.Roll, Attitude.Pitch, Attitude.Yaw, -mpu_data.GY * 0.060975, mpu_data.GX * 0.060975, mpu_data.GZ * 0.060975);

						controllerGetActuatorOutput(&M_Roll, &M_Pitch, &M_Yaw);
					}
				}
				else
				{
					controllerResetAllPID();
					M_Thr = 1000;
					M_Roll = 0;
					M_Pitch = 0;
					M_Yaw = 0;
				}
			}

			// 100HZ
			if(++altHoldCounter >= ALTHOLD_UPDATE_DT)
			{
				altHoldCounter = 0;

				ms5611_update();
				awg = sensfusion6GetAccZWithoutGravity(Fil_Accel.X, Fil_Accel.Y, Fil_Accel.Z);

				static uint8_t cnt = 0;
				static float tmp = 0;

				if(cnt == 200)
				{
					aslZero = asl;
					InertialNav_update(0.01);
					Gravity_mss += tmp / 100;

					cnt++;
				}
				else if(cnt > 200)
				{
					InertialNav_update(0.01);
				}
				else
				{
					cnt++;
					if(cnt > 100)
					{
						tmp += awg;
					}
				}
			}

			motors_update(M_Thr, M_Pitch, M_Roll, M_Yaw);
		}
	}
}



