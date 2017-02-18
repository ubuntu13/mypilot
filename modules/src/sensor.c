#include "sensor.h"
#include "usart.h"
#include "usb_com.h"
#include "delay.h"
#include "led.h"
#include "mpu6500.h"
#include "flash.h"
#include "stdbool.h"
#include <math.h>
#include <string.h>
#include "gcs_mavlink.h"
#include "stdbool.h"

#include "com.h"
#include "stabilizer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define	 	GRAVITY_MSS 		9.800f
#define 	SENSORCALI_UPDATE   100/portTICK_RATE_MS

struct Vector3f accel_sample[6];
struct Vector3f accel_offsets;
struct Vector3f accel_scale;
struct Vector3f gyro_offsets;

struct sensor accel;
struct sensor gyro;

xTaskHandle sensorCaliHandle;
static void sensorCaliTask(void *pvParameters);
void save_parameters(void);

void sensorTaskInit(void)
{
    xTaskCreate(sensorCaliTask, (const signed char * const)"sensorCali",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/4, &sensorCaliHandle);
}

void sensor_init(void)
{
	uint8_t *read_buffer = pvPortMalloc(60);

	mpu6500_init();

	SPI_Flash_Read(read_buffer, flash_address_sensor, 60);

	delay_ms(100);

	//sensor_calibrate_gyro();
	//gyro.calibrated = true;

	if(read_buffer[0] != true)
	{
		accel.calibrated = false;
		gyro.calibrated = false;
		LED1 = 1;
	}
	else
	{
		memcpy(&accel_scale, read_buffer+1, 12);
		memcpy(&accel_offsets, read_buffer+13, 12);
		memcpy(&gyro_offsets, read_buffer+25, 12);

		accel.calibrated = true;
		gyro.calibrated = true;
		LED1 = 0;
	}

	vPortFree( read_buffer );
}

void sensor_update(void)
{
	accel_data_update();
	accel.x = accel_scale.x * mpu_data.AX - accel_offsets.x;
	accel.y = accel_scale.y * mpu_data.AY - accel_offsets.y;
	accel.z = accel_scale.z * mpu_data.AZ - accel_offsets.z;

	gyro_data_update();
	gyro.x = mpu_data.GX - gyro_offsets.x;
	gyro.y = mpu_data.GY - gyro_offsets.y;
	gyro.z = mpu_data.GZ - gyro_offsets.z;
}

void sensor_calibrate_gyro(void)
{
	uint8_t i;
	struct Vector3f gyro_sum;

	gyro_sum.x = 0;
	gyro_sum.y = 0;
	gyro_sum.z = 0;

	gyro_data_update();

	for(i=0; i<200; i++)
	{
		gyro_sum.x += mpu_data.GX;
		gyro_sum.y += mpu_data.GY;
		gyro_sum.z += mpu_data.GZ;
	}

	gyro_offsets.x = gyro_sum.x / i;
	gyro_offsets.y = gyro_sum.y / i;
	gyro_offsets.z = gyro_sum.z / i;
}

void sensor_calibrate_accel_init(void)
{
	uint8_t i;

	for(i=0; i<6; i++)
	{
		accel_sample[i].x = 0;
		accel_sample[i].y = 0;
		accel_sample[i].z = 0;
	}

	accel_offsets.x = 0;
	accel_offsets.y = 0;
	accel_offsets.z = 0;
	accel_scale.x = 0;
	accel_scale.y = 0;
	accel_scale.z = 0;
}

void save_parameters(void)
{
	uint8_t *write_buffer = pvPortMalloc(37);

	accel.calibrated = true;
	gyro.calibrated = true;
	write_buffer[0] = 1;
	memcpy(write_buffer+1, &accel_scale, 12);
	memcpy(write_buffer+13, &accel_offsets, 12);
	memcpy(write_buffer+25, &gyro_offsets, 12);

	SPI_Flash_Write(write_buffer, flash_address_sensor, 37);

	vPortFree( write_buffer );
}

// calibrate_accel - perform accelerometer calibration including providing user
// instructions and feedback Gauss-Newton accel calibration routines borrowed
// from Rolfe Schmidt blog post describing the method:
// http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
// original sketch available at
// http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
uint8_t calibrate_continue = 0;
extern uint8_t comm_process_en;
bool sensor_calibrate_accel(void)
{
	sensor_calibrate_accel_init();

    // capture data from 6 positions
    for (uint8_t i=0; i<6; i++)
    {
        // wait for user input
        while( comm_process_en == 0 )
        {
        	gcd_mavlink_sendmsg_8dof(0, 0, 0, 0, 0, 0, 0, 0);
        	delay_ms(20);
        	LED1 = !LED1;
        }
        comm_process_en = 0;
        LED3 = !LED3;

        // average 32 samples
        uint8_t num_samples = 0;

        while (num_samples < 32)
        {
            // read samples from ins
        	accel_data_update();

            // capture sample
        	accel_sample[i].x += mpu_data.AX;
        	accel_sample[i].y += mpu_data.AY;
        	accel_sample[i].z += mpu_data.AZ;

        	delay_ms(2);
            num_samples++;
        }

        accel_sample[i].x /= num_samples;
        accel_sample[i].y /= num_samples;
        accel_sample[i].z /= num_samples;
    }

    // run the calibration routine
    if( sensor_calibrate_accel_compute() == false)
    {
    	gcd_mavlink_sendmsg_8dof(4, 41, 0, 0, 0, 0, 0, 0);

    	return false;
    }

    gcd_mavlink_sendmsg_8dof(4, 40, 0, 0, 0, 0, 0, 0);

    save_parameters();

    // calculate the trims as well from primary accels and pass back to caller
    //_calculate_trim(samples[0][0], trim_roll, trim_pitch);

    return true;
}

static void sensorCaliTask(void *pvParameters)
{
	uint32_t lastWakeTime;

	lastWakeTime = xTaskGetTickCount();

	sensor_calibrate_gyro();

	sensor_calibrate_accel_init();

	while(true)
	{
		vTaskDelayUntil(&lastWakeTime, SENSORCALI_UPDATE);

		// capture data from 6 positions
		for (uint8_t i=0; i<6; i++)
		{
			// wait for user input
		    while( comm_process_en == 0 )
		    {
		    	gcd_mavlink_sendmsg_8dof(0, 0, 0, 0, 0, 0, 0, 0);
		    	vTaskDelay( 50/portTICK_RATE_MS );
		        LED1 = !LED1;
		    }
		    comm_process_en = 0;
		    LED3 = !LED3;

		    // average 32 samples
		    uint8_t num_samples = 0;

		    while (num_samples < 32)
		    {
		    	// read samples from ins
		        accel_data_update();

		        // capture sample
		        accel_sample[i].x += mpu_data.AX;
		        accel_sample[i].y += mpu_data.AY;
		        accel_sample[i].z += mpu_data.AZ;

		        vTaskDelay( 2/portTICK_RATE_MS );
		        num_samples++;
		    }

		    accel_sample[i].x /= num_samples;
		    accel_sample[i].y /= num_samples;
		    accel_sample[i].z /= num_samples;
		}

		// run the calibration routine
		if( sensor_calibrate_accel_compute() == false)
		{
		    gcd_mavlink_sendmsg_8dof(4, 41, 0, 0, 0, 0, 0, 0);
		}
		else
		{
			gcd_mavlink_sendmsg_8dof(4, 40, 0, 0, 0, 0, 0, 0);
			save_parameters();
		}

		comTaskInit();
		stabilizerInit();
		vTaskDelete(sensorCaliHandle);
	}
}

// _calibrate_model - perform low level accel calibration
// accel_sample are accelerometer samples collected in 6 different positions
// accel_offsets are output from the calibration routine
// accel_scale are output from the calibration routine
// returns true if successful
bool sensor_calibrate_accel_compute(void)
{
    int16_t i;
    int16_t num_iterations = 0;
    float eps = 0.000000001;
    float change = 100.0;
    float data[3];
    float beta[6];
    float delta[6];
    float ds[6];
    float JS[6][6];
    bool success = true;

    // reset
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;

    while( num_iterations < 20 && change > eps )
    {
        num_iterations++;

        sensor_calibrate_reset_matrices(ds, JS);

        for( i=0; i<6; i++ )
        {
            data[0] = accel_sample[i].x;
            data[1] = accel_sample[i].y;
            data[2] = accel_sample[i].z;
            sensor_calibrate_update_matrices(ds, JS, beta, data);
        }

        sensor_calibrate_find_delta(ds, JS, delta);

        change =    delta[0]*delta[0] +
                    delta[0]*delta[0] +
                    delta[1]*delta[1] +
                    delta[2]*delta[2] +
                    delta[3]*delta[3] / (beta[3]*beta[3]) +
                    delta[4]*delta[4] / (beta[4]*beta[4]) +
                    delta[5]*delta[5] / (beta[5]*beta[5]);

        for( i=0; i<6; i++ )
        {
            beta[i] -= delta[i];
        }
    }

    // copy results out
    accel_scale.x = beta[3] * GRAVITY_MSS;
    accel_scale.y = beta[4] * GRAVITY_MSS;
    accel_scale.z = beta[5] * GRAVITY_MSS;
    accel_offsets.x = beta[0] * accel_scale.x;
    accel_offsets.y = beta[1] * accel_scale.y;
    accel_offsets.z = beta[2] * accel_scale.z;

    // sanity check scale
    if( fabsf(accel_scale.x-1.0f) > 0.1f || fabsf(accel_scale.y-1.0f) > 0.1f || fabsf(accel_scale.z-1.0f) > 0.1f )
    {
        success = false;
    }
    // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
    if( fabsf(accel_offsets.x) > 3.5f || fabsf(accel_offsets.y) > 3.5f || fabsf(accel_offsets.z) > 3.5f )
    {
        success = false;
    }

    // return success or failure
    return success;
}

void sensor_calibrate_update_matrices(float dS[6], float JS[6][6],
                                    float beta[6], float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];

    for( j=0; j<3; j++ )
    {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }

    for( j=0; j<6; j++ )
    {
        dS[j] += jacobian[j]*residual;
        for( k=0; k<6; k++ )
        {
            JS[j][k] += jacobian[j]*jacobian[k];
        }
    }
}

// _calibrate_reset_matrices - clears matrices
void sensor_calibrate_reset_matrices(float dS[6], float JS[6][6])
{
    int16_t j,k;
    for( j=0; j<6; j++ )
    {
        dS[j] = 0.0f;
        for( k=0; k<6; k++ )
        {
            JS[j][k] = 0.0f;
        }
    }
}

void sensor_calibrate_find_delta(float dS[6], float JS[6][6], float delta[6])
{
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int16_t i,j,k;
    float mu;

    //make upper triangular
    for( i=0; i<6; i++ )
    {
        //eliminate all nonzero entries below JS[i][i]
        for( j=i+1; j<6; j++ )
        {
            mu = JS[i][j]/JS[i][i];
            if( mu != 0.0f )
            {
                dS[j] -= mu*dS[i];
                for( k=j; k<6; k++ )
                {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }

    //back-substitute
    for( i=5; i>=0; i-- )
    {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;

        for( j=0; j<i; j++ )
        {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }

    for( i=0; i<6; i++ )
    {
        delta[i] = dS[i];
    }
}





