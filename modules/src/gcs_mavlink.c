#include "gcs_mavlink.h"

#include "led.h"
#include "spi.h"
#include "com.h"
#include "motors.h"
#include "flash.h"

#include "rc.h"
#include "mavlink.h"
#include "common.h"
#include "usb_com.h"
#include "sensfusion6.h"
#include "controller.h"
#include "timer.h"
#include "string.h"
#include "sensor.h"
#include "mpu6500.h"
#include "inav.h"
#include "stabilizer.h"

#include "usb_com.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"

#include "FreeRTOS.h"
#include "task.h"

/******************************encode**********************************/
void gcs_mavlink_sendmsg_ping(void)
{
	uint8_t len;
	mavlink_message_t msg_ping;

	mavlink_msg_ping_pack(MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO1, &msg_ping, 0, 4382, MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO2);

	len = msg_ping.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	uint8_t com_buffer[len];
	mavlink_msg_to_send_buffer(com_buffer, &msg_ping);

	usbcom_send(com_buffer, len);
}

void gcd_mavlink_sendmsg_heartbeat(void)
{
	uint8_t len;
	mavlink_message_t msg_heartbeat;

	mavlink_msg_heartbeat_pack(MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO1, &msg_heartbeat,
			MAV_TYPE_QUADROTOR,  MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_GUIDED_ENABLED,
			0, MAV_STATE_UNINIT);

	len = msg_heartbeat.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	uint8_t com_buffer[len];
	mavlink_msg_to_send_buffer(com_buffer, &msg_heartbeat);

	usbcom_send(com_buffer, len);
}

void gcd_mavlink_sendmsg_8dof(float data1, float data2, float data3, float data4,
		float data5, float data6, float data7, float data8)
{
	uint8_t len;
	mavlink_message_t msg_8dof;

	mavlink_msg_setpoint_8dof_pack(MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO1, &msg_8dof, MAV_AUTOPILOT_GENERIC,
			 data1, data2, data3, data4, data5, data6, data7, data8);

	len = msg_8dof.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	uint8_t com_buffer[len];
	mavlink_msg_to_send_buffer(com_buffer, &msg_8dof);

	usbcom_send(com_buffer, len);
}

void gcs_mavlink_sendmsg_attitude(void)
{
	uint8_t len;
	mavlink_message_t msg_attitude;

	mavlink_msg_attitude_pack(MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO1, &msg_attitude, 0,
			Attitude.Roll, Attitude.Pitch, Attitude.Yaw, 4, 5, 6);

	len = msg_attitude.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	uint8_t com_buffer[len];
	mavlink_msg_to_send_buffer(com_buffer, &msg_attitude);

	usbcom_send(com_buffer, len);
}

void gcs_mavlink_sendmsg_RollPid(void)
{
	uint8_t len;
	mavlink_message_t msg_pid;

	mavlink_msg_setpoint_8dof_pack(MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO1, &msg_pid, 0, 1, 1, pidRoll.kp, pidRoll.ki, pidRoll.kd, pidRollRate.kp, pidRollRate.ki, pidRollRate.kd);

	len = msg_pid.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	uint8_t com_buffer[len];
	mavlink_msg_to_send_buffer(com_buffer, &msg_pid);

	usbcom_send(com_buffer, len);
}

void gcs_mavlink_sendmsg_PitchPid(void)
{
	uint8_t len;
	mavlink_message_t msg_pid;

	mavlink_msg_setpoint_8dof_pack(MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO1, &msg_pid, 0, 1, 2, pidPitch.kp, pidPitch.ki, pidPitch.kd, pidPitchRate.kp, pidPitchRate.ki, pidPitchRate.kd);

	len = msg_pid.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	uint8_t com_buffer[len];
	mavlink_msg_to_send_buffer(com_buffer, &msg_pid);

	usbcom_send(com_buffer, len);
}

void gcs_mavlink_sendmsg_YawPid(void)
{
	uint8_t len;
	mavlink_message_t msg_pid;

	mavlink_msg_setpoint_8dof_pack(MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO1, &msg_pid, 0, 1, 3, pidYaw.kp, pidYaw.ki, pidYaw.kd, pidYawRate.kp, pidYawRate.ki, pidYawRate.kd);

	len = msg_pid.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	uint8_t com_buffer[len];
	mavlink_msg_to_send_buffer(com_buffer, &msg_pid);

	usbcom_send(com_buffer, len);
}

void gcs_mavlink_sendmsg_altitude(void)
{
	uint8_t len;
	mavlink_message_t msg_altitude;

	//mavlink_msg_setpoint_8dof_pack(MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO1, &msg_altitude, 0, 3, 1, _position_error_z, _position_correction_z, _position_base_z, 0, 0, 0);
	mavlink_msg_setpoint_8dof_pack(MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO1, &msg_altitude, 0, 3, 1, _position_z, _velocity_z, altitude*100, 0, 0, 0);

	len = msg_altitude.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	uint8_t com_buffer[len];
	mavlink_msg_to_send_buffer(com_buffer, &msg_altitude);

	usbcom_send(com_buffer, len);
}

void gcs_mavlink_sendmsg_rc_channels_scaled(void)
{
	uint8_t len;
	mavlink_message_t msg_rc;

	mavlink_msg_rc_channels_scaled_pack(MAV_TYPE_GENERIC, MAV_COMP_ID_SERVO1, &msg_rc, 0, 1,
			chList[0], chList[1], chList[2], chList[3], chList[4], chList[5], chList[6], chList[7], 80);

	len = msg_rc.len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	uint8_t com_buffer[len];
	mavlink_msg_to_send_buffer(com_buffer, &msg_rc);

	usbcom_send(com_buffer, len);
}

/********************************decode*************************************/

uint32_t mavlink_ping_decode( uint8_t buffer[] )
{
	mavlink_message_t msg_get;
	mavlink_ping_t data_get;

	memcpy(&msg_get.magic, buffer, buffer[1] + MAVLINK_NUM_NON_PAYLOAD_BYTES);

	mavlink_msg_ping_decode(&msg_get, &data_get);

	if(data_get.target_system == MAV_TYPE_GENERIC && data_get.target_component == MAV_COMP_ID_SERVO1)
	{
		return data_get.seq;
	}

	return 0;
}

const char auth_key[32] = "19491001";
uint8_t mavlink_auth_key_decode( uint8_t buffer[] )
{
	mavlink_message_t msg_get;
	mavlink_auth_key_t data_get;

	memcpy(&msg_get.magic, buffer, buffer[1] + MAVLINK_NUM_NON_PAYLOAD_BYTES);

	mavlink_msg_auth_key_decode(&msg_get, &data_get);

	if( strcmp(data_get.key, auth_key) == 0 )
	{
		return 1;
	}

	return 0;
}

uint8_t mavlink_8dof_decode( uint8_t buffer[] )
{
	mavlink_message_t msg_get;
	mavlink_setpoint_8dof_t data_get;

	memcpy(&msg_get.magic, buffer, buffer[1] + MAVLINK_NUM_NON_PAYLOAD_BYTES);

	mavlink_msg_setpoint_8dof_decode(&msg_get, &data_get);

	if(data_get.val1 == 1)
	{
		if(data_get.val2 == 1)
		{
			memcpy(&pidRoll.kp, &data_get.val3, 24);
			memcpy(&pidRollRate.kp, &data_get.val6, 24);
		}
		else if(data_get.val2 == 2)
		{
			memcpy(&pidPitch.kp, &data_get.val3, 24);
			memcpy(&pidPitchRate.kp, &data_get.val6, 24);
		}
		else if(data_get.val2 == 3)
		{
			memcpy(&pidYaw.kp, &data_get.val3, 24);
			memcpy(&pidYawRate.kp, &data_get.val6, 24);

			return 1;
		}
	}
	else if(data_get.val1 == 2) //gcs read pid
	{
		return 2;
	}
	else if(data_get.val1 == 4)
	{
		if(data_get.val2 == 0)
		{
			return 3;
		}
	}
	else if(data_get.val1 == 5)
	{
		if(data_get.val2 == 1)
		{
			if(bDeviceState != UNCONNECTED)
			{
				motors_set_frequency( (uint16_t)data_get.val2 );
			}
		}
	}
	else if(data_get.val1 == 6)
	{
		if(data_get.val2 == 1)
		{
			maxchList[0] = (uint16_t)data_get.val3;
			maxchList[1] = (uint16_t)data_get.val4;
			maxchList[2] = (uint16_t)data_get.val5;
			maxchList[3] = (uint16_t)data_get.val6;
			maxchList[4] = (uint16_t)data_get.val7;
			maxchList[5] = (uint16_t)data_get.val8;
		}
		else if(data_get.val2 == 2)
		{
			minchList[0] = (uint16_t)data_get.val3;
			minchList[1] = (uint16_t)data_get.val4;
			minchList[2] = (uint16_t)data_get.val5;
			minchList[3] = (uint16_t)data_get.val6;
			minchList[4] = (uint16_t)data_get.val7;
			minchList[5] = (uint16_t)data_get.val8;
		}
		else if(data_get.val2 == 3)
		{
			maxchList[6] = (uint16_t)data_get.val3;
			maxchList[7] = (uint16_t)data_get.val4;
			minchList[6] = (uint16_t)data_get.val5;
			minchList[7] = (uint16_t)data_get.val6;

			return 4;
		}
	}

	return 0;
}

void mavlink_quad_motors_decode( uint8_t buffer[] )
{
	mavlink_message_t msg_get;
	mavlink_set_quad_motors_setpoint_t data_get;

	memcpy(&msg_get.magic, buffer, buffer[1] + MAVLINK_NUM_NON_PAYLOAD_BYTES);

	mavlink_msg_set_quad_motors_setpoint_decode(&msg_get, &data_get);

	MOTOR1 = 1000 + data_get.motor_front_nw;
	MOTOR2 = 1000 + data_get.motor_right_ne;
	MOTOR3 = 1000 + data_get.motor_back_se;
	MOTOR4 = 1000 + data_get.motor_left_sw;
}









