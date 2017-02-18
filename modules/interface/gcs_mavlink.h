#ifndef __GCS_MAVLINK_H
#define __GCS_MAVLINK_H

#include "sys.h"

void gcs_mavlink_sendmsg_ping(void);
void gcd_mavlink_sendmsg_heartbeat(void);
void gcd_mavlink_sendmsg_8dof(float data1, float data2, float data3, float data4,
		float data5, float data6, float data7, float data8);
void gcs_mavlink_sendmsg_attitude(void);
void gcs_mavlink_sendmsg_RollPid(void);
void gcs_mavlink_sendmsg_PitchPid(void);
void gcs_mavlink_sendmsg_YawPid(void);
void gcs_mavlink_sendmsg_altitude(void);
void gcs_mavlink_sendmsg_rc_channels_scaled(void);

uint32_t mavlink_ping_decode( uint8_t buffer[] );
uint8_t mavlink_auth_key_decode( uint8_t buffer[] );
uint8_t mavlink_8dof_decode( uint8_t buffer[]);
void mavlink_quad_motors_decode( uint8_t buffer[] );

#endif

