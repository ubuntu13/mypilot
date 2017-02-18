#ifndef _mpu6500_H
#define _mpu6500_H

#include "sys.h"

#define        MPU6500_ID          0x70
#define        MPU9250_ID          0x71

#define 	   PIOS_MPU60X0_SLV0_ADDR_REG        0x25
#define 	   PIOS_MPU60X0_SLV0_REG_REG         0x26
#define 	   PIOS_MPU60X0_SLV0_CTRL_REG        0x27
#define        PIOS_MPU60X0_SLV4_ADDR_REG        0x31
#define        PIOS_MPU60X0_SLV4_REG_REG         0x32
#define        PIOS_MPU60X0_SLV4_DO_REG          0x33
#define        PIOS_MPU60X0_SLV4_CTRL_REG        0x34
#define        PIOS_MPU60X0_SLV4_DI_REG          0x35
#define        PIOS_MPU60X0_I2C_MST_STATUS_REG   0x36
#define        PIOS_MPU60X0_INT_CFG_REG          0x37
#define        PIOS_MPU60X0_INT_EN_REG           0x38
#define        PIOS_MPU60X0_INT_STATUS_REG       0x3A

/* I2C master status register bits */
#define PIOS_MPU60X0_I2C_MST_SLV4_DONE    0x40
#define PIOS_MPU60X0_I2C_MST_LOST_ARB     0x20
#define PIOS_MPU60X0_I2C_MST_SLV4_NACK    0x10
#define PIOS_MPU60X0_I2C_MST_SLV0_NACK    0x01

/* I2C SLV register bits */
#define PIOS_MPU60X0_I2CSLV_EN            0x80
#define PIOS_MPU60X0_I2CSLV_BYTE_SW       0x40
#define PIOS_MPU60X0_I2CSLV_REG_DIS       0x20
#define PIOS_MPU60X0_I2CSLV_GRP           0x10

/* User control functionality */
#define PIOS_MPU60X0_USERCTL_FIFO_EN      0X40
#define PIOS_MPU60X0_USERCTL_I2C_MST_EN   0X20
#define PIOS_MPU60X0_USERCTL_DIS_I2C      0X10
#define PIOS_MPU60X0_USERCTL_FIFO_RST     0X02
#define PIOS_MPU60X0_USERCTL_GYRO_RST     0X01

#define PIOS_MPU60X0_USER_CTRL_REG        0x6A

#define        SMPLRT_DIV          0x19
#define        CONFIG              0x1A
#define        GYRO_CONFIG         0x1B
#define        ACCEL_CONFIG        0x1C
#define        ACCEL_CONFIG2       0x1D
#define        ACCEL_XOUT_H        0x3B
#define        ACCEL_XOUT_L        0x3C
#define        ACCEL_YOUT_H        0x3D
#define        ACCEL_YOUT_L        0x3E
#define        ACCEL_ZOUT_H        0x3F
#define        ACCEL_ZOUT_L        0x40
#define        TEMP_OUT_H          0x41
#define        TEMP_OUT_L          0x42
#define        GYRO_XOUT_H         0x43
#define        GYRO_XOUT_L         0x44
#define        GYRO_YOUT_H         0x45
#define        GYRO_YOUT_L         0x46
#define        GYRO_ZOUT_H         0x47
#define        GYRO_ZOUT_L         0x48

#define        ACCEL_XOUT          ACCEL_XOUT_H
#define        ACCEL_YOUT          ACCEL_YOUT_H
#define        ACCEL_ZOUT          ACCEL_ZOUT_H

#define        GYRO_XOUT           GYRO_XOUT_H
#define        GYRO_YOUT           GYRO_YOUT_H
#define        GYRO_ZOUT           GYRO_ZOUT_H

#define        TEMP_OUT            TEMP_OUT_H

#define        PWR_MGMT_1          0x6B
#define        PWR_MGMT_2          0x6C
#define    	   WHO_AM_I            0x75

#define PIOS_MPU9250_AK8963_ADDR     	    0x0C
#define AK8963_WHOAMI_REG                   0x00
#define AK8963_WHOAMI_ID                    0x48
#define AK8963_ST1_REG                      0x02
#define AK8963_ST2_REG                      0x09
#define AK8963_ST1_DOR                      0x02
#define AK8963_ST1_DRDY                     0x01
#define AK8963_ST2_BITM                     0x10
#define AK8963_ST2_HOFL                     0x08
#define AK8963_CNTL1_REG                    0x0A
#define AK8963_CNTL2_REG                    0x0A
#define AK8963_CNTL2_SRST                   0x01
#define AK8963_MODE_CONTINUOUS_FAST_16B     0x16

struct mpu
{
	float GX;
	float GY;
	float GZ;
	float AX;
	float AY;
	float AZ;
	float AM;
	float Ave_AX;
	float Ave_AY;
	float Ave_AZ;
};
extern struct mpu mpu_data;

struct S16_XYZ
{
	s16 X;
	s16 Y;
	s16 Z;
};

struct S32_XYZ
{
	s32 X;
	s32 Y;
	s32 Z;
};

struct F_XYZ
{
	float X;
	float Y;
	float Z;
};

uint8_t mpu6500_write_reg(uint8_t reg, uint8_t value);
uint8_t mpu6500_read_reg(uint8_t reg);
u8 mpu6500_check(void);
void mpu6500_init(void);
short getdata(u8 reg);
void accel_data_update(void);
void gyro_data_update(void);

#endif
