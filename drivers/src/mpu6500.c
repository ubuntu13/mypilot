#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "mpu6500.h"
#include "spi.h"
#include "sensfusion6.h"
#include "led.h"

uint8_t read_id;

uint8_t mpu6500_Data_Buf[14] =
{
	'G','Y'
};

struct mpu mpu_data;

uint8_t mpu6500_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t status = 0;
	
	mpu6500_cs = 0;
	status = SPI2_ReadWriteByte(reg);
	SPI2_ReadWriteByte(value);
	mpu6500_cs = 1;
	
	return status;
}

uint8_t mpu6500_read_reg(uint8_t reg)
{
	uint8_t value = 0;
	
	mpu6500_cs = 0;
	SPI2_ReadWriteByte(reg | 0x80);
	value = SPI2_ReadWriteByte(0xff);
	mpu6500_cs = 1;
	
	return value;
}

uint8_t mpu6500_check(void)
{
	read_id = mpu6500_read_reg(WHO_AM_I);
	
	if(read_id == MPU6500_ID || read_id == MPU9250_ID) {return 0;}
	else {return 1;}
}

/**
 * @brief Writes one byte to the AK8963 register using MPU9250 I2C master
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * @returns 0 when success
 */
static int32_t PIOS_MPU9250_Mag_WriteReg(uint8_t reg, uint8_t data)
{
	uint32_t timeout = 0;
	uint8_t status = 0;
	// we will use I2C SLV4 to manipulate with AK8963 control registers
	if (mpu6500_write_reg(PIOS_MPU60X0_SLV4_REG_REG, reg) != 0)
		return -1;
	mpu6500_write_reg(PIOS_MPU60X0_SLV4_ADDR_REG, PIOS_MPU9250_AK8963_ADDR);
	mpu6500_write_reg(PIOS_MPU60X0_SLV4_DO_REG, data);
	mpu6500_write_reg(PIOS_MPU60X0_SLV4_CTRL_REG, PIOS_MPU60X0_I2CSLV_EN);


	// wait for I2C transaction done, use simple safety
	// escape counter to prevent endless loop in case
	// MPU9250 is broken

	do {
		if (timeout++ > 50)
			return -2;

		status = mpu6500_read_reg(PIOS_MPU60X0_I2C_MST_STATUS_REG);
	} while ((status & PIOS_MPU60X0_I2C_MST_SLV4_DONE) == 0);

	if (status & PIOS_MPU60X0_I2C_MST_SLV4_NACK)
		return -3;

	return 0;
}

/**
 * @brief Reads one byte from the AK8963 register using MPU9250 I2C master
 * \param[in] reg Register address
 * \param[in] data Byte to write
 */
static uint8_t PIOS_MPU9250_Mag_ReadReg(uint8_t reg)
{
	uint32_t timeout = 0;
	uint8_t status = 0;
	// we will use I2C SLV4 to manipulate with AK8963 control registers
	mpu6500_write_reg(PIOS_MPU60X0_SLV4_REG_REG, reg);
	mpu6500_write_reg(PIOS_MPU60X0_SLV4_ADDR_REG, PIOS_MPU9250_AK8963_ADDR | 0x80);
	mpu6500_write_reg(PIOS_MPU60X0_SLV4_CTRL_REG, PIOS_MPU60X0_I2CSLV_EN);


	// wait for I2C transaction done, use simple safety
	// escape counter to prevent endless loop in case
	// MPU9250 is broken

	do {
		if (timeout++ > 50)
			return 0;

		status = mpu6500_read_reg(PIOS_MPU60X0_I2C_MST_STATUS_REG);
	} while ((status & PIOS_MPU60X0_I2C_MST_SLV4_DONE) == 0);

	return mpu6500_read_reg(PIOS_MPU60X0_SLV4_DI_REG);
}

/**
 * @brief Initialize the AK8963 magnetometer inside MPU9250
 * \return 0 if success
 *
 */
static int32_t PIOS_MPU9250_Mag_Config(void)
{
	uint8_t id = PIOS_MPU9250_Mag_ReadReg(AK8963_WHOAMI_REG);
	if(id != AK8963_WHOAMI_ID)
		return -2;

	// reset AK8963
	if (PIOS_MPU9250_Mag_WriteReg(AK8963_CNTL2_REG, AK8963_CNTL2_SRST) != 0)
		return -3;

	// give chip some time to initialize
	delay_ms(2);

	// set magnetometer sampling rate to 100Hz and 16-bit resolution
	PIOS_MPU9250_Mag_WriteReg(AK8963_CNTL1_REG, AK8963_MODE_CONTINUOUS_FAST_16B);

	// configure mpu9250 to read ak8963 data range from STATUS1 to STATUS2 at ODR
	mpu6500_write_reg(PIOS_MPU60X0_SLV0_REG_REG, AK8963_ST1_REG);
	mpu6500_write_reg(PIOS_MPU60X0_SLV0_ADDR_REG, PIOS_MPU9250_AK8963_ADDR | 0x80);
	mpu6500_write_reg(PIOS_MPU60X0_SLV0_CTRL_REG, PIOS_MPU60X0_I2CSLV_EN | 8);

	return 0;
}

void mpu6500_init(void)
{
	static uint8_t mag_status = 0;

	mpu6500_write_reg(PWR_MGMT_1, 0x80);
	delay_ms(100);
	mpu6500_write_reg(PWR_MGMT_1, 0x03);
	mpu6500_write_reg(PWR_MGMT_2, 0x00);

	if(read_id == MPU9250_ID)
	{
		// user control
		mpu6500_write_reg(PIOS_MPU60X0_USER_CTRL_REG, PIOS_MPU60X0_USERCTL_DIS_I2C | PIOS_MPU60X0_USERCTL_I2C_MST_EN);
		delay_ms(10);
		mag_status = PIOS_MPU9250_Mag_Config();

		while(mag_status)
		{
			LED1 = ~LED1;
			delay_ms(500);
		}
	}

	mpu6500_write_reg(SMPLRT_DIV, 0x04);
	mpu6500_write_reg(CONFIG, 0x04);
	mpu6500_write_reg(GYRO_CONFIG, 0x18);
	mpu6500_write_reg(ACCEL_CONFIG, 0x08);
	mpu6500_write_reg(ACCEL_CONFIG2, 0x02);
}

int16_t getdata(uint8_t reg)
{
	uint8_t H = 0, L = 0;
	int16_t data = 0;

	H = mpu6500_read_reg(reg);
	L = mpu6500_read_reg(reg + 1);

	data =  ( H << 8 ) + L;

	return data;
}

void accel_data_update(void)
{
	mpu_data.AX = getdata(ACCEL_XOUT) * 0.001196289f;
	mpu_data.AY = getdata(ACCEL_YOUT) * 0.001196289f;
	mpu_data.AZ = getdata(ACCEL_ZOUT) * 0.001196289f;

	if(read_id == MPU9250_ID)
	{
		static uint8_t mag;
		mag = PIOS_MPU9250_Mag_ReadReg(0x03);
		mag = PIOS_MPU9250_Mag_ReadReg(0x04);
		mag = PIOS_MPU9250_Mag_ReadReg(0x05);
		mag = PIOS_MPU9250_Mag_ReadReg(0x06);
	}
}

void gyro_data_update(void)
{
	mpu_data.GX = getdata(GYRO_XOUT);
	mpu_data.GY = getdata(GYRO_YOUT);
	mpu_data.GZ = getdata(GYRO_ZOUT);
}

