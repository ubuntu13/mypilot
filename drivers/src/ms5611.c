#include "ms5611.h"

//float baro_buffer[10];
//#define baro_buffer_num   10
uint8_t exchange_Pres_num[8];
uint8_t exchange_Temp_num[8];

uint16_t Cal_C[7];
uint32_t D1_Pres = 8442380, D2_Temp = 0;
float Pressure = 0;
float dT = 0, Temperature = 0, Temperature2 = 0;
double OFF = 0, SENS = 0;

static float aslRaw = 0;
static float aslAlpha = 0.95;

float aslZero = 0;
float asl = 0;
float vspeedAsl = 0;

float altitude = 0;

void MS611_Write(uint8_t reg)
{
	ms5611_cs = 0;
	SPI2_ReadWriteByte(reg);
	ms5611_cs = 1;
}

uint16_t MS611_Read_16bit(uint8_t reg)
{
	uint8_t temp1, temp2;
	uint16_t value = 0;
	
	ms5611_cs = 0;
	SPI2_ReadWriteByte(reg);
	temp1 = SPI2_ReadWriteByte(0x00);
	temp2 = SPI2_ReadWriteByte(0x00);
	ms5611_cs = 1;
	
	value = (temp1<<8) + temp2;
	
	return value;
}

uint32_t MS611_Read_24bit(uint8_t reg)
{
	uint8_t temp1, temp2, temp3;
	uint32_t value = 0;
	
	ms5611_cs = 0;
	SPI2_ReadWriteByte(reg);
	temp1 = SPI2_ReadWriteByte(0x00);
	temp2 = SPI2_ReadWriteByte(0x00);
	temp3 = SPI2_ReadWriteByte(0x00);
	ms5611_cs = 1;
	
	value = (temp1<<16) + (temp2<<8) + temp3;
	
	return value;
}

void MS561101BA_RESET(void)
{
	MS611_Write(MS561101BA_RST);
}

void MS561101BA_PROM_READ(void)
{
	uint8_t i;
	
	for(i=0; i<=6; i++)
	{
		Cal_C[i] = MS611_Read_16bit(MS561101BA_PROM_RD + i * 2);
	}
}

void MS561101BA_Init(void)
{
	uint8_t i=0;

	MS561101BA_RESET();
	delay_ms(5); //must wait for 2.8ms, I set 5ms, enough.
	MS561101BA_PROM_READ();
	delay_ms(5);
	MS611_Write(MS561101BA_D2_OSR_4096);
	delay_ms(11);
	for(i=0; i<50; i++)
	{
		ms5611_update();
		delay_ms(11);
	}
	ms5611_calculate();
	asl = aslRaw;
	aslZero = asl;
}

#define ms5611_updatemode 0

void ms5611_update(void)
{
	static uint8_t state = 0;

#if ms5611_updatemode
	if(state == 0)
	{
		D2_Temp= MS611_Read_24bit(0x00);

		MS611_Write(MS561101BA_D1_OSR_4096);

		state++;
	}
	else
	{
		D1_Pres= MS611_Read_24bit(0x00);

		MS611_Write(MS561101BA_D2_OSR_4096);

		state = 0;
	}
#else
	if(state == 0)
	{
		D2_Temp= MS611_Read_24bit(0x00);
		state++;
		MS611_Write(MS561101BA_D1_OSR_4096);
	}
	else
	{
		D1_Pres= MS611_Read_24bit(0x00);
		state++;

		if(state == 5)
		{
			MS611_Write(MS561101BA_D2_OSR_4096);
			state = 0;
		}
		else
		{
			MS611_Write(MS561101BA_D1_OSR_4096);
		}
	}
#endif

	ms5611_calculate();
	asl_fliter();
}

void ms5611_calculate(void)
{
	float Aux, OFF2, SENS2;
	
	dT = D2_Temp - (((uint32_t)Cal_C[5])<<8);
	Temperature = 2000 + dT*((uint32_t)Cal_C[6])/8388608.0;
	
	OFF=(uint32_t)(Cal_C[2]<<16)+((uint32_t)Cal_C[4]*dT)/128.0;
	SENS=(uint32_t)(Cal_C[1]<<15)+((uint32_t)Cal_C[3]*dT)/256.0;
	
	if(Temperature < 2000)// second order temperature compensation when under 20 degrees C
	{
		Temperature2 = (dT*dT) / 0x80000000;
		Aux = (Temperature-2000)*(Temperature-2000);
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		if(Temperature < -1500)
		{
			Aux = (Temperature+1500)*(Temperature+1500);
			OFF2 = OFF2 + 7*Aux;
			SENS2 = SENS + 5.5*Aux;
		}
	}
	else  //(Temperature > 2000)
	{
		Temperature2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	
	Temperature = Temperature - Temperature2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	Pressure = ( (D1_Pres*SENS/2097152.0-OFF)/32768.0 );
	
	Pressure *= 0.01;
	Temperature *= 0.01;
	
	if(Pressure < ms5611_pmin)
		Pressure = ms5611_pmin;
	else if(Pressure > ms5611_pmax)
		Pressure = ms5611_pmax;
	if(Temperature < ms5611_tmin)
		Temperature = ms5611_tmin;
	else if(Temperature > ms5611_tmax)
		Temperature = ms5611_tmax;

	aslRaw = ((pow((1015.7 / Pressure), CONST_PF) - 1.0) * (Temperature + 273.15)) / 0.0065;
}

void asl_fliter(void)
{
	asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
	altitude = asl - aslZero;
}



















