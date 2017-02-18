#ifndef __SPI_H
#define __SPI_H

#include "sys.h"

#define 	mpu6500_cs 	PAout(8)
#define 	ms5611_cs  	PBout(5)
#define 	w25x16_cs  	PBout(12)

void SPI2_Init(void);
void SPI2_SetSpeed(uint8_t SpeedSet);
uint8_t SPI2_ReadWriteByte(uint8_t TxData);
		 
#endif

