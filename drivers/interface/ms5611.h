#ifndef __MS5611_H__
#define __MS5611_H__


#include "sys.h"
#include "spi.h"
#include "delay.h"
#include "usart.h"
#include "filter.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define  MS561101BA_D1 0x40 
#define  MS561101BA_D2 0x50 
#define  MS561101BA_RST 0x1E 

//#define  MS561101BA_D1_OSR_256 0x40 
//#define  MS561101BA_D1_OSR_512 0x42 
//#define  MS561101BA_D1_OSR_1024 0x44 
//#define  MS561101BA_D1_OSR_2048 0x46 
#define  MS561101BA_D1_OSR_4096 0x48 

//#define  MS561101BA_D2_OSR_256 0x50 
//#define  MS561101BA_D2_OSR_512 0x52 
//#define  MS561101BA_D2_OSR_1024 0x54 
//#define  MS561101BA_D2_OSR_2048 0x56 
#define  MS561101BA_D2_OSR_4096 0x58 

#define  MS561101BA_ADC_RD 0x00 
#define  MS561101BA_PROM_RD 0xA0 
#define  MS561101BA_PROM_CRC 0xAE

#define CONST_SEA_PRESSURE 102610.f //1026.1f //http://www.meteo.physik.uni-muenchen.de/dokuwiki/doku.php?id=wetter:stadt:messung
#define CONST_PF 0.1902630958 //(1/5.25588f) Pressure factor
#define CONST_PF2 44330.0f

#define ms5611_pmin 20
#define ms5611_pmax 1200
#define ms5611_tmin -40
#define ms5611_tmax 85

void MS611_Write(uint8_t reg);
uint16_t MS611_Read_16bit(uint8_t reg);
uint32_t MS611_Read_24bit(uint8_t reg);
void MS561101BA_RESET(void); 
void MS561101BA_PROM_READ(void); 
uint32_t MS561101BA_DO_CONVERSION(uint8_t command); 
void MS561101BA_Init(void);
void ms5611_update(void);
void ms5611_calculate(void);
void asl_fliter(void);

#endif




