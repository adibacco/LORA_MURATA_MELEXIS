#ifndef _MLX90640_H_
#define _MLX90640_H_

#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"

#define MLX90640_SAMPLE_DATA	1
#define MLX90640_EEPROM_SIZE	832
#define MLX90640_FRAME_SIZE		834
#define MLX90640_MAX_PIXELS		768

#define TA_SHIFT				8

void MLX90640_GetEEPROM();

void MLX90640_GetParameters();

void MLX90640_GetPixelsTemp();

#endif
