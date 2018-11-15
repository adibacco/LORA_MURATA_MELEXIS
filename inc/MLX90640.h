#ifndef _MLX90640_H_
#define _MLX90640_H_

#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"

//#define MLX90640_SAMPLE_DATA		1
//#define MLX90640_SAMPLE_EEPROM		1

#define MLX90640_EEPROM_SIZE	832
#define MLX90640_FRAME_SIZE		834
#define MLX90640_MAX_PIXELS		768
#define MLX90640_COLS			32
#define MLX90640_ROWS			24
#define MIN_ROW					0
#define MAX_ROW					(MLX90640_ROWS-1)
#define MIN_COL					0
#define MAX_COL					(MLX90640_COLS-1)

#define NEAR_ROW(i, di)			((((i) + (di)) >= MIN_ROW)? ((((i) + (di)) <= MAX_ROW)? (i) + (di): MAX_ROW): MIN_ROW)
#define NEAR_COL(j, dj)			((((j) + (dj)) >= MIN_COL)? ((((j) + (dj)) <= MAX_COL)? (j) + (dj): MAX_COL): MIN_COL)


#define TA_SHIFT				8

int MLX90640_GetEEPROM();

void MLX90640_GetParameters();

void MLX90640_GetPixelsTemp(uint16_t* mlxFrame, uint8_t mlxTemp[][32], uint8_t* info);

float GetEquivalentTempForRegion(float alpha, int imax, int jmax, float Tp4[24][32], float* Tback);

#endif
