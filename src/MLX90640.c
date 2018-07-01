#include <stdlib.h>
#include <stdint.h>
#include <malloc.h>
#include "MLX90640.h"
#include <math.h>
#include "vcom.h"


__attribute__((__section__(".mlx90640_eeprom_data"))) 	uint16_t mlx90640_ee[MLX90640_EEPROM_SIZE];
__attribute__((__section__(".mlx90640_params_data"))) 	paramsMLX90640	mlx90640_params;


#ifdef MLX90640_SAMPLE_DATA
extern uint16_t frameData_sample[];
#endif

#ifdef MLX90640_SAMPLE_EEPROM
extern uint16_t eeData_sample[];
#endif



int MLX90640_GetEEPROM() {

	uint32_t crcEeSensor = 0;
	uint32_t crcEeFlash = 0;
	int ret = 0;

	uint16_t* eeMLX90640 = (uint16_t*) memalign(16, sizeof(uint16_t)*MLX90640_EEPROM_SIZE);

	MLX90640_DumpEE(MLX90640_I2C_ADDR, eeMLX90640);
	int error = (eeMLX90640[15] != 0xbe33);

	if (error == 0) {
		for (int i = 0; i < MLX90640_EEPROM_SIZE; i++)
			crcEeSensor += eeMLX90640[i];

		for (int i = 0; i < MLX90640_EEPROM_SIZE; i++)
			crcEeFlash += mlx90640_ee[i];

		if ((crcEeSensor != crcEeFlash)) {
			uint32_t address = (uint32_t) mlx90640_ee;
			int size = sizeof(mlx90640_ee);
			FLASH_If_Erase(address, size);
			FLASH_If_Write(address, eeMLX90640, size);
			ret = 1;
			vcom_Send("MLX90640 eeprom copied to FLASH\r\n");
		}
	}

	free(eeMLX90640);

	return ret;

}

void MLX90640_GetParameters() {
	paramsMLX90640* params = (paramsMLX90640*) memalign(16, sizeof(paramsMLX90640));
	uint16_t* eeData;

#ifndef MLX90640_SAMPLE_EEPROM
	eeData = mlx90640_ee;
#else
	eeData = eeData_sample;
#endif

	MLX90640_ExtractParameters(eeData, params);

	uint32_t address = (uint32_t) &mlx90640_params;
	int size = sizeof(paramsMLX90640);
	FLASH_If_Erase(address, size);
	FLASH_If_Write(address, params, size);

	free(params);

}


 void MLX90640_GetPixelsTemp(uint16_t* mlxFrame, uint8_t mlxTemp[][32], uint8_t* info) {
	float emissivity = 0.95f;
	float tr;
	int counter = 0;


		for (int p = 0; p < 2; p++) {
			int res = MLX90640_GetFrameData(MLX90640_I2C_ADDR, mlxFrame);

			tr = MLX90640_GetTa(mlxFrame, &mlx90640_params)-TA_SHIFT;
			int subPage = MLX90640_GetSubPageNumber(mlxFrame);
			//reflected temperature based on the sensor, ambient temperature
			MLX90640_CalculateTo(mlxFrame, -1, &mlx90640_params, emissivity, tr, mlxTemp, NULL);
		}

		if (info != NULL)
		{
			uint8_t max = 0;
			uint8_t min = 255;
			int imax= -1, jmax = -1;
			int imin= -1, jmin = -1;

			for (int i = MIN_ROW; i < MAX_ROW; i++) {
				for (int j = MIN_COL; j < MAX_COL; j++) {
					if (mlxTemp[i][j] > max) {
						max = mlxTemp[i][j];
						imax = i;
						jmax = j;
					}
					if (mlxTemp[i][j] < min) {
						min = mlxTemp[i][j];
						imin = i;
						jmin = j;
					}
				}
			}


			info[0] = max;
			info[1] = imax;
			info[2] = jmax;
			info[3] = min;
			info[4] = imin;
			info[5] = jmin;
		}
}

