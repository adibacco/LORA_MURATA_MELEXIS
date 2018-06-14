#include <stdlib.h>
#include <stdint.h>
#include <malloc.h>
#include "MLX90640.h"

//static uint16_t mlx90640Frame[MLX90640_FRAME_SIZE];
//static float mlx90640To[768];

__attribute__((__section__(".mlx90640_eeprom_data"))) 	uint16_t mlx90640_ee[MLX90640_EEPROM_SIZE];
__attribute__((__section__(".mlx90640_params_data"))) 	paramsMLX90640	mlx90640_params;

void MLX90640_GetEEPROM() {

	uint16_t* eeMLX90640 = (uint16_t*) memalign(16, sizeof(uint16_t)*MLX90640_EEPROM_SIZE);

	MLX90640_DumpEE(MLX90640_I2C_ADDR, eeMLX90640);
	int error = (eeMLX90640[15] != 0xbe33);

	if (error == 0) {
		uint32_t address = (uint32_t) mlx90640_ee;
		int size = sizeof(mlx90640_ee);
		FLASH_If_Erase(address, size);
		FLASH_If_Write(address, eeMLX90640, size);
	}

	free(eeMLX90640);

}

void MLX90640_GetParameters() {
	paramsMLX90640* params = (paramsMLX90640*) memalign(16, sizeof(paramsMLX90640));

	MLX90640_ExtractParameters(mlx90640_ee, params);

	uint32_t address = (uint32_t) &mlx90640_params;
	int size = sizeof(paramsMLX90640);
	FLASH_If_Erase(address, size);
	FLASH_If_Write(address, params, size);

	free(params);

}

 void MLX90640_GetPixelsTemp() {
	float emissivity = 0.95;
	float tr;
	float mlx90640To[768];

	uint16_t* mlx90640Frame = (uint16_t*) memalign(16, sizeof(uint16_t)*MLX90640_FRAME_SIZE);

	  // Select STEP MODE
	  //MLX90640_I2CWrite(MLX90640_I2C_ADDR, 0x800D, 0x1903);

	int res = MLX90640_GetFrameData_StepMode(MLX90640_I2C_ADDR, mlx90640Frame);

	tr = MLX90640_GetTa(mlx90640Frame, &mlx90640_params)-TA_SHIFT;
	//reflected temperature based on the sensor, ambient temperature
	MLX90640_CalculateTo(mlx90640Frame, &mlx90640_params, emissivity, tr, mlx90640To);

	free(mlx90640Frame);
}

