/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "MLX90640_I2C_Driver.h"
#include "hw_conf.h"
#include "hw.h"
#include "i2c_api.h"

extern I2C_HandleTypeDef hi2c1;

void MLX90640_I2CInit()
{   
		/* check if we re-init after a detected issue */
	  if( hi2c1.Instance == I2C1)
		  HAL_I2C_DeInit(&hi2c1);

	  /* Configure I2C structure */
	  hi2c1.Instance 	     = I2C1;
	  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	#if defined (STM32F302x8)
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	#elif defined (STM32F401xE)
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;
	#endif
	  hi2c1.Init.OwnAddress1     = 0;
	  hi2c1.Init.OwnAddress2     = 0;
	#if (defined USE_STM32F4XX_NUCLEO) || (defined USE_STM32L1XX_NUCLEO) || \
		  (defined USE_STM32F1XX_NUCLEO)
	  hi2c1.Init.ClockSpeed      = MLX90640_I2C_SPEED_100;
	  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
	#elif (defined USE_STM32F0XX_NUCLEO) || (defined USE_STM32L0XX_NUCLEO) || (defined USE_B_L072Z_LRWAN1) || \
	      (defined USE_STM32F3XX_NUCLEO) || (defined USE_STM32L4XX_NUCLEO)
	  hi2c1.Init.Timing          = MLX90640_I2C_SPEED_100;
	#endif



	if(HAL_I2C_Init(&hi2c1) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }

}



int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    uint8_t sa;                           
    int ack = 0;
    int cnt = 0;
    int i = 0;
    char cmd[2] = {0,0};
    char i2cData[1664] = {0};
    uint16_t *p;
    
    p = data;
    sa = (slaveAddr << 1);
    cmd[0] = startAddress >> 8;
    cmd[1] = startAddress & 0x00FF;
    
    //HAL_Delay(1);
    ack = i2c_write(&hi2c1, sa, cmd, 2, 0);

    if (ack < 0x00)
    {
        return -1;
    }

    sa = sa | 0x01;
    ack = i2c_read(&hi2c1, sa, i2cData, 2*nMemAddressRead, 0);
    if (ack < 0x00)
    {
        return -1;
    }
    i2c_stop(&hi2c1);
    
    for(cnt=0; cnt < nMemAddressRead; cnt++)
    {
        i = cnt << 1;
        *p++ = (uint16_t)i2cData[i]*256 + (uint16_t)i2cData[i+1];
    }
    
    return 0;   
} 


int MLX90640_I2CRead_Bulk(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
	int b = 0;
	uint16_t* p = data;

	while (nMemAddressRead > 0) {
		int res = MLX90640_I2CRead(slaveAddr, startAddress + b*64, (nMemAddressRead > 64)? 64 : nMemAddressRead, p + b*64);
		if (res < 0)
			return res;
		b++;
		nMemAddressRead -= 64;
	}

}

void MLX90640_I2CFreqSet(int freq)
{
    //i2c_frequency(1000*freq);
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t sa;
    int ack = 0;
    char cmd[4] = {0,0,0,0};
    static uint16_t dataCheck;
    

    sa = (slaveAddr << 1);
    cmd[0] = writeAddress >> 8;
    cmd[1] = writeAddress & 0x00FF;
    cmd[2] = data >> 8;
    cmd[3] = data & 0x00FF;

    //HAL_Delay(1);
    ack = i2c_write(&hi2c1, sa, cmd, 4, 0);

    if (ack < 0x00)
    {
        return -1;
    }
    i2c_stop(&hi2c1);

    MLX90640_I2CRead(slaveAddr,writeAddress,1, &dataCheck);
    
    if ( dataCheck != data)
    {
        return -2;
    }    
    
    return 0;
}

