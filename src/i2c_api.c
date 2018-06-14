
/*******************************************************************************
 * @file    hw_spi.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   manages the SPI interface
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
#include "hw.h"
#include "i2c_api.h"


#define FLAG_TIMEOUT ((int)0x1000)
#define LONG_TIMEOUT ((int)0x8000)


int i2c_start(I2C_HandleTypeDef *i2c_dev) {
	I2C_TypeDef *i2c = (I2C_TypeDef *)(i2c_dev->Instance);
	int timeout;


	// Clear Acknowledge failure flag
	__HAL_I2C_CLEAR_FLAG(i2c_dev, I2C_FLAG_AF);

	// Generate the START condition
	i2c->CR2 |= I2C_CR2_START;

	// Wait the START condition has been correctly sent
	timeout = FLAG_TIMEOUT;
	while (__HAL_I2C_GET_FLAG(i2c_dev, I2C_FLAG_BUSY) == RESET) {
		if ((timeout--) == 0) {
			return 1;
		}
	}

	return 0;
}

int i2c_stop(I2C_HandleTypeDef *i2c_dev) {
	I2C_TypeDef *i2c = (I2C_TypeDef *)(i2c_dev->Instance);

	int timeout = FLAG_TIMEOUT;

	// Generate the STOP condition
	i2c->CR2 |= I2C_CR2_STOP;

	return 0;
}

int i2c_read(I2C_HandleTypeDef *i2c_dev, int address, char *data, int length, int stop) {
	I2C_TypeDef *i2c = (I2C_TypeDef *)(i2c_dev->Instance);
	int timeout;
	int count;
	int value;

	/* update CR2 register */
	i2c->CR2 = (i2c->CR2 & (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP)))
                		   | (uint32_t)(((uint32_t)address & I2C_CR2_SADD) | (((uint32_t)length << 16) & I2C_CR2_NBYTES) | (uint32_t)I2C_SOFTEND_MODE | (uint32_t)I2C_GENERATE_START_READ);

	// Read all bytes
	for (count = 0; count < length; count++) {
		value = i2c_byte_read(i2c_dev, 0);
		data[count] = (char)value;
	}

	// Wait transfer complete
	timeout = FLAG_TIMEOUT;
	while (__HAL_I2C_GET_FLAG(i2c_dev, I2C_FLAG_TC) == RESET) {
		timeout--;
		if (timeout == 0) {
			return -1;
		}
	}
	__HAL_I2C_CLEAR_FLAG(i2c_dev, I2C_FLAG_TC);

	// If not repeated start, send stop.
	if (stop) {
		i2c_stop(i2c_dev);
		/* Wait until STOPF flag is set */
		timeout = FLAG_TIMEOUT;
		while (__HAL_I2C_GET_FLAG(i2c_dev, I2C_FLAG_STOPF) == RESET) {
			timeout--;
			if (timeout == 0) {
				return -1;
			}
		}
		/* Clear STOP Flag */
		__HAL_I2C_CLEAR_FLAG(i2c_dev, I2C_FLAG_STOPF);
	}

	return length;
}

int i2c_write(I2C_HandleTypeDef *i2c_dev, int address, const char *data, int length, int stop) {
	I2C_TypeDef *i2c = (I2C_TypeDef *)(i2c_dev->Instance);
	int timeout;
	int count;

	/* update CR2 register */
	i2c->CR2 = (i2c->CR2 & (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP)))
                		   | (uint32_t)(((uint32_t)address & I2C_CR2_SADD) | (((uint32_t)length << 16) & I2C_CR2_NBYTES) | (uint32_t)I2C_SOFTEND_MODE | (uint32_t)I2C_GENERATE_START_WRITE);



	for (count = 0; count < length; count++) {
		i2c_byte_write(i2c_dev, data[count]);
	}

	// Wait transfer complete
	timeout = FLAG_TIMEOUT;
	while (__HAL_I2C_GET_FLAG(i2c_dev, I2C_FLAG_TC) == RESET) {
		timeout--;
		if (timeout == 0) {
			return -1;
		}
	}
	__HAL_I2C_CLEAR_FLAG(i2c_dev, I2C_FLAG_TC);

	// If not repeated start, send stop.
	if (stop) {
		i2c_stop(i2c_dev);
		/* Wait until STOPF flag is set */
		timeout = FLAG_TIMEOUT;
		while (__HAL_I2C_GET_FLAG(i2c_dev, I2C_FLAG_STOPF) == RESET) {
			timeout--;
			if (timeout == 0) {
				return -1;
			}
		}
		/* Clear STOP Flag */
		__HAL_I2C_CLEAR_FLAG(i2c_dev, I2C_FLAG_STOPF);
	}

	return count;
}

int i2c_byte_read(I2C_HandleTypeDef *i2c_dev, int last) {
	I2C_TypeDef *i2c = (I2C_TypeDef *)(i2c_dev->Instance);
	int timeout;

	// Wait until the byte is received
	timeout = FLAG_TIMEOUT;
	while (__HAL_I2C_GET_FLAG(i2c_dev, I2C_FLAG_RXNE) == RESET) {
		if ((timeout--) == 0) {
			return -1;
		}
	}

	return (int)i2c->RXDR;
}

int i2c_byte_write(I2C_HandleTypeDef *i2c_dev, int data) {
	I2C_TypeDef *i2c = (I2C_TypeDef *)(i2c_dev->Instance);
	int timeout;

	// Wait until the previous byte is transmitted
	timeout = FLAG_TIMEOUT;
	//uint32_t isr = i2c_dev->Instance->ISR;
	while (__HAL_I2C_GET_FLAG(i2c_dev, I2C_FLAG_TXIS) == RESET) {
		if ((timeout--) == 0) {
			return 0;
		}
	}

	i2c->TXDR = (uint8_t)data;

	return 1;
}

void i2c_reset(I2C_HandleTypeDef *i2c_dev) {
	I2C_TypeDef *i2c = (I2C_TypeDef *)(i2c_dev->Instance);

	int timeout;

	// wait before reset
	timeout = LONG_TIMEOUT;
	while((__HAL_I2C_GET_FLAG(i2c_dev, I2C_FLAG_BUSY)) && (timeout-- != 0));

	if (i2c == I2C1) {
		__I2C1_FORCE_RESET();
		__I2C1_RELEASE_RESET();
	}
	if (i2c == I2C2) {
		__I2C2_FORCE_RESET();
		__I2C2_RELEASE_RESET();
	}
}
