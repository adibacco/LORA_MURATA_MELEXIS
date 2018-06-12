/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: contains all hardware driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /******************************************************************************
  * @file    hw.h
  * @author  MCD Application Team
  * @version V1.1.5
  * @date    30-March-2018
  * @brief   contains all hardware driver
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_H__
#define __HW_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "hw_conf.h"
#include "hw_gpio.h"
#include "hw_spi.h"
#include "hw_rtc.h"
#include "hw_msp.h"
#include "debug.h"


 /* I2C functionality is not mapped on the same Alternate function regarding the MCU used */
 #if (defined USE_STM32F4XX_NUCLEO) || (defined USE_STM32F3XX_NUCLEO) || (defined USE_B_L072Z_LRWAN1) || \
     (defined USE_STM32L0XX_NUCLEO) || (defined USE_STM32L1XX_NUCLEO) || (defined USE_STM32L4XX_NUCLEO)
   #define I2Cx_SCL_AF 											      	GPIO_AF4_I2C1
 #elif (defined USE_STM32F0XX_NUCLEO)
 	#define I2Cx_SCL_AF 											      	GPIO_AF1_I2C1
 #elif (defined USE_STM32F1XX_NUCLEO)
   /* Not supported */
 #endif

 /* I2C SPEED
  * F4 uses directly the speed (100,400) and F0, L0, L3 use the TIMMINGR register defined below */
 #if (defined USE_STM32F4XX_NUCLEO) || (defined USE_STM32F1XX_NUCLEO) || \
     (defined USE_STM32L1XX_NUCLEO)
 	#define MLX90640_I2C_SPEED_10													10000
 	#define MLX90640_I2C_SPEED_100													100000
 	#define MLX90640_I2C_SPEED_400													400000
 	#define MLX90640_I2C_SPEED_1000												1000000
 /* Timing samples with PLLCLK 48MHz set in SystemClock_Config(), I2C CLK on SYSCLK value computed with CubeMx */
 #elif (defined USE_STM32F0XX_NUCLEO)
 	#define MLX90640_I2C_SPEED_10													0x9010DEFF
 	#define MLX90640_I2C_SPEED_100													0x20303E5D
 	#define MLX90640_I2C_SPEED_400													0x2010091A
 	#define MLX90640_I2C_SPEED_1000												0x00200818
 /* Timing samples with PLLCLK 32MHz set in SystemClock_Config(), I2C CLK on SYSCLK value computed with CubeMx */
 #elif (defined USE_STM32L0XX_NUCLEO) || (defined USE_B_L072Z_LRWAN1)
 	#define MLX90640_I2C_SPEED_10													0x6010C7FF
 	#define MLX90640_I2C_SPEED_100													0x00707CBB
 	#define MLX90640_I2C_SPEED_400													0x00300F38
 	#define MLX90640_I2C_SPEED_1000												0x00100413
 /* Timing samples with PLLCLK 64MHz set in SystemClock_Config(), I2C CLK on SYSCLK value computed with CubeMx */
 #elif (defined USE_STM32F3XX_NUCLEO)
 	#define MLX90640_I2C_SPEED_10													0xE010A9FF
 	#define MLX90640_I2C_SPEED_100													0x10707DBC
 	#define MLX90640_I2C_SPEED_400													0x00602173
 	#define MLX90640_I2C_SPEED_1000												0x00300B29
 #elif (defined USE_STM32L4XX_NUCLEO)
 	#define MLX90640_I2C_SPEED_10													0xF000F3FE /* Clock 80MHz, Fast Mode, Analog Filter ON, Rise time 25ns, Fall time 10ns */
 	#define MLX90640_I2C_SPEED_100													0x203012F1 /* Clock 80Mhz, Fast Mode, Analog Filter ON, Rise time 50ns, Fall time 10ns */
 	#define MLX90640_I2C_SPEED_400													0x00B0298B /* Clock 80Mhz, Fast Mode, Analog Filter ON, Rise time 50ns, Fall time 25ns */
 	#define MLX90640_I2C_SPEED_1000												0x00700E2E /* Clock 80Mhz, Fast Mode Plus, Analog Filter ON, Rise time 50ns, Fall time 25ns */
 #else
 	#error "You need to update your code to this new microcontroller"
 #endif

#ifdef __cplusplus
}
#endif

#endif /* __HW_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

