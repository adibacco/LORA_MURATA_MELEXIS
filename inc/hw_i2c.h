
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_I2C_H__
#define __HW_I2C_H__

#ifdef __cplusplus
 extern "C" {
#endif



#if (defined USE_STM32F0XX_NUCLEO || defined USE_STM32F1XX_NUCLEO || defined USE_STM32F3XX_NUCLEO)
#define __GPIOA_CLK_ENABLE() 								__HAL_RCC_GPIOA_CLK_ENABLE()
#define __GPIOB_CLK_ENABLE() 								__HAL_RCC_GPIOB_CLK_ENABLE()
#define INIT_CLK_GPO_RFD() 								__HAL_RCC_GPIOA_CLK_ENABLE()
#define I2Cx_CLK_ENABLE()                   			 	                __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()       						__HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       						__HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C1_FORCE_RESET()                                                              __HAL_RCC_I2C1_FORCE_RESET()
#define I2C1_RELEASE_RESET()                                                            __HAL_RCC_I2C1_RELEASE_RESET()
#else
#define INIT_CLK_GPO_RFD() 								__GPIOA_CLK_ENABLE()
#define I2Cx_CLK_ENABLE()                   			 	                __I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()       						__GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       						__GPIOB_CLK_ENABLE()
#define I2C1_FORCE_RESET()               						__I2C1_FORCE_RESET()
#define I2C1_RELEASE_RESET()             						__I2C1_RELEASE_RESET()
#endif

#if (defined USE_STM32F0XX_NUCLEO || defined USE_STM32F1XX_NUCLEO || defined USE_STM32F3XX_NUCLEO)
#define GPIO_SPEED_HIGH                                                                 GPIO_SPEED_FREQ_HIGH
#endif

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

#endif  /* __HW_SPI_H__ */
