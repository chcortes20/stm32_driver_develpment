/*
 * stm32l47xx_gpio_driver.h
 *
 *  Created on: Nov 17, 2021
 *      Author: christian cortes
 */

#ifndef INC_STM32L47XX_GPIO_DRIVER_H_
#define INC_STM32L47XX_GPIO_DRIVER_H_

#include "stm32l47xx.h"

typedef struct
{
	uint8_t Pin;
	uint8_t Mode;
	uint8_t Pull;
	uint8_t Speed;
	uint8_t Alternate;
	uint8_t OPType;

}GPIOx_PinConfig_t;

/*
 * This is a Handle structure for the GPIO pin
 */
typedef struct
{
	GPIO_TypeDef *pGPIOx;
	GPIOx_PinConfig_t GPIO_PinConfig;
}GPIOx_Handle_t;



/******************************************************************************************************
 * 									APIs supported by the driver
 * 				For more information about the APIs check the function definitions
 * ****************************************************************************************************/

/*
 * peripheral clock set up
 */
void GPIO_PeripheralClockControl(void);

/*
 * Init and De-Init
 */
void GPIO_Init(void);
void GPIO_DeInit(void);

/*
 * Data Read and Write
 */
void GPIO_ReadFromPin(void);
void GPIO_ReadFromPort(void);
void GPIO_WriteToPin(void);
void GPIO_WriteToPort(void);
void GPIO_TogglePin(void);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);

























#endif /* INC_STM32L47XX_GPIO_DRIVER_H_ */
