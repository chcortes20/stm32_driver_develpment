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


/*
 * define the modes of the GPIO pins
 * */

#define	GPIO_MODE_IN		0
#define	GPIO_MODE_OUT		1
#define	GPIO_MODE_ALTFN		2
#define	GPIO_MODE_ANALOG	3

#define	GPIO_MODE_IT_FT		4
#define	GPIO_MODE_IT_RT		5
#define	GPIO_MODE_IT_RFT	6




/*
 * Define the possible output types of the GPIOx pin
 * */

#define	GPIO_OP_TYPE_PP 0
#define	GPIO_OP_TYPE_OD 1


/******************************************************************************************************
 * 									APIs supported by the driver
 * 				For more information about the APIs check the function definitions
 * ****************************************************************************************************/

/*
 * peripheral clock set up
 */
void GPIO_PeripheralClockControl(GPIO_TypeDef *pGPIOx, EnDiType_t enable );

/*
 * Init and De-Init
 */
void GPIO_Init(GPIOx_Handle_t *pGPIOxHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);

/*
 * Data Read and Write
 */
uint32_t GPIO_ReadFromPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);
uint32_t GPIO_ReadFromPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteToPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint32_t value);
void GPIO_WriteToPort(GPIO_TypeDef *pGPIOx,  uint32_t value);
void GPIO_TogglePin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

























#endif /* INC_STM32L47XX_GPIO_DRIVER_H_ */
