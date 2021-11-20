/*
 * stm32l47xx_gpio_driver.c
 *
 *  Created on: Nov 17, 2021
 *      Author: christian cortes
 */

#include "stm32l47xx_gpio_driver.h"

/***********************************************************************************
 * @fn			- GPIO_PeripheralClockControl
 *
 * @brief 		- This function enables or disables the peripheral clock on the GPIOx Port
 *
 * @param[in]  	- The GIPOx Port that will be enables of disabled
 * @param[in] 	- ENABLE or DISABLE value of emum type EnDiType_t
 *
 * @return		- None
 */
void GPIO_PeripheralClockControl(GPIO_TypeDef *pGPIOx, EnDiType_t enable )
{
	if(enable == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else{

		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else{

		}
	}
}

/***********************************************************************************
 *
 */
void GPIO_Init(GPIOx_Handle_t *pGPIOxHandle)
{
	// 1. configure the mode of the GPIO pin

	// 2. configure the speed

	// 3. configure the PUPD

	// 4.

}

/***********************************************************************************
 *
 */
void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{

}

/***********************************************************************************
 *
 */
uint32_t GPIO_ReadFromPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{

}

/***********************************************************************************
 *
 */
uint32_t GPIO_ReadFromPort(GPIO_TypeDef *pGPIOx)
{

}

/***********************************************************************************
 *
 */
void GPIO_WriteToPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint32_t value)
{

}

/***********************************************************************************
 *
 */
void GPIO_WriteToPort(GPIO_TypeDef *pGPIOx,  uint32_t value)
{

}

/***********************************************************************************
 *
 */
void GPIO_TogglePin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{

}

/***********************************************************************************
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/***********************************************************************************
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

#include "stm32l47xx_gpio_driver.h"
