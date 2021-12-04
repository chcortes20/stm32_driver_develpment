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
 *  @fn			- GPIO_PeripheralClockControl
 *
 * @brief 		- This function enables or disables the peripheral clock on the GPIOx Port
 *
 * @param[in]  	- The GIPOx Port that will be enables of disabled
 * @param[in] 	- ENABLE or DISABLE value of emum type EnDiType_t
 *
 * @return		- None
 */
void GPIO_Init(GPIOx_Handle_t *pGPIOxHandle)
{
	uint32_t temp = 0;

	// 1. configure the mode of the GPIO pin
	if(pGPIOxHandle->GPIO_PinConfig.Mode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOxHandle->GPIO_PinConfig.Mode << (2U * pGPIOxHandle->GPIO_PinConfig.Pin));
		pGPIOxHandle->pGPIOx->MODER &= ~(0x3 << 2U * pGPIOxHandle->GPIO_PinConfig.Pin); // clearing
		pGPIOxHandle->pGPIOx->MODER |= temp; // setting

	}else{
		// will code this later for interrupt mode
	}

	temp = 0;
	// 2. configure the speed
	temp = (pGPIOxHandle->GPIO_PinConfig.Speed << (2 * pGPIOxHandle->GPIO_PinConfig.Pin));
	pGPIOxHandle->pGPIOx->OSPEED &= ~(0x3 << pGPIOxHandle->GPIO_PinConfig.Pin); // clearing
	pGPIOxHandle->pGPIOx->OSPEED |= temp;

	temp = 0;
	// 3. configure the PUPD
	temp = (pGPIOxHandle->GPIO_PinConfig.Pull << (2 * pGPIOxHandle->GPIO_PinConfig.Pin));
	pGPIOxHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOxHandle->GPIO_PinConfig.Pin); // clearing
	pGPIOxHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	// 4.configure the Otype
	temp = (pGPIOxHandle->GPIO_PinConfig.OPType << pGPIOxHandle->GPIO_PinConfig.Pin);
	pGPIOxHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOxHandle->GPIO_PinConfig.Pin); // clearing
	pGPIOxHandle->pGPIOx->OTYPER |= temp;

	temp = 0;


	// 5. configure the alt functionality
	if(pGPIOxHandle->GPIO_PinConfig.Mode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;

		temp1 = (pGPIOxHandle->GPIO_PinConfig.Pin / 8);
		temp2 = (pGPIOxHandle->GPIO_PinConfig.Pin % 8);
		pGPIOxHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2)); // clearing
		pGPIOxHandle->pGPIOx->AFR[temp1] |= (pGPIOxHandle->GPIO_PinConfig.Alternate << ( 4 * temp2));
	}

}

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
void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else{

	}

}

/***********************************************************************************
 * @fn			- GPIO_ReadFromPin
 *
 * @brief 		- This function reads the IDR register and returns a 0 or 1
 * 				  for the pin that the user wanted to read from
 *
 * @param[in]  	- The GIPOx Port that will be enables of disabled
 * @param[in] 	- the pin number that you want to read from
 *
 * @return		- 0 or 1
 */
uint8_t GPIO_ReadFromPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}

/***********************************************************************************
 * @fn			- GPIO_ReadFromPort
 *
 * @brief 		- This function reads the entire GPIO IDR port
 *
 * @param[in]  	- The GIPOx Port that will be read
 * @param[in] 	- None
 *
 * @return		- entire register value
 */
uint16_t GPIO_ReadFromPort(GPIO_TypeDef *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

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
void GPIO_WriteToPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, Pin_State_t value)
{
	if(value){
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

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
void GPIO_WriteToPort(GPIO_TypeDef *pGPIOx,  uint32_t value)
{
	pGPIOx->ODR = value;
}

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
void GPIO_TogglePin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
	pGPIOx -> ODR ^= (1 << PinNumber);
}

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
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

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
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

#include "stm32l47xx_gpio_driver.h"
