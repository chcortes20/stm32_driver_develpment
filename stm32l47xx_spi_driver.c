/*
 * stm32l47xx_spi_driver.c
 *
 *  Created on: Feb 4, 2022
 *      Author: christian cortes
 */

#include "stm32l47xx_spi_driver.h"
/***********************************************************************************
 * @fn			- SPI_PeripheralClockControl
 *
 * @brief 		- This function enables or disables the peripheral clock on the GPIOx Port
 *
 * @param[in]  	- The GIPOx Port that will be enables of disabled
 * @param[in] 	- ENABLE or DISABLE value of emum type EnDiType_t
 *
 * @return		- None
 */
void SPI_PeripheralClockControl(SPI_Typedef *pSPIx, EnDiType_t enable){

	if(enable == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else{

		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI2_PCLK_DI();
		}
		else{

		}
	}

}

/***********************************************************************************
 * @fn			- SPI_Init
 *
 * @brief 		- This function enables or disables the peripheral clock on the GPIOx Port
 *
 * @param[in]  	- The GIPOx Port that will be enables of disabled
 *
 * @return		- None
 */
void SPI_Init(SPI_Handle_t *pSPIxHandle){

	// first lets configure the SPI_CSR1 register
	uint32_t temp = 0;

	// 1. configure the device mode
	temp |= pSPIxHandle->SPI_config.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. configure the bus mode
	if(pSPIxHandle->SPI_config.SPI_BusConfig == SPI_FULL_DUPLEX){
		// bidi should be cleared
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIxHandle->SPI_config.SPI_BusConfig == SPI_HALF_DUPLEX){
		// bidi should be set
		temp &= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIxHandle->SPI_config.SPI_BusConfig == SPI_SIMPLEX_RX_ONLY){
		// bidi should be cleared
		temp &= ~(1 << SPI_CR1_BIDIMODE);

		// rx only  bit set
		temp |= (1 << SPI_CR1_RXONLY);

	}

	// 3. configure the spi serial cloak speed
	temp |= pSPIxHandle->SPI_config.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. set DS, leave a lone for now need to figure out if i used correct item
	// skip, will use 8-bits by default

	// 5. configure the CPOL
	temp |= pSPIxHandle->SPI_config.SPI_CPOL << SPI_CR1_CPOL;

	// 6. configure the CPHA
	temp |= pSPIxHandle->SPI_config.SPI_CPHA << SPI_CR1_CPHA;


	pSPIxHandle->pSPIx->CR1 = temp;

}

/***********************************************************************************
 * @fn			- SPI_DeInit
 *
 * @brief 		- This function enables or disables the peripheral clock on the GPIOx Port
 *
 * @param[in]  	- The GIPOx Port that will be enables of disabled
 * @param[in] 	- ENABLE or DISABLE value of emum type EnDiType_t
 *
 * @return		- None
 */
void SPI_DeInit(SPI_Typedef *pSPIx){

	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}

}


/***********************************************************************************
 * @fn			- SPI_DataSend
 *
 * @brief 		- This blocking function will hold until all bytes are transfered
 *
 * @param[in]  	- pSPIx that gives access to SPI registers
 * @param[in] 	- *pTxBuffer the data to be transfered
 * @param[in]   - len the total number of bytes to be transfered;
 *
 * @return		- None
 */
void SPI_DataSend(SPI_Typedef *pSPIx, uint8_t *pTxBuffer, uint32_t len){

	while(len){
		// 1. wait for the TXE flag to be set
		while(!(pSPIx->SR & (1 << SPI_SR_TXE)));

		//2. load data into the DR
//		pSPIx->DR = *((uint16_t *) pTxBuffer);
//		len --;
//		len --;
//		(uint16_t*)pTxBuffer++;
		pSPIx->DR = *pTxBuffer;
		len--;
		pTxBuffer++;
	}

}

/***********************************************************************************
 * @fn			- SPI_DataReceive
 *
 * @brief 		- This function enables or disables the peripheral clock on the GPIOx Port
 *
 * @param[in]  	- The GIPOx Port that will be enables of disabled
 * @param[in] 	- ENABLE or DISABLE value of emum type EnDiType_t
 *
 * @return		- None
 */
void SPI_DataReceive(SPI_Typedef* pSPIx, uint8_t* RxBuffer, uint32_t len){

}



/***********************************************************************************
 * @fn			- SPI_IRQConfig
 *
 * @brief 		- This function enables or disables the peripheral clock on the GPIOx Port
 *
 * @param[in]  	- The GIPOx Port that will be enables of disabled
 * @param[in] 	- ENABLE or DISABLE value of emum type EnDiType_t
 *
 * @return		- None
 */
void SPI_IRQConfig(uint8_t IRQNumber, EnDiType_t EnorDi){

}

/***********************************************************************************
 * @fn			- SPI_IRQPriorityConfig
 *
 * @brief 		- This function enables or disables the peripheral clock on the GPIOx Port
 *
 * @param[in]  	- The GIPOx Port that will be enables of disabled
 * @param[in] 	- ENABLE or DISABLE value of emum type EnDiType_t
 *
 * @return		- None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

}


/***********************************************************************************
 * @fn			- SPI_IRQHandling
 *
 * @brief 		- This function enables or disables the peripheral clock on the GPIOx Port
 *
 * @param[in]  	- The GIPOx Port that will be enables of disabled
 * @param[in] 	- ENABLE or DISABLE value of emum type EnDiType_t
 *
 * @return		- None
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle){

}


// end of file
