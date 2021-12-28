/* stm32l47xx_spi_driver.h
 *
 *  Created on: Dec 28, 2021
 *      Author: christian cortes
 */

#ifndef INC_STM32L47XX_SPI_DRIVER_H_
#define INC_STM32L47XX_SPI_DRIVER_H_

#include "stm32l47xx.h"

/*
*   Configuration structure for SPIx peripheral 
*/
typedef struct 
{
    uint8_t Mode;
    uint8_t BusConfig;
    uint8_t Speed;
    uint8_t DFF;
    uint8_t CPHA;
    uint8_t SSM;
}SPIx_Config_t;

/*
*   Handle structure for SPIx peripheral 
*/
typedef struct 
{
    SPIx_Typedef *pSPIx;    /*<! This holds the base address of SPIx (x: 1, 2, 3) peripheral*/
    SPIx_Config_t SPI_Config
}SPIx_Handle_t;

/******************************************************************************************************
 * 									APIs supported by the driver
 * 				For more information about the APIs check the function definitions
 * ****************************************************************************************************/

/*
* peripheral clock setup
*/
void SPI_PeripheralClockControl(SPIx_Typedef *pSPIx, EnDiType_t enable);

/*
*   Init and De-Init
*/
void SPI_Init(SPIx_Handle_t *pSPIxHandle);
void SPI_DeInit(SPIx_Typedef *pSPIx);

/*
*   Data Send and Receive
*/
void SPI_SendData(SPIx_Typedef *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPIx_Typedef *pSPIx, uint8_t *pRxBuffer, uint32_t length);

/*
*   IRQ configuration and handling 
*/
void SPI_IRQConfig(uint8_t IRQNumber, EnDiType_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPIx_Handle_t *pSPIxHandle);



#endif /* INC_STM32L47XX_SPI_DRIVER_H_ */