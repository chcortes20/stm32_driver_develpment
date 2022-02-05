/*
 * stm32l47xx_spi_driver.h
 *
 *  Created on: Feb 4, 2022
 *      Author: christian cortes
 */

#ifndef INC_STM32L47XX_SPI_DRIVER_H_
#define INC_STM32L47XX_SPI_DRIVER_H_

#include "stm32l47xx.h"
/**
 * configuration structure of the SPIx peripheral
 * */
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPIx_Config_t;

/*
 * Handle structure for SPIx peripheral
 * */
typedef struct
{
	SPI_Typedef *pSPIx;
	SPIx_Config_t SPI_config;
};

/******************************************************************************************************
 * 									APIs supported by the driver
 * 				For more information about the APIs check the function definitions
 * ****************************************************************************************************/

#endif /* INC_STM32L47XX_SPI_DRIVER_H_ */
