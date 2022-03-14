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
	uint8_t SPI_DS;
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
}SPI_Handle_t;


/**
 * @SPI modes
 */
#define SPI_DEVICE_MODE_SLAVE  0
#define SPI_DEVICE_MODE_MASTER 1


/**
 * @SPI Bus config
 */
#define SPI_FULL_DUPLEX        1
#define SPI_HALF_DUPLEX        2
#define SPI_SIMPLEX_RX_ONLY    3


/**
 * @SPI Baud
 */
#define SPI_SCLK_SPEED_DIV2    0
#define SPI_SCLK_SPEED_DIV4    1
#define SPI_SCLK_SPEED_DIV8    2
#define SPI_SCLK_SPEED_DIV16   3
#define SPI_SCLK_SPEED_DIV32   4
#define SPI_SCLK_SPEED_DIV64   5
#define SPI_SCLK_SPEED_DIV128  6
#define SPI_SCLK_SPEED_DIV256  7


/**
 * @ SPI  CPOL
 */
#define CLOCK_POL_LOW_IDLE     0
#define CLOCK_POL_HIGH_IDLE    1


/**
 * @ SPI CPHA
 */
#define CAP_FIRST_EDGE         0
#define CAP_SECOND_EDGE        1


/**
 * @SPI DS
 */
#define SPI_DS_8BITS         7
#define SPI_DS_16BITS        15

/**
 * @SPI SSM
 */
#define SPI_SSM_HW             0
#define SPI_SSM_SW             1

/*********************************************************************************
 * Bit position definition for the SPI peripheral
 *********************************************************************************/
#define SPI_CR1_CPHA           0
#define SPI_CR1_CPOL           1
#define SPI_CR1_MSTR           2
#define SPI_CR1_BR             3
#define SPI_CR1_SPE            6
#define SPI_CR1_LSBFIRST       7
#define SPI_CR1_SSI            8
#define SPI_CR1_SSM            9
#define SPI_CR1_RXONLY         10
#define SPI_CR1_CRCL           11
#define SPI_CR1_CRSNEXT        12
#define SPI_CR1_CRCEN          13
#define SPI_CR1_BIDIOE         14
#define SPI_CR1_BIDIMODE       15


#define SPI_CR2_RXDMAEN        0
#define SPI_CR2_TXDMAEN        1
#define SPI_CR2_SSOE           2
#define SPI_CR2_NSSP           3
#define SPI_CR2_FRF            4
#define SPI_CR2_ERRIE          5
#define SPI_CR2_RXNEIE         6
#define SPI_CR2_TXEIE          7
#define SPI_CR2_DS             8
#define SPI_CR2_FRXTH          12
#define SPI_CR2_LDMA_RX        13
#define SPI_CR2_LDMA_TX        14


#define SPI_SR_RXNE            0
#define SPI_SR_TXE             1
#define SPI_SR_CRCERR          4
#define SPI_SR_MODF            5
#define SPI_SR_OVR             6
#define SPI_SR_BSY             7
#define SPI_SR_FRE             8
#define SPI_SR_FRLVL           9
#define SPI_SR_FTLVL           11

/******************************************************************************************************
 *                                APIs supported by the driver
 * 	            For more information about the APIs check the function definitions
 * ****************************************************************************************************/

/*
 * peripheral Clock setup
 * */
void SPI_PeripheralClockControl(SPI_Typedef *, EnDiType_t );
/**
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *);
void SPI_DeInit(SPI_Typedef *);
/**
 * Data send and receive
 */
void SPI_DataSend(SPI_Typedef*, uint8_t*, uint32_t); // second arg points to Tx buffer
void SPI_DataReceive(SPI_Typedef*, uint8_t*, uint32_t);

/**
 * IRQ configuration and ISR handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, EnDiType_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * other functions that will be helpful for the SPI interface
 */
void SPI_enable(SPI_Typedef *pSPIx, EnDiType_t enable);
void SPI_SSIConfig(SPI_Typedef *pSPIx, EnDiType_t enable);

#endif /* INC_STM32L47XX_SPI_DRIVER_H_ */
