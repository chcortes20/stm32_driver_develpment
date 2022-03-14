/*
 * 006Spi_tx.c
 *
 *  Created on: Feb 27, 2022
 *      Author: Christian Cortes
 */


/*
 * PB12 -> NSS
 * PB13 -> SCK
 * PB14 -> MISO
 * PB15 -> MOSI
 * alt function 5
 * */
#include "stm32l47xx.h"
#include "string.h"

void SPI2_GPIOInits(){
	GPIOx_Handle_t SPIPIns;

	SPIPIns.pGPIOx = GPIOB;
	SPIPIns.GPIO_PinConfig.Mode = GPIO_MODE_ALTFN;
	SPIPIns.GPIO_PinConfig.Alternate = AF5;
	SPIPIns.GPIO_PinConfig.OPType = GPIO_OP_TYPE_PP;
	SPIPIns.GPIO_PinConfig.Pull = GPIO_NO_PUPD;
	SPIPIns.GPIO_PinConfig.Speed = GPIO_SPEED_HIGH;

	// SCCLK
	SPIPIns.GPIO_PinConfig.Pin = GPIO_PIN_13;
	GPIO_Init(&SPIPIns);

	//MOSI
	SPIPIns.GPIO_PinConfig.Pin = GPIO_PIN_15;
	GPIO_Init(&SPIPIns);

	// I configured to show a full initialization
	// but for Tx only, we can leave these pins alone.
//	// MISO
//	SPIPIns.GPIO_PinConfig.Pin = GPIO_PIN_14;
//	GPIO_Init(&SPIPIns);
//
//	// NSS
//	SPIPIns.GPIO_PinConfig.Pin = GPIO_PIN_12;
//	GPIO_Init(&SPIPIns);

}

void SPI2Init(){
	SPI_Handle_t SPIinit;

	SPIinit.pSPIx = SPI2;
	SPIinit.SPI_config.SPI_BusConfig = SPI_FULL_DUPLEX;
	SPIinit.SPI_config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIinit.SPI_config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPIinit.SPI_config.SPI_DS = SPI_DS_8BITS; // CR2
	SPIinit.SPI_config.SPI_CPOL = CLOCK_POL_LOW_IDLE;
	SPIinit.SPI_config.SPI_CPHA = CAP_FIRST_EDGE;
	SPIinit.SPI_config.SPI_SSM = SPI_SSM_SW; // SW slave management for NSS pin

	SPI_Init(&SPIinit);

}



int main(){

	// create user buffer
	char user_data[] = "Hello world";

	// this function configures the GPIO pins to work with SPI2, used GPIOB
	SPI2_GPIOInits();

	// this initializes, configures, and enables the SPI2 interface
	SPI2Init();

	// must enable SSI when using SW slave management
	// this makes internal NSS signal high, otherwise MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	// enable the SPI2 peripheral
	SPI_enable(SPI2, ENABLE);


	// send data
	SPI_DataSend(SPI2,(uint8_t*) user_data, strlen(user_data));


	// disable the SPI2 peripheral
	SPI_enable(SPI2, DISABLE);

	while(1);

	return 0;
}

