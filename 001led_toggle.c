/*
 * 001led_toggle.c
 *
 *  Created on: Dec 4, 2021
 *      Author: Christian Cortes
 */

#include "stm32l47xx.h"

void delay(){
	for (volatile uint32_t i = 0; i < 500000; i++ );
}


int main(){

	GPIOx_Handle_t gpioLed;

	gpioLed.pGPIOx = GPIOB;
	gpioLed.GPIO_PinConfig.Pin = GPIO_PIN_14;
	gpioLed.GPIO_PinConfig.Mode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.Speed = GPIO_SPEED_HIGH;
	gpioLed.GPIO_PinConfig.OPType = GPIO_OP_TYPE_OD;
	gpioLed.GPIO_PinConfig.Pull = GPIO_PIN_PU;

	GPIO_PeripheralClockControl(GPIOB, ENABLE);

	GPIO_Init(&gpioLed);

	while(1){
		GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		delay();
	}


	return 0;
}
