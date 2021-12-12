/*
 * 002led_button.c
 *
 *  Created on: Dec 5, 2021
 *      Author: chritian cortes
 */


#include "stm32l47xx.h"


void delay(){
	for (volatile uint32_t i = 0; i < 500000/2; i++ );
}


int main(){

	GPIOx_Handle_t gpioLed;
	GPIOx_Handle_t gpiobtn;



	gpioLed.pGPIOx = GPIOB;
	gpioLed.GPIO_PinConfig.Pin = GPIO_PIN_14;
	gpioLed.GPIO_PinConfig.Mode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.Speed = GPIO_SPEED_HIGH;
	gpioLed.GPIO_PinConfig.OPType = GPIO_OP_TYPE_OD;
	gpioLed.GPIO_PinConfig.Pull = GPIO_PIN_PU;

	GPIO_PeripheralClockControl(GPIOB, ENABLE);
	GPIO_Init(&gpioLed);




	gpiobtn.pGPIOx = GPIOC;
	gpiobtn.GPIO_PinConfig.Pin = GPIO_PIN_13;
	gpiobtn.GPIO_PinConfig.Mode = GPIO_MODE_IN;
	gpiobtn.GPIO_PinConfig.Speed = GPIO_SPEED_HIGH;
	gpiobtn.GPIO_PinConfig.Pull = GPIO_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpiobtn);


	while(1){
		if (!(GPIO_ReadFromPin(GPIOC, GPIO_PIN_13))){
			delay();
			GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		}


	}


	return 0;
}
