/*
 * 001LedToggle.c
 *
 *  Created on: Sep 25, 2023
 *      Author: pc
 */

#include "stm32f446xx.h"


void delay(void) {
	for (uint32_t i = 0; i < 500000 ; i ++);
}

int main(void){

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.pGPIO_PinConfig.PinNumber = GPIO_PIN_N12;
	GpioLed.pGPIO_PinConfig.PinMode = GPIO_MODE_OUT;
	GpioLed.pGPIO_PinConfig.PinSpeed = GPIO_SPEED_FAST;
	GpioLed.pGPIO_PinConfig.PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.pGPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	while(1) {

		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_N12);
		delay();
	}

}
