/*
 * 003led_button.ext.c
 *
 *  Created on: Sep 25, 2023
 *      Author: pc
 */

#include "stm32f446xx.h"

#define HIGH 1
#define LOW  0
#define BTN_PRESSED LOW

void delay(void) {
	for (uint32_t i = 0; i < 500000 ; i ++);
}

int main(void){

	GPIO_Handle_t GpioLed, GPIOBtn;

	// led configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.pGPIO_PinConfig.PinNumber = GPIO_PIN_N14;
	GpioLed.pGPIO_PinConfig.PinMode = GPIO_MODE_OUT;
	GpioLed.pGPIO_PinConfig.PinSpeed = GPIO_SPEED_FAST;
	GpioLed.pGPIO_PinConfig.PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.pGPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

	// this is btn gpio configuraton
	GPIOBtn.pGPIOx = GPIOB;
	GPIOBtn.pGPIO_PinConfig.PinNumber = GPIO_PIN_N12;
	GPIOBtn.pGPIO_PinConfig.PinMode = GPIO_MODE_IN;
	GPIOBtn.pGPIO_PinConfig.PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.pGPIO_PinConfig.PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GPIOBtn);

	while(1) {
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_N12) == BTN_PRESSED){
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_N14);
		}
	}

}
