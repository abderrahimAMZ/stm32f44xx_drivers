/*
 * 003led_button.ext.c
 *
 *  Created on: Sep 25, 2023
 *      Author: pc
 */

#include "stm32f446xx.h"
#include<string.h>

#define HIGH 1
#define LOW  0
#define BTN_PRESSED LOW

void delay(void) {
	for (uint32_t i = 0; i < 500000 ; i ++);
}

int main(void){

	GPIO_Handle_t GpioLed, GPIOBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));

	// led configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.pGPIO_PinConfig.PinNumber = GPIO_PIN_N12;
	GpioLed.pGPIO_PinConfig.PinMode = GPIO_MODE_OUT;
	GpioLed.pGPIO_PinConfig.PinSpeed = GPIO_SPEED_FAST;
	GpioLed.pGPIO_PinConfig.PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.pGPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	// this is btn gpio configuraton
	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.pGPIO_PinConfig.PinNumber = GPIO_PIN_N5;
	GPIOBtn.pGPIO_PinConfig.PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.pGPIO_PinConfig.PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.pGPIO_PinConfig.PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GPIOBtn);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);



	while(1);

}



void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_N5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_N12);
}
