/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Oct 5, 2023
 *      Author: pc
 */
#include "stm32f446xx.h"
#include<string.h>

// command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54


#define LED_ON		1
#define LED_OFF		0

// arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

#define LED_PIN 	9

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == 0xf5){

		//ack
		return 1;
	}
	else {
		//nack
		return 0;
	}
}

/*
 * we will use port b for the SPI experiment
 * PB12 -> SPI2_NSS
 * PB13 -> SPI2_SCK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 */

void SPI2_GPIOBInits(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.pGPIO_PinConfig.PinMode = GPIO_MODE_ALTEN;
	SPIPins.pGPIO_PinConfig.PinAltFunMode = 5;
	SPIPins.pGPIO_PinConfig.PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.pGPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.pGPIO_PinConfig.PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.pGPIO_PinConfig.PinNumber = GPIO_PIN_N13;

	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.pGPIO_PinConfig.PinNumber = GPIO_PIN_N15;

	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.pGPIO_PinConfig.PinNumber = GPIO_PIN_N14;

	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.pGPIO_PinConfig.PinNumber = GPIO_PIN_N12;

	GPIO_Init(&SPIPins);


}
void SPI2_Inits(){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // generate sclk of 8MHz
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management

	SPI_Init(&SPI2handle);
}
void GPIO_ButtonInit(void){
		GPIO_Handle_t GPIOBtn;
		GPIOBtn.pGPIOx = GPIOA;
		GPIOBtn.pGPIO_PinConfig.PinNumber = GPIO_PIN_N0;
		GPIOBtn.pGPIO_PinConfig.PinMode = GPIO_MODE_IN;
		GPIOBtn.pGPIO_PinConfig.PinSpeed = GPIO_SPEED_FAST;
		GPIOBtn.pGPIO_PinConfig.PinPuPdControl = GPIO_PIN_PU;



		GPIO_Init(&GPIOBtn);
}
void delay(void) {
	for (uint32_t i = 0; i < 500000 ; i ++);
}

int main() {
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;


	SPI2_GPIOBInits();

	SPI2_Inits();

	// enabling SSOE for SPI peripheral

	SPI_SSOEConfig(SPI2,ENABLE);
	// enabling SSI for SPI peripheral
	//SPI_SSIConfig(SPI2, ENABLE);
	//enable the SPI2 peripheral
	while(1){

	while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_N0));


	delay();

	SPI_PeripheralControl(SPI2, ENABLE);


	// 1. cmd_led_ctrl	<pin no(1)>  <value(1)>
	uint8_t commandcode = COMMAND_LED_CTRL;
	uint8_t ackbyte;
	uint8_t args[2];
	SPI_sendData(SPI2,&commandcode,1);
	// we need dummy read here
	SPI_ReceiveData(SPI2,&dummy_read,1);

	// send some dummy bits (1 byte ) to fetch the response from the slave.

	SPI_sendData(SPI2,&dummy_write,1);
	SPI_ReceiveData(SPI2, &ackbyte, 1);

	// 1. send first command :




	if (SPI_VerifyResponse(ackbyte) ) {
		// send arguments
		args[0]= LED_PIN;
		args[1]= LED_ON;
		SPI_sendData(SPI2, args, 2);
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);
	}

	while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_N0));

	delay();
	commandcode = COMMAND_SENSOR_READ;
	// send second code :
	SPI_sendData(SPI2,&commandcode,1);
	// we need dummy read here
	SPI_ReceiveData(SPI2,&dummy_read,1);

	// send some dummy bits (1 byte ) to fetch the response from the slave.

	SPI_sendData(SPI2,&dummy_write,1);
	SPI_ReceiveData(SPI2, &ackbyte, 1);

	// 1. send first command :




	if (SPI_VerifyResponse(ackbyte) ) {
		// send arguments
		args[0]= ANALOG_PIN0;
		SPI_sendData(SPI2, args, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		uint8_t analog_read;

		delay();
		SPI_sendData(SPI2,&dummy_write,1);
		SPI_ReceiveData(SPI2, &analog_read,1);


	}
	// to receive analog response


	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);


	}
	return 0;
}


