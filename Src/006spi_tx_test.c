/*
 * 006spi_tx_test.c
 *
 *  Created on: Oct 4, 2023
 *      Author: pc
 */
#include "stm32f446xx.h"
#include<string.h>

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
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // generate sclk of 8MHz
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}


int main() {
	char user_data[] = "Hello world";

	SPI2_GPIOBInits();

	SPI2_Inits();

	// enabling SSI for SPI peripheral
	SPI_SSIConfig(SPI2, ENABLE);
	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_sendData(SPI2,user_data,strlen(user_data));

	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
}
