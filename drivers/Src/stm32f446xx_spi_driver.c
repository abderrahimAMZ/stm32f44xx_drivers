/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Oct 1, 2023
 *      Author: pc
 */

#include "stm32f446xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);



void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE) {

		if (pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4){
			SPI4_PCLK_EN();
		}

	}
	else {
		if (pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4){
			SPI4_PCLK_DI();
		}

}

}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if (pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle){

	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	uint32_t tempreg = 0;

	//1.configure the device mode

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// enable bidi mode
		tempreg &= ~(1 << 15);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// enable half mode
		tempreg |= (1 << 15);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// enalbe RXONLY mode
		tempreg &= ~(1 << 15);
		tempreg |= (1 << 10);
	}


	// setting speed value
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << 3);

	// setting up data frame
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << 11);

	// setting up CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << 0);

	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << 1);


	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << 9);

	pSPIHandle->pSPIx->CR1 = tempreg;

}
void SPI_DeInit(SPI_RegDef_t * pSPIx){
			if (pSPIx == SPI1){
				SPI1_REG_RESET();
			}
			else if (pSPIx == SPI2){
				SPI2_REG_RESET();			}
			else if (pSPIx == SPI3){
				SPI3_REG_RESET();
			}
			else if (pSPIx == SPI4){
				SPI4_REG_RESET();
			}

}

/*
 * Data Send and Receive
 */

void SPI_sendData(SPI_RegDef_t * pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		// check if the TX buffer is empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_SET){
			// yey, it's empty, let's feed it some data!
			if (pSPIx->CR1 & 1 <<SPI_CR1_DFF){
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;

			(uint16_t*)pTxBuffer++;
			}
			else {
			pSPIx->DR = *(pTxBuffer);
			Len --;
			pTxBuffer++;
			}
		}
	}
}


void SPI_ReceiveData(SPI_RegDef_t * pSPIx,uint8_t *pRxBuffer, uint32_t Len){
	while(Len < 0){
		while(SPI_GetFlagStatus(pSPIx,SPI_SR_RXNE)== FLAG_SET){

			if(pSPIx->CR1 & 1 << SPI_CR1_DFF){
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}
			else {
				*(pRxBuffer) = pSPIx->DR;
				Len--;
				pRxBuffer++;

			}
		}
	}
}


void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if (EnorDi == ENABLE){
			// enable interrupt
			if (IRQNumber <= 31){
				*NVIC_ISER0 |= (1 << IRQNumber);
			}
			else if( IRQNumber > 31 && IRQNumber < 64) {
				*NVIC_ISER1 |= (1 << IRQNumber %32);
			}
			else if ( IRQNumber >= 64 && IRQNumber < 96){
				*NVIC_ISER2 |= (1 << IRQNumber %64);
			}
		}else {
			// desiable interrupt
			if (IRQNumber <= 31){
				*NVIC_ICER0 |= (1 << IRQNumber);
			}
			else if( IRQNumber > 31 && IRQNumber < 64) {
				*NVIC_ICER1 |= (1 << IRQNumber %32);
				}
			else if ( IRQNumber >= 64 && IRQNumber < 96){
				*NVIC_ICER2 |= (1 << IRQNumber %64);
				}
		}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber /4 ;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section);
	*(NVIC_PR_BASE_ADDR+ iprx) |= (IRQPriority << shift_amount);
}
void SPI_IRQHandling(SPI_Handle_t* pHandle){

	uint8_t temp1, temp2;

	// first lets check for TXE

	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2){
		spi_txe_interrupt_handle(pHandle);
	}
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);


	if (temp1 && temp2 ){

		spi_rxne_interrupt_handle(pHandle);
	}
	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2){
		spi_ovr_err_interrupt_handle(pHandle);
	}

}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE){
		pSPIx->CR1 |= 1 << SPI_CR1_SPE;
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE){
			pSPIx->CR1 |= 1 << SPI_CR1_SSI;
		}
		else {
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE){
				pSPIx->CR2 |= 1 << SPI_CR2_SSOE;
			}
			else {
				pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
			}
}




uint8_t SPI_sendDataIT(SPI_Handle_t* pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX){

	// savign infos to pSPIHandle
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;



	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

}
	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t * pSPIHandle,uint8_t *pRxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX){

	// savign infos to pSPIHandle
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;



	pSPIHandle->RxState = SPI_BUSY_IN_RX;

	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

}
	return state;
}




static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if (pSPIHandle->pSPIx->CR1 & 1 <<SPI_CR1_DFF)
			{
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;

			(uint16_t*)pSPIHandle->pTxBuffer++;
			}
	else {
				pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
	if (! pSPIHandle->TxLen){
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEvenCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);
			}


}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if (pSPIHandle->pSPIx->CR1 & 1 <<SPI_CR1_DFF)
	{
	*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
	pSPIHandle->RxLen--;
	pSPIHandle->RxLen--;

	(uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else {
		*pSPIHandle->pRxBuffer  = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;

		pSPIHandle->pRxBuffer++;
	}
	if (! pSPIHandle->RxLen){
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEvenCallBack(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}



static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t	temp;
	//1. clear the ovr flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEvenCallBack(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){

}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}


__weak void SPI_ApplicationEvenCallBack(SPI_Handle_t *pSPIHandle,uint8_t AppEv){

}




