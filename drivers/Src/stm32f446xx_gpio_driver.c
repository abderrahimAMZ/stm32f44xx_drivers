/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Sep 24, 2023
 *      Author: pc
 */


/*
 * Peripheral Clock setup
 */
#include <stdint.h>
#include "stm32f446xx_gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE) {

		if (pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else {
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}

		}
}

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	// enable the clock for the user without needing to do it manually;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	uint32_t temp=0;
	if (pGPIOHandle->pGPIO_PinConfig.PinMode <= GPIO_MODE_ANALOG){
		temp = pGPIOHandle->pGPIO_PinConfig.PinMode << (2 * pGPIOHandle->pGPIO_PinConfig.PinNumber );
		pGPIOHandle->pGPIOx->MODER |=(0x3 << pGPIOHandle->pGPIO_PinConfig.PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else {
		// later
	}
	temp = 0;

	temp = pGPIOHandle->pGPIO_PinConfig.PinSpeed << (2 * pGPIOHandle->pGPIO_PinConfig.PinNumber );
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->pGPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |=temp;

	temp = 0;

	temp = pGPIOHandle->pGPIO_PinConfig.PinPuPdControl << (2 * pGPIOHandle->pGPIO_PinConfig.PinNumber );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->pGPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |=temp;

	temp = 0;

	temp = pGPIOHandle->pGPIO_PinConfig.PinOPType << (2 * pGPIOHandle->pGPIO_PinConfig.PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->pGPIO_PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |=temp;

	if (pGPIOHandle->pGPIO_PinConfig.PinMode == GPIO_MODE_ALTEN) {
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->pGPIO_PinConfig.PinNumber/8;
		temp2 = pGPIOHandle->pGPIO_PinConfig.PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->pGPIO_PinConfig.PinAltFunMode << (4 * temp2));
	}

	else {

		if ( pGPIOHandle->pGPIO_PinConfig.PinMode == GPIO_MODE_IT_FT){
			//1. configure the FISR
			EXTI->FTSR |= (1 << pGPIOHandle->pGPIO_PinConfig.PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->pGPIO_PinConfig.PinNumber);
		}
		else if (pGPIOHandle->pGPIO_PinConfig.PinMode == GPIO_MODE_IT_RT){
			//1. configure the RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->pGPIO_PinConfig.PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->pGPIO_PinConfig.PinNumber);
		}
		else if (pGPIOHandle->pGPIO_PinConfig.PinMode == GPIO_MODE_IT_RFT){
			//1 . configure both
			EXTI->FTSR |= (1 << pGPIOHandle->pGPIO_PinConfig.PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->pGPIO_PinConfig.PinNumber);
		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->pGPIO_PinConfig.PinNumber /4;
		uint8_t temp2 = pGPIOHandle->pGPIO_PinConfig.PinNumber %4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

		//3. enable the exti interrupt delivery
		EXTI->IMR |= pGPIOHandle->pGPIO_PinConfig.PinNumber;
	}

	}


void GPIO_DeInit(GPIO_RegDef_t * pGPIOx){
	if (pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}


}



/*
 * read from and write to pin/port
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t )((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value){
	if (Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^= ( 1 << PinNumber);
}


/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber /4 ;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section);
	*(NVIC_PR_BASE_ADDR+ iprx) |= (IRQPriority << shift_amount);
}


void GPIO_IRQHandling(uint8_t PinNumber){
	if (EXTI->PR & ( 1 << PinNumber)){
		//clear by writing one again
		EXTI->PR |= ( 1 << PinNumber);
	}
}
