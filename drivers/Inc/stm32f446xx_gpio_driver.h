/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Sep 24, 2023
 *      Author: pc
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_



#include "stm32f446xx.h"
#include <stdint.h>

typedef struct {
	uint8_t PinNumber;   			/* possible values from @GPIO_PIN_NUMBER	*/
	uint8_t PinMode;				/* possible values from  @gpio_pin_modes */
	uint8_t PinSpeed;				/* possible values from  @gpio_pin_speed */
	uint8_t PinPuPdControl;			/* possible values from  @gpio_pin_PuPdControl */
	uint8_t PinOPType;				/* possible values from  @gpio_pin_OPType */
	uint8_t PinAltFunMode;
}GPIO_PinConfig_t;
typedef struct {

	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t pGPIO_PinConfig;

}GPIO_Handle_t;

/* gpio pin mode
 * @gpio_pin_modes
 */
#define GPIO_PIN_N0 		0
#define GPIO_PIN_N1 		1
#define GPIO_PIN_N2 		2
#define GPIO_PIN_N3 		3
#define GPIO_PIN_N4 		4
#define GPIO_PIN_N5 		5
#define GPIO_PIN_N6 		6
#define GPIO_PIN_N7 		7
#define GPIO_PIN_N8 		8
#define GPIO_PIN_N9 		9
#define GPIO_PIN_N10 		10
#define GPIO_PIN_N11 		11
#define GPIO_PIN_N12 		12
#define GPIO_PIN_N13 		13
#define GPIO_PIN_N14 		14
#define GPIO_PIN_N15 		15
#


/*
 * GPIO Pin numbers
 * @GPIO_PIN_NUMBER
 *  */

#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTEN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * gpio pin output type
 * @gpio_pin_OPType
 */

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1


/* gpio pin speed
 * @gpio_pin_speed
 * */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


/* pull up pull down configuration
 * @gpio_pin_PuPdControl
 * */

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t * pGPIOx);

/*
 * read from and write to pin/port
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);


/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t IRQNumber);









#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
