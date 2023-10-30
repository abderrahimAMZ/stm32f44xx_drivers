/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Oct 14, 2023
 *      Author: pc
 */

#include "stm32f446xx.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,32,64,128,256,512};
uint16_t APB1_PreScaler[8] = {2,4,8,16};


static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ClearADDRFlag(I2C_RegDef_t* pI2Cx);

uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1,SystemClk;
	uint8_t clkrsc;
	uint8_t ahbp,apb1p;
	uint8_t temp;

	clkrsc = (RCC->CFGR >> 2) & 0x3;

	if (clkrsc == 0){
		SystemClk = 16000000;

	}else if (clkrsc == 1){
		SystemClk = 8000000;
	}else if (clkrsc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}
	//ahb1
	temp = (RCC->CFGR >> 4) & 0xf;

	if (temp <8)
	{
		ahbp = 1;
	}
	else {
		ahbp = AHB_PreScaler[temp-8];
	}
	//apb1
	temp = (RCC->CFGR >> 10) & 0x7;
	if (temp <4)
		{
			apb1p = 1;
		}
		else {
			apb1p = APB1_PreScaler[temp -4 ];
		}
	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;

}
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE) {

			if (pI2Cx == I2C1){
				I2C1_PCLK_EN();
			}
			else if (pI2Cx == I2C2){
				I2C2_PCLK_EN();
			}
			else if (pI2Cx == I2C3){
				I2C3_PCLK_EN();
			}


		}
		else {
			if (pI2Cx == I2C1){
				I2C1_PCLK_DI();
			}
			else if (pI2Cx == I2C2){
				I2C2_PCLK_DI();
			}
			else if (pI2Cx == I2C3){
				I2C3_PCLK_DI();
			}


}
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if ( EnOrDi == ENABLE){
		pI2Cx->CR1 |= (I2C_CR1_PE << 0);
	}else {
		pI2Cx->CR1 &= ~(I2C_CR1_PE << 0);
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle){
	// enable the clock:
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t tempreg = 0;

	// configure ACK
	tempreg = pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	tempreg = 0;
	// configure the FREQ:
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= tempreg & 0x3f;

	// configure address
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= 1 << 14;
	pI2CHandle->pI2Cx->OAR1 |=  tempreg;
	// configure CCR:
	uint16_t ccr_value = 0;
	tempreg =0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		// mode is standard
		ccr_value = RCC_GetPCLK1Value() / (2* pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= ccr_value & 0xfff;
	}else {
		// mode is fast:
		tempreg |= (1 << 15); // to put mode to fast
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = RCC_GetPCLK1Value() / (3* pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else {
			ccr_value = RCC_GetPCLK1Value() / (2* pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xfff);

	}
	pI2CHandle->pI2Cx->CCR = tempreg;



}

void I2C_DeInit(I2C_RegDef_t * pI2Cx){
	if (pI2Cx == I2C1){
		I2C1_REG_RESET();
	}else if (pI2Cx == I2C1){
		I2C1_REG_RESET();
	}else if (pI2Cx == I2C1){
		I2C1_REG_RESET();
	}
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr){
  // 1. generate the start condition
  I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
    // 2. confirm that start generation is completed by checking the SB flag in the SR1
    // Note: Until SB is cleared SCl will be stretched (pull to LOW)
    while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

    //3. Send the address of the slave with r/nw bit set to w(0)
    I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

    //4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
    while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    //5. clear the ADDR flag according to its software sequence
    // Note: until ADDR is cleared SCL will be stretched (pulled to LOW)

    I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

    //6. send the data until len becomes 0;
    while (Len > 0)
    {
    	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
    	pI2CHandle->pI2Cx->DR = *pTxbuffer;
    	pTxbuffer++;
    	Len++;
    }

    // when Len becomes zero wait for TXE=1 and BTF=1 before generating the Stop condition
    // Note: TxE=1 , BTF=1, means that both SR and DR are empty and next transmission should begin
    // when BTF=1 SCL will be stretched (pulled to LOW)
    while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
    while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));
    //8. Generate stop condition and master need not to wiat for the completion of stop condition.
    //	Note : generating Stop, automatically clears the BTF.
    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


}




void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi){
    if (EnOrDi == ENABLE){
        pI2Cx->CR1 |= (1 << I2C_CR1_START);
    }
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
    if (pI2Cx->SR1 & FlagName){
        return FLAG_SET;
    }
    return FLAG_RESET;
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); // SlaveAddr is Slave address + r/w
	pI2Cx->DR = SlaveAddr;

}


static void I2C_ClearADDRFlag(I2C_RegDef_t* pI2Cx){
	uint32_t dummyRead = pI2Cx->SR1;
	 dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

}









