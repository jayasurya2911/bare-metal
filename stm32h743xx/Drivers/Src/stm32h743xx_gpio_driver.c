/*
 * stm32h743xx_gpio_driver.c
 *
 *  Created on: Jul 23, 2025
 *      Author: jayas
 */

#include "stm32h743xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Periclkcontrol(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_PCLK_EN();
			}else if (pGPIOx == GPIOG)
			{
				GPIOG_PCLK_EN();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();
			}else if (pGPIOx == GPIOI)
			{
				GPIOI_PCLK_EN();
			}
		}
		else
		{
			   if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}

		}
}



/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral and gpio pin config
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{;
	uint32_t temp=0;
	//1 . configure the mode of gpio pin

	GPIO_Periclkcontrol(pGPIOHandle->pGPIOx, ENABLE);

	if(pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode<=GPIO_MODE_ANALOG)
	{
		temp=(pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode<<(2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<(2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));   // clearing
		pGPIOHandle->pGPIOx->MODER |=temp;

	}
	else
	{
				//interrupt mode
		if(pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode==GPIO_MODE_IT_FT)
		{
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
				// 1 configure ftsr
			EXTI->FTSR1	|=(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
			//clear the corresponding rtsr register
			EXTI->RTSR1 &= ~(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);


		}else if(pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode==GPIO_MODE_IT_RT)
		{pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
			// 1 configure rtsr
			EXTI->RTSR1	|=(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
				//clear the corresponding rtsr register
			EXTI->FTSR1 &= ~(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode==GPIO_MODE_IT_RFT)
		{pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));
			// 1 configure ftsr and rtsr both
			EXTI->RTSR1	|=(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
			//clear the corresponding rtsr register
			EXTI->FTSR1 |= (1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);


		}
			//2  configure syscfg register
			uint8_t temp1 = pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber / 4 ;
			uint8_t temp2 = (pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber % 4)*4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] &= ~(0xFU << temp2);
			SYSCFG->EXTICR[temp1] |= portcode<<(temp2);

	// 3 enable the exti  interrupt delivery using IMR
			EXTI->IMR1 |=(1<<pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
	}


	temp=0;
	//1 . configure the output speed
	 temp=pGPIOHandle->GPIO_Pin_Config.GPIO_PinSpeed<<(2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
	 pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<(2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));			// clearing
	 pGPIOHandle->pGPIOx->OSPEEDR |=temp;

temp=0;

	//1 . configure the output type

	temp= pGPIOHandle->GPIO_Pin_Config.GPIO_PinOPType<<(pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<(pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber));		// clearing
	 pGPIOHandle->pGPIOx->OTYPER |=temp;

temp=0;
	//1 . configure the pull-up or pull down settings

	temp= pGPIOHandle->GPIO_Pin_Config.GPIO_PinPuPdControl<<( 2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber);
	 pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber) );		// clearing
	 pGPIOHandle->pGPIOx->PUPDR |=temp;

	//1 . configure the alternate functionality of the pin

	 if(pGPIOHandle->GPIO_Pin_Config.GPIO_PinMode == GPIO_MODE_ALTFN)
	 {
		 //configure alt functionality
		 uint32_t temp1,temp2;
		 temp1= pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber/8;
		 temp2=pGPIOHandle->GPIO_Pin_Config.GPIO_PinNumber%8;
		 pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2) );  // clearing
		 pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_Pin_Config.GPIO_PinAltFunMode<< (4 * temp2) );  // setting


	 }









}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function initializes given GPIO port
 * @param[in]         - base address of the gpio peripheral means just gpio port name
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
				{
					GPIOA_REG_RST();
				}else if (pGPIOx == GPIOB)
				{
					GPIOB_REG_RST();
				}else if (pGPIOx == GPIOC)
				{
					GPIOC_REG_RST();
				}else if (pGPIOx == GPIOD)
				{
					GPIOD_REG_RST();
				}else if (pGPIOx == GPIOE)
				{
					GPIOE_REG_RST();
				}else if (pGPIOx == GPIOF)
				{
					GPIOF_REG_RST();
				}else if (pGPIOx == GPIOG)
				{
					GPIOG_REG_RST();
				}else if (pGPIOx == GPIOH)
				{
					GPIOH_REG_RST();
				}else if (pGPIOx == GPIOI)
				{
					GPIOI_REG_RST();
				}



}


/*********************************************************************
 * @fn      		  - GPIO_ReadfromInputPin
 *
 * @brief             - This function helps us to read the data from the particular pin of a particular port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pin number
 * @param[in]         -
 *
 * @return            -  value which is in the pin 8 bit just 0 or  1
 *
 * @Note              -  none

 */
uint8_t GPIO_ReadfromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t Pinnumber)
{
	uint8_t value;
	value= (uint8_t)(pGPIOx->IDR >> Pinnumber) & 0x00000001;
	return value;

}

/*********************************************************************
 * @fn      		  - GPIO_ReadfromInputPort
 *
 * @brief             - This function helps us to read the data from a particular port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  value which is stored in the entire port 16 bit because 16 pin in a single port
 *
 * @Note              -  none

 */
uint16_t GPIO_ReadfromInputPort(GPIO_RegDef_t *pGPIOx)
{
		uint16_t value;
		value=(uint16_t)(pGPIOx->IDR);
		return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WritetoOutputPin
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t Pinnumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write  1  at the corresponding bit field corresponding to the pin number
		pGPIOx->ODR |=(1<< Pinnumber);
	}
	else
	{
		pGPIOx->ODR &=~(1<< Pinnumber);
	}

}

/*********************************************************************
 * @fn      		  - GPIO_WritetoOutputPort
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |=Value;

}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t Pinnumber )
{

	pGPIOx->ODR  ^= ( 1 << Pinnumber);

}


/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQConfig(uint8_t IRQNumber,  uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(IRQNumber<=31)
		{
			*NVIC_ISER0 |=(1<<IRQNumber);

		}
		else if(IRQNumber>31 &&  IRQNumber<64)
		{
			*NVIC_ISER1 |=((1<<IRQNumber % 32));
		}
		else if(IRQNumber>=64 &&  IRQNumber<=95)
		{
			*NVIC_ISER2 |=((1<<IRQNumber % 64));
		}
	}
	else{
		if(IRQNumber<=31)
				{
					*NVIC_ICER0 |=(1<<IRQNumber);
				}
				else if(IRQNumber>31 &&  IRQNumber<64)
				{
					*NVIC_ICER1 |=((1<<IRQNumber % 32));
				}
				else if(IRQNumber>=64 &&  IRQNumber<=95)
				{
					*NVIC_ICER2 |=((1<<IRQNumber % 64));
				}

	}


}





void GPIO_IRQPriorityConfig(uint32_t IRQNumber,uint32_t IRQPriority)
{

			uint8_t iprx= IRQNumber/4;
			uint8_t iprx_section= IRQNumber%4;

			uint32_t shift_amount = (8 * iprx_section)+(8 - NO_PR_BITS_IMPLEMENTED);
			*(NVIC_PR_BASE_ADDR + iprx	)=(IRQPriority<<shift_amount);


}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - pin number
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR1 & (1<<PinNumber))
		{
		EXTI->PR1 |= (1<<PinNumber);
		}

}

