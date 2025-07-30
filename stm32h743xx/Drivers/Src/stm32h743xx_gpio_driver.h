/*
 * stm32h743xx_gpio_driver.h
 *
 *  Created on: Jul 23, 2025
 *      Author: jayas
 */

#ifndef INC_STM32H743XX_GPIO_DRIVER_H_
#define INC_STM32H743XX_GPIO_DRIVER_H_

#include "stm32h743xx.h"

typedef struct
{
		uint8_t GPIO_PinNumber;
		uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
		uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
		uint8_t GPIO_PinPuPdControl;
		uint8_t GPIO_PinOPType;
		uint8_t GPIO_PinAltFunMode;

}GPIO_Pin_Config_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;					/*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_Pin_Config_t GPIO_Pin_Config;			/*!< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2 				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15




/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin possible OP types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1


/*
 * @GPIO_PIN_OUTPUT_SPEEDS
 * GPIO pin possible OP speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 *
 * GPIO pull up and pull down macros
 */
#define GPIO_NO_PU_PD		0
#define GPIO_PU				1
#define GPIO_PD				2


				/******************************************************************************************
				 *								APIs supported by this driver
				 *		 For more information about the APIs check the function definitions
				 ******************************************************************************************/


/*
 * peripheral clock control
 * */
void GPIO_Periclkcontrol(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi);

/*
 * init and denit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



/*
 * Data Read and Write
 * */
uint8_t GPIO_ReadfromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t Pinnumber);
uint16_t GPIO_ReadfromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t Pinnumber, uint8_t Value);
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t Pinnumber);

/*
 * IRQ Configuration  and ISRHandling
 * */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint32_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t Pinnumber);		// processor side(NVIC REGISTERS



















#endif /* INC_STM32H743XX_GPIO_DRIVER_H_ */
//
