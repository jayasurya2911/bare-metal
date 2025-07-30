/*
 * 003button_interrupt.c
 *
 *  Created on: Jul 29, 2025
 *      Author: jayas
 */


/*
 * 002led_btn.c
 *
 *  Created on: Jul 27, 2025
 *      Author: jayas
 */
#include "stm32h743xx.h"

#include "stm32h743xx_gpio_driver.h"


#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

	GPIO_Handle_t gpioled, GPIOBtn;


//	memset(&GpioLed,0,sizeof(GpioLed));
//	memset(&GPIOBtn,0,sizeof(GpioLed));
//
		gpioled.pGPIOx=GPIOB;
		gpioled.GPIO_Pin_Config.GPIO_PinNumber=GPIO_PIN_NO_14;
		gpioled.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
		gpioled.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OP_TYPE_PP;
		gpioled.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_SPEED_HIGH;
		gpioled.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PU_PD;



	GPIO_Periclkcontrol(GPIOB,ENABLE);

	GPIO_Init(&gpioled);


	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_Pin_Config.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOBtn.GPIO_Pin_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_Pin_Config.GPIO_PinPuPdControl = GPIO_NO_PU_PD;

	GPIO_Periclkcontrol(GPIOC,ENABLE);

	GPIO_Init(&GPIOBtn);

	//IRQ configurations
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10,NVIC_IRQ_PRI15);
	GPIO_IRQConfig	(IRQ_NO_EXTI15_10,ENABLE);

	 while(1);
	return 0;
}

void EXTI15_10_IRQHandler(void)
{

	 /// delay(); //200ms . wait till button de-bouncing gets over
		GPIO_IRQHandling(GPIO_PIN_NO_13); //clear the pending event from exti line
		GPIO_ToggleOutputPin(GPIOB,GPIO_PIN_NO_14);

}
