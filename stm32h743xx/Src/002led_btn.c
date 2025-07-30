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
		gpioled.pGPIOx=GPIOB;
		gpioled.GPIO_Pin_Config.GPIO_PinNumber=GPIO_PIN_NO_7;
		gpioled.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
		gpioled.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OP_TYPE_PP;
		gpioled.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_SPEED_HIGH;
		gpioled.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PU_PD;



	GPIO_Periclkcontrol(GPIOB,ENABLE);

	GPIO_Init(&gpioled);


	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_Pin_Config.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_Pin_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_Pin_Config.GPIO_PinPuPdControl = GPIO_NO_PU_PD;

	GPIO_Periclkcontrol(GPIOC,ENABLE);

	GPIO_Init(&GPIOBtn);
	while(1)
	{
		if(GPIO_ReadfromInputPin(GPIOC,GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			delay();

			GPIO_ToggleOutputPin(GPIOB,GPIO_PIN_NO_7);
		}
	}
	return 0;
}
