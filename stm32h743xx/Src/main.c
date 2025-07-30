
#include "stm32h743xx.h"

#include "stm32h743xx_gpio_driver.h"

void delay()
{
	for( __vo uint32_t i=0;i<500000;i++);
}

int main(void)
{
	GPIO_Handle_t gpioled;
	gpioled.pGPIOx=GPIOB;
	gpioled.GPIO_Pin_Config.GPIO_PinNumber=GPIO_PIN_NO_7;
	gpioled.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	gpioled.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	gpioled.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_SPEED_HIGH;
	gpioled.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PU_PD;


	 GPIO_Periclkcontrol(GPIOB, ENABLE);

	 GPIO_Init(&gpioled);

	while(1)
	{
		 GPIO_ToggleOutputPin(GPIOB,GPIO_PIN_NO_7);

		 delay();


	}





	return 0;
}
