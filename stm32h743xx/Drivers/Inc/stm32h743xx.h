/*
 * stm32h743xx.h
 *
 *  Created on: Jul 22, 2025
 *      Author: jayas
 */

#ifndef INC_STM32H743XX_H_
#define INC_STM32H743XX_H_

#include <stddef.h>
#include<stdint.h>

#define __vo volatile



/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0 ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 ((__vo uint32_t*)0xE000E10C)




/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */

#define NVIC_ICER0 ((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1 ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2 ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3 ((__vo uint32_t*)0xE000E18C)


/*
 * ARM Cortex Mx Processor NVIC Priority register Addresses
 */

#define  NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)




/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4


/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U   		/*!<explain this macro briefly here  */
#define DTCM_BASEADDR						0x20000000U  		/*!<explain this macro briefly here  */
#define AXISRAM_BASEADDR					0x24000000U 		/*!<explain this macro briefly here  */
#define SRAM1_BASEADDR						0x30000000U  		/*!<explain this macro briefly here  */
#define SRAM2_BASEADDR						0x30020000U  		/*!<explain this macro briefly here  */
#define SRAM4_BASEADDR						0x38000000U  		/*!<explain this macro briefly here  */
#define BACKUPSRAM_BASEADDR					0x38800000U 		/*!<explain this macro briefly here  */
#define ROM1_BASEADDR						0x1FFF0000U
#define ROM2_BASEADDR						0x1FFF4000U
#define SRAM 								DTCM_BASEADDR


/*
 * base addresses of Peripherals
 */


#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define APB3PERIPH_BASEADDR						0x50000000U
#define APB4PERIPH_BASEADDR						0x58000000U

#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x48020000U
#define AHB3PERIPH_BASEADDR						0x51000000U
#define AHB4PERIPH_BASEADDR						0x58020000U

/*
 * Base addresses of peripherals which are hanging on AHB4 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASEADDR                   (AHB4PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB4PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (AHB4PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (AHB4PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (AHB4PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 (AHB4PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 (AHB4PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 (AHB4PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					 (AHB4PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR                     (AHB4PERIPH_BASEADDR + 0x4400)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 *  : Complete for all other peripherals
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)
#define UART7_BASEADDR						(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR						(APB1PERIPH_BASEADDR + 0x7C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)




/*
 * Base addresses of peripherals which are hanging on APB2 bus
 *  : Complete for all other peripherals
 */

#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR						(APB2PERIPH_BASEADDR + 0x5000)

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)



/*
 * Base addresses of peripherals which are hanging on APB4 bus
 * TODO : Complete for all other peripherals
 */
#define SPI6_BASEADDR						(APB4PERIPH_BASEADDR + 0x1400)

#define I2C4_BASEADDR						(APB4PERIPH_BASEADDR + 0x1C00)

#define EXTI_BASEADDR						(APB4PERIPH_BASEADDR + 0x0000)

#define SYSCFG_BASEADDR						(APB4PERIPH_BASEADDR + 0x0400)



/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32H7x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Hx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;                       /*!< ,     										Address offset: 0x04      */
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;




typedef struct
{
	 __vo uint32_t CR;            /*!< ,     										Address offset: 0x00 */
	  __vo uint32_t ICSCR;       /*!< TODO,     										Address offset: 0x04 */
	 // __vo uint32_t HSICFGR;          /*!< TODO,     										Address offset: 0x08 */
	  __vo uint32_t CRRCR;           /*!< TODO,     										Address offset: 0x0C */
	  uint32_t RESERVED0;
	 // __vo uint32_t CSICFGR;      /*!< TODO,     										Address offset: 0x10 */
	  __vo uint32_t CFGR;      /*!< TODO,     										Address offset: 0x14 */
	   uint32_t RESERVED1;      /*!< TODO,     										Address offset: 0x18 */
	  __vo uint32_t D1CFGR;     /*!< Reserved, 0x1C                                                       */
	  __vo uint32_t D2CFGR;      /*!< TODO,     										Address offset: 0x20 */
	  __vo uint32_t D3CFGR;      /*!< TODO,     										Address offset: 0x24 */
	   uint32_t   RESERVED2;  /*!< Reserved, 0x28-0x2C                                                  */
	  __vo uint32_t PLLCLKSELR;       /*!< TODO,     										Address offset: 0x30 */
	  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x34 */
	  __vo uint32_t PLL1DIV4;       /*!< TODO,     										Address offset: 0x38 */
	  __vo uint32_t      PLL1FRACR;     /*!< Reserved, 0x3C                                                       */
	  __vo uint32_t PLL2DIVR;       /*!< TODO,     										Address offset: 0x40 */
	  __vo uint32_t PLL2FRACR;       /*!< TODO,     										Address offset: 0x44 */
	  __vo  uint32_t      PLL3DIVR;  /*!< Reserved, 0x48-0x4C                                                  */
	  __vo uint32_t PLL3FRACR;     /*!< TODO,     										Address offset: 0x50 */
	   uint32_t RESERVED3;     /*!< TODO,     										Address offset: 0x54 */
	  __vo uint32_t D1CCIPR;     /*!< TODO,     										Address offset: 0x58 */
	  __vo  uint32_t  D2CCIP1R;     /*!< Reserved, 0x5C                                                       */
	  __vo uint32_t D2CCIP2R;     /*!< TODO,     										Address offset: 0x60 */
	  __vo uint32_t D3CCIPR;     /*!< RTODO,     										Address offset: 0x64 */
	  uint32_t RESERVED4;
	  __vo uint32_t      CIER;  /*!< Reserved, 0x68-0x6C                                                  */
	  __vo uint32_t CIFR;          /*!< TODO,     										Address offset: 0x70 */
	  __vo uint32_t CICR;           /*!< TODO,     										Address offset: 0x74 */
	   uint32_t      RESERVED5;  /*!< Reserved, 0x78-0x7C                                                  */
	  __vo uint32_t BDCR;         /*!< TODO,     										Address offset: 0x80 */
	  __vo uint32_t CSR;    /*!< TODO,     										Address offset: 0x84 */
	   uint32_t RESERVED6;    /*!< TODO,     										Address offset: 0x88 */
	  __vo uint32_t AHB3RSTR;       /*!< TODO,     										Address offset: 0x8C */
	  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x90 */
	  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x94 */

	  __vo uint32_t AHB4RSTR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB3RSTR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB1LRSTR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB1HRSTR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB4RSTR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t RCC_GCR;      /*!< TODO,     										Address offset: 0x94 */
	   uint32_t RESERVED7;      /*!< TODO,     										Address offset: 0x94 */

	  __vo uint32_t D3AMR;      /*!< TODO,     										Address offset: 0x94 */
	  uint32_t RESERVED8[9];
	  __vo uint32_t RSR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t AHB3ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t AHB1ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t AHB2ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t AHB4ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB3ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB1LENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB1HENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t  APB2ENR;      /*!< TODO,     										Address offset: 0x94 *///
	  __vo uint32_t APB4ENR;      /*!< TODO,     										Address offset: 0x94 */
	   uint32_t RESERVED9;      /*!< TODO,     										Address offset: 0x94 */

	  __vo uint32_t AHB3LPENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t AHB1LPENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t AHB2LPENR;
	  __vo uint32_t AHB4LPENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t  APB3LPENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB1LLPENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB1HLPENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB2LPENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t APB4LPENR;      /*!< TODO,     										Address offset: 0x94 */
	   uint32_t RESERVED10[5];      /*!< TODO,     										Address offset: 0x94 */

	  __vo uint32_t C1_AHB3ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t C1_AHB1ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t C1_AHB2ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t C1_AHB4ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t C1_APB3ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t C1_APB1LENR;      /*!< TODO,     										Address offset: 0x94 */

	  __vo uint32_t C1_APB1HENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t C1_APB2ENR;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t C1_APB4ENR;      /*!< TODO,     										Address offset: 0x94 */
	   uint32_t RESERVED11;      /*!< TODO,     										Address offset: 0x94 */
	  __vo uint32_t C1_AHB3LPENR;      /*!< ,     										Address offset: 0x94 */
	  __vo uint32_t C1_AHB1LPENR;      /*!< ,     										Address offset: 0x94 */
	  __vo uint32_t C1_AHB2LPENR;      /*!< ,     										Address offset: 0x94 */
	  __vo uint32_t C1_AHB4LPENR;      /*!< ,     										Address offset: 0x94 */
	  __vo uint32_t C1_APB3LPENR;      /*!< ,     										Address offset: 0x94 */
	  __vo uint32_t C1_APB1LLPENR;      /*!< ,     										Address offset: 0x94 */
	  __vo uint32_t C1_APB1HLPENR;      /*!< ,     										Address offset: 0x94 */
	  __vo uint32_t C1_APB2LPENR;      /*!< ,     										Address offset: 0x94 */
	  __vo uint32_t C1_APB4LPENR;      /*!< ,     										Address offset: 0x94 */
	   uint32_t RESERVED12[32];      /*!< ,     										Address offset: 0x94 */

}RCC_RegDef_t;


typedef struct
{
		__vo uint32_t RTSR1;
		__vo uint32_t FTSR1;
		__vo uint32_t SWIER1;
		__vo uint32_t D3PMR1;
		__vo uint32_t D3PCR1L;
		__vo uint32_t D3PCR1H;
		 uint32_t RESERVED0[2];
		__vo uint32_t RTSR2;
		__vo uint32_t FTSR2;
		__vo uint32_t SWIER2;
		__vo uint32_t D3PMR2;
		__vo uint32_t D3PCR2L;
		__vo uint32_t D3PCR2H;
		 uint32_t RESERVED1[2];
		__vo uint32_t RTSR3;
		__vo uint32_t FTSR3;
		__vo uint32_t SWIER3;
		__vo uint32_t D3PMR3;
		__vo uint32_t D3PCR3L;
		__vo uint32_t D3PCR3H;
		 uint32_t RESERVED2[10];

		 __vo uint32_t IMR1;
		 __vo uint32_t EMR1;
		 __vo uint32_t PR1;
		 uint32_t RESERVED3;
		 __vo uint32_t IMR2;
		 __vo uint32_t EMR2;
		 __vo uint32_t PR2;
		 uint32_t RESERVED4;
		 __vo uint32_t IMR3;
		 __vo uint32_t EMR3;
		 __vo uint32_t PR3;
		 uint32_t RESERVED5[6];
}EXTI_RegDef_t;


typedef struct
{
		uint32_t RESERVED1;
		__vo uint32_t PMCR;
		__vo uint32_t EXTICR[4];
		__vo uint32_t CFGR;
		uint32_t RESERVED2;
		__vo uint32_t CCSR;
		__vo uint32_t CCVR;
		__vo uint32_t CCCR;
		__vo uint32_t PWRCR;
		 uint32_t RESERVED3[61];
		__vo uint32_t PKGR;
		 uint32_t RESERVED4[118];
		__vo uint32_t UR0;
		 uint32_t RESERVED5;
		__vo uint32_t UR2;
		__vo uint32_t UR3;
		__vo uint32_t UR4;
		__vo uint32_t UR5;
		 __vo uint32_t UR6;
		 __vo uint32_t UR7;
		 __vo uint32_t UR8;
		 __vo uint32_t UR9;
		 __vo uint32_t UR10;
		 __vo uint32_t UR11;
		 __vo uint32_t UR12;
		 __vo uint32_t UR13;
		 __vo uint32_t UR14;
		 __vo uint32_t UR15;
		 __vo uint32_t UR16;
		 __vo uint32_t UR17;

}SYSCFG_RegDef_t;





/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)


#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*
 clock enable macros for gpio peripherals
 */

#define GPIOA_PCLK_EN() 	(RCC->AHB4ENR |=(1<<0))
#define GPIOB_PCLK_EN()	    (RCC->AHB4ENR |=(1<<1))          							//(RCC->AHB4ENR |=(1<<1))
#define GPIOC_PCLK_EN() 	(RCC->AHB4ENR |=(1<<2))
#define GPIOD_PCLK_EN() 	(RCC->AHB4ENR |=(1<<3))
#define GPIOE_PCLK_EN() 	(RCC->AHB4ENR |=(1<<4))
#define GPIOF_PCLK_EN() 	(RCC->AHB4ENR |=(1<<5))
#define GPIOG_PCLK_EN() 	(RCC->AHB4ENR |=(1<<6))
#define GPIOH_PCLK_EN() 	(RCC->AHB4ENR |=(1<<7))
#define GPIOI_PCLK_EN() 	(RCC->AHB4ENR |=(1<<8))



/*
 clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() 	(RCC->APB1LENR |=(1<<21))
#define I2C2_PCLK_EN() 	(RCC->APB1LENR |=(1<<22))
#define I2C3_PCLK_EN() 	(RCC->APB1LENR |=(1<<23))
#define I2C4_PCLK_EN() 	(RCC->APB4ENR |=(1<<7))

/*
 clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() 	(RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN() 	(RCC->APB1LENR |=(1<<14))
#define SPI3_PCLK_EN() 	(RCC->APB1LENR |=(1<<15))
#define SPI4_PCLK_EN() 	(RCC->APB2ENR |=(1<<13))
#define SPI5_PCLK_EN() 	(RCC->APB2ENR |=(1<<20))
#define SPI6_PCLK_EN() 	(RCC->APB4ENR |=(1<<5))

/*
 clock enable macros for USARTx peripherals
 */
#define USART1_PCLK_EN() 	(RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN() 	(RCC->APB1LENR |=(1<<17))
#define USART3_PCLK_EN() 	(RCC->APB1LENR |=(1<<18))
#define USART6_PCLK_EN() 	(RCC->APB2ENR |=(1<<5))

#define UART4_PCLK_EN() 	(RCC->APB1LENR |=(1<<19))
#define UART5_PCLK_EN() 	(RCC->APB1LENR |=(1<<20))
#define UART7_PCLK_EN() 	(RCC->APB1LENR |=(1<<30))
#define UART8_PCLK_EN() 	(RCC->APB1LENR |=(1<<31))


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB4ENR |= (1 << 1))



/*
 clock disable macros for gpio peripherals
 */

#define GPIOA_PCLK_DI() 	(RCC->AHB4ENR &= ~(1<<0))
#define GPIOB_PCLK_DI() 	(RCC->AHB4ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() 	(RCC->AHB4ENR &= ~(1<<2))
#define GPIOD_PCLK_DI() 	(RCC->AHB4ENR &= ~(1<<3))
#define GPIOE_PCLK_DI() 	(RCC->AHB4ENR &= ~(1<<4))
#define GPIOF_PCLK_DI() 	(RCC->AHB4ENR &= ~(1<<5))
#define GPIOG_PCLK_DI() 	(RCC->AHB4ENR &= ~(1<<6))
#define GPIOH_PCLK_DI() 	(RCC->AHB4ENR &= ~(1<<7))
#define GPIOI_PCLK_DI() 	(RCC->AHB4ENR &= ~(1<<8))


/*
 clock disable macros for I2Cx peripherals
 */
#define I2C2_PCLK_DI() 	(RCC->APB1LENR &= ~(1<<22))
#define I2C3_PCLK_DI() 	(RCC->APB1LENR &= ~(1<<23))
#define I2C4_PCLK_DI() 	(RCC->APB4ENR &= ~(1<<7))

/*
 clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI() 	(RCC->APB1LENR &= ~(1<<14))
#define SPI3_PCLK_DI()	(RCC->APB1LENR &= ~(1<<15))
#define SPI4_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<20))
#define SPI6_PCLK_DI() 	(RCC->APB4ENR &= ~(1<<5))

/*
 clock disable macros for USARTx peripherals
 */
#define USART1_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI() 	(RCC->APB1LENR &= ~(1<<17))
#define USART3_PCLK_DI() 	(RCC->APB1LENR &= ~(1<<18))
#define USART6_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<5))

#define UART4_PCLK_DI()	(RCC->APB1LENR &= ~(1<<19))
#define UART5_PCLK_DI() 	(RCC->APB1LENR &= ~(1<<20))
#define UART7_PCLK_DI() 	(RCC->APB1LENR &= ~(1<<30))
#define UART8_PCLK_DI() 	(RCC->APB1LENR &= ~(1<<31))


/*
 * Clock disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() (RCC->APB4ENR &= ~(1 << 1))


/*
 *			Register reset macros
 */
#define GPIOA_REG_RST()		do{(RCC->AHB4RSTR |=(1<<0));    (RCC->AHB4RSTR &=~(1<<0));}while(0)
#define GPIOB_REG_RST()		do{(RCC->AHB4RSTR |=(1<<1));    (RCC->AHB4RSTR &=~(1<<1));}while(0)
#define GPIOC_REG_RST()		do{(RCC->AHB4RSTR |=(1<<2));    (RCC->AHB4RSTR &=~(1<<2));}while(0)
#define GPIOD_REG_RST()		do{(RCC->AHB4RSTR |=(1<<3));    (RCC->AHB4RSTR &=~(1<<3));}while(0)
#define GPIOE_REG_RST()		do{(RCC->AHB4RSTR |=(1<<4));    (RCC->AHB4RSTR &=~(1<<4));}while(0)
#define GPIOF_REG_RST()		do{(RCC->AHB4RSTR |=(1<<5));    (RCC->AHB4RSTR &=~(1<<5));}while(0)
#define GPIOG_REG_RST()		do{(RCC->AHB4RSTR |=(1<<6));    (RCC->AHB4RSTR &=~(1<<6));}while(0)
#define GPIOH_REG_RST()		do{(RCC->AHB4RSTR |=(1<<7));    (RCC->AHB4RSTR &=~(1<<7));}while(0)
#define GPIOI_REG_RST()		do{(RCC->AHB4RSTR |=(1<<8));    (RCC->AHB4RSTR &=~(1<<8));}while(0)




/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)        		((x==GPIOA)?0:\
												(x == GPIOB)?1:\
												(x == GPIOC)?2:\
												(x == GPIOD)?3:\
										        (x == GPIOE)?4:\
										        (x == GPIOF)?5:\
										        (x == GPIOG)?6:\
										        (x == GPIOH)?7:\
										        (x == GPIOI)?8:0)

/*
 * IRQ(Interrupt Request) Numbers of STM32H743x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15


//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET








#endif /* INC_STM32H743XX_H_ */

//
