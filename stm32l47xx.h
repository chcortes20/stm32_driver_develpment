/*
 * stm32l47xx.h
 *
 *  Created on: Nov 7, 2021
 *      Author: Christian Cortes
 */

#ifndef INC_STM32L47XX_H_
#define INC_STM32L47XX_H_

#include <stdint.h>

#define __IO volatile

/********************************************* START : Processor Specific details ******************************************************
 * */

 /*
 * ARM Cortex Mx Processor NVIC ISERx  register Addresses
 */
#define NVIC_ISER0				((__IO uint32_t *)0xE000E100)
#define NVIC_ISER1				((__IO uint32_t *)0xE000E104)
#define NVIC_ISER2				((__IO uint32_t *)0xE000E108)
#define NVIC_ISER3				((__IO uint32_t *)0xE000E10C)

 /*
 * ARM Cortex Mx Processor NVIC ICERx  register Addresses
 */
#define NVIC_ICER0				((__IO uint32_t *)0xE000E180)
#define NVIC_ICER1				((__IO uint32_t *)0xE000E184)
#define NVIC_ICER2				((__IO uint32_t *)0xE000E188)
#define NVIC_ICER3				((__IO uint32_t *)0xE000E18C)



/*
 * ARM Cotex Mx Processor NVIC IPRx register Address
 */
#define NVIC_IPR_BASEADDR		((__IO uint32_t *)0xE000E400)

#define NO_IPR_BITS_IMPLEMENTED	4


#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

#define NVIC_IRQ_PRIO15			15

/*
 * base addresses for Flash and SRAM memories
*/

#define FLASH_BASEADDR				0x08000000U     /* !< the base address of flash memory */
#define SRAM1_BASEADDR				0x20000000U     /* !< the SRAM1 block 96 Kbytes */
#define SRAM2_BASEADDR				0x10000000U     /* !< the SRAM2 hardware parity check 32 Kbytes */
#define ROM							0x1FFF0000U
#define SRAM 						SRAM1_BASEADDR  /* !<just aliasing the SRAM1_BASEADDR */



/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x48000000U
#define AHB3PERIPH_BASE				0xA0000000U


/*
 *  Base addresses of the peripherals that are hanging on the AHB3 Bus
 */
#define FMC_BASEADDR				(AHB3PERIPH_BASE + 0x0000U)
#define QUADSPI_BASEADDR			(AHB3PERIPH_BASE + 0x1000U)


/*
 * Base addresses of the peripherals that are hanging in the AHB2 bus
 */
#define GPIOA_BASEADDR 				(AHB2PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR				(AHB2PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR				(AHB2PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR				(AHB2PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR				(AHB2PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR				(AHB2PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR				(AHB2PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR				(AHB2PERIPH_BASE + 0x1C00U)


/*
 * Base addresses of the peripherals that are hanging in the AHB1 bus
 */
#define DMA1_BASEADDR 				(AHB1PERIPH_BASE + 0x0000U)
#define DMA2_BASEADDR				(AHB1PERIPH_BASE + 0x0400U)
#define RCC_BASEADDR				(AHB1PERIPH_BASE + 0x1000U)
#define FLASH1_BASEADDR				(AHB1PERIPH_BASE + 0x2000U)
#define CRC_BASEADDR				(AHB1PERIPH_BASE + 0x3000U)
#define TSC_BASEADDR				(AHB1PERIPH_BASE + 0x4000U)


/*
 * Base addresses of the peripherals that are hanging in the APB2 bus
 */
#define SYSCFG_BASEADDR				(APB2PERIPH_BASE + 0x0000U)
#define VREFBUF_BASEADDR			(APB2PERIPH_BASE + 0x0030U)
#define COMP_BASEADDR				(APB2PERIPH_BASE + 0x0200U)
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x0400U)
#define FIREWALL_BASEADDR			(APB2PERIPH_BASE + 0x1C00U)
#define SDMMC1_BASEADDR				(APB2PERIPH_BASE + 0x2800U)
#define TIM1_BASEADDR				(APB2PERIPH_BASE + 0x2C00U)
#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000U)
#define TIM8_BASEADDR				(APB2PERIPH_BASE + 0x3400U)
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x3800U)
#define TIM15_BASEADDR				(APB2PERIPH_BASE + 0x4000U)
#define TIM16_BASEADDR				(APB2PERIPH_BASE + 0x4400U)
#define TIM17_BASEADDR				(APB2PERIPH_BASE + 0x4800U)
#define SAI1_BASEADDR				(APB2PERIPH_BASE + 0x5400U)
#define SAI2_BASEADDR				(APB2PERIPH_BASE + 0x5800U)
#define DFSDM1_BASEADDR				(APB2PERIPH_BASE + 0x6000U)



/*
 * Base addresses of the peripherals that are hanging in the APB1 bus
 */
#define TIM2_BASEADDR				(APB1PERIPH_BASE + 0x0000U)
#define TIM3_BASEADDR				(APB1PERIPH_BASE + 0x0400U)
#define TIM4_BASEADDR				(APB1PERIPH_BASE + 0x0800U)
#define TIM5_BASEADDR				(APB1PERIPH_BASE + 0x0C00U)
#define TIM6_BASEADDR				(APB1PERIPH_BASE + 0x1000U)
#define TIM7_BASEADDR				(APB1PERIPH_BASE + 0x1400U)
#define RTC_BASEADDR				(APB1PERIPH_BASE + 0x2800U)
#define WWDG_BASEADDR				(APB1PERIPH_BASE + 0x2C00U)
#define IWDG_BASEADDR				(APB1PERIPH_BASE + 0x3000U)
#define SPI2_BASEADDR				(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR				(APB1PERIPH_BASE + 0x3C00U)
#define USART2_BASEADDR				(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR				(APB1PERIPH_BASE + 0x4800U)
#define UART4_BASEADDR				(APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASEADDR				(APB1PERIPH_BASE + 0x5000U)
#define I2C1_BASEADDR				(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR				(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR				(APB1PERIPH_BASE + 0x5C00U)
#define CAN1_BASEADDR				(APB1PERIPH_BASE + 0x6400U)
#define PWR_BASEADDR				(APB1PERIPH_BASE + 0x7000U)
#define DAC1_BASEADDR				(APB1PERIPH_BASE + 0x7400U)
#define OPAMP_BASEADDR				(APB1PERIPH_BASE + 0x7800U)
#define LPTIM1_BASEADDR				(APB1PERIPH_BASE + 0x7C00U)
#define LPUART1_BASEADDR			(APB1PERIPH_BASE + 0x8000U)
#define SWPMI1_BASEADDR				(APB1PERIPH_BASE + 0x8800U)
#define LPTIM2_BASEADDR				(APB1PERIPH_BASE + 0x9400U)


/************************* peripheral register structure definitions **********************************/



typedef struct
{
	__IO uint32_t MODER;	/*!< GPIO port mode register, 		  		 	Address offset:0x00		*/
	__IO uint32_t OTYPER;	/*!< GPIO port output type register,  			Address offset:0x04		*/
	__IO uint32_t OSPEED;	/*!< GPIO port output speed register, 		 	Address offset:0x08		*/
	__IO uint32_t PUPDR;	/*!< GPIO port pull up/pull down register, 	 	Address offset:0x0C		*/
	__IO uint32_t IDR;		/*!< GPIO port input data register, 			Address offset:0x10		*/
	__IO uint32_t ODR;		/*!< GPIO port output data register, 		 	Address offset:0x14		*/
	__IO uint32_t BSRR;		/*!< GPIO port bit set/reset register, 		 	Address offset:0x18		*/
	__IO uint32_t LCKR;		/*!< GPIO port configuration lock register,  	Address offset:0x1C		*/
	__IO uint32_t AFR[2];	/*!< GPIO port alternate function registers, 	Address offset:0x20-0x24*/
	__IO uint32_t BRR;		/*!< GPIO port bit reset register, 			 	Address offset:0x28		*/
	__IO uint32_t ASCR;		/*!< GPIO port analog switch control register, 	Address offset:0x2C		*/
}GPIO_TypeDef;


typedef struct
{
	__IO uint32_t CR;			/*!< RCC control register, 		  		 									Address offset:0x00		*/
	__IO uint32_t ICSCR;		/*!< RCC internal clock source calibration register, 		  		 		Address offset:0x04		*/
	__IO uint32_t CFGR;			/*!< RCC configuration register, 		  		 							Address offset:0x08		*/
	__IO uint32_t PLLCFGR;		/*!< RCC PLL configuration register, 		  		 						Address offset:0x0C		*/
	__IO uint32_t PLLSAI1CFGR;	/*!< RCC register, 		  		 											Address offset:0x10		*/
	__IO uint32_t PLLSAI2CFGR;	/*!< RCC mode register, 		  		 									Address offset:0x14		*/
	__IO uint32_t CIER;			/*!< RCC interrupt enable register, 		  		 						Address offset:0x18		*/
	__IO uint32_t CIFR;			/*!< RCC interrupt flag register, 		  		 							Address offset:0x1C		*/
	__IO uint32_t CICR;			/*!< RCC interrupt clear register, 		  		 							Address offset:0x20		*/
	uint32_t RESERVED0;			/*!< Reserved register, 		  		 									Address offset:0x24		*/
	__IO uint32_t AHB1RSTR;		/*!< RCC AHB1 peripheral reset register, 		  		 					Address offset:0x28		*/
	__IO uint32_t AHB2RSTR;		/*!< RCC AHB2 peripheral reset register, 		  		 					Address offset:0x2C		*/
	__IO uint32_t AHB3RSTR;		/*!< RCC AHB3 peripheral reset register, 		  		 					Address offset:0x30		*/
	uint32_t RESERVED1;			/*!< RCC reserved register, 		  		 								Address offset:0x34		*/
	__IO uint32_t APB1RSTR1;	/*!< RCC APB1 peripheral reset register 1, 		  		 					Address offset:0x38		*/
	__IO uint32_t APB1RSTR2;	/*!< RCC APB1 peripheral reset register 2, 		  		 					Address offset:0x3C		*/
	__IO uint32_t APB2RSTR;		/*!< RCC APB2 peripheral reset register, 		  		 					Address offset:0x40		*/
	uint32_t RESERVED2;			/*!< RCC reserved register, 		  		 								Address offset:0x44		*/
	__IO uint32_t AHB1ENR;		/*!< RCC AHB1 peripheral clock enable register, 		  		 			Address offset:0x48		*/
	__IO uint32_t AHB2ENR;		/*!< RCC AHB2 peripheral clock enable register, 		  		 			Address offset:0x4C		*/
	__IO uint32_t AHB3ENR;		/*!< RCC AHB3 peripheral clock enable register, 		  		 			Address offset:0x50		*/
	uint32_t RESERVED3;			/*!< RCC reserved register, 		  		 								Address offset:0x54		*/
	__IO uint32_t APB1ENR1;		/*!< RCC APB1 peripheral clock enable register 1, 		  		 			Address offset:0x58		*/
	__IO uint32_t APB1ENR2;		/*!< RCC APB1 peripheral clock enable register 2, 		  		 			Address offset:0x5C		*/
	__IO uint32_t APB2ENR;		/*!< RCC APB2 peripheral clock enable register, 		  		 			Address offset:0x60		*/
	uint32_t RESERVED4;			/*!< RCC reserved register, 		  		 								Address offset:0x64		*/
	__IO uint32_t AHB1SMENR;	/*!< RCC AHB1 peripheral clocks enable in sleep and stop modes register,	Address offset:0x68		*/
	__IO uint32_t AHB2SMENR;	/*!< RCC AHB2 peripheral clocks enable in sleep and stop modes register,  	Address offset:0x6C		*/
	__IO uint32_t AHB3SMENR;	/*!< RCC AHB3 peripheral clocks enable in sleep and stop modes register, 	Address offset:0x70		*/
	uint32_t RESERVED5;			/*!< RCC reserved register, 		  		 								Address offset:0x74		*/
	__IO uint32_t APB1SMENR1;	/*!< RCC APB1 peripheral clocks enable in sleep and stop modes register 1, 	Address offset:0x78		*/
	__IO uint32_t APB1SMENR2;	/*!< RCC APB1 peripheral clocks enable in sleep and stop modes register 2, 	Address offset:0x7C		*/
	__IO uint32_t APB2SMENR;	/*!< RCC APB2 peripheral clocks enable in sleep and stop modes register, 	Address offset:0x80		*/
	uint32_t RESERVED6;			/*!< RCC reserved register, 		  		 								Address offset:0x84		*/
	__IO uint32_t CCIPR;		/*!< RCC peripheral independent clock configuration register, 		  		Address offset:0x88		*/
	uint32_t RESERVED7;			/*!< RCC reserved register, 		  		 								Address offset:0x8C		*/
	__IO uint32_t BDCR;			/*!< RCC Backup domain control register, 		  		 					Address offset:0x90		*/
	__IO uint32_t CSR;			/*!< RCC control and status register, 		  		 						Address offset:0x94		*/
	__IO uint32_t CRRCR;		/*!< RCC clock recovery RC register, 		  		 						Address offset:0x98		*/
	__IO uint32_t CCIPR2;		/*!< RCC peripherals independent clock configuration register, 		  		Address offset:0x9C		*/

}RCC_TypeDef;


typedef struct{
	__IO uint32_t IMR1;		/*!< EXTI interrupt mask register 1,            Address offset:0x00 */
	__IO uint32_t EMR1;		/*!< EXTI event mask register 1,	            Address offset:0x04 */
	__IO uint32_t RTSR1;	/*!< EXTI rising trigger selection register 1,  Address offset:0x08 */
	__IO uint32_t FTSR1;	/*!< EXTI falling trigger selection register 1, Address offset:0x0C */
	__IO uint32_t SWIER1;	/*!< EXTI software interrupt event register 1,  Address offset:0x10 */
	__IO uint32_t PR1;		/*!< EXTI pending register 1,                   Address offset:0x14 */
	uint32_t RESERVED[2];	/*!< EXTI reserved register,                    Address offset:0x18-0x1C */
	__IO uint32_t IMR2;		/*!< EXTI interrupt mask register 2,            Address offset:0x20 */
	__IO uint32_t EMR2;		/*!< EXTI event mask register 2,                Address offset:0x24 */
	__IO uint32_t RTSR2;	/*!< EXTI rising trigger selection register 2,  Address offset:0x28 */
	__IO uint32_t FTSR2;	/*!< EXTI falling trigger selection register 2, Address offset:0x2C */
	__IO uint32_t SWIER2;	/*!< EXTI software interrupt event register 2,  Address offset:0x30 */
	__IO uint32_t PR2;		/*!< EXTI pending register 2,                   Address offset:0x34 */

}EXTI_TypeDef;



typedef struct{
	__IO uint32_t MEMRMP;    /*!< SYSCFG memory remap register,                       Address offset:0x00 */
	__IO uint32_t CFGR1;     /*!< SYSCFG configuration register 1,                    Address offset:0x04 */
	__IO uint32_t EXTICR[4]; /*!< SYSCFG external interrupt configuration register 1, Address offset:0x08-0x14*/
	__IO uint32_t SCSR;      /*!< SYSCFG control and status register,                 Address offset:0x18 */
	__IO uint32_t CFGR2;     /*!< SYSCFG configuration register 2,                    Address offset:0x1C */
	__IO uint32_t SWPR;      /*!< SYSCFG SRAM2 write protection register 0-31,        Address offset:0x20 */
	__IO uint32_t SKR;       /*!< SYSCFG SRAM2 key register,                          Address offset:0x24 */
	__IO uint32_t SWPR2;     /*!< SYSCFG SRAM2 write protection register 32-63,       Address offset:0x28 */
}SYSCFG_TypeDef;



typedef struct{
	__IO uint32_t CR1;    /*<! SPI control register 1      Address offset:0x00 */
	__IO uint32_t CR2;    /*<! SPI control register 2      Address offset:0x04 */
	__IO uint32_t SR;     /*<! SPI status register         Address offset:0x08 */
	__IO uint32_t DR;     /*<! SPI data register           Address offset:0x0C */
	__IO uint32_t CRCPR;  /*<! SPI CRC polynomial register Address offset:0x10 */
	__IO uint32_t RXCRCR; /*<! SPI Rx CRC register         Address offset:0x14 */
	__IO uint32_t TXCRCR; /*<! SPI Tx CRC register         Address offset:0x18 */
}SPI_Typedef;

/*************************     peripheral declarations     **********************************/

#define GPIOA 					((GPIO_TypeDef *) GPIOA_BASEADDR)
#define GPIOB 					((GPIO_TypeDef *) GPIOB_BASEADDR)
#define GPIOC 					((GPIO_TypeDef *) GPIOC_BASEADDR)
#define GPIOD 					((GPIO_TypeDef *) GPIOD_BASEADDR)
#define GPIOE 					((GPIO_TypeDef *) GPIOE_BASEADDR)
#define GPIOF 					((GPIO_TypeDef *) GPIOF_BASEADDR)
#define GPIOG 					((GPIO_TypeDef *) GPIOG_BASEADDR)
#define GPIOH 					((GPIO_TypeDef *) GPIOH_BASEADDR)



#define RCC 					((RCC_TypeDef *) RCC_BASEADDR)

#define EXTI					((EXTI_TypeDef *) EXTI_BASEADDR)

#define SYSCFG 					((SYSCFG_TypeDef *) SYSCFG_BASEADDR)

#define SPI1 					((SPI_Typedef *) SPI1_BASEADDR)
#define SPI2					((SPI_Typedef *) SPI2_BASEADDR)
#define SPI3					((SPI_Typedef *) SPI3_BASEADDR)


/*************************     peripheral clock enable      **********************************/
/*
 * clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()			(RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB2ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB2ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB2ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB2ENR |= (1 << 7))
#define OTGFS_PCLK_EN()			(RCC->AHB2ENR |= (1 << 12))
#define ADC_PCLK_EN()			(RCC->AHB2ENR |= (1 << 13))
#define DCMI_PCLK_EN()			(RCC->AHB2ENR |= (1 << 14))
#define AES_PCLK_EN()			(RCC->AHB2ENR |= (1 << 16))
#define HASH_PCLK_EN()			(RCC->AHB2ENR |= (1 << 17))
#define RNG_PCLK_EN()			(RCC->AHB2ENR |= (1 << 18))

/*
 * clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR1 |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR1 |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR1 |= (1 << 23))


#define I2C4_PCLK_EN()			(RCC->APB1ENR2 |= (1 << 1))


/*
 * clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR  |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR1 |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR1 |= (1 << 15))

/*
 * clock enable macros for USARTx peripherals
 */
#define USART2_PCLK_EN()		(RCC->APB1ENR1 |= (1 << 17))
#define USART3_PCLK_EN()		(RCC->APB1ENR1 |= (1 << 18))
#define UART4_PCLK_EN()			(RCC->APB1ENR1 |= (1 << 19))
#define UART5_PCLK_EN()			(RCC->APB1ENR1 |= (1 << 20))

#define LPUART1_PCLK_EN()		(RCC->APB1ENR2 |= (1 << 0))

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))


/*
 * clock enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 0))


/*************************     peripheral clock disable     **********************************/
/*
 * clock Disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 7))
#define OTGFS_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 12))
#define ADC_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 13))
#define DCMI_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 14))
#define AES_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 16))
#define HASH_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 17))
#define RNG_PCLK_DI()			(RCC->AHB2ENR &= ~(1 << 18))

/*
 * clock Disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()			(RCC->APB1ENR1 &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR1 &= ~(1 << 22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR1 &= ~(1 << 23))


#define I2C4_PCLK_DI()			(RCC->APB1ENR2 &= ~(1 << 1))


/*
 * clock Disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR  &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR1 &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR1 &= ~(1 << 15))

/*
 * clock Disable macros for USARTx peripherals
 */
#define USART2_PCLK_DI()		(RCC->APB1ENR1 &= ~(1 << 17))
#define USART3_PCLK_DI()		(RCC->APB1ENR1 &= ~(1 << 18))
#define UART4_PCLK_DI()			(RCC->APB1ENR1 &= ~(1 << 19))
#define UART5_PCLK_DI()			(RCC->APB1ENR1 &= ~(1 << 20))

#define LPUART1_PCLK_DI()		(RCC->APB1ENR2 &= ~(1 << 0))

#define USART1_PCLK_DI()		(RCC->APB2ENR  &= ~(1 << 14))


/*
 * clock Disable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR  &= ~(1 << 0))


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()		do{(RCC->AHB2RSTR |= (1 << 0)); (RCC->AHB2RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB2RSTR |= (1 << 1)); (RCC->AHB2RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB2RSTR |= (1 << 2)); (RCC->AHB2RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB2RSTR |= (1 << 3)); (RCC->AHB2RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB2RSTR |= (1 << 4)); (RCC->AHB2RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB2RSTR |= (1 << 5)); (RCC->AHB2RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB2RSTR |= (1 << 6)); (RCC->AHB2RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB2RSTR |= (1 << 7)); (RCC->AHB2RSTR &= ~(1 << 7));}while(0)


#define SPI1_REG_RESET()        do{(RCC->APB2RSTR  |= (1 << 12));(RCC->APB2RSTR  &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()        do{(RCC->APB1RSTR1 |= (1 << 14));(RCC->APB1RSTR1 &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()        do{(RCC->APB1RSTR1 |= (1 << 15));(RCC->APB1RSTR1 &= ~(1 << 15));}while(0)


/*
 * Macro the converts GPIOx base address to Port code
 */
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)?0:\
                                     (x == GPIOB)?1:\
                                     (x == GPIOC)?2:\
                                     (x == GPIOD)?3:\
									 (x == GPIOE)?4:\
									 (x == GPIOF)?5:\
									 (x == GPIOG)?6:\
									 (x == GPIOH)?7:0 )



/* generic macros that will also be used by different drivers */
typedef enum
{
	RESET = 0,
	SET
}Pin_State_t;

typedef enum
{
	DISABLE = 0,
	ENABLE
}EnDiType_t;


#include "stm32l47xx_gpio_driver.h"
#include "stm32l47xx_spi_driver.h"

#endif /* INC_STM32L47XX_H_ */
