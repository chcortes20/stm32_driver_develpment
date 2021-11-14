/*
 * stm32l47xx.h
 *
 *  Created on: Nov 7, 2021
 *      Author: Christian Cortes
 */

#ifndef INC_STM32L47XX_H_
#define INC_STM32L47XX_H_


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

#define __IO volatile

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








/*************************     peripheral declarations     **********************************/

#define GPIOA 					((GPIO_TypeDef *) GPIOA_BASEADDR)
#define GPIOB 					((GPIO_TypeDef *) GPIOB_BASEADDR)
#define GPIOC 					((GPIO_TypeDef *) GPIOC_BASEADDR)
#define GPIOD 					((GPIO_TypeDef *) GPIOD_BASEADDR)
#define GPIOE 					((GPIO_TypeDef *) GPIOE_BASEADDR)
#define GPIOF 					((GPIO_TypeDef *) GPIOF_BASEADDR)
#define GPIOG 					((GPIO_TypeDef *) GPIOG_BASEADDR)
#define GPIOH 					((GPIO_TypeDef *) GPIOH_BASEADDR)
#define GPIOI 					((GPIO_TypeDef *) GPIOI_BASEADDR)















#endif /* INC_STM32L47XX_H_ */
