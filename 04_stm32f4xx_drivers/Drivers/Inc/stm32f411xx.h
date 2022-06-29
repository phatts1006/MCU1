/*
 * stm32f411xx.h
 *
 *  Created on: Jun 21, 2022
 *      Author: PhatTS
 *      Email : phathenry113@gmail.com
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0		((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1		((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2		((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3		((volatile uint32_t *)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0		((volatile uint32_t *)0XE000E180)
#define NVIC_ICER1		((volatile uint32_t *)0XE000E184)
#define NVIC_ICER2		((volatile uint32_t *)0XE000E188)
#define NVIC_ICER3		((volatile uint32_t *)0XE000E18C)

/*
 * ARM Cortex Mx Processor NVIC priority register Address
 */
#define NVIC_PR_BASE_ADDR ((volatile uint32_t *)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED	4

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U //112KB
#define SRAM2_BASEADDR					0x2001C000U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM		 					SRAM1_BASEADDR

/*
 * APBx and AHBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR					0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x50000000U

/*
 * Base addresses of peripherals which are handing on AHB1 bus
 * TODO: Complete for all other peipherals
 */

#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00)

#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses of peripherals which are handing on APB1 bus
 * TODO: Complete for all other peipherals
 */

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)

/*
 * Base addresses of peripherals which are handing on APB2 bus
 * TODO: Complete for all other peipherals
 */

#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)

#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR					(APB2PERIPH_BASEADDR + 0x5000)

#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)

/**************************** Peripheral register definition structures *************************/

/*
 * Note : Registers of a peripheral are spetific to MCU
 * e.g : Number of registers of SPI peripheral of STM32F4X family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your device Reference Manual
 */

typedef struct{
	volatile uint32_t MODER;	/* GPIO port mode register 					Address offset : 0x00 */
	volatile uint32_t OTYPER;	/* GPIO port output type register 			Address offset : 0x04 */
	volatile uint32_t OSPEEDR;	/* GPIO port output speed register			Address offset : 0x08 */
	volatile uint32_t PUPDR;	/* GPIO port pull-up/pull-down register 	Address offset : 0x0C */
	volatile uint32_t IDR;		/* GPIO port input data register			Address offset : 0x10 */
	volatile uint32_t ODR;		/* GPIO port output data register			Address offset : 0x14 */
	volatile uint32_t BSRR;		/* GPIO port bit set/reset register 		Address offset : 0x18 */
	volatile uint32_t LCKR;		/* GPIO port configuration lock register	Address offset : 0x1C */
	volatile uint32_t AFR[2];	/* GPIO alternate function				Address offset : 0x20-0x24 */
}GPIO_RegDef_t;

typedef struct{
	volatile uint32_t CR;		/* RCC clock control register 				Address offset : 0x00 */
	volatile uint32_t PLLCFGR;  /* RCC PLL configuration register 			Address offset : 0x04 */
	volatile uint32_t CFGR;		/* RCC clock configuration register 		Address offset : 0x08 */
	volatile uint32_t CIR;		/* RCC clock interrupt register 			Address offset : 0x0C */
	volatile uint32_t AHB1RSTR; /* RCC AHB1 peripheral reset register 		Address offset : 0x10 */
	volatile uint32_t AHB2RSTR; /* RCC AHB2 peripheral reset register 		Address offset : 0x14 */
	uint32_t RESERVED1[2];		/* Reserved 2 bytes 						Address offset : 0x18 - 0x1C */
	volatile uint32_t APB1RSTR; /* RCC APB1 peripheral reset register for 	Address offset : 0x20 */
	volatile uint32_t APB2RSTR; /* RCC APB2 peripheral reset register 		Address offset : 0x24 */
	uint32_t RESERVED2[2];		/* Reserved 2 bytes				 			Address offset : 0x28 - 0x2C */
	volatile uint32_t AHB1ENR;	/* RCC AHB1 peripheral clock enable register	Address offset : 0x30 */
	volatile uint32_t AHB2ENR;	/* RCC AHB2 peripheral clock enable register 	Address offset : 0x34 */
	uint32_t RESERVED3[2];		/* Reserved 2 bytes								Address offset : 0x38 - 0x3C */
	volatile uint32_t APB1ENR;	/* RCC APB1 peripheral clock enable register	Address offset : 0x40 */
	volatile uint32_t APB2ENR;	/* RCC APB2 peripheral clock enable register 	Address offset : 0x44 */
	uint32_t RESERVED4[2];		/* Reserved 2 bytes								Address offset : 0x48 - 0x4C */
	volatile uint32_t AHB1LPENR;/* RCC AHB1 peripheral clock enable in low power mode register 	Address offset : 0x50 */
	volatile uint32_t AHB2LPENR;/* RCC AHB2 peripheral clock enable in low power mode register 	Address offset : 0x54 */
	uint32_t RESERVED5[2];		/* Reserved 2 bytes 							Address offset : 0x58 - 0x5C */
	volatile uint32_t APB1LPENR;/* RCC APB1 peripheral clock enable in low power mode register 	Address offset : 0x60 */
	volatile uint32_t APB2LPENR;/* RCC APB2 peripheral clock enable in low power mode register 	Address offset : 0x64 */
	uint32_t RESERVED6[2];		/* Reserved 2 bytes 						Address offset : 0x68 - 0x6C */
	volatile uint32_t BDCR;		/* RCC Backup domain control register 		Address offset : 0x70 */
	volatile uint32_t CSR;		/* RCC clock control & status register 		Address offset : 0x74 */
	uint32_t RESERVED7[2];		/* Reserved 2 bytes 						Address offset : 0x78 - 0x7C */
	volatile uint32_t SSCGR;	/* RCC spread spectrum clock generation register Address offset : 0x80 */
	volatile uint32_t PLLI2SCFGR;/* RCC PLLI2S configuration register 		Address offset : 0x84 */
	uint32_t RESERVED8;			/* RCC PLL configuration register 			Address offset : 0x88 */
	volatile uint32_t DCKCFGR;	/* RCC PLL configuration register 			Address offset : 0x8C */
}RCC_RegDef_t;

typedef struct{
	volatile uint32_t IMR;	/* Interrupt mask register 					Address offset : 0x00 */
	volatile uint32_t EMR;	/* Event mask register			 			Address offset : 0x04 */
	volatile uint32_t RTSR;	/* Rising trigger selection register		Address offset : 0x08 */
	volatile uint32_t FTSR;	/* Falling trigger selection register 		Address offset : 0x0C */
	volatile uint32_t SWIER;/* Software interrupt event register		Address offset : 0x10 */
	volatile uint32_t PR;	/* Pending register							Address offset : 0x14 */	
}EXTI_RegDef_t;

typedef struct{
	volatile uint32_t MEMRMP;	/* Memory remap register 						Address offset : 0x00 */
	volatile uint32_t PMC;		/* Peripheral mode configuration register		Address offset : 0x04 */
	volatile uint32_t EXTICR[4];/* External interrupt configuration register	Address offset : 0x08 - 0x14 */
	volatile uint32_t CMPCR;	/* Compensation cell control register 			Address offset : 0x20 */	
}SYSCFG_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA		((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOH		((GPIO_RegDef_t *) GPIOH_BASEADDR)

#define RCC			((RCC_RegDef_t *) RCC_BASEADDR)
#define EXTI		((EXTI_RegDef_t *) EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

/*
 * Clock Enable/Disable Macros for GPIOs peripherals
 */

#define GPIOA_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 7))

#define GPIOA_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DISABLE()		(RCC->AHB1ENR &= ~(1 << 7))

/*
 * Macros to reset GPIO peripherals
 */

#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)

/*
 * Return port code fro given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0:\
									(x == GPIOB) ? 1:\
									(x == GPIOC) ? 2:\
									(x == GPIOD) ? 3:\
									(x == GPIOE) ? 4:\
									(x == GPIOH) ? 7:0)


/*
 * Clock Enable/Disable Macros for SPIs peripherals
 */

#define SPI1_PCLK_ENABLE()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_ENABLE()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_ENABLE()		(RCC->APB2ENR |= (1 << 20))

#define SPI1_PCLK_DISABLE()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DISABLE()		(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DISABLE()		(RCC->APB2ENR &= ~(1 << 20))

/*
 * Clock Enable/Disable Macros for I2Cs peripherals
 */

#define I2C1_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 23))

#define I2C1_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Enable/Disable Macros for USARTs peripherals
 */

#define USART1_PCLK_ENABLE()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_ENABLE()		(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_ENABLE()		(RCC->APB2ENR |= (1 << 5))

#define USART1_PCLK_DISABLE()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DISABLE()		(RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DISABLE()		(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Enable/Disable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_ENABLE()		(RCC->APB2ENR |= (1 << 14))
#define SYSCFG_PCLK_DISABLE()		(RCC->APB2ENR &= ~(1 << 14))

/*
 * Generic macros
 */

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET



#endif /* INC_STM32F411XX_H_ */
