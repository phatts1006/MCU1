/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define RCC_AHB1ENR			*((uint32_t *) 0x40023830)

#define GPIOD_MODER			*((uint32_t *) 0x40020C00)

#define GPIOD_OTYPER		*((uint32_t *) 0x40020C04)

#define GPIOD_ODR			*((uint32_t *) 0x40020C14)



int main(void)
{
	/*Enable clock of port D*/
	RCC_AHB1ENR |= (1UL << 3);

	/*Clear mode bits */
	GPIOD_MODER &= ~(3UL << 24);

	/*Set mode port D as output*/
	GPIOD_MODER |= (1UL << 24);

	/*Select push-pull output*/
	GPIOD_OTYPER &= ~(1UL << 12);

	/*Enable pin 12 of port D*/
	GPIOD_ODR |= (1UL << 12);
}
