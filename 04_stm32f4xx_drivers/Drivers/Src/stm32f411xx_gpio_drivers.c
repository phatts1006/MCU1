/*
 * stm32f411xx_gpio_drivers.c
 *
 *  Created on: Jun 22, 2022
 *      Author: PhatTS
 *      Email: phathenry113@gmail.com
 */

#include "stm32f411xx_gpio_drivers.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriCLockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_ENABLE();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_ENABLE();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_ENABLE();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_ENABLE();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_ENABLE();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_ENABLE();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DISABLE();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DISABLE();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DISABLE();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DISABLE();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DISABLE();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DISABLE();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes for GPIO peripheral
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t TempValue = 0;
	/* 1. Configure the mode of GPIO pin */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		/* Handle for non interrupt */
		TempValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= TempValue;
	}
	else{
		/* Handle for interrupt */
		/* 1. Handling for Interrrupt mode */
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			/* FTSR mode */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear the responding bit RTSR bit */
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			/* RTSR mode */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear the responding bit FTSR bit */
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			/* Both FTSR and RTSR */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		/* 2. Configure the GPIO port selection in SYSCFG_EXTICR */
		uint8_t TempValue1, TempValue2;
		TempValue1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		TempValue2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_ENABLE();
		SYSCFG->EXTICR[TempValue1] |= (portcode << TempValue2 * 4);
		/* 3. Enable the EXTI interrupt delivery using IMR */

		
		
	}

	TempValue = 0;
	/* 2. Configure the speed */
	TempValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= TempValue;

	TempValue = 0;
	/* 3. Configure the pull-up pull-down setttings */
	TempValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPDControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= TempValue;

	TempValue = 0;
	/* 4. Configure the output type */
	TempValue = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= TempValue;


	TempValue = 0;
	/* 5. Configure the alt funtionality */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode == GPIO_MODE_ALTFN){

		uint32_t TempValue1, TempValue2;
		TempValue1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		TempValue2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[TempValue1] &= ~( 0xF << (4 * TempValue2));
		pGPIOHandle->pGPIOx->AFR[TempValue1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * TempValue2);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function de-initializes/resets for GPIO peripheral
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads data from input GPIO pin
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - Number of GPIO pin
 * @param[in]         -
 *
 * @return            -  Value of GPIO input pin
 *
 * @Note              -  none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t TempValue;
	TempValue = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return TempValue;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads data from input GPIO port
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  Value of GPIO input port
 *
 * @Note              -  none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t TempValue;
	TempValue = (uint16_t)(pGPIOx->IDR);
	return TempValue;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes data to output GPIO pin
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - Number of GPIO pin
 * @param[in]         - Value GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes data to output GPIO port
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - Write a value to port
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_TogglePin
 *
 * @brief             - This function toggle a pin GPIO peripheral
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - Number of GPIO pin
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if (EnorDi == ENABLE){
		if (IRQNumber <= 31){
			/* Program ISER0 register */
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64){
			/* Program ISER1 register */
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){
			/* Program ISER2 register */
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}	
	}
	else {
		if (IRQNumber <= 31){
			/* Program ICER0 register */
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64){
			/* Program ICER1 register */
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){
			/* Program ICER2 register */
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
	
}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQHandling(uint8_t PinNumber){

}
