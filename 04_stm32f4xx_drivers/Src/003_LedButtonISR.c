/*
 * 003_LedButtonISR.c
 *
 *  Created on: Jul 2, 2022
 *      Author: PhatTS
 */

#include "stm32f411xx.h"
#include "stm32f411xx_gpio_drivers.h"

#define HIGH        ENABLE
#define BTN_PRESS   HIGH

int main(void) {
    GPIO_Handle_t GPIOLed, GPIOButton;
    /* Configure for Led */
    GPIOLed.pGPIOx = GPIOD;

    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;

    GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;

    GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

    GPIOLed.GPIO_PinConfig.GPIO_PinPuPDControl = GPIO_NO_PUPD;

    GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

    GPIO_PeriCLockControl(GPIOD, ENABLE);

    GPIO_Init(&GPIOLed);

    /* Configure for button */
    GPIOButton.pGPIOx = GPIOA;

    GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;

    GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;

    GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    GPIOButton.GPIO_PinConfig.GPIO_PinPuPDControl = GPIO_NO_PUPD;

    GPIO_PeriCLockControl(GPIOA, ENABLE);

    GPIO_Init(&GPIOButton);

    GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_RESET);
    /* IRQ configuration */
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI13);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

    while(1);

}

void EXTI9_5_IRQHandler(void){
	/* IRQ Handling */
	GPIO_IRQHandling(GPIO_PIN_NO_0); //clear the pending event from exti line
	/* Led toggle */
	GPIO_TogglePin(GPIOD, GPIO_PIN_NO_12);
}
