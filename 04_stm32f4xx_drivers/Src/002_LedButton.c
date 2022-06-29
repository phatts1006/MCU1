/*
 * 002_LedButton.c
 *
 *  Created on: Jun 26, 2022
 *      Author: PhatTS
 */

#include "stm32f411xx.h"
#include "stm32f411xx_gpio_drivers.h"

#define HIGH        ENABLE
#define BTN_PRESS   HIGH

void delay(uint8_t seconds){
    for (uint32_t i = 0; i < seconds*200000; i++)
    {
        /* Do nothing */
    }
}

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

    GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;

    GPIOButton.GPIO_PinConfig.GPIO_PinPuPDControl = GPIO_NO_PUPD;

    GPIO_PeriCLockControl(GPIOA, ENABLE);

    GPIO_Init(&GPIOButton);

    while(1){
        if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESS){
            delay(1);
            GPIO_TogglePin(GPIOD, GPIO_PIN_NO_12);
        }
    }
}

