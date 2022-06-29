/*
 * 001_LedToggle.c
 *
 *  Created on: Jun 26, 2022
 *      Author: PhatTS
 */

#include "stm32f411xx.h"
#include "stm32f411xx_gpio_drivers.h"

#include <stdint.h>

void delay(uint8_t seconds){
    for (uint32_t i = 0; i < seconds*500000; i++)
    {
        /* Do nothing */
    }
}

int main(void)
{
    GPIO_Handle_t GPIOLed;
    /* Configure GPIO D peripheral */
    GPIOLed.pGPIOx = GPIOD;
    /* Select pin 15 of port D GPIO */
    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    /* Configure  output mode for Port D*/
    GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    /* Configure medium speed mode for Port D*/
    GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
    /* Configure output type for Port D as push-pull */
    GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

    /* Enable clock port D GPIO peripheral */
    GPIO_PeriCLockControl(GPIOD, ENABLE);

    /* Initialize GPIO D peripheral */
    GPIO_Init(&GPIOLed);

    while(1){
    GPIO_TogglePin(GPIOLed.pGPIOx, GPIO_PIN_NO_12);
    delay(1);
    }
}
