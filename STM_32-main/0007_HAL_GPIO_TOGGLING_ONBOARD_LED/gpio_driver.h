#ifndef __GPIO_DRIVER_H
#define __GPIO_DRIVER_H

#include "stm32l4xx_hal.h"

#define LED_GPIO_PORT   GPIOA
#define LED_PIN         GPIO_PIN_5

void GPIO_Init_All(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

#endif 
