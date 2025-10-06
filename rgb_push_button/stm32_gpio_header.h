/*
 * stm32_gpio_header.h
 *
 *  Created on: Oct 6, 2025
 *      Author: sivareddy
 */

#ifndef SRC_STM32_GPIO_HEADER_H_
#define SRC_STM32_GPIO_HEADER_H_


#ifndef STM32L7_GPIO_DRIVER_H
#define STM32L7_GPIO_DRIVER_H

#include <stdint.h>
#include "stm32l7_rcc_gpio.h"

/* GPIO Modes */
#define GPIO_MODE_INPUT     0x0U
#define GPIO_MODE_OUTPUT    0x1U
#define GPIO_MODE_ALT       0x2U
#define GPIO_MODE_ANALOG    0x3U

/* Output types */
#define GPIO_OTYPE_PP       0x0U
#define GPIO_OTYPE_OD       0x1U

/* Speed */
#define GPIO_SPEED_LOW      0x0U
#define GPIO_SPEED_MEDIUM   0x1U
#define GPIO_SPEED_HIGH     0x2U
#define GPIO_SPEED_VERYHIGH 0x3U

/* Pull-up / Pull-down */
#define GPIO_NO_PUPD        0x0U
#define GPIO_PULLUP         0x1U
#define GPIO_PULLDOWN       0x2U

/* GPIO Helper Macros */
#define GPIO_PIN(n)         (1U << (n))
#define GPIO_MODER_MASK(n)  (0x3U << ((n)*2))
#define GPIO_OTYPER_MASK(n) (1U << (n))
#define GPIO_PUPDR_MASK(n)  (0x3U << ((n)*2))


/**
 * @brief  Enable GPIO clock for a given GPIO port
 * @param  GPIOx: Pointer to GPIO peripheral (GPIOA, GPIOB, etc.)
 */
void GPIO_EnableClock(GPIO_RegDef_t *GPIOx);

/**
 * @brief  Initialize GPIO pin
 * @param  GPIOx: Pointer to GPIO peripheral
 * @param  pin: Pin number (0-15)
 * @param  mode: GPIO_MODE_INPUT, OUTPUT, ALT, ANALOG
 * @param  otype: GPIO_OTYPE_PP or OD
 * @param  speed: GPIO_SPEED_LOW, MEDIUM, HIGH, VERYHIGH
 * @param  pupd: GPIO_NO_PUPD, PULLUP, PULLDOWN
 */
void GPIO_Init(GPIO_RegDef_t *GPIOx, uint8_t pin, uint8_t mode, uint8_t otype, uint8_t speed, uint8_t pupd);

/**
 * @brief  Write value to a GPIO pin
 * @param  GPIOx: GPIO peripheral
 * @param  pin: Pin number
 * @param  value: 0=Low, 1=High
 */
void GPIO_WritePin(GPIO_RegDef_t *GPIOx, uint8_t pin, uint8_t value);

/**
 * @brief  Toggle GPIO pin
 * @param  GPIOx: GPIO peripheral
 * @param  pin: Pin number
 */
void GPIO_TogglePin(GPIO_RegDef_t *GPIOx, uint8_t pin);

/**
 * @brief  Read GPIO pin
 * @param  GPIOx: GPIO peripheral
 * @param  pin: Pin number
 * @return Pin state (0 or 1)
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *GPIOx, uint8_t pin);

/**
 * @brief  Reset all pins of a GPIO port
 * @param  GPIOx: GPIO peripheral
 */
void GPIO_ResetPort(GPIO_RegDef_t *GPIOx);

#endif /* STM32L7_GPIO_DRIVER_H */

#endif /* SRC_STM32_GPIO_HEADER_H_ */
