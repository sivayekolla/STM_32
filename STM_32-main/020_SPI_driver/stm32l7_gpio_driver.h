#ifndef STM32L7_GPIO_DRIVER_H
#define STM32L7_GPIO_DRIVER_H

#include "stm32l476xx.h"

#define GPIO_MODE_INPUT        0x0U
#define GPIO_MODE_OUTPUT       0x1U
#define GPIO_MODE_ALTFN        0x2U
#define GPIO_MODE_ANALOG       0x3U

#define GPIO_OP_TYPE_PP        0x0U
#define GPIO_OP_TYPE_OD        0x1U

#define GPIO_SPEED_LOW         0x0U
#define GPIO_SPEED_MEDIUM      0x1U
#define GPIO_SPEED_FAST        0x2U
#define GPIO_SPEED_HIGH        0x3U
#define GPIO_NO_PUPD           0x0U
#define GPIO_PIN_PU            0x1U
#define GPIO_PIN_PD            0x2U

#define GPIO_PIN_0             0
#define GPIO_PIN_1             1
#define GPIO_PIN_2             2
#define GPIO_PIN_3             3
#define GPIO_PIN_4             4
#define GPIO_PIN_5             5
#define GPIO_PIN_6             6
#define GPIO_PIN_7             7
#define GPIO_PIN_8             8
#define GPIO_PIN_9             9
#define GPIO_PIN_10            10
#define GPIO_PIN_11            11
#define GPIO_PIN_12            12
#define GPIO_PIN_13            13
#define GPIO_PIN_14            14
#define GPIO_PIN_15            15

#define GPIO_INT_FALLING       0x1U
#define GPIO_INT_RISING        0x2U
#define GPIO_INT_BOTH          0x3U

typedef struct
{
    uint32_t PinNumber;
    uint32_t Mode;
    uint32_t Speed;
    uint32_t PuPd;
    uint32_t OType;
    uint32_t AltFunc;
} GPIO_PinConfig_t;

typedef struct
{
    GPIO_TypeDef *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);

uint8_t GPIO_ReadPin(GPIO_TypeDef *pGPIOx, uint16_t PinNumber);
void GPIO_WritePin(GPIO_TypeDef *pGPIOx, uint16_t PinNumber, uint8_t Value);
void GPIO_TogglePin(GPIO_TypeDef *pGPIOx, uint16_t PinNumber);

void GPIO_ConfigInterrupt(uint16_t PinNumber, uint8_t EdgeSel);
void GPIO_EnableInterrupt(uint16_t PinNumber, uint8_t IRQn, uint8_t priority);
void GPIO_ClearInterrupt(uint16_t PinNumber);

#endif 
