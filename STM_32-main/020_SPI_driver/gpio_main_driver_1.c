#include "stm32l7_gpio_driver.h"

 static void GPIO_ClockEnable(GPIO_TypeDef *pGPIOx)
{
    if (pGPIOx == GPIOA) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    else if (pGPIOx == GPIOB) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    else if (pGPIOx == GPIOC) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    else if (pGPIOx == GPIOD) RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
    else if (pGPIOx == GPIOE) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
    else if (pGPIOx == GPIOH) RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;
    uint32_t pin = pGPIOHandle->GPIO_PinConfig.PinNumber;

    GPIO_ClockEnable(pGPIOHandle->pGPIOx);

    if (pGPIOHandle->GPIO_PinConfig.Mode <= GPIO_MODE_ANALOG)
    {
        temp = pGPIOHandle->pGPIOx->MODER;
        temp &= ~(0x3U << (2 * pin));
        temp |= (pGPIOHandle->GPIO_PinConfig.Mode << (2 * pin));
        pGPIOHandle->pGPIOx->MODER = temp;
    }

    temp = pGPIOHandle->pGPIOx->OTYPER;
    temp &= ~(1U << pin);
    temp |= (pGPIOHandle->GPIO_PinConfig.OType << pin);
    pGPIOHandle->pGPIOx->OTYPER = temp;

    temp = pGPIOHandle->pGPIOx->OSPEEDR;
    temp &= ~(0x3U << (2 * pin));
    temp |= (pGPIOHandle->GPIO_PinConfig.Speed << (2 * pin));
    pGPIOHandle->pGPIOx->OSPEEDR = temp;

    temp = pGPIOHandle->pGPIOx->PUPDR;
    temp &= ~(0x3U << (2 * pin));
    temp |= (pGPIOHandle->GPIO_PinConfig.PuPd << (2 * pin));
    pGPIOHandle->pGPIOx->PUPDR = temp;

    if (pGPIOHandle->GPIO_PinConfig.Mode == GPIO_MODE_ALTFN)
    {
        uint32_t afr_index = pin / 8;
        uint32_t afr_pos = (pin % 8) * 4;
        pGPIOHandle->pGPIOx->AFR[afr_index] &= ~(0xFU << afr_pos);
        pGPIOHandle->pGPIOx->AFR[afr_index] |= (pGPIOHandle->GPIO_PinConfig.AltFunc << afr_pos);
    }
}

void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{
    if (pGPIOx == GPIOA) RCC->AHB2RSTR |= RCC_AHB2RSTR_GPIOARST, RCC->AHB2RSTR &= ~RCC_AHB2RSTR_GPIOARST;
    else if (pGPIOx == GPIOB) RCC->AHB2RSTR |= RCC_AHB2RSTR_GPIOBRST, RCC->AHB2RSTR &= ~RCC_AHB2RSTR_GPIOBRST;
    else if (pGPIOx == GPIOC) RCC->AHB2RSTR |= RCC_AHB2RSTR_GPIOCRST, RCC->AHB2RSTR &= ~RCC_AHB2RSTR_GPIOCRST;
}

uint8_t GPIO_ReadPin(GPIO_TypeDef *pGPIOx, uint16_t PinNumber)
{
    return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
}

void GPIO_WritePin(GPIO_TypeDef *pGPIOx, uint16_t PinNumber, uint8_t Value)
{
    if (Value)
        pGPIOx->BSRR = (1U << PinNumber);
    else
        pGPIOx->BSRR = (1U << (PinNumber + 16U));
}

void GPIO_TogglePin(GPIO_TypeDef *pGPIOx, uint16_t PinNumber)
{
    pGPIOx->ODR ^= (1U << PinNumber);
}

void GPIO_ConfigInterrupt(uint16_t PinNumber, uint8_t EdgeSel)
{
    if (EdgeSel & GPIO_INT_FALLING)
        EXTI->FTSR1 |= (1U << PinNumber);
    else
        EXTI->FTSR1 &= ~(1U << PinNumber);

    if (EdgeSel & GPIO_INT_RISING)
        EXTI->RTSR1 |= (1U << PinNumber);
    else
        EXTI->RTSR1 &= ~(1U << PinNumber);

    EXTI->IMR1 |= (1U << PinNumber);
}

void GPIO_EnableInterrupt(uint16_t PinNumber, uint8_t IRQn, uint8_t priority)
{
    uint8_t portcode = 0;
    if (GPIOC == GPIOC) portcode = 0x2U; 
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[PinNumber / 4] &= ~(0xFU << (4 * (PinNumber % 4)));
    SYSCFG->EXTICR[PinNumber / 4] |= (portcode << (4 * (PinNumber % 4)));

    NVIC_SetPriority(IRQn, priority);
    NVIC_EnableIRQ(IRQn);
}

void GPIO_ClearInterrupt(uint16_t PinNumber)
{
    if (EXTI->PR1 & (1U << PinNumber))
        EXTI->PR1 = (1U << PinNumber);
}
