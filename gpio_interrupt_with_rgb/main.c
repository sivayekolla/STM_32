#include "stm32l476xx.h"

#define LED_PORT GPIOA
#define LED_PIN  0

#define BTN_PORT GPIOC
#define BTN_PIN  13

void delay(void)
{
    for (volatile uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
    RCC->AHB2ENR |= (1 << 0);
    RCC->AHB2ENR |= (1 << 2);
    RCC->APB2ENR |= (1 << 0);

    LED_PORT->MODER &= ~(3 << (2 * LED_PIN));
    LED_PORT->MODER |=  (1 << (2 * LED_PIN));
    LED_PORT->OTYPER &= ~(1 << LED_PIN);
    LED_PORT->OSPEEDR |=  (3 << (2 * LED_PIN));
    LED_PORT->PUPDR &= ~(3 << (2 * LED_PIN));

    BTN_PORT->MODER &= ~(3 << (2 * BTN_PIN));
    BTN_PORT->PUPDR &= ~(3 << (2 * BTN_PIN));

    SYSCFG->EXTICR[3] &= ~(0xF << 4);
    SYSCFG->EXTICR[3] |=  (0x2 << 4);

    EXTI->IMR1  |=  (1 << BTN_PIN);
    EXTI->RTSR1 &= ~(1 << BTN_PIN);
    EXTI->FTSR1 |=  (1 << BTN_PIN);

    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 2);

    while (1)
    {
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & (1 << BTN_PIN))
    {
        EXTI->PR1 |= (1 << BTN_PIN);
        LED_PORT->ODR ^= (1 << LED_PIN);
        delay();
    }
}
