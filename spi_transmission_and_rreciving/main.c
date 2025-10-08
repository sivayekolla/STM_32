#include "stm32l7_gpio_driver.h"

int main(void)
{
    GPIO_Handle_t led;
    led.pGPIOx = GPIOA;
    led.GPIO_PinConfig.PinNumber = GPIO_PIN_5;
    led.GPIO_PinConfig.Mode = GPIO_MODE_OUTPUT;
    led.GPIO_PinConfig.Speed = GPIO_SPEED_LOW;
    led.GPIO_PinConfig.PuPd = GPIO_NO_PUPD;
    led.GPIO_PinConfig.OType = GPIO_OP_TYPE_PP;
    GPIO_Init(&led);

    GPIO_Handle_t btn;
    btn.pGPIOx = GPIOC;
    btn.GPIO_PinConfig.PinNumber = GPIO_PIN_13;
    btn.GPIO_PinConfig.Mode = GPIO_MODE_INPUT;
    btn.GPIO_PinConfig.PuPd = GPIO_NO_PUPD;
    GPIO_Init(&btn);

    GPIO_ConfigInterrupt(GPIO_PIN_13, GPIO_INT_FALLING);
    GPIO_EnableInterrupt(GPIO_PIN_13, EXTI15_10_IRQn, 1);

    while (1)
    {
        __WFI();
    }
}

void EXTI15_10_IRQHandler(void)
{
    GPIO_ClearInterrupt(GPIO_PIN_13);
    GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
