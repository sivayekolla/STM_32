#include "main.h"
#include "gpio_driver.h"

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    GPIO_Init_All();

    while (1)
    {
        LED_Toggle();
        HAL_Delay(500);
    }
}
