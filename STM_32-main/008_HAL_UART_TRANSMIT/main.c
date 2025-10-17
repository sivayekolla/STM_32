#include "main.h"
#include "gpio.h"
#include "usart.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();   // Optional if using default startup
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    while (1)
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            char msg[] = "Button Pressed\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);
            HAL_Delay(500);
        }
    }
}
