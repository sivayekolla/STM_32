#include "stm32l476xx.h"
#include "i2c_driver.h"
#include <string.h>

void delay(volatile uint32_t d)
{
    while(d--);
}

int main(void)
{
    I2C1_Init();

    uint8_t msg[] = "Hello ESP32 from STM32!";

    while(1)
    {
        I2C1_SendData(SLAVE_ADDR, msg, strlen((char*)msg));
        delay(8000000);
    }
}
