#include "lcd_gpio.h"
#include "ultrasonic_gpio.h"

int main(void)
{
    LCD_Init();          // LCD shows "READY"
    Ultrasonic_Init();   // Initialize ultrasonic GPIO

    while (1)
    {
        uint32_t distance = Ultrasonic_Read();
        LCD_Clear();

        LCD_Print("Dist:");
        char buf[10];
        int i = 0, temp = distance;
        if (temp == 0) buf[i++] = '0';
        while (temp > 0) {
            buf[i++] = (temp % 10) + '0';
            temp /= 10;
        }
        for (int j = i - 1; j >= 0; j--)
            LCD_SendData(buf[j]);
        LCD_Print("cm");

        for (volatile int d = 0; d < 500000; d++);
    }
}
