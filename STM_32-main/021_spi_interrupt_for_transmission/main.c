#include "stm32l476xx.h"

void SPI_Init_IRQ(SPI_TypeDef *SPIx);
void SPI_SendHello_IRQ(SPI_TypeDef *SPIx);
void SPI1_IRQHandler(void);

#define BTN_PORT GPIOC
#define BTN_PIN  13

void delay(volatile uint32_t count){ while(count--); }

int main(void)
{
    RCC->AHB2ENR |= (1<<2);

    BTN_PORT->MODER &= ~(3<<(2*BTN_PIN));
    BTN_PORT->PUPDR &= ~(3<<(2*BTN_PIN));
    BTN_PORT->PUPDR |= (1<<(2*BTN_PIN));

    SPI_Init_IRQ(SPI1);

    while(1)
    {
        if((BTN_PORT->IDR & (1<<BTN_PIN)) == 0)
        {

            while((BTN_PORT->IDR & (1<<BTN_PIN)) == 0);
        }
    }
}
