#include "stm32l7_rcc_gpio.h"
#include <stddef.h>

#define RED_PORT    GPIOA
#define RED_PIN     0
#define GREEN_PORT  GPIOA
#define GREEN_PIN   1
#define BLUE_PORT   GPIOA
#define BLUE_PIN    2

#define BUTTON_PORT GPIOC
#define BUTTON_PIN  13


void delay(volatile uint32_t count)
{
    while(count--);
}

int main(void)
{
    GPIOA_CLK_EN();
    GPIOC_CLK_EN();

    GPIOA->MODER &= ~(GPIO_MODER_MASK(RED_PIN) | GPIO_MODER_MASK(GREEN_PIN) | GPIO_MODER_MASK(BLUE_PIN));
    GPIOA->MODER |=  (GPIO_MODE_OUTPUT << (RED_PIN*2)) |
                     (GPIO_MODE_OUTPUT << (GREEN_PIN*2)) |
                     (GPIO_MODE_OUTPUT << (BLUE_PIN*2));

    GPIOA->OTYPER &= ~(GPIO_OTYPER_MASK(RED_PIN) | GPIO_OTYPER_MASK(GREEN_PIN) | GPIO_OTYPER_MASK(BLUE_PIN));
    GPIOA->OSPEEDR |= (0x3 << (RED_PIN*2)) | (0x3 << (GREEN_PIN*2)) | (0x3 << (BLUE_PIN*2));
    GPIOA->PUPDR &= ~(GPIO_PUPDR_MASK(RED_PIN) | GPIO_PUPDR_MASK(GREEN_PIN) | GPIO_PUPDR_MASK(BLUE_PIN));

    GPIOC->MODER &= ~(GPIO_MODER_MASK(BUTTON_PIN));
    GPIOC->PUPDR &= ~GPIO_PUPDR_MASK(BUTTON_PIN);
    GPIOC->PUPDR |= (GPIO_PULLUP << (BUTTON_PIN*2));

    uint8_t led_state = 0;
    uint8_t last_button = 1;


    while(1)
    {
        uint8_t button_now = GPIO_READ_PIN(BUTTON_PORT, BUTTON_PIN);

        if(last_button == 1 && button_now == 0)
        {
            delay(2000);

            led_state = (led_state + 1) % 4;

            GPIO_WRITE_PIN(RED_PORT, RED_PIN, 0);
            GPIO_WRITE_PIN(GREEN_PORT, GREEN_PIN, 0);
            GPIO_WRITE_PIN(BLUE_PORT, BLUE_PIN, 0);

            switch(led_state)
            {
                case 0: GPIO_WRITE_PIN(RED_PORT, RED_PIN, 1); break;
                case 1: GPIO_WRITE_PIN(GREEN_PORT, GREEN_PIN, 1); break;
                case 2: GPIO_WRITE_PIN(BLUE_PORT, BLUE_PIN, 1); break;
                case 3: break;
                default:
                	break;
            }
        }

        last_button = button_now;
    }
}

