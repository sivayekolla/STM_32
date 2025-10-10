#include"stm32l7_rcc_gpio.h"

#define port 	GPIOA
#define pin 	5
void delay(int n)
{
	for(int i=0;i<n*(120000);i++)
	{}
}
int main()
{
	/*GPIOA_CLK_EN();

	delay(3000);
	GPIO_TOGGLE_PIN(port,pin);
	delay(30);*/
	GPIOA_CLK_EN();                     // Enable clock for GPIOA
	GPIOA->MODER &= ~(3 << (5 * 2));    // Clear mode bits for PA5
	GPIOA->MODER |=  (1 << (5 * 2));    // Set PA5 as output

	while (1) {
	    GPIOA->ODR ^= (1 << 5);         // Toggle PA5 (LED)
	    for (volatile int i = 0; i < 500; i++); // Delay
	}

}
