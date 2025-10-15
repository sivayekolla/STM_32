#include "i2c_driver.h"

void I2C1_Init(void)
{
    // 1. Enable GPIOB and I2C1 clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    // 2. Configure PB8 (SCL), PB9 (SDA) to AF4 (I2C1)
    GPIOB->MODER &= ~((3 << (2 * I2C1_SCL_PIN)) | (3 << (2 * I2C1_SDA_PIN)));
    GPIOB->MODER |=  ((2 << (2 * I2C1_SCL_PIN)) | (2 << (2 * I2C1_SDA_PIN)));  // AF mode
    GPIOB->OTYPER |= ((1 << I2C1_SCL_PIN) | (1 << I2C1_SDA_PIN));              // open-drain
    GPIOB->OSPEEDR |= ((3 << (2 * I2C1_SCL_PIN)) | (3 << (2 * I2C1_SDA_PIN))); // high speed
    GPIOB->PUPDR &= ~((3 << (2 * I2C1_SCL_PIN)) | (3 << (2 * I2C1_SDA_PIN)));  // no pull
    GPIOB->AFR[1] |= (4 << ((I2C1_SCL_PIN - 8) * 4)) | (4 << ((I2C1_SDA_PIN - 8) * 4));

    // 3. Reset I2C1
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;

    // 4. Configure I2C timing for 100kHz @ 16MHz PCLK1
    I2C1->TIMINGR = 0x00707CBB;

    // 5. Enable I2C1 peripheral
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_Start(void)
{
    I2C1->CR2 &= ~I2C_CR2_START;
    while (I2C1->ISR & I2C_ISR_BUSY);
}

void I2C1_Stop(void)
{
    I2C1->CR2 |= I2C_CR2_STOP;
    while (I2C1->CR2 & I2C_CR2_STOP);
}

void I2C1_Write(uint8_t data)
{
    while (!(I2C1->ISR & I2C_ISR_TXIS)); // Wait until TX ready
    I2C1->TXDR = data;
    while (!(I2C1->ISR & I2C_ISR_TC));   // Wait transfer complete
}

void I2C1_SendData(uint8_t slaveAddr, uint8_t *data, uint32_t len)
{
    // Configure slave address and number of bytes
    I2C1->CR2 = (slaveAddr << 1) | (len << 16) | I2C_CR2_AUTOEND | I2C_CR2_START | I2C_CR2_WRITE;

    for (uint32_t i = 0; i < len; i++)
    {
        while (!(I2C1->ISR & I2C_ISR_TXIS)); // wait until TX buffer empty
        I2C1->TXDR = data[i];
    }

    while (!(I2C1->ISR & I2C_ISR_STOPF)); // Wait for stop condition
    I2C1->ICR |= I2C_ICR_STOPCF;          // Clear stop flag
}
