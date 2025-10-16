#include "i2c_driver.h"

volatile uint8_t i2c_busy = 0;
static uint8_t i2c_tx_buf[32];
static volatile uint8_t i2c_tx_len = 0;
static volatile uint8_t i2c_tx_idx = 0;

//-------------------------------------------
// Initialize I2C1 Peripheral (PB8=SCL, PB9=SDA)
//-------------------------------------------
void I2C1_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;  // Enable GPIOB clock
    GPIOB->MODER &= ~((3<<(8*2)) | (3<<(9*2))); 
    GPIOB->MODER |= ((2<<(8*2)) | (2<<(9*2)));  // Alternate function
    GPIOB->OTYPER |= (1<<8) | (1<<9);           // Open drain
    GPIOB->OSPEEDR |= (3<<(8*2)) | (3<<(9*2));  // High speed
    GPIOB->AFR[1] |= (4<<((8-8)*4)) | (4<<((9-8)*4));  // AF4 for I2C1

    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;       // Enable I2C1 clock
    I2C1->CR1 &= ~I2C_CR1_PE;                   // Disable I2C before config

    // Configure timing for 100kHz @16MHz PCLK
    I2C1->TIMINGR = 0x00303D5B;

    I2C1->OAR1 &= ~I2C_OAR1_OA1EN;  // No slave address (master mode)
    I2C1->CR1 |= I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ERRIE; // Enable interrupts
    I2C1->CR1 |= I2C_CR1_PE;        // Enable I2C

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

//-------------------------------------------
// Start Non-blocking Transmit
//-------------------------------------------
void I2C1_Master_Transmit_IT(uint8_t *data, uint8_t len)
{
    while (i2c_busy);  // wait if busy
    i2c_busy = 1;
    i2c_tx_len = len;
    i2c_tx_idx = 0;

    for (uint8_t i=0; i<len; i++)
        i2c_tx_buf[i] = data[i];

    // Set transfer parameters
    I2C1->CR2 = (I2C_ADDR_ESP32 << 1) | (len << 16) | I2C_CR2_AUTOEND | I2C_CR2_START;
    I2C1->CR1 |= I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_ERRIE;
}

//-------------------------------------------
// Event Interrupt Handler
//-------------------------------------------
void I2C1_EV_IRQHandler(void)
{
    if (I2C1->ISR & I2C_ISR_TXIS)  // TX ready
    {
        I2C1->TXDR = i2c_tx_buf[i2c_tx_idx++];
    }

    if (I2C1->ISR & I2C_ISR_STOPF)  // STOP detected
    {
        I2C1->ICR |= I2C_ICR_STOPCF; // clear flag
        i2c_busy = 0;
    }
}

//-------------------------------------------
// Error Interrupt Handler
//-------------------------------------------
void I2C1_ER_IRQHandler(void)
{
    if (I2C1->ISR & I2C_ISR_NACKF)
        I2C1->ICR |= I2C_ICR_NACKCF;

    if (I2C1->ISR & I2C_ISR_BERR)
        I2C1->ICR |= I2C_ICR_BERRCF;

    if (I2C1->ISR & I2C_ISR_ARLO)
        I2C1->ICR |= I2C_ICR_ARLOCF;

    i2c_busy = 0;
}
