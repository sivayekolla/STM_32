#include "stm32l476xx.h"

#define TX_BUFFER_SIZE 5
#define RX_BUFFER_SIZE 5

uint8_t txBuffer[TX_BUFFER_SIZE] = {'H','E','L','L','O'};
uint8_t rxBuffer[RX_BUFFER_SIZE] = {0};

volatile int txIndex = 0;
volatile int rxIndex = 0;
volatile int txSize = TX_BUFFER_SIZE;
volatile int rxSize = RX_BUFFER_SIZE;

void SPI_Init_IRQ(SPI_TypeDef *SPIx)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    GPIOA->MODER &= ~((3U << (4*2)) | (3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOA->MODER |=  ((2U << (4*2)) | (2U << (5*2)) | (2U << (6*2)) | (2U << (7*2)));

    GPIOA->AFR[0] &= ~((0xFU << (4*4)) | (0xFU << (5*4)) | (0xFU << (6*4)) | (0xFU << (7*4)));
    GPIOA->AFR[0] |=  ((5U << (4*4)) | (5U << (5*4)) | (5U << (6*4)) | (5U << (7*4)));

    GPIOA->OSPEEDR |= ((3U << (4*2)) | (3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
    GPIOA->OTYPER  &= ~((1U << 4) | (1U << 5) | (1U << 6) | (1U << 7));
    GPIOA->PUPDR   &= ~((3U << (4*2)) | (3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));

    SPIx->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    SPIx->CR2 = SPI_CR2_TXEIE | SPI_CR2_RXNEIE;
    SPIx->CR1 |= SPI_CR1_SPE;

    NVIC_EnableIRQ(SPI1_IRQn);
    NVIC_SetPriority(SPI1_IRQn, 1);
}

void SPI_StartTX(SPI_TypeDef *SPIx, uint8_t *buffer, uint32_t size)
{
    txIndex = 0;
    txSize = (size < TX_BUFFER_SIZE) ? size : TX_BUFFER_SIZE;
    for (int i = 0; i < txSize; i++)
        txBuffer[i] = buffer[i];

    SPIx->DR = txBuffer[txIndex++];
}

void SPI_ResetRX(uint8_t *buffer, uint32_t size)
{
    rxIndex = 0;
    rxSize = (size < RX_BUFFER_SIZE) ? size : RX_BUFFER_SIZE;
    for (int i = 0; i < rxSize; i++)
        buffer[i] = 0;
}

void SPI1_IRQHandler(void)
{
    if (SPI1->SR & SPI_SR_TXE)
    {
        if (txIndex < txSize)
            SPI1->DR = txBuffer[txIndex++];
        else
            SPI1->CR2 &= ~SPI_CR2_TXEIE;
    }

    if (SPI1->SR & SPI_SR_RXNE)
    {
        if (rxIndex < rxSize)
            rxBuffer[rxIndex++] = SPI1->DR;
    }
}


