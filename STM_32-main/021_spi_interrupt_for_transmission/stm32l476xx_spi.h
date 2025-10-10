#ifndef STM32L476XX_SPI_H
#define STM32L476XX_SPI_H

#include "stm32l476xx.h"
#include <stdint.h>

void SPI_Init_IRQ(SPI_TypeDef *SPIx);
void SPI_SendData_IRQ(SPI_TypeDef *SPIx, uint8_t data);

#endif
