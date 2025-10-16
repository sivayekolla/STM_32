#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "stm32l476xx.h"
#include <stdint.h>

#define I2C_ADDR_ESP32   0x28  // 7-bit address of ESP32 slave

void I2C1_Init(void);
void I2C1_Master_Transmit_IT(uint8_t *data, uint8_t len);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);

extern volatile uint8_t i2c_busy;

#endif
