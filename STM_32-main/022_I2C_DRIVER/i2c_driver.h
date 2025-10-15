#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "stm32l476xx.h"
#include <stdint.h>

#define I2C1_SCL_PIN   8
#define I2C1_SDA_PIN   9
#define SLAVE_ADDR     0x68

void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Write(uint8_t data);
void I2C1_SendData(uint8_t slaveAddr, uint8_t *data, uint32_t len);

#endif
