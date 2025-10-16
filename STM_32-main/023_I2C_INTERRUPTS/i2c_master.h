#ifndef I2C_MASTER_H
#define I2C_MASTER_H


#include <stdint.h>


#define I2C_ADDR_ESP32 (0x42U)
#define I2C_BUF_SIZE 16


void I2C1_MasterInit(void);
void I2C1_MasterSendBytes(uint8_t *buf, uint8_t len);


// state
extern volatile uint8_t i2c_tx_buf[I2C_BUF_SIZE];
extern volatile uint8_t i2c_tx_len;


#endif // I2C_MASTER_H