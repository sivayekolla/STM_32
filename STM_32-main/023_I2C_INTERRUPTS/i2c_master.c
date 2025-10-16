#include "i2c_master.h"


// Generate START and send address+write
// Set NBYTES and RELOAD = 0, AUTOEND = 1 for automatic stop
I2C1->CR2 = (I2C_ADDR_ESP32 << 1) | (i2c_tx_len << 16) | I2C_CR2_AUTOEND | I2C_CR2_START;
// enable TX interrupt (already set in init)
}


// I2C1 event IRQ: handle TXIS and TC and STOP
void I2C1_EV_IRQHandler(void) {
uint32_t isr = I2C1->ISR;


// TXIS: transmit data register empty and ready for next byte
if (isr & I2C_ISR_TXIS) {
if (i2c_tx_idx < i2c_tx_len) {
I2C1->TXDR = i2c_tx_buf[i2c_tx_idx++];
} else {
// nothing more to send
}
}


// STOPF handled by hardware when AUTOEND set -> STOPF flag
if (isr & I2C_ISR_STOPF) {
// clear STOPF
I2C1->ICR = I2C_ICR_STOPCF;
i2c_busy = 0;
i2c_tx_len = 0;
i2c_tx_idx = 0;
}


// TC (transfer complete) - not used with AUTOEND
}


void I2C1_ER_IRQHandler(void) {
uint32_t isr = I2C1->ISR;
if (isr & I2C_ISR_NACKF) {
// clear and abort
I2C1->ICR = I2C_ICR_NACKCF;
i2c_busy = 0;
}
if (isr & I2C_ISR_ARLO) {
I2C1->ICR = I2C_ICR_ARLOCF;
i2c_busy = 0;
}
if (isr & I2C_ISR_BERR) {
I2C1->ICR = I2C_ICR_BERRCF;
i2c_busy = 0;
}
}