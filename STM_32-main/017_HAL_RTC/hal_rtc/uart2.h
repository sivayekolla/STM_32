#ifndef UART2_H
#define UART2_H

#include "stm32l4xx_hal.h"

void InitUart2(void);
void Uart2TxData(uint8_t* pucTxDataBuff, uint16_t uhTxDataSize);

#endif /* UART2_H */