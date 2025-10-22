#ifndef UART2_H_
#define UART2_H_

#include "stm32l4xx_hal.h"

#define UART2_PINS_POSRT	(GPIOA)
#define UART2_TX_PIN		(GPIO_PIN_2)
#define UART2_RX_PIN		(GPIO_PIN_3)

void InitUart2(void);
void Uart2TxData(uint8_t* pucTxDataBuff, uint16_t uhTxDataSize);



#endif /* UART2_H_ */
