/*
 * uart1.h
 *
 *  Created on: Sep 9, 2025
 *      Author: sfkfa
 */

#ifndef UART1_H_
#define UART1_H_

#include "stm32l4xx_hal.h"

#define UART2_PINS_POSRT	(GPIOA)
#define UART1_TX_PIN		(GPIO_PIN_9)
#define UART1_RX_PIN		(GPIO_PIN_10)

void InitUart1(void);
void Uart1TxData(uint8_t* pucTxDataBuff, uint16_t uhTxDataSize);
void Uart1RxData(uint8_t* pucRxDataBuff, uint16_t uhRxDataSize);
void TestUart1Interrupt(void);
void TestUartDMA(void);



#endif /* UART1_H_ */
