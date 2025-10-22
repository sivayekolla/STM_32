/*
 * uart2.c
 *
 *  Created on: Sep 8, 2025
 *      Author: sfkfa
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart2.h"

UART_HandleTypeDef tUart2 = {0};




void InitUart2(void)
{
	// PA2 -- > Uart2 TX
	// PA3 -- > Uart2 RX

	GPIO_InitTypeDef tUart2Gpio = {0};

	/* Enable Clock for GPIOA peripheral */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Enable Clock for USART2 peripheral */
	__HAL_RCC_USART2_CLK_ENABLE();

	/* GPIO pins act as alternate function pins. */
	tUart2Gpio.Pin = (UART2_RX_PIN | UART2_TX_PIN);

	/* Select the mode as alternate function. */
	tUart2Gpio.Mode = GPIO_MODE_AF_PP;

	/* Alternate function 7. */
	tUart2Gpio.Alternate = GPIO_AF7_USART2;

	/* No pull up resistor. */
	tUart2Gpio.Pull = GPIO_NOPULL;

	/* Configure speed as very high. */
	tUart2Gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	/* Init the GPIO pins. */
	HAL_GPIO_Init(UART2_PINS_POSRT, &tUart2Gpio);

	/* Configure UART. */
	/* Select the UART instance. */
	tUart2.Instance = USART2;

	/* Set the baud rate. */
	tUart2.Init.BaudRate = 115200;

	/* Select word length as 8 bits. */
	tUart2.Init.WordLength = UART_WORDLENGTH_8B;

	/* Select stop bits. */
	tUart2.Init.StopBits = UART_STOPBITS_1;

	/*Set Parity. */
	tUart2.Init.Parity = UART_PARITY_NONE;

	/* Select the mode. */
	tUart2.Init.Mode = UART_MODE_TX_RX;

	/* Set the hw flow control. */
	tUart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;

	/* Set the over sampling. */
	tUart2.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&tUart2);

	return;
}

void Uart2TxData(uint8_t* pucTxDataBuff, uint16_t uhTxDataSize)
{
	if(NULL != pucTxDataBuff
	&& NULL != (pucTxDataBuff + uhTxDataSize))
	{

		HAL_UART_Transmit(&tUart2, pucTxDataBuff, uhTxDataSize, 100);
	}
}
