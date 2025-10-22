/*
 * uart2.c
 *
 *  Created on: Sep 8, 2025
 *      Author: sfkfa
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart1.h"

UART_HandleTypeDef tUart1 = {0};
uint32_t uiRxCntr;
uint32_t uiTxCntr;
uint8_t ucTxBuffer[] = "Hello World\r\n";
uint16_t uiTxBuffSize = sizeof(ucTxBuffer);
uint8_t ucRxBuff[100];

DMA_HandleTypeDef tDmaUart1Tx;
DMA_HandleTypeDef tDmaUart1Rx;



void InitUart1(void)
{
	// PA2 -- > Uart2 TX
	// PA3 -- > Uart2 RX

	GPIO_InitTypeDef tUart1Gpio = {0};

	/* Enable Clock for GPIOA peripheral */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Enable Clock for USART2 peripheral */
	__HAL_RCC_USART1_CLK_ENABLE();

	/* Enable Clock for DMA1 peripheral */
	 __HAL_RCC_DMA1_CLK_ENABLE();

	/* GPIO pins act as alternate function pins. */
	tUart1Gpio.Pin = (UART1_RX_PIN | UART1_TX_PIN);

	/* Select the mode as alternate function. */
	tUart1Gpio.Mode = GPIO_MODE_AF_PP;

	/* Alternate function 7. */
	tUart1Gpio.Alternate = GPIO_AF7_USART2;

	/* No pull up resistor. */
	tUart1Gpio.Pull = GPIO_NOPULL;

	/* Configure speed as very high. */
	tUart1Gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	/* Init the GPIO pins. */
	HAL_GPIO_Init(GPIOA, &tUart1Gpio);

	/* Configure UART. */
	/* Select the UART instance. */
	tUart1.Instance = USART1;

	/* Set the baud rate. */
	tUart1.Init.BaudRate = 115200;

	/* Select word length as 8 bits. */
	tUart1.Init.WordLength = UART_WORDLENGTH_8B;

	/* Select stop bits. */
	tUart1.Init.StopBits = UART_STOPBITS_1;

	/*Set Parity. */
	tUart1.Init.Parity = UART_PARITY_NONE;

	/* Select the mode. */
	tUart1.Init.Mode = UART_MODE_TX_RX;

	/* Set the hw flow control. */
	tUart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;

	/* Set the over sampling. */
	tUart1.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&tUart1);

	/* Configure USART1 RX DMA*/
	tDmaUart1Rx.Instance = DMA1_Channel5;
	tDmaUart1Rx.Init.Request = DMA_REQUEST_2;
	tDmaUart1Rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	tDmaUart1Rx.Init.PeriphInc = DMA_PINC_DISABLE;
	tDmaUart1Rx.Init.MemInc = DMA_MINC_ENABLE;
	tDmaUart1Rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	tDmaUart1Rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	tDmaUart1Rx.Init.Mode = DMA_NORMAL;
	tDmaUart1Rx.Init.Priority = DMA_PRIORITY_LOW;

	HAL_DMA_Init(&tDmaUart1Rx);

	//link uart1 rx to dma
	tUart1.hdmarx = &tDmaUart1Rx;

	/* Configure USART1 RX DMA*/
	tDmaUart1Tx.Instance = DMA1_Channel4;
	tDmaUart1Tx.Init.Request = DMA_REQUEST_2;
	tDmaUart1Tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	tDmaUart1Tx.Init.PeriphInc = DMA_PINC_DISABLE;
	tDmaUart1Tx.Init.MemInc = DMA_MINC_ENABLE;
	tDmaUart1Tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	tDmaUart1Tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	tDmaUart1Tx.Init.Mode = DMA_NORMAL;
	tDmaUart1Tx.Init.Priority = DMA_PRIORITY_LOW;

	HAL_DMA_Init(&tDmaUart1Tx);

	//link uart1 rx to dma
	tUart1.hdmatx = &tDmaUart1Tx;


	//DMA1_Channel4_IRQn configuration
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    //DMA1_Channel5_IRQn configuration
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	return;
}

void Uart1TxData(uint8_t* pucTxDataBuff, uint16_t uhTxDataSize)
{
	if(NULL != pucTxDataBuff
	&& NULL != (pucTxDataBuff + uhTxDataSize))
	{

		HAL_UART_Transmit_IT(&tUart1, pucTxDataBuff, uhTxDataSize);
	}

	return;
}

void Uart1RxData(uint8_t* pucRxDataBuff, uint16_t uhRxDataSize)
{
	if(NULL != pucRxDataBuff
	&& NULL != pucRxDataBuff + uhRxDataSize)
	{
		HAL_UART_Receive_IT(&tUart1, pucRxDataBuff, uhRxDataSize);
	}
}

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&tUart1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uiRxCntr += 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uiTxCntr += 1;
}

void TestUart1Interrupt(void)
{
	Uart1TxData(ucTxBuffer, uiTxBuffSize);
	Uart1RxData(ucRxBuff, uiTxBuffSize);
}

void DMA1_CH5_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&tDmaUart1Rx);
}

void DMA1_CH4_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&tDmaUart1Tx);
}

void TestUartDMA(void)
{
	HAL_UART_Transmit_DMA(&tUart1, ucTxBuffer, uiTxBuffSize);
	HAL_UART_Receive_DMA(&tUart1, ucRxBuff, uiTxBuffSize);
}
