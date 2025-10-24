#include "main.h"
#include "uart2.h"

UART_HandleTypeDef huart2;

void InitUart2(void)
{
    /* Basic UART configuration - this should work for most cases */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    
    /* Initialize UART */
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}

void Uart2TxData(uint8_t* pucTxDataBuff, uint16_t uhTxDataSize)
{
    if((pucTxDataBuff != NULL) && (uhTxDataSize > 0))
    {
        /* Transmit data with timeout */
        HAL_UART_Transmit(&huart2, pucTxDataBuff, uhTxDataSize, 1000);
    }
}
