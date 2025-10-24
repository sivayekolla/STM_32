#include "main.h"
#include "uart2.h"

UART_HandleTypeDef huart2;

void InitUart2(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLING_DISABLED;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

void Uart2TxData(uint8_t* pucTxDataBuff, uint16_t uhTxDataSize)
{
    if((pucTxDataBuff != NULL) && (uhTxDataSize > 0))
    {
        HAL_UART_Transmit(&huart2, pucTxDataBuff, uhTxDataSize, HAL_MAX_DELAY);
    }
}