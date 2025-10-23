#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include "adc.h"
#include "uart2.h"



static void ADC_PC0_ContinousConversionInit(void);
static HAL_StatusTypeDef ADC_WaitForEOC(uint32_t timeout);

ADC_HandleTypeDef tAdc1;


static void ADC_PC0_ContinousConversionInit(void)
{
	//Configure PC0 as analog pin.
	GPIO_InitTypeDef tAdcGpio = {0};
	ADC_ChannelConfTypeDef tAdcChanConf = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();

	tAdcGpio.Pin = GPIO_PIN_0;
	tAdcGpio.Mode = GPIO_MODE_ANALOG;
	tAdcGpio.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOC, &tAdcGpio);


	//Configure ADc module for continuous conversion.
	__HAL_RCC_ADC_CLK_ENABLE();
	tAdc1.Instance = ADC1;
	tAdc1.Init.ClockPrescaler  = ADC_CLOCK_SYNC_PCLK_DIV2;
	tAdc1.Init.Resolution = ADC_RESOLUTION_12B;
	tAdc1.Init.ContinuousConvMode = ENABLE;
	tAdc1.Init.DiscontinuousConvMode = DISABLE;
	tAdc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	tAdc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	tAdc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	tAdc1.Init.NbrOfConversion = 1;
	tAdc1.Init.DMAContinuousRequests = DISABLE;
	tAdc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	HAL_ADC_Init(&tAdc1);

	tAdcChanConf.Channel = ADC_CHANNEL_1;
	tAdcChanConf.Rank = 1;
	tAdcChanConf.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;

	HAL_ADC_ConfigChannel(&tAdc1, &tAdcChanConf);

	return;
}


void ADC_PC0_SingleConversionInit(void)
{
	//Configure PC0 as analog pin.
	GPIO_InitTypeDef tAdcGpio = {0};
	ADC_ChannelConfTypeDef tAdcChanConf = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();

	tAdcGpio.Pin = GPIO_PIN_0;
	tAdcGpio.Mode = GPIO_MODE_ANALOG;
	tAdcGpio.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOC, &tAdcGpio);


	//Configure ADc module for continuous conversion.
	__HAL_RCC_ADC_CLK_ENABLE();
	tAdc1.Instance = ADC1;
	tAdc1.Init.ClockPrescaler  = ADC_CLOCK_SYNC_PCLK_DIV2;
	tAdc1.Init.Resolution = ADC_RESOLUTION_12B;
	tAdc1.Init.ContinuousConvMode = DISABLE;
	tAdc1.Init.DiscontinuousConvMode = DISABLE;
	tAdc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	tAdc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	tAdc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	tAdc1.Init.NbrOfConversion = 1;
	tAdc1.Init.DMAContinuousRequests = DISABLE;
	tAdc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	HAL_ADC_Init(&tAdc1);

	tAdcChanConf.Channel = ADC_CHANNEL_1;
	tAdcChanConf.Rank = 1;
	tAdcChanConf.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;

	HAL_ADC_ConfigChannel(&tAdc1, &tAdcChanConf);

	return;
}

uint16_t GetADC1Channel1Val(void)
{
	uint32_t uiAdcval = 0;


	HAL_ADC_Start(&tAdc1);

	HAL_ADC_PollForConversion(&tAdc1, 1);

	uiAdcval = HAL_ADC_GetValue(&tAdc1);

	return ((uint16_t)uiAdcval);
}

void ADC1Start(void)
{
	ADC_PC0_ContinousConversionInit();

	HAL_ADC_Start(&tAdc1);

	return;
}

void ADC1StartIt(void)
{
	HAL_ADC_Start_IT(&tAdc1);
}

void ADC_PC0_SingleConversionInitIt(void)
{
	//Configure PC0 as analog pin.
	GPIO_InitTypeDef tAdcGpio = {0};
	ADC_ChannelConfTypeDef tAdcChanConf = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();

	tAdcGpio.Pin = GPIO_PIN_0;
	tAdcGpio.Mode = GPIO_MODE_ANALOG;
	tAdcGpio.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOC, &tAdcGpio);


	//Configure ADc module for continuous conversion.
	__HAL_RCC_ADC_CLK_ENABLE();
	tAdc1.Instance = ADC1;
	tAdc1.Init.ClockPrescaler  = ADC_CLOCK_SYNC_PCLK_DIV2;
	tAdc1.Init.Resolution = ADC_RESOLUTION_12B;
	tAdc1.Init.ContinuousConvMode = DISABLE;
	tAdc1.Init.DiscontinuousConvMode = DISABLE;
	tAdc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	tAdc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	tAdc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	tAdc1.Init.NbrOfConversion = 1;
	tAdc1.Init.DMAContinuousRequests = DISABLE;
	tAdc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	HAL_ADC_Init(&tAdc1);

	tAdcChanConf.Channel = ADC_CHANNEL_1;
	tAdcChanConf.Rank = 1;
	tAdcChanConf.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;

	HAL_ADC_ConfigChannel(&tAdc1, &tAdcChanConf);

	HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
	return;
}

void ADC1_2_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&tAdc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint16_t uiAdcval = 0;
	uint8_t ucUartTxBuff[100] = {0};
	uint16_t uhTxBuffSize = 0;

	uiAdcval = HAL_ADC_GetValue(&tAdc1);

	uiAdcval &= 0x0FFF;

	uhTxBuffSize = sprintf((char*)ucUartTxBuff, "ADC1 Channel 1 Value is : %d\r\n", uiAdcval);

	Uart2TxData(ucUartTxBuff, uhTxBuffSize);

}

static HAL_StatusTypeDef ADC_WaitForEOC(uint32_t timeout)
{
    return HAL_ADC_PollForConversion(&tAdc1, timeout);
}
