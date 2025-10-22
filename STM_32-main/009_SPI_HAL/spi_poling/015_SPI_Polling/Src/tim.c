#include "tim.h"
#include "uart2.h"


TIM_HandleTypeDef TimHandle;

/* Configure timer to timeout every second
 * Meaning 1 Hz
 *
 * System Default freq : 16 000 000
 * Default PCLK1 freq : 16 000 000
 * Default PCLK2 freq : 16 000 000
 * Timer prescaler : 1600 => 16000000/ 1600 = 10000KHz
 * Timer Period (Auto Reload Value) :  10000
 */
void TimeBaseInit(void)
{

	TIMx_CLK_EN;
	TimHandle.Instance = TIMx;
	TimHandle.Init.Prescaler = (1600 - 1);
	TimHandle.Init.Period = (10000 - 1);
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(&TimHandle);
	HAL_NVIC_SetPriority(TIMx_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIMx_IRQn);

	/* Start the time base generation in interrupt mode. */
	HAL_TIM_Base_Start_IT(&TimHandle);

	return;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//do something
	uint8_t ucTxDataBuff[] = "Timer 3 interrupt got triggered\r\n";
	Uart2TxData(ucTxDataBuff, sizeof(ucTxDataBuff));
}

void TIMx_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TimHandle);
}

