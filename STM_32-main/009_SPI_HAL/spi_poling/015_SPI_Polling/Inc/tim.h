#ifndef TIM_H_
#define TIM_H_

#include "stm32l4xx_hal.h"

#define TIMx				TIM3
#define TIMx_CLK_EN			__HAL_RCC_TIM3_CLK_ENABLE()
#define TIMx_IRQn			TIM3_IRQn
#define TIMx_IRQHandler		TIM3_IRQHandler



void TimeBaseInit(void);



#endif /* TIM_H_ */
