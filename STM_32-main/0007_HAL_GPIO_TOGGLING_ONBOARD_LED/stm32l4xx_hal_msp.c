#include "main.h"

void HAL_MspInit(void)
{
    // System interrupt init (basic setup)
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}
