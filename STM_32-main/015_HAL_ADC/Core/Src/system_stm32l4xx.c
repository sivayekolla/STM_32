#include "stm32l4xx.h"

uint32_t SystemCoreClock = 4000000U;

void SystemInit(void)
{
  /* Configure FPU settings ----------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif

  /* Reset the RCC clock configuration to the default reset state ------------*/
  RCC->CR |= 0x00000001U;          /* Set HSION bit */
  RCC->CFGR = 0x00000000U;         /* Reset CFGR register */
  RCC->CR &= 0xEAF6FFFFU;          /* Reset HSEON, CSSON, PLLON bits */
  RCC->PLLCFGR = 0x00001000U;      /* Reset PLLCFGR register */
  RCC->CR &= 0xFFFBFFFFU;          /* Reset HSEBYP bit */
  RCC->CIER = 0x00000000U;         /* Disable all RCC interrupts */

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM1_BASE | 0x00U;
#else
  SCB->VTOR = FLASH_BASE | 0x00U;
#endif
}
