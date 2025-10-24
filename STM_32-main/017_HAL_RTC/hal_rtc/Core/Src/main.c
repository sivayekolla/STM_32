#include <stdint.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "main.h"
#include "uart2.h"

void Error_Handler(void);

#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0xF9
#define BKP_FLAG             0x8888

RTC_HandleTypeDef rtc_handle;
uint8_t time[32];
uint8_t date[32];

void SystemClock_Config(void);
void rtc_calender_show(void);
void rtc_calender_config(void);
void rtc_init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    InitUart2();

    __HAL_RTC_RESET_HANDLE_STATE(&rtc_handle);
    rtc_init();

    if (HAL_RTCEx_BKUPRead(&rtc_handle, RTC_BKP_DR0) != BKP_FLAG)
    {
        rtc_calender_config();
    }

    while (1)
    {
        rtc_calender_show();
        HAL_Delay(1000);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

void rtc_init(void)
{
    RCC_OscInitTypeDef RCC_Oscinit = {0};
    RCC_PeriphCLKInitTypeDef RCC_pclk = {0};

    RCC_Oscinit.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
    RCC_Oscinit.LSEState = RCC_LSE_OFF;
    RCC_Oscinit.LSIState = RCC_LSI_ON;

    if (HAL_RCC_OscConfig(&RCC_Oscinit) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_pclk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    RCC_pclk.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

    if (HAL_RCCEx_PeriphCLKConfig(&RCC_pclk) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_RCC_RTC_ENABLE();

    rtc_handle.Instance = RTC;
    rtc_handle.Init.HourFormat = RTC_HOURFORMAT_24;
    rtc_handle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
    rtc_handle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
    rtc_handle.Init.OutPut = RTC_OUTPUT_DISABLE;

    if (HAL_RTC_Init(&rtc_handle) != HAL_OK)
    {
        Error_Handler();
    }
}

void rtc_calender_config(void)
{
    RTC_DateTypeDef datestruct = {0};
    RTC_TimeTypeDef timestruct = {0};

    datestruct.Year = 0x25;
    datestruct.Month = RTC_MONTH_SEPTEMBER;
    datestruct.Date = 0x10;
    datestruct.WeekDay = RTC_WEEKDAY_WEDNESDAY;

    if (HAL_RTC_SetDate(&rtc_handle, &datestruct, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }

    timestruct.Hours = 0x09;
    timestruct.Minutes = 0x43;
    timestruct.Seconds = 0x00;
    timestruct.TimeFormat = RTC_HOURFORMAT12_AM;
    timestruct.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    timestruct.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(&rtc_handle, &timestruct, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_RTCEx_BKUPWrite(&rtc_handle, RTC_BKP_DR0, BKP_FLAG);
}

void rtc_calender_show(void)
{
    uint16_t uhTimeSize = 0;
    uint16_t uhDateSize = 0;
    RTC_DateTypeDef datestruct = {0};
    RTC_TimeTypeDef timestruct = {0};

    HAL_RTC_GetDate(&rtc_handle, &datestruct, RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&rtc_handle, &timestruct, RTC_FORMAT_BIN);

    uhTimeSize = sprintf((char*)time, "Time: %02d:%02d:%02d\r\n",
                        timestruct.Hours, timestruct.Minutes, timestruct.Seconds);
    uhDateSize = sprintf((char*)date, "Date: %02d/%02d/20%02d\r\n",
                        datestruct.Date, datestruct.Month, datestruct.Year);

    Uart2TxData(time, uhTimeSize);
    Uart2TxData(date, uhDateSize);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
