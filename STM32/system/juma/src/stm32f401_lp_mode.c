#include "stm32f401_lp_mode.h"
#include "bluenrg_sdk_api.h"


static void RTC_CalendarConfig(void);
static void SYSCLKConfig_STOP(void);
/* Private variables ---------------------------------------------------------*/
/* RTC handler declaration */
RTC_HandleTypeDef RTCHandle;
/*RTC Init*/
void rtc_init(void)
{
    /* Configure RTC prescaler and RTC data registers as follow:
    - Hour Format = Format 24
    - Asynch Prediv = Value according to source clock
    - Synch Prediv = Value according to source clock
    - OutPut = Output Disable
    - OutPutPolarity = High Polarity
    - OutPutType = Open Drain */
    RTCHandle.Instance = RTC;
    RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
    RTCHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
    RTCHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
    RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
    RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
    RTC_CalendarConfig();

}

/*config wake up timer*/
void rtc_wake_up_timer_config(uint32_t time)
{
    /*## Configure the Wake up timer ###########################################*/
    /*  RTC Wakeup Interrupt Generation:
        Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))
        Wakeup Time = Wakeup Time Base * WakeUpCounter
                    = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI)) * WakeUpCounter
        ==> WakeUpCounter = Wakeup Time / Wakeup Time Base

        To configure the wake up timer to 20s the WakeUpCounter is set to 0xA017:
          RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16
          Wakeup Time Base = 16 /(~32.768KHz) = ~0,488 ms
          Wakeup Time = ~20s = 0,488ms  * WakeUpCounter
          ==> WakeUpCounter = ~20s/0,488ms = 40983 = 0xA017 */
    /* Disable Wake-up timer */
    time = (time*1000)/488;
    HAL_RTCEx_DeactivateWakeUpTimer(&RTCHandle);

    /*## Clear all related wakeup flags ########################################*/
    /* Clear PWR wake up Flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    /* Clear RTC Wake Up timer Flag */
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RTCHandle, RTC_FLAG_WUTF);

    /*## Setting the Wake up time ##############################################*/
    HAL_RTCEx_SetWakeUpTimer_IT(&RTCHandle, time-1, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}

/**
  * @brief  Configure the current time and date.
  * @param  None
  * @retval None
  */
static void RTC_CalendarConfig(void)
{
    RTC_DateTypeDef sdatestructure;
    RTC_TimeTypeDef stimestructure;

    /*##-1- Configure the Date #################################################*/
    /* Set Date: Tuesday April 14th 2015 */
    sdatestructure.Year = 0x14;
    sdatestructure.Month = RTC_MONTH_JANUARY;
    sdatestructure.Date = 0x01;
    sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;

    if(HAL_RTC_SetDate(&RTCHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-2- Configure the Time #################################################*/
    /* Set Time: 02:00:00 */
    stimestructure.Hours = 0x00;
    stimestructure.Minutes = 0x00;
    stimestructure.Seconds = 0x00;
    stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

    if(HAL_RTC_SetTime(&RTCHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

}


uint32_t RTC_CalendarShow(uint8_t* showtime)
{
    uint32_t current_time;

    RTC_DateTypeDef sdatestructureget;
    RTC_TimeTypeDef stimestructureget;
    /* Get the RTC current Time */
    HAL_RTC_GetTime(&RTCHandle, &stimestructureget, RTC_FORMAT_BIN);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&RTCHandle, &sdatestructureget, RTC_FORMAT_BIN);
#ifdef RTC_LSI
    current_time = (
                       (1000 - (stimestructureget.SubSeconds)*1000/1032) +
                       (stimestructureget.Seconds )*1000+
                       (stimestructureget.Minutes )*1000*60
                   );
#endif

#ifdef RTC_LSE
    current_time = (
                       (1000 - (stimestructureget.SubSeconds)*1000/1023) +
                       (stimestructureget.Seconds )*1000+
                       (stimestructureget.Minutes )*1000*60
                   );
#endif

    current_time &= 0x00FFFFFF;

    return current_time;
}

/**
  * @brief  This function configures the system to enter Sleep mode for
  *         current consumption measurement purpose.
  *         Sleep Mode
  *         ==========
  *            - System Running at PLL (84MHz)
  *            - Flash 2 wait state
  *            - Instruction and Data caches ON
  *            - Prefetch ON
  *            - Code running from Internal FLASH
  *            - All peripherals disabled.
  *            - Wakeup using EXTI Line (Key Button)
  * @param  None
  * @retval None
  */
void SleepMode_Measure(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* Configure all GPIO as analog to reduce current consumption on non used IOs */
    /* Enable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();

    /* Suspend Tick increment to prevent wakeup by Systick interrupt.
       Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base) */
    HAL_SuspendTick();

    /* Request to enter SLEEP mode */
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);

    /* Resume Tick interrupt if disabled prior to sleep mode entry */
    HAL_ResumeTick();
}


void StandbyMode_Measure(void)
{
    /* Enable Power Clock*/
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Allow access to Backup */
    HAL_PWR_EnableBkUpAccess();

    /* Reset RTC Domain */
    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();

    /* The Following Wakeup sequence is highly recommended prior to each Standby mode entry
       mainly  when using more than one wakeup source this is to not miss any wakeup event.
         - Disable all used wakeup sources,
         - Clear all related wakeup flags,
         - Re-enable all used wakeup sources,
         - Enter the Standby mode.
    */
    /*#### Disable all used wakeup sources: WKUP pin ###########################*/
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

    /*#### Clear all related wakeup flags ######################################*/
    /* Clear PWR wake up Flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    /* Enable WKUP pin */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

    /* Request to enter STANDBY mode */
    HAL_PWR_EnterSTANDBYMode();
}

/**
  * @brief  This function configures the system to enter Stop mode with RTC
  *         clocked by LSE or LSI  for current consumption measurement purpose.
  *         STOP Mode with RTC clocked by LSE/LSI
  *         =====================================
  *           - RTC Clocked by LSE or LSI
  *           - Regulator in LP mode
  *           - HSI, HSE OFF and LSI OFF if not used as RTC Clock source
  *           - No IWDG
  *           - FLASH in deep power down mode
  *           - Automatic Wakeup using RTC clocked by LSE/LSI (~20s)
  * @param  None
  * @retval None
  */
void StopMode_Measure(void)
{
#ifdef STOP_IO

    GPIO_InitTypeDef GPIO_InitStruct;

    /* Configure all GPIO as analog to reduce current consumption on non used IOs */
    /* Enable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
#endif

    /* FLASH Deep Power Down Mode enabled */
    HAL_PWREx_EnableFlashPowerDown();

    /*## Enter Stop Mode #######################################################*/
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);
    /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
    PLL as system clock source (HSE and PLL are disabled in STOP mode) */
    SYSCLKConfig_STOP();

    /* Disable Wake-up timer */
    if(HAL_RTCEx_DeactivateWakeUpTimer(&RTCHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}


/**
  * @brief  This function configures the system to enter Standby mode with RTC
  *         clocked by LSE or LSI and with Backup SRAM ON for current consumption
  *         measurement purpose.
  *         STANDBY Mode with RTC clocked by LSE/LSI and BKPSRAM
  *         ====================================================
  *           - RTC Clocked by LSE or LSI
  *           - Backup SRAM ON
  *           - IWDG OFF
  *           - Automatic Wakeup using RTC clocked by LSE/LSI (after ~20s)
  * @param  None
  * @retval None
  */
void StandbyRTCBKPSRAMMode_Measure(void)
{
//	/*rtc_init */
//  rtc_init();
//	/*wake up timer*/
//	rtc_wake_up_timer_config(0x5000);
    /* Enable BKPRAM Clock */
    __HAL_RCC_BKPSRAM_CLK_ENABLE();

    /* Enable the Backup SRAM low power Regulator */
    HAL_PWREx_EnableBkUpReg();

    /*## Enter Standby Mode ####################################################*/
    /* Request to enter STANDBY mode  */
    HAL_PWR_EnterSTANDBYMode();
}


/**
  * @brief  Configures system clock after wake-up from STOP: enable HSI, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
static void SYSCLKConfig_STOP(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    uint32_t pFLatency = 0;

    /* Get the Oscillators configuration according to the internal RCC registers */
    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

    /* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    //RCC_OscInitStruct.HSICalibrationValue = 0x10;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Get the Clocks configuration according to the internal RCC registers */
    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
    {
        Error_Handler();
    }
}






