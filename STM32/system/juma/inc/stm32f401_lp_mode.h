#ifndef STM32F401_LP_MODE_H_
#define STM32F401_LP_MODE_H_
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#ifdef RTC_LSI
	#define RTC_ASYNCH_PREDIV    0x1F
	#define RTC_SYNCH_PREDIV     0x0408
#endif

#ifdef RTC_LSE
	#define RTC_ASYNCH_PREDIV    0x1F
	#define RTC_SYNCH_PREDIV     0x03FF
#endif


/*RTC Init*/
uint32_t RTC_CalendarShow(uint8_t* showtime);
void rtc_init(void);
void rtc_wake_up_timer_config(uint32_t time);
void StandbyMode_Measure(void);
void StopMode_Measure(void);
void SleepMode_Measure(void);
void StandbyRTCBKPSRAMMode_Measure(void);
#endif //STM32F401_LP_MODE_H_



