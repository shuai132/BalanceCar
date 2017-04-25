/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : gp_timer.h
* Author             : AMS - HEA&RF BU
* Version            : V1.0.0
* Date               : 19-July-2012
* Description        : General purpose timer library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __GP_TIMER_H__
#define __GP_TIMER_H__

#include "clock.h"
#include "ble_status.h"
#ifdef __DMA_LP__
#include "stm32xx_timerserver.h"
#endif /* __DMA_LP__ */

/**
 * timer
 *
 * A structure that represents a timer. Use Timer_Set() to set the timer.
 *
 */
struct timer {
    
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    
  tClockTime start;
  tClockTime interval;
  
#endif
};

typedef void (* TIMER_HCI_TIMEOUT_NOTIFY_CALLBACK_TYPE)(void);

/**
 * Timer_Set
 *
 * @param[in] t             Pointer to a timer structure
 * @param[in] interval      timeout value
 *
 * This function sets the timeout value of a timer.
 *
 */
void Timer_Set(struct timer *t, tClockTime interval);

/**
 * Timer_Reset
 * 
 * @param[in] t     Pointer to a timer structure
 * 
 * This function resets the timer with the same interval given
 * with Timer_Set, starting from the time it previously expired.
 * 
 */ 
void Timer_Reset(struct timer *t);

/**
 * Timer_Restart
 * 
 * @param[in]  t   Pointer to a timer structure
 * 
 * This function resets the timer with the same interval given
 * with Timer_Set, starting from the current time.
 * 
 */ 
void Timer_Restart(struct timer *t);

/**
 * Timer_Expired
 *
 * @param[in] t    Pointer to a timer structure
 *
 * This function returns TRUE if timer is expired, FALSE otherwise.
 *
 */
int Timer_Expired(struct timer *t);

/**
 * Timer_Expired
 *
 * @param[in] t    Pointer to a timer structure
 *
 * This function returns the time needed for expiration.
 *
 * @return  Time before timer's expiration.
 */
tClockTime Timer_Remaining(struct timer *t);

#ifdef __DMA_LP__
tBleStatus Blue_NRG_HCI_Timer_Start(uint32_t expiryTime,
		TIMER_HCI_TIMEOUT_NOTIFY_CALLBACK_TYPE timercb,
					   uint8_t *timerID);

tBleStatus Blue_NRG_HCI_Timer_Stop(uint8_t timerID);
#endif /* __DMA_LP__ */

#endif /* __GP_TIMER_H__ */
