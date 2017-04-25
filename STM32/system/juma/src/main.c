/**
******************************************************************************
* @file    main.c
* @author  CL
* @version V1.0.0
* @date    04-July-2014
* @brief   This application is used to control banlance car 
*          
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "osal.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_sdk_api.h"
#include "bluenrg_sdk_host_api.h"
#include "stm32f401_lp_mode.h"
//#include "dispatch.h"
#include "stm32f4xx_hal_msp.h"
#include "juma_sensor.h"

extern uint32_t sleep_time;
__IO uint32_t uwCounter = 0;

/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @defgroup SensorDemo
 *  @{
 */

/** @defgroup MAIN
 * @{
 */

/** @defgroup MAIN_Private_Defines
 * @{
 */
/* Private defines -----------------------------------------------------------*/

/**
 * @}
 */

/* Private macros ------------------------------------------------------------*/

/** @defgroup MAIN_Private_Variables
 * @{
 */
/* Private variables ---------------------------------------------------------*/

/**
 * @}
 */

/** @defgroup MAIN_Private_Function_Prototypes
 * @{
 */
/* Private function prototypes -----------------------------------------------*/
void user_process(void);
/**
 * @}
 */

/**
 * @brief  Main function to show how to use the BlueNRG Bluetooth Low Energy
 *         expansion board to send data from a Nucleo board to a smartphone
 *         with the support BLE and the "BlueNRG" app freely available on both
 *         GooglePlay and iTunes.
 *         The URL to the iTunes for the "BlueNRG" app is
 *         http://itunes.apple.com/app/bluenrg/id705873549?uo=5
 *         The URL to the GooglePlay is
 *         https://play.google.com/store/apps/details?id=com.st.bluenrg
 *         The source code of the "BlueNRG" app, both for iOS and Android, is
 *         freely downloadable from the developer website at
 *         http://software.g-maps.it/
 *         The board will act as Server-Peripheral.
 *
 *         After connection has been established:
 *          - by pressing the USER button on the board, the cube showed by
 *            the app on the smartphone will rotate.
 *
 *         The communication is done using a vendor specific profile.
 *
 * @param  None
 * @retval None
 */
/*host*/
extern volatile uint8_t host_notification_enabled;
extern volatile uint8_t set_connectable;
extern volatile   scan_device_found_info device_info;
volatile uint8_t device_connectable = 0;
extern RTC_HandleTypeDef RTCHandle;
/* Buffers used for displaying Time and Date */
uint8_t aShowTime[50] = {0};
uint8_t aShowDate[50] = {0},sleep_flag;
uint32_t stime ;

int main(void)
{
    /* STM32Cube HAL library initialization:
     *  - Configure the Flash prefetch, Flash preread and Buffer caches
     *  - Systick timer is configured by default as source of time base, but user
     *    can eventually implement his proper time base source (a general purpose
     *    timer for example or other time source), keeping in mind that Time base
     *    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
     *    handled in milliseconds basis.
     *  - Low Level Initialization
     */

    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();
    /* Configure LED0 */
    BSP_LED_Init(LED0);
    /*UART2 init*/
    UART_Init();
    HAL_Delay(100);
    /*sensor layer init*/
#ifndef SENSOR_FIFO
    jsensor_sys_init();
#endif
    /* Initialize the BlueNRG SPI driver */
    BNRG_SPI_Init();
    /* Initialize the BlueNRG HCI */
    HCI_Init();
    /* Reset BlueNRG hardware */
    BlueNRG_RST();
    /*Gatt And Gap Init*/
    ble_init_bluenrg();
    /* Enable Power Clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    //rtc_init();
    //dispatch_init();
    //send_acc_data(NULL);
    
    on_ready();

    while(1);
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    uwCounter++;
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    /* Turn LED0 on: Alarm generation */
    BSP_LED_On(LED0);
}

void sleep_flag_set(uint8_t flag)
{
    sleep_flag = flag;
}

void Error_Handler(void)
{
    while (1)
    {
        /* Toggle LED2 with a period of one second */
        BSP_LED_Toggle(LED0);
        HAL_Delay(1000);
    }
}


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
