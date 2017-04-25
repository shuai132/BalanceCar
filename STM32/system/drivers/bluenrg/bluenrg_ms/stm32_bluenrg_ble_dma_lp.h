/**
  ******************************************************************************
  * @file    stm32_bluenrg_ble_dma_lp.h
  * @author  CL
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_BLUENRG_BLE_DMA_LP_H
#define __STM32_BLUENRG_BLE_DMA_LP_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/ 
#ifdef USE_STM32F4XX_NUCLEO
  #include "stm32f4xx_hal.h"
  #include "stm32f4xx_nucleo.h"
  #include "stm32f4xx_nucleo_bluenrg_dma_lp.h"
  #include "stm32f4xx_hal_bluenrg_gpio.h"
  #include "stm32f4xx_hal_bluenrg_spi.h"
  #include "stm32f4xx_hal_bluenrg_dma.h"
#endif /* USE_STM32F4XX_NUCLEO */
   
#ifdef USE_STM32L0XX_NUCLEO
  #include "stm32l0xx_hal.h"
  #include "stm32l0xx_nucleo.h"
  #include "stm32l0xx_nucleo_bluenrg_dma_lp.h"
  #include "stm32l0xx_hal_bluenrg_gpio.h"
  #include "stm32l0xx_hal_bluenrg_spi.h"
  #include "stm32l0xx_hal_bluenrg_dma.h"   
#endif /* USE_STM32L0XX_NUCLEO */
  
#include "stm32xx_lpm.h"  
#include "stm32xx_timerserver.h"

/** @addtogroup BSP
 *  @{
 */

/** @addtogroup X-NUCLEO-IDB04A1
 *  @{
 */

/** @addtogroup STM32_BLUENRG_BLE_DMA_LP
 *  @{
 */
 
/** @defgroup STM32_BLUENRG_BLE_DMA_LP_Exported_Constants 
 * @{
 */ 
 
#define	SPI_END_RECEIVE_FIX	1	/**< Need some delay after receiving an event to prevent dummy read */

 /**
  * All timeout below are given in term of Ticks of the TimerServer.
  * The tick of the timerserver depends on the LPO clock used as input and the
  * prescaler of the wakeup timer.
  * In order to get the most efficient implementation, the wakeuptimer prescaler should divide
  * the input clock by 2
  *
  * The values below are based on the LSI clock @37Kz.
  * For this, the tick is 54us
  */
#define SPI_FIX_TIMEOUT	                2  /**< 150 us - Note: 2 ticks result into more than 2*54us due to some inaccuracies and latencies */
#define SPI_TX_TIMEOUT                  3  /**< Value to be tuned to prevent trying to send a command to BlueNRG if it is not yet woken up */
#define	SPI_END_RECEIVE_FIX_TIMEOUT	    2
#define	BLUENRG_HOLD_TIME_IN_RESET	    1
#define	BLUENRG_HOLD_TIME_AFTER_RESET	93 /**< 5ms */
 
/**
 * When the SPI does not have any FIFO (STM32L0, etc...), this parameter should be set to 1
 * When the SPI does have a FIFO, this parameter shall be set to the size of the FIFO in bytes (STM32L4 = 4)
 */
#define	SPI_FIFO_RX_DEPTH	1

/**
 * @}
 */
 
/** @defgroup STM32_BLUENRG_BLE_DMA_LP_Exported_Functions 
 * @{
 */
// FIXME: add prototypes for BlueNRG here
void BNRG_SPI_Close(void);
void BNRG_SPI_Init(void);
void BlueNRG_RST(void);
void BlueNRG_SPI_Write(uint8_t* header_data,
                       uint8_t* payload_data,
                       uint8_t header_size,
                       uint8_t payload_size);
void BlueNRG_SPI_Request_Events(uint8_t *buffer, uint8_t buff_size);
void BlueNRG_DMA_RxCallback(void);
void BlueNRG_DMA_TxCallback(void);
void BlueNRG_SPI_IRQ_Callback(void);
void BNRG_MSP_SPI_Init(SPI_HandleTypeDef * hspi);

/**
 * @}
 */
 
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __STM32_BLUENRG_BLE_DMA_LP_H */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

