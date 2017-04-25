/**
  ******************************************************************************
  * @file    stm32_bluenrg_ble_dma_lp.c
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
  
/* Includes ------------------------------------------------------------------*/
#include "stm32_bluenrg_ble_dma_lp.h"
#include "hci_const.h"

/** @addtogroup BSP
 *  @{
 */

/** @addtogroup X-NUCLEO-IDB04A1
 *  @{
 */

/** @defgroup STM32_BLUENRG_BLE_DMA_LP
 *  @{
 */

/** @defgroup STM32_BLUENRG_BLE_DMA_LP_Private_Defines 
 *  @{
 */
 
#define TEST_TX_BUFFER_LIMITATION 0
#define MAX_TX_BUFFER_SIZE 0x7F
#define BLUENRG_READY_STATE 0x02

#define HEADER_SIZE 5
#define MAX_BUFFER_SIZE 255

/**
 * @}
 */
 
/** @defgroup STM32_BLUENRG_BLE_DMA_LP_Private_Types
* @{
*/ 

typedef enum
{
  SPI_HEADER_TRANSMIT,
  SPI_PAYLOAD_TRANSMIT
} SPI_TRANSMIT_REQUEST_t;

typedef enum
{
  SPI_HEADER_TRANSMITTED,
  SPI_PAYLOAD_TRANSMITTED
} SPI_TRANSMIT_EVENT_t;

typedef enum
{
  SPI_REQUEST_VALID_HEADER_FOR_RX,
  SPI_REQUEST_VALID_HEADER_FOR_TX,
  SPI_REQUEST_PAYLOAD
} SPI_RECEIVE_REQUEST_t;

typedef enum
{
  SPI_CHECK_RECEIVED_HEADER_FOR_RX,
  SPI_CHECK_RECEIVED_HEADER_FOR_TX,
  SPI_RECEIVE_END
} SPI_RECEIVE_EVENT_t;

typedef enum
{
  SPI_AVAILABLE,
  SPI_BUSY
} SPI_PERIPHERAL_STATUS_t;

typedef struct
{
  SPI_TRANSMIT_EVENT_t Spi_Transmit_Event;
  uint8_t* header_data;
  uint8_t* payload_data;
  uint8_t header_size;
  uint8_t payload_size;
  uint8_t payload_size_to_transmit;
  uint8_t packet_cont;
  uint8_t RequestPending;
} SPI_Transmit_Context_t;

typedef struct
{
  SPI_RECEIVE_EVENT_t Spi_Receive_Event;
  uint16_t payload_len;
  uint8_t* buffer;
  uint8_t buffer_size;
} SPI_Receive_Context_t;

typedef struct
{
  SPI_HandleTypeDef *hspi;
  SPI_PERIPHERAL_STATUS_t Spi_Peripheral_State;
  SPI_Receive_Context_t SPI_Receive_Context;
  SPI_Transmit_Context_t SPI_Transmit_Context;
} SPI_Context_t;

/**
 * @}
 */

/** @defgroup STM32_BLUENRG_BLE_DMA_LP_Private_Variables
 * @{
 */

SPI_HandleTypeDef SpiHandle;
SPI_Context_t SPI_Context;

const uint8_t Write_Header_CMD[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
const uint8_t Read_Header_CMD[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
const uint8_t dummy_bytes = 0xFF;

uint8_t Received_Header[HEADER_SIZE];

#ifdef ENABLE_SPI_FIX
static uint8_t StartupTimerId;
#endif
static uint8_t TxRxTimerId;
volatile uint8_t ubnRFresetTimerLock;
pf_TIMER_TimerCallBack_t pTimerTxRxCallback;

/**
 * @}
 */

/** @defgroup STM32_BLUENRG_BLE_DMA_LP_Extern_Variables
 * @{
 */
extern uint8_t* HCI_read_packet;

/**
 * @}
 */

/** @defgroup STM32_BLUENRG_BLE_DMA_LP_Private_Function_Prototypes 
 *  @{
 */
 
/* Private function prototypes -----------------------------------------------*/
static void SPI_Transmit_Manager(SPI_TRANSMIT_REQUEST_t TransmitRequest);
static void SPI_Receive_Manager(SPI_RECEIVE_REQUEST_t ReceiveRequest);
static void set_irq_as_output(void);
static void set_irq_as_input(void);
static void Disable_SPI_Receiving_Path(void);
static void Enable_SPI_Receiving_Path(void);
static void Enable_SPI_CS(void);
static void Disable_SPI_CS(void);
static void DisableEnable_SPI_CS(void);
static void TransmitClosure(void);
static void ReceiveClosure(void);
static void ReceiveHeader(SPI_RECEIVE_EVENT_t ReceiveEvent, uint8_t * DataHeader);
static void WakeupBlueNRG(void);
static void TimerStartupCallback(void);
static void TimerTransmitCallback(void);
static void pf_nRFResetTimerCallBack(void);
static void TimerTxRxCallback(void);
static void ProcessEndOfReceive(void);

/**
 * @}
 */

/** @defgroup STM32_BLUENRG_BLE_DMA_LP_Exported_Functions 
 * @{
 */ 
 
/**
 * @brief  This function notify when then BlueNRG nRESET may be released
 * @param  None
 * @retval None
 */
static void pf_nRFResetTimerCallBack(void)
{
  ubnRFresetTimerLock = 0;
  
  return;
}

/**
 * @brief  Timer callback to handle RxTx Timers
 * @param  None
 * @retval None
 */
static void TimerTxRxCallback(void)
{
  pTimerTxRxCallback();
  
  return;
}

/**
 * @brief  Close the receiver path
 * @param  None
 * @retval None
 */
static void ReceiveClosure(void)
{
  /*
   *  Disable both DMA
   */
  __HAL_DMA_DISABLE(SPI_Context.hspi->hdmatx);
  __HAL_DMA_DISABLE(SPI_Context.hspi->hdmarx);
  
  /*
   * Check if a command is pending
   */
  __disable_irq();
  if(SPI_Context.SPI_Transmit_Context.RequestPending == TRUE)
  {
    SPI_Context.SPI_Transmit_Context.RequestPending = FALSE;
    SPI_Context.Spi_Peripheral_State = SPI_BUSY;
    Disable_SPI_Receiving_Path();
    __enable_irq();
    WakeupBlueNRG();
  }
  else
  {
    SPI_Context.Spi_Peripheral_State = SPI_AVAILABLE;
    __enable_irq();
  }
  
  return;
}

/**
 * @brief  Delay Notification to the App to prevent dummy event read
 * @param  None
 * @retval None
 */
static void ProcessEndOfReceive(void)
{
  ReceiveClosure();
  
  HCI_Isr(HCI_read_packet, SPI_Context.SPI_Receive_Context.payload_len);
  
  return;
}

/**
 * @brief  Timer callback to apply timeout SPI FIX
 * @param  None
 * @retval None
 */
static void TimerTransmitCallback(void)
{
  SPI_Receive_Manager(SPI_REQUEST_VALID_HEADER_FOR_TX);	/**< BlueNRG not ready for writing */
  LPM_Mode_Request(eLPM_SPI_TX, eLPM_Mode_Sleep);
  
  return;
}

/**
 * @brief  Closes the SPI when BLE is disabled by the application
 * 		   Releases allocated resources
 * @param  None
 * @retval None
 */
void BNRG_SPI_Close(void)
{
#ifdef ENABLE_SPI_FIX
  TIMER_Delete(StartupTimerId);
#endif
  TIMER_Delete(TxRxTimerId);
  
  return;
}

/**
 * @brief  Initializes the SPI communication with the BlueNRG Shield.
 * @param  None
 * @retval None
 */
void BNRG_SPI_Init(void)
{
  BNRG_MSP_SPI_Init(&SpiHandle);
  
  SPI_Context.hspi = &SpiHandle;  
  
  SPI_Context.Spi_Peripheral_State = SPI_AVAILABLE;
  SPI_Context.SPI_Transmit_Context.RequestPending = FALSE;
  
  __HAL_BLUENRG_SPI_ENABLE_DMAREQ(&SpiHandle, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
  
  __HAL_SPI_ENABLE(&SpiHandle);

#ifdef ENABLE_SPI_FIX
  TIMER_Create(eTimerModuleID_Interrupt, &StartupTimerId, eTimerMode_SingleShot, TimerStartupCallback);
#endif
  TIMER_Create(eTimerModuleID_Interrupt, &TxRxTimerId, eTimerMode_SingleShot, TimerTxRxCallback);
  return;
}

/**
 * @brief  Initializes the SPI communication with the BlueNRG Shield
 * @param  None
 * @retval None
 */
void BNRG_MSP_SPI_Init(SPI_HandleTypeDef * hspi)
{
  hspi->Instance = BNRG_SPI_INSTANCE;
  hspi->Init.Mode = BNRG_SPI_MODE;
  hspi->Init.Direction = BNRG_SPI_DIRECTION;
  hspi->Init.DataSize = BNRG_SPI_DATASIZE;
  hspi->Init.CLKPolarity = BNRG_SPI_CLKPOLARITY;
  hspi->Init.CLKPhase = BNRG_SPI_CLKPHASE;
  hspi->Init.NSS = BNRG_SPI_NSS;
  hspi->Init.FirstBit = BNRG_SPI_FIRSTBIT;
  hspi->Init.TIMode = BNRG_SPI_TIMODE;
  hspi->Init.CRCPolynomial = BNRG_SPI_CRCPOLYNOMIAL;
  hspi->Init.BaudRatePrescaler = BNRG_SPI_BAUDRATEPRESCALER;
  hspi->Init.CRCCalculation = BNRG_SPI_CRCCALCULATION;

  HAL_SPI_Init(hspi);

  return;
}

/**
 * @brief  Resets the BlueNRG.
 * @param  None
 * @retval None
 */
void BlueNRG_RST(void)
{
  uint8_t ubnRFResetTimerID;
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Pin = BNRG_SPI_RESET_PIN;
  GPIO_InitStruct.Speed = BNRG_SPI_RESET_SPEED;
  TIMER_Create(eTimerModuleID_Interrupt, &ubnRFResetTimerID, eTimerMode_SingleShot, pf_nRFResetTimerCallBack);
  
  BNRG_SPI_RESET_CLK_ENABLE();
  
  HAL_GPIO_WritePin(BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BNRG_SPI_RESET_PORT, &GPIO_InitStruct);
  
  TIMER_Start(ubnRFResetTimerID, BLUENRG_HOLD_TIME_IN_RESET);
  ubnRFresetTimerLock = 1;
  while(ubnRFresetTimerLock == 1);
  
  HAL_GPIO_WritePin(BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_SET);
  
#if 1
  /*
   * Limitation in HAL V1.1.0
   * The HAL_GPIO_Init() is first configuring the Mode of the IO before the Pull UP configuration
   * To avoid glitch on the IO, the configuration shall go through an extra step OUTPUT/PULLUP
   * to set upfront the PULL UP configuration.
   */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNRG_SPI_RESET_PORT, &GPIO_InitStruct);
#endif
  
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNRG_SPI_RESET_PORT, &GPIO_InitStruct);
  
  TIMER_Start(ubnRFResetTimerID, BLUENRG_HOLD_TIME_AFTER_RESET);
  ubnRFresetTimerLock = 1;
  while(ubnRFresetTimerLock == 1);
  TIMER_Delete(ubnRFResetTimerID);
  
  return;
}

/**
 * @brief  Writes data from local buffer to SPI.
 * @param  hspi: SPI handle
 * @param  header_data: First data buffer to be written
 * @param  payload_data: Second data buffer to be written
 * @param  header_size: Size of first data buffer to be written
 * @param  payload_size: Size of second data buffer to be written
 * @retval Number of read bytes
 */
void BlueNRG_SPI_Write(uint8_t* header_data, uint8_t* payload_data, uint8_t header_size, uint8_t payload_size)
{  
  SPI_Context.SPI_Transmit_Context.header_data = header_data;
  SPI_Context.SPI_Transmit_Context.payload_data = payload_data;
  SPI_Context.SPI_Transmit_Context.header_size = header_size;
  SPI_Context.SPI_Transmit_Context.payload_size = payload_size;
  
  SPI_Context.SPI_Transmit_Context.packet_cont = FALSE;
  
  __disable_irq();
  if(SPI_Context.Spi_Peripheral_State == SPI_AVAILABLE)
  {
    SPI_Context.Spi_Peripheral_State = SPI_BUSY;
    Disable_SPI_Receiving_Path();
    __enable_irq();
    WakeupBlueNRG();
  }
  else
  {
    SPI_Context.SPI_Transmit_Context.RequestPending = TRUE;
    __enable_irq();
  }
  
  return;
}

/**
 * @brief  Set in Output mode the IRQ.
 * @param  None
 * @retval None
 */
static void set_irq_as_output(void)
{
  HAL_GPIO_WritePin(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN, GPIO_PIN_SET);
  HAL_LPPUART_GPIO_Set_Mode(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN_POSITION, GPIO_MODE_OUTPUT_PP);
  __HAL_GPIO_EXTI_CLEAR_IT(BNRG_SPI_IRQ_PIN);
}

/**
 * @brief  Set the IRQ in input mode.
 * @param  None
 * @retval None
 */
static void set_irq_as_input(void)
{
  HAL_GPIO_WritePin(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN, GPIO_PIN_RESET); // WARNING: it may conflict with BlueNRG driving High
  HAL_LPPUART_GPIO_Set_Mode(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN_POSITION, GPIO_MODE_INPUT);
}

/**
 * @brief  Enable SPI IRQ.
 * @param  None
 * @retval None
 */
static void Enable_SPI_Receiving_Path(void)
{  
  __HAL_GPIO_EXTI_CLEAR_IT(BNRG_SPI_EXTI_PIN);
  HAL_NVIC_ClearPendingIRQ(BNRG_SPI_EXTI_IRQn);
  HAL_NVIC_EnableIRQ(BNRG_SPI_EXTI_IRQn);
  
  if (HAL_GPIO_ReadPin(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN) == GPIO_PIN_SET)
  {
    __HAL_GPIO_EXTI_GENERATE_SWIT(BNRG_SPI_IRQ_PIN);
  }
}

/**
 * @brief  Disable SPI IRQ.
 * @param  None
 * @retval None
 */
static void Disable_SPI_Receiving_Path(void)
{  
  HAL_NVIC_DisableIRQ(BNRG_SPI_EXTI_IRQn);
}

/**
 * @brief  Enable SPI CS.
 * @param  None
 * @retval None
 */
static void Enable_SPI_CS(void)
{
  /* CS reset */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Disable SPI CS.
 * @param  None
 * @retval None
 */
static void Disable_SPI_CS(void)
{
  while (__HAL_SPI_GET_FLAG(SPI_Context.hspi,SPI_FLAG_BSY) == SET);
  
  /* CS set */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Disable and Enable SPI CS.
 * @param  None
 * @retval None
 */
static void DisableEnable_SPI_CS(void)
{
  while (__HAL_SPI_GET_FLAG(SPI_Context.hspi,SPI_FLAG_BSY) == SET);
  
  /* CS set */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
  
  /* CS reset */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Tx and Rx Transfer completed callbacks
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *              the configuration information for SPI module.
 * @retval None
 */
void BlueNRG_DMA_RxCallback(void)
{
  uint16_t byte_count;
  uint8_t ready_state;
  
  __HAL_DMA_CLEAR_FLAG(SPI_Context.hspi->hdmarx, BNRG_SPI_RX_DMA_TC_FLAG);
  
  /**
   * The TCIF shall be cleared to be able to start a new DMA transmission later on when required.
   * When receiving data, the TCIE is not set as there is no need to handle the interrupt
   * handler of the DMA_Tx.
   * The TCIF clearing is mandatory on STM32F4 but not on STM32L0.
   * In order to keep code identical across platform, the TCIF clearing may be kept as well on
   * the STM32L0 and all other MCUs.
   */
  __HAL_DMA_CLEAR_FLAG(SPI_Context.hspi->hdmatx, BNRG_SPI_TX_DMA_TC_FLAG);
  switch (SPI_Context.SPI_Receive_Context.Spi_Receive_Event)
  {
  case SPI_CHECK_RECEIVED_HEADER_FOR_RX:
    byte_count = (Received_Header[4]<<8)|Received_Header[3];
    ready_state = Received_Header[0];
    
    if ((byte_count == 0) || (ready_state != BLUENRG_READY_STATE))
    {
      if (HAL_GPIO_ReadPin(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN) == GPIO_PIN_RESET)
      {
        /**
         * This USE CASE shall never happen as this may break the IRQ/CS specification
         * The IRQ line shall never be low when CS is low to avoid BlueNRG race condition when
         * entering low power mode
         * the SPI_END_RECEIVE_FIX has been implemented to make sure this USE CASE never occurs
         * However, even when the behavior is not compliant to the specification, the BlueNRG
         * may not fail so it is increasing robustness by adding this checking just in case the
         * timeout define in the workaround is too short which will end up to marginally brake
         * the specification.
         * This checking will poping BluenRG for a dummy even
         */
        
        /* Release CS line */
        Disable_SPI_CS();
        
        LPM_Mode_Request(eLPM_SPI_RX, eLPM_Mode_LP_Stop);
        
        ReceiveClosure();
      }
      else
      {
        DisableEnable_SPI_CS();
        SPI_Receive_Manager(SPI_REQUEST_VALID_HEADER_FOR_RX); /**< BlueNRG not ready for reading */
      }
    }
    else
    {
      SPI_Receive_Manager(SPI_REQUEST_PAYLOAD);	/**< BlueNRG is ready for reading */
    }
    break;
    
  case SPI_RECEIVE_END:
    /* Release CS line */
    Disable_SPI_CS();
    
    LPM_Mode_Request(eLPM_SPI_RX, eLPM_Mode_LP_Stop);
    
#if (SPI_END_RECEIVE_FIX == 1)
    pTimerTxRxCallback = ProcessEndOfReceive;
    TIMER_Start(TxRxTimerId, SPI_END_RECEIVE_FIX_TIMEOUT);
#else
    ProcessEndOfReceive();
#endif
    break;
    
  case SPI_CHECK_RECEIVED_HEADER_FOR_TX:
    byte_count = (Received_Header[2]<<8)|Received_Header[1];
    ready_state = Received_Header[0];
    
    if ((byte_count == 0) || (ready_state != BLUENRG_READY_STATE))
    {
      DisableEnable_SPI_CS();
      SPI_Receive_Manager(SPI_REQUEST_VALID_HEADER_FOR_TX);	/**< BlueNRG not ready for writing */
    }
    else
    {
#if (TEST_TX_BUFFER_LIMITATION == 1)
      if(byte_count > MAX_TX_BUFFER_SIZE)
      {
        byte_count = MAX_TX_BUFFER_SIZE;
      }
#endif
      
      if(SPI_Context.SPI_Transmit_Context.packet_cont != TRUE)
      {
        if( byte_count < (SPI_Context.SPI_Transmit_Context.header_size + SPI_Context.SPI_Transmit_Context.payload_size))
        {
          SPI_Context.SPI_Transmit_Context.payload_size_to_transmit = byte_count - SPI_Context.SPI_Transmit_Context.header_size;
          SPI_Context.SPI_Transmit_Context.payload_size -= SPI_Context.SPI_Transmit_Context.payload_size_to_transmit;
          SPI_Context.SPI_Transmit_Context.packet_cont = TRUE;
        }
        else
        {
          SPI_Context.SPI_Transmit_Context.payload_size_to_transmit = SPI_Context.SPI_Transmit_Context.payload_size;
        }
        
        SPI_Transmit_Manager(SPI_HEADER_TRANSMIT);
      }
      else
      {
        if( byte_count < SPI_Context.SPI_Transmit_Context.payload_size)
        {
          SPI_Context.SPI_Transmit_Context.payload_size_to_transmit = byte_count;
          SPI_Context.SPI_Transmit_Context.payload_size -= SPI_Context.SPI_Transmit_Context.payload_size_to_transmit;
        }
        else
        {
          SPI_Context.SPI_Transmit_Context.payload_size_to_transmit = SPI_Context.SPI_Transmit_Context.payload_size;
          SPI_Context.SPI_Transmit_Context.payload_size = 0;
        }
        
        SPI_Transmit_Manager(SPI_PAYLOAD_TRANSMIT);
      }
    }
    
    break;
    
  default:
    break;
  }
}

#ifdef ENABLE_SPI_FIX
/**
 * @brief  Wakeup BlueNRG
 * @param  None
 * @retval None
 */
static void WakeupBlueNRG(void)
{
  Disable_SPI_Receiving_Path();
  pTimerTxRxCallback = TimerTransmitCallback;
  set_irq_as_output();
  TIMER_Start(StartupTimerId, SPI_FIX_TIMEOUT);
  TIMER_Start(TxRxTimerId, SPI_FIX_TIMEOUT+SPI_TX_TIMEOUT);
  LPM_Mode_Request(eLPM_SPI_TX, eLPM_Mode_LP_Stop);
    
  return;
}

/**
 * @brief  Timer callback to apply timeout SPI FIX
 * @param  None
 * @retval None
 */
static void TimerStartupCallback(void)
{
  Enable_SPI_CS();
  
  return;
}

#else
/**
 * @brief  Wakeup BlueNRG
 * @param  None
 * @retval None
 */
static void WakeupBlueNRG(void)
{
  Disable_SPI_Receiving_Path();
  pTimerTxRxCallback = TimerTransmitCallback;
  Enable_SPI_CS();
  TIMER_Start(TxRxTimerId, SPI_TX_TIMEOUT);
  LPM_Mode_Request(eLPM_SPI_TX, eLPM_Mode_LP_Stop);
  
  return;
}
#endif /* ENABLE_SPI_FIX */

/**
 * @brief  Tx and Rx Transfer completed callbacks
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval None
 */
void BlueNRG_DMA_TxCallback(void)
{
  __HAL_DMA_CLEAR_FLAG(SPI_Context.hspi->hdmatx, BNRG_SPI_TX_DMA_TC_FLAG);
  
  switch (SPI_Context.SPI_Transmit_Context.Spi_Transmit_Event)
  {
  case SPI_HEADER_TRANSMITTED:
    if(SPI_Context.SPI_Transmit_Context.payload_size_to_transmit != 0)
    {
      SPI_Transmit_Manager(SPI_PAYLOAD_TRANSMIT);
    }
    else
    {
      TransmitClosure();
    }
    break;
    
  case SPI_PAYLOAD_TRANSMITTED:
    if( (SPI_Context.SPI_Transmit_Context.packet_cont == TRUE) && (SPI_Context.SPI_Transmit_Context.payload_size != 0))
    {
      SPI_Context.SPI_Transmit_Context.payload_data += SPI_Context.SPI_Transmit_Context.payload_size_to_transmit;
      DisableEnable_SPI_CS();
      SPI_Receive_Manager(SPI_REQUEST_VALID_HEADER_FOR_TX);
    }
    else
    {
      TransmitClosure();
    }
    break;
    
  default:
    break;
  }
  
  return;
}

/**
 * @brief  Close the transmit mechanism after packet has been sent
 *         Wait for the event to come back
 * @param  None
 * @retval None
 */
static void TransmitClosure(void)
{ 
  LPM_Mode_Request(eLPM_SPI_TX, eLPM_Mode_LP_Stop);
  SPI_Context.Spi_Peripheral_State = SPI_AVAILABLE;
  Disable_SPI_CS();
  /*
   *  Disable both DMA
   */
  __HAL_DMA_DISABLE(SPI_Context.hspi->hdmatx);
  __HAL_DMA_DISABLE(SPI_Context.hspi->hdmarx);
  Enable_SPI_Receiving_Path();
  
  return;
}

/**
 * @brief  Manage the SPI transmit
 * @param  TransmitRequest: the transmit request
 * @retval None
 */
static void SPI_Transmit_Manager(SPI_TRANSMIT_REQUEST_t TransmitRequest)
{
  /*
   *  Disable both DMA
   */
  __HAL_DMA_DISABLE(SPI_Context.hspi->hdmatx);
  __HAL_DMA_DISABLE(SPI_Context.hspi->hdmarx);
  
  __HAL_DMA_DISABLE_IT(SPI_Context.hspi->hdmarx, DMA_IT_TC); /**< Disable Receive packet notification */
  
  __HAL_DMA_CLEAR_FLAG(SPI_Context.hspi->hdmatx, BNRG_SPI_TX_DMA_TC_FLAG); /**< Clear flag in DMA */
	HAL_NVIC_ClearPendingIRQ(BNRG_SPI_DMA_TX_IRQn); /**< Clear DMA pending bit in NVIC */
  __HAL_DMA_ENABLE_IT(SPI_Context.hspi->hdmatx, DMA_IT_TC);	/**< Enable Transmit packet notification */
  
  __HAL_BLUENRG_DMA_SET_MINC(SPI_Context.hspi->hdmatx); /**< Configure DMA to send Tx packet */
  
  switch (TransmitRequest)
  {
  case SPI_HEADER_TRANSMIT:
    SPI_Context.SPI_Transmit_Context.Spi_Transmit_Event = SPI_HEADER_TRANSMITTED;
    
#ifdef ENABLE_SPI_FIX
    set_irq_as_input();
#endif
    
    __HAL_BLUENRG_DMA_SET_COUNTER(SPI_Context.hspi->hdmatx, SPI_Context.SPI_Transmit_Context.header_size); /**< Set counter in DMA TX */
    __HAL_BLUENRG_DMA_SET_MEMORY_ADDRESS(SPI_Context.hspi->hdmatx, (uint32_t)SPI_Context.SPI_Transmit_Context.header_data); /**< Set memory address in DMA TX */
    break;
    
  case SPI_PAYLOAD_TRANSMIT:
    SPI_Context.SPI_Transmit_Context.Spi_Transmit_Event = SPI_PAYLOAD_TRANSMITTED;
    
    __HAL_BLUENRG_DMA_SET_COUNTER(SPI_Context.hspi->hdmatx, SPI_Context.SPI_Transmit_Context.payload_size_to_transmit); /**< Set counter in DMA TX */
    __HAL_BLUENRG_DMA_SET_MEMORY_ADDRESS(SPI_Context.hspi->hdmatx, (uint32_t)SPI_Context.SPI_Transmit_Context.payload_data); /**< Set memory address in DMA TX */
    break;
    
  default:
    break;
  }
  
  __HAL_DMA_ENABLE(SPI_Context.hspi->hdmatx); /**< Enable DMA TX */
  
}

/**
 * @brief  Manage the SPI receive
 * @param  ReceiveRequest: the receive request
 * @retval None
 */
static void SPI_Receive_Manager(SPI_RECEIVE_REQUEST_t ReceiveRequest)
{
  uint16_t byte_count;
	uint8_t localloop;
  
  /*
   *  Disable both DMA
   */
  __HAL_DMA_DISABLE(SPI_Context.hspi->hdmatx);
  __HAL_DMA_DISABLE(SPI_Context.hspi->hdmarx);
  
  /**
   * Flush the Rx register or FIFO
   */
  for (localloop = 0 ; localloop < SPI_FIFO_RX_DEPTH ; localloop++)
  {
    *(volatile uint8_t*)__HAL_BLUENRG_SPI_GET_RX_DATA_REGISTER_ADDRESS(SPI_Context.hspi);
  }
  
  __HAL_DMA_ENABLE_IT(SPI_Context.hspi->hdmarx, DMA_IT_TC);	/**< Enable Receive packet notification */
  __HAL_DMA_DISABLE_IT(SPI_Context.hspi->hdmatx, DMA_IT_TC); /**< Disable Transmit packet notification */
  
  switch (ReceiveRequest)
  {
  case SPI_REQUEST_VALID_HEADER_FOR_RX:
    ReceiveHeader(SPI_CHECK_RECEIVED_HEADER_FOR_RX, (uint8_t *)Read_Header_CMD);
    break;
    
  case SPI_REQUEST_VALID_HEADER_FOR_TX:
    ReceiveHeader(SPI_CHECK_RECEIVED_HEADER_FOR_TX, (uint8_t *)Write_Header_CMD);
    break;
    
  case SPI_REQUEST_PAYLOAD:
    SPI_Context.SPI_Receive_Context.Spi_Receive_Event = SPI_RECEIVE_END;
    
    /*
     * Check data to received is not available buffer size
     */
    byte_count = (Received_Header[4]<<8)|Received_Header[3];
    if (byte_count > SPI_Context.SPI_Receive_Context.buffer_size)
    {
      byte_count = SPI_Context.SPI_Receive_Context.buffer_size;
    }
    SPI_Context.SPI_Receive_Context.payload_len = byte_count;
    
    __HAL_BLUENRG_DMA_CLEAR_MINC(SPI_Context.hspi->hdmatx); /**< Configure DMA to send same Byte */
    
    /*
     *  Set counter in both DMA
     */
    __HAL_BLUENRG_DMA_SET_COUNTER(SPI_Context.hspi->hdmarx, byte_count);
    __HAL_BLUENRG_DMA_SET_COUNTER(SPI_Context.hspi->hdmatx, byte_count);
    
    /*
     *  Set memory address in both DMA
     */
    __HAL_BLUENRG_DMA_SET_MEMORY_ADDRESS(SPI_Context.hspi->hdmarx, (uint32_t)SPI_Context.SPI_Receive_Context.buffer);
    __HAL_BLUENRG_DMA_SET_MEMORY_ADDRESS(SPI_Context.hspi->hdmatx, (uint32_t)&dummy_bytes);
    break;
    
  default:
    break;
  }
  
  /*
   *  Enable both DMA - Rx First
   */
  __HAL_DMA_ENABLE(SPI_Context.hspi->hdmarx);
  __HAL_DMA_ENABLE(SPI_Context.hspi->hdmatx);
  
  return;
}

/**
 * @brief Receive header
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval None
 */
static void ReceiveHeader(SPI_RECEIVE_EVENT_t ReceiveEvent, uint8_t * DataHeader)
{
  SPI_Context.SPI_Receive_Context.Spi_Receive_Event = ReceiveEvent;
  
  __HAL_BLUENRG_DMA_SET_MINC(SPI_Context.hspi->hdmatx);	/**< Configure DMA to send Tx packet */
  
  /*
   *  Set counter in both DMA
   */
  __HAL_BLUENRG_DMA_SET_COUNTER(SPI_Context.hspi->hdmatx, HEADER_SIZE);
  __HAL_BLUENRG_DMA_SET_COUNTER(SPI_Context.hspi->hdmarx, HEADER_SIZE);
  
  /*
   *  Set memory address in both DMA
   */
  __HAL_BLUENRG_DMA_SET_MEMORY_ADDRESS(SPI_Context.hspi->hdmarx, (uint32_t)Received_Header);
  __HAL_BLUENRG_DMA_SET_MEMORY_ADDRESS(SPI_Context.hspi->hdmatx, (uint32_t)DataHeader);
  
  return;
}

/**
 * @brief  BlueNRG SPI request event
 * @param  buffer: the event
 * @param  buff_size: the event size
 * @retval None
 */
void BlueNRG_SPI_Request_Events(uint8_t *buffer, uint8_t buff_size)
{
  SPI_Context.SPI_Receive_Context.buffer = buffer;
  SPI_Context.SPI_Receive_Context.buffer_size = buff_size;
  
  Enable_SPI_Receiving_Path();
  
  return;
}

/**
 * @brief  BlueNRG SPI IRQ Callback
 * @param  None
 * @retval None
 */
void BlueNRG_SPI_IRQ_Callback(void)
{  
  __disable_irq();
  if(SPI_Context.Spi_Peripheral_State == SPI_AVAILABLE)
  {
    SPI_Context.Spi_Peripheral_State = SPI_BUSY;
    __enable_irq();
    Enable_SPI_CS();
    SPI_Receive_Manager(SPI_REQUEST_VALID_HEADER_FOR_RX);
    LPM_Mode_Request(eLPM_SPI_RX, eLPM_Mode_Sleep);
  }
  else
  {
    __enable_irq();
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

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
