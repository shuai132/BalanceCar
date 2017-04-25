/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : osal.h
* Author             : AMS - HEA&RF BU
* Version            : V1.0.0
* Date               : 19-July-2012
* Description        : This header file defines the OS abstraction layer used by
*                      the BLE stack. OSAL defines the set of functions
*                      which needs to be ported to target operating system and
*                      target platform.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __OSAL_H__
#define __OSAL_H__

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <hal_types.h>
#ifdef __ICCARM__
#include <intrinsics.h>
#endif

/******************************************************************************
 * Macros
 *****************************************************************************/


/******************************************************************************
 * Function Prototypes
 *****************************************************************************/

/**
 * This function copies size number of bytes from a 
 * memory location pointed by src to a destination 
 * memory location pointed by dest
 * 
 * @param[in] dest Destination address
 * @param[in] src  Source address
 * @param[in] size size in the bytes  
 * 
 * @return  Address of the destination
 */
 
extern void* Osal_MemCpy(void *dest,const void *src, unsigned int size);


/**
 * This function sets first number of bytes, specified
 * by size, to the destination memory pointed by ptr 
 * to the specified value
 * 
 * @param[in] ptr    Destination address
 * @param[in] value  Value to be set
 * @param[in] size   Size in the bytes  
 * 
 * @return  Address of the destination
 */
 
extern void* Osal_MemSet(void *ptr, int value, unsigned int size);

/**
 * Osal_Get_Cur_Time
 * 
 * returns the current time in milliseconds
 */
/**
 * Returns the number of ticks (1 tick = 1 millisecond)
 * 
 * @return  Time in milliseconds
 */
uint32_t Osal_Get_Cur_Time(void);


#endif /* __OSAL_H__ */
