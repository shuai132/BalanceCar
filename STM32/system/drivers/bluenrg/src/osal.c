/**
******************************************************************************
* @file    osal.c 
* @author  AMS - HEA&RF BU / CL
* @version V1.0.0
* @date    04-July-2014
* @brief   Implementation of OS abstraction layer functions used by the
*          library.
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
#include <string.h>
#include <osal.h>
 
 /**
 * @brief  Osal_MemCpy
 * @param  dest: Pointer to the destination buffer
 * @param  src : Pointer to the source buffer
 * @param  size: Number of bytes to copy from the source to the destination
 *               buffer
 * @retval Pointer to the destination buffer
 */
void* Osal_MemCpy(void *dest, const void *src, unsigned int size)
{
    return(memcpy(dest,src,size)); 
}

/**
 * @brief  Osal_MemSet
 * @param  ptr  : Pointer to block of memory to fill  
 * @param  value: Value to assign to each byte of the memory block
 * @param  size : Number of bytes to be set to "value"
 * @retval Pointer to the filled block of memory
 */
void* Osal_MemSet(void *ptr, int value, unsigned int size)
{
    return(memset(ptr,value,size));
}

/******************************************************************************
 * local Functions
 *****************************************************************************/ 
 
 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
