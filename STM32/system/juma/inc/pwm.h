#ifndef _PWM_H_
#define _PWM_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

HAL_StatusTypeDef TIM_PWM_Init(TIM_HandleTypeDef *htim,  uint32_t Channel,GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint32_t Freq,uint32_t Pulse);

HAL_StatusTypeDef TIM_PWM_Duty(TIM_HandleTypeDef *htim,  uint32_t Channel,uint32_t Pulse);

#endif //_PWM_H_
