#include "function.h"

FATFS SDFatFs;  /* File system object for SD disk logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD disk logical drive path */ 


TIM_HandleTypeDef        TimHandleT1;//编码器
TIM_HandleTypeDef        TimHandleT2;//舵机
TIM_HandleTypeDef        TimHandleT3;//舵机
TIM_HandleTypeDef        TimHandleT4;//电机
TIM_HandleTypeDef        TimHandleT5;//编码器
TIM_HandleTypeDef        TimHandleT9;//舵机
TIM_OC_InitTypeDef       pwmConfig;//PWM控制
/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle;

SD_HandleTypeDef hsd;

extern	void Error_Handler(void);

void Adc_Init(){
    ADC_ChannelConfTypeDef sConfig;
    GPIO_InitTypeDef          GPIO_InitStruct;

    /*##-- Enable peripherals and GPIO Clocks #################################*/
    /* ADC3 Periph clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
    /* Enable GPIO clock ****************************************/
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*##-- Configure peripheral GPIO ##########################################*/
    /* ADC3 Channel8 GPIO pin configuration */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    AdcHandle.Instance          = ADC1;

    AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
    AdcHandle.Init.ScanConvMode          = DISABLE;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.NbrOfDiscConversion   = 0;
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.NbrOfConversion       = 1;
    AdcHandle.Init.DMAContinuousRequests = DISABLE;
    AdcHandle.Init.EOCSelection          = DISABLE;

    if(HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-- Configure ADC regular channel ######################################*/
    sConfig.Channel      = ADC_CHANNEL_13;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    sConfig.Offset       = 0;

    if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
    {
        /* Channel Configuration Error */
        Error_Handler();
    }

}

void Get_Adc(uint32_t *Adc){
    static uint32_t pre_Adc = 0;
    static int isFirst = 1;
    uint32_t now_Adc;
    /*##-- Start the conversion process #######################################*/
    if(HAL_ADC_Start(&AdcHandle) != HAL_OK)
    {
        /* Start Conversation Error */
        Error_Handler();
    }
    /*##- Wait for the end of conversion #####################################*/
    HAL_ADC_PollForConversion(&AdcHandle, 10);
    /* Check if the continuous conversion of regular channel is finished */
    if((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
    {
        /*##-5- Get the converted value of regular channel  ######################*/
        now_Adc = HAL_ADC_GetValue(&AdcHandle);
        if(isFirst) {
            isFirst = 0;
        }
        else {
            now_Adc=now_Adc*0.1+pre_Adc*0.9;
        }
        pre_Adc=now_Adc;
        *Adc =now_Adc;
    }
}

void SD_Init(void){	

    if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
    {
        if(f_mount(&SDFatFs,(TCHAR const*)SDPath, 0) != FR_OK)
        {
            Error_Handler();
        }
        else
        {
            if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS  | FA_WRITE) != FR_OK)
            {
                Error_Handler();
            }
            else
            {
                f_puts("First string in my file\n", &MyFile);
                /*
          res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
          if((byteswritten == 0) || (res != FR_OK))
          {
            Error_Handler();
          }
          else
          {
                                                */
                f_close(&MyFile);
                BSP_LED_On(LED0);
                BSP_LED_Toggle(LED0);
                HAL_Delay(100);
                BSP_LED_Toggle(LED0);
                HAL_Delay(100);
                BSP_LED_Toggle(LED0);
                HAL_Delay(100);
                BSP_LED_Toggle(LED0);
                HAL_Delay(100);
                BSP_LED_Toggle(LED0);
                HAL_Delay(100);
                BSP_LED_Toggle(LED0);
                HAL_Delay(100);
                BSP_LED_Toggle(LED0);
                //   }
            }
        }
    }
    FATFS_UnLinkDriver(SDPath);
}

/**
  * @brief  编码器初始化.
  * @param  None
  * @retval None
  */
void Encoder_Init(void){
    //A8 A9引脚定义
    GPIO_InitTypeDef   GPIO_InitStruct;
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //A0 A1引脚定义
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //设置TIM1为编码器读数功能
    TIM_Encoder_InitTypeDef  encoderConfig;
    TimHandleT1.Instance = TIM1;
    TimHandleT1.Init.Period =  0xFFFF;
    TimHandleT1.Init.Prescaler = 0;
    TimHandleT1.Init.ClockDivision = 0;
    TimHandleT1.Init.CounterMode = TIM_COUNTERMODE_UP;
    encoderConfig.EncoderMode =TIM_ENCODERMODE_TI12;
    encoderConfig.IC1Polarity =TIM_ICPOLARITY_RISING;
    encoderConfig.IC1Selection=TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC1Prescaler=0;
    encoderConfig.IC1Filter   =6;
    encoderConfig.IC2Polarity =TIM_ICPOLARITY_RISING;
    encoderConfig.IC2Selection=TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC2Prescaler=0;
    encoderConfig.IC2Filter   =6;
    HAL_TIM_Encoder_Init(&TimHandleT1,  &encoderConfig);
    HAL_TIM_Encoder_Start(&TimHandleT1,TIM_CHANNEL_1);
    //设置TIM5为编码器读数功能
    TimHandleT5.Instance = TIM5;
    TimHandleT5.Init.Period =  0xFFFF;
    TimHandleT5.Init.Prescaler = 0;
    TimHandleT5.Init.ClockDivision = 0;
    TimHandleT5.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_Encoder_Init(&TimHandleT5,  &encoderConfig);
    HAL_TIM_Encoder_Start(&TimHandleT5,TIM_CHANNEL_1);
}

/**
  * @brief  初始化电机PWM.
  * @param  None
  * @retval None
  */
void Motor_Pwm_Init(void){
    //PWM引脚
    GPIO_InitTypeDef   GPIO_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct2;
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    //TIM4

    TimHandleT4.Instance = TIM4;
    TimHandleT4.Init.Period =  1000 - 1;;
    TimHandleT4.Init.Prescaler = 10-1;
    TimHandleT4.Init.ClockDivision = 0;
    TimHandleT4.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandleT4);

    pwmConfig.OCMode=TIM_OCMODE_PWM1;
    pwmConfig.Pulse=0;
    HAL_TIM_PWM_ConfigChannel(&TimHandleT4, &pwmConfig, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&TimHandleT4, &pwmConfig, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&TimHandleT4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TimHandleT4, TIM_CHANNEL_4);

    //电机控制引脚/*
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct2.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct2.Pull      = GPIO_NOPULL;
    GPIO_InitStruct2.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct2.Pin     =GPIO_PIN_5|GPIO_PIN_13;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct2);

}

int16_t M1_Pulse_Max = 500;
int16_t M2_Pulse_Max = 500;
void Motor_Control_1(int16_t Pulse){
    if(Pulse>=0){
        if(Pulse>1000)Pulse=M1_Pulse_Max;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_TIM_PWM_Pulse(&TimHandleT4,TIM_CHANNEL_4,Pulse);
    }
    else{
        if(Pulse<-1000)Pulse=-M1_Pulse_Max;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_TIM_PWM_Pulse(&TimHandleT4,TIM_CHANNEL_4,-Pulse);
    }
}

void Motor_Control_2(int16_t Pulse){
    if(Pulse>=0){
        if(Pulse>1000)Pulse=M2_Pulse_Max;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_TIM_PWM_Pulse(&TimHandleT4,TIM_CHANNEL_3,Pulse);
    }
    else{
        if(Pulse<-1000)Pulse=-M2_Pulse_Max;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );
        HAL_TIM_PWM_Pulse(&TimHandleT4,TIM_CHANNEL_3,-Pulse);
    }
}

/**
  * @brief  初始化舵机PWM.
  * @param  None
  * @retval None
  */
void Steer_Pwm_Init(void){
    //B0引脚定义
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    //C7引脚定义
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    //TIM3
    TimHandleT3.Instance = TIM3;
    TimHandleT3.Init.Period =  1000 - 1;;
    TimHandleT3.Init.Prescaler = 280-1;
    TimHandleT3.Init.ClockDivision = 0;
    TimHandleT3.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandleT3);

    pwmConfig.OCMode=TIM_OCMODE_PWM1;
    pwmConfig.Pulse=450;
    HAL_TIM_PWM_ConfigChannel(&TimHandleT3, &pwmConfig, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&TimHandleT3, &pwmConfig, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TimHandleT3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TimHandleT3, TIM_CHANNEL_2);
}

/**
  * @brief  获得两次时间间隔中的编码器读数差.
  * @param  None
  * @retval None
  */
void Get_Speed(int32_t *speedL,int32_t *speedR,float *speedA){
    static	int32_t Pre_Speed_L;
    static int32_t Pre_Speed_R;
    static	float Pre_Speed_A;
    uint32_t TempL,TempR;
    int32_t Speed_L,Speed_R,Speed_A;
    TempL=HAL_TIM_ReadCapturedValue(&TimHandleT1, TIM_CHANNEL_1);//编码器读取
    //  Speed_L=TempL;
    Speed_L=TempL-Pre_Speed_L;
    Pre_Speed_L=TempL;
    if(Speed_L<-20000){
        Speed_L+=65535;}
    else   if(Speed_L>20000){
        Speed_L-=65535;}

    TempR=HAL_TIM_ReadCapturedValue(&TimHandleT5, TIM_CHANNEL_1);//编码器读取

    //	Speed_R=TempR;
    Speed_R=TempR-Pre_Speed_R;
    Pre_Speed_R=TempR;
    if(Speed_R<-20000){
        Speed_R+=65535;}
    else   if(Speed_R>20000){
        Speed_R-=65535;}
    //一阶低通滤波
    Speed_A=(Speed_R-Speed_L)/2.0*0.7+Pre_Speed_A*0.3;
    Pre_Speed_A=Speed_A;

    *speedL=-Speed_L;
    *speedR=Speed_R;
    *speedA=Speed_A;
}

/**
  * @brief  set PWM duty cycle.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIM_PWM_Pulse(TIM_HandleTypeDef *htim,  uint32_t Channel,uint32_t Pulse){
    switch (Channel)
    {
    case TIM_CHANNEL_1:
    {
        htim->Instance->CCR1 =Pulse;
    }break;
    case TIM_CHANNEL_2:
    {
        htim->Instance->CCR2 =Pulse;
    }break;
    case TIM_CHANNEL_3:
    {
        htim->Instance->CCR3 =Pulse;
    }break;
    case TIM_CHANNEL_4:
    {
        htim->Instance->CCR4 =Pulse;
    }break;
    default:
        break;
    }
    return HAL_OK;
}
