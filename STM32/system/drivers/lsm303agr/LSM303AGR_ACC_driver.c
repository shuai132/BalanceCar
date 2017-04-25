/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : LSM303AGR_ACC_driver.c
* Author             : MSH Application Team
* Version            : v1.00
* Date               : 28/Apr/2015
* Description        : LSM303AGR ACC driver source file
*
* Reviewed by: Armando Visconti
*-------------------------------------------------------------------------------
*
*
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "LSM303AGR_ACC_driver.h"
#include "stm32f4xx_hal.h"
#include "x_nucleo_iks01a1.h"
//#include "i2C_mems.h"												//[Example]

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name		: SwapHighLowByte
* Description		: Swap High/low byte in multiple byte values
*                     It works with minimum 2 byte for every dimension.
*                     Example x,y,z with 2 byte for every dimension
*
* Input				: bufferToSwap -> buffer to swap
*                     numberOfByte -> the buffer length in byte
*                     dimension -> number of dimension
*
* Output			: bufferToSwap -> buffer swapped
* Return			: None
*******************************************************************************/
void LSM303AGR_ACC_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension)
{

    u8_t numberOfByteForDimension, i, j;
    u8_t tempValue[10];

    numberOfByteForDimension=numberOfByte/dimension;

    for (i=0; i<dimension; i++ )
    {
        for (j=0; j<numberOfByteForDimension; j++ )
            tempValue[j]=bufferToSwap[j+i*numberOfByteForDimension];
        for (j=0; j<numberOfByteForDimension; j++ )
            *(bufferToSwap+i*(numberOfByteForDimension)+j)=*(tempValue+(numberOfByteForDimension-1)-j);
    }
}

/* Exported functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_x_data_avail
* Description    : Read 1DA
* Input          : Pointer to LSM303AGR_ACC_1DA_t
* Output         : Status of 1DA see LSM303AGR_ACC_1DA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_x_data_avail(LSM303AGR_ACC_1DA_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG_AUX, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_1DA_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_y_data_avail
* Description    : Read 2DA_
* Input          : Pointer to LSM303AGR_ACC_2DA__t
* Output         : Status of 2DA_ see LSM303AGR_ACC_2DA__t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_y_data_avail(LSM303AGR_ACC_2DA__t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG_AUX, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_2DA__MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_z_data_avail
* Description    : Read 3DA_
* Input          : Pointer to LSM303AGR_ACC_3DA__t
* Output         : Status of 3DA_ see LSM303AGR_ACC_3DA__t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_z_data_avail(LSM303AGR_ACC_3DA__t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG_AUX, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_3DA__MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_xyz_data_avail
* Description    : Read 321DA_
* Input          : Pointer to LSM303AGR_ACC_321DA__t
* Output         : Status of 321DA_ see LSM303AGR_ACC_321DA__t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_xyz_data_avail(LSM303AGR_ACC_321DA__t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG_AUX, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_321DA__MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_DataXOverrun
* Description    : Read 1OR_
* Input          : Pointer to LSM303AGR_ACC_1OR__t
* Output         : Status of 1OR_ see LSM303AGR_ACC_1OR__t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_DataXOverrun(LSM303AGR_ACC_1OR__t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG_AUX, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_1OR__MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_DataYOverrun
* Description    : Read 2OR_
* Input          : Pointer to LSM303AGR_ACC_2OR__t
* Output         : Status of 2OR_ see LSM303AGR_ACC_2OR__t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_DataYOverrun(LSM303AGR_ACC_2OR__t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG_AUX, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_2OR__MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_DataZOverrun
* Description    : Read 3OR_
* Input          : Pointer to LSM303AGR_ACC_3OR__t
* Output         : Status of 3OR_ see LSM303AGR_ACC_3OR__t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_DataZOverrun(LSM303AGR_ACC_3OR__t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG_AUX, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_3OR__MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_DataXYZOverrun
* Description    : Read 321OR_
* Input          : Pointer to LSM303AGR_ACC_321OR__t
* Output         : Status of 321OR_ see LSM303AGR_ACC_321OR__t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_DataXYZOverrun(LSM303AGR_ACC_321OR__t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG_AUX, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_321OR__MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_int_counter
* Description    : Read IC
* Input          : Pointer to u8_t
* Output         : Status of IC
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_int_counter(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT_COUNTER_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_IC_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_IC_POSITION; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_WHO_AM_I
* Description    : Read WHO_AM_I
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_WHO_AM_I(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_WHO_AM_I_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_WHO_AM_I_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_WHO_AM_I_POSITION; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_TEMP_EN_bits
* Description    : Write TEMP_EN
* Input          : LSM303AGR_ACC_TEMP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_TEMP_EN_bits(LSM303AGR_ACC_TEMP_EN_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_TEMP_CFG_REG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_TEMP_EN_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_TEMP_CFG_REG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_TEMP_EN_bits
* Description    : Read TEMP_EN
* Input          : Pointer to LSM303AGR_ACC_TEMP_EN_t
* Output         : Status of TEMP_EN see LSM303AGR_ACC_TEMP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_TEMP_EN_bits(LSM303AGR_ACC_TEMP_EN_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_TEMP_CFG_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_TEMP_EN_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_ADC_PD
* Description    : Write ADC_PD
* Input          : LSM303AGR_ACC_ADC_PD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_ADC_PD(LSM303AGR_ACC_ADC_PD_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_TEMP_CFG_REG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_ADC_PD_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_TEMP_CFG_REG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ADC_PD
* Description    : Read ADC_PD
* Input          : Pointer to LSM303AGR_ACC_ADC_PD_t
* Output         : Status of ADC_PD see LSM303AGR_ACC_ADC_PD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ADC_PD(LSM303AGR_ACC_ADC_PD_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_TEMP_CFG_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ADC_PD_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_XEN
* Description    : Write XEN
* Input          : LSM303AGR_ACC_XEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_XEN(LSM303AGR_ACC_XEN_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG1, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_XEN_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG1, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_XEN
* Description    : Read XEN
* Input          : Pointer to LSM303AGR_ACC_XEN_t
* Output         : Status of XEN see LSM303AGR_ACC_XEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_XEN(LSM303AGR_ACC_XEN_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG1, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XEN_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_YEN
* Description    : Write YEN
* Input          : LSM303AGR_ACC_YEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_YEN(LSM303AGR_ACC_YEN_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG1, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_YEN_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG1, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_YEN
* Description    : Read YEN
* Input          : Pointer to LSM303AGR_ACC_YEN_t
* Output         : Status of YEN see LSM303AGR_ACC_YEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_YEN(LSM303AGR_ACC_YEN_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG1, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YEN_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_ZEN
* Description    : Write ZEN
* Input          : LSM303AGR_ACC_ZEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_ZEN(LSM303AGR_ACC_ZEN_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG1, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_ZEN_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG1, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ZEN
* Description    : Read ZEN
* Input          : Pointer to LSM303AGR_ACC_ZEN_t
* Output         : Status of ZEN see LSM303AGR_ACC_ZEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ZEN(LSM303AGR_ACC_ZEN_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG1, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZEN_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_LOWPWR_EN
* Description    : Write LPEN
* Input          : LSM303AGR_ACC_LPEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_LOWPWR_EN(LSM303AGR_ACC_LPEN_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG1, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_LPEN_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG1, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_LOWPWR_EN
* Description    : Read LPEN
* Input          : Pointer to LSM303AGR_ACC_LPEN_t
* Output         : Status of LPEN see LSM303AGR_ACC_LPEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_LOWPWR_EN(LSM303AGR_ACC_LPEN_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG1, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_LPEN_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_ODR
* Description    : Write ODR
* Input          : LSM303AGR_ACC_ODR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_ODR(LSM303AGR_ACC_ODR_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG1, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_ODR_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG1, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ODR
* Description    : Read ODR
* Input          : Pointer to LSM303AGR_ACC_ODR_t
* Output         : Status of ODR see LSM303AGR_ACC_ODR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ODR(LSM303AGR_ACC_ODR_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG1, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ODR_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_hpf_aoi_en_int1
* Description    : Write HPIS1
* Input          : LSM303AGR_ACC_HPIS1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_hpf_aoi_en_int1(LSM303AGR_ACC_HPIS1_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_HPIS1_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_hpf_aoi_en_int1
* Description    : Read HPIS1
* Input          : Pointer to LSM303AGR_ACC_HPIS1_t
* Output         : Status of HPIS1 see LSM303AGR_ACC_HPIS1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_hpf_aoi_en_int1(LSM303AGR_ACC_HPIS1_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_HPIS1_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_hpf_aoi_en_int2
* Description    : Write HPIS2
* Input          : LSM303AGR_ACC_HPIS2_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_hpf_aoi_en_int2(LSM303AGR_ACC_HPIS2_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_HPIS2_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_hpf_aoi_en_int2
* Description    : Read HPIS2
* Input          : Pointer to LSM303AGR_ACC_HPIS2_t
* Output         : Status of HPIS2 see LSM303AGR_ACC_HPIS2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_hpf_aoi_en_int2(LSM303AGR_ACC_HPIS2_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_HPIS2_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_hpf_click_en
* Description    : Write HPCLICK
* Input          : LSM303AGR_ACC_HPCLICK_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_hpf_click_en(LSM303AGR_ACC_HPCLICK_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_HPCLICK_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_hpf_click_en
* Description    : Read HPCLICK
* Input          : Pointer to LSM303AGR_ACC_HPCLICK_t
* Output         : Status of HPCLICK see LSM303AGR_ACC_HPCLICK_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_hpf_click_en(LSM303AGR_ACC_HPCLICK_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_HPCLICK_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Data_Filter
* Description    : Write FDS
* Input          : LSM303AGR_ACC_FDS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Data_Filter(LSM303AGR_ACC_FDS_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_FDS_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Data_Filter
* Description    : Read FDS
* Input          : Pointer to LSM303AGR_ACC_FDS_t
* Output         : Status of FDS see LSM303AGR_ACC_FDS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Data_Filter(LSM303AGR_ACC_FDS_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_FDS_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_hpf_cutoff_freq
* Description    : Write HPCF
* Input          : LSM303AGR_ACC_HPCF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_hpf_cutoff_freq(LSM303AGR_ACC_HPCF_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_HPCF_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_hpf_cutoff_freq
* Description    : Read HPCF
* Input          : Pointer to LSM303AGR_ACC_HPCF_t
* Output         : Status of HPCF see LSM303AGR_ACC_HPCF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_hpf_cutoff_freq(LSM303AGR_ACC_HPCF_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_HPCF_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_hpf_mode
* Description    : Write HPM
* Input          : LSM303AGR_ACC_HPM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_hpf_mode(LSM303AGR_ACC_HPM_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_HPM_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG2, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_hpf_mode
* Description    : Read HPM
* Input          : Pointer to LSM303AGR_ACC_HPM_t
* Output         : Status of HPM see LSM303AGR_ACC_HPM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_hpf_mode(LSM303AGR_ACC_HPM_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_HPM_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FIFO_Overrun_on_INT1
* Description    : Write I1_OVERRUN
* Input          : LSM303AGR_ACC_I1_OVERRUN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FIFO_Overrun_on_INT1(LSM303AGR_ACC_I1_OVERRUN_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_I1_OVERRUN_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG3, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FIFO_Overrun_on_INT1
* Description    : Read I1_OVERRUN
* Input          : Pointer to LSM303AGR_ACC_I1_OVERRUN_t
* Output         : Status of I1_OVERRUN see LSM303AGR_ACC_I1_OVERRUN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FIFO_Overrun_on_INT1(LSM303AGR_ACC_I1_OVERRUN_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_I1_OVERRUN_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FIFO_Watermark_on_INT1
* Description    : Write I1_WTM
* Input          : LSM303AGR_ACC_I1_WTM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FIFO_Watermark_on_INT1(LSM303AGR_ACC_I1_WTM_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_I1_WTM_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG3, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FIFO_Watermark_on_INT1
* Description    : Read I1_WTM
* Input          : Pointer to LSM303AGR_ACC_I1_WTM_t
* Output         : Status of I1_WTM see LSM303AGR_ACC_I1_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FIFO_Watermark_on_INT1(LSM303AGR_ACC_I1_WTM_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_I1_WTM_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FIFO_DRDY2_on_INT1
* Description    : Write I1_DRDY2
* Input          : LSM303AGR_ACC_I1_DRDY2_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FIFO_DRDY2_on_INT1(LSM303AGR_ACC_I1_DRDY2_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_I1_DRDY2_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG3, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FIFO_DRDY2_on_INT1
* Description    : Read I1_DRDY2
* Input          : Pointer to LSM303AGR_ACC_I1_DRDY2_t
* Output         : Status of I1_DRDY2 see LSM303AGR_ACC_I1_DRDY2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FIFO_DRDY2_on_INT1(LSM303AGR_ACC_I1_DRDY2_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_I1_DRDY2_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FIFO_DRDY1_on_INT1
* Description    : Write I1_DRDY1
* Input          : LSM303AGR_ACC_I1_DRDY1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FIFO_DRDY1_on_INT1(LSM303AGR_ACC_I1_DRDY1_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_I1_DRDY1_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG3, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FIFO_DRDY1_on_INT1
* Description    : Read I1_DRDY1
* Input          : Pointer to LSM303AGR_ACC_I1_DRDY1_t
* Output         : Status of I1_DRDY1 see LSM303AGR_ACC_I1_DRDY1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FIFO_DRDY1_on_INT1(LSM303AGR_ACC_I1_DRDY1_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_I1_DRDY1_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FIFO_AOL2_on_INT1
* Description    : Write I1_AOI2
* Input          : LSM303AGR_ACC_I1_AOI2_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FIFO_AOL2_on_INT1(LSM303AGR_ACC_I1_AOI2_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_I1_AOI2_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG3, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FIFO_AOL2_on_INT1
* Description    : Read I1_AOI2
* Input          : Pointer to LSM303AGR_ACC_I1_AOI2_t
* Output         : Status of I1_AOI2 see LSM303AGR_ACC_I1_AOI2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FIFO_AOL2_on_INT1(LSM303AGR_ACC_I1_AOI2_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_I1_AOI2_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FIFO_AOL1_on_INT1
* Description    : Write I1_AOI1
* Input          : LSM303AGR_ACC_I1_AOI1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FIFO_AOL1_on_INT1(LSM303AGR_ACC_I1_AOI1_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_I1_AOI1_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG3, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FIFO_AOL1_on_INT1
* Description    : Read I1_AOI1
* Input          : Pointer to LSM303AGR_ACC_I1_AOI1_t
* Output         : Status of I1_AOI1 see LSM303AGR_ACC_I1_AOI1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FIFO_AOL1_on_INT1(LSM303AGR_ACC_I1_AOI1_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_I1_AOI1_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FIFO_Click_on_INT1
* Description    : Write I1_CLICK
* Input          : LSM303AGR_ACC_I1_CLICK_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FIFO_Click_on_INT1(LSM303AGR_ACC_I1_CLICK_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_I1_CLICK_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG3, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FIFO_Click_on_INT1
* Description    : Read I1_CLICK
* Input          : Pointer to LSM303AGR_ACC_I1_CLICK_t
* Output         : Status of I1_CLICK see LSM303AGR_ACC_I1_CLICK_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FIFO_Click_on_INT1(LSM303AGR_ACC_I1_CLICK_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG3, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_I1_CLICK_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_SPI_mode
* Description    : Write SIM
* Input          : LSM303AGR_ACC_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_SPI_mode(LSM303AGR_ACC_SIM_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_SIM_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_SPI_mode
* Description    : Read SIM
* Input          : Pointer to LSM303AGR_ACC_SIM_t
* Output         : Status of SIM see LSM303AGR_ACC_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_SPI_mode(LSM303AGR_ACC_SIM_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_SIM_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_SelfTest
* Description    : Write ST
* Input          : LSM303AGR_ACC_ST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_SelfTest(LSM303AGR_ACC_ST_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_ST_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_SelfTest
* Description    : Read ST
* Input          : Pointer to LSM303AGR_ACC_ST_t
* Output         : Status of ST see LSM303AGR_ACC_ST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_SelfTest(LSM303AGR_ACC_ST_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ST_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_HiRes
* Description    : Write HR
* Input          : LSM303AGR_ACC_HR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_HiRes(LSM303AGR_ACC_HR_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_HR_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_HiRes
* Description    : Read HR
* Input          : Pointer to LSM303AGR_ACC_HR_t
* Output         : Status of HR see LSM303AGR_ACC_HR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_HiRes(LSM303AGR_ACC_HR_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_HR_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FullScale
* Description    : Write FS
* Input          : LSM303AGR_ACC_FS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FullScale(LSM303AGR_ACC_FS_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_FS_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FullScale
* Description    : Read FS
* Input          : Pointer to LSM303AGR_ACC_FS_t
* Output         : Status of FS see LSM303AGR_ACC_FS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FullScale(LSM303AGR_ACC_FS_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_FS_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_LittleBigEndian
* Description    : Write BLE
* Input          : LSM303AGR_ACC_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_LittleBigEndian(LSM303AGR_ACC_BLE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_BLE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_LittleBigEndian
* Description    : Read BLE
* Input          : Pointer to LSM303AGR_ACC_BLE_t
* Output         : Status of BLE see LSM303AGR_ACC_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_LittleBigEndian(LSM303AGR_ACC_BLE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_BLE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_BlockDataUpdate
* Description    : Write BDU
* Input          : LSM303AGR_ACC_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_BlockDataUpdate(LSM303AGR_ACC_BDU_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_BDU_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG4, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_BlockDataUpdate
* Description    : Read BDU
* Input          : Pointer to LSM303AGR_ACC_BDU_t
* Output         : Status of BDU see LSM303AGR_ACC_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_BlockDataUpdate(LSM303AGR_ACC_BDU_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG4, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_BDU_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_4D_on_INT2
* Description    : Write D4D_INT2
* Input          : LSM303AGR_ACC_D4D_INT2_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_4D_on_INT2(LSM303AGR_ACC_D4D_INT2_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_D4D_INT2_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG5, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_4D_on_INT2
* Description    : Read D4D_INT2
* Input          : Pointer to LSM303AGR_ACC_D4D_INT2_t
* Output         : Status of D4D_INT2 see LSM303AGR_ACC_D4D_INT2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_4D_on_INT2(LSM303AGR_ACC_D4D_INT2_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_D4D_INT2_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_LatchInterrupt_on_INT2
* Description    : Write LIR_INT2
* Input          : LSM303AGR_ACC_LIR_INT2_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_LatchInterrupt_on_INT2(LSM303AGR_ACC_LIR_INT2_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_LIR_INT2_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG5, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_LatchInterrupt_on_INT2
* Description    : Read LIR_INT2
* Input          : Pointer to LSM303AGR_ACC_LIR_INT2_t
* Output         : Status of LIR_INT2 see LSM303AGR_ACC_LIR_INT2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_LatchInterrupt_on_INT2(LSM303AGR_ACC_LIR_INT2_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_LIR_INT2_MASK; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_4D_on_INT1
* Description    : Write D4D_INT1
* Input          : LSM303AGR_ACC_D4D_INT1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_4D_on_INT1(LSM303AGR_ACC_D4D_INT1_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_D4D_INT1_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG5, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_4D_on_INT1
* Description    : Read D4D_INT1
* Input          : Pointer to LSM303AGR_ACC_D4D_INT1_t
* Output         : Status of D4D_INT1 see LSM303AGR_ACC_D4D_INT1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_4D_on_INT1(LSM303AGR_ACC_D4D_INT1_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_D4D_INT1_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_LatchInterrupt_on_INT1
* Description    : Write LIR_INT1
* Input          : LSM303AGR_ACC_LIR_INT1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_LatchInterrupt_on_INT1(LSM303AGR_ACC_LIR_INT1_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_LIR_INT1_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG5, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_LatchInterrupt_on_INT1
* Description    : Read LIR_INT1
* Input          : Pointer to LSM303AGR_ACC_LIR_INT1_t
* Output         : Status of LIR_INT1 see LSM303AGR_ACC_LIR_INT1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_LatchInterrupt_on_INT1(LSM303AGR_ACC_LIR_INT1_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_LIR_INT1_MASK; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FIFO_EN
* Description    : Write FIFO_EN
* Input          : LSM303AGR_ACC_FIFO_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FIFO_EN(LSM303AGR_ACC_FIFO_EN_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_FIFO_EN_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG5, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FIFO_EN
* Description    : Read FIFO_EN
* Input          : Pointer to LSM303AGR_ACC_FIFO_EN_t
* Output         : Status of FIFO_EN see LSM303AGR_ACC_FIFO_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FIFO_EN(LSM303AGR_ACC_FIFO_EN_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_FIFO_EN_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_RebootMemory
* Description    : Write BOOT
* Input          : LSM303AGR_ACC_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_RebootMemory(LSM303AGR_ACC_BOOT_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_BOOT_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG5, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_RebootMemory
* Description    : Read BOOT
* Input          : Pointer to LSM303AGR_ACC_BOOT_t
* Output         : Status of BOOT see LSM303AGR_ACC_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_RebootMemory(LSM303AGR_ACC_BOOT_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG5, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_BOOT_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_IntActive
* Description    : Write H_LACTIVE
* Input          : LSM303AGR_ACC_H_LACTIVE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_IntActive(LSM303AGR_ACC_H_LACTIVE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_H_LACTIVE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG6, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_IntActive
* Description    : Read H_LACTIVE
* Input          : Pointer to LSM303AGR_ACC_H_LACTIVE_t
* Output         : Status of H_LACTIVE see LSM303AGR_ACC_H_LACTIVE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_IntActive(LSM303AGR_ACC_H_LACTIVE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_H_LACTIVE_MASK; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_P2_ACT
* Description    : Write P2_ACT
* Input          : LSM303AGR_ACC_P2_ACT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_P2_ACT(LSM303AGR_ACC_P2_ACT_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_P2_ACT_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG6, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_P2_ACT
* Description    : Read P2_ACT
* Input          : Pointer to LSM303AGR_ACC_P2_ACT_t
* Output         : Status of P2_ACT see LSM303AGR_ACC_P2_ACT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_P2_ACT(LSM303AGR_ACC_P2_ACT_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_P2_ACT_MASK; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Boot_on_INT2
* Description    : Write BOOT_I1
* Input          : LSM303AGR_ACC_BOOT_I1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Boot_on_INT2(LSM303AGR_ACC_BOOT_I1_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_BOOT_I1_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG6, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Boot_on_INT2
* Description    : Read BOOT_I1
* Input          : Pointer to LSM303AGR_ACC_BOOT_I1_t
* Output         : Status of BOOT_I1 see LSM303AGR_ACC_BOOT_I1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Boot_on_INT2(LSM303AGR_ACC_BOOT_I1_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_BOOT_I1_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_I2_on_INT2
* Description    : Write I2_INT2
* Input          : LSM303AGR_ACC_I2_INT2_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_I2_on_INT2(LSM303AGR_ACC_I2_INT2_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_I2_INT2_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG6, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_I2_on_INT2
* Description    : Read I2_INT2
* Input          : Pointer to LSM303AGR_ACC_I2_INT2_t
* Output         : Status of I2_INT2 see LSM303AGR_ACC_I2_INT2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_I2_on_INT2(LSM303AGR_ACC_I2_INT2_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_I2_INT2_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_I2_on_INT1
* Description    : Write I2_INT1
* Input          : LSM303AGR_ACC_I2_INT1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_I2_on_INT1(LSM303AGR_ACC_I2_INT1_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_I2_INT1_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG6, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_I2_on_INT1
* Description    : Read I2_INT1
* Input          : Pointer to LSM303AGR_ACC_I2_INT1_t
* Output         : Status of I2_INT1 see LSM303AGR_ACC_I2_INT1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_I2_on_INT1(LSM303AGR_ACC_I2_INT1_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_I2_INT1_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Click_on_INT2
* Description    : Write I2_CLICKEN
* Input          : LSM303AGR_ACC_I2_CLICKEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Click_on_INT2(LSM303AGR_ACC_I2_CLICKEN_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_I2_CLICKEN_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CTRL_REG6, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Click_on_INT2
* Description    : Read I2_CLICKEN
* Input          : Pointer to LSM303AGR_ACC_I2_CLICKEN_t
* Output         : Status of I2_CLICKEN see LSM303AGR_ACC_I2_CLICKEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Click_on_INT2(LSM303AGR_ACC_I2_CLICKEN_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CTRL_REG6, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_I2_CLICKEN_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_ReferenceVal
* Description    : Write REF
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_ReferenceVal(u8_t newValue)
{
    u8_t value;

    newValue = newValue << LSM303AGR_ACC_REF_POSITION; //mask
    newValue &= LSM303AGR_ACC_REF_MASK; //coerce

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_REFERENCE, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_REF_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_REFERENCE, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ReferenceVal
* Description    : Read REF
* Input          : Pointer to u8_t
* Output         : Status of REF
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ReferenceVal(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_REFERENCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_REF_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_REF_POSITION; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_XDataAvail
* Description    : Read XDA
* Input          : Pointer to LSM303AGR_ACC_XDA_t
* Output         : Status of XDA see LSM303AGR_ACC_XDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_XDataAvail(LSM303AGR_ACC_XDA_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XDA_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_YDataAvail
* Description    : Read YDA
* Input          : Pointer to LSM303AGR_ACC_YDA_t
* Output         : Status of YDA see LSM303AGR_ACC_YDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_YDataAvail(LSM303AGR_ACC_YDA_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YDA_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ZDataAvail
* Description    : Read ZDA
* Input          : Pointer to LSM303AGR_ACC_ZDA_t
* Output         : Status of ZDA see LSM303AGR_ACC_ZDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ZDataAvail(LSM303AGR_ACC_ZDA_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZDA_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_XYZDataAvail
* Description    : Read ZYXDA
* Input          : Pointer to LSM303AGR_ACC_ZYXDA_t
* Output         : Status of ZYXDA see LSM303AGR_ACC_ZYXDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_XYZDataAvail(LSM303AGR_ACC_ZYXDA_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZYXDA_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_XDataOverrun
* Description    : Read XOR
* Input          : Pointer to LSM303AGR_ACC_XOR_t
* Output         : Status of XOR see LSM303AGR_ACC_XOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_XDataOverrun(LSM303AGR_ACC_XOR_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XOR_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_YDataOverrun
* Description    : Read YOR
* Input          : Pointer to LSM303AGR_ACC_YOR_t
* Output         : Status of YOR see LSM303AGR_ACC_YOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_YDataOverrun(LSM303AGR_ACC_YOR_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YOR_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ZDataOverrun
* Description    : Read ZOR
* Input          : Pointer to LSM303AGR_ACC_ZOR_t
* Output         : Status of ZOR see LSM303AGR_ACC_ZOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ZDataOverrun(LSM303AGR_ACC_ZOR_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZOR_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_XYZDataOverrun
* Description    : Read ZYXOR
* Input          : Pointer to LSM303AGR_ACC_ZYXOR_t
* Output         : Status of ZYXOR see LSM303AGR_ACC_ZYXOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_XYZDataOverrun(LSM303AGR_ACC_ZYXOR_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_STATUS_REG2, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZYXOR_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FifoThreshold
* Description    : Write FTH
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FifoThreshold(u8_t newValue)
{
    u8_t value;

    newValue = newValue << LSM303AGR_ACC_FTH_POSITION; //mask
    newValue &= LSM303AGR_ACC_FTH_MASK; //coerce

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_FIFO_CTRL_REG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_FTH_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_FIFO_CTRL_REG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FifoThreshold
* Description    : Read FTH
* Input          : Pointer to u8_t
* Output         : Status of FTH
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FifoThreshold(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_FIFO_CTRL_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_FTH_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_FTH_POSITION; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_TriggerSel
* Description    : Write TR
* Input          : LSM303AGR_ACC_TR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_TriggerSel(LSM303AGR_ACC_TR_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_FIFO_CTRL_REG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_TR_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_FIFO_CTRL_REG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_TriggerSel
* Description    : Read TR
* Input          : Pointer to LSM303AGR_ACC_TR_t
* Output         : Status of TR see LSM303AGR_ACC_TR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_TriggerSel(LSM303AGR_ACC_TR_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_FIFO_CTRL_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_TR_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_FifoMode
* Description    : Write FM
* Input          : LSM303AGR_ACC_FM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_FifoMode(LSM303AGR_ACC_FM_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_FIFO_CTRL_REG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_FM_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_FIFO_CTRL_REG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FifoMode
* Description    : Read FM
* Input          : Pointer to LSM303AGR_ACC_FM_t
* Output         : Status of FM see LSM303AGR_ACC_FM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FifoMode(LSM303AGR_ACC_FM_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_FIFO_CTRL_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_FM_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FifoSamplesAvail
* Description    : Read FSS
* Input          : Pointer to u8_t
* Output         : Status of FSS
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FifoSamplesAvail(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_FIFO_SRC_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_FSS_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_FSS_POSITION; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FifoEmpty
* Description    : Read EMPTY
* Input          : Pointer to LSM303AGR_ACC_EMPTY_t
* Output         : Status of EMPTY see LSM303AGR_ACC_EMPTY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FifoEmpty(LSM303AGR_ACC_EMPTY_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_FIFO_SRC_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_EMPTY_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_FifoOverrun
* Description    : Read OVRN_FIFO
* Input          : Pointer to LSM303AGR_ACC_OVRN_FIFO_t
* Output         : Status of OVRN_FIFO see LSM303AGR_ACC_OVRN_FIFO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_FifoOverrun(LSM303AGR_ACC_OVRN_FIFO_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_FIFO_SRC_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_OVRN_FIFO_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_WatermarkLevel
* Description    : Read WTM
* Input          : Pointer to LSM303AGR_ACC_WTM_t
* Output         : Status of WTM see LSM303AGR_ACC_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_WatermarkLevel(LSM303AGR_ACC_WTM_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_FIFO_SRC_REG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_WTM_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int1EnXLo
* Description    : Write XLIE
* Input          : LSM303AGR_ACC_XLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int1EnXLo(LSM303AGR_ACC_XLIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_XLIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT1_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1EnXLo
* Description    : Read XLIE
* Input          : Pointer to LSM303AGR_ACC_XLIE_t
* Output         : Status of XLIE see LSM303AGR_ACC_XLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1EnXLo(LSM303AGR_ACC_XLIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XLIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int1EnXHi
* Description    : Write XHIE
* Input          : LSM303AGR_ACC_XHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int1EnXHi(LSM303AGR_ACC_XHIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_XHIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT1_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1EnXHi
* Description    : Read XHIE
* Input          : Pointer to LSM303AGR_ACC_XHIE_t
* Output         : Status of XHIE see LSM303AGR_ACC_XHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1EnXHi(LSM303AGR_ACC_XHIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XHIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int1EnYLo
* Description    : Write YLIE
* Input          : LSM303AGR_ACC_YLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int1EnYLo(LSM303AGR_ACC_YLIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_YLIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT1_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1EnYLo
* Description    : Read YLIE
* Input          : Pointer to LSM303AGR_ACC_YLIE_t
* Output         : Status of YLIE see LSM303AGR_ACC_YLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1EnYLo(LSM303AGR_ACC_YLIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YLIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int1EnYHi
* Description    : Write YHIE
* Input          : LSM303AGR_ACC_YHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int1EnYHi(LSM303AGR_ACC_YHIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_YHIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT1_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1EnYHi
* Description    : Read YHIE
* Input          : Pointer to LSM303AGR_ACC_YHIE_t
* Output         : Status of YHIE see LSM303AGR_ACC_YHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1EnYHi(LSM303AGR_ACC_YHIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YHIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int1EnZLo
* Description    : Write ZLIE
* Input          : LSM303AGR_ACC_ZLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int1EnZLo(LSM303AGR_ACC_ZLIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_ZLIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT1_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1EnZLo
* Description    : Read ZLIE
* Input          : Pointer to LSM303AGR_ACC_ZLIE_t
* Output         : Status of ZLIE see LSM303AGR_ACC_ZLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1EnZLo(LSM303AGR_ACC_ZLIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZLIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int1EnZHi
* Description    : Write ZHIE
* Input          : LSM303AGR_ACC_ZHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int1EnZHi(LSM303AGR_ACC_ZHIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_ZHIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT1_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1EnZHi
* Description    : Read ZHIE
* Input          : Pointer to LSM303AGR_ACC_ZHIE_t
* Output         : Status of ZHIE see LSM303AGR_ACC_ZHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1EnZHi(LSM303AGR_ACC_ZHIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZHIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int1_6D
* Description    : Write 6D
* Input          : LSM303AGR_ACC_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int1_6D(LSM303AGR_ACC_6D_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_6D_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT1_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_6D
* Description    : Read 6D
* Input          : Pointer to LSM303AGR_ACC_6D_t
* Output         : Status of 6D see LSM303AGR_ACC_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_6D(LSM303AGR_ACC_6D_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_6D_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int1_AOI
* Description    : Write AOI
* Input          : LSM303AGR_ACC_AOI_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int1_AOI(LSM303AGR_ACC_AOI_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_AOI_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT1_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_AOI
* Description    : Read AOI
* Input          : Pointer to LSM303AGR_ACC_AOI_t
* Output         : Status of AOI see LSM303AGR_ACC_AOI_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_AOI(LSM303AGR_ACC_AOI_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_AOI_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int2EnXLo
* Description    : Write XLIE
* Input          : LSM303AGR_ACC_XLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int2EnXLo(LSM303AGR_ACC_XLIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_XLIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT2_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2EnXLo
* Description    : Read XLIE
* Input          : Pointer to LSM303AGR_ACC_XLIE_t
* Output         : Status of XLIE see LSM303AGR_ACC_XLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2EnXLo(LSM303AGR_ACC_XLIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XLIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int2EnXHi
* Description    : Write XHIE
* Input          : LSM303AGR_ACC_XHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int2EnXHi(LSM303AGR_ACC_XHIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_XHIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT2_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2EnXHi
* Description    : Read XHIE
* Input          : Pointer to LSM303AGR_ACC_XHIE_t
* Output         : Status of XHIE see LSM303AGR_ACC_XHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2EnXHi(LSM303AGR_ACC_XHIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XHIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int2EnYLo
* Description    : Write YLIE
* Input          : LSM303AGR_ACC_YLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int2EnYLo(LSM303AGR_ACC_YLIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_YLIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT2_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2EnYLo
* Description    : Read YLIE
* Input          : Pointer to LSM303AGR_ACC_YLIE_t
* Output         : Status of YLIE see LSM303AGR_ACC_YLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2EnYLo(LSM303AGR_ACC_YLIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YLIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int2EnYHi
* Description    : Write YHIE
* Input          : LSM303AGR_ACC_YHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int2EnYHi(LSM303AGR_ACC_YHIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_YHIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT2_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2EnYHi
* Description    : Read YHIE
* Input          : Pointer to LSM303AGR_ACC_YHIE_t
* Output         : Status of YHIE see LSM303AGR_ACC_YHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2EnYHi(LSM303AGR_ACC_YHIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YHIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int2EnZLo
* Description    : Write ZLIE
* Input          : LSM303AGR_ACC_ZLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int2EnZLo(LSM303AGR_ACC_ZLIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_ZLIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT2_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2EnZLo
* Description    : Read ZLIE
* Input          : Pointer to LSM303AGR_ACC_ZLIE_t
* Output         : Status of ZLIE see LSM303AGR_ACC_ZLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2EnZLo(LSM303AGR_ACC_ZLIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZLIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int2EnZHi
* Description    : Write ZHIE
* Input          : LSM303AGR_ACC_ZHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int2EnZHi(LSM303AGR_ACC_ZHIE_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_ZHIE_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT2_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2EnZHi
* Description    : Read ZHIE
* Input          : Pointer to LSM303AGR_ACC_ZHIE_t
* Output         : Status of ZHIE see LSM303AGR_ACC_ZHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2EnZHi(LSM303AGR_ACC_ZHIE_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZHIE_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int2_6D
* Description    : Write 6D
* Input          : LSM303AGR_ACC_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int2_6D(LSM303AGR_ACC_6D_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_6D_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT2_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_6D
* Description    : Read 6D
* Input          : Pointer to LSM303AGR_ACC_6D_t
* Output         : Status of 6D see LSM303AGR_ACC_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_6D(LSM303AGR_ACC_6D_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_6D_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int2_AOI
* Description    : Write AOI
* Input          : LSM303AGR_ACC_AOI_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int2_AOI(LSM303AGR_ACC_AOI_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_AOI_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT2_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_AOI
* Description    : Read AOI
* Input          : Pointer to LSM303AGR_ACC_AOI_t
* Output         : Status of AOI see LSM303AGR_ACC_AOI_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_AOI(LSM303AGR_ACC_AOI_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_AOI_MASK; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_Xlo
* Description    : Read XL
* Input          : Pointer to LSM303AGR_ACC_XL_t
* Output         : Status of XL see LSM303AGR_ACC_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_Xlo(LSM303AGR_ACC_XL_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XL_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_XHi
* Description    : Read XH
* Input          : Pointer to LSM303AGR_ACC_XH_t
* Output         : Status of XH see LSM303AGR_ACC_XH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_XHi(LSM303AGR_ACC_XH_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XH_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_YLo
* Description    : Read YL
* Input          : Pointer to LSM303AGR_ACC_YL_t
* Output         : Status of YL see LSM303AGR_ACC_YL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_YLo(LSM303AGR_ACC_YL_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YL_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_YHi
* Description    : Read YH
* Input          : Pointer to LSM303AGR_ACC_YH_t
* Output         : Status of YH see LSM303AGR_ACC_YH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_YHi(LSM303AGR_ACC_YH_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YH_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_Zlo
* Description    : Read ZL
* Input          : Pointer to LSM303AGR_ACC_ZL_t
* Output         : Status of ZL see LSM303AGR_ACC_ZL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_Zlo(LSM303AGR_ACC_ZL_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZL_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_ZHi
* Description    : Read ZH
* Input          : Pointer to LSM303AGR_ACC_ZH_t
* Output         : Status of ZH see LSM303AGR_ACC_ZH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_ZHi(LSM303AGR_ACC_ZH_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZH_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_IA
* Description    : Read IA
* Input          : Pointer to LSM303AGR_ACC_IA_t
* Output         : Status of IA see LSM303AGR_ACC_IA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_IA(LSM303AGR_ACC_IA_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_IA_MASK; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_Xlo
* Description    : Read XL
* Input          : Pointer to LSM303AGR_ACC_XL_t
* Output         : Status of XL see LSM303AGR_ACC_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_Xlo(LSM303AGR_ACC_XL_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XL_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_XHi
* Description    : Read XH
* Input          : Pointer to LSM303AGR_ACC_XH_t
* Output         : Status of XH see LSM303AGR_ACC_XH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_XHi(LSM303AGR_ACC_XH_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XH_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_YLo
* Description    : Read YL
* Input          : Pointer to LSM303AGR_ACC_YL_t
* Output         : Status of YL see LSM303AGR_ACC_YL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_YLo(LSM303AGR_ACC_YL_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YL_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_YHi
* Description    : Read YH
* Input          : Pointer to LSM303AGR_ACC_YH_t
* Output         : Status of YH see LSM303AGR_ACC_YH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_YHi(LSM303AGR_ACC_YH_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YH_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_Zlo
* Description    : Read ZL
* Input          : Pointer to LSM303AGR_ACC_ZL_t
* Output         : Status of ZL see LSM303AGR_ACC_ZL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_Zlo(LSM303AGR_ACC_ZL_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZL_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_ZHi
* Description    : Read ZH
* Input          : Pointer to LSM303AGR_ACC_ZH_t
* Output         : Status of ZH see LSM303AGR_ACC_ZH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_ZHi(LSM303AGR_ACC_ZH_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZH_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_IA
* Description    : Read IA
* Input          : Pointer to LSM303AGR_ACC_IA_t
* Output         : Status of IA see LSM303AGR_ACC_IA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_IA(LSM303AGR_ACC_IA_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_SOURCE, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_IA_MASK; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int1_Threshold
* Description    : Write THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int1_Threshold(u8_t newValue)
{
    u8_t value;

    newValue = newValue << LSM303AGR_ACC_THS_POSITION; //mask
    newValue &= LSM303AGR_ACC_THS_MASK; //coerce

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_THS, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_THS_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT1_THS, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_Threshold
* Description    : Read THS
* Input          : Pointer to u8_t
* Output         : Status of THS
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_Threshold(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_THS, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_THS_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_THS_POSITION; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int2_Threshold
* Description    : Write THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int2_Threshold(u8_t newValue)
{
    u8_t value;

    newValue = newValue << LSM303AGR_ACC_THS_POSITION; //mask
    newValue &= LSM303AGR_ACC_THS_MASK; //coerce

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_THS, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_THS_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT2_THS, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_Threshold
* Description    : Read THS
* Input          : Pointer to u8_t
* Output         : Status of THS
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_Threshold(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_THS, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_THS_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_THS_POSITION; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int1_Duration
* Description    : Write D
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int1_Duration(u8_t newValue)
{
    u8_t value;

    newValue = newValue << LSM303AGR_ACC_D_POSITION; //mask
    newValue &= LSM303AGR_ACC_D_MASK; //coerce

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_DURATION, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_D_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT1_DURATION, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int1_Duration
* Description    : Read D
* Input          : Pointer to u8_t
* Output         : Status of D
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int1_Duration(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT1_DURATION, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_D_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_D_POSITION; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_Int2_Duration
* Description    : Write D
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_Int2_Duration(u8_t newValue)
{
    u8_t value;

    newValue = newValue << LSM303AGR_ACC_D_POSITION; //mask
    newValue &= LSM303AGR_ACC_D_MASK; //coerce

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_DURATION, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_D_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_INT2_DURATION, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_Int2_Duration
* Description    : Read D
* Input          : Pointer to u8_t
* Output         : Status of D
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_Int2_Duration(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_INT2_DURATION, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_D_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_D_POSITION; //mask

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_XSingle
* Description    : Write XS
* Input          : LSM303AGR_ACC_XS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_XSingle(LSM303AGR_ACC_XS_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_XS_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CLICK_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_XSingle
* Description    : Read XS
* Input          : Pointer to LSM303AGR_ACC_XS_t
* Output         : Status of XS see LSM303AGR_ACC_XS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_XSingle(LSM303AGR_ACC_XS_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XS_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_XDouble
* Description    : Write XD
* Input          : LSM303AGR_ACC_XD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_XDouble(LSM303AGR_ACC_XD_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_XD_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CLICK_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_XDouble
* Description    : Read XD
* Input          : Pointer to LSM303AGR_ACC_XD_t
* Output         : Status of XD see LSM303AGR_ACC_XD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_XDouble(LSM303AGR_ACC_XD_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_XD_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_YSingle
* Description    : Write YS
* Input          : LSM303AGR_ACC_YS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_YSingle(LSM303AGR_ACC_YS_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_YS_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CLICK_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_YSingle
* Description    : Read YS
* Input          : Pointer to LSM303AGR_ACC_YS_t
* Output         : Status of YS see LSM303AGR_ACC_YS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_YSingle(LSM303AGR_ACC_YS_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YS_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_YDouble
* Description    : Write YD
* Input          : LSM303AGR_ACC_YD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_YDouble(LSM303AGR_ACC_YD_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_YD_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CLICK_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_YDouble
* Description    : Read YD
* Input          : Pointer to LSM303AGR_ACC_YD_t
* Output         : Status of YD see LSM303AGR_ACC_YD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_YDouble(LSM303AGR_ACC_YD_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_YD_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_ZSingle
* Description    : Write ZS
* Input          : LSM303AGR_ACC_ZS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_ZSingle(LSM303AGR_ACC_ZS_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_ZS_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CLICK_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ZSingle
* Description    : Read ZS
* Input          : Pointer to LSM303AGR_ACC_ZS_t
* Output         : Status of ZS see LSM303AGR_ACC_ZS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ZSingle(LSM303AGR_ACC_ZS_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZS_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_ZDouble
* Description    : Write ZD
* Input          : LSM303AGR_ACC_ZD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_ZDouble(LSM303AGR_ACC_ZD_t newValue)
{
    u8_t value;

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_ZD_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CLICK_CFG, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ZDouble
* Description    : Read ZD
* Input          : Pointer to LSM303AGR_ACC_ZD_t
* Output         : Status of ZD see LSM303AGR_ACC_ZD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ZDouble(LSM303AGR_ACC_ZD_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_CFG, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_ZD_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ClickX
* Description    : Read X
* Input          : Pointer to LSM303AGR_ACC_X_t
* Output         : Status of X see LSM303AGR_ACC_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ClickX(LSM303AGR_ACC_X_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_SRC, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_X_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ClickY
* Description    : Read Y
* Input          : Pointer to LSM303AGR_ACC_Y_t
* Output         : Status of Y see LSM303AGR_ACC_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ClickY(LSM303AGR_ACC_Y_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_SRC, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_Y_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ClickZ
* Description    : Read Z
* Input          : Pointer to LSM303AGR_ACC_Z_t
* Output         : Status of Z see LSM303AGR_ACC_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ClickZ(LSM303AGR_ACC_Z_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_SRC, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_Z_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ClickSign
* Description    : Read SIGN
* Input          : Pointer to LSM303AGR_ACC_SIGN_t
* Output         : Status of SIGN see LSM303AGR_ACC_SIGN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ClickSign(LSM303AGR_ACC_SIGN_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_SRC, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_SIGN_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_SingleCLICK
* Description    : Read SCLICK
* Input          : Pointer to LSM303AGR_ACC_SCLICK_t
* Output         : Status of SCLICK see LSM303AGR_ACC_SCLICK_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_SingleCLICK(LSM303AGR_ACC_SCLICK_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_SRC, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_SCLICK_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_DoubleCLICK
* Description    : Read DCLICK
* Input          : Pointer to LSM303AGR_ACC_DCLICK_t
* Output         : Status of DCLICK see LSM303AGR_ACC_DCLICK_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_DoubleCLICK(LSM303AGR_ACC_DCLICK_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_SRC, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_DCLICK_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_IA
* Description    : Read IA
* Input          : Pointer to LSM303AGR_ACC_IA_t
* Output         : Status of IA see LSM303AGR_ACC_IA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_CLICK_IA(LSM303AGR_ACC_CLICK_IA_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_SRC, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_IA_MASK; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_ClickThreshold
* Description    : Write THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_ClickThreshold(u8_t newValue)
{
    u8_t value;

    newValue = newValue << LSM303AGR_ACC_THS_POSITION; //mask
    newValue &= LSM303AGR_ACC_THS_MASK; //coerce

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_THS, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_THS_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_CLICK_THS, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ClickThreshold
* Description    : Read THS
* Input          : Pointer to u8_t
* Output         : Status of THS
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ClickThreshold(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_CLICK_THS, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_THS_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_THS_POSITION; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_ClickTimeLimit
* Description    : Write TLI
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_ClickTimeLimit(u8_t newValue)
{
    u8_t value;

    newValue = newValue << LSM303AGR_ACC_TLI_POSITION; //mask
    newValue &= LSM303AGR_ACC_TLI_MASK; //coerce

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_TIME_LIMIT, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_TLI_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_TIME_LIMIT, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ClickTimeLimit
* Description    : Read TLI
* Input          : Pointer to u8_t
* Output         : Status of TLI
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ClickTimeLimit(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_TIME_LIMIT, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_TLI_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_TLI_POSITION; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_ClickTimeLatency
* Description    : Write TLA
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_ClickTimeLatency(u8_t newValue)
{
    u8_t value;

    newValue = newValue << LSM303AGR_ACC_TLA_POSITION; //mask
    newValue &= LSM303AGR_ACC_TLA_MASK; //coerce

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_TIME_LATENCY, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_TLA_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_TIME_LATENCY, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ClickTimeLatency
* Description    : Read TLA
* Input          : Pointer to u8_t
* Output         : Status of TLA
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ClickTimeLatency(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_TIME_LATENCY, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_TLA_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_TLA_POSITION; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM303AGR_ACC_W_ClickTimeWindow
* Description    : Write TW
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303AGR_ACC_W_ClickTimeWindow(u8_t newValue)
{
    u8_t value;

    newValue = newValue << LSM303AGR_ACC_TW_POSITION; //mask
    newValue &= LSM303AGR_ACC_TW_MASK; //coerce

    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_TIME_WINDOW, &value) )
        return MEMS_ERROR;

    value &= ~LSM303AGR_ACC_TW_MASK;
    value |= newValue;

    if( HAL_OK!=LSM303AGR_ACC_WriteReg(LSM303AGR_ACC_TIME_WINDOW, value) )
        return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303AGR_ACC_R_ClickTimeWindow
* Description    : Read TW
* Input          : Pointer to u8_t
* Output         : Status of TW
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM303AGR_ACC_R_ClickTimeWindow(u8_t *value)
{
    if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_TIME_WINDOW, (u8_t *)value) )
        return MEMS_ERROR;

    *value &= LSM303AGR_ACC_TW_MASK; //coerce
    *value = *value >> LSM303AGR_ACC_TW_POSITION; //mask

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : status_t LSM303AGR_ACC_Get_Voltage_ADC(u8_t *buff)
* Description    : Read Voltage_ADC output register
* Input          : pointer to [u8_t]
* Output         : Voltage_ADC buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_ACC_Get_Voltage_ADC(u8_t *buff)
{
    u8_t i, j, k;
    u8_t numberOfByteForDimension;

    numberOfByteForDimension=6/3;

    k=0;
    for (i=0; i<3; i++ )
    {
        for (j=0; j<numberOfByteForDimension; j++ )
        {
            if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_OUT_ADC1_L+k, &buff[k]))
                return MEMS_ERROR;
            k++;
        }
    }

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM303AGR_ACC_Get_Raw_Acceleration(u8_t *buff)
* Description    : Read Acceleration output register
* Input          : pointer to [u8_t]
* Output         : Acceleration buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303AGR_ACC_Get_Raw_Acceleration(u8_t *buff)
{
    u8_t i, j, k;
    u8_t numberOfByteForDimension;

    numberOfByteForDimension=6/3;

    k=0;
    for (i=0; i<3; i++ )
    {
        for (j=0; j<numberOfByteForDimension; j++ )
        {
            if( HAL_OK!=LSM303AGR_ACC_ReadReg(LSM303AGR_ACC_OUT_X_L+k, &buff[k]))
                return MEMS_ERROR;
            k++;
        }
    }

    return MEMS_SUCCESS;
}

/*
 * Following is the table of sensitivity values for each case.
 * Values are espressed in ug/digit.
 */
const long long LSM303AGR_ACC_Sensitivity_List[3][4] = {
    /* HR 12-bit */
    {
        980,	/* FS @2g */
        1950,	/* FS @4g */
        3900,	/* FS @8g */
        11720,	/* FS @16g */
    },

    /* Normal 10-bit */
    {
        3900,	/* FS @2g */
        7820,	/* FS @4g */
        15630,	/* FS @8g */
        46900,	/* FS @16g */
    },

    /* LP 8-bit */
    {
        15630,	/* FS @2g */
        31260,	/* FS @4g */
        62520,	/* FS @8g */
        187580,	/* FS @16g */
    },
};

/*
 * Values returned are espressed in mg.
 */
status_t LSM303AGR_ACC_Get_Acceleration(int *buff)
{
    Type3Axis16bit_U raw_data_tmp;
    u8_t op_mode = 0, fs_mode = 0, shift = 0;
    LSM303AGR_ACC_LPEN_t lp;
    LSM303AGR_ACC_HR_t hr;
    LSM303AGR_ACC_FS_t fs;

    /* Determine which operational mode the acc is set */
    LSM303AGR_ACC_R_HiRes(&hr);
    LSM303AGR_ACC_R_LOWPWR_EN(&lp);

    if (lp == LSM303AGR_ACC_LPEN_ENABLED && hr == LSM303AGR_ACC_HR_DISABLED) {
        /* op mode is LP 8-bit */
        op_mode = 2;
        shift = 8;
    } else if (lp == LSM303AGR_ACC_LPEN_DISABLED && hr == LSM303AGR_ACC_HR_DISABLED) {
        /* op mode is Normal 10-bit */
        op_mode = 1;
        shift = 6;
    } else if (lp == LSM303AGR_ACC_LPEN_DISABLED && hr == LSM303AGR_ACC_HR_ENABLED) {
        /* op mode is HR 12-bit */
        op_mode = 0;
        shift = 4;
    } else
        return MEMS_ERROR;

    /* Determine the Full Scale the acc is set */
    LSM303AGR_ACC_R_FullScale(&fs);
    switch (fs) {
    case LSM303AGR_ACC_FS_2G:
        fs_mode = 0;
        break;

    case LSM303AGR_ACC_FS_4G:
        fs_mode = 1;
        break;

    case LSM303AGR_ACC_FS_8G:
        fs_mode = 2;
        break;

    case LSM303AGR_ACC_FS_16G:
        fs_mode = 3;
        break;
    }

    /* Read out raw accelerometer samples */
    LSM303AGR_ACC_Get_Raw_Acceleration(raw_data_tmp.u8bit);

    /* Apply proper shift and sensitivity */
    buff[0] = ((raw_data_tmp.i16bit[0] >> shift) * LSM303AGR_ACC_Sensitivity_List[op_mode][fs_mode] + 500) / 1000;
    buff[1] = ((raw_data_tmp.i16bit[1] >> shift) * LSM303AGR_ACC_Sensitivity_List[op_mode][fs_mode] + 500) / 1000;
    buff[2] = ((raw_data_tmp.i16bit[2] >> shift) * LSM303AGR_ACC_Sensitivity_List[op_mode][fs_mode] + 500) / 1000;

    return MEMS_SUCCESS;
}
