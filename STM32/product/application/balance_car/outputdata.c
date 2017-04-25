
#include "outputdata.h"
#include "stm32f4xx_hal.h"

float OutData[4] = { 0 };
long int CheckSum_OutData[4] = {0};


static unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}
extern UART_HandleTypeDef UartHandle;
void OutPut_Data(void)
{
    int temp[4] = {0};
    
    unsigned int temp1[4] = {0};
    unsigned char databuf[10] = {0};
    unsigned char i;
    unsigned short CRC16 = 0;

    for(i=0;i<4;i++)
    {

        temp[i]  = (int)OutData[i];
        temp1[i] = (unsigned int)temp[i];

    }

    for(i=0;i<4;i++)
    {
        databuf[i*2]   = (unsigned char)(temp1[i]%256);
        databuf[i*2+1] = (unsigned char)(temp1[i]/256);
    }
    
    CRC16 = CRC_CHECK(databuf,8);
    databuf[8] = CRC16%256;
    databuf[9] = CRC16/256;

    HAL_UART_Transmit(&UartHandle,databuf,10,100);
}


void OutPut_CheckSumData(void)
{
    unsigned short CRC_Tmp,i;
    unsigned char databuf[9] = {0};
    int temp[4] = {0};
    unsigned int temp1[4] = {0};


    for(i=0;i<4;i++)
    {
        temp[i]  = (int)CheckSum_OutData[i];
        temp1[i] = (unsigned int)temp[i];
    }
    for(i=0;i<4;i++)
    {
        databuf[i*2]   = (unsigned char)(temp1[i]%256);
        databuf[i*2+1] = (unsigned char)(temp1[i]/256);
    }

    CRC_Tmp = 0;
    for(i = 0; i < 8; i++) CRC_Tmp += databuf[i];
    databuf[8] = CRC_Tmp ;
}
