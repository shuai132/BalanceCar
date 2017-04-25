#ifndef _outputdata_H
#define _outputdata_H

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
/* UART Driver context memory */

#define DLY(x) if (1) {volatile uint32_t dl = (x); while (dl--) {}}

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

int uartrom_config(void);
int uartrom_init(void);
extern float OutData[4];
void OutPut_Data(void);
const char *uart_puts(const char *buf);
extern long int CheckSum_OutData[4];
void OutPut_CheckSumData(void);

#endif 
