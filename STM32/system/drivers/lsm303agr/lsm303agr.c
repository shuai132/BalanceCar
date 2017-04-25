
#include "lsm303agr.h"
#define FIFO_SIZE	32
#define FIFO_THSLD	FIFO_SIZE - 2

/* This macro can be used to switch on/off the evaluation with interrupts */
#define TEST_WITH_WTM_INTERRUPT     1

int Acceleration_mG[FIFO_SIZE][3];
int Magnetic_mGa[3];

#ifdef WAKEUP_TEST
static u32_t LSM303AGR_ACC_sample_calls = 0;


/*
 * Callback to handle the ACC event.
 * It must be registered to be called at interrupt time.
 *
 * This is triggered by FIFO WTM interrupt
 */
static void LSM303AGR_ACC_sample_Callback(u8_t intID)
{
    LSM303AGR_ACC_sample_calls++;

    u8_t num = 0, cnt = 0;

    /*
     * Read ACC samples from FIFO
     */
    LSM303AGR_ACC_R_FifoSamplesAvail(&num);

    while (num-- > 0)
        LSM303AGR_ACC_Get_Acceleration(Acceleration_mG[cnt++]);

    /*
     * Read MAG samples from FIFO
     */
    LSM303AGR_MAG_Get_Magnetic(Magnetic_mGa);
}
#endif

/* init the FIFO in stream mode with a given threshold */
static void init_LSM303AGR_FIFO(u8_t fifo_thld)
{
    /* Set stream mode */
    LSM303AGR_ACC_W_FifoMode(LSM303AGR_ACC_FM_STREAM);

    /* Set Threshold */
    LSM303AGR_ACC_W_FifoThreshold(fifo_thld);

#if (TEST_WITH_WTM_INTERRUPT == 1)
    /* register callback */
//  RegisterInterrupt(LSM303AGR_ACC_sample_Callback);

    /* Drive WTM event on INT1 pad */
    LSM303AGR_ACC_W_FIFO_Watermark_on_INT1(LSM303AGR_ACC_I1_WTM_ENABLED);
#endif

    /* Enable FIFO */
    LSM303AGR_ACC_W_FIFO_EN(LSM303AGR_ACC_FIFO_EN_ENABLED);
}

/*
 * configure LSM303AGR device
 */
int init_LSM303AGR_acc(LSM303AGR_ACC_FS_t fs, LSM303AGR_ACC_ODR_t odr, u8_t fifo_thld)
{
    u8_t who_am_i;

    /* Read WHO_AM_I  and check if device is really the correct one */
    LSM303AGR_ACC_R_WHO_AM_I(&who_am_i);
    if (who_am_i != LSM303AGR_ACC_WHO_AM_I)
        return -1;

    /* init the FIFO in stream mode with a given threshold */
    if (fifo_thld)
        init_LSM303AGR_FIFO(fifo_thld);

    /* Set Acc Full Scale */
    LSM303AGR_ACC_W_FullScale(fs);

    /* Set high resolution */
    LSM303AGR_ACC_W_HiRes(LSM303AGR_ACC_HR_ENABLED);

    /* Set Acc Output Data Rate */
    LSM303AGR_ACC_W_ODR(odr);

    /* Enable Acc Block Data Update */
    LSM303AGR_ACC_W_BlockDataUpdate(LSM303AGR_ACC_BDU_ENABLED);

    /* Enable Acc X/Y/Z axis */
    LSM303AGR_ACC_W_XEN(LSM303AGR_ACC_XEN_ENABLED);
    LSM303AGR_ACC_W_YEN(LSM303AGR_ACC_YEN_ENABLED);
    LSM303AGR_ACC_W_ZEN(LSM303AGR_ACC_ZEN_ENABLED);

    return(0);
}

int init_LSM303AGR_mag(LSM303AGR_MAG_ODR_t odr)
{
    u8_t who_am_i;
    //u16_t value;

    /* Read WHO_AM_I  and check if device is really the correct one */
    LSM303AGR_MAG_R_WHO_AM_I(&who_am_i);
    if (who_am_i != LSM303AGR_MAG_WHO_AM_I)
        return -1;

    /* Enable the magnetometer */
    LSM303AGR_MAG_W_ODR(odr);
    LSM303AGR_MAG_W_MD(LSM303AGR_MAG_MD_CONTINUOS_MODE);

    return(0);
}

#ifdef SAMPLE_AQUISITION
/* Test Acquisition of sensor samples */
static  void Loop_Test_Sample_Aquisition(void)
{
    /* init accelerometer */
    if (init_LSM303AGR_acc(LSM303AGR_ACC_FS_2G, LSM303AGR_ACC_ODR_DO_100Hz, FIFO_THSLD) < 0)
       return; /* handle error */

    /*
     * Set mag to lowest ODR, because in this example we read one
     * mag samples every FIFO-threshold acc samples.
     */
    if (init_LSM303AGR_mag(LSM303AGR_MAG_ODR_10Hz) < 0)
        return; /* handle error */

#if (TEST_WITH_WTM_INTERRUPT == 1)
    while(1) {
        /* Event will be handled in driver callback */
    }
#else
    /*
     * Read samples in polling mode (no int)
     */
    while(1)
    {
        LSM303AGR_ACC_WTM_t wtm;
        u8_t num = 0, cnt = 0;

        /* Read LSM303AGR output only if new ACC values are available */
        LSM303AGR_ACC_R_WatermarkLevel(&wtm);

        if (wtm == LSM303AGR_ACC_WTM_OVERFLOW) {
            LSM303AGR_ACC_R_FifoSamplesAvail(&num);

            while (num-- > 0)
                LSM303AGR_ACC_Get_Acceleration(Acceleration_mG[cnt++]);
        }

        /* Read LSM303AGR also the MAG sample */
        LSM303AGR_MAG_Get_Magnetic(Magnetic_mGa);

    }
#endif
}
#endif

#ifdef WAKEUP_TEST
/*
 * Wakeup feature
 */

static u32_t LSM303AGR_ACC_wakeup_ev_num = 0;

/*
 * Callback to handle the Wakeup event.
 * It must be registered to be called at interrupt time.
 */
static void LSM303AGR_ACC_Wakeup_Callback(u8_t intID)
{
    LSM303AGR_ACC_IA_t WakeupStatus;

    LSM303AGR_ACC_wakeup_ev_num++;

    /* clear wakeup event */
    LSM303AGR_ACC_R_Int2_IA(&WakeupStatus);
}
#endif

#ifdef WAKEUP_TEST
/* Init the Wakeup feature */
static void init_LSM303AGR_Wakeup(void)
{
    /* register callback */
//  RegisterInterrupt(LSM303AGR_ACC_Wakeup_Callback);

    /* init accelerometer */
    if (init_LSM303AGR_acc(LSM303AGR_ACC_FS_2G, LSM303AGR_ACC_ODR_DO_100Hz, 0) < 0)
        Error_Handler(); /* handle error */

    /* Ebale I2 function on INT2 pad */
    LSM303AGR_ACC_W_I2_on_INT2(LSM303AGR_ACC_I2_INT2_ENABLED);

    /* Enable XH and YH event on INT2 */
    LSM303AGR_ACC_W_Int2EnXHi(LSM303AGR_ACC_XHIE_ENABLED);
    LSM303AGR_ACC_W_Int2EnYHi(LSM303AGR_ACC_YHIE_ENABLED);

    /* Set INT2 threshold and Duration */
    LSM303AGR_ACC_W_Int2_Threshold(0x25);
    LSM303AGR_ACC_W_Int2_Duration(0);

    /* Enable LIR on INT2 */
    LSM303AGR_ACC_W_LatchInterrupt_on_INT2(LSM303AGR_ACC_LIR_INT2_ENABLED);
}

#endif

/* Test Wakeup */
#ifdef WAKEUP_TEST
static void  Loop_Test_Wakeup(void)
{
    LSM303AGR_ACC_IA_t WakeupStatus;

    /* Clear INT2_IA event*/
    LSM303AGR_ACC_R_Int2_IA(&WakeupStatus);

    /* configure wakeup */
    init_LSM303AGR_Wakeup();

    while(1) {
        /* Event will be handled in driver callback */
    }
}
#endif

#ifdef FREE_FALL_TEST
/*
 * Free-Fall feature
 */
static u32_t LSM303AGR_ACC_FreeFall_ev_num = 0;

/*
 * Callback to handle the FreeFall event.
 * It must be registered to be called at interrupt time.
 */
static void LSM303AGR_ACC_FreeFall_Callback(u8_t intID)
{
    LSM303AGR_ACC_IA_t FreeFallStatus;

    LSM303AGR_ACC_FreeFall_ev_num++;

    /* clear FreeFall event */
    LSM303AGR_ACC_R_Int2_IA(&FreeFallStatus);
}
#endif

#ifdef FREE_FALL_TEST
/* Init the FreeFall feature */
static void init_LSM303AGR_FreeFall(void)
{
    /* register callback */
//  RegisterInterrupt(LSM303AGR_ACC_FreeFall_Callback);

    /* init accelerometer */
    if (init_LSM303AGR_acc(LSM303AGR_ACC_FS_2G, LSM303AGR_ACC_ODR_DO_100Hz, 0) < 0)
        while(1); /* handle error */

    /* Ebale I2 function on INT2 pad */
    LSM303AGR_ACC_W_I2_on_INT2(LSM303AGR_ACC_I2_INT2_ENABLED);

    /* Enable AOI, ZL, XL and YL event on INT2 */
    LSM303AGR_ACC_W_Int2EnXLo(LSM303AGR_ACC_XLIE_ENABLED);
    LSM303AGR_ACC_W_Int2EnYLo(LSM303AGR_ACC_YLIE_ENABLED);
    LSM303AGR_ACC_W_Int2EnZLo(LSM303AGR_ACC_ZLIE_ENABLED);
    LSM303AGR_ACC_W_Int2_AOI(LSM303AGR_ACC_AOI_AND);

    /* Set INT2 threshold and Duration */
    LSM303AGR_ACC_W_Int2_Threshold(10);
    LSM303AGR_ACC_W_Int2_Duration(3);

    /* Enable LIR on INT2 */
    LSM303AGR_ACC_W_LatchInterrupt_on_INT2(LSM303AGR_ACC_LIR_INT2_ENABLED);
}
#endif

#ifdef FREE_FALL_TEST
/* Test Free-Fall */
static void  Loop_Test_FreeFall(void)
{
    LSM303AGR_ACC_IA_t FreeFallStatus;

    /* Clear INT2_IA event*/
    LSM303AGR_ACC_R_Int2_IA(&FreeFallStatus);

    /* configure FreeFall */
    init_LSM303AGR_FreeFall();

    while(1) {
        /* Event will be handled in driver callback */
    }
}
#endif
/*******************************************************************************
* Function Name  : main.
* Description    : Simple LSM303AGR Example.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
//int main(void)
//{
////  InitHardware();
////  I2C_MEMS_Init();

//  /*
//   * Test routines.
//   * Uncomment the one you need to exec.
//   */

//  /* Test sensor samples acquisition */
//  Loop_Test_Sample_Aquisition();

//  /* Test Wakeup */
//  //Loop_Test_Wakeup();

//  /* Test Free-Fall */
//  //Loop_Test_FreeFall();

//} // end main
