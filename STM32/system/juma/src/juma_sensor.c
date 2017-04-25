
#include "juma_sensor.h"
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_hum_temp.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_imu_6axes.h"
#include "hts221.h"
#include "lps25hb.h"
#include "lps25h.h"
#include "lsm6ds3.h"
#include "lsm303agr.h"

static void hum_temp_monitor_init(void);
static void pressure_sensor_init(void);
static void lsm6ds3_6_axis_init(void);
static void lsm303agr_init(void);

static void read_pressure(void* args);
static void read_temp_hum(void* args);
static void read_6_Axis_data(void* args);
static void read_raw_magnetometer_data(void* args);

/**
 * Should be implemented by application user.
 */
__weak void jsensor_app_set_sensors(void)
{
    return ;
}

/**
 * Should be called from application user.
 *
 * TODO: the input parameter is by sensor type or id?
 */
void jsensor_app_set_sensor(uint16_t sid)
{
    if (sid == JSENSOR_TYPE_HUMITY_TEMP) {
        hum_temp_monitor_init();
    }
    else if (sid == JSENSOR_TYPE_PRESSURE) {
        pressure_sensor_init();
    }
    else if (sid == JSENSOR_TYPE_MOTION_6AXIS) {
        lsm6ds3_6_axis_init();
    }
    else if(sid == JSENSOR_TYPE_MAGNET) {
        lsm303agr_init();
    }
    return;
}

JSensor_Status jsensor_app_read_sensor(uint16_t sid, void *data)
{
    // user should pass data to store data otherwise do not call me?
    assert_param(data);

    if (sid == JSENSOR_TYPE_HUMITY_TEMP)
        read_temp_hum(data);
    else if (sid == JSENSOR_TYPE_PRESSURE)
        read_pressure(data);
    else if (sid == JSENSOR_TYPE_MOTION_6AXIS)
        read_6_Axis_data(data);
    else if (sid == JSENSOR_TYPE_MAGNET)
        read_raw_magnetometer_data(data);
    return JSENSOR_OK;
}


void __jsensor_init_sensor(void)
{
    return;
}

void jsensor_sys_init(void)
{
    // Get wantted sensor list from app user.
    jsensor_app_set_sensors();
    __jsensor_init_sensor();
}

/*init hts221*/
static void hum_temp_monitor_init(void)
{
    BSP_HUM_TEMP_Init();
    /* Initialize the HUM temp */
    while(BSP_HUM_TEMP_isInitialized() != 1) {
        BSP_HUM_TEMP_Init();
        HAL_Delay(10);
    }
}

/*init lps25hb*/
static void pressure_sensor_init(void)
{
    BSP_PRESSURE_Init();
    while((BSP_PRESSURE_isInitialized() != 1)) {
        BSP_PRESSURE_Init();
        HAL_Delay(10);
    }
}

/*init lsm6ds3*/
static void lsm6ds3_6_axis_init(void)
{
    BSP_IMU_6AXES_Init();
    while(BSP_IMU_6AXES_isInitialized() != 1)	{
        BSP_IMU_6AXES_Init();
        HAL_Delay(10);
    }
}

/*init lsm303agr*/
static void lsm303agr_init(void)
{
    int8_t ret;
#ifdef CANNON_V1
    uint8_t byte_read;

    //set hub read
    byte_read = 0x80;
    LSM6DS3_IO_Write(&byte_read,	NULL,	0x01,	1);   //enable embeded register

    byte_read = 0x3d;
    LSM6DS3_IO_Write(&byte_read,	NULL,	0x02,	1);   //set lis3mdl i2c address and read mode

    byte_read = 0x68;
    LSM6DS3_IO_Write(&byte_read,	NULL,	0x03,	1);   //set register address as 0x68

    byte_read = 0x06;
    LSM6DS3_IO_Write(&byte_read,	NULL, 0x04,	1);   //6 register length


    byte_read = 0x00;
    LSM6DS3_IO_Write(&byte_read,	NULL, 0x01,	1);   //disable embeded register

    //byte_read = 0x04;
    byte_read = 0x3c;
    LSM6DS3_IO_Write(&byte_read,	NULL, 0x19,	1);   //enable embeded funciton

    byte_read = 0x09;
    LSM6DS3_IO_Write(&byte_read,	NULL, 0x1a,	1);   //enable pull_up and master
#endif

#ifdef CANNON_V2
    ret = init_LSM303AGR_mag(LSM303AGR_MAG_ODR_100Hz);
    while(ret == -1)
    {

    }
#endif
}

/*Accept humidity,tempreture data*/
static void read_temp_hum(void* args)
{
    JSensor_HUM_TEMP_Typedef *p = (JSensor_HUM_TEMP_Typedef *)args;

    uint16_t temp = 0;
    float fTmp = 0;

    if (p->humidity != NULL) {
        BSP_HUM_TEMP_GetHumidity(&fTmp);
        temp = (uint16_t)(fTmp*100);
        *p->humidity = ((temp >> 8) & 0xFF) | ((temp & 0xFF) << 8);
    }

    if (p->temperature != NULL) {
        BSP_HUM_TEMP_GetTemperature(&fTmp);
        temp = (uint16_t)(fTmp*100);
        *p->temperature = ((temp >> 8) & 0xFF) | ((temp & 0xFF) << 8);
    }

    return;
}
/*Accept pressure data*/
static void read_pressure(void* args)
{
    float fPress = 0;
    uint32_t pressure;

    JSensor_Press_Typedef *p = (JSensor_Press_Typedef *)args;

    if (p->pressure != NULL) {
        /*pressure*/
        BSP_PRESSURE_GetPressure(&fPress);

        pressure = (uint32_t)(fPress*100);
        *p->pressure = (
                           (((pressure >> 16) & 0xFF) << 0)  |
                           (((pressure >> 8)  & 0xFF) << 8)  |
                           (((pressure >> 0)  & 0xFF)  << 16)
                       );
    }
}

/*Read 6 Axis Data*/
static void read_6_Axis_data(void* args)
{
    Axes_TypeDef pData = {0};

    JSensor_AXIS_Typedef *p = (JSensor_AXIS_Typedef *)args;

    if (p->ACC) {
        BSP_IMU_6AXES_X_GetAxes(&pData);
        p->ACC[0] = (pData.AXIS_X >> 8) & 0xFF;
        p->ACC[1] = (pData.AXIS_X) & 0xFF;
        p->ACC[2] = (pData.AXIS_Y >> 8) & 0xFF;
        p->ACC[3] = (pData.AXIS_Y) & 0xFF;
        p->ACC[4] = (pData.AXIS_Z >> 8) & 0xFF;
        p->ACC[5] = (pData.AXIS_Z) & 0xFF;
    }
    if (p->GRO) {
        BSP_IMU_6AXES_G_GetAxes(&pData);
        p->GRO[0] = (pData.AXIS_X >> 8) & 0xFF;
        p->GRO[1] = (pData.AXIS_X) & 0xFF;
        p->GRO[2] = (pData.AXIS_Y >> 8) & 0xFF;
        p->GRO[3] = (pData.AXIS_Y) & 0xFF;
        p->GRO[4] = (pData.AXIS_Z >> 8) & 0xFF;
        p->GRO[5] = (pData.AXIS_Z) & 0xFF;
    }

}
/*Read raw magnetometer data*/
static void read_raw_magnetometer_data(void* args)
{
    int32_t _Magnetic_mGa[3] = {0};
    JSensor_MAG_Typedef *p = (JSensor_MAG_Typedef *)args;
    
#ifdef CANNON_V1
    uint8_t *mag = p->MAG;
    LSM6DS3_IO_Read(mag, NULL, 0x2e, 6); //read magnetometer data from sensorhub_reg[0]
#endif

#ifdef CANNON_V2

    LSM303AGR_MAG_Get_Magnetic(_Magnetic_mGa);
    p->MAG[0] = (_Magnetic_mGa[0] >> 8) & 0xFF;
    p->MAG[1] = (_Magnetic_mGa[0]) & 0xFF;
    p->MAG[2] = (_Magnetic_mGa[1] >> 8) & 0xFF;
    p->MAG[3] = (_Magnetic_mGa[1] ) & 0xFF;
    p->MAG[4] = (_Magnetic_mGa[2] >> 8) & 0xFF;
    p->MAG[5] = (_Magnetic_mGa[2]) & 0xFF;
#endif
}
