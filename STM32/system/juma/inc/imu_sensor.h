#ifndef _IMU_SENSOR_H_
#define _IMU_SENSOR_H_
#include "cube_hal.h"
#include "lsm6ds3.h"
#include "lsm303agr.h"
#include "stm32f4xx_hal_msp.h"
#include "bluenrg_sdk_api.h"
/*sensor feature*/
#define IMU_SENSOR_FEATURE_ACC  0x01

#define IMU_SENSOR_FEATURE_GYRO 0x02

#define IMU_SENSOR_FEATURE_MAG  0x04

/*fifo threthold level*/
#define FIFO_WATER_MARK         6

/*FIFO CTRL MASK*/
#define LSM6DS3_XG_FIFO_THRESHOLD_MASK                      ((uint8_t)0x0F)
#define LSM6DS3_XG_FIFO_INT_THRESHOLD_MASK                 ((uint8_t)0x08)

/* Six axes sensor IO functions */
extern IMU_6AXES_StatusTypeDef LSM6DS3_IO_Init( void );
extern IMU_6AXES_StatusTypeDef LSM6DS3_IO_Write( uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,
    uint16_t NumByteToWrite );
extern IMU_6AXES_StatusTypeDef LSM6DS3_IO_Read( uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,
    uint16_t NumByteToRead );
extern void LSM6DS3_IO_ITConfig( void );

enum _imu_status{
   imu_status_ok = 0,
   imu_status_fail = 1,
};

typedef enum 
{
   OUTPUT_DISABLE = 0, 
   OUTPUT_ENABLE = !OUTPUT_DISABLE
}output_state;

typedef enum _sensor_selection{
   ACC_ENABLE           = 1,
   GYRO_ENABLE          = 2,
   ACC_AND_GYRO_ENABLE  = 3,
   MAG_ENABLE           = 4,
   ALL_ENABLE           = 7,
} sensor_selsection_t;

typedef enum sensor_data_type{
   TYPE_ACC_DATA  = 0,
   TYPE_GYRO_DATA = 1,
   TYPE_MAG_DATA  = 2,
} sensor_data_type_t;
typedef struct _imu_sensor_data_t {


    float acc[3];

    float gyro[3];

    float mag[3];

} imu_sensor_data_t;

typedef struct _imu_sensor_raw_data_t {


    int16_t acc[3];

    int16_t gyro[3];

    int16_t mag[3];

} imu_sensor_raw_data_t;

typedef struct _sensor_data_sensitivity{
   
   float acc_sensitivity;
 
   float gyro_sensitivity;
 
} imu_sensor_data_sensitivity_t;


		typedef struct
{
  float pitch;
	float roll;
	float yaw;
} imu_euler_data_t;			

	typedef struct
{
  int16_t A_X;
  int16_t A_Y;
  int16_t A_Z;
	int16_t G_X;
  int16_t G_Y;
  int16_t G_Z;
	int16_t M_X;
  int16_t M_Y;
  int16_t M_Z;
} IMU_Offset;
typedef struct _sensor_data_read_param_t{
    uint8_t delay_time;
    uint8_t group_number;
    uint32_t sample_rate;
}sensor_data_read_param_t;

typedef enum _imu_status imu_status_t;
typedef struct _imu_sensor_data_t imu_sensor_data_t; 

/*IMU Sensor API */

imu_status_t imu_sensor_reset(void);

imu_status_t imu_sensor_select_features(sensor_selsection_t features);

imu_status_t imu_sensor_set_data_rate(uint32_t* p_data_rate, uint8_t mode); 

imu_status_t imu_sensor_start(void); 

imu_status_t imu_sensor_stop(void);

void on_imu_sensor_data(imu_sensor_data_t* data); 

imu_status_t imu_sensor_read_data_from_fifo(imu_sensor_raw_data_t* Sensor_Raw_Data,imu_sensor_data_t* Sensor_Data,imu_euler_data_t* Sensor_Euler_Angle);

imu_status_t imu_sensor_filter(void);

#endif /*_IMU_SENSOR_H_*/



