#ifndef _IMU_SENSOR_FUSION_H_
#define _IMU_SENSOR_FUSION_H_
#include "cube_hal.h"
#include "imu_sensor.h"


typedef struct {
    float x, y, z;
} vec3f_t;

typedef struct {
    float w, x, y, z;
} quat4f_t;

typedef struct _sensor_fusion_angle_t{
 
   float pitch;
   
   float roll;

   float yaw;
 
} sensor_fusion_angle_t;

typedef struct _imu_sensor_fusion_1_context_t{
 
   float k_acc_1;
 
   float k_acc_2;
  
   float k_gyr_1;
 
   float k_gyr_2;
  
   float k_mag_1;
 
   float k_mag_2;
 
   float k_offset;
 
   float gyro_offset_x;
 
   float gyro_offset_y;
  
   float gyro_offset_z;

} imu_sensor_fusion_1_context_t;

typedef struct {
    uint32_t flags;
    vec3f_t gravity;

    // variables for gyroscopic calibration
    
    vec3f_t acc_min;
    vec3f_t acc_max;
    vec3f_t gyro_min;
    vec3f_t gyro_max;
    uint8_t cycle;
    vec3f_t drift_sum;
    vec3f_t drift; // gyroscopic drifting rate
} gravity_filter_context_t;

void complementary_filter(float acc_raw[3], float gyr_raw[3], float mag_raw[3], float *pitch, float *roll, float *yaw);

void imu_sensor_fusion_1(imu_sensor_data_t* sensor_raw, sensor_fusion_angle_t* sensor_angle, imu_sensor_fusion_1_context_t* sensor_context);

void gravity_filter_init(gravity_filter_context_t* cx);
void gravity_filter_run(gravity_filter_context_t* cx, imu_sensor_data_t* sensor);

#endif /*_IMU_SENSOR_FUSION_H_*/

