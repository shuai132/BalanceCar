#include "env_sensor.h"
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_hum_temp.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_imu_6axes.h"
#include "hts221.h"
#include "lps25hb.h"
#include "lps25h.h"
#include "lsm6ds3.h"
#include "lsm303agr.h"

static env_sensor_selsection_t env_sensor_selection;

/*sensor selection*/
env_status_t env_sensor_select_features(env_sensor_selsection_t features)
{
    env_sensor_selection = features;
    return env_status_ok;
}
/*sensor start*/
env_status_t env_sensor_start(void)
{
   /*tempeture and humidity start*/
   if(TEMP_HUMI_ENABLE & env_sensor_selection){
      BSP_HUM_TEMP_Init();
      /* Initialize the HUM temp */
      while(BSP_HUM_TEMP_isInitialized() != 1) {
          BSP_HUM_TEMP_Init();
          HAL_Delay(1);
      }
   }
   if(AIR_PRESSURE_ENABLE & env_sensor_selection){
        BSP_PRESSURE_Init();
        while((BSP_PRESSURE_isInitialized() != 1)) {
            BSP_PRESSURE_Init();
            HAL_Delay(1);
        } 
    }
    return env_status_ok;
}
/*get temperature*/
env_status_t env_sensor_get_temperature(float* temperature)
{
    if( BSP_HUM_TEMP_GetTemperature( temperature ) != env_status_ok){
       
       return env_status_error;
    }
    
    return env_status_ok;
}
/*get humidity*/
env_status_t env_sensor_get_humidity(float* humidity)
{
    if( BSP_HUM_TEMP_GetHumidity( humidity ) != env_status_ok){
       
       return env_status_error;
    }    
    return env_status_ok;
}
/*get tempperature and humidity*/
env_status_t env_sensor_get_temperature_humidity(float* temperature, float* humidity)
{
    if( BSP_HUM_TEMP_GetTemperature( temperature ) != env_status_ok){
       
       return env_status_error;
    } 
    if( BSP_HUM_TEMP_GetHumidity( humidity ) != env_status_ok){
       
       return env_status_error;
    }    
    return env_status_ok;
}
/*get air pressure*/
env_status_t env_sensor_get_air_pressure(float* air_pressure)
{
    if( BSP_PRESSURE_GetPressure( air_pressure ) != env_status_ok ){
     
        return env_status_error;
    }
    return env_status_ok;
}

