#include "imu_sensor_fusion.h"
#include "imu_sensor.h"
#include "math.h"
#include "app.h"

#define GYROSCOPE_SENSITIVITY     4.375
#define ACCELERATOR_SENSITIVITY   0.061
#define GYRO_DEGREE_OFFSET        3.0
#define ACC_1_G                   1000.0

#define M_PI 3.1415926

#define dt 0.00125							// 1.25 ms sample rate!    

void complementary_filter(float acc_raw[3], float gyr_raw[3], float mag_raw[3], float *pitch, float *roll, float *yaw)
{
    float pitch_acc, roll_acc, yaw_mag;

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += (double)gyr_raw[0] * dt; // Angle around the X-axis
    *roll -= (double)gyr_raw[1] * dt;    // Angle around the Y-axis
    *yaw  += (double)gyr_raw[2] * dt;
//    // Turning around the X axis results in a vector on the Y-axis
    pitch_acc = (double)atan2f(acc_raw[0], acc_raw[2]) * 180 / M_PI;
    *pitch = (double)*pitch * 0.98 + (double)pitch_acc * 0.02;
    // Turning around the Y axis results in a vector on the X-axis
    roll_acc = (double)atan2f(acc_raw[1], acc_raw[2]) * 180 / M_PI;
    *roll = (double)*roll * 0.98 + (double)roll_acc * 0.02;
    yaw_mag = (double)atan2f(mag_raw[1], mag_raw[0]) * 180 / M_PI;
    *yaw = (double)*yaw * 0.98 + (double)yaw_mag * 0.02;
}

void imu_sensor_fusion_1(imu_sensor_data_t* sensor_raw, sensor_fusion_angle_t* sensor_angle,  imu_sensor_fusion_1_context_t* sensor_context)
{
    float k_acc, k_gyr, k_mag;
    float delta_gyr, delta_acc, delta_mag, delta_angle;
    float forceMagnitudeApprox, gyro_offset ;

    /*gyro factor*/
    forceMagnitudeApprox = sqrt(abs(sensor_raw->gyro[0]) * abs(sensor_raw->gyro[0]) +
                                abs(sensor_raw->gyro[1]) * abs(sensor_raw->gyro[1]) +
                                abs(sensor_raw->gyro[2]) * abs(sensor_raw->gyro[2]));

    gyro_offset = sqrt( (sensor_context->gyro_offset_x * sensor_context->gyro_offset_x) +
                        (sensor_context->gyro_offset_y * sensor_context->gyro_offset_y) +
                        (sensor_context->gyro_offset_z * sensor_context->gyro_offset_z));

    if(forceMagnitudeApprox > gyro_offset)
    {
        k_gyr = sensor_context->k_gyr_1;
    } else {
        k_gyr = sensor_context->k_gyr_2;
    }

    /*acc factor*/
    forceMagnitudeApprox = sqrt(abs(sensor_raw->acc[0]) * abs(sensor_raw->acc[0]) +
                                abs(sensor_raw->acc[1]) * abs(sensor_raw->acc[1]) +
                                abs(sensor_raw->acc[2]) * abs(sensor_raw->acc[2]));

    if((abs((double)forceMagnitudeApprox - ACC_1_G) / ACC_1_G) > 0.01 )
    {
        k_acc = sensor_context->k_acc_2;
    } else {
        k_acc = sensor_context->k_acc_1;
    }
    
    /*mag factor*/
    k_mag = sensor_context->k_mag_1;

    /*pitch*/
    sensor_raw->gyro[0] = sensor_raw->gyro[0] -  sensor_context->gyro_offset_x;
    delta_gyr =  (double)sensor_raw->gyro[0] * dt;
    delta_acc = ((double)atan2f(sensor_raw->acc[0], sensor_raw->acc[2]) * 180.0 / M_PI) - sensor_angle->pitch;
    delta_angle = k_gyr * delta_gyr + k_acc * delta_acc;
    sensor_context->gyro_offset_x += (   sensor_context->k_offset * (  ( sensor_raw->gyro[0] - ((double)delta_angle / dt) ) -  sensor_context->gyro_offset_x  )   );
    sensor_angle->pitch += delta_angle;
    
    /*roll*/
    sensor_raw->gyro[1] = sensor_raw->gyro[1] - sensor_context->gyro_offset_y;
    delta_gyr =  (double)sensor_raw->gyro[1] * dt;
    delta_acc = ((double)atan2f(sensor_raw->acc[1], sensor_raw->acc[2]) * 180.0 / M_PI) - sensor_angle->roll;
    delta_angle = k_gyr * delta_gyr + k_acc * delta_acc;
    sensor_context->gyro_offset_y += (   sensor_context->k_offset * (  ( sensor_raw->gyro[1] - ((double)delta_angle / dt) ) - sensor_context->gyro_offset_y  )   );
    sensor_angle->roll += delta_angle;

    /*yaw*/
    sensor_raw->gyro[2] = sensor_raw->gyro[2] - sensor_context->gyro_offset_z;
    delta_gyr =  (double)sensor_raw->gyro[2] * dt;
    delta_mag = ((double)atan2f(sensor_raw->mag[1], sensor_raw->mag[0]) * 180.0 / M_PI) - sensor_angle->yaw;
    delta_angle = k_gyr * delta_gyr + k_mag * delta_mag;
    sensor_context->gyro_offset_z += (   sensor_context->k_offset * (  ( sensor_raw->gyro[2] - ((double)delta_angle / dt) ) - sensor_context->gyro_offset_z  )   );
    sensor_angle->yaw += delta_angle;
    
    
}

vec3f_t vec3f_add(vec3f_t u, vec3f_t v)
{
    vec3f_t r;

    r.x = u.x + v.x;
    r.y = u.y + v.y;
    r.z = u.z + v.z;

    return r;
}

vec3f_t vec3f_mul_scalar(float s, vec3f_t v)
{
    vec3f_t r;

    r.x = s*v.x;
    r.y = s*v.y;
    r.z = s*v.z;

    return r;
}

vec3f_t vec3f_mul_cross(vec3f_t u, vec3f_t v)
{
    vec3f_t r;

    r.x = u.y * v.z - u.z * v.y;
    r.y = u.z * v.x - u.x * v.z;
    r.z = u.x * v.y - u.y * v.x;
    
    return r;
}

float vec3f_mul_dot(vec3f_t u, vec3f_t v)
{
    return (u.x * v.x + u.y * v.y + u.z * v.z);
}

// computer the vector caused by rotation q on vector v
vec3f_t vec3f_rotate(vec3f_t v, quat4f_t q)
{
    float w;
    vec3f_t r, wv, rv, tmp1, tmp2, delta, result;
    
    w = q.w;
    r.x = q.x;
    r.y = q.y;
    r.z = q.z;

    // v + 2r X (r X v + wv)

    // computer wv
    wv = vec3f_mul_scalar(w, v);
    // computer r X v
    rv = vec3f_mul_cross(r, v);
    // computer r X v + wv
    tmp1 = vec3f_add(rv, wv);
    // computer 2r
    tmp2 = vec3f_mul_scalar(2, r);
    // computer delta
    delta = vec3f_mul_cross(tmp2, tmp1);
    // computer final result
    result = vec3f_add(v, delta);
    
    return result;
}

float vec3f_magnitude(vec3f_t v)
{
    float magnitude;
    
    magnitude = sqrt(v.x * v.x +
                     v.y * v.y +
                     v.z * v.z);

    return magnitude;
}

void gravity_filter_init(gravity_filter_context_t* cx)
{
    memset(cx, 0, sizeof(gravity_filter_context_t));
}

#define ZERO_MOTION_THRESHOLD_ACC     10
#define ZERO_MOTION_THRESHOLD_GYRO    3
#define ZERO_MOTION_CYCLES        100
#define K_GYRO_DRIFT              0.1

void gravity_filter_run(gravity_filter_context_t* cx, imu_sensor_data_t* sensor)
{
    quat4f_t q;
    vec3f_t acc_norm, angular_rate, axis_angles, e, gravity_new;
    float acc_magnitude, gravity_magnitude, err_magnitude, angle_magnitude, gain;
    
    acc_magnitude = sqrt(sensor->acc[0] * sensor->acc[0] +
                         sensor->acc[1] * sensor->acc[1] +
                         sensor->acc[2] * sensor->acc[2]);

    // normalize sensed gravity
    acc_norm.x = (double)1000.0 * sensor->acc[0] / acc_magnitude;
    acc_norm.y = (double)1000.0 * sensor->acc[1] / acc_magnitude;
    acc_norm.z = (double)1000.0 * sensor->acc[2] / acc_magnitude;
    
    if (cx->flags == 0) {
        cx->gravity.x = acc_norm.x;
        cx->gravity.y = acc_norm.y;
        cx->gravity.z = acc_norm.z;
        cx->flags = 1;
        return;
    }

    angular_rate.x = sensor->gyro[0] - cx->drift.x;
    angular_rate.y = sensor->gyro[1] - cx->drift.y;
    angular_rate.z = sensor->gyro[2] - cx->drift.z;
    
    // computer axis angles measured by gyroscopic
    axis_angles.x = -((double)angular_rate.x * dt * M_PI / 180.0);
    axis_angles.y = -((double)angular_rate.y * dt * M_PI / 180.0);
    axis_angles.z = -((double)angular_rate.z * dt * M_PI / 180.0);

    // form a quaternion
    // hereby, we assume the angles are tiny
    q.w = 1;
    q.x = axis_angles.x / 2;
    q.y = axis_angles.y / 2;
    q.z = axis_angles.z / 2;

    // update gravity vector by rotation
    gravity_new = vec3f_rotate(cx->gravity, q);
    
    // gyroscopic autocalibration
    if (cx->cycle < ZERO_MOTION_CYCLES) {
        if (cx->cycle == 0) {
            cx->acc_min.x = acc_norm.x;
            cx->acc_min.y = sensor->acc[1];
            cx->acc_min.z = sensor->acc[2];
            cx->acc_max.x = sensor->acc[0];
            cx->acc_max.y = sensor->acc[1];
            cx->acc_max.z = sensor->acc[2];
            
            cx->gyro_min.x = sensor->gyro[0];
            cx->gyro_min.y = sensor->gyro[1];
            cx->gyro_min.z = sensor->gyro[2];
            cx->gyro_max.x = sensor->gyro[0];
            cx->gyro_max.y = sensor->gyro[1];
            cx->gyro_max.z = sensor->gyro[2];
            cx->drift_sum.x = 0;
            cx->drift_sum.y = 0;
            cx->drift_sum.z = 0;
        } else {
            if (sensor->acc[0] < cx->acc_min.x) cx->acc_min.x = sensor->acc[0];
            if (sensor->acc[1] < cx->acc_min.y) cx->acc_min.y = sensor->acc[1];
            if (sensor->acc[2] < cx->acc_min.z) cx->acc_min.z = sensor->acc[2];
            if (sensor->acc[0] > cx->acc_max.x) cx->acc_max.x = sensor->acc[0];
            if (sensor->acc[1] > cx->acc_max.y) cx->acc_max.y = sensor->acc[1];
            if (sensor->acc[2] > cx->acc_max.z) cx->acc_max.z = sensor->acc[2];
            
            if (sensor->gyro[0] < cx->gyro_min.x) cx->gyro_min.x = sensor->gyro[0];
            if (sensor->gyro[1] < cx->gyro_min.y) cx->gyro_min.y = sensor->gyro[1];
            if (sensor->gyro[2] < cx->gyro_min.z) cx->gyro_min.z = sensor->gyro[2];
            if (sensor->gyro[0] > cx->gyro_max.x) cx->gyro_max.x = sensor->gyro[0];
            if (sensor->gyro[1] > cx->gyro_max.y) cx->gyro_max.y = sensor->gyro[1];
            if (sensor->gyro[2] > cx->gyro_max.z) cx->gyro_max.z = sensor->gyro[2];
        }
        cx->drift_sum.x += sensor->gyro[0];
        cx->drift_sum.y += sensor->gyro[1];
        cx->drift_sum.z += sensor->gyro[2];
        cx->cycle++;
    }

    if (cx->cycle >= ZERO_MOTION_CYCLES) {
        if ((abs(cx->acc_max.x - cx->acc_min.x) < ZERO_MOTION_THRESHOLD_ACC) &&
            (abs(cx->acc_max.y - cx->acc_min.y) < ZERO_MOTION_THRESHOLD_ACC) &&
            (abs(cx->acc_max.z - cx->acc_min.z) < ZERO_MOTION_THRESHOLD_ACC)&&
            (abs(cx->gyro_max.x - cx->gyro_min.x) < ZERO_MOTION_THRESHOLD_GYRO) &&
            (abs(cx->gyro_max.y - cx->gyro_min.y) < ZERO_MOTION_THRESHOLD_GYRO) &&
            (abs(cx->gyro_max.z - cx->gyro_min.z) < ZERO_MOTION_THRESHOLD_GYRO)) {
            cx->drift.x += (double)K_GYRO_DRIFT * (cx->drift_sum.x/ZERO_MOTION_CYCLES - cx->drift.x);
            cx->drift.y += (double)K_GYRO_DRIFT * (cx->drift_sum.y/ZERO_MOTION_CYCLES - cx->drift.y);
            cx->drift.z += (double)K_GYRO_DRIFT * (cx->drift_sum.z/ZERO_MOTION_CYCLES - cx->drift.z);
        }
        cx->cycle = 0;
    }

    // computer prediction error according to accelerometer's measurements
    e.x = acc_norm.x - gravity_new.x;
    e.y = acc_norm.y - gravity_new.y;
    e.z = acc_norm.z - gravity_new.z;
    if (cx->cycle == 0) printf("%f ", angular_rate.x);
    err_magnitude = vec3f_magnitude(e);
    angle_magnitude = vec3f_magnitude(angular_rate);

    // todo
    // should revise the gain
    // the principle is:
    //   * when accelerometer reading is jumping around, filter it out
    //   * otherwise, we should follow it quickly
    
    if (err_magnitude < 10) {
        // gyroscopic predicted result matches accelerometer readings
        // we are lucky!
        gain = 0.1;
    } else {
        // gyroscopic does not agree with accelerometer
        if ((angle_magnitude < 100) &&
            (abs(acc_magnitude - 1000) > 40)) {
            // angular rate is low & acceleration is high
            // most likely there is high linear acceleration
            // so we don't trust accelerometer now
            gain = 0.0001;
        } else {
            // looks like there is rapid motion.
            // both gyroscopic and accelerometer are noisy in this case
            // follow accelerometer to prevent big lag
            gain = 0.5;
        }
    }

    // computer error estimation
    e = vec3f_mul_scalar(gain, e);
    
    // correct predicted gravity
    cx->gravity = vec3f_add(gravity_new, e);

    // renormalise
    gravity_magnitude = sqrt(cx->gravity.x * cx->gravity.x +
                         cx->gravity.y * cx->gravity.y +
                         cx->gravity.z * cx->gravity.z);

    cx->gravity.x = (double)1000.0 * cx->gravity.x / gravity_magnitude;
    cx->gravity.y = (double)1000.0 * cx->gravity.y / gravity_magnitude;
    cx->gravity.z = (double)1000.0 * cx->gravity.z / gravity_magnitude;
}

