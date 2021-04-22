#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>
#include <sensors/mpu9250.h>
#include <msgbus/messagebus.h>
#include <sensors/imu.h>

/** Message containing one measurement from the IMU. */
typedef struct {
    float acceleration[NB_AXIS]; // m/s^2
    float gyro_rate[NB_AXIS]; // rad/s
    float temperature;
    float magnetometer[NB_AXIS]; //uT
    int16_t acc_raw[NB_AXIS]; //raw values
    int16_t gyro_raw[NB_AXIS]; //raw values
    int16_t acc_offset[NB_AXIS]; //raw offsets
    int16_t gyro_offset[NB_AXIS]; //raw offsets
    int16_t acc_filtered[NB_AXIS];
    int16_t gyro_filtered[NB_AXIS];
    uint8_t status;
} imu_msg_t;

void get_accyz_offset(void);

float imu_compute_units(int8_t axis);

float compute_direction(float acc_y, float acc_z);

//void imu_compute_offset(messagebus_topic_t * imu_topic, uint16_t nb_samples);

 /**
 * @brief   Returns the last accelerometer value measured
 *          for the axis given
 *
 * @param axis      0-2, respectively x,y or z
 *
 * @return          Last accelerometer value measured
 */



#ifdef __cplusplus
}
#endif
#endif /* IMU_H */
