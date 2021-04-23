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

void compute_accyz_offset(void);
float compute_gravity_y(void);
float imu_compute_units(int8_t axis);

#ifdef __cplusplus
}
#endif
#endif /* IMU_H */
