#ifndef GRAVITY_H
#define GRAVITY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>
#include <sensors/mpu9250.h> //?????
#include <sensors/imu.h>


/*
 * @brief This function is called in the calibration process.
 * It gets the value of the acceleration offsets that will be used to compute correct values for the rest of the execution time.
 */
void compute_accyz_offset(void);


//A REGARDER : FUSIONNER LES DEUX FONCTIONS QUI SUIVENT?

/*
 * @brief When called, this function collects the last measured value for the acceleration on the z axis.
 * It counts how many samples have been collected :
 * - if the value is NB_SAMPLES, it returns 1 and resets the sample count
 * - otherwise, it returns 0
 */
bool collect_samples(void);


/*
 * @brief Computes mean acceleration error (difference between theoretical and measured value of gravity along z axis)
 * from measurements done by collect_samples, from NB_SAMPLES. If not enough samples have been collected, it returns 0.
 */
float get_mean_error(void);

#ifdef __cplusplus
}
#endif
#endif /* GRAVITY_H */
