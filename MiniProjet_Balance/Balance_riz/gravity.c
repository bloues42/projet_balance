#include "gravity.h"

#include <stdio.h>
#include <stdlib.h>

#include <ch.h>
#include <hal.h>
#include <math.h>

/*
 * The defines are partially from TP3 code.
 */
#define STANDARD_GRAVITY    9.80665f
#define RES_2G				2.0f
#define MAX_INT16			32768.0f
#define ACC_RAW2G			(RES_2G / MAX_INT16)

#define NB_SAMPLES			10
#define DIFF_SIGN_MIN		NB_SAMPLES/2
#define MARGIN_GRAV_Z  		0.045f
#define THRESHOLD_SHIFT		0.035

/***************************MODULE STATIC VARIABLES************************************/
static int16_t acc_offset_y = 0, acc_offset_z = 0;
static float grav_y = 0;
static float mean_error = 0;
static float sum_grav_z = 0;
static bool still_moving = 1;
static bool all_samples_collected = 0;


/***************************INTERNAL FUNCTIONS************************************/

 /**
 * @brief   Computes the measurements of the imu into readable measurements
 * 			RAW accelerometer to m/s^2 acceleration
 */
float imu_compute_units(int8_t axis){
	float acceleration = 0;

	if (axis == Y_AXIS){
		acceleration = ((get_acc(Y_AXIS)-acc_offset_y)*STANDARD_GRAVITY * ACC_RAW2G);
	}
	else if (axis == Z_AXIS){
		acceleration = ((get_acc(Z_AXIS)-acc_offset_z)*STANDARD_GRAVITY * ACC_RAW2G) - STANDARD_GRAVITY;
	}

	return acceleration;
}


/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void compute_accyz_offset(void){
    acc_offset_y = get_acc_offset(Y_AXIS);
    acc_offset_z = get_acc_offset(Z_AXIS);
}


/*
 * For each sample, we check if the acceleration measured in z is larger than standard gravity (meaning there's been
 * an external acceleration that disturbed the measurement)
 * 	 -> we only take the samples with a magnitude smaller than STANDARD_GRAVITY
 *
 * For each group of NB_SAMPLES, we check if there are significantly more samples measured while the acceleration
 * in y was positive (nb_pos) rather than negative (nb_neg) or vice versa
 * 	 -> too similar nb_neg and nb_pos (|nb_pos-nb_neg| < DIFF_SIGN_MIN) might indicate oscillation
 * 	 	so sum_grav_z is set to 0
 */
bool collect_samples(void){
	static uint8_t samples_collected_nb = 0;
	static uint8_t nb_neg = 0, nb_pos = 0;
	grav_y = imu_compute_units(Y_AXIS);
	float error = imu_compute_units(Z_AXIS) + STANDARD_GRAVITY;

	if(error >= 0){
		if(grav_y<0){
			error = - error;
			nb_neg++;
		}else{
			nb_pos++;
		}
		all_samples_collected = 0;
		samples_collected_nb++;
		sum_grav_z += error;
	}

	if(samples_collected_nb == NB_SAMPLES){
		if(abs(nb_neg-nb_pos)<DIFF_SIGN_MIN){
			sum_grav_z = 0;
		}
		all_samples_collected = 1;
		samples_collected_nb = 0;
		nb_neg = 0;
		nb_pos = 0;
		return 1;
	}

	return 0;
}

/*
 * Computes the mean of the sampled errors, mean_error.
 * Then checks whether it's within a certain domain around zero (the domain is asymmetrical).
 * To ensure that equilibrium is reached before stopping the robot, mean_error has to have been set to zero
 * for two consecutive function calls, in which case the function returns zero (indicating the motors will stop).
 */
float get_mean_error(void){
	if(!(all_samples_collected)){
		return 0;
	}
	float mean_temp = (sum_grav_z/NB_SAMPLES);
	sum_grav_z = 0;

	if((mean_temp < MARGIN_GRAV_Z+THRESHOLD_SHIFT) && (mean_temp > -MARGIN_GRAV_Z+THRESHOLD_SHIFT)){
		if(still_moving == 0){
			mean_error = 0;
		}else{
			still_moving = 0;
			mean_error = mean_temp;
		}
	}
	else{
		mean_error = mean_temp;
		still_moving = 1;
	}

	return mean_error;
}

/**************************END PUBLIC FUNCTIONS***********************************/

