#include "gravity.h"

#include <ch.h>
#include <hal.h>
#include <math.h>
//#include <msgbus/messagebus.h>
#include <chprintf.h>

#define STANDARD_GRAVITY    9.80665f
#define DEG2RAD(deg) 		(deg / 180 * M_PI)

#define RES_2G		2.0f
#define RES_250DPS	250.0f
#define MAX_INT16	32768.0f

#define ACC_RAW2G		(RES_2G / MAX_INT16)
#define GYRO_RAW2DPS	(RES_250DPS / MAX_INT16)

#define GRAV_Y_MARGE 0.1 	// TO CHANGE

static int16_t acc_offset_y = 0, acc_offset_z = 0;

/***************************INTERNAL FUNCTIONS************************************/

 /**
 * @brief   Computes the measurements of the imu into readable measurements
 * 			RAW accelerometer to m/s^2 acceleration
 * 			RAW gyroscope to rad/s speed
 */


void get_accyz_offset(void){
    acc_offset_y = get_acc_offset(Y_AXIS);
    acc_offset_z = get_acc_offset(Z_AXIS);
}

float imu_compute_units(int8_t axis){
	/*
    *   TASK 10 : TO COMPLETE
    */
	float acceleration = 0;

	if (axis == Y_AXIS){
		acceleration = ((get_acc(Y_AXIS)-acc_offset_y)*STANDARD_GRAVITY * ACC_RAW2G);
	}
	else if (axis == Z_AXIS){
		acceleration = ((get_acc(Z_AXIS)-acc_offset_z)*STANDARD_GRAVITY * ACC_RAW2G) - STANDARD_GRAVITY;
	}

	return acceleration;
}


float compute_direction(float acc_y, float acc_z){

	float cos_alpha = 0, sin_alpha = 0, grav_y = 0;

	cos_alpha = fabs(acc_z)/STANDARD_GRAVITY;
	sin_alpha = sqrt(1-cos_alpha*cos_alpha);
	grav_y = STANDARD_GRAVITY*sin_alpha; // >= 0

	if(acc_y < -GRAV_Y_MARGE){	// This condition is sufficient as the epuck motor acceleration is lower than gravity acc
		grav_y = -grav_y;
	}else if(fabs(acc_y) < GRAV_Y_MARGE){
		grav_y = 0;
	}

	/*
	if(fabs(acc_y) < (grav_y - GRAV_Y_MARGE)){
		if(acc_y > 0)
			grav_y =  -grav_y;

	}else if(fabs(acc_y) > (grav_y + GRAV_Y_MARGE)){
		if(acc_y < 0)
			grav_y = - grav_y;
	}else{
		grav_y = 0;
	}
	*/

	return grav_y;
}


 /**
 * @brief   Thread which updates the measurements and publishes them
 */


/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/
 /*
void imu_compute_offset(messagebus_topic_t * imu_topic, uint16_t nb_samples){


    *   TASK 9 : TO COMPLETE


	//imu_msg_t curr_values;
	int32_t tot_gyro[NB_AXIS]={0}, tot_acc[NB_AXIS]={0};

	for(uint16_t s=0; s<nb_samples; s++){
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		tot_gyro[X_AXIS] += imu_values.gyro_raw[X_AXIS];
		tot_gyro[Y_AXIS] += imu_values.gyro_raw[Y_AXIS];
		tot_gyro[Z_AXIS] += imu_values.gyro_raw[Z_AXIS];

		tot_acc[X_AXIS] += imu_values.acc_raw[X_AXIS];
		tot_acc[Y_AXIS] += imu_values.acc_raw[Y_AXIS];
		tot_acc[Z_AXIS] += imu_values.acc_raw[Z_AXIS];
	}


	tot_gyro[X_AXIS] /= nb_samples;
	imu_values.gyro_offset[X_AXIS] = tot_gyro[X_AXIS];

	imu_values.gyro_offset[Y_AXIS] = tot_gyro[Y_AXIS]/nb_samples;
	imu_values.gyro_offset[Z_AXIS] = tot_gyro[Z_AXIS]/nb_samples;


	imu_values.acc_offset[X_AXIS] = tot_acc[X_AXIS]/nb_samples;
//	imu_values.acc_offset[Y_AXIS] = tot_acc[Y_AXIS]/nb_samples;
	//pour z, il faut ajouter la gravité
	imu_values.acc_offset[Z_AXIS] = (tot_acc[Z_AXIS]/nb_samples) + (MAX_INT16 / RES_2G);

	palClearPad(GPIOB, GPIOB_LED_BODY);

	//chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
	 //               imu_values.acc_offset[X_AXIS], imu_values.acc_offset[Y_AXIS], imu_values.acc_offset[Z_AXIS],
	 //               imu_values.gyro_offset[X_AXIS], imu_values.gyro_offset[Y_AXIS], imu_values.gyro_offset[Z_AXIS]);

}
*/

/**************************END PUBLIC FUNCTIONS***********************************/

