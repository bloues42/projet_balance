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

#define WEIGHT_CURR		0.7
#define WEIGHT_LAST		0.3

static int16_t acc_offset_y = 0, acc_offset_z = 0;
static float grav_y;
static float grav_z = -STANDARD_GRAVITY;
static float error_grav_z = 0;
static uint8_t still_moving = 1;

/***************************INTERNAL FUNCTIONS************************************/

 /**
 * @brief   Computes the measurements of the imu into readable measurements
 * 			RAW accelerometer to m/s^2 acceleration
 * 			RAW gyroscope to rad/s speed
 */
float imu_compute_units(int8_t axis){
	float acceleration = 0;

	if (axis == Y_AXIS){
		acceleration = ((get_acc(Y_AXIS)-acc_offset_y)*STANDARD_GRAVITY * ACC_RAW2G);
	}
	else if (axis == Z_AXIS){
		acceleration = ((get_acc(Z_AXIS)-acc_offset_z)*STANDARD_GRAVITY * ACC_RAW2G) - STANDARD_GRAVITY;
	}

	return acceleration;	//CHANGE BACK ZZZ BY ACCELERATION
}


/**
* @brief   Computes the value of the projection of gravity along the y-axis using the acceleration measured
* along the z-axis of the robot.
*/
void compute_grav_direction(float acc_y, float acc_z){

	float cos_alpha = 0, sin_alpha = 0;
	static uint16_t i=0;

	if(fabs(acc_z+STANDARD_GRAVITY) < MARGIN_GRAV_Z){
		grav_y = 0;
	}else{
		cos_alpha = fabs(acc_z)/STANDARD_GRAVITY;
		sin_alpha = sqrtf(fabs(1-cos_alpha*cos_alpha));// fabs in case acc_z > STANDARD_GRAVITY for whatever reason...
		grav_y =  STANDARD_GRAVITY*sin_alpha; // >= 0

		if(acc_y < 0){	// This condition is sufficient as the epuck motor acceleration is lower than gravity acc
			grav_y = -grav_y;
		}
	}

	if (i==5){
			chprintf((BaseSequentialStream *)&SD3, "accz=%f    gravy=%f \r\n", acc_z, grav_y);
			i=0;
		}
		i++;

}

void update_grav(void){
	grav_y = imu_compute_units(Y_AXIS);
	grav_z = WEIGHT_LAST*grav_z + WEIGHT_CURR*imu_compute_units(Z_AXIS);
	//error_grav_z = grav_z + STANDARD_GRAVITY;	//we add STANDARD_GRAVITY instead of substracting because grav_z is negative

/*
	if((fabs(error_grav_z) < MARGIN_GRAV_Z) || (grav_z < -STANDARD_GRAVITY)){
			error_grav_z = 0;
	}
*/
	//we check two conditions :
	// - if the acceleration measured in z is within a certain margin of the standard gravity
	// - if the acceleration measured in z is larger than standard gravity (meaning there's been an external acceleration that disturbed the measurement)
	// in either of these cases, we keep the previous value of the error
	// else, we update the value
	// The purpose of these conditions is to ensure that equilibrium is reached before stopping.
	// Thus we only change the error to 0 if the previous error was already 0.

	if((fabs(grav_z + STANDARD_GRAVITY) < MARGIN_GRAV_Z)){
		if(still_moving == 0){
			error_grav_z = 0;
		}
		else{
			still_moving = 0;
		}
	}
	else if(grav_z >= -STANDARD_GRAVITY){
		error_grav_z = grav_z + STANDARD_GRAVITY;
		if(grav_y<0){
			error_grav_z = - error_grav_z;
		}
	}

	chprintf((BaseSequentialStream *)&SD3, "accy=%f   gravz=%f  error=%f\r\n", grav_y, grav_z, error_grav_z);
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void compute_accyz_offset(void){
    acc_offset_y = get_acc_offset(Y_AXIS);
    acc_offset_z = get_acc_offset(Z_AXIS);
}

/*
float compute_gravity_y(void){
	compute_grav_direction(imu_compute_units(Y_AXIS), imu_compute_units(Z_AXIS));
	return grav_y;
}
*/

float compute_error(void){
	update_grav();
	return error_grav_z;
}

/**************************END PUBLIC FUNCTIONS***********************************/

