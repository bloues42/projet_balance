#include "regulator.h"

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <gravity.h>
#include <main.h>
#include <motors.h>
#include <sensors/proximity.h>

//simple regulator implementation
//error is zero under a certain threshold
int16_t regulator(float error){
	static float last_error = 0;
	float speed = 0;
	int8_t signe = ((error > 0) ? 1 : ((error < 0) ? -1 : 0));

	speed = KP * error; //+ signe * SPEED_MIN;

	last_error = error;
    return (int16_t)speed;
}

int16_t rotation_regulator(int16_t speed){
	static uint16_t last_fr_right = 0;
	static uint16_t last_fr_left = 0;
	float speed_correction = 0;
	uint16_t rot_error;
	int32_t fr_left = 0, fr_right = 0, left = 0, right = 0;

	//ir_left = get_calibrated_prox(IR_LEFT);
	//ir_right = get_calibrated_prox(IR_RIGHT);

	left = get_calibrated_prox(IR_LEFT);
	right = get_calibrated_prox(IR_RIGHT);

	fr_left = get_calibrated_prox(IR_FRONT_LEFT45);
	fr_right = get_calibrated_prox(IR_FRONT_RIGHT45);

	if(speed > 0){
		//computes a correction factor to let the robot rotate in the middle of the platform
		if(fr_left > fr_right){
			speed_correction = ROTATION_COEFF * (fr_left-last_fr_left);
		}

		if(fr_right > fr_left){
			speed_correction = - ROTATION_COEFF * (fr_right-last_fr_right);
		}

		last_fr_left = fr_left;
		last_fr_right = fr_right;

		if((left > SIDE_STOP) && (speed_correction < 0)){
			speed_correction = 0;
		}

		if((right > SIDE_STOP) && (speed_correction > 0)){
			speed_correction = 0;
		}

	}


	else if(speed < 0){
		//computes a correction factor to let the robot rotate in the middle of the platform
		if(fr_left > fr_right){
			speed_correction = - ROTATION_COEFF * (fr_left-last_fr_left);
		}

		if(fr_right > fr_left){
			speed_correction = ROTATION_COEFF * (fr_right-last_fr_right);
		}

		last_fr_left = fr_left;
		last_fr_right = fr_right;

		if((left > SIDE_STOP) && (speed_correction > 0)){
			speed_correction = 0;
		}

		if((right > SIDE_STOP) && (speed_correction < 0)){
			speed_correction = 0;
		}
	}else{
		speed_correction = 0;
	}

	//speed_correction = ROTATION_COEFF * (rot_error-last_rot_error);

	//last_rot_error = rot_error;

	chprintf((BaseSequentialStream *)&SD3, "frontleft=%d   frontright=%d    correction=%f \r\n", fr_left, fr_right, speed_correction);
    return (int16_t)speed_correction;
}

static THD_WORKING_AREA(waRegulator, 256);
static THD_FUNCTION(Regulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    static int16_t speed = 0;
    int16_t speed_correction = 0;
    static uint16_t j = 0, k = 0;
    int32_t ir_back;


    while(1){
        time = chVTGetSystemTime();
        
        //the function collect samples returns 1 if it has collected all the samples it needs to compute the mean
        //else, it returns 0
        if(collect_samples()){

			//computes the speed to give to the motors
			//the value of the error is obtained from the mean of the collected samples
        	speed = regulator(get_mean_error());
        	speed_correction = rotation_regulator(speed);
        }

/* ALLER_RETOUR
        if(j<=300){
			speed = -1;
			j++;
		}
		else if(j>300){
			speed = 1;
			j++;
		}
        if(j==600){
        	j=0;
        }

        if(k==10){
    	//compute speed correction to straighten the robot
        	speed_correction = rotation_regulator(speed);
    		//chprintf((BaseSequentialStream *)&SD3, "frontleft=%d   frontright=%d    correction=%d \r\n", get_calibrated_prox(IR_FRONT_LEFT45), get_calibrated_prox(IR_FRONT_RIGHT45), speed_correction);
    		k=0;
    	}
    	 k++;
*/
		//chprintf((BaseSequentialStream *)&SD3, "mean_err=%f speed=%d \r\n", mean, speed);
		//chprintf((BaseSequentialStream *)&SD3, "frontleft=%d   frontright=%d    LEFT=%d   RIGHT=%d \r\n", get_calibrated_prox(IR_FRONT_LEFT45), get_calibrated_prox(IR_FRONT_RIGHT45), get_calibrated_prox(IR_LEFT),get_calibrated_prox(IR_RIGHT));



		//checks if the robot is too close to the back wall
		//in which case, it only allows the robot to move forward
		ir_back = (get_calibrated_prox(IR_BACK_LEFT) + get_calibrated_prox(IR_BACK_RIGHT))/2;
		if(ir_back > BACK_WALL_STOP){
			if(speed < 0){
				speed = 0;
				speed_correction = 0;
			}
		}

		//applies the speed from the regulator and the correction for the rotation
		right_motor_set_speed(speed - speed_correction);
		left_motor_set_speed(speed + speed_correction);



		//right_motor_set_speed(500*speed - speed_correction);
		//left_motor_set_speed(500*speed + speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void regulator_start(void){
	chThdCreateStatic(waRegulator, sizeof(waRegulator), NORMALPRIO, Regulator, NULL);
}
