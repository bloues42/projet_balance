#include "regulator.h"

#include "ch.h"
#include "hal.h"
#include <math.h>

#include <gravity.h>

/*
 * @brief Proportional regulator for linear speed of the robot.
 * The input error is zero under a certain threshold
 */
int16_t linear_regulator(float error){
	float speed = 0;

	speed = KP_LINEAR * error;

    return (int16_t)speed;
}


/*
 * @brief Differential regulator for rotation.
 */
int16_t rotation_regulator(int16_t speed){
	static uint16_t last_fr_right = 0;
	static uint16_t last_fr_left = 0;
	float speed_correction = 0;

	int32_t left = get_calibrated_prox(IR_LEFT);
	int32_t right = get_calibrated_prox(IR_RIGHT);

	int32_t fr_left = get_calibrated_prox(IR_FRONT_LEFT45);
	int32_t fr_right = get_calibrated_prox(IR_FRONT_RIGHT45);

	if(speed > 0){
		//computes a correction factor to let the robot rotate in the middle of the platform
		if(fr_left > fr_right){
			speed_correction = KD_ROTATION_FORWARD * (fr_left-last_fr_left);
		}

		if(fr_right > fr_left){
			speed_correction = - KD_ROTATION_FORWARD * (fr_right-last_fr_right);
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
			speed_correction = - KD_ROTATION_BACK * (fr_left-last_fr_left);
		}

		if(fr_right > fr_left){
			speed_correction = KD_ROTATION_BACK * (fr_right-last_fr_right);
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

    return (int16_t)speed_correction;
}

static THD_WORKING_AREA(waRegulator, 256);
static THD_FUNCTION(Regulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    static int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //the function collect samples returns 1 if it has collected all the samples it needs to compute the mean
        //else, it returns 0
        if(collect_samples()){
			//computes the speed to give to the motors
			//the value of the error is obtained from the mean of the collected samples
        	speed = linear_regulator(get_mean_error());
        	speed_correction = rotation_regulator(speed);
        }


		//checks if the robot is too close to the back wall
		//in which case, it only allows the robot to move forward
		if((get_calibrated_prox(IR_BACK_LEFT) > BACK_WALL_STOP) && (get_calibrated_prox(IR_BACK_RIGHT) > BACK_WALL_STOP)){
			if(speed < 0){
				speed = 0;
				speed_correction = 0;
			}
		}

		//applies the speed from the regulator and the correction for the rotation
		right_motor_set_speed(speed - speed_correction);
		left_motor_set_speed(speed + speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void regulator_start(void){
	chThdCreateStatic(waRegulator, sizeof(waRegulator), NORMALPRIO, Regulator, NULL);
}
