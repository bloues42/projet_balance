#include "regulator.h"

#include "ch.h"
#include "hal.h"
#include <math.h>

#include <gravity.h>

#define KP_LINEAR				2000.0f
#define SPEED_MIN				200
#define ROTATION_THRESHOLD		5
#define BACK_WALL_STOP			40
#define SIDE_STOP				300

/*
 * Two different KD because the distance-value characteristic is non-linear
 * (much higher deltas when the sensor is close to a wall).
 */
#define KD_ROTATION_BACK		2.0f
#define KD_ROTATION_FORWARD		1.0f

enum	IR_sensor_pos{IR_FRONT_RIGHT=0, IR_FRONT_RIGHT45, IR_RIGHT, IR_BACK_RIGHT,
						IR_BACK_LEFT, IR_LEFT, IR_FRONT_LEFT45, IR_FRONT_LEFT};


/*
 * @brief Proportional regulator for linear speed of the robot.
 * The input error is zero under a certain threshold
 */
int16_t linear_regulator(float error){
	return (int16_t)(KP_LINEAR*error);
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
	//using the value from the sensor closest to a side, to have better sensitivity
		if(fr_left > fr_right){
			speed_correction = KD_ROTATION_FORWARD * (fr_left-last_fr_left);
		}

		if(fr_right > fr_left){
			speed_correction = - KD_ROTATION_FORWARD * (fr_right-last_fr_right);
		}

		last_fr_left = fr_left;
		last_fr_right = fr_right;

	//checking if the robot is too close to a side wall, in which case turning towards the wall is not allowed
		if((left > SIDE_STOP) && (speed_correction < 0)){
			speed_correction = 0;
		}

		if((right > SIDE_STOP) && (speed_correction > 0)){
			speed_correction = 0;
		}

	}


	else if(speed < 0){
	//using the value from the sensor closest to a side, to have better sensitivity
		if(fr_left > fr_right){
			speed_correction = - KD_ROTATION_BACK * (fr_left-last_fr_left);
		}

		if(fr_right > fr_left){
			speed_correction = KD_ROTATION_BACK * (fr_right-last_fr_right);
		}

		last_fr_left = fr_left;
		last_fr_right = fr_right;

	//checking if the robot is too close to a side wall, in which case turning towards the wall is not allowed
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
        
        //the function collect_samples returns 1 if it has collected all the samples it needs to compute the mean
        //else, it returns 0
        if(collect_samples()){
			//computes the speed to give to the motors
			//the value of the error is obtained from the mean of the collected samples
        	speed = linear_regulator(get_mean_error());
        	speed_correction = rotation_regulator(speed);

        	//applies the speed from the regulator and the correction for the rotation
			right_motor_set_speed(speed - speed_correction);
			left_motor_set_speed(speed + speed_correction);
        }


		//checks if the robot is too close to the back wall
		//in which case, it only allows the robot to move forward
		if((get_calibrated_prox(IR_BACK_LEFT) > BACK_WALL_STOP) && (get_calibrated_prox(IR_BACK_RIGHT) > BACK_WALL_STOP)){
			if(speed < 0){
				right_motor_set_speed(0);
				left_motor_set_speed(0);
			}
		}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void regulator_start(void){
	chThdCreateStatic(waRegulator, sizeof(waRegulator), NORMALPRIO, Regulator, NULL);
}
