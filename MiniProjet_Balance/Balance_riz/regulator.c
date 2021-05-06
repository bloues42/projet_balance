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
	float speed = 0;
	int8_t signe = ((error > 0) ? 1 : ((error < 0) ? -1 : 0));

	speed = KP * error + signe * SPEED_MIN;

    return (int16_t)speed;
}

int16_t rotation_regulator(int16_t speed){
	static uint16_t last_rot_error = 0;
	float speed_correction = 0;
	uint16_t rot_error;
	int32_t ir_left = 0, ir_right = 0;

	ir_left = get_calibrated_prox(IR_LEFT);
	ir_right = get_calibrated_prox(IR_RIGHT);

	if(speed > 0){
		//computes a correction factor to let the robot rotate in the middle of the platform
		if(ir_left>ROTATION_THRESHOLD){
			rot_error = ir_left;
		}
		if(ir_right>ROTATION_THRESHOLD){
			rot_error = - ir_right;
		}
	}else if(speed < 0){
		//computes a correction factor to let the robot rotate in the middle of the platform
		if(ir_left>ROTATION_THRESHOLD){
			rot_error = - ir_left;
		}
		if(ir_right>ROTATION_THRESHOLD){
			rot_error = ir_right;
		}
	}else{
		rot_error = 0;
	}

	speed_correction = ROTATION_COEFF * (rot_error-last_rot_error);

	last_rot_error = rot_error;
    return (int16_t)speed_correction;
}

static THD_WORKING_AREA(waRegulator, 256);
static THD_FUNCTION(Regulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    static int16_t speed = 0;
    int16_t speed_correction = 0;
    static int j = 0;
    int32_t ir_back;
    float mean; //A SUPPRIMER

    while(1){
        time = chVTGetSystemTime();
        
        //the function collect samples returns 1 if it has collected all the samples it needs to compute the mean
        //else, it returns 0
        if(collect_samples()){

			//computes the speed to give to the motors
			//the value of the error is obtained from the mean of the collected samples
			mean = get_mean_error(); //A SUPPRIMER
        	speed = regulator(mean);
        }

    	//compute speed correction to straighten the robot
    	speed_correction = rotation_regulator(speed);

		//chprintf((BaseSequentialStream *)&SD3, "mean_err=%f speed=%d \r\n", mean, speed);
		//chprintf((BaseSequentialStream *)&SD3, "corr=%d LEFT=%d RIGHT=%d \r\n", speed_correction, ir_left, ir_right);


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

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void regulator_start(void){
	chThdCreateStatic(waRegulator, sizeof(waRegulator), NORMALPRIO, Regulator, NULL);
}
