#include "pi_regulator.h"

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <gravity.h>
#include <main.h>
#include <motors.h>
#include <sensors/proximity.h>

//simple PI regulator implementation
int16_t pi_regulator(float grav_y){

	float speed = 0;
	static float sum_grav_y = 0;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want
	//and it accounts for inaccuracy in the acceleration measurement
	if(fabs(grav_y) < GRAV_Y_MARGE){
		return 0;
	}
	static int i = 0;

	sum_grav_y += grav_y;

	if (i==5){
		chprintf((BaseSequentialStream *)&SD3, "grav_sum=%f grav=%f vitesse=%f \r\n",sum_grav_y, grav_y, speed);
		i=0;
	}
	i++;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_grav_y > MAX_SUM_ERROR){
		sum_grav_y = MAX_SUM_ERROR;
	}else if(sum_grav_y < -MAX_SUM_ERROR){
		sum_grav_y = -MAX_SUM_ERROR;
	}

	speed = KP * grav_y + KI * sum_grav_y;


    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed=50;
    int16_t speed_correction = 0;        static int j = 0;
    int32_t pi_ir_left = 0, pi_ir_right = 0;
    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //the value of the gravity is obtained from imu thread

        //speed = pi_regulator(compute_gravity_y());

        //computes a correction factor to let the robot rotate in the middle of the platform
        pi_ir_left = get_calibrated_prox(IR_LEFT);
        pi_ir_right = get_calibrated_prox(IR_RIGHT);
        speed_correction = pi_ir_left - pi_ir_right;

        //if the difference of distance is too small, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }


        if (j==5){
			chprintf((BaseSequentialStream *)&SD3, "corr=%d LEFT=%d RIGHT=%d \r\n",speed_correction, pi_ir_left, pi_ir_right);
			j=0;
		}
		j++;

        //applies the speed from the PI regulator and the correction for the rotation
			right_motor_set_speed(speed- ROTATION_COEFF * speed_correction/50);
			left_motor_set_speed(speed + ROTATION_COEFF * speed_correction/50);


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
