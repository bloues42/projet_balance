#include "pi_regulator.h"

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <gravity.h>
#include <main.h>
#include <motors.h>


//simple PI regulator implementation
int16_t pi_regulator(float grav_y){

	float speed = 0;
	static float sum_grav_y = 10;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want
	//and it accounts for inaccuracy in the acceleration measurement
	if(fabs(grav_y) < GRAV_Y_MARGE){
		return 0;
	}
	static int i = 0;


	sum_grav_y += grav_y;

	if (i==500){
		chprintf((BaseSequentialStream *)&SD3, "%grav=%-7f %vitesse=%-7f \r\n",sum_grav_y, speed);
		i=0;
	}
	i++;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_grav_y > MAX_SUM_ERROR){
		sum_grav_y = MAX_SUM_ERROR;
	}else if(sum_grav_y < -MAX_SUM_ERROR){
		sum_grav_y = -MAX_SUM_ERROR;
	}

	speed = KP * grav_y; //+ KI * sum_grav_y;



    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;
    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //the value of the gravity is obtained from imu thread
        speed = pi_regulator(compute_gravity_y());


/* A FAIRE
        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }
*/


        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
