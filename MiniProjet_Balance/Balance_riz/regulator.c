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
	//static float sum_grav_y = 0;

	static int i = 0;

/* pour le régulateur I
	sum_grav_y += grav_y;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_grav_y > MAX_SUM_ERROR){
		sum_grav_y = MAX_SUM_ERROR;
	}else if(sum_grav_y < -MAX_SUM_ERROR){
		sum_grav_y = -MAX_SUM_ERROR;
	}
*/
	int8_t signe = ((error > 0) ? 1 : ((error < 0) ? -1 : 0));

	speed = KP * error + signe*SPEED_MIN; //+ KI * sum_grav_y;
/*
	if (i==5){
		chprintf((BaseSequentialStream *)&SD3, "erreur=%f vitesse=%f \r\n", error, speed);
		i=0;
	}
	i++;
*/
    return (int16_t)speed;
}

static THD_WORKING_AREA(waRegulator, 256);
static THD_FUNCTION(Regulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed = 0;
    int16_t speed_correction = 0;
    static int j = 0;
    int32_t ir_left = 0, ir_right = 0;
    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //the value of the gravity is obtained from imu thread

       speed = regulator(compute_error());

        //computes a correction factor to let the robot rotate in the middle of the platform
        ir_left = get_calibrated_prox(IR_LEFT);
        ir_right = get_calibrated_prox(IR_RIGHT);
        speed_correction = ir_left - ir_right;

        //if the difference of distance is too small, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }

/*
        if (j==5){
			chprintf((BaseSequentialStream *)&SD3, "corr=%d LEFT=%d RIGHT=%d \r\n",speed_correction, ir_left, ir_right);
			j=0;
		}
		j++;
*/
        //applies the speed from the regulator and the correction for the rotation
			right_motor_set_speed(speed- ROTATION_COEFF * speed_correction/50);
			left_motor_set_speed(speed + ROTATION_COEFF * speed_correction/50);


        //10Hz
        chThdSleepUntilWindowed(time, time + MS2ST(100));
    }
}

void regulator_start(void){
	chThdCreateStatic(waRegulator, sizeof(waRegulator), NORMALPRIO, Regulator, NULL);
}
