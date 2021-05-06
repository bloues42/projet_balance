#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <gravity.h>

#include <sensors/imu.h>
#include <leds.h>
//#include <msgbus/messagebus.h>

#include <regulator.h>

//#include <communications.h> FROM TP5
//#include <arm_math.h>		  FROM TP5

#include <sensors/proximity.h>

#define NB_SAMPLES_OFFSET     100

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

/*
 PRATIQUE PR MESURER TPS EXECUTION FCT
static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        ** 1MHz timer clock in order to measure uS.
        NULL,           ** Timer callback.
        0,
        0
    };
    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}
*/

int main(void)
{
	halInit();
    chSysInit();
    mpu_init();

	chThdSleepMilliseconds(700);

    set_body_led(1);
    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    // timer12_start();		LATER ?
    //inits the motors
    motors_init();
    //starts IR proximity sensor
    proximity_start();

    //starts imu
    imu_start();


/*
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);
    imu_msg_t imu_values;
*/

    //calibrate acceleration sensor
    calibrate_acc();
    compute_accyz_offset();

	//calibrate IR proximity sensor
	calibrate_ir();

	set_body_led(0);
	//stars the threads for the pi regulator
	regulator_start();

	uint32_t i = 0, j=0;


/*	int16_t speed = 500;
    right_motor_set_speed(speed);
    left_motor_set_speed(speed);
*/
    float acc_y = 0, acc_z = 0;
    float error = 0;

    int32_t ir_left = 0, ir_right = 0, difference = 0;

    while (1) {

    	//if (i==5000){
/*
    		ir_left = get_calibrated_prox(IR_RIGHT);
    		ir_right = get_calibrated_prox(IR_LEFT);
    		difference = ir_right - ir_left;
    		chprintf((BaseSequentialStream *)&SD3, "left=%d right=%d difference=%d \r\n", ir_left, ir_right, difference);
    	//	i=0;
    	//}
    	//i++;
        //chThdSleepMilliseconds(1000);


		acc_y = imu_compute_units(Y_AXIS); //get_acceleration(Y_AXIS); //
		acc_z = imu_compute_units(Z_AXIS); //get_acceleration(Z_AXIS); //


		error = compute_error();

		if (j==3){
			chprintf((BaseSequentialStream *)&SD3, "%Ay=%-7f Az=%-7f error=%-7f \r\n",
					acc_y, acc_z, error);

			j=0;
		}
		j++;

        if(i==10){
        	speed = -speed;
			right_motor_set_speed(speed);
			left_motor_set_speed(speed);
			i=0;
        }
        i++;
*/
    	chThdSleepMilliseconds(300);
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
