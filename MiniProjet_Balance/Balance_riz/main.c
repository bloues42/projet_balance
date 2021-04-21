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
//#include <gravity.h>
#include <sensors/imu.h>
//#include <msgbus/messagebus.h>

//#include <pi_regulator.h>

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
	//stars the threads for the pi regulator
	//pi_regulator_start();
    //starts imu
    imu_start();


/*
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);
    imu_msg_t imu_values;
*/
    //calibrate acceleration sensor
    calibrate_acc();



	//calibrate IR proximity sensor
	calibrate_ir();

	uint32_t i = 0;
	uint32_t dist_sensor0 = 0;

    uint8_t speed = 10;
	right_motor_set_speed(speed);
	left_motor_set_speed(speed);

    while (1) {

    //	if (i==5000){
    /*
    		dist_sensor0 = get_calibrated_prox(0);
    		chprintf((BaseSequentialStream *)&SD3, "distance=%d\n", dist_sensor0);
    	//	i=0;
    	//}
    	//i++;
        chThdSleepMilliseconds(1000);
    */
    	float acc_x=0, acc_z=0;
    	acc_x = get_acc(X_AXIS);
    	acc_z = get_acc(Z_AXIS);

    	chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Az=%-7d \r\n",
    			acc_x, acc_z);
    	chThdSleepMilliseconds(400);
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
