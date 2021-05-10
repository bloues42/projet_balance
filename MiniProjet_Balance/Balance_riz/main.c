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
#include <regulator.h>
#include <sensors/proximity.h>

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


int main(void)
{
	halInit();
    chSysInit();
    mpu_init();

	chThdSleepMilliseconds(700);

	//turns on the body LED, it will be turned off when the calibration is done and the robot can be used
	set_body_led(1);


    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();

    //inits the motors
    motors_init();
    //starts IR proximity sensor
    proximity_start();

    //starts imu
    imu_start();

    //calibrate acceleration sensor
    calibrate_acc();
    compute_accyz_offset();

	//calibrate IR proximity sensor
	calibrate_ir();

	set_body_led(0);
	//stars the threads for the pi regulator
	regulator_start();

    while (1) {
    	chThdSleepMilliseconds(300);
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
