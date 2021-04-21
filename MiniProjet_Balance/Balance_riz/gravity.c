#include "gravity.h"

#include <ch.h>
#include <hal.h>
#include <math.h>
//#include <msgbus/messagebus.h>
#include <chprintf.h>

#define STANDARD_GRAVITY    9.80665f
#define DEG2RAD(deg) 		(deg / 180 * M_PI)

#define RES_2G		2.0f
#define RES_250DPS	250.0f
#define MAX_INT16	32768.0f

#define ACC_RAW2G		(RES_2G / MAX_INT16)
#define GYRO_RAW2DPS	(RES_250DPS / MAX_INT16)

#define GRAV_X_MARGE 0.1 	// TO CHANGE


extern messagebus_t bus;

static imu_msg_t imu_values;

static thread_t *imuThd;
static bool imu_configured = false;

/***************************INTERNAL FUNCTIONS************************************/

 /**
 * @brief   Computes the measurements of the imu into readable measurements
 * 			RAW accelerometer to m/s^2 acceleration
 * 			RAW gyroscope to rad/s speed
 */
void imu_compute_units(void){
	/*
    *   TASK 10 : TO COMPLETE
    */
	imu_values.acceleration[X_AXIS] = ((imu_values.acc_raw[X_AXIS]-imu_values.acc_offset[X_AXIS])*STANDARD_GRAVITY * ACC_RAW2G);
	imu_values.acceleration[Y_AXIS] = ((imu_values.acc_raw[Y_AXIS]-imu_values.acc_offset[Y_AXIS])*STANDARD_GRAVITY * ACC_RAW2G);
	imu_values.acceleration[Z_AXIS] = ((imu_values.acc_raw[Z_AXIS]-imu_values.acc_offset[Z_AXIS])*STANDARD_GRAVITY * ACC_RAW2G);

/*
	imu_values.gyro_rate[X_AXIS] = ((imu_values.gyro_raw[X_AXIS]-imu_values.gyro_offset[X_AXIS])*DEG2RAD(GYRO_RAW2DPS));
	imu_values.gyro_rate[Y_AXIS] = ((imu_values.gyro_raw[Y_AXIS]-imu_values.gyro_offset[Y_AXIS])*DEG2RAD(GYRO_RAW2DPS));
	imu_values.gyro_rate[Z_AXIS] = ((imu_values.gyro_raw[Z_AXIS]-imu_values.gyro_offset[Z_AXIS])*DEG2RAD(GYRO_RAW2DPS));
*/
}

void compute_direction(void){

	float cos_alpha = 0, sin_alpha = 0, grav_x = 0;

	cos_alpha = fabs(imu_values.acceleration[Z_AXIS])/STANDARD_GRAVITY;
	sin_alpha = sqrt(1-cos_alpha*cos_alpha);
	grav_x = STANDARD_GRAVITY*sin_alpha; // >= 0

	/*
	if(fabs(imu_values.acceleration[X_AXIS]) < (grav_x - GRAV_X_MARGE)){
		if(imu_values.acceleration[X_AXIS] > 0)
			grav_x = - grav_x;

	if(fabs(imu_values.acceleration[X_AXIS]) > (grav_x + GRAV_X_MARGE)){
		if(imu_values.acceleration[X_AXIS] < 0)
			grav_x = - grav_x;
	}
	*/
}

 /**
 * @brief   Thread which updates the measurements and publishes them
 */
static THD_FUNCTION(imu_reader_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     // Declares the topic on the bus.
     messagebus_topic_t imu_topic;
     MUTEX_DECL(imu_topic_lock);
     CONDVAR_DECL(imu_topic_condvar);
     messagebus_topic_init(&imu_topic, &imu_topic_lock, &imu_topic_condvar, &imu_values, sizeof(imu_values));
     messagebus_advertise_topic(&bus, &imu_topic, "/imu");

     systime_t time;

     while (chThdShouldTerminateX() == false) {
    	 time = chVTGetSystemTime();

    	if(imu_configured == true){
	 		/* Reads the incoming measurement. */
    		//float *gyro, float *acc, float *temp, float *magnet, int16_t *gyro_raw, int16_t *acc_raw, int16_t *gyro_offset, int16_t *acc_offset, uint8_t *status
			mpu9250_read(imu_values.gyro_rate, imu_values.acceleration, &imu_values.temperature, imu_values.magnetometer,
					imu_values.gyro_raw, imu_values.acc_raw, imu_values.gyro_offset, imu_values.acc_offset, &imu_values.status);
			/* computes the raw values into readable values*/
			imu_compute_units();
     	}

     	/* Publishes it on the bus. */
		messagebus_topic_publish(&imu_topic, &imu_values, sizeof(imu_values));

        chThdSleepUntilWindowed(time, time + MS2ST(4)); //reduced the sample rate to 250Hz

     }
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void imu_start(void)
{
	int8_t status = MSG_OK;

    status = mpu9250_setup(MPU9250_ACC_FULL_RANGE_2G
		                  | MPU9250_GYRO_FULL_RANGE_250DPS
		                  | MPU9250_SAMPLE_RATE_DIV(100));
		                  //| MPU60X0_LOW_PASS_FILTER_6)

    //not tested yet because the auxilliary I2C of the MPU-9250 is condamned due
    //to PCB correction on the e-puck2-F4, so the magnetometer cannot be read...
    // if(status == MSG_OK){
    // 	status = mpu9250_magnetometer_setup();
    // }

    if(status == MSG_OK){
    	imu_configured = true;
    }

    static THD_WORKING_AREA(imu_reader_thd_wa, 1024);
    imuThd = chThdCreateStatic(imu_reader_thd_wa, sizeof(imu_reader_thd_wa), NORMALPRIO, imu_reader_thd, NULL);
}

void imu_stop(void) {
    chThdTerminate(imuThd);
    chThdWait(imuThd);
    imuThd = NULL;
}

void imu_compute_offset(messagebus_topic_t * imu_topic, uint16_t nb_samples){

    /*
    *   TASK 9 : TO COMPLETE
    */

	//imu_msg_t curr_values;
	int32_t tot_gyro[NB_AXIS]={0}, tot_acc[NB_AXIS]={0};

	for(uint16_t s=0; s<nb_samples; s++){
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
/*
		tot_gyro[X_AXIS] += imu_values.gyro_raw[X_AXIS];
		tot_gyro[Y_AXIS] += imu_values.gyro_raw[Y_AXIS];
		tot_gyro[Z_AXIS] += imu_values.gyro_raw[Z_AXIS];
*/
		tot_acc[X_AXIS] += imu_values.acc_raw[X_AXIS];
		tot_acc[Y_AXIS] += imu_values.acc_raw[Y_AXIS];
		tot_acc[Z_AXIS] += imu_values.acc_raw[Z_AXIS];
	}

/*
	tot_gyro[X_AXIS] /= nb_samples;
	imu_values.gyro_offset[X_AXIS] = tot_gyro[X_AXIS];

	imu_values.gyro_offset[Y_AXIS] = tot_gyro[Y_AXIS]/nb_samples;
	imu_values.gyro_offset[Z_AXIS] = tot_gyro[Z_AXIS]/nb_samples;
*/

	imu_values.acc_offset[X_AXIS] = tot_acc[X_AXIS]/nb_samples;
//	imu_values.acc_offset[Y_AXIS] = tot_acc[Y_AXIS]/nb_samples;
	//pour z, il faut ajouter la gravité
	imu_values.acc_offset[Z_AXIS] = (tot_acc[Z_AXIS]/nb_samples) + (MAX_INT16 / RES_2G);

	palClearPad(GPIOB, GPIOB_LED_BODY);

	//chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
	 //               imu_values.acc_offset[X_AXIS], imu_values.acc_offset[Y_AXIS], imu_values.acc_offset[Z_AXIS],
	 //               imu_values.gyro_offset[X_AXIS], imu_values.gyro_offset[Y_AXIS], imu_values.gyro_offset[Z_AXIS]);

}

int16_t get_acc(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.acc_raw[axis];
	}
	return 0;
}

void get_acc_all(int16_t *values) {
	values[X_AXIS] = imu_values.acc_raw[X_AXIS];
	values[Y_AXIS] = imu_values.acc_raw[Y_AXIS];
	values[Z_AXIS] = imu_values.acc_raw[Z_AXIS];
}


int16_t get_acc_offset(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.acc_offset[axis];
	}
	return 0;
}

float get_acceleration(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.acceleration[axis];
	}
	return 0;
}


int16_t get_gyro(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.gyro_raw[axis];
	}
	return 0;
}

void get_gyro_all(int16_t *values) {
	values[X_AXIS] = imu_values.gyro_raw[X_AXIS];
	values[Y_AXIS] = imu_values.gyro_raw[Y_AXIS];
	values[Z_AXIS] = imu_values.gyro_raw[Z_AXIS];
}

int16_t get_gyro_offset(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.gyro_offset[axis];
	}
	return 0;
}

float get_gyro_rate(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.gyro_rate[axis];
	}
	return 0;
}

float get_temperature(void) {
	return imu_values.temperature;
}

/**************************END PUBLIC FUNCTIONS***********************************/

