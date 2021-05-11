#ifndef REGULATOR_H
#define REGULATOR_H

#include <motors.h>
#include <sensors/proximity.h>

#define KP_LINEAR				2000.0f
#define SPEED_MIN				200
#define ROTATION_THRESHOLD		5
#define BACK_WALL_STOP			40
#define SIDE_STOP				300 //A REGLER??
#define KD_ROTATION_BACK		2.0f//1.0f
#define KD_ROTATION_FORWARD		1.0f

enum	IR_sensor_pos{IR_FRONT_RIGHT=0, IR_FRONT_RIGHT45, IR_RIGHT, IR_BACK_RIGHT,
						IR_BACK_LEFT, IR_LEFT, IR_FRONT_LEFT45, IR_FRONT_LEFT};

//start the regulator thread
void regulator_start(void);

#endif /* REGULATOR_H */
