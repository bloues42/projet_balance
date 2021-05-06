#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include <motors.h>

#define KP						800.0f
#define KI						12.0f//3.5f
#define SPEED_MIN				200
#define MAX_SUM_ERROR			(MOTOR_SPEED_LIMIT/KI)
//#define GRAV_Y_MARGE			0.1f //1.3
#define ROTATION_THRESHOLD		5 //20
#define BACK_WALL_STOP			80 		//A MODIFIER
#define ROTATION_COEFF			0.2f

enum	IR_sensor_pos{IR_FRONT_RIGHT=0, IR_FRONT_RIGHT45, IR_RIGHT, IR_BACK_RIGHT, IR_BACK_LEFT, IR_LEFT, IR_FRONT_LEFT45, IR_FRONT_LEFT};



/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
