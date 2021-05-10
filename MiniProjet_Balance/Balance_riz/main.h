#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include <motors.h>

enum	IR_sensor_pos{IR_FRONT_RIGHT=0, IR_FRONT_RIGHT45, IR_RIGHT, IR_BACK_RIGHT, IR_BACK_LEFT, IR_LEFT, IR_FRONT_LEFT45, IR_FRONT_LEFT};



/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
