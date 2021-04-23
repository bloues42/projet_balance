#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include <motors.h>

#define KP						800.0f
#define KI						3.5f
#define MAX_SUM_ERROR			(MOTOR_SPEED_LIMIT/KI)
#define GRAV_Y_MARGE			0.1
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
