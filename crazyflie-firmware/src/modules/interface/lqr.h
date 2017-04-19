/*
 * lqr.h
 *
 *  Created on: Feb 15, 2017
 *      Author: bitcraze
 */

#ifndef SRC_MODULES_INTERFACE_LQR_H_
#define SRC_MODULES_INTERFACE_LQR_H_

#include <stdbool.h>
#include <stdint.h>
 
#include "stabilizer_types.h"

/*
 * phi value is related to attitude.roll
 * theta value is related to attitude.pitch
 * psy value is related to attitude.yaw
 * Parameter values declared here
 * HW_TODO: Are these values to be changed
 * as PARAMETERS, or set values on
 * attitude_pid_controller.c
 */

void LQRController(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);

#endif /* SRC_MODULES_INTERFACE_LQR_H_ */