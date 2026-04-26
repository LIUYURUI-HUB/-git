/*
 * leg_task.h
 *
 *  Created on: Mar 30, 2026
 *      Author: 22569
 */

#ifndef LEG_TASK_H_
#define LEG_TASK_H_

#include "kinematics.h"
#include "gait.h"
#include "motor_controller.h"
#include "3508_driver.h"
#include "string.h"
#include "Chassis_Control.h"

void vision_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now);
void state_zero_with_compensation(float* leg_y_offsets);
void state_crouch(void);

#endif /* LEG_TASK_H_ */
