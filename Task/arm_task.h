#ifndef VISION_TASK_H
#define VISION_TASK_H

#include <stdint.h>
#include "protocol_handler.h"

void Task_Vision_State_Machine(void);
uint8_t Task_Vision_GetStateCode(void);
const char* Task_Vision_GetStateName(void);
Arm_Target_t Task_Vision_GetCurrentTarget(void);
void Task_Vision_ResetStateMachine(void);

#endif
