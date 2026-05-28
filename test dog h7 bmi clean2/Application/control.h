#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_
#include "../../BSP/AS01/AS01.h"
#include "../../MIddleware/kinematics.h"
#include "../../Application/gait.h"
#include "../../BSP/MOTORS/8010/motor_controller.h"
#include "../../BSP/MOTORS/3508/3508_driver.h"
#include "string.h"
typedef enum {
    ROBOT_STATE_IDLE = 0,
    ROBOT_STATE_FORWARD,
    ROBOT_STATE_BACKWARD,
    ROBOT_STATE_LEFT,
    ROBOT_STATE_RIGHT
} RobotState_e;
typedef enum {
    GAIT_MODE_TROT,
    GAIT_MODE_WALK,
	GAIT_MODE_BOUND,
	GAIT_MODE_PRONK
} GaitMode_e;
void state_zero0(uint32_t now);
void joystick_control(MotorController*ctrl1,MotorController*ctrl2,QuadrupedGait*gait,uint32_t startTime,LegAngles angles,uint32_t now);
void button_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t currentTime);
void AS01_rx(MotorController*ctrl1,MotorController*ctrl2,QuadrupedGait*gait,uint32_t startTime,LegAngles angles,uint32_t now);
void state_zero(uint32_t now, float current_kp);
void Chassis_Attitude_Update(void);
#endif
