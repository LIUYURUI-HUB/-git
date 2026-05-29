#ifndef __PROTOCOL_HANDLER_H
#define __PROTOCOL_HANDLER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Protocol frame:
 * [0x55][0xAA][FuncID][Len][Payload...][CheckSum]
 */

#define PROTOCOL_HEAD1       0x55
#define PROTOCOL_HEAD2       0xAA

#define FUNC_TIME_SYNC       0x01
#define FUNC_ERROR_REPORT    0x02
#define FUNC_CHASSIS_MOVE    0x10
#define FUNC_GAIT_SWITCH     0x11
#define FUNC_ARM_CONTROL     0x12
#define FUNC_SUCTION_CONTROL 0x13
#define FUNC_PATH_POINT      0x20
#define FUNC_OBSTACLE_INFO   0x21
#define FUNC_JOINT_FEEDBACK  0x30

#define DOG_STATE_INIT       0x00
#define DOG_STATE_IDLE       0x01
#define DOG_STATE_RUNNING    0x02
#define DOG_STATE_ESTOP      0x03

#define ERROR_NONE           0x00
#define ERROR_LOW_BAT        0x01
#define ERROR_MOTOR_HOT      0x02
#define ERROR_COMM_LOST      0x03

typedef struct {
    float x;
    float y;
    float z;
} Arm_Target_t;

typedef struct {
    uint8_t mode;
} Suction_Control_t;

typedef struct {
    float vx;
    float vy;
    float wz;
    uint8_t state;
} Chassis_Move_t;

typedef struct {
    uint8_t gait_id;
} Gait_Switch_t;

typedef struct {
    float leg_joints[12];
    float arm_joints[4];
    float est_vx;
    float est_vy;
    float est_wz;
} Joint_Feedback_t;

typedef struct {
    uint8_t dog_state;
    uint8_t error_code;
} Dog_Status_t;

void Protocol_Init(void);
void Protocol_ProcessByte(uint8_t data);
void Protocol_ProcessBuffer(uint8_t* data, uint32_t length);

Arm_Target_t Protocol_GetArmTarget(void);
uint8_t Protocol_GetSuctionControl(void);
Chassis_Move_t Protocol_GetChassisMove(void);
uint8_t Protocol_GetGaitSwitch(void);

bool Protocol_IsNewArmData(void);
bool Protocol_IsNewSuctionData(void);
bool Protocol_IsNewChassisData(void);
bool Protocol_IsNewGaitData(void);

void Protocol_ClearArmDataFlag(void);
void Protocol_ClearSuctionDataFlag(void);
void Protocol_ClearChassisDataFlag(void);
void Protocol_ClearGaitDataFlag(void);

int Protocol_SendJointFeedback(Joint_Feedback_t* feedback);
int Protocol_SendErrorReport(Dog_Status_t* status);
int Protocol_SendArmPositionFeedback(float x, float y, float z);

Dog_Status_t* Protocol_GetDogStatus(void);
void Protocol_SetDogState(uint8_t state);
void Protocol_SetErrorCode(uint8_t error);

bool Protocol_IsChassisTimeout(void);

#endif /* __PROTOCOL_HANDLER_H */
