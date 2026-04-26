/**
 * @file protocol_handler.h
 * @brief 通信协议处理头文件
 *
 * 协议帧格式:
 * [0x55][0xAA][FuncID][Len][Payload...][CheckSum]
 */

#ifndef __PROTOCOL_HANDLER_H
#define __PROTOCOL_HANDLER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ==================== 协议定义 ====================

// 帧头
#define PROTOCOL_HEAD1       0x55
#define PROTOCOL_HEAD2       0xAA

// 功能字 (FuncID)
#define FUNC_TIME_SYNC       0x01    // 时间同步校准
#define FUNC_ERROR_REPORT    0x02    // 错误报告
#define FUNC_CHASSIS_MOVE    0x10    // 底盘运动指令
#define FUNC_GAIT_SWITCH     0x11    // 步态/姿态切换
#define FUNC_ARM_CONTROL     0x12    // 机械臂关节控制
#define FUNC_SUCTION_CONTROL 0x13    // 机械臂吸盘控制
#define FUNC_PATH_POINT      0x20    // 局部路径点
#define FUNC_OBSTACLE_INFO   0x21    // 地形/避障信息
#define FUNC_JOINT_FEEDBACK  0x30    // 关节与运动反馈

// 状态码
#define DOG_STATE_INIT       0x00    // 初始化
#define DOG_STATE_IDLE       0x01    // 空闲
#define DOG_STATE_RUNNING    0x02    // 任务执行
#define DOG_STATE_ESTOP      0x03    // 急停锁定

#define ERROR_NONE           0x00    // 正常
#define ERROR_LOW_BAT        0x01    // 电池低压
#define ERROR_MOTOR_HOT      0x02    // 电机过热
#define ERROR_COMM_LOST      0x03    // 通讯丢失

// ==================== 数据结构 ====================

typedef struct {
    float x;     // 目标 X
    float y;     // 目标 Y
    float z;     // 目标 Z
} Arm_Target_t;

// 吸盘控制 (0x13)
typedef struct {
    uint8_t mode;    // 0:关, 1:开
} Suction_Control_t;

// 底盘运动指令 (0x10)
typedef struct {
    float vx;        // X轴线速度 (m/s)
    float vy;        // Y轴线速度 (m/s)
    float wz;        // Z轴角速度 (rad/s)
    uint8_t state;   // 新增：保存上位机下发的直接状态码
} Chassis_Move_t;

// 步态切换 (0x11)
typedef struct {
    uint8_t gait_id; // 0:被动, 1:站立, 2:Trot, 3:Walk, 4:Jump
} Gait_Switch_t;

// 关节反馈 (0x30)
typedef struct {
    float leg_joints[12];   // 12个腿部电机角度
    float arm_joints[4];    // 3个机械臂电机角度
    float est_vx;           // 当前X速度
    float est_vy;           // 当前Y速度
    float est_wz;
} Joint_Feedback_t;

// 狗状态
typedef struct {
    uint8_t dog_state;
    uint8_t error_code;
} Dog_Status_t;

// ==================== 函数声明 ====================

/**
 * @brief 初始化协议处理模块
 */
void Protocol_Init(void);

/**
 * @brief 处理接收到的字节数据
 * @param data 接收到的单字节
 */
void Protocol_ProcessByte(uint8_t data);

/**
 * @brief 处理接收到的数据缓冲区
 * @param data 数据缓冲区
 * @param length 数据长度
 */
void Protocol_ProcessBuffer(uint8_t* data, uint32_t length);

/**
 * @brief 获取机械臂目标角度
 * @return 机械臂目标角度结构体
 */
Arm_Target_t Protocol_GetArmTarget(void);

/**
 * @brief 获取吸盘控制命令
 * @return 吸盘开关状态
 */
uint8_t Protocol_GetSuctionControl(void);

/**
 * @brief 获取底盘运动指令
 * @return 底盘运动结构体
 */
Chassis_Move_t Protocol_GetChassisMove(void);

/**
 * @brief 获取步态切换指令
 * @return 步态ID
 */
uint8_t Protocol_GetGaitSwitch(void);

/**
 * @brief 检查是否有新的机械臂指令
 * @return true:有新指令
 */
bool Protocol_IsNewArmData(void);

/**
 * @brief 检查是否有新的吸盘指令
 * @return true:有新指令
 */
bool Protocol_IsNewSuctionData(void);

/**
 * @brief 清除机械臂数据标志
 */
void Protocol_ClearArmDataFlag(void);

/**
 * @brief 清除吸盘数据标志
 */
void Protocol_ClearSuctionDataFlag(void);

/**
 * @brief 发送关节反馈 (0x30)
 * @param feedback 关节反馈数据
 * @return 发送是否成功
 */
int Protocol_SendJointFeedback(Joint_Feedback_t* feedback);

/**
 * @brief 发送错误报告 (0x02)
 * @param status 狗状态
 * @return 发送是否成功
 */
int Protocol_SendErrorReport(Dog_Status_t* status);

/**
 * @brief 获取狗状态结构体
 */
Dog_Status_t* Protocol_GetDogStatus(void);

/**
 * @brief 设置狗状态
 */
void Protocol_SetDogState(uint8_t state);

/**
 * @brief 设置错误码
 */
void Protocol_SetErrorCode(uint8_t error);
int Protocol_SendArmPositionFeedback(float x, float y, float z);

#endif /* __PROTOCOL_HANDLER_H */
