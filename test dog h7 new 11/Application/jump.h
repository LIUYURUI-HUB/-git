#ifndef __JUMP_CONTROL_H__
#define __JUMP_CONTROL_H__

#include <stdint.h>
#include "../../MIddleware/kinematics.h"
#include "../../BSP/MOTORS/8010/motor_controller.h"

// 跳跃状态机枚举
typedef enum {
    JUMP_STATE_IDLE = 0,   // 正常站立
    JUMP_STATE_SQUAT,      // 蓄力下蹲
    JUMP_STATE_THRUST,     // 爆发蹬地
    JUMP_STATE_FLIGHT,     // 腾空收腿
    JUMP_STATE_LANDING     // 触地缓冲
} JumpState_t;

// 跳跃参数配置结构体
typedef struct {
    float y_stand;         // 正常站立时 Y 坐标 (cm)
    float y_squat;         // 下蹲时 Y 坐标 (cm)
    float y_extend;        // 蹬地伸展时 Y 坐标 (cm)
    float y_flight;        // 腾空收腿时 Y 坐标 (cm)

    uint32_t t_squat;      // 下蹲耗时 (ms)
    uint32_t t_thrust;     // 蹬地耗时 (ms) - 越短爆发力越强
    uint32_t t_flight;     // 腾空时间预估 (ms)
    uint32_t t_landing;    // 落地缓冲耗时 (ms)
} JumpConfig_t;

// 跳跃控制器状态
typedef struct {
    JumpState_t current_state;
    uint32_t state_start_time; // 当前状态开始的时间戳
    float current_target_y;    // 当前插值计算出的Y坐标
    float current_kp;          // 当前刚度
    float current_kd;          // 当前阻尼
    JumpConfig_t config;
} JumpController_t;

void JumpControl_Init(JumpController_t *jc);
void JumpControl_Trigger(JumpController_t *jc, uint32_t current_time);
void JumpControl_Update(JumpController_t *jc, MotorController* ctrl, uint32_t current_time, int hip_id, int calf_id);

#endif // __JUMP_CONTROL_H__
