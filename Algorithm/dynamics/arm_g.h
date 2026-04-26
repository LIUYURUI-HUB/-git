#ifndef ARM_G_H
#define ARM_G_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
extern float arm_payload_mass;
extern uint8_t arm_loaded_mode;

typedef struct {
    float m1;       // 大臂质量 (kg)
    float m2;       // 小臂质量 (kg)
    float L1;       // 大臂连杆长度 (m)
    float L2;       // 小臂连杆长度 (m)
    float L3;       // 小臂末端连杆长度 (m)
    float Lc1;      // 大臂质心向距 (m)
    float Dc1;      // 大臂质心法向偏置 (m)
    float Lc2;      // 小臂质心向距 (m)
    float Dc2;      // 小臂质心法向偏置 (m)
    float g;        // 重力加速度 (m/s^2)

} ArmGravityParams_t;

typedef struct {
    float q1;       // 输入：大臂电机原始物理角度 alpha (rad)-M2_ZERO_OFFSET后的
    float q2;       // 输入：小臂电机原始物理角度 beta (rad)-M3_ZERO_OFFSET后的
    float payload;  // 输入：末端负载质量 (kg)

} ArmJoint_POS_t;

typedef struct {
    float tau1;     // 输出：大臂需要的前馈扭矩 (N*m)
    float tau2;     // 输出：小臂需要的前馈扭矩 (N*m)
    float tau3;     // 输出：小臂末端需要的前馈扭矩 (N*m)
} ArmGravityTorque_t;

// 初始化参数
void Arm_Gravity_InitParams(ArmGravityParams_t* params);

// 核心计算
void Arm_Gravity_Calc(const ArmGravityParams_t* params, const ArmJoint_POS_t* state, ArmGravityTorque_t* torque);

#ifdef __cplusplus
}
#endif

#endif // ARM_G_H
