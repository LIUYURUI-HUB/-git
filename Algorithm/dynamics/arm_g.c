#include "arm_g.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

ArmGravityParams_t arm_g_params;
#define Q    0.2217f
//初始化函数


void Arm_Gravity_InitParams(ArmGravityParams_t* params) {
    if (params == NULL) return;

    // 载入你提供的定制物理参数
    params->m1 = 0.617f;         // 大臂质量 (kg)
    params->m2 = 0.6107f;         // 小臂质量 (kg)
    params->L1 = 0.30f;       // 大臂长度 (m)
    params->L2 = 0.32f;        // 小臂长度 (m)

    params->Lc1 = 0.22963f;
    params->Dc1 = 0.001f;
    params->Lc2 = 0.266f;
    params->Dc2 = 0.000f;

    params->g = 9.81f;         // 重力加速度
}

void Arm_Gravity_Calc(const ArmGravityParams_t* params, const ArmJoint_POS_t* POS, ArmGravityTorque_t* torque) {
    if (params == NULL || POS == NULL || torque == NULL) return;

    float alpha = POS->q1;                                  // 大臂运算角度
    float theta = POS->q2-POS->q1+Q ;   // 小臂相对于垂直方向的角度

    // 提前计算三角函数以提高运行效率
    float c1 = cosf(alpha);
    float s1 = sinf(alpha);
    float c2 = cosf(theta);
    float s2 = sinf(theta);

    // ==========================================================
    // 2. 小臂关节 (Joint 2) 重力补偿计算
    // ==========================================================
    // 小臂质心在水平方向上的投影力臂
    float arm2_com_x = params->Lc2 * c2 - params->Dc2 * s2;
    float tau2_self = params->m2 * params->g * arm2_com_x;

    // 负载在水平方向上的投影力臂
    float payload_x = params->L2 * c2;
    float tau2_payload = POS->payload * params->g * payload_x;

    // 小臂电机所需总扭矩
    torque->tau2 = tau2_self + tau2_payload;
    torque->tau3 = arm_payload_mass* params->g*params->L3;
    // ==========================================================
    // 3. 大臂关节 (Joint 1) 重力补偿计算
    // ==========================================================
    // 大臂质心在水平方向上的投影力臂
    float arm1_com_x = params->Lc1 * c1 - params->Dc1 * s1;
    float tau1_self = params->m1 * params->g * arm1_com_x;

    // 将小臂质量和负载质量等效为悬挂在肘关节处的“死重”
    float tau1_deadweight = (params->m2 + POS->payload) * params->g * params->L1 * c1;

    // 大臂电机所需总扭矩 = 大臂自身扭矩 + 死重扭矩 + 小臂传递过来的反作用扭矩
    torque->tau1 = tau1_self + tau1_deadweight - torque->tau2;
}



