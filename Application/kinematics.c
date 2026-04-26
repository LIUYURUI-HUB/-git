#include "kinematics.h"
#include <math.h>
//#define PI 3.141f
float global_pos_offset[10] = {0.0f};
int global_has_offset[10] = {0};
int last_is_calibrated = 0;
extern int is_calibrated;
Currentpos ForwardKinematics(MotorController *ctrl, int hip_id, int knee_id) {
    MotorData_t *motor_s = &ctrl->datas[hip_id];
    MotorData_t *motor_r = &ctrl->datas[knee_id];
    Currentpos currentpos = {0.0f, 0.0f};

    float L1 = 18.50f;
    float L2 = 21.50f;
    float theta1 = 0.0f;
    float theta2 = 0.0f;

    // 校准防呆逻辑（上升沿检测）
    if (is_calibrated && !last_is_calibrated) {
        for (int i = 0; i < 10; i++) {
            global_has_offset[i] = 0;
        }
    }
    last_is_calibrated = is_calibrated;

    if (is_calibrated) {
        if (fabsf(motor_s->Pos) > 0.001f || fabsf(motor_r->Pos) > 0.001f) {
            if (global_has_offset[hip_id] == 0) {
                global_pos_offset[hip_id] = motor_s->Pos;
                global_has_offset[hip_id] = 1;
            }
            if (global_has_offset[knee_id] == 0) {
                global_pos_offset[knee_id] = motor_r->Pos;
                global_has_offset[knee_id] = 1;
            }
        } else {
            return currentpos;
        }
    }

    if (!global_has_offset[hip_id] || !global_has_offset[knee_id]) {
        return currentpos;
    }

    float offset_s = global_pos_offset[hip_id];
    float offset_r = global_pos_offset[knee_id];

    // --- 已验证的正确正解公式 ---
    if (hip_id == 1) {
        theta1 = (motor_s->Pos - offset_s) / 6.33f ;
        theta2 = (offset_r - motor_r->Pos) / 6.33f ;
    }
    else if (hip_id == 6) {
        theta1 = (motor_s->Pos - offset_s) / 6.33f;
        theta2 = (offset_r - motor_r->Pos) / 6.33f;
    }
    else if (hip_id == 7) {
        theta1 = (offset_s - motor_s->Pos) / 6.33f;
        theta2 = (motor_r->Pos - offset_r) / 6.33f;
    }
    else if (hip_id == 3) {
        theta1 = (offset_s - motor_s->Pos) / 6.33f;
        theta2 = (motor_r->Pos - offset_r) / 6.33f;
    }

    currentpos.X = L1 * sinf(theta1) + L2 * sinf(theta2 + theta1 + 0.215f * (float)M_PI);
    currentpos.Y = L1 * cosf(theta1) + L2 * cosf(theta2 + theta1 + 0.215f * (float)M_PI);

    return currentpos;
}
LegAngles InverseKinematics(float X, float Y, MotorController* ctrl, int hip_id, int knee_id, float kp, float kd) {
    LegAngles angles = {0.0f, 0.0f};

    // 1. 安全防护：没有零位记忆就不乱发指令
    if (!global_has_offset[hip_id] || !global_has_offset[knee_id]) {
        return angles;
    }

    float L1 = 18.5f;
    float L2 = 21.5f;
    float L3 = sqrtf(X*X + Y*Y);

    // 2. 结构限制防护 (防 NaN)
    if (L3 > L1 + L2) L3 = L1 + L2 - 0.001f;
    if (L3 < fabsf(L1 - L2)) L3 = fabsf(L1 - L2) + 0.001f;

    // 3. 余弦定理计算
    float cos_val1 = (L3*L3 - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    if (cos_val1 > 1.0f) cos_val1 = 1.0f;
    if (cos_val1 < -1.0f) cos_val1 = -1.0f;
    float theta2_ext = acosf(cos_val1);

    float cos_val2 = (L3*L3 + L1*L1 - L2*L2) / (2.0f * L1 * L3);
    if (cos_val2 > 1.0f) cos_val2 = 1.0f;
    if (cos_val2 < -1.0f) cos_val2 = -1.0f;
    float theta4 = acosf(cos_val2);

    // 4. 【坐标系归一】：无视象限，直接求出目标绝对角度
    float t1_target = atan2f(X, Y) - theta4;
    float t2_target = theta2_ext - 0.215f * (float)M_PI;

    // 获取当前腿共享的零位校准值
    float offset_s = global_pos_offset[hip_id];
    float offset_r = global_pos_offset[knee_id];

    float pos_s = 0.0f;
    float pos_r = 0.0f;

    // 5. 【完全对称逆向映射】：根据你最新的纯净正解反推
    if (hip_id == 1 || hip_id == 6) {
        pos_s = t1_target * 6.33f + offset_s;
        pos_r = offset_r - t2_target * 6.33f;
    }
    else if (hip_id == 7 || hip_id == 3) {
        pos_s = offset_s - t1_target * 6.33f;
        pos_r = t2_target * 6.33f + offset_r;
    }

    angles.theta1 = pos_s;
    angles.theta2 = pos_r;

    // 最后发送指令给控制器
    MotorController_SetCommand(ctrl, knee_id, 1, 0.0f, 0.0f, angles.theta2, kp, kd);
    MotorController_SetCommand(ctrl, hip_id,  1, 0.0f, 0.0f, angles.theta1, kp, kd);

    return angles;
}
