#include "../../Application/jump.h"
#include <math.h>

// 简单的平滑插值函数 (余弦插值，避免速度突变)
static float smooth_interp(float start, float end, float t, float duration) {
    if (t >= duration) return end;
    // 使用 1 - cos(pi * t / T) 实现S型速度曲线
    float ratio = (1.0f - cosf((t / duration) * 3.1415926f)) / 2.0f;
    return start + (end - start) * ratio;
}

// 线性插值 (爆发蹬地时需要直接线性给到位，越快越好)
static float linear_interp(float start, float end, float t, float duration) {
    if (t >= duration) return end;
    return start + (end - start) * (t / duration);
}

void JumpControl_Init(JumpController_t *jc) {
    jc->current_state = JUMP_STATE_IDLE;
    jc->state_start_time = 0;

    // 初始化跳跃配置参数 (需根据你的机器人机械结构微调)
    // 假设大腿 18.5, 小腿 21.5，最大伸长为 40.0
    jc->config.y_stand = 26.0f;
    jc->config.y_squat = 14.0f;    // 下蹲深度
    jc->config.y_extend = 33.0f;   // 极力伸直
    jc->config.y_flight = 20.0f;   // 空中收腿高度

    jc->config.t_squat = 800;      // 下蹲较慢 (800ms)
    jc->config.t_thrust = 80;      // 蹬地极快，产生爆发力 (80ms)
    jc->config.t_flight = 300;     // 滞空预估时间 (300ms)
    jc->config.t_landing = 500;    // 缓冲时间 (500ms)

    jc->current_target_y = jc->config.y_stand;
    jc->current_kp = 0.2f;  // 默认刚度
    jc->current_kd = 0.1f;  // 默认阻尼
}

void JumpControl_Trigger(JumpController_t *jc, uint32_t current_time) {
    if (jc->current_state == JUMP_STATE_IDLE) {
        jc->current_state = JUMP_STATE_SQUAT;
        jc->state_start_time = current_time;
    }
}

void JumpControl_Update(JumpController_t *jc, MotorController* ctrl, uint32_t current_time, int hip_id, int calf_id) {
    uint32_t elapsed = current_time - jc->state_start_time;
    switch (jc->current_state) {
        case JUMP_STATE_IDLE:
            jc->current_target_y = jc->config.y_stand;
            jc->current_kp = 0.2f;
            jc->current_kd = 0.1f;
            break;

        case JUMP_STATE_SQUAT:
            jc->current_target_y = smooth_interp(jc->config.y_stand, jc->config.y_squat, elapsed, jc->config.t_squat);
            jc->current_kp = 0.2f; // 蓄力时刚度稍微变大
            jc->current_kd = 0.2f;
            if (elapsed >= jc->config.t_squat) {
                jc->current_state = JUMP_STATE_THRUST;
                jc->state_start_time = current_time;
            }
            break;

        case JUMP_STATE_THRUST:
            // 蹬地阶段使用线性插值，期望位置迅速推到最远，强迫电机输出峰值电流
            jc->current_target_y = linear_interp(jc->config.y_squat, jc->config.y_extend, elapsed, jc->config.t_thrust);
            jc->current_kp = 0.3f; // 【关键】蹬地阶段刚度拉满！输出最大力矩
            jc->current_kd = 0.1f; // 降低阻尼，避免阻碍速度
            if (elapsed >= jc->config.t_thrust) {
                jc->current_state = JUMP_STATE_FLIGHT;
                jc->state_start_time = current_time;
            }
            break;

        case JUMP_STATE_FLIGHT:
            jc->current_target_y = smooth_interp(jc->config.y_extend, jc->config.y_flight, elapsed, jc->config.t_flight);
            jc->current_kp = 0.2f; // 空中刚度恢复中等
            jc->current_kd = 0.2f;
            if (elapsed >= jc->config.t_flight) {
                jc->current_state = JUMP_STATE_LANDING;
                jc->state_start_time = current_time;
            }
            break;

        case JUMP_STATE_LANDING:
            jc->current_target_y = smooth_interp(jc->config.y_flight, jc->config.y_stand, elapsed, jc->config.t_landing);
            jc->current_kp = 0.15f; // 【关键】落地时降低刚度(Kp)，变得柔软(Compliance)
            jc->current_kd = 0.2f;  // 增大阻尼(Kd)吸收动能，防止弹跳
            if (elapsed >= jc->config.t_landing) {
                jc->current_state = JUMP_STATE_IDLE; // 完成跳跃，恢复站立
            }
            break;
    }

    // 调用逆运动学解算并下发指令，X轴保持为0 (原地跳跃)
    // 注意：需要修改原有的 InverseKinematics 接受 kp 和 kd 作为参数
    InverseKinematics(0.0f, jc->current_target_y, ctrl, hip_id, calf_id, jc->current_kp, jc->current_kd);
}

