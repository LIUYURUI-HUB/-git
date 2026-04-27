#include "gait.h"
#include <string.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 电机物理/软限位定义 */
#define MOTOR_HIP_MAX   150.0f
#define MOTOR_HIP_MIN   -150.0f
#define MOTOR_KNEE_MAX  160.0f
#define MOTOR_KNEE_MIN  5.0f

/**
 * @brief 角度限位保护
 */
static float clamp_angle(float angle, float min_val, float max_val) {
    if (isnan(angle)) return 0.0f;
    if (angle > max_val) return max_val;
    if (angle < min_val) return min_val;
    return angle;
}

/**
 * @brief 初始化步态参数
 */
void gait_trot(QuadrupedGait *gait){
	set_gait_type(gait, 0.0f, 0.0f, 0.5f, 0.5f);
}
void gait_walk(QuadrupedGait *gait){
	set_gait_type(gait, 0.0f, 0.5f, 0.25f, 0.75f);
}
void init_quadruped_gait_trot(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height)
{
    memset(gait, 0, sizeof(QuadrupedGait));
    gait->gait_cycle_time = cycle_time;
    gait->stride_length = stride_length;
    gait->lift_height = lift_height;
    gait->is_running = 0;

    for (uint8_t i = 0; i < 4; i++) {
        gait->legs[i].x_base = 0;
        gait->legs[i].y_base = 0;
        // 默认初始化相位为 0
        gait->legs[i].phase_offset = 0.0f;
    }
    gait_trot(gait);

}
void init_quadruped_gait_walk(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height)
{
    memset(gait, 0, sizeof(QuadrupedGait));
    gait->gait_cycle_time = cycle_time;
    gait->stride_length = stride_length;
    gait->lift_height = lift_height;
    gait->is_running = 0;

    for (uint8_t i = 0; i < 4; i++) {
        gait->legs[i].x_base = 0;
        gait->legs[i].y_base = 0;
        // 默认初始化相位为 0
        gait->legs[i].phase_offset = 0.0f;
    }
    gait_walk(gait);
}
/**
 * @brief 设置单条腿的相位偏移
 * @param phase_offset 相位偏移，范围 [0.0, 1.0)
 */
void set_leg_phase_offset(QuadrupedGait *gait, uint8_t leg_index, float phase_offset)
{
    if (leg_index >= 4) return;

    // 限制在 0.0 ~ 1.0 之间
    while (phase_offset >= 1.0f) phase_offset -= 1.0f;
    while (phase_offset < 0.0f)  phase_offset += 1.0f;

    gait->legs[leg_index].phase_offset = phase_offset;
}

/**
 * @brief 快速配置四条腿的整体步态类型
 */
void set_gait_type(QuadrupedGait *gait, float phase_0, float phase_1, float phase_2, float phase_3)
{
    set_leg_phase_offset(gait, 0, phase_0);
    set_leg_phase_offset(gait, 1, phase_1);
    set_leg_phase_offset(gait, 2, phase_2);
    set_leg_phase_offset(gait, 3, phase_3);
}

/**
 * @brief 校准基准位置
 */
void calibrate_leg_base_position(QuadrupedGait *gait, uint8_t leg_index, MotorController *ctrl,int hip_id,int knee_id,int i)
{
    if (leg_index >= 4) return;
    Currentpos pos = ForwardKinematics(ctrl,hip_id,knee_id);
    gait->legs[leg_index].x_base = pos.X;
    gait->legs[leg_index].y_base = pos.Y;
    gait->legs[leg_index].direction = i;
}

/**
 * @brief 核心摆线轨迹生成（Y向下为正版本）
 */
Cycloid2D_Pose calculate_cycloid_step(float dt, float x_start, float x_end, float y_base, float lift_height, uint8_t is_swing)
{
    Cycloid2D_Pose pose;

    if (dt < 0.0f) dt = 0.0f;
    if (dt > 1.0f) dt = 1.0f;

    if (is_swing) {
        float phase = 2.0f * PI * dt;
        pose.x = x_start + (x_end - x_start) * (dt - sinf(phase) / (2.0f * PI));
        pose.y = y_base - lift_height * 0.5f * (1.0f - cosf(phase));
    } else {
        float smooth_dt = dt - sinf(2.0f * PI * dt) / (2.0f * PI);
        pose.x = x_start + (x_end - x_start) * smooth_dt;
        pose.y = y_base;
    }

    return pose;
}

/**
 * @brief 获取当前腿的轨迹点 (全新相位可调版本)
 */
Cycloid2D_Pose get_leg_trajectory(QuadrupedGait *gait, uint8_t leg_index, float current_time)
{
    LegGaitState *leg = &gait->legs[leg_index];
    if (!gait->is_running) return (Cycloid2D_Pose){leg->x_base, leg->y_base};

    float T = gait->gait_cycle_time;
    float elapsed_time = current_time - gait->start_time;

    // --- 核心改动：用单腿独立的相代替原本的 GroupA/GroupB ---
    // elapsed_time / T 是逝去的周期数，加上各腿自带的相位偏移，计算出这条腿的“虚拟总周期”
    float leg_total_cycles = (elapsed_time / T) + leg->phase_offset;
    uint32_t leg_cycle_count = (uint32_t)leg_total_cycles;
    float t_norm = leg_total_cycles - (float)leg_cycle_count; // 取小数部分，归一化时间 t_norm 属于 [0.0, 1.0)

    float half_S = gait->stride_length / 2.0f;
    float x_back  = leg->x_base - half_S;
    float x_front = leg->x_base + half_S;

    float swing_x_start, swing_x_end;
    if (leg->direction == 1) {
        swing_x_start = x_back;
        swing_x_end   = x_front;
    } else {
        swing_x_start = x_front;
        swing_x_end   = x_back;
    }

    float dt;
    uint8_t phase_is_swing;
    float current_x_start, current_x_end;

    // 我们目前保持触地比 (Duty Factor) 为 0.5
    // 即前 50% 是摆动相 (Swing)，后 50% 是支撑相 (Stance)
    if (t_norm < 0.5f) {
        phase_is_swing = 1;         // 正在抬腿摆动
        dt = t_norm * 2.0f;         // 将 0.0~0.5 映射到 0.0~1.0

        // 【新增】：告诉底盘控制，这条腿在空中，权重设为 0，千万别给它加姿态补偿！
        Chassis_Set_Leg_Weight(leg_index, 0.0f);

        // 如果是该腿启动的第一步（还没有经过一次完整的支撑相），则直接从原点起步，防止瞬间跳跃
        current_x_start = (leg_cycle_count == 0) ? leg->x_base : swing_x_start;
        current_x_end = swing_x_end;
    } else {
        phase_is_swing = 0;         // 落地支撑向后划
        dt = (t_norm - 0.5f) * 2.0f; // 将 0.5~1.0 映射到 0.0~1.0

        // 【新增】：告诉底盘控制，这条腿踩在地上了，权重设为 1，用它来平衡机身！
        Chassis_Set_Leg_Weight(leg_index, 1.0f);


        // 第一步如果落在支撑相，也是从原点开始向后划
        current_x_start = (leg_cycle_count == 0) ? leg->x_base : swing_x_end;
        current_x_end = swing_x_start;
    }

    return calculate_cycloid_step(dt, current_x_start, current_x_end, leg->y_base, gait->lift_height, phase_is_swing);
}

/**
 * @brief 角度解算（含坐标系适配）- 修复版
 */
LegAngles get_leg_angles(QuadrupedGait* gait, int leg_id, float time, MotorController* ctrl, int id1, int id2, float kp, float kd, float y_offset)
{
    // 统一使用传入的变量 leg_id 和 time
    Cycloid2D_Pose pose = get_leg_trajectory(gait, leg_id, time);

    // 核心：加上 Y 方向姿态补偿
    pose.y = pose.y + y_offset;

    if (pose.y < 20.0f) pose.y = 20.0f;

    // 统一使用传入的变量 id1 和 id2
    LegAngles result = InverseKinematics(pose.x, pose.y, ctrl, id1, id2, kp, kd);

    if (isnan(result.theta1) || isnan(result.theta2)) {
        return gait->legs[leg_id].current_angles;
    }

    result.theta1 = clamp_angle(result.theta1, MOTOR_HIP_MIN, MOTOR_HIP_MAX);
    result.theta2 = clamp_angle(result.theta2, MOTOR_KNEE_MIN, MOTOR_KNEE_MAX);

    gait->legs[leg_id].current_angles = result;
    return result;
}

void start_quadruped_gait(QuadrupedGait *gait1,float current_time)
{
    gait1->start_time = current_time;
    gait1->is_running = 1;
}
