//#include "gait.h"
//#include <string.h>
//#include <math.h>
//
//#ifndef PI
//#define PI 3.14159265358979323846f
//#endif
//
///* 电机物理/软限位定义 */
//#define MOTOR_HIP_MAX   150.0f
//#define MOTOR_HIP_MIN   -150.0f
//#define MOTOR_KNEE_MAX  160.0f
//#define MOTOR_KNEE_MIN  5.0f
//
///**
// * @brief 角度限位保护
// */
//static float clamp_angle(float angle, float min_val, float max_val) {
//    if (isnan(angle)) return 0.0f;
//    if (angle > max_val) return max_val;
//    if (angle < min_val) return min_val;
//    return angle;
//}
//
///**
// * @brief 动态缓冲参数设置
// */
//void set_cushion_params(QuadrupedGait *gait, float depth, float duration) {
//    gait->cushion_depth = depth;
//    if (duration > 0.5f) duration = 0.5f;
//    if (duration < 0.0f) duration = 0.0f;
//    gait->cushion_duration = duration;
//}
//
//void set_gait_duty_factor(QuadrupedGait *gait, float duty_factor) {
//    if (duty_factor < 0.05f) duty_factor = 0.05f;
//    if (duty_factor > 0.95f) duty_factor = 0.95f;
//    gait->duty_factor = duty_factor;
//}
//void gait_trot(QuadrupedGait *gait){
//    set_gait_type(gait, 0.5f, 0.5f, 0.0f, 0.0f);
//    set_gait_duty_factor(gait, 0.4f);
//}
//
//void gait_walk(QuadrupedGait *gait){
//    set_gait_type(gait, 0.0f, 0.5f, 0.25f, 0.75f);
//    set_gait_duty_factor(gait, 0.75f);
//}
//
//void gait_bound(QuadrupedGait *gait){
//    set_gait_type(gait, 0.0f, 0.0f, 0.5f, 0.5f);
//    set_gait_duty_factor(gait, 0.2f);
//}
//
//void gait_pronk(QuadrupedGait *gait){
//    // 四足齐跳
//    set_gait_type(gait, 0.0f, 0.0f, 0.0f, 0.0f);
//    set_gait_duty_factor(gait, 0.2f);
//}
//
//static void _base_init(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height) {
//    memset(gait, 0, sizeof(QuadrupedGait));
//    gait->gait_cycle_time = cycle_time;
//    gait->stride_length = stride_length;
//    gait->lift_height = lift_height;
//    gait->is_running = 0;
//
//    // 默认设置：10mm退让深度，占触地相前20%的时间
//    set_cushion_params(gait, 10.0f, 0.2f);
//
//    for (uint8_t i = 0; i < 4; i++) {
//        gait->legs[i].x_base = 0;
//        gait->legs[i].y_base = 0;
//        gait->legs[i].phase_offset = 0.0f;
//    }
//}
//
//void init_quadruped_gait_trot(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height) {
//    _base_init(gait, cycle_time, stride_length, lift_height);
//    gait_trot(gait);
//}
//void init_quadruped_gait_walk(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height) {
//    _base_init(gait, cycle_time, stride_length, lift_height);
//    gait_walk(gait);
//}
//void init_quadruped_gait_bound(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height) {
//    _base_init(gait, cycle_time, stride_length, lift_height);
//    gait_bound(gait);
//}
//void init_quadruped_gait_pronk(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height) {
//    _base_init(gait, cycle_time, stride_length, lift_height);
//    gait_pronk(gait);
//}
//
//void set_leg_phase_offset(QuadrupedGait *gait, uint8_t leg_index, float phase_offset) {
//    if (leg_index >= 4) return;
//    while (phase_offset >= 1.0f) phase_offset -= 1.0f;
//    while (phase_offset < 0.0f)  phase_offset += 1.0f;
//    gait->legs[leg_index].phase_offset = phase_offset;
//}
//
//void set_gait_type(QuadrupedGait *gait, float phase_0, float phase_1, float phase_2, float phase_3) {
//    set_leg_phase_offset(gait, 0, phase_0);
//    set_leg_phase_offset(gait, 1, phase_1);
//    set_leg_phase_offset(gait, 2, phase_2);
//    set_leg_phase_offset(gait, 3, phase_3);
//}
//
//void bind_leg_motors(QuadrupedGait *gait, uint8_t leg_index, uint8_t hip_id, uint8_t knee_id) {}
//
//void calibrate_leg_base_position(QuadrupedGait *gait, uint8_t leg_index, MotorController *ctrl,int hip_id,int knee_id,int i) {
//    if (leg_index >= 4) return;
//    Currentpos pos = ForwardKinematics(ctrl,hip_id,knee_id);
//    gait->legs[leg_index].x_base = pos.X;
//    gait->legs[leg_index].y_base = pos.Y;
//    gait->legs[leg_index].direction = i;
//}
//
///**
// * @brief 摆线轨迹生成
// */
//Cycloid2D_Pose calculate_cycloid_step(float dt, float x_start, float x_end, float y_base, float lift_height, uint8_t is_swing) {
//    Cycloid2D_Pose pose;
//    if (dt < 0.0f) dt = 0.0f;
//    if (dt > 1.0f) dt = 1.0f;
//
//    if (is_swing) {
//        float phase = 2.0f * PI * dt;
//        pose.x = x_start + (x_end - x_start) * (dt - sinf(phase) / (2.0f * PI));
//        pose.y = y_base - lift_height * 0.5f * (1.0f - cosf(phase));
//    } else {
//        float smooth_dt = dt - sinf(2.0f * PI * dt) / (2.0f * PI);
//        pose.x = x_start + (x_end - x_start) * smooth_dt;
//        pose.y = y_base;
//    }
//    return pose;
//}
//
///**
// * @brief 获取当前轨迹点（已融入虚拟悬挂轨迹缓冲）
// */
//Cycloid2D_Pose get_leg_trajectory(QuadrupedGait *gait, uint8_t leg_index, float current_time) {
//    LegGaitState *leg = &gait->legs[leg_index];
//    if (!gait->is_running) return (Cycloid2D_Pose){leg->x_base, leg->y_base};
//
//    float T = gait->gait_cycle_time;
//    float elapsed_time = current_time - gait->start_time;
//
//    float leg_total_cycles = (elapsed_time / T) + leg->phase_offset;
//    uint32_t leg_cycle_count = (uint32_t)leg_total_cycles;
//    float t_norm = leg_total_cycles - (float)leg_cycle_count;
//
//    float half_S = gait->stride_length / 2.0f;
//    float swing_x_start = (leg->direction == 1) ? (leg->x_base - half_S) : (leg->x_base + half_S);
//    float swing_x_end   = (leg->direction == 1) ? (leg->x_base + half_S) : (leg->x_base - half_S);
//
//    float stance_ratio = gait->duty_factor;
//    float swing_ratio  = 1.0f - stance_ratio;
//
//    float dt;
//    uint8_t phase_is_swing;
//    float current_x_start, current_x_end;
//
//    // 前半段为摆动相，后半段为支撑相
//    if (t_norm < swing_ratio) {
//        phase_is_swing = 1;
//        dt = t_norm / swing_ratio;
//        current_x_start = (leg_cycle_count == 0) ? leg->x_base : swing_x_start;
//        current_x_end = swing_x_end;
//    } else {
//        phase_is_swing = 0;
//        dt = (t_norm - swing_ratio) / stance_ratio;
//        current_x_start = (leg_cycle_count == 0) ? leg->x_base : swing_x_end;
//        current_x_end = swing_x_start;
//    }
//
//    // 更新状态记录以供刚度调节使用
//    leg->is_swing_phase = phase_is_swing;
//    leg->current_phase_dt = dt;
//
//    // --- 第一重保护：虚拟弹簧悬挂缓冲 (轨迹退让) ---
//    float target_y_base = leg->y_base;
//    if (phase_is_swing == 0 && gait->cushion_duration > 0.0f && dt < gait->cushion_duration) {
//        // 利用正弦波前180度(0~PI)模拟压缩与回弹
//        float cushion_phase = PI * (dt / gait->cushion_duration);
//        // Y向下为正，减去深度相当于腿长临时缩短，进行机械卸力
//        float y_cushion = -gait->cushion_depth * sinf(cushion_phase);
//        target_y_base += y_cushion;
//    }
//
//    return calculate_cycloid_step(dt, current_x_start, current_x_end, target_y_base, gait->lift_height, phase_is_swing);
//}
//
///**
// * @brief 角度解算（已融入阻尼调节缓冲）
// */
//LegAngles get_leg_angles(QuadrupedGait *gait, uint8_t leg_index, float current_time, MotorController* ctrl, int hip_id, int calf_id,float kp,float kd) {
//    Cycloid2D_Pose pose = get_leg_trajectory(gait, leg_index, current_time);
//    LegGaitState *leg = &gait->legs[leg_index];
//
//    // 防止逆解坐标撞到底盘
//    if (pose.y < 5.0f) pose.y = 5.0f;
//
//    // --- 第二重保护：动态阻抗/刚度缓冲 (软着陆) ---
//    float adj_kp = kp;
//    float adj_kd = kd;
//
//    if (gait->is_running) {
//        if (leg->is_swing_phase == 0) { // 支撑相(触地)
//            if (leg->current_phase_dt < gait->cushion_duration) {
//                // 刚落地瞬间：大幅降低刚度Kp，提升一点阻尼Kd吸收能量
//                float blend = leg->current_phase_dt / gait->cushion_duration;
//                adj_kp = kp * (0.7f + 0.5f * blend); // Kp 从 50% 逐渐恢复到 100%
//                adj_kd = kd * (1.4f - 0.2f * blend); // Kd 从 120% 逐渐恢复到 100%
//            } else {
//                // 支撑相平稳期：保持强壮的原始刚度
//                adj_kp = kp;
//                adj_kd = kd;
//            }
//        } else {
//            // 空中摆动相：没必要硬抗，稍微调软一点省电且动作更柔和
//            adj_kp = kp * 0.8f;
//            adj_kd = kd * 0.8f;
//        }
//    }
//
//    // 使用调节后的 adj_kp 和 adj_kd 计算并发送
//    LegAngles result = InverseKinematics(pose.x, pose.y, ctrl, hip_id, calf_id, adj_kp, adj_kd);
//
//    if (isnan(result.theta1) || isnan(result.theta2)) {
//        return leg->current_angles;
//    }
//
//    result.theta1 = clamp_angle(result.theta1, MOTOR_HIP_MIN, MOTOR_HIP_MAX);
//    result.theta2 = clamp_angle(result.theta2, MOTOR_KNEE_MIN, MOTOR_KNEE_MAX);
//
//    leg->current_angles = result;
//    return result;
//}
//
//void start_quadruped_gait(QuadrupedGait *gait, float current_time) {
//    gait->start_time = current_time;
//    gait->is_running = 1;
//}
//
//void stop_quadruped_gait(QuadrupedGait *gait) {
//    gait->is_running = 0;
//}



#include "gait.h"
#include <string.h>
#include <math.h>
#include "Chassis_Control.h"

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
    // 剥离当前姿态补偿值：站立时电机已包含补偿，步态轨迹会重新叠加，避免双倍补偿导致腿打直
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

    float leg_total_cycles = (elapsed_time / T) + leg->phase_offset;
    uint32_t leg_cycle_count = (uint32_t)leg_total_cycles;
    float t_norm = leg_total_cycles - (float)leg_cycle_count;

    float startup_blend = elapsed_time / (T * 2.0f);
    if (startup_blend < 0.0f) startup_blend = 0.0f;
    if (startup_blend > 1.0f) startup_blend = 1.0f;

    float half_S = (gait->stride_length * startup_blend) / 2.0f;
//    float half_S = gait->stride_length ;

    // 【已恢复：正确的物理坐标系映射】
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

    // 【修复瞬移 Bug】精确判断当前腿是否处于启动后的第一个“半步”内
    if (t_norm < 0.5f) {
        phase_is_swing = 1;         // 抬腿摆动
        dt = t_norm * 2.0f;
        // 只有真正意义上的第一步，才从原点起步，否则平滑衔接
        current_x_start = swing_x_start;
        current_x_end = swing_x_end;
    } else {
        phase_is_swing = 0;         // 落地支撑
        dt = (t_norm - 0.5f) * 2.0f;
        current_x_start = swing_x_end;
        current_x_end = swing_x_start;
    }

    leg->is_swing_phase = phase_is_swing;
    Chassis_Set_Leg_Weight(leg_index, phase_is_swing ? 0.0f : 1.0f);

    return calculate_cycloid_step(dt, current_x_start, current_x_end, leg->y_base, gait->lift_height * startup_blend, phase_is_swing);
}
//Cycloid2D_Pose get_leg_trajectory(QuadrupedGait *gait, uint8_t leg_index, float current_time)
//{
//    LegGaitState *leg = &gait->legs[leg_index];
//    if (!gait->is_running) return (Cycloid2D_Pose){leg->x_base, leg->y_base};
//
//    float T = gait->gait_cycle_time;
//    float elapsed_time = current_time - gait->start_time;
//
//    // --- 核心改动：用单腿独立的相代替原本的 GroupA/GroupB ---
//    // elapsed_time / T 是逝去的周期数，加上各腿自带的相位偏移，计算出这条腿的“虚拟总周期”
//    float leg_total_cycles = (elapsed_time / T) + leg->phase_offset;
//    uint32_t leg_cycle_count = (uint32_t)leg_total_cycles;
//    float t_norm = leg_total_cycles - (float)leg_cycle_count; // 取小数部分，归一化时间 t_norm 属于 [0.0, 1.0)
//
//    float half_S = gait->stride_length / 2.0f;
//    float x_back  = leg->x_base - half_S;
//    float x_front = leg->x_base + half_S;
//
//    float swing_x_start, swing_x_end;
//    if (leg->direction == 1) {
//        swing_x_start = x_back;
//        swing_x_end   = x_front;
//    } else {
//        swing_x_start = x_front;
//        swing_x_end   = x_back;
//    }
//
//    float dt;
//    uint8_t phase_is_swing;
//    float current_x_start, current_x_end;
//    if (t_norm < 0.5f) {
//        phase_is_swing = 1;         // 正在抬腿摆动
//        dt = t_norm * 2.0f;         // 将 0.0~0.5 映射到 0.0~1.0
//
//        // 如果是该腿启动的第一步（还没有经过一次完整的支撑相），则直接从原点起步，防止瞬间跳跃
//        current_x_start = (leg_cycle_count == 0) ? leg->x_base : swing_x_start;
//        current_x_end = swing_x_end;
//    } else {
//        phase_is_swing = 0;         // 落地支撑向后划
//        dt = (t_norm - 0.5f) * 2.0f; // 将 0.5~1.0 映射到 0.0~1.0
//
//        // 第一步如果落在支撑相，也是从原点开始向后划
//        current_x_start = (leg_cycle_count == 0) ? leg->x_base : swing_x_end;
//        current_x_end = swing_x_start;
//    }
//
//    return calculate_cycloid_step(dt, current_x_start, current_x_end, leg->y_base, gait->lift_height, phase_is_swing);
//}

/**
 * @brief 角度解算（含坐标系适配）- 修复版
 */
/**
 * @brief 角度解算（含坐标系适配）- 修复版
 */
LegAngles get_leg_angles(QuadrupedGait *gait, uint8_t leg_index, float current_time, MotorController* ctrl, int hip_id, int calf_id,float kp,float kd)
{
    // 1. 获取原本步态算出的当前坐标
    Cycloid2D_Pose pose = get_leg_trajectory(gait, leg_index, current_time);

    // ==========================================
    // 【核心新增】：将底盘姿态补偿量直接叠加到步态足端Y
    // ==========================================
    pose.y += Chassis_Get_Leg_Offset(leg_index);

    // 2. 原本的防撞底盘限位 (这句保留在补偿之后很安全，防止补偿把腿缩得太短)
    if (pose.y < 20.0f) pose.y = 20.0f;
    if (pose.y > 32.0f) pose.y = 32.0f;

    // 3. 送去逆解
    LegAngles result = InverseKinematics(pose.x, pose.y, ctrl, hip_id, calf_id, kp, kd);

    if (isnan(result.theta1) || isnan(result.theta2)) {
        return gait->legs[leg_index].current_angles;
    }

    result.theta1 = clamp_angle(result.theta1, MOTOR_HIP_MIN, MOTOR_HIP_MAX);
    result.theta2 = clamp_angle(result.theta2, MOTOR_KNEE_MIN, MOTOR_KNEE_MAX);

    gait->legs[leg_index].current_angles = result;
    return result;
}

void start_quadruped_gait(QuadrupedGait *gait1,float current_time)
{
    gait1->start_time = current_time;
    gait1->is_running = 1;
}

void stop_quadruped_gait(QuadrupedGait *gait)
{
    gait->is_running = 0;
    // 将所有腿的触地权重复位，确保姿态补偿在站立模式下正常工作
    Chassis_Set_All_Leg_Weights(1.0f, 1.0f, 1.0f, 1.0f);
}
