#include "leg_kinematics.h"
#include "motor_controller.h"
#include "gom_protocol.h"
#include <math.h>

extern MotorController ctrl1; // 仅保留硬件句柄用于发送指令
extern MotorController ctrl2;


/**
 * @brief 正运动学 (FK): 实际电机角度 -> 实际足端坐标
 * @param config  机械参数配置
 * @param hip_ret 髋关节电机反馈数据
 * @param knee_ret 膝关节电机反馈数据
 * @param pos     输出的足端实际坐标 (m)
 */
void leg_fk(LegConfig_T *config, FootPos_T *pos ,LegMotors_T *motors,MotorData_t *data1,MotorData_t *data2) {

    // 1. 从电机反馈结构体中提取当前位置 (Pos)
    // 2. 角度转换为弧度并补偿安装偏移
	motors->cur_hip_q1=data1->Pos;
	motors->cur_knee_q2 =data2->Pos;

    float q1 = motors->cur_hip_q1 + ANGLE_TO_RAD(config->offset_hip);
    float q2 = motors->cur_knee_q2  + ANGLE_TO_RAD(config->offset_knee);

    // 3. 几何解算
    pos->x = config->L1 * sinf(q1) + config->L2 * sinf(q2);
    pos->z = config->L1 * cosf(q1) + config->L2 * cosf(q2);
}

/**
 * @brief 逆运动学解算 (IK) - 将目标坐标转换为电机角度
 */
int8_t leg_ik(LegConfig_T *config, FootPos_T *pos, LegMotors_T *motors) {
    float x = pos->x;
    float z = pos->z;

    float d_sq = x * x + z * z;
    float d = sqrtf(d_sq);

    // 1. 物理范围检查
    if (d > (config->L1 + config->L2) || d < fabsf(config->L1 - config->L2)) {
        return -1; // 目标点不可达
    }

    // 2. 余弦定理求膝关节内角 phi（大小腿间夹角）

    float cos_phi = (config->L1 * config->L1 + config->L2 * config->L2 - d_sq) /
                    (2.0f * config->L1 * config->L2);
    if (cos_phi > 1.0f) cos_phi = 1.0f;
    if (cos_phi < -1.0f) cos_phi = -1.0f;
    float phi = acosf(cos_phi);

    // 3. 计算方位角 alpha 和修正角 gamma
    // alpha: 髋关节到足端连线与垂直方向的夹角。atan2 自动处理 x 的正负。
    // gamma: 大腿连杆与 (髋关节-足端) 连线之间的夹角
    float alpha = atan2f(x, z);
    float cos_gamma = (config->L1 * config->L1 + d_sq - config->L2 * config->L2) /
                      (2.0f * config->L1 * d);
    if (cos_gamma > 1.0f) cos_gamma = 1.0f;
    if (cos_gamma < -1.0f) cos_gamma = -1.0f;
    float gamma = acosf(cos_gamma);

    // 4. 映射到电机指令

    float hip_angle = alpha - gamma;

    // 5. 计算小腿绝对角
    // 空间绝对角 = 大腿角 - (PI - 膝关节内角)
    // 这里 PI - phi 是小腿相对于大腿的转动量
    float knee_angle = hip_angle + (PI - phi);

    // 6.映射到电机绝对角度（减去安装零位）
    motors->target_hip_q1 = hip_angle - ANGLE_TO_RAD(config->offset_hip);
    motors->target_knee_q2 = knee_angle - ANGLE_TO_RAD(config->offset_knee);


    return 0;
}


/**
 * @brief 摆线步态生成
 * @param start_action: 0为组A(首发摆动腿)，1为组B(首发支撑腿)
 */
FootPos_T cycloidal_gait(Cycloidal_T *gait, float walk_t, uint8_t start_action) {
    FootPos_T out;
    float T = gait->T;
    float S = gait->S;
    float H = gait->H;
    float z_base = gait->z_base;
    float stance_ratio = gait->stance_ratio;

    float swing_t = T * (1.0f - stance_ratio);
    float stance_t = T * stance_ratio;

    // STARTUP 启动阶段：走起步的前半个周期 (0 < walk_t < swing_t)
    // 实现从站立原点 (x=0) 到稳态两极 (+S/2 与 -S/2) 的过渡

    if (walk_t < swing_t) {
        if (start_action == 0) {
            // 组 A: 首发动作是【摆动】
            // 步幅系数减半，起始偏移量设为 0 (执行 0 -> +S/2)
            float phase = 2.0f * PI * (walk_t / swing_t);
            float S_startup = S / 2.0f;
            out.x = (S_startup / (2.0f * PI)) * (phase - sinf(phase));
            out.z = z_base - H * (1.0f - cosf(phase)) / 2.0f;
        } else {
            // 组 B: 首发动作是【支撑】
            // 从 0 开始向后推地，到周期结束时达到 -S/2
            float t_s = walk_t;
            out.x = 0.0f - (S / 2.0f) * (t_s / stance_t);
            out.z = z_base;
        }
        return out;
    }


    // 稳态阶段 (walk_t >= swing_t)
    // 半个周期后，双腿都已到达标准位置 (-S/2 和 +S/2)，切入常规摆线公式
    float local_t = walk_t;
    if (start_action) {
        // 组 B 在稳态阶段相位延迟半个周期
        local_t += swing_t;
    }

    float curr_t = fmodf(local_t, T);

    if (curr_t < swing_t) {
        // 稳态摆动相：-S/2 -> +S/2
        float phase = 2.0f * PI * (curr_t / swing_t);
        out.x = (S / (2.0f * PI)) * (phase - sinf(phase)) - (S / 2.0f);
        out.z = z_base - H * (1.0f - cosf(phase)) / 2.0f;
    } else {
        // 稳态支撑相：+S/2 -> -S/2
        float t_s = curr_t - swing_t;
        out.x = (S / 2.0f) - (S * (t_s / stance_t));
        out.z = z_base;
    }

    return out;
}


/**
 * @brief 计算单腿的轮子目标角速度
 */
float calculate_wheel_w(float walk_t, LegConfig_T *config, Cycloidal_T *gait, uint8_t start_action) {
    if (gait->S <= 0.0f) return 0.0f;

    float T = gait->T;
    float swing_t = T * (1.0f - gait->stance_ratio);
    float stance_t = T * gait->stance_ratio;

    // STARTUP 阶段的轮速
    if (walk_t < swing_t) {
        if (start_action == 0) {
            return 0.0f; // 组 A：腾空状态，轮子停转
        } else {
            // 组 B：向后推地距离 S/2，耗时 stance_t。速度是稳态的一半
            float wheel_v = (gait->S / 2.0f) / stance_t;
            return wheel_v / config->wheel_r;
        }
    }

    // STABLE 阶段的轮速
    float local_t = walk_t;
    if (start_action) {
        local_t += swing_t;
    }
    float curr_t = fmodf(local_t, T);

    if (curr_t >= swing_t) {
        // 稳态支撑相：推地距离为 S，耗时 stance_t
        float wheel_v = gait->S / stance_t;
        return wheel_v / config->wheel_r;
    } else {
        return 0.0f;
    }
}

/**
 * @brief 四足 Trot 步态
 * @param time 系统当前运行时间
 * @param fl, fr, hl, rr 四条腿对应的电机控制结构体
 */

int8_t Leg_Control(float time, LegMotors_T *fl, LegMotors_T *fr, LegMotors_T *rl, LegMotors_T *rr, LegConfig_T *config, Cycloidal_T *gait)  {
    FootPos_T front_left, front_right, rear_left, rear_right;
    float T_prep = 5.0f;

    if (time < T_prep) {
        // --- 待机阶段 ---
        front_left.x = front_right.x = rear_left.x = rear_right.x = 0.0f;
        front_left.z = front_right.z = rear_left.z = rear_right.z = gait->z_base;
        fl->wheel_w = fr->wheel_w = rl->wheel_w = rr->wheel_w = 0.0f;
    } else {
        // --- 行走阶段 ---
        // walk_t 传入 cycloidal_gait 后，函数内部会自动进行启动阶段到稳定阶段的平滑切换
        float walk_t = time - T_prep;

        // 组 A
        front_left = cycloidal_gait(gait, walk_t, 0);
        rear_right = cycloidal_gait(gait, walk_t, 0);

        // 组 B
        front_right = cycloidal_gait(gait, walk_t, 1);
        rear_left = cycloidal_gait(gait, walk_t, 1);

        // 轮速计算
        float w_A = calculate_wheel_w(walk_t, config, gait, 0);
        fl->wheel_w = w_A;
        rr->wheel_w = w_A;

        float w_B = calculate_wheel_w(walk_t, config, gait, 1);
        fr->wheel_w = w_B;
        rl->wheel_w = w_B;
    }

    // 逆运动学映射到各个腿
    int8_t res = 0;
    res |= leg_ik(config, &front_left, fl);
    res |= leg_ik(config, &front_right, fr);
    res |= leg_ik(config, &rear_left, rl);
    res |= leg_ik(config, &rear_right, rr);

    return res;
}
