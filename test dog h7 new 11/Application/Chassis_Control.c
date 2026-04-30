/*
 * Chassis_Control.c
 *
 * Created on: Mar 27, 2026
 * Author: 22569
 */
#include "Chassis_Control.h"
#include "Attitude_solution.h" // 引入IMU姿态解算数据
#include "kalman.h"

/* 存储四条腿实时高度补偿的数组 */
float leg_y_offsets[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* 【新增】：四条腿的触地权重 (默认全部在地上) */
static float leg_weights[4] = {1.0f, 1.0f, 1.0f, 1.0f};

/* 期望的动态姿态目标 */
static float expected_target_roll = 0.0f;
static float expected_target_pitch = 0.0f;

/* PID 控制器实例：参数需根据你的机器狗重量和腿长比例微调 */
static PID_Controller_t pid_roll  = { .kp = 0.8f, .ki = 0.02f, .kd = 0.1f, .integral_limit = 15.0f };
static PID_Controller_t pid_pitch = { .kp = 0.8f, .ki = 0.02f, .kd = 0.1f, .integral_limit = 15.0f };

/* ========== 基础 PID 计算 ========== */
static float PID_Calculate(PID_Controller_t *pid, float target, float current, float dt)
{
    pid->error = target - current;
    pid->integral += pid->error * dt;

    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = (pid->error - pid->last_error) / dt;
    pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    pid->last_error = pid->error;

    return pid->output;
}

void Chassis_Control_Init(void)
{
    expected_target_roll = 0.0f;
    expected_target_pitch = 0.0f;
    for(int i=0; i<4; i++) leg_weights[i] = 1.0f;
}

void Chassis_Set_Target_Attitude(float target_roll, float target_pitch)
{
    expected_target_roll = target_roll;
    expected_target_pitch = target_pitch;
}

/* ====== 【核心新增】：步态规划调用，告诉底层哪条腿在空中 ====== */
void Chassis_Set_Leg_Weight(uint8_t leg_index, float weight)
{
    if(leg_index < 4) {
        // 限制权重范围在 0.0 到 1.0 之间
        if(weight > 1.0f) weight = 1.0f;
        if(weight < 0.0f) weight = 0.0f;
        leg_weights[leg_index] = weight;
    }
}

/* ====== 【便利新增】：一次性设置四条腿的权重 ====== */
void Chassis_Set_All_Leg_Weights(float w0, float w1, float w2, float w3)
{
    Chassis_Set_Leg_Weight(0, w0);
    Chassis_Set_Leg_Weight(1, w1);
    Chassis_Set_Leg_Weight(2, w2);
    Chassis_Set_Leg_Weight(3, w3);
}

/* ====== 姿态补偿主循环 ====== */
void Chassis_Attitude_Loop(float dt)
{
    float target_roll = expected_target_roll;
    float target_pitch = expected_target_pitch;

    // 1. 获取姿态误差的补偿量
    float roll_comp = PID_Calculate(&pid_roll, target_roll, g_Attitude.Angle_Roll, dt);
    float pitch_comp = PID_Calculate(&pid_pitch, target_pitch, g_Attitude.Angle_Pitch, dt);

    // 2. 增加绝对限幅
    if(pid_roll.output > 15.0f) pid_roll.output = 15.0f;
    else if(pid_roll.output < -15.0f) pid_roll.output = -15.0f;

    if(pid_pitch.output > 15.0f) pid_pitch.output = 15.0f;
    else if(pid_pitch.output < -15.0f) pid_pitch.output = -15.0f;

    roll_comp = pid_roll.output;
    pitch_comp = pid_pitch.output;

    /* 3. 动力分配到四条腿 (乘以触地权重！) */
    // 只有权重大于0的支撑腿，才会参与姿态调节
    leg_y_offsets[0] = (-pitch_comp + roll_comp) * leg_weights[0]; // 前左
    leg_y_offsets[2] = (-pitch_comp - roll_comp) * leg_weights[2]; // 前右
    leg_y_offsets[3] = ( pitch_comp + roll_comp) * leg_weights[3]; // 后左
    leg_y_offsets[1] = ( pitch_comp - roll_comp) * leg_weights[1]; // 后右
}

float Chassis_Get_Leg_Offset(uint8_t leg_index)
{
    if(leg_index < 4) return leg_y_offsets[leg_index];
    return 0.0f;
}
//void state_zero_with_compensation(void)
//{
//    // 1. 因为是原地站立，四条腿都在地上，权重全设为最高 (1.0)
//    Chassis_Set_All_Leg_Weights(1.0f, 1.0f, 1.0f, 1.0f);
//
//    // 2. 期望姿态为绝对水平（Roll=0, Pitch=0）
//    Chassis_Set_Target_Attitude(0.0f, 0.0f);
//    float dt = 0.005f;
//    Chassis_Attitude_Loop(dt);
//
//    // 4. 将算出的补偿量叠加到默认站立高度上
//    // 注意：这里 default_y 的值需要替换为你机器狗正常的站立腿长(比如 120.0f 或 0.15f 等)
//    float default_x = 0.0f;
//    float default_y = 25.0f; // ★ 请务必改成你自己的零位 Y 坐标
//
//    // 5. 遍历四条腿，进行逆运动学解算并下发给电机
//    for(int i = 0; i < 4; i++) {
//        // 从底层获取 PID 算出来的单腿高度偏移量
//        float y_offset = Chassis_Get_Leg_Offset(i);
//
//        // 目标位置 = 基础站立位置 + 补偿量
//        float target_y = default_y + y_offset;
//      }
//}
