/*
 * Attitude_Compensation.c
 *
 *  Created on: May 13, 2026
 *      Author: Changan Zhou
 */
#include "Chassis_Control.h"
#include "Attitude_solution.h" // 引入IMU姿态解算数据
#include "kalman.h"
#include <math.h>

/* 存储四条腿实时高度补偿的数组 */
float leg_y_offsets[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* 【新增】：四条腿的触地权重 (默认全部在地上) */
static float leg_weights[4] = {1.0f, 1.0f, 1.0f, 1.0f};

/* 期望的动态姿态目标 */
static float expected_target_roll = 0.0f;
static float expected_target_pitch = 0.0f;
static float attitude_comp_scale = 0.0f;

void Chassis_Control_Init(void)
{
    expected_target_roll = 0.0f;
    expected_target_pitch = 0.0f;
    for(int i=0; i<4; i++) {
        leg_weights[i] = 1.0f;
        leg_y_offsets[i] = 0.0f;
    }
    attitude_comp_scale = 0.0f;
}

void Chassis_Set_Target_Attitude(float target_roll, float target_pitch)
{
    expected_target_roll = target_roll;
    expected_target_pitch = target_pitch;
}

void Chassis_Reset_Attitude_Filter(void)
{
    attitude_comp_scale = 0.0f;
    for (int i = 0; i < 4; i++) {
        leg_y_offsets[i] = 0.0f;
    }
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

/* ====== 姿态补偿主循环：根据腿触地权重混合补偿量，低通滤波平滑过渡 ====== */
void Chassis_Attitude_Loop(float dt)
{
    const float Y_KP = 1.00f;
    const float MAX_TOTAL_COMP = 4.0f;
    const float ANGLE_DEADZONE = 0.8f;
    const float COMP_RAMP_TIME = 1.0f;
    const float OFFSET_SMOOTH = 0.85f; // 低通系数：越大越平滑但响应越慢
    float L_arm_length = 22.75f / 2.0f;
    float L_arm_width  = 8.69f / 2.0f;

    float current_pitch = g_Attitude.Angle_Pitch;
    float current_roll  = g_Attitude.Angle_Roll;

    float pitch_error = current_pitch - expected_target_pitch;
    float roll_error  = current_roll - expected_target_roll;

    if (dt <= 0.0f) {
        dt = 0.005f;
    }
    if (attitude_comp_scale < 1.0f) {
        attitude_comp_scale += dt / COMP_RAMP_TIME;
        if (attitude_comp_scale > 1.0f) {
            attitude_comp_scale = 1.0f;
        }
    }

    float pitch_rad = pitch_error * 3.14159f / 180.0f;
    float roll_rad  = roll_error * 3.14159f / 180.0f;

    float base_pitch_delta = L_arm_length * sinf(pitch_rad);
    float base_roll_delta  = L_arm_width * sinf(roll_rad);

    float target_delta[4];
    target_delta[0] =  base_pitch_delta + base_roll_delta; // FL: front left
    target_delta[1] = -base_pitch_delta - base_roll_delta; // RR: rear right
    target_delta[2] =  base_pitch_delta - base_roll_delta; // FR: front right
    target_delta[3] = -base_pitch_delta + base_roll_delta; // RL: rear left

    uint8_t all_legs_in_air = (leg_weights[0] < 0.1f && leg_weights[1] < 0.1f &&
                               leg_weights[2] < 0.1f && leg_weights[3] < 0.1f);

    for (int i = 0; i < 4; i++) {
        float raw_output = 0.0f;

        if (!all_legs_in_air && leg_weights[i] > 0.1f) {
            if (fabsf(pitch_error) > ANGLE_DEADZONE || fabsf(roll_error) > ANGLE_DEADZONE) {
                raw_output = target_delta[i] * Y_KP;
            }

            if (raw_output > MAX_TOTAL_COMP)  raw_output = MAX_TOTAL_COMP;
            if (raw_output < -MAX_TOTAL_COMP) raw_output = -MAX_TOTAL_COMP;

            // 权重混合：触地权重连续缩放补偿量，避免落地/离地瞬间足端跳变
            raw_output *= leg_weights[i] * attitude_comp_scale;
        }

        // 一阶低通滤波：平滑补偿量变化，消除 IMU 噪声引起的足端抖动
        leg_y_offsets[i] = OFFSET_SMOOTH * leg_y_offsets[i]
                         + (1.0f - OFFSET_SMOOTH) * raw_output;
    }
}


float Chassis_Get_Leg_Offset(uint8_t leg_index)
{
    if(leg_index < 4) return leg_y_offsets[leg_index];
    return 0.0f;
}
