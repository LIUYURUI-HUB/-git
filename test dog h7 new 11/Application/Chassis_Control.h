/*
 * Chassis_Control.h
 *
 * Created on: Mar 27, 2026
 * Author: 22569
 */

#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include <stdint.h>

/* ========== 结构体定义 ========== */
// 简易PID参数结构体
typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float output;
    float integral_limit; // 积分限幅
} PID_Controller_t;

/* ========== 外部接口函数 ========== */

/**
 * @brief  初始化底盘姿态控制模块
 */
void Chassis_Control_Init(void);

/**
 * @brief  设置底盘期望的动态姿态角 (由步态规划器实时调用)
 * @param  target_roll  期望横滚角(度)
 * @param  target_pitch 期望俯仰角(度)
 */
void Chassis_Set_Target_Attitude(float target_roll, float target_pitch);

/**
 * @brief  【核心新增】：设置指定腿的触地权重 (用于隔离空中腿的补偿)
 * @param  leg_index 腿编号 (0:前左, 2:前右, 3:后左, 1:后右)
 * @param  weight    权重系数 (1.0f: 完全在地面支撑; 0.0f: 完全在空中摆动)
 */
void Chassis_Set_Leg_Weight(uint8_t leg_index, float weight);

/**
 * @brief  【便利新增】：一次性设置四条腿的触地权重 (方便多步态切换)
 * @param  w0, w1, w2, w3 分别对应腿 0, 1, 2, 3 的权重
 */
void Chassis_Set_All_Leg_Weights(float w0, float w1, float w2, float w3);

/**
 * @brief  底盘姿态补偿主计算循环
 * @param  dt 两次调用之间的时间间隔(秒)，用于PID积分微分计算
 */
void Chassis_Attitude_Loop(float dt);

/**
 * @brief  获取指定腿的高度(Y轴)补偿量
 * @param  leg_index 腿编号
 * @return 补偿的高度差 Δy
 */
float Chassis_Get_Leg_Offset(uint8_t leg_index);

#endif
