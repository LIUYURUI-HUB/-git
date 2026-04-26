/*
 * Chassis_Control.h
 *
 *  Created on: Mar 27, 2026
 *      Author: 22569
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
 * @brief  底盘姿态补偿主计算循环
 * @param  dt 两次调用之间的时间间隔(秒)，用于PID积分微分计算
 * @note   建议放在与 IMU 解算相同的定时器或主循环中调用 (如 200Hz ~ 500Hz)
 */
void Chassis_Attitude_Loop(float dt);

/**
 * @brief  获取指定腿的高度(Y轴)补偿量
 * @param  leg_index 腿编号 (0:前左, 2:前右, 3:后左, 1:后右)
 * @return 补偿的高度差 Δy
 */
float Chassis_Get_Leg_Compensation(uint8_t leg_index);

#endif /* __CHASSIS_CONTROL_H */ /* CHASSIS_CONTROL_H_ */
