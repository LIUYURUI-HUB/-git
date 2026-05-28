/*
 * kalman.h
 *
 *  Created on: Apr 11, 2026
 *      Author: 22569
 */

#ifndef KALMAN_H_
#define KALMAN_H_

/* ========== 卡尔曼滤波器状态结构体 ========== */
typedef struct {
    float angle;      // [状态 1] 滤波后得出的最优角度
    float bias;       // [状态 2] 计算出的陀螺仪零偏 (漂移)

    float P[2][2];    // 误差协方差矩阵 (2x2)

    // 调参变量
    float Q_angle;    // 角度过程噪声 (相信陀螺仪的程度)
    float Q_bias;     // 零偏过程噪声 (相信零偏不变的程度)
    float R_measure;  // 测量噪声 (相信加速度计的程度)
} Kalman_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化卡尔曼滤波器参数
 * @param  kf: 卡尔曼滤波器实例指针
 * @param  Q_angle: 角度过程噪声方差
 * @param  Q_bias: 零偏过程噪声方差
 * @param  R_measure: 测量噪声方差
 */
void Kalman_Init(Kalman_t *kf, float Q_angle, float Q_bias, float R_measure);

/**
 * @brief  核心运算：卡尔曼状态更新
 * @param  kf: 卡尔曼滤波器实例指针
 * @param  newAngle: 加速度计计算出的角度 (观测值 Z)
 * @param  newRate: 陀螺仪的角速度 (控制量 U)
 * @param  dt: 两次调用的时间间隔(秒)
 * @retval 滤波后的最优真实角度
 */
float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt);

#endif /* KALMAN_H_ */
