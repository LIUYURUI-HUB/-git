/*
 * kalman.c
 *
 *  Created on: Apr 11, 2026
 *      Author: 22569
 */
#include "Kalman.h"

void Kalman_Init(Kalman_t *kf, float Q_angle, float Q_bias, float R_measure)
{
    /* 初始状态设定为绝对水平，无零偏 */
    kf->angle = 0.0f;
    kf->bias = 0.0f;

    /* 初始化协方差矩阵 P 为全 0 (也可以设为 1.0f，跑几次就会迅速收敛) */
    kf->P[0][0] = 0.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 0.0f;

    /* 设定噪声参数 */
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;
}

float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt)
{
    /* =======================================
     * 1. 预测步 (Predict)
     * ======================================= */
    // a. 预测当前状态: 角度 = 上次角度 + (角速度 - 零偏) * dt
    float rate = newRate - kf->bias;
    kf->angle += dt * rate;

    // b. 预测协方差 P: P = F*P*F^T + Q (矩阵乘法已完全代数展开，极大加速计算)
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    /* =======================================
     * 2. 更新步 (Update)
     * ======================================= */
    // c. 计算误差残差 y = Z - H*X (传感器测量值 - 模型推算值)
    float y = newAngle - kf->angle;

    // d. 计算卡尔曼增益 K = P*H^T / (H*P*H^T + R)
    float S = kf->P[0][0] + kf->R_measure; // S矩阵其实是一个标量
    float K[2]; // 增益矩阵 2x1
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // e. 修正状态估计最优值 X = X + K*y
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y; // 后台自动追踪并消除陀螺仪零偏

    // f. 更新协方差矩阵 P = (I - K*H)*P
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

