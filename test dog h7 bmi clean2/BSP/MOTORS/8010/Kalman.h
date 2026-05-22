/*
 * Kalman.h
 *
 *  Created on: Oct 7, 2025
 *      Author: ASUS
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

// 前向声明结构体类型
typedef struct _Kalman_filter Kalman_filter;

// 卡尔曼滤波结构体
struct _Kalman_filter
{
        float LastP; // 前序协方差
        float NowP;         // 当前协方差
        float out;         // 滤波结果
        float Kg;                 // 卡尔曼增益
        float Q;                 // 背景白噪音
        float R;                 // 器件方差
        void (*filt)(Kalman_filter *ekf, float input);
};


// 一维卡尔曼
void kalman(Kalman_filter *ekf, float input);

//类卡尔曼
double ema_filter(double new_value, double old_value, float alpha);

#endif /* INC_KALMAN_H_ */
