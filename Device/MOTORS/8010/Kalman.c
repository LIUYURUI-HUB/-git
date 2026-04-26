/*
 * Kalman.c
 *
 *  Created on: Oct 7, 2025
 *      Author: ASUS
 */

#include <Kalman.h>

void kalman(Kalman_filter *ekf, float input)
{
        //预测位置 = 上一时刻滤波位置 + 速度 × 时间间隔
        ekf->NowP = ekf->LastP + ekf->Q;

        //如果 NowP较小，说明预测比较可靠，Kg 会较小，更倾向于相信预测值；
        //如果 R 较小，说明测量比较可靠，Kg会较大，更倾向于相信测量值。
        ekf->Kg = ekf->NowP / (ekf->NowP + ekf->R);

        //这里 ekf->out 是上一时刻滤波后的位置（7.2 米），input 是传感器测量的当前位置（9.5 米）。
        //经过计算得到的新的 ekf->out 就是当前时刻滤波后的更准确的位置。
        ekf->out = ekf->out + ekf->Kg * (input - ekf->out);

        //更新数据
        ekf->LastP = (1 - ekf->Kg) * ekf->NowP;
}


double ema_filter(double new_value, double old_value, float alpha) {
    return alpha * new_value + (1 - alpha) * old_value;
}
