#include "Attitude_solution.h"
#include "main.h"
#include <math.h>

/* ========== 全局变量 ========== */
Attitude_Data_t g_Attitude = {0};

/* ========== 静态辅助函数 ========== */

/**
 * @brief 一阶低通滤波
 * @param new_val: 新采样值
 * @param prev_val: 上一次滤波值
 * @param alpha: 滤波系数 (0~1，越大越平滑但延迟越高)
 */
static float Low_Pass_Filter(float new_val, float prev_val, float alpha)
{
    return alpha * prev_val + (1.0f - alpha) * new_val;
}

/**
 * @brief 计算动态互补滤波系数
 * @param gx, gy, gz: 陀螺仪三轴角速度
 * @return 互补滤波系数 alpha (0.90~0.98)
 */
static float Get_Dynamic_Alpha(float gx, float gy, float gz)
{
    float gyro_magnitude = sqrtf(gx*gx + gy*gy + gz*gz);

    if (gyro_magnitude <= GYRO_STATIC_THRESHOLD) {
        /* 静止状态：线性过渡到 STATIC 系数 */
        return ALPHA_GYRO_STATIC + (ALPHA_GYRO_MOVING - ALPHA_GYRO_STATIC)
               * (gyro_magnitude / GYRO_STATIC_THRESHOLD);
    } else {
        /* 运动状态：高度信任陀螺仪 */
        return ALPHA_GYRO_MOVING;
    }
}

/* ========== 公共函数实现 ========== */

/**
 * @brief 姿态解算模块初始化
 */
void Attitude_Init(void)
{
    uint8_t status = BMI088_init();

    if (status == BMI088_NO_ERROR) {
        g_Attitude.Init_OK = 1;
        g_Attitude.Calibrated = 0;

        /* 角度清零 */
        g_Attitude.Angle_Pitch = 0.0f;
        g_Attitude.Angle_Roll  = 0.0f;
        g_Attitude.Angle_Yaw   = 0.0f;

        /* 滤波变量清零 */
        g_Attitude.flt_ax = 0; g_Attitude.flt_ay = 0; g_Attitude.flt_az = 0;
        g_Attitude.flt_gx = 0; g_Attitude.flt_gy = 0; g_Attitude.flt_gz = 0;
    } else {
        g_Attitude.Init_OK = 0;
    }
}

/**
 * @brief 零偏校准 (保持传感器静止 2 秒)
 * @note 校准时请将板子水平放置
 */
void Attitude_Calibrate_ZeroBias(void)
{
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float gyro[3], accel[3], temp;
    const int sample_count = 2000;  /* 200 次 * 10ms = 2 秒 */

    if (!g_Attitude.Init_OK) {
        return;
    }

    /* 采样 200 次 */
    for (int i = 0; i < sample_count; i++) {
        BMI088_read(gyro, accel, &temp);

        ax_sum += accel[0];
        ay_sum += accel[1];
        az_sum += accel[2];
        gx_sum += gyro[0];
        gy_sum += gyro[1];
        gz_sum += gyro[2];

        HAL_Delay(1);
    }

    /* 计算平均值作为零偏 */
    g_Attitude.Offset_Accel_X = ax_sum / sample_count;
    g_Attitude.Offset_Accel_Y = ay_sum / sample_count;
    /* Z 轴包含重力，水平放置时应为 GRAVITY */
    g_Attitude.Offset_Accel_Z = (az_sum / sample_count) - GRAVITY;

    g_Attitude.Offset_Gyro_X = gx_sum / sample_count;
    g_Attitude.Offset_Gyro_Y = gy_sum / sample_count;
    g_Attitude.Offset_Gyro_Z = gz_sum / sample_count;

    g_Attitude.Calibrated = 1;
}

/**
 * @brief 重置 Yaw 角 (用于重新定向)
 */
void Attitude_ResetYaw(void)
{
    g_Attitude.Angle_Yaw = 0.0f;
}

/**
 * @brief 姿态解算更新函数 (每循环调用一次)
 * @param dt: 距离上次更新的时间间隔 (秒)
 */
void Attitude_Update(float dt)
{
    float gyro[3], accel[3], temp;

    if (!g_Attitude.Init_OK) {
        return;
    }

    /* 1. 读取 BMI088 原始数据 */
    BMI088_read(gyro, accel, &temp);

    g_Attitude.Gyro_X = gyro[0];
    g_Attitude.Gyro_Y = gyro[1];
    g_Attitude.Gyro_Z = gyro[2];
    g_Attitude.Accel_X = accel[0];
    g_Attitude.Accel_Y = accel[1];
    g_Attitude.Accel_Z = accel[2];
    g_Attitude.Temperature = temp;

    /* 2. 扣除零偏 */
    float ax = g_Attitude.Accel_X - g_Attitude.Offset_Accel_X;
    float ay = g_Attitude.Accel_Y - g_Attitude.Offset_Accel_Y;
    float az = g_Attitude.Accel_Z - g_Attitude.Offset_Accel_Z;

    float gx = g_Attitude.Gyro_X - g_Attitude.Offset_Gyro_X;
    float gy = g_Attitude.Gyro_Y - g_Attitude.Offset_Gyro_Y;
    float gz = g_Attitude.Gyro_Z - g_Attitude.Offset_Gyro_Z;

    /* 3. 低通滤波 */
    g_Attitude.flt_gx = Low_Pass_Filter(gx, g_Attitude.flt_gx, GYRO_LPF_ALPHA);
    g_Attitude.flt_gy = Low_Pass_Filter(gy, g_Attitude.flt_gy, GYRO_LPF_ALPHA);
    g_Attitude.flt_gz = Low_Pass_Filter(gz, g_Attitude.flt_gz, GYRO_LPF_ALPHA);

    g_Attitude.flt_ax = Low_Pass_Filter(ax, g_Attitude.flt_ax, ACCEL_LPF_ALPHA);
    g_Attitude.flt_ay = Low_Pass_Filter(ay, g_Attitude.flt_ay, ACCEL_LPF_ALPHA);
    g_Attitude.flt_az = Low_Pass_Filter(az, g_Attitude.flt_az, ACCEL_LPF_ALPHA);

    /* 4. 加速度计角度 (静态参考) */
    float acc_pitch = atan2f(g_Attitude.flt_ay,
                      sqrtf(g_Attitude.flt_ax * g_Attitude.flt_ax +
                            g_Attitude.flt_az * g_Attitude.flt_az)) * RAD_TO_DEG;

    float acc_roll  = atan2f(-g_Attitude.flt_ax,
                      sqrtf(g_Attitude.flt_ay * g_Attitude.flt_ay +
                            g_Attitude.flt_az * g_Attitude.flt_az)) * RAD_TO_DEG;

    /* 5. Yaw 轴死区抑制 */
    float flt_gz_deadzone = g_Attitude.flt_gz;
    if (fabsf(flt_gz_deadzone) < YAW_DEADZONE_THRESHOLD) {
        flt_gz_deadzone = 0.0f;
    }

    /* 6. 陀螺仪积分 (动态角度) */
    g_Attitude.Angle_Pitch += g_Attitude.flt_gx * dt;
    g_Attitude.Angle_Roll  += g_Attitude.flt_gy * dt;
    g_Attitude.Angle_Yaw   += flt_gz_deadzone * dt;

    /* 7. 动态互补滤波融合 */
    float alpha = Get_Dynamic_Alpha(g_Attitude.flt_gx,
                                    g_Attitude.flt_gy,
                                    g_Attitude.flt_gz);

    g_Attitude.Angle_Pitch = alpha * g_Attitude.Angle_Pitch + (1.0f - alpha) * acc_pitch;
    g_Attitude.Angle_Roll  = alpha * g_Attitude.Angle_Roll  + (1.0f - alpha) * acc_roll;
    /* Yaw 角无法用加速度计修正，只靠陀螺仪积分 */
}
