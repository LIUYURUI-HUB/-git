#include "Attitude_solution.h"
#include "main.h"
#include <math.h>

/* ========== 全局变量 ========== */
Attitude_Data_t g_Attitude = {0};

/* ========== 实例化卡尔曼滤波器 ========== */
static Kalman_t kalman_pitch;
static Kalman_t kalman_roll;

/* ========== 私有函数声明 ========== */
static void Attitude_Align_Gravity(void);

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

        /* 角度与偏移清零 */
        g_Attitude.Angle_Pitch = 0.0f;
        g_Attitude.Angle_Roll  = 0.0f;
        g_Attitude.Angle_Yaw   = 0.0f;
        g_Attitude.Yaw_Offset  = 0.0f;

        /* 默认无加速度计零偏  */
        g_Attitude.Offset_Accel_X = 0.0f;
        g_Attitude.Offset_Accel_Y = 0.0f;
        g_Attitude.Offset_Accel_Z = 0.0f;

        /* ====== 核心部署：初始化卡尔曼滤波器 ====== */
        // 参数依次为：滤波器实例, Q_angle(角度信任度), Q_bias(零偏信任度), R_measure(加速度计噪声)
        Kalman_Init(&kalman_pitch, 0.001f, 0.003f, 0.03f);
        Kalman_Init(&kalman_roll,  0.001f, 0.003f, 0.03f);

        /* ====== 上电重力对齐 ====== */
        // 利用加速度计强制初始化卡尔曼滤波器的起点
        Attitude_Align_Gravity();

    } else {
        g_Attitude.Init_OK = 0;
    }
}

/**
 * @brief 利用重力向量初始化卡尔曼滤波起点 (取代四元数初始对齐)
 */
static void Attitude_Align_Gravity(void)
{
    float gyro[3], accel[3], temp;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    const int sample_count = 50;

    /* 1. 采集50次加速度计数据求平均，滤除高频噪声 */
    for (int i = 0; i < sample_count; i++) {
        BMI088_read(gyro, accel, &temp);
        ax_sum += accel[0];
        ay_sum += accel[1];
        az_sum += accel[2];
        HAL_Delay(2); // 采样间隔
    }

    float ax = ax_sum / sample_count;
    float ay = ay_sum / sample_count;
    float az = az_sum / sample_count;

    /* 2. 直接计算初始欧拉角 (角度制) */
    float initial_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float initial_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    /* 3. 强行将算出的静止角度，赋给卡尔曼滤波器的初始状态 */
    kalman_roll.angle  = initial_roll;
    kalman_pitch.angle = initial_pitch;

    g_Attitude.Angle_Roll  = initial_roll;
    g_Attitude.Angle_Pitch = initial_pitch;
    g_Attitude.Angle_Yaw   = 0.0f; // 上电默认机头正前方为0度
}

/**
 * @brief 动态校准陀螺仪零偏 (允许倾斜上电，只要保持静止即可)
 */
void Attitude_Calibrate_Static_Gyro(void)
{
    float gyro[3], accel[3], temp;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const int sample_count = 200;  /* 200 次采样 */

    if (!g_Attitude.Init_OK) return;

    for (int i = 0; i < sample_count; i++) {
        BMI088_read(gyro, accel, &temp);
        gx_sum += gyro[0];
        gy_sum += gyro[1];
        gz_sum += gyro[2];
        HAL_Delay(10);
    }

    g_Attitude.Offset_Gyro_X = gx_sum / sample_count;
    g_Attitude.Offset_Gyro_Y = gy_sum / sample_count;
    g_Attitude.Offset_Gyro_Z = gz_sum / sample_count;

    g_Attitude.Calibrated = 1;
}

/**
 * @brief 重置 Yaw 角
 */
void Attitude_ResetYaw(void)
{
    g_Attitude.Yaw_Offset = g_Attitude.Angle_Yaw + g_Attitude.Yaw_Offset;
    g_Attitude.Angle_Yaw = 0.0f;
}

/**
 * @brief 姿态解算更新函数 (每循环调用一次)
 */
void Attitude_Update(float dt)
{
    float gyro[3], accel[3], temp;

    if (!g_Attitude.Init_OK) return;

    /* 1. 读取 BMI088 原始数据 */
    BMI088_read(gyro, accel, &temp);

    /* 2. 扣除陀螺仪零偏 */
    float ax = accel[0] - g_Attitude.Offset_Accel_X;
    float ay = accel[1] - g_Attitude.Offset_Accel_Y;
    float az = accel[2] - g_Attitude.Offset_Accel_Z;

    float gx = gyro[0] - g_Attitude.Offset_Gyro_X;
    float gy = gyro[1] - g_Attitude.Offset_Gyro_Y;
    float gz = gyro[2] - g_Attitude.Offset_Gyro_Z;

    // 记录对外数据
    g_Attitude.Accel_X = ax; g_Attitude.Accel_Y = ay; g_Attitude.Accel_Z = az;
    g_Attitude.Gyro_X = gx;  g_Attitude.Gyro_Y = gy;  g_Attitude.Gyro_Z = gz;
    g_Attitude.Temperature = temp;

    /* 3. 使用加速度计计算观测角度 Z (包含大量震动噪声) */
    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    /* 4. 【卡尔曼滤波接管】输入带噪声的角度Z 和 陀螺仪角速度U */
    // 注意：旋转轴对应关系：X轴角速度(gx)对应横滚(Roll)，Y轴角速度(gy)对应俯仰(Pitch)
    g_Attitude.Angle_Roll  = Kalman_GetAngle(&kalman_roll,  acc_roll,  gx, dt);
    g_Attitude.Angle_Pitch = Kalman_GetAngle(&kalman_pitch, acc_pitch, gy, dt);

    /* 5. 偏航角 Yaw 处理 (依然使用死区限制纯积分) */
    if (fabsf(gz) > YAW_DEADZONE_THRESHOLD) {
        g_Attitude.Angle_Yaw += gz * dt;
    }

    float adjusted_yaw = g_Attitude.Angle_Yaw - g_Attitude.Yaw_Offset;
    while (adjusted_yaw > 180.0f)  adjusted_yaw -= 360.0f;
    while (adjusted_yaw < -180.0f) adjusted_yaw += 360.0f;
    g_Attitude.Angle_Yaw = adjusted_yaw;
}
