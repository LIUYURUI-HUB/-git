#ifndef __ATTITUDE_SOLUTION_H
#define __ATTITUDE_SOLUTION_H

#include "stdint.h"
#include "BMI088driver.h"

/* ========== 数学常量 ========== */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define RAD_TO_DEG    57.2957795f
#define DEG_TO_RAD    0.017453292f
#define GRAVITY       9.80665f

/* ========== 滤波参数 (可调整) ========== */
#define GYRO_STATIC_THRESHOLD     2.0f       /* 陀螺仪静止阈值 (deg/s) */
#define ALPHA_GYRO_MOVING         0.98f      /* 运动时互补滤波系数 */
#define ALPHA_GYRO_STATIC         0.90f      /* 静止时互补滤波系数 */
#define GYRO_LPF_ALPHA            0.6f       /* 陀螺仪低通滤波系数 */
#define ACCEL_LPF_ALPHA           0.2f       /* 加速度计低通滤波系数 */
#define YAW_DEADZONE_THRESHOLD    0.15f      /* Yaw 轴死区阈值 (deg/s) */

/* ========== 姿态数据结构 ========== */
typedef struct {
    /* --- 状态标志 --- */
    uint8_t Init_OK;
    uint8_t Calibrated;

    /* --- 原始数据 (BMI088 直接输出) --- */
    float Accel_X;        /* 加速度 X (m/s²) */
    float Accel_Y;        /* 加速度 Y (m/s²) */
    float Accel_Z;        /* 加速度 Z (m/s²) */
    float Gyro_X;         /* 角速度 X (deg/s) */
    float Gyro_Y;         /* 角速度 Y (deg/s) */
    float Gyro_Z;         /* 角速度 Z (deg/s) */
    float Temperature;    /* 温度 (°C) */

    /* --- 零偏校准值 --- */
    float Offset_Accel_X;
    float Offset_Accel_Y;
    float Offset_Accel_Z;
    float Offset_Gyro_X;
    float Offset_Gyro_Y;
    float Offset_Gyro_Z;

    /* --- 解算角度 --- */
    float Angle_Pitch;    /* 俯仰角 (绕 Y 轴，-180~180°) */
    float Angle_Roll;     /* 横滚角 (绕 X 轴，-180~180°) */
    float Angle_Yaw;      /* 偏航角 (绕 Z 轴，累积积分) */

    /* --- 滤波中间变量 --- */
    float flt_ax, flt_ay, flt_az;
    float flt_gx, flt_gy, flt_gz;

} Attitude_Data_t;

/* ========== 函数声明 ========== */
void Attitude_Init(void);
void Attitude_Calibrate_ZeroBias(void);
void Attitude_Update(float dt);
void Attitude_ResetYaw(void);

/* ========== 外部访问接口 ========== */
extern Attitude_Data_t g_Attitude;

#endif /* __ATTITUDE_SOLUTION_H */
