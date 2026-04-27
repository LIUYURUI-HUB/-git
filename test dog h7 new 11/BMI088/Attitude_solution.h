#ifndef __ATTITUDE_SOLUTION_H
#define __ATTITUDE_SOLUTION_H

#include "stdint.h"
#include "BMI088driver.h"
#include "kalman.h"  /* 新增：引入刚刚添加的卡尔曼核心库 */

/* ========== 数学常量 ========== */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define RAD_TO_DEG    57.2957795f
#define DEG_TO_RAD    0.017453292f
#define GRAVITY       9.80665f

/* ========== 滤波边界参数 ========== */
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

    /* --- 静态零偏校准值 --- */
    float Offset_Accel_X;
    float Offset_Accel_Y;
    float Offset_Accel_Z;
    float Offset_Gyro_X;
    float Offset_Gyro_Y;
    float Offset_Gyro_Z;

    /* --- 解算欧拉角 (对外输出结果) --- */
    float Angle_Pitch;    /* 俯仰角 (绕 Y 轴，-90~90°) */
    float Angle_Roll;     /* 横滚角 (绕 X 轴，-180~180°) */
    float Angle_Yaw;      /* 偏航角 (绕 Z 轴，-180~180°) */

    float Yaw_Offset;     /* 用于 Yaw 重置清零的偏移量 */

} Attitude_Data_t;

/* ========== 函数声明 ========== */
void Attitude_Init(void);
void Attitude_Calibrate_Static_Gyro(void);
void Attitude_Update(float dt);
void Attitude_ResetYaw(void);

/* ========== 外部访问接口 ========== */
extern Attitude_Data_t g_Attitude;

#endif /* __ATTITUDE_SOLUTION_H */
