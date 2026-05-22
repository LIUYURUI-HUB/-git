#ifndef __ATTITUDE_SOLUTION_H
#define __ATTITUDE_SOLUTION_H

#include "stdint.h"
#include "BMI088driver.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define RAD_TO_DEG    57.2957795f
#define DEG_TO_RAD    0.017453292f
#define GRAVITY       9.80665f

#define GYRO_STATIC_THRESHOLD     2.0f
#define ALPHA_GYRO_MOVING         0.98f
#define ALPHA_GYRO_STATIC         0.90f
#define GYRO_LPF_ALPHA            0.6f
#define ACCEL_LPF_ALPHA           0.7f
#define YAW_DEADZONE_THRESHOLD    0.15f

typedef struct {
    uint8_t Init_OK;
    uint8_t Calibrated;

    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    float Temperature;

    float Offset_Accel_X;
    float Offset_Accel_Y;
    float Offset_Accel_Z;
    float Offset_Gyro_X;
    float Offset_Gyro_Y;
    float Offset_Gyro_Z;

    float Angle_Pitch;
    float Angle_Roll;
    float Angle_Yaw;

    float flt_ax;
    float flt_ay;
    float flt_az;
    float flt_gx;
    float flt_gy;
    float flt_gz;
} Attitude_Data_t;

void Attitude_Init(void);
void Attitude_Calibrate_ZeroBias(void);
void Attitude_Update(float dt);
void Attitude_ResetYaw(void);

extern Attitude_Data_t g_Attitude;

#endif
