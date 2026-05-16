#include "Attitude_solution.h"
#include "main.h"
#include <math.h>

Attitude_Data_t g_Attitude = {0};

static float Low_Pass_Filter(float new_val, float prev_val, float alpha)
{
    return alpha * prev_val + (1.0f - alpha) * new_val;
}

static float Get_Dynamic_Alpha(float gx, float gy, float gz)
{
    float gyro_magnitude = sqrtf(gx * gx + gy * gy + gz * gz);

    if (gyro_magnitude <= GYRO_STATIC_THRESHOLD) {
        return ALPHA_GYRO_STATIC + (ALPHA_GYRO_MOVING - ALPHA_GYRO_STATIC) *
               (gyro_magnitude / GYRO_STATIC_THRESHOLD);
    }

    return ALPHA_GYRO_MOVING;
}

void Attitude_Init(void)
{
    uint8_t status = BMI088_init();

    if (status == BMI088_NO_ERROR) {
        g_Attitude.Init_OK = 1;
        g_Attitude.Calibrated = 0;

        g_Attitude.Angle_Pitch = 0.0f;
        g_Attitude.Angle_Roll = 0.0f;
        g_Attitude.Angle_Yaw = 0.0f;

        g_Attitude.flt_ax = 0.0f;
        g_Attitude.flt_ay = 0.0f;
        g_Attitude.flt_az = 0.0f;
        g_Attitude.flt_gx = 0.0f;
        g_Attitude.flt_gy = 0.0f;
        g_Attitude.flt_gz = 0.0f;
    } else {
        g_Attitude.Init_OK = 0;
    }
}

void Attitude_Calibrate_ZeroBias(void)
{
    float ax_sum = 0.0f;
    float ay_sum = 0.0f;
    float az_sum = 0.0f;
    float gx_sum = 0.0f;
    float gy_sum = 0.0f;
    float gz_sum = 0.0f;
    float gyro[3];
    float accel[3];
    float temp;
    const int sample_count = 2000;

    if (!g_Attitude.Init_OK) {
        return;
    }

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

    g_Attitude.Offset_Accel_X = ax_sum / sample_count;
    g_Attitude.Offset_Accel_Y = ay_sum / sample_count;
    g_Attitude.Offset_Accel_Z = (az_sum / sample_count) - GRAVITY;

    g_Attitude.Offset_Gyro_X = gx_sum / sample_count;
    g_Attitude.Offset_Gyro_Y = gy_sum / sample_count;
    g_Attitude.Offset_Gyro_Z = gz_sum / sample_count;

    g_Attitude.Calibrated = 1;
}

void Attitude_ResetYaw(void)
{
    g_Attitude.Angle_Yaw = 0.0f;
}

void Attitude_Update(float dt)
{
    float gyro[3];
    float accel[3];
    float temp;

    if (!g_Attitude.Init_OK) {
        return;
    }

    BMI088_read(gyro, accel, &temp);

    g_Attitude.Gyro_X = gyro[0];
    g_Attitude.Gyro_Y = gyro[1];
    g_Attitude.Gyro_Z = gyro[2];
    g_Attitude.Accel_X = accel[0];
    g_Attitude.Accel_Y = accel[1];
    g_Attitude.Accel_Z = accel[2];
    g_Attitude.Temperature = temp;

    float ax = g_Attitude.Accel_X - g_Attitude.Offset_Accel_X;
    float ay = g_Attitude.Accel_Y - g_Attitude.Offset_Accel_Y;
    float az = g_Attitude.Accel_Z - g_Attitude.Offset_Accel_Z;

    float gx = g_Attitude.Gyro_X - g_Attitude.Offset_Gyro_X;
    float gy = g_Attitude.Gyro_Y - g_Attitude.Offset_Gyro_Y;
    float gz = g_Attitude.Gyro_Z - g_Attitude.Offset_Gyro_Z;

    g_Attitude.flt_gx = Low_Pass_Filter(gx, g_Attitude.flt_gx, GYRO_LPF_ALPHA);
    g_Attitude.flt_gy = Low_Pass_Filter(gy, g_Attitude.flt_gy, GYRO_LPF_ALPHA);
    g_Attitude.flt_gz = Low_Pass_Filter(gz, g_Attitude.flt_gz, GYRO_LPF_ALPHA);

    g_Attitude.flt_ax = Low_Pass_Filter(ax, g_Attitude.flt_ax, ACCEL_LPF_ALPHA);
    g_Attitude.flt_ay = Low_Pass_Filter(ay, g_Attitude.flt_ay, ACCEL_LPF_ALPHA);
    g_Attitude.flt_az = Low_Pass_Filter(az, g_Attitude.flt_az, ACCEL_LPF_ALPHA);

    float acc_pitch = atan2f(g_Attitude.flt_ay,
                      sqrtf(g_Attitude.flt_ax * g_Attitude.flt_ax +
                            g_Attitude.flt_az * g_Attitude.flt_az)) * RAD_TO_DEG;

    float acc_roll = atan2f(-g_Attitude.flt_ax,
                     sqrtf(g_Attitude.flt_ay * g_Attitude.flt_ay +
                           g_Attitude.flt_az * g_Attitude.flt_az)) * RAD_TO_DEG;

    float flt_gz_deadzone = g_Attitude.flt_gz;
    if (fabsf(flt_gz_deadzone) < YAW_DEADZONE_THRESHOLD) {
        flt_gz_deadzone = 0.0f;
    }

    g_Attitude.Angle_Pitch += g_Attitude.flt_gx * dt;
    g_Attitude.Angle_Roll += g_Attitude.flt_gy * dt;
    g_Attitude.Angle_Yaw += flt_gz_deadzone * dt;

    float alpha = Get_Dynamic_Alpha(g_Attitude.flt_gx,
                                    g_Attitude.flt_gy,
                                    g_Attitude.flt_gz);

    g_Attitude.Angle_Pitch = alpha * g_Attitude.Angle_Pitch + (1.0f - alpha) * acc_pitch;
    g_Attitude.Angle_Roll = alpha * g_Attitude.Angle_Roll + (1.0f - alpha) * acc_roll;
}
