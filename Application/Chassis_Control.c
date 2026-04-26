/*
 * Chassis_Control.c
 *
 *  Created on: Mar 27, 2026
 *      Author: 22569
 */
#include "Chassis_Control.h"
#include "arm_task.h"
#include "../Device/BMI088/Attitude_solution.h" // 引入IMU姿态解算数据

/* 存储四条腿实时高度补偿的数组 */
static float leg_y_offsets[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* PID 控制器实例：参数需根据你的机器狗重量和腿长比例微调 */
static PID_Controller_t pid_roll  = { .kp = 0.8f, .ki = 0.02f, .kd = 0.1f, .integral_limit = 15.0f };
static PID_Controller_t pid_pitch = { .kp = 0.8f, .ki = 0.02f, .kd = 0.1f, .integral_limit = 15.0f };

/* ========== 基础 PID 计算 ========== */
static float PID_Calculate(PID_Controller_t *pid, float target, float current, float dt)//目标位置，当前位置，采样周期
{
    //误差与积分累加
    pid->error = target - current;
    pid->integral += pid->error * dt;

    // 积分限幅防饱和
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = (pid->error - pid->last_error) / dt;
    pid->last_error = pid->error;

    pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    return pid->output;
}

/* ========== 接口实现 ========== */

void Chassis_Control_Init(void)
{
    for(int i = 0; i < 4; i++) {
        leg_y_offsets[i] = 0.0f;
    }
}



void Chassis_Attitude_Loop(float dt)
{
    // 目标姿态：保持水平(0度)
    float target_roll = 0.0f;
    float target_pitch = 0.0f;

    // 1. 获取姿态误差的补偿量 (依赖 Attitude_solution.h 中的 g_Attitude)
    // 补偿量对应腿的延伸/收缩长度
    float roll_comp = PID_Calculate(&pid_roll, target_roll, g_Attitude.Angle_Roll, dt);
    float pitch_comp = PID_Calculate(&pid_pitch, target_pitch, g_Attitude.Angle_Pitch, dt);

        // 限位防止输出超出硬件能承受的范围
    if(pid_roll.output > 15.0f) pid_roll.output = 15.0f;
    if(pid_pitch.output > 15.0f) pid_pitch.output = 15.0f;
    /* * 2. 动力分配到四条腿：
     * 逆向运动学中 Y通常为正值(向下延伸)。
     * Pitch>0 (抬头) -> 前腿需缩短(Y减小)，后腿需伸长(Y增大)。
     * Roll>0 (右倾，即左高右低) -> 左腿缩短(Y减小)，右腿伸长(Y增大)。
     * * 假设腿编号: 0-前左(FL), 1-前右(FR), 2-后左(HL), 3-后右(HR)
     */
    leg_y_offsets[0] =  pitch_comp + roll_comp;  // 前左: 叠加pitch与roll的修正
    leg_y_offsets[2] =  pitch_comp - roll_comp;  // 前右: pitch修正，扣除roll修正
    leg_y_offsets[3] = -pitch_comp + roll_comp;  // 后左: 扣除pitch修正，叠加roll修正
    leg_y_offsets[1] = -pitch_comp - roll_comp;  // 后右: 扣除两种修正
}

float Chassis_Get_Leg_Compensation(uint8_t leg_index)
{
    if(leg_index < 4) {
        return leg_y_offsets[leg_index];
    }
    return 0.0f;
}

