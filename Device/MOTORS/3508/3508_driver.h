#ifndef __3508_DRIVER_H__
#define __3508_DRIVER_H__


#include "main.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define D3508_REDUCTION_RATIO    19.0f
#define D3508_ENCODER_RES        8192.0f
#define D3508_RAD_PER_TICK       (2.0f * M_PI / (D3508_ENCODER_RES * D3508_REDUCTION_RATIO))
#define D3508_RPM_TO_WHEEL_RADPS (2.0f * M_PI / (60.0f * D3508_REDUCTION_RATIO))

// 1. 电机数据结构体
typedef struct {
    // 反馈数据
    uint16_t cur_angle;      // 0-8191 当前机械角度
    int16_t  cur_speed;      // 转速 RPM
    float    filter_speed; // 过滤速度
    int16_t  actual_current; // 实际电流
    uint8_t  temperature;    // 温度

    // 角度

    uint16_t last_angle;     // 上一次的角度
    int64_t  total_angle;    // 累计总角度
    int32_t  total_round; //圈数计数

    // 控制数据
    int64_t  target_angle;  //目标角度
    float    target_speed;   // 目标速度
    int16_t  Out_Current;    // 输出电流
    float    speed_err_sum;        // PID积分
    float    position_err_sum;
    float    torque_err_sum;
    float    err_last;        // PID积分

    uint8_t init_flag;    // 解决 Canvas 中 D3508_Decode 的首次上电保护报错
} Motor_3508_T;

// 2. PID 参数结构体
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Max_Out;
    float Max_Sum;
} PID_3508_T;

// 3. 全局变量
extern Motor_3508_T Motors[4];
extern PID_3508_T Speed_PID;
extern PID_3508_T Position_PID;

// 4. 函数声明
//过滤器配置函数
void FDCAN1_Filter_Init(void);
void FDCAN3_Filter_Init(void);

//初始化电机数据
void D3508_Init(void);

//控制电流发送
void send_current(void);

//PID计算
void Clear_Motor_PID(int motor_id);
void PID_Calc_Speed(int i);
void PID_Calc_Position(int i, float target_angle);
void PID_Calc_Torque(int i, float target_torque);
void MIT_Wheel_Control(int i, int64_t P_des, float V_des, float kp, float kd, float t_ff_wheel, float max_wheel_torque);

//拆解接收数据
void D3508_Decode(uint8_t* RxData, uint16_t ID);

//功率限制函数
int16_t Power_Limit(float desire_current, float rpm);

#endif
