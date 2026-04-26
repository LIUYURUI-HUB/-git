#ifndef __LEG_KINEMATICS_H__
#define __LEG_KINEMATICS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>
#include "leg_kinematics.h"
#include "motor_controller.h"
#include "gom_protocol.h"


/* --- 数学常量与宏 --- */
#ifndef PI
#define PI 3.1415926535f
#endif

#define ANGLE_TO_RAD(x) ((x) * PI / 180.0f)
#define RAD_TO_ANGLE(x)   ((x) * 180.0f / PI)

/* --- 结构体定义 --- */

/**
 * @brief 足端空间坐标 (直角坐标系)
 */
typedef struct {
    float x; // 水平方向位移 (m)，正向为前
    float z; // 垂直方向位移 (m)，正向为下
} FootPos_T;

/**
 * @brief 电机输出状态 (弧度与角速度)
 */
// 腿部状态数据 (包含反馈和目标)
typedef struct {
    // --- 反馈状态 (由 FK 使用) ---
    float cur_hip_q1;   // 当前髋关节弧度
    float cur_knee_q2;  // 当前膝关节弧度

    // --- 目标指令 (由 IK 计算) ---
    float target_hip_q1; // 目标髋关节弧度
    float target_knee_q2;// 目标膝关节弧度

    // --- 轮足电机 ---
    float wheel_w;      // 轮子目标角速度 (rad/s)
} LegMotors_T;

/**
 * @brief 腿部机械参数配置
 */
typedef struct {
    float L1;           // 大腿连杆长度 (m)
    float L2;           // 小腿连杆长度 (m)
    float offset_hip;   // 髋关节电机安装零位偏移 (deg)
    float offset_knee;  // 膝关节电机安装零位偏移 (deg)
    float wheel_r;      // 足端轮子半径 (m)

//    // --- 硬件映射 ---
//    uint8_t motor_id_hip;   // 髋关节对应电机ID
//    uint8_t motor_id_knee;  // 膝关节对应电机ID
//    uint8_t motor_id_wheel; // 轮端电机ID (若为轮足)

} LegConfig_T;

/**
 * @brief 摆线步态参数配置
 */
typedef struct {
    float T;            // 步态总周期 (s)
    float S;            // 步长 (m)，正数前进，负数后退
    float H;            // 抬腿高度 (m)
    float z_base;       // 站立基准高度 (m)
    float stance_ratio; // 支撑相占比 (0.0~1.0)，通常取 0.5
} Cycloidal_T;

/* --- 函数原型声明 --- */

/**
 * @brief 正运动学 (FK): 电机角度 -> 足端坐标
 */
void leg_fk(LegConfig_T *config, FootPos_T *pos ,LegMotors_T *motors, MotorData_t *data1, MotorData_t *data2);

/**
 * @brief 逆运动学 (IK): 足端坐标 -> 电机角度
 * @return 0: 正常, -1: 坐标超出物理范围
 */
int8_t leg_ik(LegConfig_T *config, FootPos_T *pos, LegMotors_T *motors);

/**
 * @brief 摆线步态轨迹点生成
 */
FootPos_T cycloidal_gait(Cycloidal_T *gait, float walk_t, uint8_t start_action);

/**
 * @brief 计算单腿的轮子目标角速度
 */
float calculate_wheel_w(float walk_t, LegConfig_T *config, Cycloidal_T *gait, uint8_t start_action) ;

/**
 * @brief 腿部运动控制核心入口
 * @param config  机械参数
 * @param gait    步态参数
 * @param time    系统运行时间 (s)
 * @param out_motors 输出的目标电机状态
 * @return 0: 正常, -1: 计算异常
 */
int8_t Leg_Control(float time,LegMotors_T *fl, LegMotors_T *fr, LegMotors_T *rl, LegMotors_T *rr,LegConfig_T *config, Cycloidal_T *gait);

#ifdef __cplusplus
}
#endif

#endif /* __LEG_KINEMATICS_H__ */
