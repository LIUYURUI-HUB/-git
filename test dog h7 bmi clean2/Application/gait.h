///**
// * @file gait.h
// * @brief 四足机器人步态控制核心头文件（融入双重落地缓冲保护机制版）
// */
//
//#ifndef GAIT_H_
//#define GAIT_H_
//
//#include <stdint.h>
//#include <math.h>
//#include "../../MIddleware/kinematics.h"
//
//#define PI 3.14159265358979323846f
//
//typedef struct {
//    float x;
//    float y;
//} Cycloid2D_Pose;
//
//typedef enum {
//    GAIT_DIRECTION_FORWARD = 0,
//    GAIT_DIRECTION_BACKWARD = 1
//} GaitDirection;
//
//typedef enum {
//    GAIT_PHASE_CYCLOID_FORWARD = 0,   // 摆动相 (空中)
//    GAIT_PHASE_STRAIGHT_BACKWARD = 1  // 支撑相 (触地)
//} GaitPhase;
//
///**
// * @struct LegGaitState
// * @brief 单腿状态机与物理参数
// */
//typedef struct {
//    MotorController ctrl;
//
//    // --- 物理校准参考点 ---
//    float x_base;
//    float y_base;
//
//    // --- 实时运行参数 ---
//    float phase_offset;      // 相位偏移 [0.0, 1.0)
//
//    // 状态记录 (用于缓冲反馈)
//    uint8_t is_swing_phase;  // 记录当前是否处于摆动相 1:空中 0:触地
//    float current_phase_dt;  // 记录在当前相位(摆动/支撑)中的进度 [0.0, 1.0)
//
//    float x_start;
//    float x_target;
//    float y_start;
//    float y_target;
//    float lift_height;
//    float cycle_time;
//    float start_time;
//    int direction;
//    GaitPhase phase;
//    uint8_t is_active;
//    LegAngles current_angles;
//} LegGaitState;
//
///**
// * @struct QuadrupedGait
// * @brief 整体步态控制器
// */
//typedef struct {
//    LegGaitState legs[4];
//    float gait_cycle_time;
//    float stride_length;
//    float lift_height;
//    float start_time;
//    float duty_factor;       // 支撑相占空比
//
//    // --- 落地缓冲(虚拟悬挂)参数 ---
//    float cushion_depth;     // 落地缓冲最大下缩深度 (mm)，默认10~15mm
//    float cushion_duration;  // 缓冲过程占整个支撑相的比例 [0.0, 1.0]，默认0.2
//
//    uint8_t is_running;
//} QuadrupedGait;
//
///* ================= 核心函数声明 ================= */
//// 步态类型快捷配置
//void gait_trot(QuadrupedGait *gait);
//void gait_walk(QuadrupedGait *gait);
//void gait_bound(QuadrupedGait *gait);
//void gait_pronk(QuadrupedGait *gait);
//
//// 步态初始化
//void init_quadruped_gait_trot(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height);
//void init_quadruped_gait_walk(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height);
//void init_quadruped_gait_bound(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height);
//void init_quadruped_gait_pronk(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height);
//
//// 核心参数设置
//void set_leg_phase_offset(QuadrupedGait *gait, uint8_t leg_index, float phase_offset);
//void set_gait_type(QuadrupedGait *gait, float phase_0, float phase_1, float phase_2, float phase_3);
//void set_gait_duty_factor(QuadrupedGait *gait, float duty_factor);
//void set_cushion_params(QuadrupedGait *gait, float depth, float duration);
//
//// 运动控制
//void bind_leg_motors(QuadrupedGait *gait, uint8_t leg_index, uint8_t hip_id, uint8_t knee_id);
//void calibrate_leg_base_position(QuadrupedGait *gait, uint8_t leg_index, MotorController *ctrl,int hip_id,int knee_id,int i);
//void start_quadruped_gait(QuadrupedGait *gait, float current_time);
//void stop_quadruped_gait(QuadrupedGait *gait);
//Cycloid2D_Pose get_leg_trajectory(QuadrupedGait *gait, uint8_t leg_index, float current_time);
//LegAngles get_leg_angles(QuadrupedGait *gait, uint8_t leg_index, float current_time, MotorController* ctrl, int hip_id, int calf_id,float kp,float kd);
//
//#endif















/**
 * @file gait.h
 * @brief 四足机器人步态控制核心头文件（重构可调相位版）
 */

#ifndef GAIT_H_
#define GAIT_H_

#include <stdint.h>
#include <math.h>
#include "../../MIddleware/kinematics.h"

#define PI 3.14159265358979323846f

typedef struct {
    float x;
    float y;
} Cycloid2D_Pose;

typedef enum {
    GAIT_DIRECTION_FORWARD = 0,
    GAIT_DIRECTION_BACKWARD = 1
} GaitDirection;

typedef enum {
    GAIT_PHASE_CYCLOID_FORWARD = 0,   // 摆动相
    GAIT_PHASE_STRAIGHT_BACKWARD = 1  // 支撑相
} GaitPhase;

/**
 * @struct LegGaitState
 * @brief 增加了电机绑定和物理原点记录
 */
typedef struct {
	MotorController ctrl;

    // --- 物理校准参考点 (由正运动学计算得出) ---
    float x_base;            // 站立时的物理X原点
    float y_base;            // 站立时的物理Y原点

    // --- 实时运行参数 ---
    float phase_offset;      // 新增：该腿在当前步态周期内的相位偏移，范围 [0.0, 1.0)
    uint8_t is_swing_phase;  // 当前摆动/支撑状态: 1=空中摆动, 0=触地支撑

    float x_start;
    float x_target;
    float y_start;
    float y_target;
    float lift_height;
    float cycle_time;
    float start_time;
    int direction;
    GaitPhase phase;
    uint8_t is_active;
    LegAngles current_angles;
} LegGaitState;

typedef struct {
    LegGaitState legs[4];
    float gait_cycle_time;
    float stride_length;
    float lift_height;
    float start_time;
    uint8_t is_running;
} QuadrupedGait;

/* 函数声明 */
void gait_trot(QuadrupedGait *gait);
void gait_walk(QuadrupedGait *gait);
void init_quadruped_gait_trot(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height);
void init_quadruped_gait_walk(QuadrupedGait *gait, float cycle_time, float stride_length, float lift_height);
void set_leg_phase_offset(QuadrupedGait *gait, uint8_t leg_index, float phase_offset);
void set_gait_type(QuadrupedGait *gait, float phase_0, float phase_1, float phase_2, float phase_3);

void bind_leg_motors(QuadrupedGait *gait, uint8_t leg_index, uint8_t hip_id, uint8_t knee_id);
void calibrate_leg_base_position(QuadrupedGait *gait, uint8_t leg_index, MotorController *ctrl,int hip_id,int knee_id,int i);
void start_quadruped_gait(QuadrupedGait *gait, float current_time);
void stop_quadruped_gait(QuadrupedGait *gait);
Cycloid2D_Pose get_leg_trajectory(QuadrupedGait *gait, uint8_t leg_index, float current_time);
LegAngles get_leg_angles(QuadrupedGait *gait, uint8_t leg_index, float current_time, MotorController* ctrl, int hip_id, int calf_id,float kp,float kd);

#endif
