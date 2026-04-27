/**
 * @file gait.h
 * @brief 四足机器人步态控制核心头文件（重构版）
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

    float phase_offset;
    // --- 实时运行参数 ---
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
    GaitDirection direction;
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
LegAngles get_leg_angles(QuadrupedGait* gait, int leg_id, float time, MotorController* ctrl, int id1, int id2, float kp, float kd, float y_offset);

#endif
