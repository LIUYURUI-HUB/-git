// LEGACY / DISABLED:
// This file is intentionally kept as historical reference only.
// The active leg/chassis control path is:
//   main.c -> AS01_rx() -> joystick_control() -> Application/control.c
// Do not re-enable this file without first reconciling its state machine with control.c.

///*
// * leg_task.c
// *
// * Created on: Mar 30, 2026
// * Author: 22569
// */
//#include "protocol_handler.h"
//#include "Chassis_Control.h"
//#include "gait.h"
//#include "leg_task.h"
//#include <string.h> // 确保 memcpy 可用
//
//// --- 补充缺失的函数前置声明，解决隐式声明警告 (Implicit Declaration) ---
//extern void init_quadruped_gait_trot(QuadrupedGait* gait, float param1, float param2, float param3);
//extern void state_zero(float* offsets); // 注意：如果这里你想调用的是下面定义的 state_zero_with_compensation，请将下方 122 行的代码改掉。
//
//
////extern uint8_t rx_buffer[32];
////uint8_t as01_status = 0;
//extern __attribute__((section(".RAM_D1")))__attribute__((section(".RAM_D1")))MotorController ctrl2;
//extern MotorController ctrl1;
//extern float leg_y_offsets[4]; // 声明外部的补偿数组
//
//void state_zero_with_compensation(float* leg_y_offsets){
//	LegAngles angles1 = InverseKinematics(9,30+leg_y_offsets[0],&ctrl2, 1, 2, 0.2, 0.1);
//	MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles1.theta2, 0.2, 0.1);
//	MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles1.theta1, 0.2, 0.1);
//
//	LegAngles angles2 = InverseKinematics(9,30+leg_y_offsets[1],&ctrl2, 7, 8, 0.2, 0.1);
//	MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles2.theta2, 0.2, 0.1);
//	MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles2.theta1, 0.2, 0.1);
//
//	LegAngles angles3 = InverseKinematics(9,30+leg_y_offsets[2],&ctrl1, 6, 5, 0.2, 0.1);
//	MotorController_SetCommand(&ctrl1, 5, 1, 0.0, 0.0f, angles3.theta2, 0.2, 0.1);
//	MotorController_SetCommand(&ctrl1, 6, 1, 0.0, 0.0f, angles3.theta1, 0.2, 0.1);
//
//	LegAngles angles4 = InverseKinematics(9,30+leg_y_offsets[3],&ctrl1, 3, 4, 0.2, 0.1);
//    MotorController_SetCommand(&ctrl1, 4, 1, 0.0, 0.0f, angles4.theta2, 0.2, 0.1);
//    MotorController_SetCommand(&ctrl1, 3, 1, 0.0, 0.0f, angles4.theta1, 0.2, 0.1);
//}
//
//// --- 修复点：将嵌套函数展开为平级的全局函数，并将原本的局部静态变量提升为文件级静态变量 ---
//// 增加静态变量记录刚开始下蹲时的基准坐标和当前下蹲深度
//static float crouch_base_X[4];
//static float crouch_base_Y[4];
//static float current_dy = 0.0f;
//
//// 在刚切换到蹲伏状态时调用，记录当前的站立高度
//void state_crouch_init(void) {
//    current_dy = 0.0f; // 重置下蹲深度
//
//    Currentpos pos1 = ForwardKinematics(&ctrl2,1,2);
//    Currentpos pos2 = ForwardKinematics(&ctrl2,7,8);
//    Currentpos pos3 = ForwardKinematics(&ctrl1,6,5);
//    Currentpos pos4 = ForwardKinematics(&ctrl1,3,4);
//
//    crouch_base_X[0] = pos1.X; crouch_base_Y[0] = pos1.Y;
//    crouch_base_X[1] = pos2.X; crouch_base_Y[1] = pos2.Y;
//    crouch_base_X[2] = pos3.X; crouch_base_Y[2] = pos3.Y;
//    crouch_base_X[3] = pos4.X; crouch_base_Y[3] = pos4.Y;
//}
//
//// 在 5ms 定时循环中持续调用，实现缓慢下降
//void state_crouch_update(void) {
//    float target_dy = 10.0f; // 最终想要下降的深度
//    float step = 0.05f;      // 每次下降的步长。5ms一次，0.02f的步长意味着1秒钟下降4，非常平滑。
//
//    // 如果还没有降到底，就继续增加下蹲深度
//    if (current_dy < target_dy) {
//        current_dy += step;
//        if (current_dy > target_dy) {
//            current_dy = target_dy; // 限制最大深度
//        }
//    }
//
//    // 基于初始高度持续计算 IK 并下发
//    LegAngles angles1 = InverseKinematics(crouch_base_X[0], crouch_base_Y[0] - current_dy, &ctrl2, 1, 2, 0.2, 0.1);
//    MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles1.theta2, 0.2, 0.1);
//    MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles1.theta1, 0.2, 0.1);
//
//    LegAngles angles2 = InverseKinematics(crouch_base_X[1], crouch_base_Y[1] - current_dy, &ctrl2, 7, 8, 0.2, 0.1);
//    MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles2.theta2, 0.2, 0.1);
//    MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles2.theta1, 0.2, 0.1);
//
//    LegAngles angles3 = InverseKinematics(crouch_base_X[2], crouch_base_Y[2] - current_dy, &ctrl1, 6, 5, 0.2, 0.1);
//    MotorController_SetCommand(&ctrl1, 5, 1, 0.0, 0.0f, angles3.theta2, 0.2, 0.1);
//    MotorController_SetCommand(&ctrl1, 6, 1, 0.0, 0.0f, angles3.theta1, 0.2, 0.1);
//
//    LegAngles angles4 = InverseKinematics(crouch_base_X[3], crouch_base_Y[3] - current_dy, &ctrl1, 3, 4, 0.2, 0.1);
//    MotorController_SetCommand(&ctrl1, 4, 1, 0.0, 0.0f, angles4.theta2, 0.2, 0.1);
//    MotorController_SetCommand(&ctrl1, 3, 1, 0.0, 0.0f, angles4.theta1, 0.2, 0.1);
//}
//
//
//// 定义机器人的运动状态枚举
//typedef enum {
//    ROBOT_STATE_IDLE = 0,
//    ROBOT_STATE_FORWARD = 1,
//    ROBOT_STATE_BACKWARD = 2,
//    ROBOT_STATE_LEFT = 3,
//    ROBOT_STATE_RIGHT = 4,
//    ROBOT_STATE_CROUCH = 5
//} RobotState_e;
//
//// 使用静态变量记忆当前状态
//static RobotState_e current_state = ROBOT_STATE_IDLE;
//
////上位机控制处理
//// 上位机控制处理
//void vision_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
//
//    Chassis_Move_t cmd = Protocol_GetChassisMove();
//    float currentTimeSec = now / 1000.0f;
//
//    // 1. 目标状态判断（直接读取上位机下发的状态码）
//    RobotState_e target_state = (RobotState_e)cmd.state;
//
//    // 安全保护：如果收到了非法的状态码，强制置为待机
//    if (target_state > ROBOT_STATE_CROUCH) {
//        target_state = ROBOT_STATE_IDLE;
//    }
//
//    // 2. 状态切换检测（仅在切换瞬时执行初始化）
//    if (target_state != current_state) {
//        current_state = target_state;
//
//        switch (current_state) {
//            case ROBOT_STATE_FORWARD:
//                init_quadruped_gait_trot(gait, 0.8f, 25.849f, 8.0f);
//                calibrate_leg_base_position(gait, 0, ctrl2, 7, 8, 1);
//                calibrate_leg_base_position(gait, 1, ctrl2, 1, 2, 1);
//                calibrate_leg_base_position(gait, 2, ctrl1, 6, 5, 1);
//                calibrate_leg_base_position(gait, 3, ctrl1, 3, 4, 1);
//                start_quadruped_gait(gait, currentTimeSec);
//                break;
//
//            case ROBOT_STATE_BACKWARD:
//                init_quadruped_gait_trot(gait, 0.8f, 25.849f, 8.0f);
//                calibrate_leg_base_position(gait, 0, ctrl2, 7, 8, 0);
//                calibrate_leg_base_position(gait, 1, ctrl2, 1, 2, 0);
//                calibrate_leg_base_position(gait, 2, ctrl1, 6, 5, 0);
//                calibrate_leg_base_position(gait, 3, ctrl1, 3, 4, 0);
//                start_quadruped_gait(gait, currentTimeSec);
//                break;
//
//            // 左转
//            case ROBOT_STATE_LEFT:
//                init_quadruped_gait_trot(gait, 0.8f, 25.849f, 8.0f);
//                calibrate_leg_base_position(gait, 0, ctrl2, 7, 8, 0);
//                calibrate_leg_base_position(gait, 2, ctrl1, 3, 4, 1);
//                calibrate_leg_base_position(gait, 1, ctrl2, 1, 2, 1);
//                calibrate_leg_base_position(gait, 3, ctrl1, 6, 5, 0);
//                start_quadruped_gait(gait, currentTimeSec);
//                break;
//
//            // 右转
//            case ROBOT_STATE_RIGHT:
//                init_quadruped_gait_trot(gait, 0.8f, 25.849f, 8.0f);
//                calibrate_leg_base_position(gait, 0, ctrl2, 7, 8, 1);
//                calibrate_leg_base_position(gait, 2, ctrl1, 3, 4, 0);
//                calibrate_leg_base_position(gait, 1, ctrl2, 1, 2, 0);
//                calibrate_leg_base_position(gait, 3, ctrl1, 6, 5, 1);
//                start_quadruped_gait(gait, currentTimeSec);
//                break;
//
//            case ROBOT_STATE_IDLE:
//                state_zero(leg_y_offsets);
//                break;
//
//            case ROBOT_STATE_CROUCH:
//                state_crouch_init(); // 记录当前基准高度，准备缓慢下蹲
//                break;
//        }
//    }
//
//    // 3. 连续执行的动作（频率控制：200Hz / 5ms）
//    static uint32_t last_calc_time = 0;
//    if (now - last_calc_time >= 5) {
//        last_calc_time = now;
//
//        if (current_state == ROBOT_STATE_IDLE) {
//            // 原地站立时的连续姿态补偿
//            state_zero_with_compensation(leg_y_offsets);
//        }
//        else if (current_state == ROBOT_STATE_CROUCH) {
//            // 缓慢下蹲更新
//            state_crouch_update();
//        }
//        else {
//            // 行走时的连续姿态补偿
//            angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, 0.2, 0.1);
//            angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, 0.2, 0.1);
//            angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, 0.2, 0.1);
//            angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, 0.2, 0.1);
//        }
//    }
//}
