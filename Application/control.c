#include "control.h"
#include <string.h> // 确保 memcpy 可用
#include <math.h>
#include "3508_driver.h" // 新增：引入 3508 电机驱动头文件

extern RemoteData_t myPacket;
extern uint8_t rx_buffer[32];
uint8_t as01_status = 0;
extern uint8_t rx_data[32];
extern __attribute__((section(".RAM_D1")))__attribute__((section(".RAM_D1")))MotorController ctrl2;
extern MotorController ctrl1;
static float kp = 2.4f;
static GaitMode_e current_gait_mode = GAIT_MODE_TROT;
static uint8_t force_gait_update = 0;
static uint8_t is_single_action = 0;         // 单次动作标志位
static uint32_t single_action_end_time = 0;  // 单次动作结束时间
static GaitMode_e previous_gait_mode = GAIT_MODE_TROT; // 记录执行动作前的步态
static RobotState_e current_state = ROBOT_STATE_IDLE;
static uint8_t is_zeroing = 0;               // 标志位：是否正在执行复位插值
static uint32_t zeroing_start_time = 0;      // 记录复位动作开始的时间戳
static Currentpos zero_start_pos[4];         // 记录复位动作开始瞬间，4条腿的初始坐标
static const float ZERO_TARGET_X = 10.0f;     // 零点目标 X 坐标
static const float ZERO_TARGET_Y = 25.0f;    // 零点目标 Y 坐标
static const uint32_t ZEROING_DURATION_MS = 3000; // 复位动作耗时 (1000ms = 1秒)
static uint8_t is_adjusting_pose = 0;               // 标志位：是否正在执行高度微调
static uint32_t pose_adjust_start_time = 0;         // 记录微调动作开始的时间戳
static Currentpos pose_start_pos[4];                // 微调的起点
static Currentpos pose_target_pos[4];               // 微调的终点
static const uint32_t POSE_ADJUST_DURATION_MS = 500; // 微调耗时 (300ms)，让动作迅捷且平滑
static int64_t locked_angle[4] = {0}; // 记录锁死时的目标编码器角度
static const float CHASSIS_WHEEL_KD_SPEED = 0.2f;
static const float CHASSIS_WHEEL_KP_LOCK = 5.0f;
static const float CHASSIS_WHEEL_KD_LOCK = 0.2f;
static const float CHASSIS_WHEEL_TFF_NM = 0.0f;
static const float CHASSIS_WHEEL_MAX_TORQUE_SPEED = 1.0f;
static const float CHASSIS_WHEEL_MAX_TORQUE_LOCK = 1.5f;
static inline void apply_current_gait(QuadrupedGait* gait) {
    float straight_period = 1.2f; // 周期 (秒)
    float straight_length = 8.5f; // 步长
    float straight_height = 4.5f; // 抬腿高度
    float turn_period = 0.8f;     // 转向周期 (通常更快一点)
    float turn_length = 3.0f;     // 转向步长 (建议改小，以轮子差速为主)
    float turn_height = 5.0f;     // 转向抬腿高度 (建议改高，防止腿绊住地面)

    // 根据当前状态选择应用的参数
    float p = straight_period;
    float l = straight_length;
    float h = straight_height;

    if (current_state == ROBOT_STATE_LEFT || current_state == ROBOT_STATE_RIGHT) {
        p = turn_period;
        l = turn_length;
        h = turn_height;
    }

    switch(current_gait_mode){
        case GAIT_MODE_WALK:
            init_quadruped_gait_walk(gait, 2.0f, l, h + 2.0f); // 举例：Walk也可以应用独立参数
            break;
        case GAIT_MODE_TROT:
            init_quadruped_gait_trot(gait, p, l, h);
            break;
        case GAIT_MODE_BOUND:
//          init_quadruped_gait_bound(gait, 0.6f, 10.849f, 25.0f);
            break;
        case GAIT_MODE_PRONK:
//          init_quadruped_gait_pronk(gait, 0.6f, 10.849f, 25.0f);
            break;
    }
}
//static inline void apply_current_gait(QuadrupedGait* gait) {
//    switch(current_gait_mode){
//        case GAIT_MODE_WALK:
//    	    init_quadruped_gait_walk(gait, 2.0f, 8.5f, 6.5f);
//    	break;
//        case GAIT_MODE_TROT:
//        	init_quadruped_gait_trot(gait, 1.2f, 8.5f, 4.5f);
//        	break;
//        case GAIT_MODE_BOUND:
////        	init_quadruped_gait_bound(gait, 0.6f, 10.849f, 25.0f);
//        	break;
//        case GAIT_MODE_PRONK:
////        	init_quadruped_gait_pronk(gait, 0.6f, 10.849f, 25.0f);
//        	break;
//    }
//}
/**
 * @brief 机械臂旋钮直接角度控制 (200Hz 调用)
 */

void arm_knob_direct_control(void) {
    // 1. 将 0~100 的旋钮ADC值，映射到电机的目标弧度 (需根据机械实际限位修改)
    float target_m1 = ((float)myPacket.knob[0] / 100.0f) * M_PI ;
    float target_m2 = ((float)myPacket.knob[1] / 100.0f) * M_PI;
    float target_m3 = -((float)myPacket.knob[2] / 100.0f) * M_PI ;
    float target_m4 = ((float)myPacket.knob[3] / 100.0f) * 3.1415f - 1.5708f;

    // 2. 一阶低通滤波 (防抽搐核心代码，alpha越小跟随越柔和，越大越暴躁)
    static float current_q[4] = {0, 0, 0, 0};

    // 【新增】保存上一帧的命令用于推导期望速度
    static float last_q[4] = {0, 0, 0, 0};
    static uint8_t first_run = 1;
    float alpha = 0.05f;
    float dt = 0.005f; // 【关键】与调用频率 200Hz (5ms) 匹配

    // 初次运行，把目标直接赋值，防止开机从 0 暴力跃迁到目标点
    if(first_run) {
        current_q[0] = last_q[0] = target_m1;
        current_q[1] = last_q[1] = target_m2;
        current_q[2] = last_q[2] = target_m3;
        current_q[3] = last_q[3] = target_m4;
        first_run = 0;
        return;
    }

    current_q[0] += alpha * (target_m1 - current_q[0]);
    current_q[1] += alpha * (target_m2 - current_q[1]);
    current_q[2] += alpha * (target_m3 - current_q[2]);
    current_q[3] += alpha * (target_m4 - current_q[3]);

    // 【核心修复 1】计算真实期望速度 V_des
    float v_des_m1 = (current_q[0] - last_q[0]) / dt;
    float v_des_m2 = (current_q[1] - last_q[1]) / dt;
    float v_des_m3 = (current_q[2] - last_q[2]) / dt;
    float v_des_m4 = (current_q[3] - last_q[3]) / dt;

    last_q[0] = current_q[0];
    last_q[1] = current_q[1];
    last_q[2] = current_q[2];
    last_q[3] = current_q[3];

    // 3. 计算重力补偿前馈
    float tau_gravity_m2 = 0.0f, tau_gravity_m3 = 0.0f;
    Arm_Calc_Gravity_Torque(current_q[1], current_q[2], &tau_gravity_m2, &tau_gravity_m3);

    // 【核心修复 2】摩擦力补偿与积分补偿逻辑
    float coulomb_fric_m1 = 0.20f;
    float coulomb_fric_m2 = 0.30f;
    float coulomb_fric_m3 = 0.20f;

    float max_i_m1 = 1.000f;
    float max_i_m2 = 0.688f, max_i_m3 = 1.535f;
    float dead_zone = 0.005f;
    float Ki = 50.0f;

    static float err_i_m1 = 0.0f, err_i_m2 = 0.0f, err_i_m3 = 0.0f;
    float actual_err_m1 = current_q[0] - Arm_motors[0].POS;
    float actual_err_m2 = current_q[1] - Arm_motors[1].POS;
    float actual_err_m3 = current_q[2] - Arm_motors[2].POS;

    // 当一阶低通滤波的速度极小(接近稳态)时，开启积分补偿消除稳态误差
    if (fabsf(v_des_m1) < 0.005f) {
        if (fabsf(actual_err_m1) > dead_zone) {
            err_i_m1 += actual_err_m1 * Ki * dt;
            if(err_i_m1 > max_i_m1) err_i_m1 = max_i_m1; else if(err_i_m1 < -max_i_m1) err_i_m1 = -max_i_m1;
        }
    } else { err_i_m1 *= 0.95f; } // 运动时积分平滑衰减

    if (fabsf(v_des_m2) < 0.005f) {
        if (fabsf(actual_err_m2) > dead_zone) {
            err_i_m2 += actual_err_m2 * Ki * dt;
            if(err_i_m2 > max_i_m2) err_i_m2 = max_i_m2; else if(err_i_m2 < -max_i_m2) err_i_m2 = -max_i_m2;
        }
    } else { err_i_m2 *= 0.95f; }

    if (fabsf(v_des_m3) < 0.005f) {
        if (fabsf(actual_err_m3) > dead_zone) {
            err_i_m3 += actual_err_m3 * Ki * dt;
            if(err_i_m3 > max_i_m3) err_i_m3 = max_i_m3; else if(err_i_m3 < -max_i_m3) err_i_m3 = -max_i_m3;
        }
    } else { err_i_m3 *= 0.95f; }

    // 【核心修复 3】连续化摩擦力补偿 (Soft-Sign)
    float smooth_dir_m1 = v_des_m1 / (fabsf(v_des_m1) + 0.1f);
    float smooth_dir_m2 = v_des_m2 / (fabsf(v_des_m2) + 0.1f);
    float smooth_dir_m3 = v_des_m3 / (fabsf(v_des_m3) + 0.1f);

    float final_tau_m1 = (smooth_dir_m1 * coulomb_fric_m1) + err_i_m1;
    float final_tau_m2 = tau_gravity_m2 + (smooth_dir_m2 * coulomb_fric_m2) + err_i_m2;
    float final_tau_m3 = tau_gravity_m3 + (smooth_dir_m3 * coulomb_fric_m3) + err_i_m3;

    // 参数设定，Kp、Kd提供基本的位置刚度和阻尼
    float Kp_m1 = 30.0f, Kd_m1 = 1.5f;
    float Kp_m2 = 40.0f, Kd_m2 = 1.0f;
    float Kp_m3 = 40.0f, Kd_m3 = 1.0f;

    // 4. 下发到达妙电机 (ID, Pos, Vel, Kp, Kd, Torque)
    // 此时传入计算出的期望速度 v_des，系统跟踪性能会大幅提升
    DM_Send_Ctrl(0x01, current_q[0], v_des_m1, Kp_m1, Kd_m1, final_tau_m1);
    DM_Send_Ctrl(0x02, current_q[1], v_des_m2, Kp_m2, Kd_m2, final_tau_m2);
    DM_Send_Ctrl(0x03, current_q[2], v_des_m3, Kp_m3, Kd_m3, final_tau_m3);

    // M4 夹爪电机暂时保持纯PD控制，如果夹爪也重，可依样画葫芦加补偿
    DM_Send_Ctrl(0x04, current_q[3], v_des_m4, 20.0f, 0.5f, 0.0f);
}

/**
 * @brief 启动零位插值：捕获当前位置并锁定状态
 */
void start_zero_interpolation(uint32_t now) {
    if (is_zeroing) return; // 正在插值中则忽略
    is_zeroing = 1;
    is_adjusting_pose = 0;  // 【重要】强制打断微调状态
    zeroing_start_time = now;
    zero_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
    zero_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
    zero_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
    zero_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);
}

/**
 * @brief 零位插值执行：在 5ms 周期中平滑逼近坐标
 */
void state_zero(uint32_t now, float current_kp) {
    if (!is_zeroing) return;

    float current_target_x[4];
    float current_target_y[4];

    float progress = (float)(now - zeroing_start_time) / (float)ZEROING_DURATION_MS;
    if (progress >= 1.0f) {
        progress = 1.0f;
        is_zeroing = 0; // 进度到达 100%，结束插值
    }

    for (int i = 0; i < 4; i++) {
        current_target_x[i] = zero_start_pos[i].X + (ZERO_TARGET_X - zero_start_pos[i].X) * progress;
        current_target_y[i] = zero_start_pos[i].Y + (ZERO_TARGET_Y - zero_start_pos[i].Y) * progress;
    }

    LegAngles angles1 = InverseKinematics(current_target_x[0], current_target_y[0], &ctrl2, 1, 2, 0.3, 0.1);
    MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles1.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles1.theta1, current_kp, 0.1);

    LegAngles angles2 = InverseKinematics(current_target_x[1], current_target_y[1], &ctrl2, 7, 8, 0.3, 0.1);
    MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles2.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles2.theta1, current_kp, 0.1);

    LegAngles angles3 = InverseKinematics(current_target_x[2], current_target_y[2], &ctrl1, 6, 5, 0.3, 0.1);
    MotorController_SetCommand(&ctrl1, 5, 1, 0.0, 0.0f, angles3.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl1, 6, 1, 0.0, 0.0f, angles3.theta1, current_kp, 0.1);

    LegAngles angles4 = InverseKinematics(current_target_x[3], current_target_y[3], &ctrl1, 3, 4, 0.3, 0.1);
    MotorController_SetCommand(&ctrl1, 4, 1, 0.0, 0.0f, angles4.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl1, 3, 1, 0.0, 0.0f, angles4.theta1, current_kp, 0.1);
}
void start_pose_adjustment(uint32_t now, float delta_x, float delta_y) {
    if (is_adjusting_pose) return;

    is_adjusting_pose = 1;
    is_zeroing = 0;        // 【重要】强制打断归零插值状态
    pose_adjust_start_time = now;
    pose_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
    pose_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
    pose_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
    pose_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);

    // 计算终点目标位置
    for (int i = 0; i < 4; i++) {
        pose_target_pos[i].X = pose_start_pos[i].X + delta_x;
        pose_target_pos[i].Y = pose_start_pos[i].Y + delta_y;
    }
}

/**
 * @brief 姿态微调执行：在 5ms 周期中平滑逼近微调后的坐标
 */
void update_pose_adjustment(uint32_t now, float current_kp) {
    if (!is_adjusting_pose) return;

    float progress = (float)(now - pose_adjust_start_time) / (float)POSE_ADJUST_DURATION_MS;
    if (progress >= 1.0f) {
        progress = 1.0f;
        is_adjusting_pose = 0;
    }

    float current_target_x[4];
    float current_target_y[4];

    for (int i = 0; i < 4; i++) {
        current_target_x[i] = pose_start_pos[i].X + (pose_target_pos[i].X - pose_start_pos[i].X) * progress;
        current_target_y[i] = pose_start_pos[i].Y + (pose_target_pos[i].Y - pose_start_pos[i].Y) * progress;
    }

    LegAngles angles1 = InverseKinematics(current_target_x[0], current_target_y[0], &ctrl2, 1, 2, 0.5, 0.1);
    MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles1.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles1.theta1, current_kp, 0.1);

    LegAngles angles2 = InverseKinematics(current_target_x[1], current_target_y[1], &ctrl2, 7, 8, 0.5, 0.1);
    MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles2.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles2.theta1, current_kp, 0.1);

    LegAngles angles3 = InverseKinematics(current_target_x[2], current_target_y[2], &ctrl1, 3, 4, 0.5, 0.1);
    MotorController_SetCommand(&ctrl1, 4, 1, 0.0, 0.0f, angles3.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl1, 3, 1, 0.0, 0.0f, angles3.theta1, current_kp, 0.1);

    LegAngles angles4 = InverseKinematics(current_target_x[3], current_target_y[3], &ctrl1, 6, 5, 0.5, 0.1);
    MotorController_SetCommand(&ctrl1, 5, 1, 0.0, 0.0f, angles4.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl1, 6, 1, 0.0, 0.0f, angles4.theta1, current_kp, 0.1);
}

/**
 * @brief 摇杆控制处理
 */
void joystick_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
    RobotState_e target_state = ROBOT_STATE_IDLE;
    float currentTimeSec = now / 1000.0f;
    static uint32_t last_calc_time = 0;

    if (is_single_action) {
        if (now >= single_action_end_time) {
            is_single_action = 0;
            current_gait_mode = previous_gait_mode;
            apply_current_gait(gait);
            current_state = ROBOT_STATE_IDLE;
            start_zero_interpolation(now);          // 触发跳跃后的平滑复位
        } else {
            if (now - last_calc_time >= 5) {
                last_calc_time = now;
                angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
                angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, kp, 0.1);
                angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
                angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);

                // 单次动作时保持 3508 轮子停止
                for (int i = 0; i < 4; i++) {
                    MIT_Wheel_Control(i, Motors[i].total_angle, 0.0f, 0.0f, CHASSIS_WHEEL_KD_SPEED, CHASSIS_WHEEL_TFF_NM, CHASSIS_WHEEL_MAX_TORQUE_SPEED);
                }
            }
        }
        return;
    }

    if (myPacket.joy_lx > 60) {
        target_state = ROBOT_STATE_BACKWARD;
    } else if (myPacket.joy_lx < 40) {
        target_state = ROBOT_STATE_FORWARD;
    } else if (myPacket.joy_rx > 60) {
        target_state = ROBOT_STATE_LEFT;
    } else if (myPacket.joy_rx < 40) {
        target_state = ROBOT_STATE_RIGHT;
    } else {
        target_state = ROBOT_STATE_IDLE;
    }

    // 状态切换检测
    if (target_state != current_state) {
        current_state = target_state;
        force_gait_update = 0;
        switch (current_state) {
            case ROBOT_STATE_FORWARD:
            	apply_current_gait(gait);
                calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
                calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
                calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
                start_quadruped_gait(gait, currentTimeSec);
                break;
            case ROBOT_STATE_BACKWARD:
            	apply_current_gait(gait);
                calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
                calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
                calibrate_leg_base_position(gait, 2, ctrl1,6,5,0);
                calibrate_leg_base_position(gait, 3, ctrl1,3,4,0);
                start_quadruped_gait(gait, currentTimeSec);
                break;
            case ROBOT_STATE_LEFT:
            	apply_current_gait(gait);
            	calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
            	calibrate_leg_base_position(gait, 2, ctrl1,3,4,1);
                calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                calibrate_leg_base_position(gait, 3, ctrl1,6,5,0);
                start_quadruped_gait(gait, currentTimeSec);
                break;
            case ROBOT_STATE_RIGHT:
            	apply_current_gait(gait);
            	calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
            	calibrate_leg_base_position(gait, 2, ctrl1,3,4,0);
                calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
                calibrate_leg_base_position(gait, 3, ctrl1,6,5,1);
                start_quadruped_gait(gait, currentTimeSec);
                break;
            case ROBOT_STATE_IDLE:
                start_zero_interpolation(now);
                for(int i = 0; i < 4; i++) {
                   locked_angle[i] = Motors[i].total_angle;
                     }
                break;
        }
    }
        if (now - last_calc_time >= 5) {
            last_calc_time = now;

            if (current_state != ROBOT_STATE_IDLE) {

                // 修复语法错误：必须明确比较两侧
                if (current_state == ROBOT_STATE_FORWARD || current_state == ROBOT_STATE_BACKWARD) {
                    // 直行状态：使用全局 kp
                    angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
                    angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, kp, 0.1);
                    angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
                    angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);
                } else if (current_state == ROBOT_STATE_LEFT || current_state == ROBOT_STATE_RIGHT) {
                    // 转向状态：使用独立的转向 kp (2.6)
                    angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, 2.6f, 0.1);
                    angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, 2.6f, 0.1);
                    angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, 2.6f, 0.1);
                    angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, 2.6f, 0.1);
                }

                // ==========================================
                // 3508 轮毂电机差速控制逻辑
                // ==========================================
                float wheel_target_rpm_01 = 0.0f; // 电机 0, 1 的目标速度
                float wheel_target_rpm_23 = 0.0f; // 电机 2, 3 的目标速度

                float base_rpm = 1000.0f; // 直行基础转速
                float turn_rpm = 800.0f;  // 转向时的差速转速（设为正数即可，下方逻辑决定方向）

                if (current_state == ROBOT_STATE_FORWARD) {
                    wheel_target_rpm_01 = base_rpm;
                    wheel_target_rpm_23 = -base_rpm;
                } else if (current_state == ROBOT_STATE_BACKWARD) {
                    wheel_target_rpm_01 = -base_rpm;
                    wheel_target_rpm_23 = base_rpm;
                } else if (current_state == ROBOT_STATE_LEFT) {
                    // 左转：左侧轮子后退(-)，右侧轮子前进(-)
                    wheel_target_rpm_01 = -turn_rpm;
                    wheel_target_rpm_23 = -turn_rpm;
                } else if (current_state == ROBOT_STATE_RIGHT) {
                    // 右转：左侧轮子前进(+)，右侧轮子后退(+)
                    wheel_target_rpm_01 = turn_rpm;
                    wheel_target_rpm_23 = turn_rpm;
                }

                for (int i = 0; i < 2; i++) {
                    MIT_Wheel_Control(i, Motors[i].total_angle, wheel_target_rpm_01, 0.0f, CHASSIS_WHEEL_KD_SPEED, CHASSIS_WHEEL_TFF_NM, CHASSIS_WHEEL_MAX_TORQUE_SPEED);
                }
                for (int i = 2; i < 4; i++) {
                    MIT_Wheel_Control(i, Motors[i].total_angle, wheel_target_rpm_23, 0.0f, CHASSIS_WHEEL_KD_SPEED, CHASSIS_WHEEL_TFF_NM, CHASSIS_WHEEL_MAX_TORQUE_SPEED);
                }

            } else {
                            if (is_zeroing) {
                                state_zero(now, kp);
                            } else if (is_adjusting_pose) {
                                update_pose_adjustment(now, kp);
                            }

                            // 【修改】在 5ms 周期里持续计算位置环，抵抗外力
                            for (int i = 0; i < 4; i++) {
                                MIT_Wheel_Control(i, locked_angle[i], 0.0f, CHASSIS_WHEEL_KP_LOCK, CHASSIS_WHEEL_KD_LOCK, 0.0f, CHASSIS_WHEEL_MAX_TORQUE_LOCK);
                            }
                        }
        }
}
//    if (now - last_calc_time >= 5) {
//        last_calc_time = now;
//        if (current_state != ROBOT_STATE_IDLE) {
//        	if (current_state == ROBOT_STATE_FORWARD || ROBOT_STATE_BACKWARD){
//            angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
//            angles = get_leg_angles(gait, 2 ,currentTimeSec, ctrl1, 6, 5, kp, 0.1);
//            angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
//            angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);
//        	}else {
//                angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, 2.6, 0.1);
//                angles = get_leg_angles(gait, 2 ,currentTimeSec, ctrl1, 6, 5, 2.6, 0.1);
//                angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, 2.6, 0.1);
//                angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, 2.6, 0.1);
//        	}
//            float wheel_target_rpm_01 = 0.0f; // 电机 0, 1 的目标速度
//            float wheel_target_rpm_23 = 0.0f; // 电机 2, 3 的目标速度
//
//            float base_rpm = 1000.0f; // 直行基础转速
//            float turn_rpm = -800.0f;  // 转向时的差速转速 (建议比直行稍慢，防止打滑)
//
//            // 假设：
//            // 电机 0, 1 安装在同一侧（例如左侧），给正值时轮子向前转。
//            // 电机 2, 3 安装在另一侧（例如右侧），镜像安装，给负值时轮子向前转。
//            if (current_state == ROBOT_STATE_FORWARD) {
//                wheel_target_rpm_01 = base_rpm;
//                wheel_target_rpm_23 = -base_rpm;
//            } else if (current_state == ROBOT_STATE_BACKWARD) {
//                wheel_target_rpm_01 = -base_rpm;
//                wheel_target_rpm_23 = base_rpm;
//            } else if (current_state == ROBOT_STATE_LEFT) {
//                // 左转：左侧轮子后退(-)，右侧轮子前进(-)
//                wheel_target_rpm_01 = -turn_rpm;
//                wheel_target_rpm_23 = -turn_rpm;
//            } else if (current_state == ROBOT_STATE_RIGHT) {
//                // 右转：左侧轮子前进(+)，右侧轮子后退(+)
//                wheel_target_rpm_01 = turn_rpm;
//                wheel_target_rpm_23 = turn_rpm;
//            }
//
//            for (int i = 0; i < 2; i++) {
//                Motors[i].target_speed = wheel_target_rpm_01;
//                PID_Calc_Speed(i);
//            }
//            for (int i = 2; i < 4; i++) {
//                Motors[i].target_speed = wheel_target_rpm_23;
//                PID_Calc_Speed(i);
//            }
//            // ==========================================
//
//        } else {
//            // 根据标志位，选择性处理相应的平滑过渡
//            if (is_zeroing) {
//                state_zero(now, kp);
//            } else if (is_adjusting_pose) {
//                update_pose_adjustment(now, kp);
//            }
//            for (int i = 0; i < 4; i++) {
//                Motors[i].target_speed = 0.0f;
//                PID_Calc_Speed(i);
//            }
//        }
//    }
//}
/**
 * @brief 按键控制逻辑
 */
#define DEBOUNCE_DELAY_MS 30
#define NUM_BUTTONS 8

void button_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t currentTime) {
    static uint8_t stable_button_states[NUM_BUTTONS] = {0};
    static uint8_t last_button_readings[NUM_BUTTONS] = {0};
    static uint32_t last_debounce_times[NUM_BUTTONS] = {0};

	float dy = 2.0f;
    float dx = -0.5f;

    // 消抖与按键处理
    for (int i = 0; i < NUM_BUTTONS; i++) {
        uint8_t current_reading = myPacket.button[i];

        if (current_reading != last_button_readings[i]) {
            last_debounce_times[i] = currentTime;
        }

        if ((currentTime - last_debounce_times[i]) > DEBOUNCE_DELAY_MS) {
            if (current_reading != stable_button_states[i]) {
                stable_button_states[i] = current_reading;
                if (stable_button_states[i] == 1) {
                    switch (i) {
                        case 0:
                            // 归零
                            start_zero_interpolation(currentTime);
//                            state_zero0(currentTime);
                            break;

                        case 1:
                            // 升高底盘 (Y+dy, X-dx) - 触发微调插值
                            start_pose_adjustment(currentTime, -dx, dy);
                            break;

                        case 2:
                            // 降低底盘 (Y-dy, X-dx) - 触发微调插值
                            start_pose_adjustment(currentTime, -dx, -dy);
                            break;

                        case 3:
                            break;

                        case 4:
                            if (current_gait_mode != GAIT_MODE_TROT) {
                                current_gait_mode = GAIT_MODE_TROT;
                                force_gait_update = 1;
                            }
                            break;

                        case 5:
                        	if (current_gait_mode != GAIT_MODE_WALK) {
                                current_gait_mode = GAIT_MODE_WALK;
                                force_gait_update = 1;
                            }
                            break;

                        case 6: // 单次执行 BOUND
                        	if (!is_single_action) {
                                previous_gait_mode = current_gait_mode;
                                current_gait_mode = GAIT_MODE_BOUND;
                                apply_current_gait(gait);
                                calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
                                calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                                calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
                                calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
                                start_quadruped_gait(gait, currentTime / 1000.0f);

                                is_single_action = 1;
                                single_action_end_time = currentTime + 600;
                        	}
                            break;

                        case 7: // 单次执行 PRONK
                        	if (!is_single_action) {
                                previous_gait_mode = current_gait_mode;
                        	    current_gait_mode = GAIT_MODE_PRONK;
                                apply_current_gait(gait);

                                calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
                                calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                                calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
                                calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
                                start_quadruped_gait(gait, currentTime / 1000.0f);

                                is_single_action = 1;
                                single_action_end_time = currentTime + 600;
                        	}
                            break;

                        default:
                            break;
                    }
                }
            }
        }
        last_button_readings[i] = current_reading;
    }
}

/**
 * @brief 接收数据并分发控制
 */
void AS01_rx(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
    if (NRF24L01_RxPacket(rx_buffer) == 0) {
        memcpy(&myPacket, rx_buffer, sizeof(RemoteData_t));
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    joystick_control(ctrl1, ctrl2, gait, startTime, angles, now);
    button_control(ctrl1, ctrl2, gait, now);
//    arm_knob_direct_control;
}
