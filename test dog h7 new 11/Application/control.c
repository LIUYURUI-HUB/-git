#include "control.h"
#include "Chassis_Control.h"
#include <string.h> // 确保 memcpy 可用
#include <math.h>
#include "3508_driver.h"

extern RemoteData_t myPacket;
extern uint8_t rx_buffer[32];
uint8_t as01_status = 0;
extern uint8_t rx_data[32];
extern __attribute__((section(".RAM_D1")))__attribute__((section(".RAM_D1")))MotorController ctrl2;
extern MotorController ctrl1;
static float kp = 2.6f;
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
static const uint32_t ZEROING_DURATION_MS = 2500; // 复位动作耗时 (1000ms = 1秒)
static uint32_t current_zeroing_duration = 2500; // 新增：用于记录当前这次动作需要的总时长
static const uint32_t ZEROING_last_ms = 1000; // 复位动作耗时 (1000ms = 1秒)
static uint8_t is_adjusting_pose = 0;               // 标志位：是否正在执行高度微调
static uint32_t pose_adjust_start_time = 0;         // 记录微调动作开始的时间戳
static Currentpos pose_start_pos[4];                // 微调的起点
static Currentpos pose_target_pos[4];               // 微调的终点
static const uint32_t POSE_ADJUST_DURATION_MS = 300; // 微调耗时 (300ms)，让动作迅捷且平滑
static uint8_t is_first_zero = 0;
static uint8_t is_custom_interp = 0;             // 标志位：是否正在执行自定义分组插值
static uint32_t custom_interp_start_time = 0;    // 记录自定义动作开始的时间戳
static Currentpos custom_start_pos[4];           // 记录动作开始瞬间，4条腿的初始坐标
static uint32_t custom_interp_duration = 1000;   // 自定义动作耗时 (默认 1000ms，可在启动时修改)
static float custom_target_x_grp1 = 0.0f; // 腿1,2 的目标X
static float custom_target_y_grp1 = 0.0f; // 腿1,2 的目标Y
static float custom_target_x_grp2 = 0.0f;  // 腿0,3 的目标X
static float custom_target_y_grp2 = 0.0f; // 腿0,3 的目标Y
/**
 * @brief 姿态补偿判定开关
 * 1: 开启补偿，机器狗会根据 IMU 数据自动调整腿长以保持平衡
 * 0: 关闭补偿，机器狗仅维持基础高度
 */
uint8_t is_attitude_comp_enabled = 0;  // 默认设为 0，等待起立完成后开启
// 【修改点1】：替换原来的 is_pure_wheel_mode，升级为3态模式
// 底盘模式：0-轮腿联动，1-纯轮模式，2-纯足模式(锁轮)
static uint8_t chassis_mode = 0;
#define is_pure_wheel_mode (chassis_mode == 1) // 兼容原有判断
// 用于记录纯足模式下4个轮子的驻车目标角度
static int64_t locked_angles[4] = {0};

static inline void apply_current_gait(QuadrupedGait* gait) {
    float straight_period = 1.2f; // 周期 (秒)
    float straight_length = 8.5f; // 步长
    float straight_height = 10.0f; // 抬腿高度
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
            init_quadruped_gait_walk(gait, 1.2f, l, h + 2.0f); // 举例：Walk也可以应用独立参数
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
/**
 * @brief 启动自定义分组插值：捕获当前位置
 * @param now 当前系统时间
 * @param duration_ms 期望完成这个动作的总毫秒数
 */
void start_custom_interpolation(uint32_t now, uint32_t duration_ms) {
    if (is_custom_interp) return;

    is_custom_interp = 1;
    is_zeroing = 0;         // 【取消注释】强制打断归零
    is_adjusting_pose = 0;  // 【取消注释】强制打断微调

    custom_interp_duration = duration_ms;
    custom_interp_start_time = now;

    custom_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2); // 组2
    custom_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8); // 组1
    custom_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5); // 组1
    custom_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4); // 组2
}
/**
 * @brief 启动零位插值：捕获当前位置并锁定状态
 */
void start_zero_interpolation(uint32_t now) {
    if (is_zeroing) return; // 正在插值中则忽略
    is_zeroing = 1;
    is_adjusting_pose = 0;  // 【重要】强制打断微调状态
    if (is_first_zero == 0) {
            current_zeroing_duration = ZEROING_DURATION_MS; // 2500ms
            is_first_zero = 1; // 标记以后不再是第一次了
        } else {
            current_zeroing_duration = ZEROING_last_ms;    // 1000ms
        }
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
    float progress = (float)(now - zeroing_start_time) / (float)current_zeroing_duration;
    if (progress >= 1.0f) {
        progress = 1.0f;
        is_zeroing = 0; // 进度到达 100%，结束插值
        if (progress >= 1.0f) {
                progress = 1.0f;
                is_zeroing = 0; // 进度到达 100%，结束插值
                // 【关键修改】：机器狗已经稳稳地站平了！此时正式激活姿态补偿！
                is_attitude_comp_enabled = 1;
                // 站立状态四腿全额参与补偿
                Chassis_Set_All_Leg_Weights(1.0f, 1.0f, 1.0f, 1.0f);
            }
    }
    for (int i = 0; i < 4; i++) {
        current_target_x[i] = zero_start_pos[i].X + (ZERO_TARGET_X - zero_start_pos[i].X) * progress;
        current_target_y[i] = zero_start_pos[i].Y + (ZERO_TARGET_Y - zero_start_pos[i].Y) * progress;
    }
//    for (int i = 0; i < 4; i++) {
//        current_target_x[i] = zero_start_pos[i].X + (ZERO_TARGET_X - zero_start_pos[i].X) * progress;
//        current_target_y[i] = zero_start_pos[i].Y + (ZERO_TARGET_Y - zero_start_pos[i].Y) * progress;
//    }
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
/**
 * @brief 自定义分组插值执行：平滑逼近两组不同的目标坐标
 * @param target_x_grp1 第一组目标 X (对应腿 1, 2)
 * @param target_y_grp1 第一组目标 Y (对应腿 1, 2)
 * @param target_x_grp2 第二组目标 X (对应腿 0, 3)
 * @param target_y_grp2 第二组目标 Y (对应腿 0, 3)
 */
void state_custom_interpolation(uint32_t now, float current_kp,
                                float target_x_grp1, float target_y_grp1,
                                float target_x_grp2, float target_y_grp2) {
    if (!is_custom_interp) return;

    float progress = (float)(now - custom_interp_start_time) / (float)custom_interp_duration;
    if (progress >= 1.0f) {
        progress = 1.0f;
        is_custom_interp = 0; // 进度到达 100%，结束插值
    }

    float current_target_x[4];
    float current_target_y[4];

    // ====== 分组2：腿0和腿3，使用 grp2 的参数 ======
    current_target_x[0] = custom_start_pos[0].X + (target_x_grp2 - custom_start_pos[0].X) * progress;
    current_target_y[0] = custom_start_pos[0].Y + (target_y_grp2 - custom_start_pos[0].Y) * progress;

    current_target_x[3] = custom_start_pos[3].X + (target_x_grp2 - custom_start_pos[3].X) * progress;
    current_target_y[3] = custom_start_pos[3].Y + (target_y_grp2 - custom_start_pos[3].Y) * progress;

    // ====== 分组1：腿1和腿2，使用 grp1 的参数 ======
    current_target_x[1] = custom_start_pos[1].X + (target_x_grp1 - custom_start_pos[1].X) * progress;
    current_target_y[1] = custom_start_pos[1].Y + (target_y_grp1 - custom_start_pos[1].Y) * progress;

    current_target_x[2] = custom_start_pos[2].X + (target_x_grp1 - custom_start_pos[2].X) * progress;
    current_target_y[2] = custom_start_pos[2].Y + (target_y_grp1 - custom_start_pos[2].Y) * progress;

    // 只有在补偿使能的情况下，才叠加补偿量
    if (is_attitude_comp_enabled) {
        for(int i = 0; i < 4; i++) {
            current_target_y[i] += Chassis_Get_Leg_Offset(i);
        }
    }

    // 下发控制指令 (逆运动学解算与电机输出)
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
    // 还原基础起步坐标（仅当补偿开启时剥离）
    pose_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
    pose_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
    pose_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
    pose_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);

    if (is_attitude_comp_enabled) {
           pose_start_pos[0].Y -= Chassis_Get_Leg_Offset(0);
           pose_start_pos[1].Y -= Chassis_Get_Leg_Offset(1);
           pose_start_pos[2].Y -= Chassis_Get_Leg_Offset(2);
           pose_start_pos[3].Y -= Chassis_Get_Leg_Offset(3);
       }
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

        // 补偿注入
        if (is_attitude_comp_enabled) {
            current_target_y[i] += Chassis_Get_Leg_Offset(i);
        }
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
                // 只有在使能的情况下才计算补偿
                if (is_attitude_comp_enabled) {
                    Chassis_Attitude_Loop(0.005f);
                }

                angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
                angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, kp, 0.1);
                angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
                angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);

                // 【修改点2】：单次动作时保持 3508 轮子状态 (纯足模式保持锁死，其余模式归零)
                if (chassis_mode == 2) {
                    for (int i = 0; i < 4; i++) {
                        Motors[i].target_angle = locked_angles[i];
                        PID_Calc_Position(i, Motors[i].target_angle);
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        Motors[i].target_speed = 0.0f;
                        PID_Calc_Speed(i);
                    }
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
        for(int i=0; i<4; i++){
                 Clear_Motor_PID(i);
                }
        switch (current_state) {
            case ROBOT_STATE_FORWARD:
                if (!is_pure_wheel_mode) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                    calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
                    calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
                    start_quadruped_gait(gait, currentTimeSec);
                }
                break;
            case ROBOT_STATE_BACKWARD:
                if (!is_pure_wheel_mode) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
                    calibrate_leg_base_position(gait, 2, ctrl1,6,5,0);
                    calibrate_leg_base_position(gait, 3, ctrl1,3,4,0);
                    start_quadruped_gait(gait, currentTimeSec);
                }
                break;
            case ROBOT_STATE_LEFT:
                if (!is_pure_wheel_mode) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
                    calibrate_leg_base_position(gait, 2, ctrl1,3,4,1);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                    calibrate_leg_base_position(gait, 3, ctrl1,6,5,0);
                    start_quadruped_gait(gait, currentTimeSec);
                }
                break;
            case ROBOT_STATE_RIGHT:
                if (!is_pure_wheel_mode) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
                    calibrate_leg_base_position(gait, 2, ctrl1,3,4,0);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
                    calibrate_leg_base_position(gait, 3, ctrl1,6,5,1);
                    start_quadruped_gait(gait, currentTimeSec);
                }
                break;
            case ROBOT_STATE_IDLE:
                start_zero_interpolation(now);
                break;
        }
    }

    if (now - last_calc_time >= 5) {
        last_calc_time = now;

        // 【控制核心】：当补偿开启时，持续计算。
        if (is_attitude_comp_enabled) {
            Chassis_Attitude_Loop(0.005f);
        }

        if (current_state != ROBOT_STATE_IDLE) {

            // 区分纯轮模式与正常步态模式的腿部运算
            if (!is_pure_wheel_mode) {
                if (current_state == ROBOT_STATE_FORWARD || current_state == ROBOT_STATE_BACKWARD) {
                    angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
                    angles = get_leg_angles(gait, 2 ,currentTimeSec, ctrl1, 6, 5, kp, 0.1);
                    angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
                    angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);
                } else {
                    angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, 2.6, 0.1);
                    angles = get_leg_angles(gait, 2 ,currentTimeSec, ctrl1, 6, 5, 2.6, 0.1);
                    angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, 2.6, 0.1);
                    angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, 2.6, 0.1);
                }
            } else {
                // 纯轮模式下前进后退时，保持腿部处于归零后的站立姿态
                if (is_zeroing) {
                    state_zero(now, kp);
                } else if (is_adjusting_pose) {
                    update_pose_adjustment(now, kp);
                }else if (is_custom_interp) {
                	state_custom_interpolation(now, kp, custom_target_x_grp1, custom_target_y_grp1,custom_target_x_grp2, custom_target_y_grp2);
                }
            }
            float linear_ratio = 0.0f; // 前后比例：前进为正，后退为负
            if (myPacket.joy_lx < 40) {
                            linear_ratio = (40.0f - myPacket.joy_lx) / 40.0f; // 摇杆越靠近 0，比例越接近 1.0
              } else if (myPacket.joy_lx > 60) {
                            linear_ratio = (60.0f - myPacket.joy_lx) / 40.0f; // 摇杆越靠近 100，比例越接近 -1.0
              }

            float turn_ratio = 0.0f; // 转向比例：右转为正，左转为负
            if (myPacket.joy_rx < 40) {
                            turn_ratio = (40.0f - myPacket.joy_rx) / 40.0f;
               } else if (myPacket.joy_rx > 60) {
                            turn_ratio = (60.0f - myPacket.joy_rx) / 40.0f;
                 }
            float max_base_rpm = is_pure_wheel_mode ? 2400.0f : 1800.0f;
            float max_turn_rpm = is_pure_wheel_mode ? 1800.0f : 1500.0f; // 注意这里用正数表示转向幅度
            float wheel_target_rpm_01 = (linear_ratio * max_base_rpm) + (turn_ratio * max_turn_rpm);
            float wheel_target_rpm_23 = -(linear_ratio * max_base_rpm) + (turn_ratio * max_turn_rpm);

            // ==========================================
            // 【修改点3】：区分速度控制（联动/纯轮）与位置控制（纯足锁死）
            if (chassis_mode == 2) {
                // 【纯足模式】：锁死3508轮子在原位
                for(int i = 0; i < 4; i++) {
                    Motors[i].target_angle = locked_angles[i];
                    PID_Calc_Position(i, Motors[i].target_angle);
                }
            } else {
                // 【联动或纯轮模式】：正常驱动轮子
                for (int i = 0; i < 2; i++) {
                    Motors[i].target_speed = wheel_target_rpm_01;
                    PID_Calc_Speed(i);
                }
                for (int i = 2; i < 4; i++) {
                    Motors[i].target_speed = wheel_target_rpm_23;
                    PID_Calc_Speed(i);
                }
            }
            // ==========================================

        } else {
            if (is_zeroing) {
                state_zero(now, kp);
            } else if (is_adjusting_pose) {
                update_pose_adjustment(now, kp);
            }else if (is_custom_interp) {
                state_custom_interpolation(now, kp,
                                           custom_target_x_grp1, custom_target_y_grp1,
                                           custom_target_x_grp2, custom_target_y_grp2);
            }
            // ==========================================
            // 【修改点4】：IDLE状态下，判断是否需要锁死
            if (chassis_mode == 2) {
                // 【纯足模式】：保持原位锁死
                for(int i = 0; i < 4; i++) {
                    Motors[i].target_angle = locked_angles[i];
                    PID_Calc_Position(i, Motors[i].target_angle);
                }
            } else {
                // 【联动或纯轮模式】：速度归零
                for (int i = 0; i < 4; i++) {
                    Motors[i].target_speed = 0.0f;
                    PID_Calc_Speed(i);
                }
            }
            // ==========================================
        }
    }
}
/**
 * @brief 按键控制逻辑
 */
#define DEBOUNCE_DELAY_MS 30
#define NUM_BUTTONS 8

void button_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t currentTime) {
    static uint8_t stable_button_states[NUM_BUTTONS] = {0};
    static uint8_t last_button_readings[NUM_BUTTONS] = {0};
    static uint32_t last_debounce_times[NUM_BUTTONS] = {0};

    float dy = 1.0f;
    float dx = 0.0f;

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
//                          state_zero0(currentTime);
                            break;

                        case 1:
                            // 升高底盘 (Y+dy, X-dx) - 触发微调插值
                            start_pose_adjustment(currentTime, dx, dy);
                            break;

                        case 2:
                            // 降低底盘 (Y-dy, X-dx) - 触发微调插值
                            start_pose_adjustment(currentTime, -dx, -dy);
                            break;

                        case 3:
                            // 【修改点5】：每次按下，在 0(联动), 1(纯轮), 2(纯足) 之间循环切换
                            chassis_mode = (chassis_mode + 1) % 3;

                            if (chassis_mode == 1) {
                                // 进入纯轮模式：腿部强制插值归零
                                start_zero_interpolation(currentTime);
                            } else if (chassis_mode == 2) {
                                // 进入纯足模式瞬间，记录当前4个轮子的实时角度
                                for (int j = 0; j < 4; j++) {
                                    locked_angles[j] = Motors[j].total_angle;
                                }
                                current_gait_mode = GAIT_MODE_TROT;
                                force_gait_update = 1;
                            } else {
                                // 恢复轮腿联动
                                current_gait_mode = GAIT_MODE_TROT;
                                force_gait_update = 1;
                            }
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
                        case 6:
                        	custom_target_x_grp1 = 21.5f;
                        	custom_target_y_grp1 = 18.5f;
                        	custom_target_x_grp2 = -21.5f;
                        	custom_target_y_grp2 = 18.5f;
                        	start_custom_interpolation(currentTime, 800);
                        	// 单次执行 BOUND
//                          if (!is_single_action) {
//                              previous_gait_mode = current_gait_mode;
//                              current_gait_mode = GAIT_MODE_BOUND;
//                              apply_current_gait(gait);
//                              calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
//                              calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
//                              calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
//                              calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
//                              start_quadruped_gait(gait, currentTime / 1000.0f);
//
//                              is_single_action = 1;
//                              single_action_end_time = currentTime + 600;
//                          }
                            break;

                        case 7:
//                        	custom_target_x_grp1 = -21.5f;
//                        	custom_target_y_grp1 = 18.5f;
//                        	custom_target_x_grp2 = 21.5f;
//                        	custom_target_y_grp2 = 18.5f;
//                        	start_custom_interpolation(currentTime, 800);
                        	// 单次执行 PRONK
//                          if (!is_single_action) {
//                              previous_gait_mode = current_gait_mode;
//                              current_gait_mode = GAIT_MODE_PRONK;
//                              apply_current_gait(gait);
//
//                              calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
//                              calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
//                              calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
//                              calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
//                              start_quadruped_gait(gait, currentTime / 1000.0f);
//
//                              is_single_action = 1;
//                              single_action_end_time = currentTime + 600;
//                          }
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
#define REMOTE_TIMEOUT_MS 300 // 定义通信超时时间为 300 毫秒（可根据实际发包频率调整）

/**
 * @brief 接收数据并分发控制 (包含超时失联保护)
 */
void AS01_rx(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
    static uint32_t last_rx_time = 0; // 静态变量：记录上一次成功接收到数据的时间戳

    // 1. 尝试接收数据
    if (NRF24L01_RxPacket(rx_buffer) == 0) {
        memcpy(&myPacket, rx_buffer, sizeof(RemoteData_t));
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        last_rx_time = now; // 成功接收，更新时间戳
    }
    if ((now - last_rx_time) > REMOTE_TIMEOUT_MS) {
        // 强制将摇杆数据归至中位 (触发 ROBOT_STATE_IDLE)
        myPacket.joy_lx = 50;
        myPacket.joy_rx = 50;
        myPacket.joy_ly = 50;
        myPacket.joy_ry = 50;
        memset(myPacket.button, 0, sizeof(myPacket.button));
    } else {
    }

    // 3. 执行控制逻辑
    joystick_control(ctrl1, ctrl2, gait, startTime, angles, now);
    button_control(ctrl1, ctrl2, gait, now);
}






//#include "control.h"
//#include <string.h> // 确保 memcpy 可用
//#include <math.h>
//#include "3508_driver.h"
//
//extern RemoteData_t myPacket;
//extern uint8_t rx_buffer[32];
//uint8_t as01_status = 0;
//extern uint8_t rx_data[32];
//extern __attribute__((section(".RAM_D1")))__attribute__((section(".RAM_D1")))MotorController ctrl2;
//extern MotorController ctrl1;
//static float kp = 2.6f;
//static GaitMode_e current_gait_mode = GAIT_MODE_TROT;
//static uint8_t force_gait_update = 0;
//static uint8_t is_single_action = 0;         // 单次动作标志位
//static uint32_t single_action_end_time = 0;  // 单次动作结束时间
//static GaitMode_e previous_gait_mode = GAIT_MODE_TROT; // 记录执行动作前的步态
//static RobotState_e current_state = ROBOT_STATE_IDLE;
//static uint8_t is_zeroing = 0;               // 标志位：是否正在执行复位插值
//static uint32_t zeroing_start_time = 0;      // 记录复位动作开始的时间戳
//static Currentpos zero_start_pos[4];         // 记录复位动作开始瞬间，4条腿的初始坐标
//static const float ZERO_TARGET_X = 10.0f;     // 零点目标 X 坐标
//static const float ZERO_TARGET_Y = 25.0f;    // 零点目标 Y 坐标
//static const uint32_t ZEROING_DURATION_MS = 2500; // 复位动作耗时 (1000ms = 1秒)
//static uint32_t current_zeroing_duration = 2500; // 新增：用于记录当前这次动作需要的总时长
//static const uint32_t ZEROING_last_ms = 1000; // 复位动作耗时 (1000ms = 1秒)
//static uint8_t is_adjusting_pose = 0;               // 标志位：是否正在执行高度微调
//static uint32_t pose_adjust_start_time = 0;         // 记录微调动作开始的时间戳
//static Currentpos pose_start_pos[4];                // 微调的起点
//static Currentpos pose_target_pos[4];               // 微调的终点
//static const uint32_t POSE_ADJUST_DURATION_MS = 300; // 微调耗时 (300ms)，让动作迅捷且平滑
//static uint8_t is_pure_wheel_mode = 0;       // 【新增】纯轮模式标志位：0-轮腿联动步态，1-纯轮模式
//static uint8_t is_first_zero = 0;
//static inline void apply_current_gait(QuadrupedGait* gait) {
//    float straight_period = 1.2f; // 周期 (秒)
//    float straight_length = 8.5f; // 步长
//    float straight_height = 10.0f; // 抬腿高度
//    float turn_period = 0.8f;     // 转向周期 (通常更快一点)
//    float turn_length = 3.0f;     // 转向步长 (建议改小，以轮子差速为主)
//    float turn_height = 5.0f;     // 转向抬腿高度 (建议改高，防止腿绊住地面)
//
//    // 根据当前状态选择应用的参数
//    float p = straight_period;
//    float l = straight_length;
//    float h = straight_height;
//
//    if (current_state == ROBOT_STATE_LEFT || current_state == ROBOT_STATE_RIGHT) {
//        p = turn_period;
//        l = turn_length;
//        h = turn_height;
//    }
//
//    switch(current_gait_mode){
//        case GAIT_MODE_WALK:
//            init_quadruped_gait_walk(gait, 1.2f, l, h + 2.0f); // 举例：Walk也可以应用独立参数
//            break;
//        case GAIT_MODE_TROT:
//            init_quadruped_gait_trot(gait, p, l, h);
//            break;
//        case GAIT_MODE_BOUND:
////          init_quadruped_gait_bound(gait, 0.6f, 10.849f, 25.0f);
//            break;
//        case GAIT_MODE_PRONK:
////          init_quadruped_gait_pronk(gait, 0.6f, 10.849f, 25.0f);
//            break;
//    }
//}
//
///**
// * @brief 启动零位插值：捕获当前位置并锁定状态
// */
//void start_zero_interpolation(uint32_t now) {
//    if (is_zeroing) return; // 正在插值中则忽略
//    is_zeroing = 1;
//    is_adjusting_pose = 0;  // 【重要】强制打断微调状态
//    if (is_first_zero == 0) {
//            current_zeroing_duration = ZEROING_DURATION_MS; // 2500ms
//            is_first_zero = 1; // 标记以后不再是第一次了
//        } else {
//            current_zeroing_duration = ZEROING_last_ms;    // 1000ms
//        }
//    zeroing_start_time = now;
//    zero_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
//    zero_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
//    zero_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
//    zero_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);
//}
//
///**
// * @brief 零位插值执行：在 5ms 周期中平滑逼近坐标
// */
//void state_zero(uint32_t now, float current_kp) {
//    if (!is_zeroing) return;
//
//    float current_target_x[4];
//    float current_target_y[4];
//    float progress = (float)(now - zeroing_start_time) / (float)current_zeroing_duration;
//    if (progress >= 1.0f) {
//        progress = 1.0f;
//        is_zeroing = 0; // 进度到达 100%，结束插值
//    }
//    current_target_x[0] = zero_start_pos[0].X + (8 - zero_start_pos[0].X) * progress;
//    current_target_y[0] = zero_start_pos[0].Y + (25 - zero_start_pos[0].Y) * progress;
//    for (int i = 1; i < 4; i++) {
//        current_target_x[i] = zero_start_pos[i].X + (ZERO_TARGET_X - zero_start_pos[i].X) * progress;
//        current_target_y[i] = zero_start_pos[i].Y + (ZERO_TARGET_Y - zero_start_pos[i].Y) * progress;
//    }
////    for (int i = 0; i < 4; i++) {
////        current_target_x[i] = zero_start_pos[i].X + (ZERO_TARGET_X - zero_start_pos[i].X) * progress;
////        current_target_y[i] = zero_start_pos[i].Y + (ZERO_TARGET_Y - zero_start_pos[i].Y) * progress;
////    }
//    LegAngles angles1 = InverseKinematics(current_target_x[0], current_target_y[0], &ctrl2, 1, 2, 0.3, 0.1);
//    MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles1.theta2, current_kp, 0.1);
//    MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles1.theta1, current_kp, 0.1);
//
//    LegAngles angles2 = InverseKinematics(current_target_x[1], current_target_y[1], &ctrl2, 7, 8, 0.3, 0.1);
//    MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles2.theta2, current_kp, 0.1);
//    MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles2.theta1, current_kp, 0.1);
//
//    LegAngles angles3 = InverseKinematics(current_target_x[2], current_target_y[2], &ctrl1, 6, 5, 0.3, 0.1);
//    MotorController_SetCommand(&ctrl1, 5, 1, 0.0, 0.0f, angles3.theta2, current_kp, 0.1);
//    MotorController_SetCommand(&ctrl1, 6, 1, 0.0, 0.0f, angles3.theta1, current_kp, 0.1);
//
//    LegAngles angles4 = InverseKinematics(current_target_x[3], current_target_y[3], &ctrl1, 3, 4, 0.3, 0.1);
//    MotorController_SetCommand(&ctrl1, 4, 1, 0.0, 0.0f, angles4.theta2, current_kp, 0.1);
//    MotorController_SetCommand(&ctrl1, 3, 1, 0.0, 0.0f, angles4.theta1, current_kp, 0.1);
//}
//void start_pose_adjustment(uint32_t now, float delta_x, float delta_y) {
//    if (is_adjusting_pose) return;
//    is_adjusting_pose = 1;
//    is_zeroing = 0;        // 【重要】强制打断归零插值状态
//    pose_adjust_start_time = now;
//    pose_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
//    pose_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
//    pose_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
//    pose_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);
//
//    // 计算终点目标位置
//    for (int i = 0; i < 4; i++) {
//        pose_target_pos[i].X = pose_start_pos[i].X + delta_x;
//        pose_target_pos[i].Y = pose_start_pos[i].Y + delta_y;
//    }
//}
//
///**
// * @brief 姿态微调执行：在 5ms 周期中平滑逼近微调后的坐标
// */
//void update_pose_adjustment(uint32_t now, float current_kp) {
//    if (!is_adjusting_pose) return;
//
//    float progress = (float)(now - pose_adjust_start_time) / (float)POSE_ADJUST_DURATION_MS;
//    if (progress >= 1.0f) {
//        progress = 1.0f;
//        is_adjusting_pose = 0;
//    }
//
//    float current_target_x[4];
//    float current_target_y[4];
//
//    for (int i = 0; i < 4; i++) {
//        current_target_x[i] = pose_start_pos[i].X + (pose_target_pos[i].X - pose_start_pos[i].X) * progress;
//        current_target_y[i] = pose_start_pos[i].Y + (pose_target_pos[i].Y - pose_start_pos[i].Y) * progress;
//    }
//
//    LegAngles angles1 = InverseKinematics(current_target_x[0], current_target_y[0], &ctrl2, 1, 2, 0.5, 0.1);
//    MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles1.theta2, current_kp, 0.1);
//    MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles1.theta1, current_kp, 0.1);
//
//    LegAngles angles2 = InverseKinematics(current_target_x[1], current_target_y[1], &ctrl2, 7, 8, 0.5, 0.1);
//    MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles2.theta2, current_kp, 0.1);
//    MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles2.theta1, current_kp, 0.1);
//
//    LegAngles angles3 = InverseKinematics(current_target_x[2], current_target_y[2], &ctrl1, 3, 4, 0.5, 0.1);
//    MotorController_SetCommand(&ctrl1, 4, 1, 0.0, 0.0f, angles3.theta2, current_kp, 0.1);
//    MotorController_SetCommand(&ctrl1, 3, 1, 0.0, 0.0f, angles3.theta1, current_kp, 0.1);
//
//    LegAngles angles4 = InverseKinematics(current_target_x[3], current_target_y[3], &ctrl1, 6, 5, 0.5, 0.1);
//    MotorController_SetCommand(&ctrl1, 5, 1, 0.0, 0.0f, angles4.theta2, current_kp, 0.1);
//    MotorController_SetCommand(&ctrl1, 6, 1, 0.0, 0.0f, angles4.theta1, current_kp, 0.1);
//}
///**
// * @brief 摇杆控制处理
// */
//void joystick_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
//    RobotState_e target_state = ROBOT_STATE_IDLE;
//    float currentTimeSec = now / 1000.0f;
//    static uint32_t last_calc_time = 0;
//
//    if (is_single_action) {
//        if (now >= single_action_end_time) {
//            is_single_action = 0;
//            current_gait_mode = previous_gait_mode;
//            apply_current_gait(gait);
//            current_state = ROBOT_STATE_IDLE;
//            start_zero_interpolation(now);          // 触发跳跃后的平滑复位
//        } else {
//            if (now - last_calc_time >= 5) {
//                last_calc_time = now;
//                angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
//                angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, kp, 0.1);
//                angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
//                angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);
//
//                // 单次动作时保持 3508 轮子停止
//                for (int i = 0; i < 4; i++) {
//                    Motors[i].target_speed = 0.0f;
//                    PID_Calc_Speed(i);
//                }
//            }
//        }
//        return;
//    }
//
//    if (myPacket.joy_lx > 60) {
//        target_state = ROBOT_STATE_BACKWARD;
//    } else if (myPacket.joy_lx < 40) {
//        target_state = ROBOT_STATE_FORWARD;
//    } else if (myPacket.joy_rx > 60) {
//        target_state = ROBOT_STATE_LEFT;
//    } else if (myPacket.joy_rx < 40) {
//        target_state = ROBOT_STATE_RIGHT;
//    } else {
//        target_state = ROBOT_STATE_IDLE;
//    }
//
//    // 状态切换检测
//    if (target_state != current_state) {
//        current_state = target_state;
//        force_gait_update = 0;
//        switch (current_state) {
//            case ROBOT_STATE_FORWARD:
//                if (!is_pure_wheel_mode) {
//                    apply_current_gait(gait);
//                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
//                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
//                    calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
//                    calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
//                    start_quadruped_gait(gait, currentTimeSec);
//                }
//                break;
//            case ROBOT_STATE_BACKWARD:
//                if (!is_pure_wheel_mode) {
//                    apply_current_gait(gait);
//                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
//                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
//                    calibrate_leg_base_position(gait, 2, ctrl1,6,5,0);
//                    calibrate_leg_base_position(gait, 3, ctrl1,3,4,0);
//                    start_quadruped_gait(gait, currentTimeSec);
//                }
//                break;
//            case ROBOT_STATE_LEFT:
//                if (!is_pure_wheel_mode) {
//                    apply_current_gait(gait);
//                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
//                    calibrate_leg_base_position(gait, 2, ctrl1,3,4,1);
//                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
//                    calibrate_leg_base_position(gait, 3, ctrl1,6,5,0);
//                    start_quadruped_gait(gait, currentTimeSec);
//                }
//                break;
//            case ROBOT_STATE_RIGHT:
//                if (!is_pure_wheel_mode) {
//                    apply_current_gait(gait);
//                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
//                    calibrate_leg_base_position(gait, 2, ctrl1,3,4,0);
//                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
//                    calibrate_leg_base_position(gait, 3, ctrl1,6,5,1);
//                    start_quadruped_gait(gait, currentTimeSec);
//                }
//                break;
//            case ROBOT_STATE_IDLE:
//                start_zero_interpolation(now);
//                break;
//        }
//    }
//
//    if (now - last_calc_time >= 5) {
//        last_calc_time = now;
//        if (current_state != ROBOT_STATE_IDLE) {
//
//            // 【修改点 2】：区分纯轮模式与正常步态模式的腿部运算
//            if (!is_pure_wheel_mode) {
//                // 修复了语法错误：必须两边都带 current_state ==
//                if (current_state == ROBOT_STATE_FORWARD || current_state == ROBOT_STATE_BACKWARD) {
//                    angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
//                    angles = get_leg_angles(gait, 2 ,currentTimeSec, ctrl1, 6, 5, kp, 0.1);
//                    angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
//                    angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);
//                } else {
//                    angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, 2.6, 0.1);
//                    angles = get_leg_angles(gait, 2 ,currentTimeSec, ctrl1, 6, 5, 2.6, 0.1);
//                    angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, 2.6, 0.1);
//                    angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, 2.6, 0.1);
//                }
//            } else {
//                // 纯轮模式下前进后退时，保持腿部处于归零后的站立姿态
//                if (is_zeroing) {
//                    state_zero(now, kp);
//                } else if (is_adjusting_pose) {
//                    update_pose_adjustment(now, kp);
//                }
//            }
//           float linear_ratio = 0.0f; // 前后比例：前进为正，后退为负
//           if (myPacket.joy_lx < 40) {
//                            linear_ratio = (40.0f - myPacket.joy_lx) / 40.0f; // 摇杆越靠近 0，比例越接近 1.0
//              } else if (myPacket.joy_lx > 60) {
//                            linear_ratio = (60.0f - myPacket.joy_lx) / 40.0f; // 摇杆越靠近 100，比例越接近 -1.0
//              }
//
//           float turn_ratio = 0.0f; // 转向比例：右转为正，左转为负
//           if (myPacket.joy_rx < 40) {
//                            turn_ratio = (40.0f - myPacket.joy_rx) / 40.0f;
//               } else if (myPacket.joy_rx > 60) {
//                            turn_ratio = (60.0f - myPacket.joy_rx) / 40.0f;
//                 }
//            float max_base_rpm = is_pure_wheel_mode ? 2400.0f : 1800.0f;
//            float max_turn_rpm = is_pure_wheel_mode ? 1800.0f : 1500.0f; // 注意这里用正数表示转向幅度
//            float wheel_target_rpm_01 = (linear_ratio * max_base_rpm) + (turn_ratio * max_turn_rpm);
//            float wheel_target_rpm_23 = -(linear_ratio * max_base_rpm) + (turn_ratio * max_turn_rpm);
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
///**
// * @brief 按键控制逻辑
// */
//#define DEBOUNCE_DELAY_MS 30
//#define NUM_BUTTONS 8
//
//void button_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t currentTime) {
//    static uint8_t stable_button_states[NUM_BUTTONS] = {0};
//    static uint8_t last_button_readings[NUM_BUTTONS] = {0};
//    static uint32_t last_debounce_times[NUM_BUTTONS] = {0};
//
//	float dy = 1.0f;
//    float dx = 0.0f;
//
//    // 消抖与按键处理
//    for (int i = 0; i < NUM_BUTTONS; i++) {
//        uint8_t current_reading = myPacket.button[i];
//
//        if (current_reading != last_button_readings[i]) {
//            last_debounce_times[i] = currentTime;
//        }
//
//        if ((currentTime - last_debounce_times[i]) > DEBOUNCE_DELAY_MS) {
//            if (current_reading != stable_button_states[i]) {
//                stable_button_states[i] = current_reading;
//                if (stable_button_states[i] == 1) {
//                    switch (i) {
//                        case 0:
//                            // 归零
//                            start_zero_interpolation(currentTime);
////                            state_zero0(currentTime);
//                            break;
//
//                        case 1:
//                            // 升高底盘 (Y+dy, X-dx) - 触发微调插值
//                            start_pose_adjustment(currentTime, dx, dy);
//                            break;
//
//                        case 2:
//                            // 降低底盘 (Y-dy, X-dx) - 触发微调插值
//                            start_pose_adjustment(currentTime, -dx, -dy);
//                            break;
//
//                        case 3:
//                            is_pure_wheel_mode = !is_pure_wheel_mode;
//                            if (is_pure_wheel_mode) {
//                            start_zero_interpolation(currentTime);
//                            } else {
//                             current_gait_mode = GAIT_MODE_TROT;
//                             force_gait_update = 1;
//                             }
//                            break;
//
//                        case 4:
//                            if (current_gait_mode != GAIT_MODE_TROT) {
//                                current_gait_mode = GAIT_MODE_TROT;
//                                force_gait_update = 1;
//                            }
//
//                            break;
//
//                        case 5:
//                        	if (current_gait_mode != GAIT_MODE_WALK) {
//                                current_gait_mode = GAIT_MODE_WALK;
//                                force_gait_update = 1;
//                            }
//
//                            break;
//                        case 6: // 单次执行 BOUND
////                        	if (!is_single_action) {
////                                previous_gait_mode = current_gait_mode;
////                                current_gait_mode = GAIT_MODE_BOUND;
////                                apply_current_gait(gait);
////                                calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
////                                calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
////                                calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
////                                calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
////                                start_quadruped_gait(gait, currentTime / 1000.0f);
////
////                                is_single_action = 1;
////                                single_action_end_time = currentTime + 600;
////                        	}
//                            break;
//
//                        case 7: // 单次执行 PRONK
////                        	if (!is_single_action) {
////                                previous_gait_mode = current_gait_mode;
////                        	    current_gait_mode = GAIT_MODE_PRONK;
////                                apply_current_gait(gait);
////
////                                calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
////                                calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
////                                calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
////                                calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
////                                start_quadruped_gait(gait, currentTime / 1000.0f);
////
////                                is_single_action = 1;
////                                single_action_end_time = currentTime + 600;
////                        	}
//                            break;
//
//                        default:
//                            break;
//                    }
//                }
//            }
//        }
//        last_button_readings[i] = current_reading;
//    }
//}
//#define REMOTE_TIMEOUT_MS 300 // 定义通信超时时间为 300 毫秒（可根据实际发包频率调整）
//
///**
// * @brief 接收数据并分发控制 (包含超时失联保护)
// */
//void AS01_rx(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
//    static uint32_t last_rx_time = 0; // 静态变量：记录上一次成功接收到数据的时间戳
//
//    // 1. 尝试接收数据
//    if (NRF24L01_RxPacket(rx_buffer) == 0) {
//        memcpy(&myPacket, rx_buffer, sizeof(RemoteData_t));
//        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//        last_rx_time = now; // 成功接收，更新时间戳
//    }
//    if ((now - last_rx_time) > REMOTE_TIMEOUT_MS) {
//        // 强制将摇杆数据归至中位 (触发 ROBOT_STATE_IDLE)
//        myPacket.joy_lx = 50;
//        myPacket.joy_rx = 50;
//        myPacket.joy_ly = 50;
//        myPacket.joy_ry = 50;
//        memset(myPacket.button, 0, sizeof(myPacket.button));
//    } else {
//    }
//
//    // 3. 执行控制逻辑
//    joystick_control(ctrl1, ctrl2, gait, startTime, angles, now);
//    button_control(ctrl1, ctrl2, gait, now);
//}
