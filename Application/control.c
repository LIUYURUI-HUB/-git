#include "control.h"
#include <math.h>
#include <string.h>

#include "protocol_handler.h"
#include "3508_driver.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==========================================
// 模块声明与外部依赖
// ==========================================
static void reset_wheel_pid_state(int motor_id);
void Arm_Calc_Gravity_Torque(float q2, float q3, float* tau2, float* tau3);

extern RemoteData_t myPacket;           // 遥控器接收数据包
extern uint8_t rx_buffer[32];           // 原始接收缓冲区
uint8_t as01_status = 0;                // AS01 无线模块状态
extern uint8_t rx_data[32];
extern __attribute__((section(".RAM_D1"))) MotorController ctrl2; // 腿部无刷电机控制器 2
extern MotorController ctrl1;                                     // 腿部无刷电机控制器 1
volatile RobotDiag_t g_robot_diag = {0};

// ==========================================
// 全局状态与控制变量
// ==========================================
static float kp = 2.6f;                                      // 腿部位置环基础 KP
static GaitMode_e current_gait_mode = GAIT_MODE_TROT;        // 当前步态模式 (默认 Trot 小跑)
static uint8_t force_gait_update = 0;                        // 强制刷新步态标志位
static uint8_t is_single_action = 0;                         // 单次动作标志位 (如跳跃等非周期动作)
static uint32_t single_action_end_time = 0;                  // 单次动作结束时间戳
static GaitMode_e previous_gait_mode = GAIT_MODE_TROT;       // 记录执行单次动作前的步态，用于动作结束后恢复
static RobotState_e current_state = ROBOT_STATE_IDLE;        // 当前机器人运动方向状态 (前后左右/待机)
SystemCtrlMode_e current_sys_mode = SYS_MODE_RC;             // 当前系统主控权 (遥控 RC 或 视觉 Vision)

// 统一指令结构体：将底层逻辑与输入源解耦
static ChassisCmd_t cmd_rc = {0};       // 存放解析后的遥控器指令
static ChassisCmd_t cmd_vision = {0};   // 存放解析后的视觉/上位机指令
static ChassisCmd_t cmd_target = {0};   // 最终被选中的执行指令 (由仲裁函数决定)

// ==========================================
// 归零与插值(动作平滑过渡)相关的变量
// ==========================================
static uint8_t is_zeroing = 0;               // 标志位：是否正在执行复位插值
static uint32_t zeroing_start_time = 0;      // 记录复位动作开始的时间戳
static Currentpos zero_start_pos[4];         // 记录复位动作开始瞬间，4条腿的初始足端坐标
static const float ZERO_TARGET_X = 10.0f;    // 标准零点/待机状态的 X 坐标
static const float ZERO_TARGET_Y = 25.0f;    // 标准零点/待机状态的 Y 坐标 (腿的高度)
static const uint32_t ZEROING_DURATION_MS = 2500; // 首次复位动作耗时 (2.5秒，足够慢以策安全)
static uint32_t current_zeroing_duration = 2500;  // 当前这次插值动作分配的总时长
static const uint32_t ZEROING_last_ms = 1000;     // 后续常规复位动作耗时 (1秒，稍快些)
static uint8_t is_first_zero = 0;                 // 标记是否是开机以来的第一次归零

// 姿态微调相关的插值变量 (上下左右平移车体)
static uint8_t is_adjusting_pose = 0;               // 标志位：是否正在执行姿态微调
static uint32_t pose_adjust_start_time = 0;
static Currentpos pose_start_pos[4];                // 微调起点
static Currentpos pose_target_pos[4];               // 微调终点
static const uint32_t POSE_ADJUST_DURATION_MS = 300; // 微调耗时仅需 300ms，要求手感迅捷平滑

// 自定义分组插值变量 (常用于做特定姿势，两组腿去往不同坐标)
static uint8_t is_custom_interp = 0;
static uint32_t custom_interp_start_time = 0;
static Currentpos custom_start_pos[4];
static uint32_t custom_interp_duration = 1000;
static float custom_target_x_grp1 = 0.0f; // 分组1(对角腿或前腿) 目标 X
static float custom_target_y_grp1 = 0.0f;
static float custom_target_x_grp2 = 0.0f; // 分组2 目标 X
static float custom_target_y_grp2 = 0.0f;

// ==========================================
// 底盘模式与运动参数限制配置
// ==========================================
// 底盘模式：0-轮腿联动(腿走轮转)，1-纯轮模式(腿站立，轮转)，2-纯足模式(轮子锁死，腿走)
static uint8_t chassis_mode = 2; // 默认修改为 2，纯足模式
#define is_pure_wheel_mode (chassis_mode == 1) // 兼容性宏定义

static int64_t locked_angle[4] = {0}; // 用于在纯足模式或待机状态下，记录 4 个轮子的驻车目标角度
static uint8_t locked_angle_ready = 0; // 3508反馈到齐后才允许用locked_angle做位置锁定

// 遥控器与底盘常量
static const float RC_JOYSTICK_CENTER = 50.0f;            // 摇杆物理中位
static const float RC_JOYSTICK_MOVE_THRESHOLD = 10.0f;    // 摇杆运动死区 (50±10 内算作中位，防误触)
static const float RC_OVERRIDE_THRESHOLD = 18.0f;         // 视觉模式下，摇杆拨动超过 18 时抢夺控制权
static const uint32_t REMOTE_TIMEOUT_MS = 300U;           // 遥控器失联超时时间 (超过 300ms 强制刹车)

// 底盘速度配置
static const float VISION_STATE_EPSILON = 0.05f;          // 视觉下发速度的极小值(死区)
static const float VISION_MAX_LINEAR_RPM = 1000.0f;       // 视觉模式：最大前进/后退轮转速
static const float VISION_MAX_TURN_RPM = 800.0f;          // 视觉模式：最大自转轮转速
static const float RC_MAX_BASE_RPM = 1800.0f;             // 遥控模式：联动时最大直线轮转速
static const float RC_MAX_TURN_RPM = 1500.0f;             // 遥控模式：联动时最大转向轮转速

// 物理学参数 (里程计解算用)
static const float CHASSIS_WHEEL_RADIUS_M = 0.03f;        // 轮子半径 3cm
static const float CHASSIS_WHEEL_REDUCTION_RATIO = 19.0f; // 3508 电机减速比 1:19
static const float CHASSIS_TURN_RADIUS_M = 0.20f;         // 底盘旋转半径 20cm
static const uint32_t CHASSIS_FEEDBACK_PERIOD_MS = 20U;   // 向外发送底盘里程计反馈的周期
static const float WHEEL_DIR_SIGN[4] = {-1.0f, 1.0f, 1.0f, -1.0f}; // 3508电机1、4安装方向相反，速度模式下统一反号

// 内部函数前置声明
static float clamp_unit(float value);
static float command_magnitude(float value);
static float wheel_rpm_to_linear_mps(float motor_rpm);
static void estimate_chassis_velocity(float* est_vx, float* est_wz);
static void send_chassis_feedback(uint32_t now);
static void clear_chassis_cmd(ChassisCmd_t* cmd);
static RobotState_e normalize_robot_state(uint8_t state);
static RobotState_e derive_robot_state(float vx, float wz);
static RobotState_e derive_rc_state(void);
static float compute_rc_linear_ratio(void);
static float compute_rc_turn_ratio(void);
static void update_rc_command(void);
static void update_vision_command(void);
static uint8_t rc_override_requested(void);
static void update_control_mode(void);
static void execute_chassis_command(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, LegAngles angles, uint32_t now);
static uint8_t sync_locked_angle_from_feedback(void);
static void hold_wheels_at_locked_angle(void);
static void set_wheel_speed_target(int motor_id, float chassis_target_rpm);

// ==========================================
// 步态参数配置器
// ==========================================
/**
 * @brief 根据当前运动状态和所选的步态类型，向步态生成器(gait)下发具体参数
 */
static inline void apply_current_gait(QuadrupedGait* gait) {
    float straight_period = 1.2f;  // 周期 (秒)
    float straight_length = 2.5f;  // 步长
    float straight_height = 4.0f; // 抬腿高度

    // 转向时的步态参数往往要求更短、更快、稍高以防绊脚
    float turn_period = 0.8f;
    float turn_length = 3.0f;
    float turn_height = 5.0f;

    float p = straight_period;
    float l = straight_length;
    float h = straight_height;

    // 动态应用状态参数
    if (current_state == ROBOT_STATE_LEFT || current_state == ROBOT_STATE_RIGHT) {
        p = turn_period;
        l = turn_length;
        h = turn_height;
    }

    switch(current_gait_mode){
        case GAIT_MODE_WALK:
            init_quadruped_gait_walk(gait, 1.2f, l, h + 2.0f); // WALK 动作更稳健，抬腿额外高 2.0
            break;
        case GAIT_MODE_TROT:
            init_quadruped_gait_trot(gait, p, l, h);
            break;
        case GAIT_MODE_BOUND:
            // 预留的 Bound 步态
            break;
        case GAIT_MODE_PRONK:
            // 预留的 Pronk 步态
            break;
    }
}

// ---------------------- 运算与通用支持函数 ----------------------
/**
 * @brief 将输出值限制在 [0.0, 1.0] 范围内，防飞车
 */
static float clamp_unit(float value) {
    if (value > 1.0f) return 1.0f;
    if (value < 0.0f) return 0.0f;
    return value;
}

/**
 * @brief 彻底复位轮毂电机(3508)的 PID 累计状态，防止状态切换瞬间由于积分饱和导致电机猛窜
 */
static void reset_wheel_pid_state(int motor_id) {
    Motors[motor_id].speed_err_sum = 0.0f;
    Motors[motor_id].position_err_sum = 0.0f;
    Motors[motor_id].torque_err_sum = 0.0f;
    Motors[motor_id].err_last = 0.0f;
    Motors[motor_id].target_speed = 0.0f;
    Motors[motor_id].target_angle = Motors[motor_id].total_angle; // 当前位置即目标位置，防抖
    Motors[motor_id].Out_Current = 0;
}

/**
 * @brief 纯足模式锁轮前，把锁定角同步到3508真实反馈位置。
 *        上电后编码器初值不是0，如果locked_angle仍为0会把轮子强行拉向0点导致抖动。
 */
static uint8_t sync_locked_angle_from_feedback(void) {
    if (locked_angle_ready) return 1U;

    for (int i = 0; i < 4; i++) {
        if (Motors[i].init_flag == 0U) {
            return 0U;
        }
    }

    for (int i = 0; i < 4; i++) {
        locked_angle[i] = Motors[i].total_angle;
        Motors[i].target_angle = locked_angle[i];
        Clear_Motor_PID(i);
    }
    locked_angle_ready = 1U;
    return 1U;
}

/**
 * @brief 纯足模式下锁住四个3508轮子；反馈未到齐前不输出位置环电流。
 */
static void hold_wheels_at_locked_angle(void) {
    if (!sync_locked_angle_from_feedback()) {
        for (int i = 0; i < 4; i++) {
            Motors[i].target_speed = 0.0f;
            Motors[i].Out_Current = 0;
        }
        return;
    }

    for (int i = 0; i < 4; i++) {
        Motors[i].target_angle = locked_angle[i];
        PID_Calc_Position(i, Motors[i].target_angle);
    }
}

/**
 * @brief 轮腿/纯轮速度模式下统一处理3508安装方向。
 */
static void set_wheel_speed_target(int motor_id, float chassis_target_rpm) {
    if (motor_id < 0 || motor_id >= 4) return;
    Motors[motor_id].target_speed = chassis_target_rpm * WHEEL_DIR_SIGN[motor_id];
    PID_Calc_Speed(motor_id);
}

/**
 * @brief 计算控制命令幅度。如果低于设定的死区 EPSILON，则视为 0 输出。
 */
static float command_magnitude(float value) {
    float magnitude = fabsf(value);
    if (magnitude < VISION_STATE_EPSILON) return 0.0f;
    return clamp_unit(magnitude);
}

/**
 * @brief 里程计解算：轮子 RPM 转为直线线速度 (米/秒)
 */
static float wheel_rpm_to_linear_mps(float motor_rpm) {
    float wheel_rpm = motor_rpm / CHASSIS_WHEEL_REDUCTION_RATIO;
    return wheel_rpm * (2.0f * (float)M_PI * CHASSIS_WHEEL_RADIUS_M) / 60.0f;
}

/**
 * @brief 估算底盘当前的前进速度 (vx) 与 自转角速度 (wz)
 */
static void estimate_chassis_velocity(float* est_vx, float* est_wz) {
    float left_mps = 0.0f, right_mps = 0.0f;
    // 分别计算左侧和右侧的平均线速度
    left_mps = 0.5f * (wheel_rpm_to_linear_mps(Motors[0].filter_speed * WHEEL_DIR_SIGN[0]) +
                       wheel_rpm_to_linear_mps(Motors[1].filter_speed * WHEEL_DIR_SIGN[1]));
    right_mps = -0.5f * (wheel_rpm_to_linear_mps(Motors[2].filter_speed * WHEEL_DIR_SIGN[2]) +
                         wheel_rpm_to_linear_mps(Motors[3].filter_speed * WHEEL_DIR_SIGN[3]));

    // 正运动学解算：线速度为两侧均值，角速度由差速产生
    if (est_vx != NULL) *est_vx = 0.5f * (left_mps + right_mps);
    if (est_wz != NULL) *est_wz = (right_mps - left_mps) / (2.0f * CHASSIS_TURN_RADIUS_M);
}

/**
 * @brief 周期性向视觉/上位机发送底盘里程计反馈信息
 */
static void send_chassis_feedback(uint32_t now) {
    static uint32_t last_feedback_time = 0;
    Joint_Feedback_t feedback;

    if ((now - last_feedback_time) < CHASSIS_FEEDBACK_PERIOD_MS) return;
    last_feedback_time = now;

    memset(&feedback, 0, sizeof(feedback));
    estimate_chassis_velocity(&feedback.est_vx, &feedback.est_wz);
    feedback.est_vy = 0.0f; // 差速底盘无侧向速度(不具备全向轮属性)
    Protocol_SendJointFeedback(&feedback);
}

/**
 * @brief 清空指定的底层指令结构体，刹车必备
 */
static void clear_chassis_cmd(ChassisCmd_t* cmd) {
    if (cmd == NULL) return;
    cmd->vx = 0.0f;
    cmd->vy = 0.0f;
    cmd->wz = 0.0f;
    cmd->state = ROBOT_STATE_IDLE;
}

// ---------------------- 核心状态推导机制 ----------------------
static RobotState_e normalize_robot_state(uint8_t state) {
    switch (state) {
        case ROBOT_STATE_FORWARD:
        case ROBOT_STATE_BACKWARD:
        case ROBOT_STATE_LEFT:
        case ROBOT_STATE_RIGHT:
            return (RobotState_e)state;
        default:
            return ROBOT_STATE_IDLE;
    }
}

/**
 * @brief 当视觉只下发矢量速度(vx, wz)时，推导出底盘的宏观机器状态(前、后、左、右)
 */
static RobotState_e derive_robot_state(float vx, float wz) {
    // 优先判断自转 (角速度幅值大于线速度)
    if (fabsf(wz) > fabsf(vx) && fabsf(wz) > VISION_STATE_EPSILON) {
        return (wz >= 0.0f) ? ROBOT_STATE_LEFT : ROBOT_STATE_RIGHT;
    }
    // 判断前后平移
    if (fabsf(vx) > VISION_STATE_EPSILON) {
        return (vx >= 0.0f) ? ROBOT_STATE_FORWARD : ROBOT_STATE_BACKWARD;
    }
    return ROBOT_STATE_IDLE;
}

/**
 * @brief 从物理遥控器摇杆原始数据(0-100)推导底盘的宏观机器状态
 */
static RobotState_e derive_rc_state(void) {
    if ((float)myPacket.joy_lx > (RC_JOYSTICK_CENTER + RC_JOYSTICK_MOVE_THRESHOLD)) return ROBOT_STATE_BACKWARD;
    if ((float)myPacket.joy_lx < (RC_JOYSTICK_CENTER - RC_JOYSTICK_MOVE_THRESHOLD)) return ROBOT_STATE_FORWARD;
    if ((float)myPacket.joy_rx > (RC_JOYSTICK_CENTER + RC_JOYSTICK_MOVE_THRESHOLD)) return ROBOT_STATE_LEFT;
    if ((float)myPacket.joy_rx < (RC_JOYSTICK_CENTER - RC_JOYSTICK_MOVE_THRESHOLD)) return ROBOT_STATE_RIGHT;
    return ROBOT_STATE_IDLE;
}

/**
 * @brief 计算摇杆前后的偏移比例 (0.0f ~ 1.0f)
 */
static float compute_rc_linear_ratio(void) {
    if ((float)myPacket.joy_lx < (RC_JOYSTICK_CENTER - RC_JOYSTICK_MOVE_THRESHOLD)) {
        return ((RC_JOYSTICK_CENTER - RC_JOYSTICK_MOVE_THRESHOLD) - (float)myPacket.joy_lx) / (RC_JOYSTICK_CENTER - RC_JOYSTICK_MOVE_THRESHOLD);
    }
    if ((float)myPacket.joy_lx > (RC_JOYSTICK_CENTER + RC_JOYSTICK_MOVE_THRESHOLD)) {
        return ((float)myPacket.joy_lx - (RC_JOYSTICK_CENTER + RC_JOYSTICK_MOVE_THRESHOLD)) / (RC_JOYSTICK_CENTER - RC_JOYSTICK_MOVE_THRESHOLD);
    }
    return 0.0f;
}

/**
 * @brief 计算摇杆左右的偏移比例 (0.0f ~ 1.0f)
 */
static float compute_rc_turn_ratio(void) {
    if ((float)myPacket.joy_rx < (RC_JOYSTICK_CENTER - RC_JOYSTICK_MOVE_THRESHOLD)) {
        return ((RC_JOYSTICK_CENTER - RC_JOYSTICK_MOVE_THRESHOLD) - (float)myPacket.joy_rx) / (RC_JOYSTICK_CENTER - RC_JOYSTICK_MOVE_THRESHOLD);
    }
    if ((float)myPacket.joy_rx > (RC_JOYSTICK_CENTER + RC_JOYSTICK_MOVE_THRESHOLD)) {
        return ((float)myPacket.joy_rx - (RC_JOYSTICK_CENTER + RC_JOYSTICK_MOVE_THRESHOLD)) / (RC_JOYSTICK_CENTER - RC_JOYSTICK_MOVE_THRESHOLD);
    }
    return 0.0f;
}

// ==========================================
// 指令调度与仲裁中心 (Command Arbitration)
// ==========================================
static void update_rc_command(void) {
    clear_chassis_cmd(&cmd_rc);
    cmd_rc.state = derive_rc_state();
    cmd_rc.vx = compute_rc_linear_ratio();
    cmd_rc.wz = compute_rc_turn_ratio();
}

static void update_vision_command(void) {
    Chassis_Move_t vision_cmd = Protocol_GetChassisMove();
    clear_chassis_cmd(&cmd_vision);

    if (Protocol_IsChassisTimeout()) return;

    cmd_vision.vx = vision_cmd.vx;
    cmd_vision.vy = vision_cmd.vy;
    cmd_vision.wz = vision_cmd.wz;

    if (normalize_robot_state(vision_cmd.state) != ROBOT_STATE_IDLE) {
        cmd_vision.state = normalize_robot_state(vision_cmd.state);
    } else {
        cmd_vision.state = derive_robot_state(vision_cmd.vx, vision_cmd.wz);
    }
}

/**
 * @brief 检查玩家是否动了遥控器摇杆，用于在自动驾驶模式下抢夺控制权
 */
static uint8_t rc_override_requested(void) {
    float delta_lx = fabsf((float)myPacket.joy_lx - RC_JOYSTICK_CENTER);
    float delta_rx = fabsf((float)myPacket.joy_rx - RC_JOYSTICK_CENTER);
    // 只要任一摇杆偏离中位超过 18，就算抢夺请求生效
    return (delta_lx > RC_OVERRIDE_THRESHOLD || delta_rx > RC_OVERRIDE_THRESHOLD) ? 1U : 0U;
}

/**
 * @brief 指令仲裁：决定最终底盘听谁的指令 (RC 还是 Vision)
 */
static void update_control_mode(void) {

    // 【安全防线 1：系统枚举篡改保护】
    // 防止 DMA 或数组越界把当前内存刷成了类似 0xD5 等未知值导致逻辑瘫痪
    if (current_sys_mode != SYS_MODE_RC && current_sys_mode != SYS_MODE_VISION) {
        current_sys_mode = SYS_MODE_RC;
    }

#if COMPETITION_AUTO_MODE
    current_sys_mode = SYS_MODE_VISION;
    cmd_target = cmd_vision;
#else
    // 抢夺逻辑：视觉模式下有明显的遥控器操作，则自动切回人工遥控托管 (安全底线)
    if (current_sys_mode == SYS_MODE_VISION && rc_override_requested()) {
        current_sys_mode = SYS_MODE_RC;
    }

    if (current_sys_mode == SYS_MODE_VISION) {
        cmd_target = cmd_vision;
    } else {
        cmd_target = cmd_rc;
    }
#endif

    // 如果失去上位机通讯心跳，则自动将目标指令清零，防止飞车
    if (current_sys_mode == SYS_MODE_VISION && Protocol_IsChassisTimeout()) {
        clear_chassis_cmd(&cmd_target);
    }
}

void Control_Init(QuadrupedGait* gait) {
    chassis_mode = 2; // 初始化为纯足模式
    current_gait_mode = GAIT_MODE_TROT;
    previous_gait_mode = current_gait_mode;
    current_state = ROBOT_STATE_IDLE;
    current_sys_mode = SYS_MODE_RC;
    force_gait_update = 0;
    is_single_action = 0;
    is_zeroing = 0;
    is_adjusting_pose = 0;
    is_custom_interp = 0;
    memset((void*)&g_robot_diag, 0, sizeof(g_robot_diag));
    memset(locked_angle, 0, sizeof(locked_angle));
    locked_angle_ready = 0;
    clear_chassis_cmd(&cmd_rc);
    clear_chassis_cmd(&cmd_vision);
    clear_chassis_cmd(&cmd_target);

    if (gait != NULL) apply_current_gait(gait);
}

SystemCtrlMode_e Control_GetSystemMode(void) {
    return current_sys_mode;
}

// ---------------------- 动作插值支持 (防摔倒、防抽搐过渡核心) ----------------------

void start_custom_interpolation(uint32_t now, uint32_t duration_ms) {
    if (is_custom_interp) return;
    is_custom_interp = 1;
    is_zeroing = 0;         // 强制打断归零过程
    is_adjusting_pose = 0;  // 强制打断微调过程
    custom_interp_duration = duration_ms;
    custom_interp_start_time = now;
    // 捕获四条腿当下的真实正运动学物理坐标，以此为起点插值
    custom_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
    custom_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
    custom_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
    custom_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);
}

/**
 * @brief 启动零位插值：捕获当前位置，准备恢复到站立点
 */
void start_zero_interpolation(uint32_t now) {
    if (is_zeroing) return;
    is_zeroing = 1;
    is_adjusting_pose = 0;
    is_custom_interp = 0;

    // 如果是开机第一次归零，放慢动作 (2500ms)，否则使用常规耗时 (1000ms)
    if (is_first_zero == 0) {
        current_zeroing_duration = ZEROING_DURATION_MS;
        is_first_zero = 1;
    } else {
        current_zeroing_duration = ZEROING_last_ms;
    }
    zeroing_start_time = now;
    zero_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
    zero_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
    zero_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
    zero_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);
}

/**
 * @brief 零位插值实际执行函数：在底层的 5ms 循环中被不断调用
 */
void state_zero(uint32_t now, float current_kp) {
    if (!is_zeroing) return;
    float current_target_x[4];
    float current_target_y[4];

    // 计算当前时间的进度百分比 (0.0 ~ 1.0)
    float progress = (float)(now - zeroing_start_time) / (float)current_zeroing_duration;
    if (progress >= 1.0f) {
        progress = 1.0f;
        is_zeroing = 0; // 进度到达 100%，结束插值周期
    }
    // 线性插值生成此刻的过渡坐标
    for (int i = 0; i < 4; i++) {
        current_target_x[i] = zero_start_pos[i].X + (ZERO_TARGET_X - zero_start_pos[i].X) * progress;
        current_target_y[i] = zero_start_pos[i].Y + (ZERO_TARGET_Y - zero_start_pos[i].Y) * progress;
    }

    // 逆运动学反解为电机角度并下发给电机执行 (ID： 1/2，7/8，6/5，3/4)
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

void state_custom_interpolation(uint32_t now, float current_kp, float target_x_grp1, float target_y_grp1, float target_x_grp2, float target_y_grp2) {
    if (!is_custom_interp) return;
    float progress = (float)(now - custom_interp_start_time) / (float)custom_interp_duration;
    if (progress >= 1.0f) {
        progress = 1.0f;
        is_custom_interp = 0;
    }
    float current_target_x[4], current_target_y[4];
    // 组 2 (腿 0 和 3) 插值
    current_target_x[0] = custom_start_pos[0].X + (target_x_grp2 - custom_start_pos[0].X) * progress;
    current_target_y[0] = custom_start_pos[0].Y + (target_y_grp2 - custom_start_pos[0].Y) * progress;
    current_target_x[3] = custom_start_pos[3].X + (target_x_grp2 - custom_start_pos[3].X) * progress;
    current_target_y[3] = custom_start_pos[3].Y + (target_y_grp2 - custom_start_pos[3].Y) * progress;

    // 组 1 (腿 1 和 2) 插值
    current_target_x[1] = custom_start_pos[1].X + (target_x_grp1 - custom_start_pos[1].X) * progress;
    current_target_y[1] = custom_start_pos[1].Y + (target_y_grp1 - custom_start_pos[1].Y) * progress;
    current_target_x[2] = custom_start_pos[2].X + (target_x_grp1 - custom_start_pos[2].X) * progress;
    current_target_y[2] = custom_start_pos[2].Y + (target_y_grp1 - custom_start_pos[2].Y) * progress;

    // 逆解与下发
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
    is_zeroing = 0;        // 【重要互斥】强制打断归零插值状态
    is_custom_interp = 0;  // 【重要互斥】强制打断分组自定义插值
    pose_adjust_start_time = now;
    pose_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
    pose_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
    pose_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
    pose_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);

    // 计算终点目标位置 (当前基准位 + 增量)
    for (int i = 0; i < 4; i++) {
        pose_target_pos[i].X = pose_start_pos[i].X + delta_x;
        pose_target_pos[i].Y = pose_start_pos[i].Y + delta_y;
    }
}

/**
 * @brief 姿态微调执行：同理按比例插值逼近目标值
 */
void update_pose_adjustment(uint32_t now, float current_kp) {
    if (!is_adjusting_pose) return;
    float progress = (float)(now - pose_adjust_start_time) / (float)POSE_ADJUST_DURATION_MS;
    if (progress >= 1.0f) {
        progress = 1.0f;
        is_adjusting_pose = 0;
    }
    float current_target_x[4], current_target_y[4];
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

// ==========================================
// 核心大一统主执行控制流 (统一调度层)
// ==========================================
/**
 * @brief 核心执行函数：负责融合所有动作执行。
 * 核心逻辑：无论接收由 RC 还是 Vision 传来的共用指令 (`cmd_target`)，底层计算步态与姿态的代码必须唯一。
 */
static void execute_chassis_command(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, LegAngles angles, uint32_t now) {
    RobotState_e target_state = cmd_target.state;
    float currentTimeSec = now / 1000.0f;
    static uint32_t last_calc_time = 0;
    static SystemCtrlMode_e last_sys_mode = SYS_MODE_RC;

    // 【安全防线 2：底盘模式防乱码保护】
    // 防止内存溢出导致 chassis_mode 变成乱数，保证总是落在安全的 0, 1, 2 范围内
    if (chassis_mode > 2) {
        chassis_mode = 2; // 默认切回纯足模式最为安全
    }

    // 当视觉与遥控主权切换时，重置机器人为待命，强制安全归零
    if (current_sys_mode != last_sys_mode) {

        // 【核心修改】：自动绑定底盘行为。确保一切入视觉，就是标准纯轮小车。
        if (current_sys_mode == SYS_MODE_VISION) {
            chassis_mode = 1; // 视觉模式默认：切为纯轮模式
        } else {
            chassis_mode = 2; // 遥控模式默认：恢复纯足模式
            for (int j = 0; j < 4; j++) {
                locked_angle[j] = Motors[j].total_angle;
            }
            locked_angle_ready = 1;
            current_gait_mode = GAIT_MODE_TROT;
            force_gait_update = 1;
        }

        // 强制触发一次平滑收腿
        start_zero_interpolation(now);

        current_state = ROBOT_STATE_IDLE;
        target_state = ROBOT_STATE_IDLE; // 强制本帧为待机，拦截可能遗留的运动指令
        last_sys_mode = current_sys_mode;
    }

    // 1. 处理单次锁定动作 (如特定的跳跃)，在此期间忽略任何其他遥控指令
    if (is_single_action) {
        if (now >= single_action_end_time) {
            is_single_action = 0;
            current_gait_mode = previous_gait_mode;
            apply_current_gait(gait);
            current_state = ROBOT_STATE_IDLE;
            start_zero_interpolation(now);          // 动作做完触发平滑复位
        } else {
            if (now - last_calc_time >= 5) {
                last_calc_time = now;
                // 单次动作时强制调用步态更新
                angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
                angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, kp, 0.1);
                angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
                angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);

                if (chassis_mode == 2) {
                    hold_wheels_at_locked_angle();
                } else {
                    // 单次动作中，不允许底盘滑动，轮子全刹车
                    for (int i = 0; i < 4; i++) {
                        Motors[i].target_speed = 0.0f;
                        PID_Calc_Speed(i);
                    }
                }
            }
        }
        return; // 单次动作执行中直接退出不执行常规操作
    }

    // 2. 状态切换(边缘触发)处理与对应腿部标定
    if (target_state != current_state) {
        current_state = target_state;
        force_gait_update = 0;

        // 状态切换时清除历史误差防止超调
        for (int i = 0; i < 4; i++) {
            if (current_sys_mode == SYS_MODE_RC) reset_wheel_pid_state(i);
            Clear_Motor_PID(i);
        }

        // 根据切换进的新状态初始化步态生成器的初相，标定不同腿的相位 (0 或 1，形成对角步态)
        switch (current_state) {
            case ROBOT_STATE_FORWARD:
                // 仅允许在 RC 且非纯轮模式下让腿部起步。如果是 Vision 或纯轮，腿部直接保持静默(归零)
                if (!is_pure_wheel_mode && current_sys_mode == SYS_MODE_RC) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                    calibrate_leg_base_position(gait, 2, ctrl1,6,5,1);
                    calibrate_leg_base_position(gait, 3, ctrl1,3,4,1);
                    start_quadruped_gait(gait, currentTimeSec);
                } else {
                    start_zero_interpolation(now);
                }
                break;
            case ROBOT_STATE_BACKWARD:
                if (!is_pure_wheel_mode && current_sys_mode == SYS_MODE_RC) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
                    calibrate_leg_base_position(gait, 2, ctrl1,6,5,0);
                    calibrate_leg_base_position(gait, 3, ctrl1,3,4,0);
                    start_quadruped_gait(gait, currentTimeSec);
                } else {
                    start_zero_interpolation(now);
                }
                break;
            case ROBOT_STATE_LEFT:
                if (!is_pure_wheel_mode && current_sys_mode == SYS_MODE_RC) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
                    calibrate_leg_base_position(gait, 2, ctrl1,3,4,1);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                    calibrate_leg_base_position(gait, 3, ctrl1,6,5,0);
                    start_quadruped_gait(gait, currentTimeSec);
                } else {
                    start_zero_interpolation(now);
                }
                break;
            case ROBOT_STATE_RIGHT:
                if (!is_pure_wheel_mode && current_sys_mode == SYS_MODE_RC) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
                    calibrate_leg_base_position(gait, 2, ctrl1,3,4,0);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
                    calibrate_leg_base_position(gait, 3, ctrl1,6,5,1);
                    start_quadruped_gait(gait, currentTimeSec);
                } else {
                    start_zero_interpolation(now);
                }
                break;
            case ROBOT_STATE_IDLE:
                // 收到刹车/停止指令时立刻触发归零，并把当下轮子角度存入 locked_angle 用于未来锁死
                start_zero_interpolation(now);
                for(int i = 0; i < 4; i++) {
                   locked_angle[i] = Motors[i].total_angle;
                }
                locked_angle_ready = 1;
                break;
        }
    }

    // 3. 5ms 主定时循环计算逻辑 (200Hz 控制频率)
    if (now - last_calc_time >= 5) {
        last_calc_time = now;

        // 如果不是静止状态，即在运动中
        if (current_state != ROBOT_STATE_IDLE) {
            float wheel_target_rpm_01 = 0.0f; // 左侧两轮速度
            float wheel_target_rpm_23 = 0.0f; // 右侧两轮速度
            float base_rpm = 0.0f;
            float turn_rpm = 0.0f;

            // 动态选择当前操作源的最大速度限幅，并解析出最终参考转速
            if (current_sys_mode == SYS_MODE_RC) {
                float linear_ratio = compute_rc_linear_ratio();
                float turn_ratio = compute_rc_turn_ratio();

                float max_base_rpm = is_pure_wheel_mode ? 2400.0f : RC_MAX_BASE_RPM;
                float max_turn_rpm = is_pure_wheel_mode ? 1800.0f : RC_MAX_TURN_RPM;

                base_rpm = linear_ratio * max_base_rpm;
                turn_rpm = turn_ratio * max_turn_rpm;
            } else {
                base_rpm = VISION_MAX_LINEAR_RPM * command_magnitude(cmd_target.vx);
                turn_rpm = VISION_MAX_TURN_RPM * command_magnitude(cmd_target.wz);
            }

            // 【腿部动作下发】：严格限定只在 RC 并且非纯轮下才让狗腿走路
            if (!is_pure_wheel_mode && current_sys_mode == SYS_MODE_RC) {
                if (current_state == ROBOT_STATE_FORWARD || current_state == ROBOT_STATE_BACKWARD) {
                    angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
                    angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, kp, 0.1);
                    angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
                    angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);
                } else if (current_state == ROBOT_STATE_LEFT || current_state == ROBOT_STATE_RIGHT) {
                    angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, 2.6f, 0.1);
                    angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, 2.6f, 0.1);
                    angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, 2.6f, 0.1);
                    angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, 2.6f, 0.1);
                }
            } else {
                // 如果是纯轮模式或视觉托管，腿部必须保持静默插值姿势，不介入运动步态
                if (is_zeroing) {
                    state_zero(now, kp);
                } else if (is_adjusting_pose) {
                    update_pose_adjustment(now, kp);
                } else if (is_custom_interp) {
                    state_custom_interpolation(now, kp, custom_target_x_grp1, custom_target_y_grp1, custom_target_x_grp2, custom_target_y_grp2);
                }
            }

            // 【差速模型解算】：把参考的前进后退和旋转解算进左右两端轮组的 RPM
            if (current_state == ROBOT_STATE_FORWARD) {
                wheel_target_rpm_01 = base_rpm;
                wheel_target_rpm_23 = -base_rpm;
            } else if (current_state == ROBOT_STATE_BACKWARD) {
                wheel_target_rpm_01 = -base_rpm;
                wheel_target_rpm_23 = base_rpm;
            } else if (current_state == ROBOT_STATE_LEFT) {
                wheel_target_rpm_01 = -turn_rpm;
                wheel_target_rpm_23 = -turn_rpm;
            } else if (current_state == ROBOT_STATE_RIGHT) {
                wheel_target_rpm_01 = turn_rpm;
                wheel_target_rpm_23 = turn_rpm;
            }

            // 【底盘轮子动作下发】
            if (chassis_mode == 2) {
                // 如果是纯足模式，直接通过位置闭环把轮子锁在原位
                hold_wheels_at_locked_angle();
            } else {
                // 正常输出运动所需的速度闭环指令
                for (int i = 0; i < 2; i++) {
                    set_wheel_speed_target(i, wheel_target_rpm_01);
                }
                for (int i = 2; i < 4; i++) {
                    set_wheel_speed_target(i, wheel_target_rpm_23);
                }
            }
        } else {
            // ==================
            // IDLE 动作托底逻辑
            // ==================
            if (is_zeroing) {
                state_zero(now, kp);
            } else if (is_adjusting_pose) {
                update_pose_adjustment(now, kp);
            } else if (is_custom_interp) {
                state_custom_interpolation(now, kp, custom_target_x_grp1, custom_target_y_grp1, custom_target_x_grp2, custom_target_y_grp2);
            }

            if (chassis_mode == 2) {
                // 纯足锁轮
                hold_wheels_at_locked_angle();
            } else {
                // 普通待命停止
                for (int i = 0; i < 4; i++) {
                    Motors[i].target_speed = 0.0f;
                    PID_Calc_Speed(i);
                }
            }
        }

        // 收集发送指令和底盘测程反馈数据
        send_current();
        send_chassis_feedback(now);
    }
}

// ---------------------- 遥控器输入拦截与执行包装 ----------------------
/**
 * @brief 遥控器按键扫描与防抖控制逻辑
 */
#define DEBOUNCE_DELAY_MS 30
#define NUM_BUTTONS 8

void button_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t currentTime) {
    static uint8_t stable_button_states[NUM_BUTTONS] = {0};
    static uint8_t last_button_readings[NUM_BUTTONS] = {0};
    static uint32_t last_debounce_times[NUM_BUTTONS] = {0};

    float dy = 1.0f;
    float dx = 0.0f;

    // 【安全防线 3：防止越界访问】
    // 虽然 NUM_BUTTONS 宏定义是 8，但为了绝对安全，取它与实际数组长度中的较小值
    int safe_btn_count = (sizeof(myPacket.button) < NUM_BUTTONS) ? sizeof(myPacket.button) : NUM_BUTTONS;

    for (int i = 0; i < safe_btn_count; i++) {
        uint8_t current_reading = myPacket.button[i];

        // 状态如果变化，重新记录时间准备防抖
        if (current_reading != last_button_readings[i]) {
            last_debounce_times[i] = currentTime;
        }

        // 防抖确认 (时间超过 30ms 算有效按下)
        if ((currentTime - last_debounce_times[i]) > DEBOUNCE_DELAY_MS) {
            if (current_reading != stable_button_states[i]) {
                stable_button_states[i] = current_reading;
                // 检测到按下上升沿
                if (stable_button_states[i] == 1) {

                    // 【安全屏障】：在视觉自动控制状态下，除了专门用于接管控制权的“按键3”，禁阻其它杂项按钮干扰
                    if (i != 3 && current_sys_mode != SYS_MODE_RC) {
                        continue;
                    }

                    switch (i) {
                        case 0:
                            start_zero_interpolation(currentTime);
                            break;
                        case 1:
                            start_pose_adjustment(currentTime, dx, dy);
                            break;
                        case 2:
                            start_pose_adjustment(currentTime, -dx, -dy);
                            break;
                        case 3:
                            // 按键 3：专职用于手动切换系统操作主权 (RC <-> VISION)
                            if (current_sys_mode == SYS_MODE_RC) {
                                current_sys_mode = SYS_MODE_VISION;
                            } else {
                                current_sys_mode = SYS_MODE_RC;
                            }
                            // 每次大模式切换，必须清空目标状态让机器归零，防暴走
                            current_state = ROBOT_STATE_IDLE;
                            clear_chassis_cmd(&cmd_target);
                            break;
                        case 4:
                            // 切小跑
                            if (current_gait_mode != GAIT_MODE_TROT) {
                                current_gait_mode = GAIT_MODE_TROT;
                                force_gait_update = 1;
                            }
                            break;
                        case 5:
                            // 切走路
                            if (current_gait_mode != GAIT_MODE_WALK) {
                                current_gait_mode = GAIT_MODE_WALK;
                                force_gait_update = 1;
                            }
                            break;
                        case 6:

                            break;
                        case 7:
                            // 在 0, 1, 2 之间循环切换底盘模式
                            chassis_mode = (chassis_mode + 1) % 3;

                            if (chassis_mode == 1) {
                                // 【切换到纯轮模式】：
                                // 立刻触发插值，让狗腿平滑收回到标准的站立姿态，之后只靠轮子转
                                start_zero_interpolation(currentTime);

                            } else if (chassis_mode == 2) {
                                // 【切换到纯足模式】：
                                // 瞬间捕获并记录当前4个轮子的实时角度（编码器位置）
                                for (int j = 0; j < 4; j++) {
                                    locked_angle[j] = Motors[j].total_angle;
                                }
                                locked_angle_ready = 1;
                                // 唤醒步态准备走路
                                current_gait_mode = GAIT_MODE_TROT;
                                force_gait_update = 1;

                            } else {
                                // 【切换回轮腿联动 (0)】：
                                // 恢复正常的小跑步态
                                current_gait_mode = GAIT_MODE_TROT;
                                force_gait_update = 1;
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
 * @brief 摇杆控制处理：顶层包装器，依次刷新输入源状态并派发到底层
 */
void joystick_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
    // 1. 采集并解析 RC 原生指令
    update_rc_command();
    // 2. 采集并解析 Vision 原生指令
    update_vision_command();
    // 3. 执行仲裁，将拥有优先权的指令复制进 cmd_target
    update_control_mode();

    // 4. 下发给控制枢纽，统一处理 100% 同构的底层动作
    execute_chassis_command(ctrl1, ctrl2, gait, angles, now);
}

/**
 * @brief 接收 AS01 遥控数据并完成整套控制调度 (包含超时保护)
 */
void AS01_rx(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
    static uint32_t last_rx_time = 0;
    static uint8_t remote_timeout_latched = 0;

    // 数据读取进 myPacket
    if (NRF24L01_RxPacket(rx_buffer) == 0) {

        // 【安全防线 4：严格限制结构体拷贝长度】
        // 这一步能彻底根绝如果 RemoteData_t 因为代码修改导致越界而带来的全局变量错位问题
        size_t copy_size = sizeof(RemoteData_t) <= sizeof(rx_buffer) ? sizeof(RemoteData_t) : sizeof(rx_buffer);
        memcpy(&myPacket, rx_buffer, copy_size);

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 接收指示灯
        g_robot_diag.nrf_rx_count++;
        g_robot_diag.nrf_last_rx_time = now;
        last_rx_time = now;
        remote_timeout_latched = 0;
    }

    // 【失联保护机制】: 如果距离上次收到心跳超过 300ms，直接将虚拟手柄归中
    if ((now - last_rx_time) > REMOTE_TIMEOUT_MS) {
        if (!remote_timeout_latched) {
            g_robot_diag.nrf_timeout_count++;
            remote_timeout_latched = 1;
        }
        myPacket.joy_lx = (uint16_t)RC_JOYSTICK_CENTER;
        myPacket.joy_rx = (uint16_t)RC_JOYSTICK_CENTER;
        myPacket.joy_ly = (uint16_t)RC_JOYSTICK_CENTER;
        myPacket.joy_ry = (uint16_t)RC_JOYSTICK_CENTER;
        memset(myPacket.button, 0, sizeof(myPacket.button));
    }

    button_control(ctrl1, ctrl2, gait, now);
    joystick_control(ctrl1, ctrl2, gait, startTime, angles, now);
}
