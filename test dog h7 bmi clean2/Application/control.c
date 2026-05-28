#include "control.h"
#include <string.h> // 纭繚 memcpy 鍙敤
#include <math.h>
#include "3508_driver.h"
#include "Chassis_Control.h"
#include "Attitude_solution.h"

extern RemoteData_t myPacket;
extern uint8_t rx_buffer[32];
uint8_t as01_status = 0;
extern uint8_t rx_data[32];
extern __attribute__((section(".RAM_D1")))__attribute__((section(".RAM_D1")))MotorController ctrl2;
extern MotorController ctrl1;
extern float leg_y_offsets[4];

float debug_att_target_y[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float debug_att_hip_cmd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float debug_att_knee_cmd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float debug_att_hip_delta[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float debug_att_knee_delta[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t debug_att_base_valid = 0;
static float debug_att_base_hip[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float debug_att_base_knee[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static float kp = 2.6f;
static GaitMode_e current_gait_mode = GAIT_MODE_TROT;
static uint8_t force_gait_update = 0;
static uint8_t is_single_action = 0;
static uint32_t single_action_end_time = 0;
static GaitMode_e previous_gait_mode = GAIT_MODE_TROT;
static RobotState_e current_state = ROBOT_STATE_IDLE;
static uint8_t is_zeroing = 0;
static uint32_t zeroing_start_time = 0;
static Currentpos zero_start_pos[4];
static const float ZERO_TARGET_X = 10.0f;
static const float ZERO_TARGET_Y = 25.0f;
static const uint32_t ZEROING_DURATION_MS = 2500;
static uint32_t current_zeroing_duration = 2500;
static const uint32_t ZEROING_last_ms = 1000;
static uint8_t is_adjusting_pose = 0;
static uint32_t pose_adjust_start_time = 0;
static Currentpos pose_start_pos[4];
static Currentpos pose_target_pos[4];
static const uint32_t POSE_ADJUST_DURATION_MS = 300;
static uint8_t is_first_zero = 0;
static uint8_t attitude_comp_enabled = 0;
static uint8_t attitude_zero_calibrating = 0;
static uint32_t attitude_zero_cal_start_ms = 0;
static float attitude_zero_roll_sum = 0.0f;
static float attitude_zero_pitch_sum = 0.0f;
static uint16_t attitude_zero_sample_count = 0;
static const uint32_t ATTITUDE_ZERO_SETTLE_MS = 500;
static const uint32_t ATTITUDE_ZERO_SAMPLE_MS = 500;
static uint8_t is_custom_interp = 0;
static uint32_t custom_interp_start_time = 0;
static Currentpos custom_start_pos[4];
static uint32_t custom_interp_duration = 1000;
static float custom_target_x_grp1 = 0.0f; // 鑵?,2 鐨勭洰鏍嘪
static float custom_target_y_grp1 = 0.0f; // 鑵?,2 鐨勭洰鏍嘫
static float custom_target_x_grp2 = 0.0f;  // 鑵?,3 鐨勭洰鏍嘪
static float custom_target_y_grp2 = 0.0f; // 鑵?,3 鐨勭洰鏍嘫
// 銆愪慨鏀圭偣1銆戯細鏇挎崲鍘熸潵鐨?is_pure_wheel_mode锛屽崌绾т负3鎬佹ā寮?// 搴曠洏妯″紡锛?-杞吙鑱斿姩锛?-绾疆妯″紡锛?-绾冻妯″紡(閿佽疆)
static uint8_t chassis_mode = 0;
#define is_pure_wheel_mode (chassis_mode == 1) // 鍏煎鍘熸湁鍒ゆ柇
// 鐢ㄤ簬璁板綍绾冻妯″紡涓?涓疆瀛愮殑椹昏溅鐩爣瑙掑害
static int64_t locked_angles[4] = {0};

static void state_stand_attitude(float current_kp);
static void update_attitude_comp_for_gait(QuadrupedGait* gait, float current_time);
static void start_attitude_zero_calibration(void);
static void update_attitude_zero_calibration(void);
static void start_gait_from_stand_base(QuadrupedGait* gait, float current_time);
static void begin_zero_interpolation(uint32_t now);
static void start_zero_interpolation_from_gait(QuadrupedGait* gait, uint32_t now, float current_time);

/**
 * @brief 濮挎€佽ˉ鍋挎洿鏂?鍦?5ms (200Hz) 涓诲惊鐜腑璋冪敤
 */
void Chassis_Attitude_Update(void)
{
    update_attitude_zero_calibration();

    if (attitude_comp_enabled && is_zeroing == 0 && is_adjusting_pose == 0 && is_custom_interp == 0 && chassis_mode != 2) {
        if (current_state == ROBOT_STATE_IDLE) {
            Chassis_Set_All_Leg_Weights(1.0f, 1.0f, 1.0f, 1.0f);
            Chassis_Attitude_Loop(0.005f);
        }
    } else {
        for (int i = 0; i < 4; i++) {
            leg_y_offsets[i] = 0.0f;
        }
    }
}

static void start_attitude_zero_calibration(void)
{
    attitude_comp_enabled = 0;
    attitude_zero_calibrating = 1;
    attitude_zero_cal_start_ms = HAL_GetTick();
    attitude_zero_roll_sum = 0.0f;
    attitude_zero_pitch_sum = 0.0f;
    attitude_zero_sample_count = 0;
    Chassis_Reset_Attitude_Filter();
}

static void update_attitude_zero_calibration(void)
{
    if (!attitude_zero_calibrating) {
        return;
    }

    for (int i = 0; i < 4; i++) {
        leg_y_offsets[i] = 0.0f;
    }

    if (is_zeroing || is_adjusting_pose || is_custom_interp) {
        attitude_zero_cal_start_ms = HAL_GetTick();
        attitude_zero_roll_sum = 0.0f;
        attitude_zero_pitch_sum = 0.0f;
        attitude_zero_sample_count = 0;
        return;
    }

    uint32_t elapsed_ms = HAL_GetTick() - attitude_zero_cal_start_ms;
    if (elapsed_ms < ATTITUDE_ZERO_SETTLE_MS) {
        return;
    }

    if (elapsed_ms < (ATTITUDE_ZERO_SETTLE_MS + ATTITUDE_ZERO_SAMPLE_MS)) {
        attitude_zero_roll_sum += g_Attitude.Angle_Roll;
        attitude_zero_pitch_sum += g_Attitude.Angle_Pitch;
        if (attitude_zero_sample_count < 0xFFFFu) {
            attitude_zero_sample_count++;
        }
        return;
    }

    if (attitude_zero_sample_count > 0) {
        float zero_roll = attitude_zero_roll_sum / (float)attitude_zero_sample_count;
        float zero_pitch = attitude_zero_pitch_sum / (float)attitude_zero_sample_count;
        Chassis_Set_Target_Attitude(zero_roll, zero_pitch);
    } else {
        Chassis_Set_Target_Attitude(g_Attitude.Angle_Roll, g_Attitude.Angle_Pitch);
    }

    Chassis_Reset_Attitude_Filter();
    attitude_zero_calibrating = 0;
    attitude_comp_enabled = 1;
}

static void update_attitude_comp_for_gait(QuadrupedGait* gait, float current_time)
{
    if (!attitude_comp_enabled || gait == 0 || !gait->is_running || gait->gait_cycle_time <= 0.001f) {
        for (int i = 0; i < 4; i++) {
            leg_y_offsets[i] = 0.0f;
        }
        return;
    }

    float support_weight[4];
    float elapsed_time = current_time - gait->start_time;

    for (int i = 0; i < 4; i++) {
        float leg_total_cycles = (elapsed_time / gait->gait_cycle_time) + gait->legs[i].phase_offset;
        float t_norm = leg_total_cycles - floorf(leg_total_cycles);
        uint8_t is_swing = (t_norm < 0.5f) ? 1U : 0U;

        support_weight[i] = is_swing ? 0.0f : 1.0f;
        gait->legs[i].is_swing_phase = is_swing;
    }

    Chassis_Set_All_Leg_Weights(support_weight[0], support_weight[1],
                                support_weight[2], support_weight[3]);
    Chassis_Attitude_Loop(0.005f);
}

static void start_gait_from_stand_base(QuadrupedGait* gait, float current_time)
{
    for (int i = 0; i < 4; i++) {
        gait->legs[i].x_base = ZERO_TARGET_X;
        gait->legs[i].y_base = ZERO_TARGET_Y;
    }

    start_quadruped_gait(gait, current_time);
}

static void state_stand_attitude(float current_kp)
{
    float target_y[4];
    for (int i = 0; i < 4; i++) {
        target_y[i] = ZERO_TARGET_Y + Chassis_Get_Leg_Offset(i);
        if (target_y[i] < 20.0f) {
            target_y[i] = 20.0f;
        }
        debug_att_target_y[i] = target_y[i];
    }

    LegAngles angles1 = InverseKinematics(ZERO_TARGET_X, target_y[0], &ctrl2, 7, 8, current_kp, 0.1);
    LegAngles angles2 = InverseKinematics(ZERO_TARGET_X, target_y[1], &ctrl2, 1, 2, current_kp, 0.1);
    LegAngles angles3 = InverseKinematics(ZERO_TARGET_X, target_y[2], &ctrl1, 6, 5, current_kp, 0.1);
    LegAngles angles4 = InverseKinematics(ZERO_TARGET_X, target_y[3], &ctrl1, 3, 4, current_kp, 0.1);

    debug_att_hip_cmd[0] = angles1.theta1;
    debug_att_knee_cmd[0] = angles1.theta2;
    debug_att_hip_cmd[1] = angles2.theta1;
    debug_att_knee_cmd[1] = angles2.theta2;
    debug_att_hip_cmd[2] = angles3.theta1;
    debug_att_knee_cmd[2] = angles3.theta2;
    debug_att_hip_cmd[3] = angles4.theta1;
    debug_att_knee_cmd[3] = angles4.theta2;

    if (!debug_att_base_valid) {
        for (int i = 0; i < 4; i++) {
            debug_att_base_hip[i] = debug_att_hip_cmd[i];
            debug_att_base_knee[i] = debug_att_knee_cmd[i];
        }
        debug_att_base_valid = 1;
    }
    for (int i = 0; i < 4; i++) {
        debug_att_hip_delta[i] = debug_att_hip_cmd[i] - debug_att_base_hip[i];
        debug_att_knee_delta[i] = debug_att_knee_cmd[i] - debug_att_base_knee[i];
    }
}

static inline void apply_current_gait(QuadrupedGait* gait) {
    float straight_period = 1.2f; // 鍛ㄦ湡 (绉?
    float straight_length = 8.5f; // 姝ラ暱
    float straight_height = 10.0f; // 鎶吙楂樺害
    float turn_period = 0.8f;     // 杞悜鍛ㄦ湡 (閫氬父鏇村揩涓€鐐?
    float turn_length = 3.0f;     // 杞悜姝ラ暱 (寤鸿鏀瑰皬锛屼互杞瓙宸€熶负涓?
    float turn_height = 5.0f;     // 杞悜鎶吙楂樺害 (寤鸿鏀归珮锛岄槻姝㈣吙缁婁綇鍦伴潰)

    // Select parameters based on current state
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
            init_quadruped_gait_walk(gait, 1.2f, l, h + 2.0f);
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
 * @brief 鍚姩鑷畾涔夊垎缁勬彃鍊硷細鎹曡幏褰撳墠浣嶇疆
 * @param now 褰撳墠绯荤粺鏃堕棿
 * @param duration_ms 鏈熸湜瀹屾垚杩欎釜鍔ㄤ綔鐨勬€绘绉掓暟
 */
void start_custom_interpolation(uint32_t now, uint32_t duration_ms) {
    if (is_custom_interp) return;

    is_custom_interp = 1;
    is_zeroing = 0;
    is_adjusting_pose = 0;
    custom_interp_duration = duration_ms;
    custom_interp_start_time = now;

    custom_start_pos[0] = ForwardKinematics(&ctrl2, 7, 8); // 缁?(鍓?
    custom_start_pos[1] = ForwardKinematics(&ctrl2, 1, 2); // 缁?(鍚?
    custom_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5); // 缁?(鍓?
    custom_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4); // 缁?(鍚?
}
/**
 * @brief 鍚姩闆朵綅鎻掑€硷細鎹曡幏褰撳墠浣嶇疆骞堕攣瀹氱姸鎬? */
static void begin_zero_interpolation(uint32_t now) {
    attitude_comp_enabled = 0;
    debug_att_base_valid = 0;
    Chassis_Control_Init();
    is_zeroing = 1;
    is_adjusting_pose = 0;
    if (is_first_zero == 0) {
            current_zeroing_duration = ZEROING_DURATION_MS; // 2500ms
            is_first_zero = 1; // 鏍囪浠ュ悗涓嶅啀鏄涓€娆′簡
        } else {
            current_zeroing_duration = ZEROING_last_ms;    // 1000ms
        }
    zeroing_start_time = now;
}

void start_zero_interpolation(uint32_t now) {
    if (is_zeroing) return;
    begin_zero_interpolation(now);
    zero_start_pos[0] = ForwardKinematics(&ctrl2, 7, 8); // FL leg
    zero_start_pos[1] = ForwardKinematics(&ctrl2, 1, 2); // RR leg
    zero_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5); // FR leg
    zero_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4); // RL leg
}

static void start_zero_interpolation_from_gait(QuadrupedGait* gait, uint32_t now, float current_time)
{
    if (is_zeroing) return;

    for (int i = 0; i < 4; i++) {
        Cycloid2D_Pose pose = get_leg_trajectory(gait, i, current_time);
        pose.y += Chassis_Get_Leg_Offset(i);
        if (pose.y < 20.0f) pose.y = 20.0f;
        if (pose.y > 32.0f) pose.y = 32.0f;
        zero_start_pos[i].X = pose.x;
        zero_start_pos[i].Y = pose.y;
    }

    begin_zero_interpolation(now);
}

/**
 * @brief 闆朵綅鎻掑€兼墽琛岋細鍦?5ms 鍛ㄦ湡涓钩婊戦€艰繎鍧愭爣
 */
void state_zero(uint32_t now, float current_kp) {
    if (!is_zeroing) return;

    float current_target_x[4];
    float current_target_y[4];
    float progress = (float)(now - zeroing_start_time) / (float)current_zeroing_duration;
    if (progress >= 1.0f) {
        progress = 1.0f;
        is_zeroing = 0;
        start_attitude_zero_calibration();
    }
    for (int i = 0; i < 4; i++) {
        current_target_x[i] = zero_start_pos[i].X + (ZERO_TARGET_X - zero_start_pos[i].X) * progress;
        current_target_y[i] = zero_start_pos[i].Y + (ZERO_TARGET_Y - zero_start_pos[i].Y) * progress;
    }
//    for (int i = 0; i < 4; i++) {
//        current_target_x[i] = zero_start_pos[i].X + (ZERO_TARGET_X - zero_start_pos[i].X) * progress;
//        current_target_y[i] = zero_start_pos[i].Y + (ZERO_TARGET_Y - zero_start_pos[i].Y) * progress;
//    }
    LegAngles angles1 = InverseKinematics(current_target_x[0], current_target_y[0], &ctrl2, 7, 8, 0.3, 0.1);
    MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles1.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles1.theta1, current_kp, 0.1);

    LegAngles angles2 = InverseKinematics(current_target_x[1], current_target_y[1], &ctrl2, 1, 2, 0.3, 0.1);
    MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles2.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles2.theta1, current_kp, 0.1);

    LegAngles angles3 = InverseKinematics(current_target_x[2], current_target_y[2], &ctrl1, 6, 5, 0.3, 0.1);
    MotorController_SetCommand(&ctrl1, 5, 1, 0.0, 0.0f, angles3.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl1, 6, 1, 0.0, 0.0f, angles3.theta1, current_kp, 0.1);

    LegAngles angles4 = InverseKinematics(current_target_x[3], current_target_y[3], &ctrl1, 3, 4, 0.3, 0.1);
    MotorController_SetCommand(&ctrl1, 4, 1, 0.0, 0.0f, angles4.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl1, 3, 1, 0.0, 0.0f, angles4.theta1, current_kp, 0.1);
}
/**
 * @brief 鑷畾涔夊垎缁勬彃鍊兼墽琛岋細骞虫粦閫艰繎涓ょ粍涓嶅悓鐨勭洰鏍囧潗鏍? * @param target_x_grp1 绗竴缁勭洰鏍?X (瀵瑰簲鍓嶈吙 0, 2)
 * @param target_y_grp1 绗竴缁勭洰鏍?Y (瀵瑰簲鍓嶈吙 0, 2)
 * @param target_x_grp2 绗簩缁勭洰鏍?X (瀵瑰簲鍚庤吙 1, 3)
 * @param target_y_grp2 绗簩缁勭洰鏍?Y (瀵瑰簲鍚庤吙 1, 3)
 */
void state_custom_interpolation(uint32_t now, float current_kp,
                                float target_x_grp1, float target_y_grp1,
                                float target_x_grp2, float target_y_grp2) {
    if (!is_custom_interp) return;

    float progress = (float)(now - custom_interp_start_time) / (float)custom_interp_duration;
    if (progress >= 1.0f) {
        progress = 1.0f;
        is_custom_interp = 0;
    }

    float current_target_x[4];
    float current_target_y[4];

    // ====== 鍒嗙粍1(鍓嶈吙)锛歴lot0(FL hip7)鍜宻lot2(FR hip6) ======
    current_target_x[0] = custom_start_pos[0].X + (target_x_grp1 - custom_start_pos[0].X) * progress;
    current_target_y[0] = custom_start_pos[0].Y + (target_y_grp1 - custom_start_pos[0].Y) * progress;

    current_target_x[2] = custom_start_pos[2].X + (target_x_grp1 - custom_start_pos[2].X) * progress;
    current_target_y[2] = custom_start_pos[2].Y + (target_y_grp1 - custom_start_pos[2].Y) * progress;

    // ====== 鍒嗙粍2(鍚庤吙)锛歴lot1(RR hip1)鍜宻lot3(RL hip3) ======
    current_target_x[1] = custom_start_pos[1].X + (target_x_grp2 - custom_start_pos[1].X) * progress;
    current_target_y[1] = custom_start_pos[1].Y + (target_y_grp2 - custom_start_pos[1].Y) * progress;

    current_target_x[3] = custom_start_pos[3].X + (target_x_grp2 - custom_start_pos[3].X) * progress;
    current_target_y[3] = custom_start_pos[3].Y + (target_y_grp2 - custom_start_pos[3].Y) * progress;

    // 涓嬪彂鎺у埗鎸囦护 (閫嗚繍鍔ㄥ瑙ｇ畻涓庣數鏈鸿緭鍑?
    LegAngles angles1 = InverseKinematics(current_target_x[0], current_target_y[0], &ctrl2, 7, 8, 0.3, 0.1);
    MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles1.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles1.theta1, current_kp, 0.1);

    LegAngles angles2 = InverseKinematics(current_target_x[1], current_target_y[1], &ctrl2, 1, 2, 0.3, 0.1);
    MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles2.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles2.theta1, current_kp, 0.1);

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
    is_zeroing = 0;
    pose_adjust_start_time = now;
    pose_start_pos[0] = ForwardKinematics(&ctrl2, 7, 8); // FL leg
    pose_start_pos[1] = ForwardKinematics(&ctrl2, 1, 2); // RR leg
    pose_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5); // FR leg
    pose_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4); // RL leg

    // 璁＄畻缁堢偣鐩爣浣嶇疆
    for (int i = 0; i < 4; i++) {
        pose_target_pos[i].X = pose_start_pos[i].X + delta_x;
        pose_target_pos[i].Y = pose_start_pos[i].Y + delta_y;
    }
}

/**
 * @brief 濮挎€佸井璋冩墽琛岋細鍦?5ms 鍛ㄦ湡涓钩婊戦€艰繎寰皟鍚庣殑鍧愭爣
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

    LegAngles angles1 = InverseKinematics(current_target_x[0], current_target_y[0], &ctrl2, 7, 8, 0.5, 0.1);
    MotorController_SetCommand(&ctrl2, 8, 1, 0.0, 0.0f, angles1.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl2, 7, 1, 0.0, 0.0f, angles1.theta1, current_kp, 0.1);

    LegAngles angles2 = InverseKinematics(current_target_x[1], current_target_y[1], &ctrl2, 1, 2, 0.5, 0.1);
    MotorController_SetCommand(&ctrl2, 2, 1, 0.0, 0.0f, angles2.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl2, 1, 1, 0.0, 0.0f, angles2.theta1, current_kp, 0.1);

    LegAngles angles3 = InverseKinematics(current_target_x[2], current_target_y[2], &ctrl1, 6, 5, 0.5, 0.1);
    MotorController_SetCommand(&ctrl1, 5, 1, 0.0, 0.0f, angles3.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl1, 6, 1, 0.0, 0.0f, angles3.theta1, current_kp, 0.1);

    LegAngles angles4 = InverseKinematics(current_target_x[3], current_target_y[3], &ctrl1, 3, 4, 0.5, 0.1);
    MotorController_SetCommand(&ctrl1, 4, 1, 0.0, 0.0f, angles4.theta2, current_kp, 0.1);
    MotorController_SetCommand(&ctrl1, 3, 1, 0.0, 0.0f, angles4.theta1, current_kp, 0.1);
}

/**
 * @brief 鎽囨潌鎺у埗澶勭悊
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
            start_zero_interpolation_from_gait(gait, now, currentTimeSec);
            stop_quadruped_gait(gait);
        } else {
            if (now - last_calc_time >= 5) {
                last_calc_time = now;
                HAL_NVIC_DisableIRQ(TIM3_IRQn);
                HAL_NVIC_DisableIRQ(TIM4_IRQn);
                angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
                angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, kp, 0.1);
                angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
                angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);
                HAL_NVIC_EnableIRQ(TIM3_IRQn);
                HAL_NVIC_EnableIRQ(TIM4_IRQn);

                // 銆愪慨鏀圭偣2銆戯細鍗曟鍔ㄤ綔鏃朵繚鎸?3508 杞瓙鐘舵€?(绾冻妯″紡淇濇寔閿佹锛屽叾浣欐ā寮忓綊闆?
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

    // State change detection
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
                    start_gait_from_stand_base(gait, currentTimeSec);
                }
                break;
            case ROBOT_STATE_BACKWARD:
                if (!is_pure_wheel_mode) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
                    calibrate_leg_base_position(gait, 2, ctrl1,6,5,0);
                    calibrate_leg_base_position(gait, 3, ctrl1,3,4,0);
                    start_gait_from_stand_base(gait, currentTimeSec);
                }
                break;
            case ROBOT_STATE_LEFT:
                if (!is_pure_wheel_mode) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,0);
                    calibrate_leg_base_position(gait, 2, ctrl1,3,4,1);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,1);
                    calibrate_leg_base_position(gait, 3, ctrl1,6,5,0);
                    start_gait_from_stand_base(gait, currentTimeSec);
                }
                break;
            case ROBOT_STATE_RIGHT:
                if (!is_pure_wheel_mode) {
                    apply_current_gait(gait);
                    calibrate_leg_base_position(gait, 0, ctrl2,7,8,1);
                    calibrate_leg_base_position(gait, 2, ctrl1,3,4,0);
                    calibrate_leg_base_position(gait, 1, ctrl2,1,2,0);
                    calibrate_leg_base_position(gait, 3, ctrl1,6,5,1);
                    start_gait_from_stand_base(gait, currentTimeSec);
                }
                break;
            case ROBOT_STATE_IDLE:
                start_zero_interpolation_from_gait(gait, now, currentTimeSec);
                stop_quadruped_gait(gait);
                break;
        }
    }

    // ... 鍓嶉潰鐨勪唬鐮?(鐘舵€佸垏鎹㈡娴嬬瓑) ...

        if (now - last_calc_time >= 5) {
            last_calc_time = now;

            // 濮挎€佽ˉ鍋?(鐢?5ms 涓诲惊鐜粺涓€璋冪敤, 杩欓噷淇濈暀鍏煎)

            if (current_state != ROBOT_STATE_IDLE) {

                // 鍖哄垎绾疆妯″紡涓庢甯告鎬佹ā寮忕殑鑵块儴杩愮畻
                if (!is_pure_wheel_mode) {
                    HAL_NVIC_DisableIRQ(TIM3_IRQn);
                    HAL_NVIC_DisableIRQ(TIM4_IRQn);
                    update_attitude_comp_for_gait(gait, currentTimeSec);
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
                    HAL_NVIC_EnableIRQ(TIM3_IRQn);
                    HAL_NVIC_EnableIRQ(TIM4_IRQn);
                } else if (attitude_comp_enabled && chassis_mode != 2) {
                    HAL_NVIC_DisableIRQ(TIM3_IRQn);
                    HAL_NVIC_DisableIRQ(TIM4_IRQn);
                    Chassis_Set_All_Leg_Weights(1.0f, 1.0f, 1.0f, 1.0f);
                    Chassis_Attitude_Loop(0.005f);
                    state_stand_attitude(kp);
                    HAL_NVIC_EnableIRQ(TIM3_IRQn);
                    HAL_NVIC_EnableIRQ(TIM4_IRQn);
                }
            } else {
                // Pure wheel mode: maintain legs at zeroed standing posture
                if (is_zeroing) {
                    state_zero(now, kp);
                } else if (is_adjusting_pose) {
                    update_pose_adjustment(now, kp);
                }else if (is_custom_interp) {
                	state_custom_interpolation(now, kp, custom_target_x_grp1, custom_target_y_grp1,custom_target_x_grp2, custom_target_y_grp2);
                } else if (attitude_comp_enabled && chassis_mode != 2) {
                    HAL_NVIC_DisableIRQ(TIM3_IRQn);
                    HAL_NVIC_DisableIRQ(TIM4_IRQn);
                    Chassis_Set_All_Leg_Weights(1.0f, 1.0f, 1.0f, 1.0f);
                    Chassis_Attitude_Loop(0.005f);
                    state_stand_attitude(kp);
                    HAL_NVIC_EnableIRQ(TIM3_IRQn);
                    HAL_NVIC_EnableIRQ(TIM4_IRQn);
                }
            }
            float linear_ratio = 0.0f; // 鍓嶅悗姣斾緥锛氬墠杩涗负姝ｏ紝鍚庨€€涓鸿礋
            if (myPacket.joy_lx < 40) {
                            linear_ratio = (40.0f - myPacket.joy_lx) / 40.0f; // 鎽囨潌瓒婇潬杩?0锛屾瘮渚嬭秺鎺ヨ繎 1.0
              } else if (myPacket.joy_lx > 60) {
                            linear_ratio = (60.0f - myPacket.joy_lx) / 40.0f; // 鎽囨潌瓒婇潬杩?100锛屾瘮渚嬭秺鎺ヨ繎 -1.0
              }

            float turn_ratio = 0.0f; // 杞悜姣斾緥锛氬彸杞负姝ｏ紝宸﹁浆涓鸿礋
            if (myPacket.joy_rx < 40) {
                            turn_ratio = (40.0f - myPacket.joy_rx) / 40.0f;
               } else if (myPacket.joy_rx > 60) {
                            turn_ratio = (60.0f - myPacket.joy_rx) / 40.0f;
                 }
            float max_base_rpm = is_pure_wheel_mode ? 2400.0f : 1800.0f;
            float max_turn_rpm = is_pure_wheel_mode ? 1800.0f : 1500.0f;
            float wheel_target_rpm_01 = (linear_ratio * max_base_rpm) + (turn_ratio * max_turn_rpm);
            float wheel_target_rpm_23 = -(linear_ratio * max_base_rpm) + (turn_ratio * max_turn_rpm);

            // ==========================================
            // 銆愪慨鏀圭偣3銆戯細鍖哄垎閫熷害鎺у埗锛堣仈鍔?绾疆锛変笌浣嶇疆鎺у埗锛堢函瓒抽攣姝伙級
            if (chassis_mode == 2) {
                // Pure foot mode: lock 3508 wheels
                for(int i = 0; i < 4; i++) {
                    Motors[i].target_angle = locked_angles[i];
                    PID_Calc_Position(i, Motors[i].target_angle);
                }
            } else {
                // 銆愯仈鍔ㄦ垨绾疆妯″紡銆戯細姝ｅ父椹卞姩杞瓙
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
            // Fix 4: In IDLE state, determine if wheel lock is needed
            if (chassis_mode == 2) {
                // 銆愮函瓒虫ā寮忋€戯細淇濇寔鍘熶綅閿佹
                for(int i = 0; i < 4; i++) {
                    Motors[i].target_angle = locked_angles[i];
                    PID_Calc_Position(i, Motors[i].target_angle);
                }
            } else {
                // 銆愯仈鍔ㄦ垨绾疆妯″紡銆戯細閫熷害褰掗浂
                for (int i = 0; i < 4; i++) {
                    Motors[i].target_speed = 0.0f;
                    PID_Calc_Speed(i);
                }
            }
            // ==========================================
        }
    }
/**
 * @brief 鎸夐敭鎺у埗閫昏緫
 */
#define DEBOUNCE_DELAY_MS 30
#define NUM_BUTTONS 8

void button_control(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t currentTime) {
    static uint8_t stable_button_states[NUM_BUTTONS] = {0};
    static uint8_t last_button_readings[NUM_BUTTONS] = {0};
    static uint32_t last_debounce_times[NUM_BUTTONS] = {0};

    float dy = 1.0f;
    float dx = 0.0f;

    // Debounce and button processing
    for (int i = 0; i < NUM_BUTTONS; i++) {
        uint8_t current_reading = (myPacket.buttons >> i) & 1;

        if (current_reading != last_button_readings[i]) {
            last_debounce_times[i] = currentTime;
        }

        if ((currentTime - last_debounce_times[i]) > DEBOUNCE_DELAY_MS) {
            if (current_reading != stable_button_states[i]) {
                stable_button_states[i] = current_reading;
                if (stable_button_states[i] == 1) {
                    switch (i) {
                        case 0:
                            // 褰掗浂
                            start_zero_interpolation(currentTime);
//                          state_zero0(currentTime);
                            break;

                        case 1:
                            // Raise chassis (Y+dy, X-dx)
                            start_pose_adjustment(currentTime, dx, dy);
                            break;

                        case 2:
                            // Lower chassis (Y-dy, X-dx)
                            start_pose_adjustment(currentTime, -dx, -dy);
                            break;

                        case 3:
                            // 銆愪慨鏀圭偣5銆戯細姣忔鎸変笅锛屽湪 0(鑱斿姩), 1(绾疆), 2(绾冻) 涔嬮棿寰幆鍒囨崲
                            chassis_mode = (chassis_mode + 1) % 3;

                            if (chassis_mode == 1) {
                                // 杩涘叆绾疆妯″紡锛氳吙閮ㄥ己鍒舵彃鍊煎綊闆?                                start_zero_interpolation(currentTime);
                            } else if (chassis_mode == 2) {
                                // 杩涘叆绾冻妯″紡鐬棿锛岃褰曞綋鍓?涓疆瀛愮殑瀹炴椂瑙掑害
                                for (int j = 0; j < 4; j++) {
                                    locked_angles[j] = Motors[j].total_angle;
                                }
                                current_gait_mode = GAIT_MODE_TROT;
                                force_gait_update = 1;
                            } else {
                                // 鎭㈠杞吙鑱斿姩
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
                        	// 鍗曟鎵ц BOUND
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
                        	// 鍗曟鎵ц PRONK
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
#define REMOTE_TIMEOUT_MS 300 // 瀹氫箟閫氫俊瓒呮椂鏃堕棿涓?300 姣锛堝彲鏍规嵁瀹為檯鍙戝寘棰戠巼璋冩暣锛?
/**
 * @brief 鎺ユ敹鏁版嵁骞跺垎鍙戞帶鍒?(鍖呭惈瓒呮椂澶辫仈淇濇姢)
 */
void AS01_rx(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
    static uint32_t last_rx_time = 0; // 闈欐€佸彉閲忥細璁板綍涓婁竴娆℃垚鍔熸帴鏀跺埌鏁版嵁鐨勬椂闂存埑

    // 1. 灏濊瘯鎺ユ敹鏁版嵁
    if (NRF24L01_RxPacket(rx_buffer) == 0) {
        memcpy(&myPacket, rx_buffer, sizeof(RemoteData_t));
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        last_rx_time = now; // 鎴愬姛鎺ユ敹锛屾洿鏂版椂闂存埑
    }
    if ((now - last_rx_time) > REMOTE_TIMEOUT_MS) {
        // 寮哄埗灏嗘憞鏉嗘暟鎹綊鑷充腑浣?(瑙﹀彂 ROBOT_STATE_IDLE)
        myPacket.joy_lx = 50;
        myPacket.joy_rx = 50;
        myPacket.joy_ly = 50;
        myPacket.joy_ry = 50;
        myPacket.buttons = 0;
    } else {
    }

    // 3. 鎵ц鎺у埗閫昏緫
    joystick_control(ctrl1, ctrl2, gait, startTime, angles, now);
    button_control(ctrl1, ctrl2, gait, now);
}






//#include "control.h"
//#include <string.h> // 纭繚 memcpy 鍙敤
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
//static uint8_t is_single_action = 0;         // 鍗曟鍔ㄤ綔鏍囧織浣?//static uint32_t single_action_end_time = 0;  // 鍗曟鍔ㄤ綔缁撴潫鏃堕棿
//static GaitMode_e previous_gait_mode = GAIT_MODE_TROT; // 璁板綍鎵ц鍔ㄤ綔鍓嶇殑姝ユ€?//static RobotState_e current_state = ROBOT_STATE_IDLE;
//static uint8_t is_zeroing = 0;               // 鏍囧織浣嶏細鏄惁姝ｅ湪鎵ц澶嶄綅鎻掑€?//static uint32_t zeroing_start_time = 0;      // 璁板綍澶嶄綅鍔ㄤ綔寮€濮嬬殑鏃堕棿鎴?//static Currentpos zero_start_pos[4];         // 璁板綍澶嶄綅鍔ㄤ綔寮€濮嬬灛闂达紝4鏉¤吙鐨勫垵濮嬪潗鏍?//static const float ZERO_TARGET_X = 10.0f;     // 闆剁偣鐩爣 X 鍧愭爣
//static const float ZERO_TARGET_Y = 25.0f;    // 闆剁偣鐩爣 Y 鍧愭爣
//static const uint32_t ZEROING_DURATION_MS = 2500; // 澶嶄綅鍔ㄤ綔鑰楁椂 (1000ms = 1绉?
//static uint32_t current_zeroing_duration = 2500; // 鏂板锛氱敤浜庤褰曞綋鍓嶈繖娆″姩浣滈渶瑕佺殑鎬绘椂闀?//static const uint32_t ZEROING_last_ms = 1000; // 澶嶄綅鍔ㄤ綔鑰楁椂 (1000ms = 1绉?
//static uint8_t is_adjusting_pose = 0;               // 鏍囧織浣嶏細鏄惁姝ｅ湪鎵ц楂樺害寰皟
//static uint32_t pose_adjust_start_time = 0;         // 璁板綍寰皟鍔ㄤ綔寮€濮嬬殑鏃堕棿鎴?//static Currentpos pose_start_pos[4];                // 寰皟鐨勮捣鐐?//static Currentpos pose_target_pos[4];               // 寰皟鐨勭粓鐐?//static const uint32_t POSE_ADJUST_DURATION_MS = 300; // 寰皟鑰楁椂 (300ms)锛岃鍔ㄤ綔杩呮嵎涓斿钩婊?//static uint8_t is_pure_wheel_mode = 0;       // 銆愭柊澧炪€戠函杞ā寮忔爣蹇椾綅锛?-杞吙鑱斿姩姝ユ€侊紝1-绾疆妯″紡
//static uint8_t is_first_zero = 0;
//static inline void apply_current_gait(QuadrupedGait* gait) {
//    float straight_period = 1.2f; // 鍛ㄦ湡 (绉?
//    float straight_length = 8.5f; // 姝ラ暱
//    float straight_height = 10.0f; // 鎶吙楂樺害
//    float turn_period = 0.8f;     // 杞悜鍛ㄦ湡 (閫氬父鏇村揩涓€鐐?
//    float turn_length = 3.0f;     // 杞悜姝ラ暱 (寤鸿鏀瑰皬锛屼互杞瓙宸€熶负涓?
//    float turn_height = 5.0f;     // 杞悜鎶吙楂樺害 (寤鸿鏀归珮锛岄槻姝㈣吙缁婁綇鍦伴潰)
//
//    // 鏍规嵁褰撳墠鐘舵€侀€夋嫨搴旂敤鐨勫弬鏁?//    float p = straight_period;
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
//            init_quadruped_gait_walk(gait, 1.2f, l, h + 2.0f); // 涓句緥锛歐alk涔熷彲浠ュ簲鐢ㄧ嫭绔嬪弬鏁?//            break;
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
// * @brief 鍚姩闆朵綅鎻掑€硷細鎹曡幏褰撳墠浣嶇疆骞堕攣瀹氱姸鎬?// */
//void start_zero_interpolation(uint32_t now) {
//    if (is_zeroing) return; // 姝ｅ湪鎻掑€间腑鍒欏拷鐣?//    is_zeroing = 1;
//    is_adjusting_pose = 0;  // 銆愰噸瑕併€戝己鍒舵墦鏂井璋冪姸鎬?//    if (is_first_zero == 0) {
//            current_zeroing_duration = ZEROING_DURATION_MS; // 2500ms
//            is_first_zero = 1; // 鏍囪浠ュ悗涓嶅啀鏄涓€娆′簡
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
// * @brief 闆朵綅鎻掑€兼墽琛岋細鍦?5ms 鍛ㄦ湡涓钩婊戦€艰繎鍧愭爣
// */
//void state_zero(uint32_t now, float current_kp) {
//    if (!is_zeroing) return;
//
//    float current_target_x[4];
//    float current_target_y[4];
//    float progress = (float)(now - zeroing_start_time) / (float)current_zeroing_duration;
//    if (progress >= 1.0f) {
//        progress = 1.0f;
//        is_zeroing = 0; // 杩涘害鍒拌揪 100%锛岀粨鏉熸彃鍊?//    }
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
//    is_zeroing = 0;        // 銆愰噸瑕併€戝己鍒舵墦鏂綊闆舵彃鍊肩姸鎬?//    pose_adjust_start_time = now;
//    pose_start_pos[0] = ForwardKinematics(&ctrl2, 1, 2);
//    pose_start_pos[1] = ForwardKinematics(&ctrl2, 7, 8);
//    pose_start_pos[2] = ForwardKinematics(&ctrl1, 6, 5);
//    pose_start_pos[3] = ForwardKinematics(&ctrl1, 3, 4);
//
//    // 璁＄畻缁堢偣鐩爣浣嶇疆
//    for (int i = 0; i < 4; i++) {
//        pose_target_pos[i].X = pose_start_pos[i].X + delta_x;
//        pose_target_pos[i].Y = pose_start_pos[i].Y + delta_y;
//    }
//}
//
///**
// * @brief 濮挎€佸井璋冩墽琛岋細鍦?5ms 鍛ㄦ湡涓钩婊戦€艰繎寰皟鍚庣殑鍧愭爣
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
// * @brief 鎽囨潌鎺у埗澶勭悊
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
//            start_zero_interpolation(now);          // 瑙﹀彂璺宠穬鍚庣殑骞虫粦澶嶄綅
//        } else {
//            if (now - last_calc_time >= 5) {
//                last_calc_time = now;
//                angles = get_leg_angles(gait, 0, currentTimeSec, ctrl2, 7, 8, kp, 0.1);
//                angles = get_leg_angles(gait, 2, currentTimeSec, ctrl1, 6, 5, kp, 0.1);
//                angles = get_leg_angles(gait, 1, currentTimeSec, ctrl2, 1, 2, kp, 0.1);
//                angles = get_leg_angles(gait, 3, currentTimeSec, ctrl1, 3, 4, kp, 0.1);
//
//                // 鍗曟鍔ㄤ綔鏃朵繚鎸?3508 杞瓙鍋滄
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
//    // 鐘舵€佸垏鎹㈡娴?//    if (target_state != current_state) {
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
//            // 銆愪慨鏀圭偣 2銆戯細鍖哄垎绾疆妯″紡涓庢甯告鎬佹ā寮忕殑鑵块儴杩愮畻
//            if (!is_pure_wheel_mode) {
//                // 淇浜嗚娉曢敊璇細蹇呴』涓よ竟閮藉甫 current_state ==
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
//                // 绾疆妯″紡涓嬪墠杩涘悗閫€鏃讹紝淇濇寔鑵块儴澶勪簬褰掗浂鍚庣殑绔欑珛濮挎€?//                if (is_zeroing) {
//                    state_zero(now, kp);
//                } else if (is_adjusting_pose) {
//                    update_pose_adjustment(now, kp);
//                }
//            }
//           float linear_ratio = 0.0f; // 鍓嶅悗姣斾緥锛氬墠杩涗负姝ｏ紝鍚庨€€涓鸿礋
//           if (myPacket.joy_lx < 40) {
//                            linear_ratio = (40.0f - myPacket.joy_lx) / 40.0f; // 鎽囨潌瓒婇潬杩?0锛屾瘮渚嬭秺鎺ヨ繎 1.0
//              } else if (myPacket.joy_lx > 60) {
//                            linear_ratio = (60.0f - myPacket.joy_lx) / 40.0f; // 鎽囨潌瓒婇潬杩?100锛屾瘮渚嬭秺鎺ヨ繎 -1.0
//              }
//
//           float turn_ratio = 0.0f; // 杞悜姣斾緥锛氬彸杞负姝ｏ紝宸﹁浆涓鸿礋
//           if (myPacket.joy_rx < 40) {
//                            turn_ratio = (40.0f - myPacket.joy_rx) / 40.0f;
//               } else if (myPacket.joy_rx > 60) {
//                            turn_ratio = (60.0f - myPacket.joy_rx) / 40.0f;
//                 }
//            float max_base_rpm = is_pure_wheel_mode ? 2400.0f : 1800.0f;
//            float max_turn_rpm = is_pure_wheel_mode ? 1800.0f : 1500.0f; // 娉ㄦ剰杩欓噷鐢ㄦ鏁拌〃绀鸿浆鍚戝箙搴?//            float wheel_target_rpm_01 = (linear_ratio * max_base_rpm) + (turn_ratio * max_turn_rpm);
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
// * @brief 鎸夐敭鎺у埗閫昏緫
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
//    // 娑堟姈涓庢寜閿鐞?//    for (int i = 0; i < NUM_BUTTONS; i++) {
//        uint8_t current_reading = (myPacket.buttons >> i) & 1;
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
//                            // 褰掗浂
//                            start_zero_interpolation(currentTime);
////                            state_zero0(currentTime);
//                            break;
//
//                        case 1:
//                            // 鍗囬珮搴曠洏 (Y+dy, X-dx) - 瑙﹀彂寰皟鎻掑€?//                            start_pose_adjustment(currentTime, dx, dy);
//                            break;
//
//                        case 2:
//                            // 闄嶄綆搴曠洏 (Y-dy, X-dx) - 瑙﹀彂寰皟鎻掑€?//                            start_pose_adjustment(currentTime, -dx, -dy);
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
//                        case 6: // 鍗曟鎵ц BOUND
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
//                        case 7: // 鍗曟鎵ц PRONK
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
//#define REMOTE_TIMEOUT_MS 300 // 瀹氫箟閫氫俊瓒呮椂鏃堕棿涓?300 姣锛堝彲鏍规嵁瀹為檯鍙戝寘棰戠巼璋冩暣锛?//
///**
// * @brief 鎺ユ敹鏁版嵁骞跺垎鍙戞帶鍒?(鍖呭惈瓒呮椂澶辫仈淇濇姢)
// */
//void AS01_rx(MotorController* ctrl1, MotorController* ctrl2, QuadrupedGait* gait, uint32_t startTime, LegAngles angles, uint32_t now) {
//    static uint32_t last_rx_time = 0; // 闈欐€佸彉閲忥細璁板綍涓婁竴娆℃垚鍔熸帴鏀跺埌鏁版嵁鐨勬椂闂存埑
//
//    // 1. 灏濊瘯鎺ユ敹鏁版嵁
//    if (NRF24L01_RxPacket(rx_buffer) == 0) {
//        memcpy(&myPacket, rx_buffer, sizeof(RemoteData_t));
//        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//        last_rx_time = now; // 鎴愬姛鎺ユ敹锛屾洿鏂版椂闂存埑
//    }
//    if ((now - last_rx_time) > REMOTE_TIMEOUT_MS) {
//        // 寮哄埗灏嗘憞鏉嗘暟鎹綊鑷充腑浣?(瑙﹀彂 ROBOT_STATE_IDLE)
//        myPacket.joy_lx = 50;
//        myPacket.joy_rx = 50;
//        myPacket.joy_ly = 50;
//        myPacket.joy_ry = 50;
//        myPacket.buttons = 0;
//    } else {
//    }
//
//    // 3. 鎵ц鎺у埗閫昏緫
//    joystick_control(ctrl1, ctrl2, gait, startTime, angles, now);
//    button_control(ctrl1, ctrl2, gait, now);
//}
