/*
 * arm_task.c
 *
 * Created on: Mar 24, 2026
 * Author: developer
 */

#include "arm_task.h"
#include "protocol_handler.h"
#include "arm.h"
#include "arm_g.h"
#include <math.h>

// =========================================================================
// 引入底层运动学和状态变量
// =========================================================================
extern End_Pos_t end_effector_pos;
extern Joint_Angles_t current_angles;
extern void run_arm_to_pos(float target_tx, float target_ty, float target_tz);

#ifdef HOST_SIM
extern void sim_run_arm_to_pos(float target_tx, float target_ty, float target_tz);
#define run_arm_to_pos sim_run_arm_to_pos
#endif

// 载荷与运输模式外部变量 (对应 arm.c 中的定义)
extern float arm_payload_mass;
extern uint8_t arm_loaded_mode;
extern uint8_t arm_transport_mode;
extern float transport_alpha_ref;
extern float transport_beta_ref;
extern float transport_omega_ref;
extern float transport_L_ref;
extern float last_ik_L_val;

// =========================================================================
// 【终极物理包络与几何安全常量定义】
// =========================================================================

// 物资箱尺寸 (250mm正方体)
#define BOX_SIZE             (0.250f)
#define BOX_HALF_SIZE        (0.125f)

// 机械狗机身尺寸 (宽400mm)
#define BODY_HALF_WIDTH      (0.200f)
#define BODY_SIDE_MARGIN     (0.035f)

// 1. 安全侧向绕行点 (0.200机身 + 0.125箱体 + 0.035余量 = 0.360m)
#define Y_SAFE_SIDE          (0.36f)

// 2. 搬运安全过顶高度 (箱底离狗背 5cm 间隙，即 0.25 + 0.05 = 0.30m)
#define TRANSPORT_X          (0.30f)

// 3. 狗背平面高度与放置高度
#define DOG_BACK_X           (0.000f)
#define SUCTION_PLACE_X      (0.250f) // 小臂末端下压到 0.25m 时，箱底刚好压在 0.00m 的狗背上

// 4. 放置时的 Y 轴偏移 (救命参数！0.015f 用于避开底座转到 -3.1415 的物理限位死区)
#define PLACE_Y_OFFSET       (0.015f)
#define ALIGN_EXTRA_X        (0.005f)

// 5. 越过机身前后的 Z 轴分界线
#define CROSS_Z              (0.000f)

// =========================================================================
// 状态机到位阈值
// =========================================================================

// 空载回 HOME，当前实测已到毫米级，因此收紧
#define HOME_POS_TOL         (0.006f)

// 抓取前悬停点，不能太松，否则下降点会偏
#define HOVER_POS_TOL        (0.008f)

// 下降抓箱会被物资箱物理抵住，必须保留较大 X 方向容错
#define DOWN_X_TOL           (0.025f)

// 抬升阶段主要判断高度，带箱后允许略宽
#define LIFT_X_TOL           (0.015f)

// 带箱绕行路径点，核心是安全通过，不追求毫米级停点
#define TRANSPORT_POS_TOL    (0.030f)

// 狗背上方横向切入，带箱且接近机身，适当收紧但不要卡死
#define ALIGN_POS_TOL        (0.020f)

// 放置阶段存在箱体接触、吸盘压缩、狗背高度误差
#define PLACE_POS_TOL        (0.015f)

// 新目标变化阈值
#define NEW_TARGET_TOL       (0.005f)

// =========================================================================

struct {
    float x;
    float y;
    float z;
} arm_home = {0.10f, 0.0f, -0.35f};

// 使用推导出的物理常量定义放置点
struct {
    float x;
    float y;
    float z;
} arm_back_place = {SUCTION_PLACE_X, PLACE_Y_OFFSET, 0.375f};

// 状态机枚举：包含 L 型轨迹切入/切出状态
typedef enum {
    TEST_STATE_WAIT_TARGET = 0,
    TEST_STATE_HOVER,
    TEST_STATE_DOWN,
    TEST_STATE_SUCTION_WAIT,
    TEST_STATE_LIFT,
    TEST_STATE_MOVE_WAYPOINT,     // 移出到侧向安全区并跨越前后分界线
    TEST_STATE_MOVE_TO_BACK,      // 沿侧向安全区向后平移到目标 Z 附近
    TEST_STATE_ALIGN_PLACE,       // 【正交切入】在安全高度横向切入中心线
    TEST_STATE_PLACE,             // 垂直下放
    TEST_STATE_RETREAT_ALIGN,     // 【正交切出】起升并横向切出中心线到安全侧
    TEST_STATE_RETREAT_WAYPOINT,  // 沿侧向安全区向前平移跨越分界线
    TEST_STATE_RETREAT
} Test_Vision_State_t;

static Test_Vision_State_t current_test_state = TEST_STATE_WAIT_TARGET;
static Arm_Target_t current_target = {0};
static float suction_cup_l = 0.03f;
static float payload_mass = 0.5f;
static uint32_t step_timer = 0;
static uint32_t last_feedback_time = 0;

uint8_t Task_Vision_GetStateCode(void)
{
    return (uint8_t)current_test_state;
}

const char* Task_Vision_GetStateName(void)
{
    switch (current_test_state) {
        case TEST_STATE_WAIT_TARGET: return "WAIT_TARGET";
        case TEST_STATE_HOVER: return "HOVER";
        case TEST_STATE_DOWN: return "DOWN";
        case TEST_STATE_SUCTION_WAIT: return "SUCTION_WAIT";
        case TEST_STATE_LIFT: return "LIFT";
        case TEST_STATE_MOVE_WAYPOINT: return "MOVE_WAYPOINT";
        case TEST_STATE_MOVE_TO_BACK: return "MOVE_TO_BACK";
        case TEST_STATE_ALIGN_PLACE: return "ALIGN_PLACE";
        case TEST_STATE_PLACE: return "PLACE";
        case TEST_STATE_RETREAT_ALIGN: return "RETREAT_ALIGN";
        case TEST_STATE_RETREAT_WAYPOINT: return "RETREAT_WAYPOINT";
        case TEST_STATE_RETREAT: return "RETREAT";
        default: return "UNKNOWN";
    }
}

Arm_Target_t Task_Vision_GetCurrentTarget(void)
{
    return current_target;
}

static void clear_transport_ref(void)
{
    arm_transport_mode = 0;
    transport_alpha_ref = 0.0f;
    transport_beta_ref  = 0.0f;
    transport_omega_ref = 0.0f;
    transport_L_ref     = 0.0f;
}

void Task_Vision_ResetStateMachine(void)
{
    current_test_state = TEST_STATE_WAIT_TARGET;
    current_target.x = 0.0f;
    current_target.y = 0.0f;
    current_target.z = 0.0f;
    step_timer = 0;
    last_feedback_time = 0;

    arm_payload_mass = 0.0f;
    arm_loaded_mode = 0;
    clear_transport_ref();
}

static float calc_dist(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float dx = x1 - x2;
    float dy = y1 - y2;
    float dz = z1 - z2;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

void Task_Vision_State_Machine(void)
{
    uint32_t current_tick = HAL_GetTick();

    if (current_tick - last_feedback_time >= 20) {
        Protocol_SendArmPositionFeedback(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z);
        last_feedback_time = current_tick;
    }

    if (Protocol_IsNewArmData()) {
        Arm_Target_t incoming_target = Protocol_GetArmTarget();
        Protocol_ClearArmDataFlag();

        // 视觉坐标按当前工程约定，在 X 方向加入吸盘长度补偿
        incoming_target.x += suction_cup_l;

        float dist_diff = calc_dist(incoming_target.x, incoming_target.y, incoming_target.z,
                                    current_target.x, current_target.y, current_target.z);

        if (dist_diff > NEW_TARGET_TOL) {
            // 确保手里拿着箱子时，绝不被新目标打断
            if (current_test_state == TEST_STATE_WAIT_TARGET ||
                current_test_state == TEST_STATE_HOVER ||
                current_test_state == TEST_STATE_RETREAT) {

                current_target = incoming_target;
                current_test_state = TEST_STATE_HOVER;
                step_timer = current_tick;
            }
        }
    }

    switch (current_test_state) {

        case TEST_STATE_WAIT_TARGET: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;
            clear_transport_ref();

            run_arm_to_pos(arm_home.x, arm_home.y, arm_home.z);
            break;
        }

        case TEST_STATE_HOVER: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;
            clear_transport_ref();

            // 悬停在抓取点上方 10cm
            float hover_x = current_target.x + 0.10f;
            run_arm_to_pos(hover_x, current_target.y, current_target.z);

            float dist = calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                                   hover_x, current_target.y, current_target.z);

            if (dist < HOVER_POS_TOL || (current_tick - step_timer > 2000)) {
                current_test_state = TEST_STATE_DOWN;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_DOWN: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;
            clear_transport_ref();

            // 往下扎紧
            run_arm_to_pos(current_target.x, current_target.y, current_target.z);

            // 【物理接触判定】：箱子会抵住末端，因此只看 X 方向并保留较大容错
            if (fabsf(end_effector_pos.x - current_target.x) < DOWN_X_TOL ||
                (current_tick - step_timer > 1500)) {

                current_test_state = TEST_STATE_SUCTION_WAIT;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_SUCTION_WAIT: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;
            clear_transport_ref();

            run_arm_to_pos(current_target.x, current_target.y, current_target.z);

            if (current_tick - step_timer > 1200) {
                // 确认吸牢，开启载荷补偿
                arm_payload_mass = payload_mass;
                arm_loaded_mode = 1;

                // 锁定当前 IK 姿态作为运输约束，避免翻箱子
                transport_alpha_ref = current_angles.alpha;
                transport_beta_ref  = current_angles.beta;
                transport_omega_ref = current_angles.omega;
                transport_L_ref     = last_ik_L_val;
                arm_transport_mode = 1;

                current_test_state = TEST_STATE_LIFT;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_LIFT: {
            arm_payload_mass = payload_mass;
            arm_loaded_mode = 1;
            arm_transport_mode = 1;

            // 确保起升高度不低于安全过顶高度 TRANSPORT_X
            float target_lift_x = current_target.x + 0.12f;
            if (target_lift_x < TRANSPORT_X) {
                target_lift_x = TRANSPORT_X;
            }

            run_arm_to_pos(target_lift_x, current_target.y, current_target.z);

            if (fabsf(end_effector_pos.x - target_lift_x) < LIFT_X_TOL ||
                (current_tick - step_timer > 2800)) {

                current_test_state = TEST_STATE_MOVE_WAYPOINT;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_MOVE_WAYPOINT: {
            arm_payload_mass = payload_mass;
            arm_loaded_mode = 1;
            arm_transport_mode = 1;

            // L型轨迹段一：在前方空间，横向拉出到安全侧边
            run_arm_to_pos(TRANSPORT_X, Y_SAFE_SIDE, CROSS_Z);

            if (calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                          TRANSPORT_X, Y_SAFE_SIDE, CROSS_Z) < TRANSPORT_POS_TOL ||
                (current_tick - step_timer > 3500)) {

                current_test_state = TEST_STATE_MOVE_TO_BACK;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_MOVE_TO_BACK: {
            arm_payload_mass = payload_mass;
            arm_loaded_mode = 1;
            arm_transport_mode = 1;

            // L型轨迹段二：贴着安全侧边，纵向平移到狗背对应位置
            run_arm_to_pos(TRANSPORT_X, Y_SAFE_SIDE, arm_back_place.z);

            if (calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                          TRANSPORT_X, Y_SAFE_SIDE, arm_back_place.z) < TRANSPORT_POS_TOL ||
                (current_tick - step_timer > 4500)) {

                current_test_state = TEST_STATE_ALIGN_PLACE;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_ALIGN_PLACE: {
            arm_payload_mass = payload_mass;
            arm_loaded_mode = 1;
            arm_transport_mode = 1;

            // L型轨迹段三：在后方空间，安全高度下横向切入放置点正上方
            // 这里额外抬高 5mm，给箱体和机身之间多一点余量
            float align_x = TRANSPORT_X + ALIGN_EXTRA_X;
            run_arm_to_pos(align_x, arm_back_place.y, arm_back_place.z);

            if (calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                          align_x, arm_back_place.y, arm_back_place.z) < ALIGN_POS_TOL ||
                (current_tick - step_timer > 2800)) {

                current_test_state = TEST_STATE_PLACE;
                step_timer = current_tick;
            }
            break;
        }


        case TEST_STATE_PLACE: {
            arm_payload_mass = payload_mass;
            arm_loaded_mode = 1;
            arm_transport_mode = 1;

            // L型轨迹段四：垂直下放至狗背放置高度
            run_arm_to_pos(arm_back_place.x, arm_back_place.y, arm_back_place.z);

            float dist = calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                                   arm_back_place.x, arm_back_place.y, arm_back_place.z);

            // 柔顺接触判定
            if (dist < PLACE_POS_TOL || (current_tick - step_timer > 3000)) {
                // 卸载箱子
                arm_payload_mass = 0.0f;
                arm_loaded_mode = 0;

                // 退出姿态锁定
                clear_transport_ref();

                current_test_state = TEST_STATE_RETREAT_ALIGN;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_RETREAT_ALIGN: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;
            clear_transport_ref();

            // 撤退段一：空载起升，并横向退回安全侧边
            run_arm_to_pos(TRANSPORT_X, Y_SAFE_SIDE, arm_back_place.z);

            if (calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                          TRANSPORT_X, Y_SAFE_SIDE, arm_back_place.z) < TRANSPORT_POS_TOL ||
                (current_tick - step_timer > 4000)) {

                current_test_state = TEST_STATE_RETREAT_WAYPOINT;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_RETREAT_WAYPOINT: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;
            clear_transport_ref();

            // 撤退段二：沿安全侧边向前平移跨越中线
            run_arm_to_pos(TRANSPORT_X, Y_SAFE_SIDE, CROSS_Z);

            if (calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                          TRANSPORT_X, Y_SAFE_SIDE, CROSS_Z) < TRANSPORT_POS_TOL ||
                (current_tick - step_timer > 4000)) {

                current_test_state = TEST_STATE_RETREAT;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_RETREAT: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;
            clear_transport_ref();

            run_arm_to_pos(arm_home.x, arm_home.y, arm_home.z);

            float dist = calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                                   arm_home.x, arm_home.y, arm_home.z);

            if (dist < HOME_POS_TOL || (current_tick - step_timer > 3500)) {
                current_test_state = TEST_STATE_WAIT_TARGET;
            }
            break;
        }
    }
}
