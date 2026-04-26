/*
 * arm_task.c
 *
 *  Created on: Mar 24, 2026
 *      Author: developer
 */

#include "arm_task.h"
#include "protocol_handler.h"
#include "arm.h"
#include "arm_g.h"
#include <math.h>

// 引入你的底层变量
extern End_Pos_t end_effector_pos;
extern void run_arm_to_pos(float target_tx, float target_ty, float target_tz);

struct {
    float x;
    float y;
    float z;
} arm_home = {0.10f, 0.0f, -0.35f};

struct {
    float x;
    float y;
    float z;
} arm_back_place = {0.26f, 0.10f, -0.12f};

typedef enum {
    TEST_STATE_WAIT_TARGET = 0,
    TEST_STATE_HOVER,
    TEST_STATE_DOWN,
    TEST_STATE_SUCTION_WAIT,
    TEST_STATE_LIFT,
    TEST_STATE_MOVE_TO_BACK,
    TEST_STATE_PLACE,
    TEST_STATE_RETREAT
} Test_Vision_State_t;

static Test_Vision_State_t current_test_state = TEST_STATE_WAIT_TARGET;
static Arm_Target_t current_target = {0};
static float suction_cup_l = 0.03f;
static float payload_mass = 0.5f;
static uint32_t step_timer = 0;
static uint32_t grab_timer = 0;
static uint32_t last_feedback_time = 0; // 用于控制反馈帧率

static float calc_dist(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float dx = x1 - x2;
    float dy = y1 - y2;
    float dz = z1 - z2;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

void Task_Vision_State_Machine(void) {

    uint32_t current_tick = HAL_GetTick();

    // 1. 定期将正解算出来的实时末端坐标上传，便于 Python 监控
    if (current_tick - last_feedback_time >= 20) {
        Protocol_SendArmPositionFeedback(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z);
        last_feedback_time = current_tick;
    }

    // 2. 检查是否有新的坐标指令 (0x12)
    if (Protocol_IsNewArmData()) {
        Arm_Target_t incoming_target = Protocol_GetArmTarget();
        Protocol_ClearArmDataFlag();

        // 【核心修改】：由于吸盘长度，在视觉传来的目标坐标 X 轴上减去吸盘长度
        incoming_target.x += suction_cup_l;

        // 计算新收到的补偿后坐标与当前跟踪的坐标之间的偏差 (距离差)
        float dist_diff = calc_dist(incoming_target.x, incoming_target.y, incoming_target.z,
                                    current_target.x, current_target.y, current_target.z);

        // 如果坐标变化大于 5mm (0.005m)，说明目标真的移动了，或者是全新的目标
                if (dist_diff > 0.005f) {
                    // 【修正】：去掉了 MOVE_TO_BACK，确保手里拿着东西时绝不三心二意
                    if (current_test_state == TEST_STATE_WAIT_TARGET ||
                        current_test_state == TEST_STATE_HOVER ||
                        current_test_state == TEST_STATE_RETREAT) {

                        current_target = incoming_target;
                        current_test_state = TEST_STATE_HOVER;
                        step_timer = current_tick;
                    }
                }
    }

    // 3. 自动化测试状态机 (不再等待 0x13 指令，坐标到位直接走流程)
    switch (current_test_state) {

        case TEST_STATE_WAIT_TARGET: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;
            // 没有目标时，回 HOME 点待命
            run_arm_to_pos(arm_home.x, arm_home.y, arm_home.z);
            break;
        }

        case TEST_STATE_HOVER: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;

            // 目标：走到补偿后目标正上方 8cm 处 (X轴是高度)
            run_arm_to_pos(current_target.x + 0.08f, current_target.y, current_target.z);

            // 计算末端距离“正上方点”的距离差
            float dist = calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                                   current_target.x + 0.08f, current_target.y, current_target.z);

            // 如果到了正上方 (三维误差 < 1cm)，或者超时 2.5 秒 (防止由于逆解死区卡住)，则果断触发下降
            if (dist < 0.01f || (HAL_GetTick() - step_timer > 2500)) {
                current_test_state = TEST_STATE_DOWN;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_DOWN: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;

            // 目标：垂直下降到补偿后的底部目标点，压住物体
            run_arm_to_pos(current_target.x, current_target.y, current_target.z);

            // 判断高度 (X轴) 是否已经压到底
            if (fabsf(end_effector_pos.x - current_target.x) < 0.005f) {
                current_test_state = TEST_STATE_SUCTION_WAIT;
                step_timer = current_tick; // 开始计时
            }
            break;
        }

        case TEST_STATE_SUCTION_WAIT: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;

            // 保持下压位置不变
            run_arm_to_pos(current_target.x, current_target.y, current_target.z);

            // 因为气泵一直通电，硬等 500ms 让真空吸盘贴合紧密即可
            if (current_tick - step_timer > 1200) {
                arm_payload_mass = payload_mass;
                arm_loaded_mode = 1;
                current_test_state = TEST_STATE_LIFT;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_LIFT: {
            arm_payload_mass = payload_mass;
            arm_loaded_mode = 1;
            //上抬12cm
            run_arm_to_pos(current_target.x + 0.12f, current_target.y, current_target.z);

            if (fabsf(end_effector_pos.x - (current_target.x + 0.12f)) < 0.01f ||
                (current_tick - step_timer > 3000)) {
                current_test_state = TEST_STATE_MOVE_TO_BACK;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_MOVE_TO_BACK: {
            arm_payload_mass = payload_mass;
            arm_loaded_mode = 1;

            run_arm_to_pos(arm_back_place.x+0.08f, arm_back_place.y, arm_back_place.z);

            if (calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                          arm_back_place.x+0.08f, arm_back_place.y, arm_back_place.z ) < 0.015f ||
                (current_tick - step_timer > 3500)) {
                current_test_state = TEST_STATE_PLACE;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_PLACE: {
            arm_payload_mass = payload_mass;
            arm_loaded_mode = 1;

            run_arm_to_pos(arm_back_place.x, arm_back_place.y, arm_back_place.z);

            float dist = calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                                   arm_back_place.x, arm_back_place.y, arm_back_place.z );

            if (dist < 0.012f ||
                (current_tick - step_timer > 2500)) {
                arm_payload_mass = 0.0f;
                arm_loaded_mode = 0;
                current_test_state = TEST_STATE_RETREAT;
                step_timer = current_tick;
            }
            break;
        }

        case TEST_STATE_RETREAT: {
            arm_payload_mass = 0.0f;
            arm_loaded_mode = 0;

            run_arm_to_pos(arm_home.x, arm_home.y, arm_home.z);

            float dist = calc_dist(end_effector_pos.x, end_effector_pos.y, end_effector_pos.z,
                                   arm_home.x, arm_home.y, arm_home.z);

            if (dist < 0.015f ||
                (current_tick - step_timer > 3000)) {
                current_test_state = TEST_STATE_WAIT_TARGET;
            }
            break;
        }
    }
}
