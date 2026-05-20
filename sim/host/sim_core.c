#include "main.h"
#include "arm.h"
#include "arm_g.h"
#include "arm_task.h"
#include "dm_driver.h"
#include "gait.h"
#include "protocol_handler.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef _WIN32
#define SIM_EXPORT __declspec(dllexport)
#else
#define SIM_EXPORT __attribute__((visibility("default")))
#endif

#define SIM_M1_ZERO_OFFSET (-0.000190734863f)
#define SIM_M1_DIR (1.0f)
#define SIM_M2_ZERO_OFFSET (-0.000190734863f)
#define SIM_M2_DIR (-1.0f)
#define SIM_M3_ZERO_OFFSET (-0.000190734863f)
#define SIM_M3_DIR (-1.0f)
#define SIM_M4_ZERO_OFFSET (-0.000190734863f)
#define SIM_M4_DIR (1.0f)

typedef struct {
    float p_des;
    float v_des;
    float kp;
    float kd;
    float t_ff;
} SimMotorCommand_t;

typedef struct {
    uint32_t tick_ms;
    float end_x;
    float end_y;
    float end_z;
    float pan_x;
    float pan_y;
    float pan_z;
    float target_x;
    float target_y;
    float target_z;
    float feedback_x;
    float feedback_y;
    float feedback_z;
    uint8_t state_code;
    uint8_t loaded_mode;
    float payload_mass;
    uint32_t feedback_count;
    uint32_t rx_frame_count;
    float motor_pos[4];
    float motor_cmd[4];
} SimSnapshot_t;

static uint32_t sim_tick_ms = 0;
static uint32_t feedback_count = 0;
static uint32_t rx_frame_count = 0;
static float feedback_x = 0.0f;
static float feedback_y = 0.0f;
static float feedback_z = 0.0f;
static float cart_cmd_x = 0.10f;
static float cart_cmd_y = 0.0f;
static float cart_cmd_z = -0.35f;
static SimMotorCommand_t motor_cmds[4];

DM_Motor_t Arm_Motors[4];
Arm_Lengths_t arm_params = {0.0f, 0.30f, 0.32f, 0.084f};
Joint_Angles_t current_angles = {0};
End_Pos_t end_effector_pos = {0};
QuadrupedGait gait = {0};

extern ArmGravityParams_t arm_g_params;
extern float arm_payload_mass;
extern uint8_t arm_loaded_mode;

uint32_t HAL_GetTick(void)
{
    return sim_tick_ms;
}

static uint8_t sim_checksum(const uint8_t* data, uint16_t length)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < length; ++i) {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
    if (Buf == NULL || Len < 5) {
        return 1;
    }

    if (Buf[0] == PROTOCOL_HEAD1 &&
        Buf[1] == PROTOCOL_HEAD2 &&
        Buf[2] == 0x20 &&
        Buf[3] == 12 &&
        Len >= 17 &&
        sim_checksum(Buf, (uint16_t)(Len - 1)) == Buf[Len - 1]) {
        memcpy(&feedback_x, &Buf[4], sizeof(float));
        memcpy(&feedback_y, &Buf[8], sizeof(float));
        memcpy(&feedback_z, &Buf[12], sizeof(float));
        feedback_count++;
    }

    return 0;
}

uint8_t DM_Send_Ctrl(uint16_t id, float p_des, float v_des, float Kp, float Kd, float t_ff)
{
    if (id < 1 || id > 4) {
        return 1;
    }

    uint16_t index = (uint16_t)(id - 1);
    motor_cmds[index].p_des = p_des;
    motor_cmds[index].v_des = v_des;
    motor_cmds[index].kp = Kp;
    motor_cmds[index].kd = Kd;
    motor_cmds[index].t_ff = t_ff;
    return 0;
}

static float sim_approach(float current, float target, float max_delta)
{
    float diff = target - current;
    if (diff > max_delta) {
        return current + max_delta;
    }
    if (diff < -max_delta) {
        return current - max_delta;
    }
    return target;
}

static void sim_update_motors(float dt)
{
    const float max_motor_rate = 12.0f;
    float max_delta = max_motor_rate * dt;

    for (int i = 0; i < 4; ++i) {
        float old_pos = Arm_Motors[i].POS;
        float new_pos = sim_approach(old_pos, motor_cmds[i].p_des, max_delta);
        Arm_Motors[i].POS = new_pos;
        Arm_Motors[i].VEL = (dt > 0.0f) ? ((new_pos - old_pos) / dt) : 0.0f;
        Arm_Motors[i].T = motor_cmds[i].t_ff;
        Arm_Motors[i].Filter_VEL = Arm_Motors[i].VEL;
    }
}

void sim_run_arm_to_pos(float target_tx, float target_ty, float target_tz)
{
    cart_cmd_x = target_tx;
    cart_cmd_y = target_ty;
    cart_cmd_z = target_tz;
}

static void sim_set_end_pose(float x, float y, float z)
{
    end_effector_pos.x = x;
    end_effector_pos.y = y;
    end_effector_pos.z = z;
    end_effector_pos.x_pan = x;
    end_effector_pos.y_pan = y;
    end_effector_pos.z_pan = z;
}

static void sim_update_cartesian_plant(float dt)
{
    float max_linear_speed = arm_loaded_mode ? 0.14f : 0.20f;
    float max_delta = max_linear_speed * dt;
    float dx = cart_cmd_x - end_effector_pos.x;
    float dy = cart_cmd_y - end_effector_pos.y;
    float dz = cart_cmd_z - end_effector_pos.z;
    float dist = sqrtf(dx * dx + dy * dy + dz * dz);

    if (dist > max_delta && dist > 0.000001f) {
        end_effector_pos.x += (dx / dist) * max_delta;
        end_effector_pos.y += (dy / dist) * max_delta;
        end_effector_pos.z += (dz / dist) * max_delta;
    } else {
        end_effector_pos.x = cart_cmd_x;
        end_effector_pos.y = cart_cmd_y;
        end_effector_pos.z = cart_cmd_z;
    }

    end_effector_pos.x_pan = end_effector_pos.x;
    end_effector_pos.y_pan = end_effector_pos.y;
    end_effector_pos.z_pan = end_effector_pos.z;

    motor_cmds[0].p_des = atan2f(end_effector_pos.y, end_effector_pos.z == 0.0f ? 0.000001f : end_effector_pos.z);
    motor_cmds[1].p_des = end_effector_pos.x;
    motor_cmds[2].p_des = end_effector_pos.z;
    motor_cmds[3].p_des = arm_loaded_mode ? 1.0f : 0.0f;
}

SIM_EXPORT void sim_init(void)
{
    sim_tick_ms = 0;
    feedback_count = 0;
    rx_frame_count = 0;
    feedback_x = 0.0f;
    feedback_y = 0.0f;
    feedback_z = 0.0f;
    cart_cmd_x = 0.10f;
    cart_cmd_y = 0.0f;
    cart_cmd_z = -0.35f;

    memset(Arm_Motors, 0, sizeof(Arm_Motors));
    memset(&current_angles, 0, sizeof(current_angles));
    memset(&end_effector_pos, 0, sizeof(end_effector_pos));
    memset(motor_cmds, 0, sizeof(motor_cmds));

    arm_params.h = 0.0f;
    arm_params.hu = 0.30f;
    arm_params.hl = 0.32f;
    arm_params.he = 0.084f;
    arm_payload_mass = 0.0f;
    arm_loaded_mode = 0;

    for (uint8_t i = 0; i < 4; ++i) {
        Arm_Motors[i].ID = (uint8_t)(i + 1);
    }

    Arm_Gravity_InitParams(&arm_g_params);
    arm_g_params.L3 = arm_params.he;
    Protocol_Init();
    Task_Vision_ResetStateMachine();

    sim_set_end_pose(0.10f, 0.0f, -0.35f);
    for (int i = 0; i < 4; ++i) {
        motor_cmds[i].p_des = Arm_Motors[i].POS;
    }
}

SIM_EXPORT void sim_process_buffer(const uint8_t* data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return;
    }

    Protocol_ProcessBuffer((uint8_t*)data, length);
    rx_frame_count++;
}

SIM_EXPORT void sim_step(uint32_t dt_ms)
{
    if (dt_ms == 0) {
        return;
    }

    sim_tick_ms += dt_ms;
    Task_Vision_State_Machine();
    sim_update_cartesian_plant((float)dt_ms / 1000.0f);
    sim_update_motors((float)dt_ms / 1000.0f);
}

SIM_EXPORT const char* sim_get_state_name(void)
{
    return Task_Vision_GetStateName();
}

SIM_EXPORT void sim_get_snapshot(SimSnapshot_t* out)
{
    if (out == NULL) {
        return;
    }

    Arm_Target_t target = Task_Vision_GetCurrentTarget();
    memset(out, 0, sizeof(*out));
    out->tick_ms = sim_tick_ms;
    out->end_x = end_effector_pos.x;
    out->end_y = end_effector_pos.y;
    out->end_z = end_effector_pos.z;
    out->pan_x = end_effector_pos.x_pan;
    out->pan_y = end_effector_pos.y_pan;
    out->pan_z = end_effector_pos.z_pan;
    out->target_x = target.x;
    out->target_y = target.y;
    out->target_z = target.z;
    out->feedback_x = feedback_x;
    out->feedback_y = feedback_y;
    out->feedback_z = feedback_z;
    out->state_code = Task_Vision_GetStateCode();
    out->loaded_mode = arm_loaded_mode;
    out->payload_mass = arm_payload_mass;
    out->feedback_count = feedback_count;
    out->rx_frame_count = rx_frame_count;

    for (int i = 0; i < 4; ++i) {
        out->motor_pos[i] = Arm_Motors[i].POS;
        out->motor_cmd[i] = motor_cmds[i].p_des;
    }
}

SIM_EXPORT void sim_get_chassis_move(Chassis_Move_t* out)
{
    if (out == NULL) {
        return;
    }

    *out = Protocol_GetChassisMove();
}

SIM_EXPORT uint8_t sim_is_new_chassis_data(void)
{
    return Protocol_IsNewChassisData() ? 1U : 0U;
}

SIM_EXPORT void sim_clear_chassis_data_flag(void)
{
    Protocol_ClearChassisDataFlag();
}

SIM_EXPORT uint8_t sim_get_gait_switch(void)
{
    return Protocol_GetGaitSwitch();
}

SIM_EXPORT uint8_t sim_is_new_gait_data(void)
{
    return Protocol_IsNewGaitData() ? 1U : 0U;
}

SIM_EXPORT void sim_clear_gait_data_flag(void)
{
    Protocol_ClearGaitDataFlag();
}

SIM_EXPORT uint8_t sim_is_chassis_timeout(void)
{
    return Protocol_IsChassisTimeout() ? 1U : 0U;
}
