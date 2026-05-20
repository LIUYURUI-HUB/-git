#include "protocol_handler.h"
#include "control.h"
#include "gait.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>

extern QuadrupedGait gait;

typedef enum {
    WAIT_HEAD1,
    WAIT_HEAD2,
    GET_FUNC_ID,
    GET_LENGTH,
    GET_PAYLOAD,
    GET_CHECKSUM
} Protocol_State_t;

static Protocol_State_t rx_state = WAIT_HEAD1;

#define RX_BUFFER_SIZE 300
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_index = 0;
static uint8_t payload_length = 0;
static uint8_t func_id = 0;

static Arm_Target_t arm_target = {0};
static Suction_Control_t suction_control = {0};
static Chassis_Move_t chassis_move = {0};
static Gait_Switch_t gait_switch = {0};

static bool new_arm_data = false;
static bool new_suction_data = false;
static bool new_chassis_data = false;
static bool new_gait_data = false;

static Dog_Status_t dog_status = {
    .dog_state = DOG_STATE_INIT,
    .error_code = ERROR_NONE
};

static uint32_t last_chassis_time = 0;
static bool has_chassis_command = false;
static bool chassis_timeout_latched = false;

#ifndef FUNC_POSITION_FEEDBACK
#define FUNC_POSITION_FEEDBACK 0x20
#endif

static uint8_t CalcChecksum(uint8_t* data, uint16_t length)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

static void ParsePayload(uint8_t current_func_id, uint8_t* payload, uint8_t length)
{
    if (payload == NULL) {
        return;
    }

    switch (current_func_id) {
    case FUNC_ARM_CONTROL:
        if (length >= 12) {
            memcpy(&arm_target.x, &payload[0], 4);
            memcpy(&arm_target.y, &payload[4], 4);
            memcpy(&arm_target.z, &payload[8], 4);
            new_arm_data = true;
        }
        break;

    case FUNC_SUCTION_CONTROL:
        if (length >= 1) {
            suction_control.mode = payload[0];
            new_suction_data = true;
        }
        break;

    case FUNC_CHASSIS_MOVE:
        /*
         * Preferred payload:
         *   vx(float) + vy(float) + wz(float) + state(uint8_t) = 13 bytes
         *
         * Backward compatibility:
         *   vx(float) + wz(float) + state(uint8_t) = 9 bytes
         */
        if (length >= 13) {
            memcpy(&chassis_move.vx, &payload[0], 4);
            memcpy(&chassis_move.vy, &payload[4], 4);
            memcpy(&chassis_move.wz, &payload[8], 4);
            chassis_move.state = payload[12];
            new_chassis_data = true;
            last_chassis_time = HAL_GetTick();
            has_chassis_command = true;
            chassis_timeout_latched = false;
            g_robot_diag.vision_rx_count++;
            g_robot_diag.vision_last_rx_time = last_chassis_time;
        } else if (length >= 9) {
            memcpy(&chassis_move.vx, &payload[0], 4);
            chassis_move.vy = 0.0f;
            memcpy(&chassis_move.wz, &payload[4], 4);
            chassis_move.state = payload[8];
            new_chassis_data = true;
            last_chassis_time = HAL_GetTick();
            has_chassis_command = true;
            chassis_timeout_latched = false;
            g_robot_diag.vision_rx_count++;
            g_robot_diag.vision_last_rx_time = last_chassis_time;
        }
        break;

    case FUNC_GAIT_SWITCH:
        if (length >= 1) {
            gait_switch.gait_id = payload[0];
            new_gait_data = true;
        }
        break;

    default:
        break;
    }
}

void Protocol_Init(void)
{
    rx_state = WAIT_HEAD1;
    rx_index = 0;
    payload_length = 0;
    func_id = 0;

    memset(&arm_target, 0, sizeof(arm_target));
    memset(&suction_control, 0, sizeof(suction_control));
    memset(&chassis_move, 0, sizeof(chassis_move));
    memset(&gait_switch, 0, sizeof(gait_switch));

    new_arm_data = false;
    new_suction_data = false;
    new_chassis_data = false;
    new_gait_data = false;

    dog_status.dog_state = DOG_STATE_IDLE;
    dog_status.error_code = ERROR_NONE;
    last_chassis_time = 0;
    has_chassis_command = false;
    chassis_timeout_latched = false;
}

void Protocol_ProcessByte(uint8_t data)
{
    switch (rx_state) {
    case WAIT_HEAD1:
        if (data == PROTOCOL_HEAD1) {
            rx_index = 0;
            rx_buffer[rx_index++] = data;
            rx_state = WAIT_HEAD2;
        }
        break;

    case WAIT_HEAD2:
        if (data == PROTOCOL_HEAD2) {
            rx_buffer[rx_index++] = data;
            rx_state = GET_FUNC_ID;
        } else {
            rx_index = 0;
            if (data == PROTOCOL_HEAD1) {
                rx_buffer[rx_index++] = data;
                rx_state = WAIT_HEAD2;
            } else {
                rx_state = WAIT_HEAD1;
            }
        }
        break;

    case GET_FUNC_ID:
        func_id = data;
        rx_buffer[rx_index++] = data;
        rx_state = GET_LENGTH;
        break;

    case GET_LENGTH:
        payload_length = data;
        rx_buffer[rx_index++] = data;

        if (payload_length == 0) {
            rx_state = GET_CHECKSUM;
        } else if (payload_length <= (RX_BUFFER_SIZE - 5)) {
            rx_state = GET_PAYLOAD;
        } else {
            rx_index = 0;
            rx_state = WAIT_HEAD1;
        }
        break;

    case GET_PAYLOAD:
        rx_buffer[rx_index++] = data;
        if (rx_index >= (uint16_t)(4 + payload_length)) {
            rx_state = GET_CHECKSUM;
        }
        break;

    case GET_CHECKSUM: {
        uint8_t calc_sum;
        rx_buffer[rx_index++] = data;
        calc_sum = CalcChecksum(rx_buffer, (uint16_t)(rx_index - 1));
        if (calc_sum == rx_buffer[rx_index - 1]) {
            ParsePayload(func_id, &rx_buffer[4], payload_length);
            if (dog_status.dog_state == DOG_STATE_IDLE) {
                dog_status.dog_state = DOG_STATE_RUNNING;
            }
        } else {
            g_robot_diag.vision_checksum_error_count++;
        }
        rx_index = 0;
        rx_state = WAIT_HEAD1;
        break;
    }

    default:
        rx_index = 0;
        rx_state = WAIT_HEAD1;
        break;
    }
}

void Protocol_ProcessBuffer(uint8_t* data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return;
    }

    for (uint32_t i = 0; i < length; i++) {
        Protocol_ProcessByte(data[i]);
    }
}

Arm_Target_t Protocol_GetArmTarget(void)
{
    Arm_Target_t temp;
    __disable_irq();
    temp = arm_target;
    __enable_irq();
    return temp;
}

uint8_t Protocol_GetSuctionControl(void)
{
    return suction_control.mode;
}

Chassis_Move_t Protocol_GetChassisMove(void)
{
    Chassis_Move_t temp;
    __disable_irq();
    temp = chassis_move;
    __enable_irq();
    return temp;
}

uint8_t Protocol_GetGaitSwitch(void)
{
    return gait_switch.gait_id;
}

bool Protocol_IsNewArmData(void)
{
    return new_arm_data;
}

bool Protocol_IsNewSuctionData(void)
{
    return new_suction_data;
}

bool Protocol_IsNewChassisData(void)
{
    return new_chassis_data;
}

bool Protocol_IsNewGaitData(void)
{
    return new_gait_data;
}

void Protocol_ClearArmDataFlag(void)
{
    new_arm_data = false;
}

void Protocol_ClearSuctionDataFlag(void)
{
    new_suction_data = false;
}

void Protocol_ClearChassisDataFlag(void)
{
    new_chassis_data = false;
}

void Protocol_ClearGaitDataFlag(void)
{
    new_gait_data = false;
}

Dog_Status_t* Protocol_GetDogStatus(void)
{
    return &dog_status;
}

void Protocol_SetDogState(uint8_t state)
{
    dog_status.dog_state = state;
}

void Protocol_SetErrorCode(uint8_t error)
{
    dog_status.error_code = error;
}

extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);

int Protocol_SendJointFeedback(Joint_Feedback_t* feedback)
{
    uint8_t payload_len;
    uint8_t tx_buffer[4 + sizeof(Joint_Feedback_t) + 1];
    uint16_t idx = 0;

    if (feedback == NULL) {
        return -1;
    }

    payload_len = (uint8_t)sizeof(Joint_Feedback_t);
    tx_buffer[idx++] = PROTOCOL_HEAD1;
    tx_buffer[idx++] = PROTOCOL_HEAD2;
    tx_buffer[idx++] = FUNC_JOINT_FEEDBACK;
    tx_buffer[idx++] = payload_len;
    memcpy(&tx_buffer[idx], feedback, payload_len);
    idx += payload_len;
    tx_buffer[idx] = CalcChecksum(tx_buffer, idx);
    idx++;

    return (int)CDC_Transmit_HS(tx_buffer, idx);
}

int Protocol_SendErrorReport(Dog_Status_t* status)
{
    uint8_t tx_buffer[4 + 2 + 1];
    uint16_t idx = 0;

    if (status == NULL) {
        return -1;
    }

    tx_buffer[idx++] = PROTOCOL_HEAD1;
    tx_buffer[idx++] = PROTOCOL_HEAD2;
    tx_buffer[idx++] = FUNC_ERROR_REPORT;
    tx_buffer[idx++] = 2;
    tx_buffer[idx++] = status->dog_state;
    tx_buffer[idx++] = status->error_code;
    tx_buffer[idx] = CalcChecksum(tx_buffer, idx);
    idx++;

    return (int)CDC_Transmit_HS(tx_buffer, idx);
}

int Protocol_SendArmPositionFeedback(float x, float y, float z)
{
    uint8_t tx_buffer[4 + 12 + 1];
    uint16_t idx = 0;

    tx_buffer[idx++] = PROTOCOL_HEAD1;
    tx_buffer[idx++] = PROTOCOL_HEAD2;
    tx_buffer[idx++] = FUNC_POSITION_FEEDBACK;
    tx_buffer[idx++] = 12;

    memcpy(&tx_buffer[idx], &x, 4);
    idx += 4;
    memcpy(&tx_buffer[idx], &y, 4);
    idx += 4;
    memcpy(&tx_buffer[idx], &z, 4);
    idx += 4;

    tx_buffer[idx] = CalcChecksum(tx_buffer, idx);
    idx++;

    return (int)CDC_Transmit_HS(tx_buffer, idx);
}

bool Protocol_IsChassisTimeout(void)
{
    uint32_t current_time;
    bool is_timeout;

    if (!has_chassis_command) {
        return false;
    }

    current_time = HAL_GetTick();
    if (current_time >= last_chassis_time) {
        is_timeout = (current_time - last_chassis_time) > 100U;
    } else {
        is_timeout = (0xFFFFFFFFU - last_chassis_time + current_time) > 100U;
    }

    if (is_timeout && !chassis_timeout_latched) {
        g_robot_diag.vision_timeout_count++;
        chassis_timeout_latched = true;
    }

    return is_timeout;
}
