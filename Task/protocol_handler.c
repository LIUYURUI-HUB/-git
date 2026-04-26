/**
 * @file protocol_handler.c
 * @brief 通信协议处理实现 - 已修复 HS 接口与预处理错误
 */

#include "protocol_handler.h"
#include "gait.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

// 声明外部的全局步态控制结构体 (请确保在 main.c 中有定义 QuadrupedGait gait;)
extern QuadrupedGait gait;
// ==================== 私有变量 ====================

// 接收状态机
typedef enum {
    WAIT_HEAD1,
    WAIT_HEAD2,
    GET_FUNC_ID,
    GET_LENGTH,
    GET_PAYLOAD,
    GET_CHECKSUM
} Protocol_State_t;

static Protocol_State_t rx_state = WAIT_HEAD1;

// 接收缓冲区
#define RX_BUFFER_SIZE 300
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_index = 0;
static uint8_t payload_length = 0;
static uint8_t func_id = 0;

// 最新收到的指令数据
static Arm_Target_t arm_target = {0};
static Suction_Control_t suction_control = {0};
static Chassis_Move_t chassis_move = {0};
static Gait_Switch_t gait_switch = {0};

// 数据更新标志
static bool new_arm_data = false;
static bool new_suction_data = false;
static bool new_chassis_data = false;
static bool new_gait_data = false;

// 狗状态
static Dog_Status_t dog_status = {
    .dog_state = DOG_STATE_INIT,
    .error_code = ERROR_NONE
};

// 上次底盘指令时间 (ms)
static uint32_t last_chassis_time = 0;

// ==================== 宏定义补充 ====================
#ifndef FUNC_POSITION_FEEDBACK
#define FUNC_POSITION_FEEDBACK 0x20  // 匹配 Python 中的末端坐标反馈功能码
#endif

// ==================== 函数实现 ====================

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
}

/**
 * @brief 计算校验和
 */
static uint8_t CalcChecksum(uint8_t* data, uint16_t length)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief 解析载荷数据
 */
static void ParsePayload(uint8_t func_id, uint8_t* payload, uint8_t length)
{
    if (payload == NULL || length == 0) return;

    switch (func_id) {
    case FUNC_ARM_CONTROL: // 0x12 指令
        if (length >= 12) { // 3个 float = 12 字节

            memcpy(&arm_target.x, &payload[0], 4);
            memcpy(&arm_target.y, &payload[4], 4);
            memcpy(&arm_target.z, &payload[8], 4);

            new_arm_data = true;
        }
        break;

        case FUNC_SUCTION_CONTROL:  // 0x13 - 吸盘控制
            if (length >= 1) {
                suction_control.mode = payload[0];
                new_suction_data = true;
            }
            break;

        case FUNC_CHASSIS_MOVE:  // 0x10 - 底盘运动
                    if (length >= 9) {   // vx(4) + wz(4) + state(1) = 9 字节
                        memcpy(&chassis_move.vx, &payload[0], 4);
                        memcpy(&chassis_move.wz, &payload[4], 4);
                        chassis_move.state = payload[8];   // 取出第9个字节作为状态码

                        last_chassis_time = HAL_GetTick(); // 记录心跳
                    }
                    break;

          case FUNC_GAIT_SWITCH:  // 0x11 - 步态切换
              if (length >= 1) {
                  gait_switch.gait_id = payload[0];
              }
              break;

          default:
              break;
    }
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

            // 严谨校验包长上限，防止后续 GET_PAYLOAD 时数组越界 (需扣除 头、长度等5字节固定开销)
            if (payload_length > 0 && payload_length <= (RX_BUFFER_SIZE - 5)) {
                rx_state = GET_PAYLOAD;
            } else if (payload_length == 0) {
                rx_state = GET_CHECKSUM;
            } else {
                // 长度异常，复位状态机
                rx_index = 0;
                rx_state = WAIT_HEAD1;
            }
            break;

        case GET_PAYLOAD:
            rx_buffer[rx_index++] = data;
            if (rx_index >= (4 + payload_length)) {
                rx_state = GET_CHECKSUM;
            }
            break;

        case GET_CHECKSUM:
            rx_buffer[rx_index++] = data;
            {
                uint8_t calc_sum = CalcChecksum(rx_buffer, rx_index - 1);
                if (calc_sum == rx_buffer[rx_index - 1]) {
                    ParsePayload(func_id, &rx_buffer[4], payload_length);
                    if (dog_status.dog_state == DOG_STATE_IDLE) {
                        dog_status.dog_state = DOG_STATE_RUNNING;
                    }
                }
            }
            rx_index = 0;
            rx_state = WAIT_HEAD1;
            break;
    }
}

void Protocol_ProcessBuffer(uint8_t* data, uint32_t length)
{

    // 防御性编程：防止底层传入空指针或长度为0
    if (data == NULL || length == 0) return;

    for (uint32_t i = 0; i < length; i++) {
        Protocol_ProcessByte(data[i]);
    }
}

// ==================== 数据获取接口 ====================

Arm_Target_t Protocol_GetArmTarget(void) {
    Arm_Target_t temp;
    __disable_irq();     // 关中断
    temp = arm_target;   // 内存拷贝 (原子的)
    __enable_irq();      // 开中断
    return temp;
}

Chassis_Move_t Protocol_GetChassisMove(void) {
    Chassis_Move_t temp;
    __disable_irq();
    temp = chassis_move;
    __enable_irq();
    return temp;
}
uint8_t Protocol_GetSuctionControl(void) { return suction_control.mode; }
uint8_t Protocol_GetGaitSwitch(void) { return gait_switch.gait_id; }
bool Protocol_IsNewArmData(void) { return new_arm_data; }
bool Protocol_IsNewSuctionData(void) { return new_suction_data; }
void Protocol_ClearArmDataFlag(void) { new_arm_data = false; }
void Protocol_ClearSuctionDataFlag(void) { new_suction_data = false; }
Dog_Status_t* Protocol_GetDogStatus(void) { return &dog_status; }
void Protocol_SetDogState(uint8_t state) { dog_status.dog_state = state; }
void Protocol_SetErrorCode(uint8_t error) { dog_status.error_code = error; }

// ==================== 发送函数 (High Speed) ====================

// 声明底层 HS 发送函数
extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);

int Protocol_SendJointFeedback(Joint_Feedback_t* feedback)
{
    if (feedback == NULL) return -1;

    // 修复：使用 sizeof(Joint_Feedback_t) 动态获取大小，避免以后修改结构体忘记改 76 导致的局部数组越界
    uint8_t payload_len = sizeof(Joint_Feedback_t);
    uint8_t tx_buffer[4 + sizeof(Joint_Feedback_t) + 1];
    uint16_t idx = 0;

    tx_buffer[idx++] = PROTOCOL_HEAD1;
    tx_buffer[idx++] = PROTOCOL_HEAD2;
    tx_buffer[idx++] = FUNC_JOINT_FEEDBACK;
    tx_buffer[idx++] = payload_len;

    memcpy(&tx_buffer[idx], feedback, payload_len);
    idx += payload_len;

    tx_buffer[idx] = CalcChecksum(tx_buffer, idx);
    idx++;

    // 调用 HS 接口
    return (int)CDC_Transmit_HS(tx_buffer, idx);
}

int Protocol_SendErrorReport(Dog_Status_t* status)
{
    if (status == NULL) return -1;

    uint8_t tx_buffer[4 + 2 + 1];
    uint16_t idx = 0;

    tx_buffer[idx++] = PROTOCOL_HEAD1;
    tx_buffer[idx++] = PROTOCOL_HEAD2;
    tx_buffer[idx++] = FUNC_ERROR_REPORT;
    tx_buffer[idx++] = 2;

    tx_buffer[idx++] = status->dog_state;
    tx_buffer[idx++] = status->error_code;

    tx_buffer[idx] = CalcChecksum(tx_buffer, idx);
    idx++;

    // 修正：此处已改为调用 CDC_Transmit_HS
    return (int)CDC_Transmit_HS(tx_buffer, idx);
}

/**
 * @brief 【新增】向视觉节点实时反馈机械臂末端的正解物理坐标 (0x20)
 * @param x 高度
 * @param y 侧向
 * @param z 前向
 */
int Protocol_SendArmPositionFeedback(float x, float y, float z)
{
    uint8_t tx_buffer[4 + 12 + 1]; // 头尾开销 + 3个float(12字节)
    uint16_t idx = 0;

    tx_buffer[idx++] = PROTOCOL_HEAD1;
    tx_buffer[idx++] = PROTOCOL_HEAD2;
    tx_buffer[idx++] = FUNC_POSITION_FEEDBACK; // 0x20
    tx_buffer[idx++] = 12; // payload length = 12

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

/**
 * @brief 检查底盘超时
 */
bool Protocol_IsChassisTimeout(void)
{
    if (last_chassis_time == 0) return false;
    uint32_t current_time = HAL_GetTick();
    if (current_time >= last_chassis_time) {
        return (current_time - last_chassis_time) > 100;
    } else {
        return (0xFFFFFFFF - last_chassis_time + current_time) > 100;
    }
}
