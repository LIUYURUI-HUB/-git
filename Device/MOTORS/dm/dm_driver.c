#include "dm_driver.h"
#include <string.h>

// 引用 fdcan.c 中定义的硬件句柄
extern FDCAN_HandleTypeDef hfdcan2;

DM_Motor_t Arm_Motors[4];

#define P_MIN -12.5f
#define P_MAX 12.5f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define DM_EMA_ALPHA 0.2f

//4310 电机参数
#define V_MIN_4310 -30.0f
#define V_MAX_4310 30.0f
#define T_MIN_4310 -10.0f
#define T_MAX_4310 10.0f

// 4340 电机参数
#define V_MIN_4340 -10.0f
#define V_MAX_4340 10.0f
#define T_MIN_4340 -28.0f
#define T_MAX_4340 28.0f

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    if(x < x_min) x = x_min;
    return (uint16_t) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

//初始化can通信
void FDCAN2_Filter_Init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x000;     //
    fdcan_filter.FilterID2 = 0x000;     // 全0掩码 （接收所有标准ID）

    if(HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter) != HAL_OK)
    {
        Error_Handler();
    }

    // 全局过滤配置
    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    // 启动CAN硬件（使能通信）
    if(HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }

    // 激活中断（实现异步通信）
    if(HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE |  // 接收can反馈
            FDCAN_IT_ERROR_WARNING |        // 基本错误检测
            FDCAN_IT_BUS_OFF,               // 总线错误恢复
            0) != HAL_OK)
    {
        Error_Handler();  // 中断配置失败处理
    }
}

// 初始化电机数据
void DM_Init(void){
    for(int i=0; i<4; i++) {
        Arm_Motors[i].ID = i + 1; // 默认ID: 1,2,3,4
        Arm_Motors[i].POS = 0.0f;
        Arm_Motors[i].VEL = 0.0f;
        Arm_Motors[i].Filter_VEL = 0.0f;
        Arm_Motors[i].T = 0.0f;
        Arm_Motors[i].T_MOS = 0;
        Arm_Motors[i].T_Rotor = 0;
    }
}

// 使能电机
void DM_Motor_Enable(uint16_t id)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    uint8_t TxData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    pTxHeader.Identifier = id;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    pTxHeader.DataLength = FDCAN_DLC_BYTES_8;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &pTxHeader, TxData);
}

// 失能电机
void DM_Motor_Disable(uint16_t id)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    uint8_t TxData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    pTxHeader.Identifier = id;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    pTxHeader.DataLength = FDCAN_DLC_BYTES_8;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &pTxHeader, TxData);
}

// 设置当前物理位置为电机零点 (每次上电执行)
void DM_Set_Zero_Pos(uint16_t id)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    uint8_t TxData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

    pTxHeader.Identifier = id;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    pTxHeader.DataLength = FDCAN_DLC_BYTES_8;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &pTxHeader, TxData);
}

/**
 * @brief   发送控制信息函数
 */
uint8_t DM_Send_Ctrl(uint16_t id,float p_des,float v_des,float Kp,float Kd,float t_ff)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    uint8_t TxData[8];
    float V_MAX = (id == 0x01 || id == 0x02) ? V_MAX_4340 : V_MAX_4310;
    float V_MIN = (id == 0x01 || id == 0x02) ? V_MIN_4340 : V_MIN_4310;
    float T_MAX = (id == 0x01 || id == 0x02) ? T_MAX_4340 : T_MAX_4310;
    float T_MIN = (id == 0x01 || id == 0x02) ? T_MIN_4340 : T_MIN_4310;

    uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    uint16_t kp_int = float_to_uint(Kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(Kd, KD_MIN, KD_MAX, 12);
    uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    TxData[0] = p_int >> 8;
    TxData[1] = p_int & 0xFF;
    TxData[2] = v_int >> 4;
    TxData[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    TxData[4] = kp_int & 0xFF;
    TxData[5] = kd_int >> 4;
    TxData[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    TxData[7] = t_int & 0xFF;

    // 配置基本参数
    pTxHeader.Identifier = id;                    // 设置ID
    pTxHeader.IdType = FDCAN_STANDARD_ID;        // 标准ID格式
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;    // 数据帧
    pTxHeader.DataLength = FDCAN_DLC_BYTES_8; // 数据长度 8 字节

    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

    // 等待发送邮箱空位
    uint32_t timeout_count = 0;
    while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0)
    {
        timeout_count++;
        if(timeout_count > 50000)
        {
            return 1; // 邮箱死锁，防死机强制退出
        }
    }

    // 调用HAL库发送函数
    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &pTxHeader, TxData) != HAL_OK)
    {
        return 1; // 发送失败
    }

    // 【最核心修改：强制微秒级延时防 RX FIFO 溢出】
    // 发送成功后，必须死等一小会儿（大约300~500微秒）。
    // 这保证了当前请求的电机有充足时间回复，并且STM32有时间把回复从邮箱读走，
    // 防止下一个电机的回复与其发生“连环车祸”挤爆接收邮箱。

    for (volatile uint32_t delay_i = 0; delay_i < 20000; delay_i++) {
        // 这里的空循环大约耗时几百微秒。
        // 4 个电机发完总耗时约 1.5ms，完全足够放在 4ms 的主控制循环中！
    }

    return 0; // 发送成功
}

/**
 * @brief   达妙解码函数 (针对已在上位机设置 MST ID = 物理 ID 的情况)
 */
void DM_RX_Decode(uint8_t* data, uint16_t can_id) {

    // 直接用硬件层面的 can_id 提取电机编号
    int motor_id = can_id;

    // ID 范围 0x01 ~ 0x04 对应数组下标 0 ~ 3
    int idx = motor_id - 1;

    if (idx < 0 || idx >= 4) return;

    DM_Motor_t *motor = &Arm_Motors[idx];

    // 判断型号：电机1和2是4340，电机3和4是4310
    float V_MAX = (motor_id == 0x01 || motor_id == 0x02) ? V_MAX_4340 : V_MAX_4310;
    float V_MIN = (motor_id == 0x01 || motor_id == 0x02) ? V_MIN_4340 : V_MIN_4310;
    float T_MAX = (motor_id == 0x01 || motor_id == 0x02) ? T_MAX_4340 : T_MAX_4310;
    float T_MIN = (motor_id == 0x01 || motor_id == 0x02) ? T_MIN_4340 : T_MIN_4310;

    // 1. ID 和 状态/错误码 (Byte 0)
    motor->ID = motor_id;
    motor->ERR = data[0] >> 4;

    // 2. 位置 (Byte 1-2)
    uint16_t p_int = (data[1] << 8) | data[2];
    motor->POS = uint_to_float(p_int, P_MIN, P_MAX, 16);

    // 3. 速度 (Byte 3-4)
    uint16_t v_int = (data[3] << 4) | (data[4] >> 4);
    motor->VEL = uint_to_float(v_int, V_MIN, V_MAX, 12);

    // 4.EMA 滤波速度
    motor->Filter_VEL = (DM_EMA_ALPHA * motor->VEL) + ((1.0f - DM_EMA_ALPHA) * motor->Filter_VEL);

    // 5. 扭矩 (Byte 4-5)
    uint16_t t_int = ((data[4] & 0x0F) << 8) | data[5];
    motor->T = uint_to_float(t_int, T_MIN, T_MAX, 12);

    // 6. 温度 (Byte 6-7)
    motor-> T_MOS = (int8_t)data[6];
    motor-> T_Rotor = (int8_t)data[7];
}
