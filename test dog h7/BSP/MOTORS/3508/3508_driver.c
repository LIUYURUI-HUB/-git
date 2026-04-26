#include "3508_driver.h"
#include <string.h>
#include <math.h> // 需要包含 math.h 以支持 fabs
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1; // STM32H7
extern FDCAN_HandleTypeDef hfdcan3;
void Clear_Motor_PID(int motor_id) {
    // 1. 清空所有 PID 历史积分和微分误差 (解决抽搐的核心！)
    Motors[motor_id].speed_err_sum = 0.0f;
    Motors[motor_id].position_err_sum = 0.0f;
    Motors[motor_id].torque_err_sum = 0.0f;
    Motors[motor_id].err_last = 0.0f;

    // 2. 清空目标指令，防止一切换模式就猛转
    Motors[motor_id].target_speed = 0.0f;
    Motors[motor_id].target_angle = Motors[motor_id].total_angle; // 如果要切入位置锁死，把目标设为当前实际角度

    // 3. 切断当前输出电流
    Motors[motor_id].Out_Current = 0;
}
// 全局变量定义
Motor_3508_T Motors[4];

// PID 参数初始化
PID_3508_T Speed_PID = {
    .Kp = 3.0f,
    .Ki = 0.3f,
    .Kd = 0.0f,
    .Max_Out = 16000.0f,
    .Max_Sum = 10000.0f
};

PID_3508_T Position_PID = {
    .Kp = 1.0f,    // 需调试：越大响应越快，太大会震荡
    .Ki = 0.03f,
    .Kd = 0.15f,   // 需调试：阻尼项，防止超调
    .Max_Out = 6000.0f, // 最大转速限制 (RPM)
    .Max_Sum = 0.0f
};

// 力矩环（电流环）PID参数
PID_3508_T Torque_PID = {
    .Kp = 0.5f,
    .Ki = 0.01f,
    .Kd = 0.0f,
    .Max_Out = 16000.0f,
    .Max_Sum = 8000.0f
};

//功率限制参数
#define POWER_LIMIT 162.0f   // 限制最大功率 163W
#define Kt   0.0156f         // 转矩常数 (Nm/A) 加了减速比
#define Int_Current   819.2f // 16384/20 = 819.2 (Value/A)  C620电调: 16384 = 20A

// EMA 滤波系数
#define D3508_EMA_ALPHA 0.3f
static const float Ts = 0.005f;

int16_t Power_Limit(float desire_current, float rpm);

// 1.CAN配置
void FDCAN1_Filter_Init(void) {
    FDCAN_FilterTypeDef sFilterConfig;

    // 配置全局过滤器
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x200;//基准ID
    sFilterConfig.FilterID2 = 0x7F0;//掩码

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();

    // 开启“新消息”中断
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();
}


void FDCAN3_Filter_Init(void) {
    FDCAN_FilterTypeDef sFilterConfig;

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x200;//基准ID
    sFilterConfig.FilterID2 = 0x7F0;//掩码

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) Error_Handler();

    // 开启“新消息”中断
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();
}

// 2.初始化电机数据
void D3508_Init(void) {
    for(int i=0; i<4; i++) {
        Motors[i].last_angle = 0;
        Motors[i].total_round = 0;
        Motors[i].total_angle = 0;
        Motors[i].target_speed = 0;
        Motors[i].target_angle = 0;
        Motors[i].Out_Current = 0;
        Motors[i].filter_speed = 0.0f;
        Motors[i].err_last = 0.0f;
        Motors[i].speed_err_sum = 0.0f;
        Motors[i].position_err_sum = 0.0f;
        Motors[i].torque_err_sum = 0.0f;
    }
}

// 3. 发送电流
//void send_current(void) {
//    FDCAN_TxHeaderTypeDef TxHeader;
//    uint8_t TxData1[8] = {0}; // 给 FDCAN1 (放电机1、2)
//    uint8_t TxData3[8] = {0}; // 给 FDCAN3 (放电机3、4)
//
//    TxHeader.Identifier = 0x200;
//    TxHeader.IdType = FDCAN_STANDARD_ID;
//    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 必须是 8 字节
//    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
//    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//    TxHeader.MessageMarker = 0;
//    TxData1[4] = (uint8_t)(((uint16_t)Motors[0].Out_Current) >> 8);
//    TxData1[5] = (uint8_t)(Motors[0].Out_Current);
//    TxData1[6] = (uint8_t)(((uint16_t)Motors[1].Out_Current) >> 8);
//    TxData1[7] = (uint8_t)(Motors[1].Out_Current);
//    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData1);
//
//    TxData3[0] = (uint8_t)(((uint16_t)Motors[2].Out_Current) >> 8);
//    TxData3[1] = (uint8_t)(Motors[2].Out_Current);
//    TxData3[2] = (uint8_t)(((uint16_t)Motors[3].Out_Current) >> 8);
//    TxData3[3] = (uint8_t)(Motors[3].Out_Current);
//    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData3);
//        TxData1[0] = (uint8_t)(((uint16_t)Motors[2].Out_Current) >> 8);
//        TxData1[1] = (uint8_t)(Motors[2].Out_Current);
//        TxData1[2] = (uint8_t)(((uint16_t)Motors[3].Out_Current) >> 8);
//        TxData1[3] = (uint8_t)(Motors[3].Out_Current);
//        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData1);
//        TxData3[4] = (uint8_t)(((uint16_t)Motors[0].Out_Current) >> 8);
//        TxData3[5] = (uint8_t)(Motors[0].Out_Current);
//        TxData3[6] = (uint8_t)(((uint16_t)Motors[1].Out_Current) >> 8);
//        TxData3[7] = (uint8_t)(Motors[1].Out_Current);
//        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData3);
//}
void send_current(void) {
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData1[8] = {0}; // 给 FDCAN1 (放电机1、2)
    uint8_t TxData3[8] = {0}; // 给 FDCAN3 (放电机3、4)

    TxHeader.Identifier = 0x200;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 必须是 8 字节
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 【修改点2】：添加 (uint16_t) 强制转换，避免负数右移高位补1造成的符号位扩展导致电调读错电流
    TxData1[0] = (uint8_t)(((uint16_t)Motors[0].Out_Current) >> 8);
    TxData1[1] = (uint8_t)(Motors[0].Out_Current);
    TxData1[2] = (uint8_t)(((uint16_t)Motors[1].Out_Current) >> 8);
    TxData1[3] = (uint8_t)(Motors[1].Out_Current);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData1);

    TxData3[4] = (uint8_t)(((uint16_t)Motors[2].Out_Current) >> 8);
    TxData3[5] = (uint8_t)(Motors[2].Out_Current);
    TxData3[6] = (uint8_t)(((uint16_t)Motors[3].Out_Current) >> 8);
    TxData3[7] = (uint8_t)(Motors[3].Out_Current);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData3);
}

// 4.速度环PID 计算
void PID_Calc_Speed(int i)
{
	if (i < 0 || i >= 4) return;
    Motor_3508_T* m = &Motors[i];

    // 计算误差
    float error = m->target_speed - m->cur_speed;

    // 积分累加
    m->speed_err_sum += error;

    // 积分限幅
    if(m->speed_err_sum > Speed_PID.Max_Sum) m->speed_err_sum = Speed_PID.Max_Sum;
    if(m->speed_err_sum < -Speed_PID.Max_Sum) m->speed_err_sum = -Speed_PID.Max_Sum;
    // 如果先赋值 m->err_last = error，那么下面 (error - m->err_last) 将永远等于 0，导致 D 项完全失效
    float d_term = Speed_PID.Kd * (error - m->err_last) / Ts;
    m->err_last = error;

    float output = (Speed_PID.Kp * error) + (Speed_PID.Ki * m->speed_err_sum * Ts) + d_term;

    // 输出限幅
    if(output > Speed_PID.Max_Out) output = Speed_PID.Max_Out;
    if(output < -Speed_PID.Max_Out) output = -Speed_PID.Max_Out;

    //调用功率限制函数
    m->Out_Current = Power_Limit(output, (float)m->filter_speed);

}

// 5.位置环计算函数
// 输入：目标角度 target_angle (单位：总编码器值，8192 = 1圈)
// 输出：写入 Motors[i].target_speed，供速度环使用
void PID_Calc_Position(int i, float target_angle)
{
	if (i < 0 || i >= 4) return;
	Motor_3508_T* m = &Motors[i];

    // 计算位置误差
	m->target_angle = (int64_t)target_angle;
    float error = m->target_angle - m->total_angle;

    // 死区
    if (fabs(error) < 25.0f) {
        error = 0.0f;               // 【关键修改】：将误差归零，这样后续算出的 target_speed 就会自然变成 0
        m->position_err_sum = 0.0f; // 清除积分，防止积分饱和引起的微小震荡
        // 绝对不要在这里手动设置 m->Out_Current = 0 或者 target_speed = 0
    }

    // 积分累加
    m->position_err_sum += error;



    // 积分限幅
    if(m->position_err_sum > Position_PID.Max_Sum) m->position_err_sum = Position_PID.Max_Sum;
    if(m->position_err_sum < -Position_PID.Max_Sum) m->position_err_sum = -Position_PID.Max_Sum;

    // 位置式 PD 控制
    float position_speed =(Position_PID.Kp * error) + (Position_PID.Ki * m->position_err_sum * Ts) - (Position_PID.Kd * m->filter_speed);

    // 限幅 (限制最大转速)
    if(position_speed > Position_PID.Max_Out) position_speed = Position_PID.Max_Out;
    if(position_speed < -Position_PID.Max_Out) position_speed = -Position_PID.Max_Out;

    //将计算出的速度 赋值给 速度环的目标
    m->target_speed = position_speed;

    // 调用速度环计算，算出电流
    PID_Calc_Speed(i);
}

// 力矩环计算函数
void PID_Calc_Torque(int i, float target_torque)
{
    if (i < 0 || i >= 4) return;
    Motor_3508_T* m = &Motors[i];

    float target_current_val = (target_torque / Kt) * Int_Current;
    float error = target_current_val - m->actual_current;

    m->torque_err_sum += error;
    if(m->torque_err_sum > Torque_PID.Max_Sum) m->torque_err_sum = Torque_PID.Max_Sum;
    if(m->torque_err_sum < -Torque_PID.Max_Sum) m->torque_err_sum = -Torque_PID.Max_Sum;

    float output = target_current_val + (Torque_PID.Kp * error) + (Torque_PID.Ki * m->torque_err_sum * Ts);

    if(output > Torque_PID.Max_Out) output = Torque_PID.Max_Out;
    if(output < -Torque_PID.Max_Out) output = -Torque_PID.Max_Out;

    m->Out_Current = Power_Limit(output, m->filter_speed);
}

// 6.功率限制
int16_t Power_Limit(float desire_current, float rpm){

	// 计算角速度
	float w = fabs(rpm) * 2 * 3.14f / 60;

	if (w < 0.5f) w = 0.5f;

	// 计算最大允许转矩
	float max_torque = POWER_LIMIT / w;

	//计算最大电流
	float max_current_A = max_torque / Kt;

	float max_current_Value = max_current_A * Int_Current;

	//限流
	if (max_current_Value > 16000.0f) max_current_Value = 16000.0f;

	//输出判断
    if (desire_current > max_current_Value) {
        return (int16_t)max_current_Value;
    } else if (desire_current < -max_current_Value) {
        return (int16_t)(-max_current_Value);
    } else {
        return (int16_t)desire_current;
    }
}

// 7.拆解接收数据
//void D3508_Decode(uint8_t* RxData, uint16_t ID, uint8_t can_bus){
void D3508_Decode(uint8_t* RxData, uint16_t ID) {
    // 检查 ID 范围
    if (ID < 0x201 || ID > 0x204) return;
        uint8_t idx = ID - 0x201;
//    if (can_bus == 3) {
//            idx += 2;
//        }
    Motor_3508_T* m = &Motors[idx];

    // 保存上一次角度
    m->last_angle = m->cur_angle;

    // 解析新数据
    m->cur_angle = (RxData[0] << 8) | RxData[1];
    m->cur_speed = (int16_t)((RxData[2] << 8) | RxData[3]);
    m->actual_current = (int16_t)((RxData[4] << 8) | RxData[5]);
    m->temperature = RxData[6];

    // 首次上电初始化保护，防止上电疯转
    if (m->init_flag == 0) {
        m->last_angle = m->cur_angle;       // 同步历史角度，消除初始 delta
        m->total_round = 0;                 // 圈数从 0 开始
        m->total_angle = m->cur_angle;      // 同步绝对总角度
        m->filter_speed = m->cur_speed;     // 速度滤波器赋初值
        m->init_flag = 1;                   // 标记已完成初始化
        return;                             // 第一帧数据不进行跨圈计算，直接返回
    }

    //EMA滤波速度
    m->filter_speed = (D3508_EMA_ALPHA * m->cur_speed) + ((1.0f - D3508_EMA_ALPHA) * m->filter_speed);

    //多圈解算
    int32_t delta = (int32_t)m->cur_angle - (int32_t)m->last_angle;
    if (delta > 4096) {
        m->total_round--;
    } else if (delta < -4096) {
        m->total_round++;
    }
    m->total_angle = m->total_round * 8192 + m->cur_angle;
}
// 7.拆解接收数据
// 新增 can_bus 参数：传入 1 代表 CAN1，传入 3 代表 CAN3
