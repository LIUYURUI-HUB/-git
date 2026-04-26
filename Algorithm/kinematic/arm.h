#ifndef ARM_H_
#define ARM_H_

/**
 * @brief 机械臂物理参数
 */
typedef struct {
    float h;  // 偏移量 (基座到肩关节 Pitch 轴的横向偏移/肩宽)
    float hu; // 大臂 (upper link) 长度
    float hl; // 小臂 (lower link) 长度
    float he;// 末端 (end link) 吸盘距离
} Arm_Lengths_t;


/**
 * @brief 关节电机角度 (单位: rad)
 * 几何法定义：
 * gamma > 0: 手臂向外侧 (左右平举) 抬起
 * alpha > 0: 大臂向前抬起 (与自然下垂垂线的夹角)
 * beta  > 0: 小臂向前弯曲 (与大臂延长线的相对夹角)
 */
typedef struct {
    float gamma; // 基座横滚角 (Roll)
    float alpha; // 大臂俯仰角 (Pitch)
    float beta;  // 小臂俯仰角 (Pitch)
    float omega;  // 末端角
} Joint_Angles_t;


typedef struct {
    float x;
    float y;
    float z;

    float x_pan;
    float y_pan;
    float z_pan;
} End_Pos_t;

//// 定义机械臂目标位置结构体
//typedef struct {
//    float x;
//    float y;
//    float z;
//} ArmTarget_t;
// 【新增】：用于在 STM32CubeIDE Live Expressions 实时观测的全局调试变量
// ==============================================================================
typedef struct {
	//重力补偿观测参数
    float q1_rad;       // 换算后送入模型的大臂角度 (rad)
    float q2_rad;       // 换算后送入模型的小臂角度 (rad)
    float tau_calc_m2;  // 理论模型算出的大臂原始重力矩 (Nm)
    float tau_calc_m3;  // 理论模型算出的小臂原始重力矩 (Nm)
    float tau_calc_m4;
    float tau_gravity_m2;    // 最终发给大臂电机的带方向前馈力矩 (Nm)
    float tau_gravity_m3;    // 最终发给小臂电机的带方向前馈力矩 (Nm)
    float tau_gravity_m4;
    //逆运动学观测参数
    float cmd_m2_pos;   // M2 目标位置指令
    float cmd_m3_pos;   // M3 目标位置指令
    float actual_m2_pos;// 【新增】M2 实际物理位置 (用于上位机计算误差和死区)
    float actual_m3_pos;// 【新增】M3 实际物理位置
    float err_i_m2;     // M2 积分补偿力矩
    float err_i_m3;     // M3 积分补偿力矩
    float tau_ff_m2;    // M2 前馈力矩 (重力+摩擦力)
    float tau_ff_m3;    // M3 前馈力矩 (重力+摩擦力)

} ArmDebug_t;

/**
 * @brief 机械臂正运动学解算
 * @param lengths 机械臂连杆长度参数
 * @param angles  当前关节角度
 * @param pos     计算得出的末端坐标输出指针
 */
void Arm_FK(const Arm_Lengths_t *lengths, const Joint_Angles_t *angles, End_Pos_t *pos);
int  Arm_IK(const Arm_Lengths_t *lengths, const End_Pos_t *target_pos, Joint_Angles_t *angles);
void run_arm_kinematics(void);
void run_arm_to_pos(float tx, float ty, float tz);
// 将电机的实际角度传给理论中力矩计算函数,得到前馈力矩
void Arm_Calc_Gravity_Torque(float input_m2_pos, float input_m3_pos, float *tau_ff_m2, float *tau_ff_m3, float *tau_ff_m4);

void run_arm_drag_teach_mode(void);


#endif /* ARM_H_ */
