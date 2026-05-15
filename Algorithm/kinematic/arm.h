#ifndef ARM_H_
#define ARM_H_
#include <stdint.h>

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


// 【新增】：用于在 STM32CubeIDE Live Expressions 实时观测的全局调试变量

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
extern ArmDebug_t arm_debug;


// ========================= 运输构型保持模式 =========================
// 0：普通模式，按原版逆解自由选解
// 1：运输模式，优先保持抓取成功时的“肘部在上”构型，主要依靠底座旋转运输
extern uint8_t arm_transport_mode;

// 抓取成功瞬间记录的运输参考姿态
extern float transport_alpha_ref;
extern float transport_beta_ref;
extern float transport_omega_ref;

// 记录抓取成功时，逆解实际采用的 L_val 符号
// 用来约束运输阶段尽量保持同侧臂展方向，避免切到会翻箱子的另一类解
extern float transport_L_ref;

// 当前一次 IK 成功解算时选择的 L_val
// 在吸附成功瞬间，状态机会把它保存到 transport_L_ref
extern float last_ik_L_val;

// 运输模式下允许的姿态偏移窗口（单位：rad）
// 如果某个候选解偏离抓取姿态过大，则认为会让箱体姿态恶化，直接丢弃
extern float transport_alpha_window;
extern float transport_beta_window;
extern float transport_omega_window;
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
