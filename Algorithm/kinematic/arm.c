#include "arm.h"
#include "arm_g.h"
#include "dm_driver.h"
#include "main.h"
#include <math.h>

// 引用在 main.c 中定义的全局变量
extern DM_Motor_t Arm_Motors[4];
extern Arm_Lengths_t arm_params;
extern Joint_Angles_t current_angles;
extern End_Pos_t end_effector_pos;
extern ArmGravityParams_t arm_g_params;
ArmDebug_t arm_debug = {0}; // 全局实例，在 STM32CubeIDE 搜索此变量添加观测

// ========================= 载荷/运输模式全局状态 =========================
// 末端附加载荷质量（kg）
float arm_payload_mass = 0.0f;

// 0：空载参数；1：带载参数
uint8_t arm_loaded_mode = 0;

// 0：普通自由求解；1：运输构型保持模式
uint8_t arm_transport_mode = 0;

// 抓取成功时记录的运输参考姿态
float transport_alpha_ref = 0.0f;
float transport_beta_ref  = 0.0f;
float transport_omega_ref = 0.0f;

// 抓取成功时记录的臂展方向参考
float transport_L_ref = 0.0f;

// 当前一次 IK 成功解出来的 L_val，供状态机在抓取成功瞬间记忆
float last_ik_L_val = 0.0f;

// 运输姿态窗口（可现场调参）
// 数值越小，运输时姿态保持越严格；数值越大，避障灵活性越高
float transport_alpha_window = 0.55f;
float transport_beta_window  = 0.55f;
float transport_omega_window = 0.70f;

// 2. 完美匹配 Python 上位机 "<I 8f" 的结构体 (总共 40 字节)
#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];      // 0xAA 0x55
    uint32_t time_us;       // 时间戳
    float m2_pos;           // M2实际位置
    float m3_pos;           // M3实际位置
    float m2_vel;           // M2实际速度
    float m3_vel;           // M3实际速度
    float m2_tau_actual;    // M2实际输出力矩
    float m3_tau_actual;    // M3实际输出力矩
    float tau_ff_m2;        // M2前馈力矩 (来自arm_debug)
    float tau_ff_m3;        // M3前馈力矩 (来自arm_debug)
    uint8_t tail[2];        // 0x55 0xAA
} ArmLogFrame;
#pragma pack(pop)

static float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}
// 计算两个角度之间的最小差值，结果范围固定在 [-pi, pi]
static float angle_diff(float a, float b) {
    return normalize_angle(a - b);
}
// 1. 底座旋转 (Gamma - 对应电机 ID: 0x01, 数组下标 0)
#define M1_ZERO_OFFSET   -0.000190734863f     // 填入: 底座朝正前方时的 POS 值
#define M1_DIR           1.0f

// 2. 大臂俯仰 (Alpha - 对应电机 ID: 0x02, 数组下标 1)
#define M2_ZERO_OFFSET   -0.000190734863f     // 填入: 大臂完全水平伸直时的 POS 值
#define M2_DIR           -1.0f                // 大臂往上抬，若 POS 变大填 1.0f，若变小填 -1.0f
#define M2_G_DIR         -1.0f

// 3. 小臂俯仰 (Beta - 对应电机 ID: 0x03, 数组下标 2)
#define M3_ZERO_OFFSET   -0.000190734863f     // 小臂完全水平伸直时的 POS 值
#define M3_DIR           -1.0f                // 小臂往上折，若 POS 变大填 1.0f，若变小填 -1.0f
#define M3_G_DIR          1.0f

// 4. 吸盘水平
#define M4_ZERO_OFFSET   -0.000190734863f
#define M4_DIR           1.0f

#define M2_TORQUE_SIGN   -1.0f
#define M3_TORQUE_SIGN    1.0f
#define M4_TORQUE_SIGN    1.0f

#define Q    0.2217f

void Arm_Calc_Gravity_Torque(float input_m2_pos, float input_m3_pos, float *out_tau_gravity_m2, float *out_tau_gravity_m3,float *out_tau_gravity_m4) {
    ArmJoint_POS_t calc_POS;
    ArmGravityTorque_t gravity_torque = {0};

    // 在这里进行物理映射到数学映射的转换
    calc_POS.q1 = (input_m2_pos - M2_ZERO_OFFSET) * M2_G_DIR;
    calc_POS.q2 = (input_m3_pos - M3_ZERO_OFFSET) * M3_G_DIR;

    calc_POS.payload = arm_payload_mass;

    arm_debug.q1_rad = calc_POS.q1;
    arm_debug.q2_rad = calc_POS.q2;

    // 此时传入的 calc_POS 已经是小臂展开为 0 度的标准数学值了
    Arm_Gravity_Calc(&arm_g_params, &calc_POS, &gravity_torque);

    arm_debug.tau_calc_m2 = gravity_torque.tau1;
    arm_debug.tau_calc_m3 = gravity_torque.tau2;
    arm_debug.tau_calc_m4 = gravity_torque.tau3;

    // 输出计算出的“纯重力前馈力矩”
    *out_tau_gravity_m2 = gravity_torque.tau1 * M2_TORQUE_SIGN;
    *out_tau_gravity_m3 = gravity_torque.tau2 * M3_TORQUE_SIGN;
    *out_tau_gravity_m4 = gravity_torque.tau3 * M4_TORQUE_SIGN;

    arm_debug.tau_gravity_m2 = *out_tau_gravity_m2;
    arm_debug.tau_gravity_m3 = *out_tau_gravity_m3;
    arm_debug.tau_gravity_m4 = *out_tau_gravity_m4;
}

/**
 * @brief 机械臂运动学正解 (核心数学模型)
 * @param lengths 机械臂连杆长度参数
 * @param angles  当前关节角度 (rad)
 * @param pos     计算得出的末端坐标输出指针
 */
void Arm_FK(const Arm_Lengths_t *lengths, const Joint_Angles_t *angles, End_Pos_t *pos) {
    // 1. theta: 小臂与水平面的夹角 (rad)
    float virtual_alpha = angles->alpha;
    float virtual_beta  = angles->beta - Q + M_PI;
    float virtual_omega = angles->omega;

    float theta = (M_PI / 2.0f) - virtual_beta - virtual_alpha;
    float q = virtual_beta + virtual_alpha + virtual_omega - M_PI_2;

    // 2. 计算大臂和小臂在二维垂直平面内的投影长度 (hu' 和 hl')
    float hu_prime = lengths->hu * cosf(virtual_alpha);
    float hl_prime = lengths->hl * sinf(theta);
    float he_prime = lengths->he * sinf(q);
    float arm_prime = hu_prime + hl_prime - he_prime;

    //求解吸盘坐标
    // 3. 求解 X 坐标 (高度)
    // 公式: x = hu*sin(α) + hl*cos(θ)
    pos->x_pan = lengths->hu * sinf(virtual_alpha) + lengths->hl * cosf(theta) + lengths->he * cosf(q);

    // 4. 求解 Y 坐标 (侧向)
    // 公式: y = h*cos(γ) + (hu' + hl')*sin(γ)
    pos->y_pan = lengths->h * cosf(angles->gamma) + arm_prime * sinf(angles->gamma);

    // 5. 求解 Z 坐标 (前向)
    // 公式: z = h*sin(γ) + (hu' + hl')*cos(γ)
    pos->z_pan = lengths->h * sinf(angles->gamma) + arm_prime * cosf(angles->gamma);

    //求解小臂末端坐标
    // 3. 求解 X 坐标 (高度)
    // 公式: x = hu*sin(α) + hl*cos(θ)
    pos->x = lengths->hu * sinf(virtual_alpha) + lengths->hl * cosf(theta);

    // 4. 求解 Y 坐标 (侧向)
    // 公式: y = h*cos(γ) + (hu' + hl')*sin(γ)
    pos->y = lengths->h * cosf(angles->gamma) + (hu_prime + hl_prime) * sinf(angles->gamma);

    // 5. 求解 Z 坐标 (前向)
    // 公式: z = h*sin(γ) + (hu' + hl')*cos(γ)
    pos->z = lengths->h * sinf(angles->gamma) + (hu_prime + hl_prime) * cosf(angles->gamma);
}

/**
 * @brief 模式一：零力拖拽 / 纯悬浮模式 (被动模式)
 * @note  因为没有目标路径，所以【必须使用传感器的真实POS】来计算重力
 */
void run_arm_drag_teach_mode(void) {
    float tau_m2 = 0.0f, tau_m3 = 0.0f,tau_m4 = 0.0f;

    // 【反馈补偿】传入真实的 Arm_Motors[x].POS
    Arm_Calc_Gravity_Torque(Arm_Motors[1].POS, Arm_Motors[2].POS, &tau_m2, &tau_m3,&tau_m4);

    // 加入微小阻尼
    // 纯 kd=0 会太滑
    float drag_kd_m1 = 0.1f;
    float drag_kd_m2 = 0.1f;   // 大臂
    float drag_kd_m3 = 0.05f;  // 小臂
    float drag_kd_m4 = 0.05f;  // 小臂

    // Kp 必须严格设为 0，失去位置刚度
    // 目标速度为 0.0f，配合 Kd 产生抵抗快速运动的阻力
    DM_Send_Ctrl(0x01, Arm_Motors[0].POS, 0.0f, 0.0f, drag_kd_m1, 0.0f);
    DM_Send_Ctrl(0x02, Arm_Motors[1].POS, 0.0f, 0.0f, drag_kd_m2, tau_m2);
    DM_Send_Ctrl(0x03, Arm_Motors[2].POS, 0.0f, 0.0f, drag_kd_m3, tau_m3);
    DM_Send_Ctrl(0x04, Arm_Motors[3].POS, 0.0f, 0.0f, drag_kd_m4, tau_m4);
}

/**
 * @brief 执行机械臂正运动学解算循环 (物理 1:1 映射)
 * @note  请在 main.c 的主循环或定时器中调用此函数
 */
void run_arm_kinematics(void) {

    // 1. 读取达妙电机的原始 POS (注意：ID 1~3 对应数组索引 0~2)
    float m1_pos = Arm_Motors[0].POS; // 底座电机
    float m2_pos = Arm_Motors[1].POS; // 大臂电机
    float m3_pos = Arm_Motors[2].POS; // 小臂电机
    float m4_pos = Arm_Motors[3].POS; // 小臂电机

    // 2. 转换为正解模型所需的标准弧度：(当前位置 - 零点位置) * 运动方向
    current_angles.gamma = (m1_pos - M1_ZERO_OFFSET) * M1_DIR;
    current_angles.alpha = (m2_pos - M2_ZERO_OFFSET) * M2_DIR;
    current_angles.beta  = (m3_pos - M3_ZERO_OFFSET) * M3_DIR;
    current_angles.omega = (m4_pos - M4_ZERO_OFFSET) * M4_DIR;

    // 3. 调用正解函数 (核心数学计算)
    End_Pos_t raw_res;
    Arm_FK(&arm_params, &current_angles, &raw_res);

    // 4. 坐标系对齐
    end_effector_pos.x = raw_res.x; // Fusion X (向上高度)
    end_effector_pos.y = raw_res.y; // Fusion Y (向左侧偏)
    end_effector_pos.z = raw_res.z; // Fusion Z (向前伸展)

    end_effector_pos.x_pan = raw_res.x_pan; // Fusion X (向上高度)
    end_effector_pos.y_pan = raw_res.y_pan; // Fusion Y (向左侧偏)
    end_effector_pos.z_pan = raw_res.z_pan; // Fusion Z (向前伸展)
}

int Arm_IK(const Arm_Lengths_t *lengths, const End_Pos_t *target_pos, Joint_Angles_t *angles) {
    float X = target_pos->x;
    float Y = target_pos->y;
    float Z = target_pos->z;

    if (Z >= 0.0f && X < 0.0f) {
        return -1;
    }

    typedef struct {
        float gamma;
        float alpha;
        float beta;
        float elbow_x;
        float L_val;
    } IKSolution;

    IKSolution candidates[4];
    int candidate_count = 0;

    float g1 = (Z == 0.0f && Y == 0.0f) ? 0.0f : atan2f(Y, Z);
    float g2 = atan2f(-Y, -Z);

    float L_total = sqrtf(Y * Y + Z * Z);
    float bases_g[2] = {g1, g2};
    float bases_L[2] = {L_total - lengths->h, -L_total - lengths->h};

    // 保留原版完整双分支求解。
    // 这样 arm_home、前方取箱点、以及原本可达的正常运动路径都不会被误杀。
    for (int i = 0; i < 2; i++) {
        float gamma = bases_g[i];
        float L = bases_L[i];

        float dist_sq = X * X + L * L;
        float dist = sqrtf(dist_sq);

        if (dist > (lengths->hu + lengths->hl) || dist < fabsf(lengths->hu - lengths->hl)) {
            continue;
        }

        float cos_B = (lengths->hu * lengths->hu + lengths->hl * lengths->hl - dist_sq) / (2.0f * lengths->hu * lengths->hl);
        if (cos_B > 1.0f) cos_B = 1.0f;
        if (cos_B < -1.0f) cos_B = -1.0f;
        float B = acosf(cos_B);

        float phi1 = atan2f(X, L);
        float cos_phi2 = (lengths->hu * lengths->hu + dist_sq - lengths->hl * lengths->hl) / (2.0f * lengths->hu * dist);
        if (cos_phi2 > 1.0f) cos_phi2 = 1.0f;
        if (cos_phi2 < -1.0f) cos_phi2 = -1.0f;
        float phi2 = acosf(cos_phi2);

        float alpha_A = normalize_angle(phi1 + phi2);
        float beta_A  = normalize_angle(B + Q - (2.0f * M_PI));

        // 构型 A
        candidates[candidate_count].gamma = gamma;
        candidates[candidate_count].alpha = alpha_A;
        candidates[candidate_count].beta  = beta_A;
        candidates[candidate_count].elbow_x = lengths->hu * sinf(alpha_A);
        candidates[candidate_count].L_val = L;
        candidate_count++;

        // 构型 B
        float alpha_B = normalize_angle(phi1 - phi2);
        float beta_B  = normalize_angle(-B + Q);

        candidates[candidate_count].gamma = gamma;
        candidates[candidate_count].alpha = alpha_B;
        candidates[candidate_count].beta  = beta_B;
        candidates[candidate_count].elbow_x = lengths->hu * sinf(alpha_B);
        candidates[candidate_count].L_val = L;
        candidate_count++;
    }

    int best_idx = -1;
    float max_elbow_x = -9999.0f;

    // 运输模式下的“最接近参考姿态”代价
    float best_transport_cost = 9999.0f;

    // 如果严格窗口内没有解，则允许在“同侧臂展方向”前提下选最接近的那个解
    // 这样比直接失败更灵活，能给机身附近小障碍留出少量绕行余量。
    int relaxed_transport_idx = -1;
    float relaxed_transport_cost = 9999.0f;

    for (int i = 0; i < candidate_count; i++) {
        float m1_pos = (candidates[i].gamma / M1_DIR) + M1_ZERO_OFFSET;
        float m2_pos = (candidates[i].alpha / M2_DIR) + M2_ZERO_OFFSET;
        float m3_pos = (candidates[i].beta / M3_DIR) + M3_ZERO_OFFSET;

        // 基础关节限位筛选
        // 这里保留你当前工程里已经接入的 M1 实际死区范围
        if (m1_pos >= -3.10997963f && m1_pos <= 2.47940063f &&
            m2_pos >= -3.19f && m2_pos <= 0.05f &&
            m3_pos >= -0.05f && m3_pos <= 3.19f)
        {
            if (arm_transport_mode) {
                float virtual_alpha = candidates[i].alpha;
                float virtual_beta  = candidates[i].beta - Q + M_PI;
                float omega;

                if (candidates[i].L_val >= 0.0f) {
                    omega = -(virtual_alpha + virtual_beta);
                } else {
                    omega = M_PI - (virtual_alpha + virtual_beta);
                }

                while (omega > M_PI) omega -= 2.0f * M_PI;
                while (omega < -M_PI) omega += 2.0f * M_PI;

                // ============================================================
                // 运输模式约束：
                // 1. 优先保持抓取成功时同一侧的臂展方向，避免切到另一类会翻箱子的构型
                // 2. 优先保持抓取成功时的肘上姿态，尽量让 M2/M3/M4 只做小幅修正
                // 3. 真正的大范围转运主要交给底座 M1 去完成
                // ============================================================
                if ((transport_L_ref > 0.0f && candidates[i].L_val < 0.0f) ||
                    (transport_L_ref < 0.0f && candidates[i].L_val > 0.0f)) {
                    continue;
                }

                float alpha_err = fabsf(angle_diff(candidates[i].alpha, transport_alpha_ref));
                float beta_err  = fabsf(angle_diff(candidates[i].beta,  transport_beta_ref));
                float omega_err = fabsf(angle_diff(omega,               transport_omega_ref));
                float transport_cost = alpha_err + beta_err + omega_err;

                // 先记录一个“同侧方向下最接近参考姿态”的候选，作为严格窗口无解时的兜底
                if (transport_cost < relaxed_transport_cost) {
                    relaxed_transport_cost = transport_cost;
                    relaxed_transport_idx = i;
                }

                // 严格窗口判断：只有姿态偏移在允许范围内，才算运输模式下的优先合法解
                if (alpha_err <= transport_alpha_window &&
                    beta_err  <= transport_beta_window  &&
                    omega_err <= transport_omega_window) {

                    if (best_idx == -1 || transport_cost < best_transport_cost) {
                        best_transport_cost = transport_cost;
                        best_idx = i;
                    }
                }
            } else {
                // 普通模式下保持原版逻辑：
                // 优先选择肘部更高的解，保证前方取箱和 arm_home 等原始路径尽量不变
                if (candidates[i].elbow_x > max_elbow_x) {
                    max_elbow_x = candidates[i].elbow_x;
                    best_idx = i;
                }
            }
        }
    }

    // 如果运输模式下严格窗口无解，则退而求其次：
    // 保持同侧臂展方向不变，选最接近参考姿态的那个解。
    if (arm_transport_mode && best_idx == -1 && relaxed_transport_idx != -1) {
        best_idx = relaxed_transport_idx;
    }

    if (best_idx != -1) {
        angles->gamma = candidates[best_idx].gamma;
        angles->alpha = candidates[best_idx].alpha;
        angles->beta  = candidates[best_idx].beta;

        float virtual_alpha = angles->alpha;
        float virtual_beta  = angles->beta - Q + M_PI;
        float chosen_L      = candidates[best_idx].L_val;

        if (chosen_L >= 0.0f) {
            angles->omega = -(virtual_alpha + virtual_beta);
        } else {
            angles->omega = M_PI - (virtual_alpha + virtual_beta);
        }

        while (angles->omega > M_PI) angles->omega -= 2.0f * M_PI;
        while (angles->omega < -M_PI) angles->omega += 2.0f * M_PI;

        // 记录当前 IK 最终选中的臂展方向，供状态机在抓取成功瞬间保存为运输参考
        last_ik_L_val = chosen_L;

        return 0;
    }

    return -1;
}


/**
 * @brief 执行平滑控制，针对视觉联调优化的笛卡尔空间跟随
 */
void run_arm_to_pos(float target_tx, float target_ty, float target_tz) {
    // 笛卡尔空间当前指令坐标记录 (用于平滑插值)
    static float current_cmd_x = 0.0f;
    static float current_cmd_y = 0.0f;
    static float current_cmd_z = 0.0f;

    // 关节控制指令记录
    static float current_m1_cmd = 0.0f;
    static float current_m2_cmd = 0.0f;
    static float current_m3_cmd = 0.0f;
    static float current_m4_cmd = 0.0f;

    static float last_m1_cmd = 0.0f;
    static float last_m2_cmd = 0.0f;
    static float last_m3_cmd = 0.0f;
    static float last_m4_cmd = 0.0f;

    static uint8_t is_init = 0;
    float dt = 0.005f; // 控制周期 (500Hz)

    if (!is_init) {
        // 第一帧：初始化关节指令为当前电机真实位置
        current_m1_cmd = last_m1_cmd = Arm_Motors[0].POS;
        current_m2_cmd = last_m2_cmd = Arm_Motors[1].POS;
        current_m3_cmd = last_m3_cmd = Arm_Motors[2].POS;
        current_m4_cmd = last_m4_cmd = Arm_Motors[3].POS;

        // 获取真实的末端XYZ作为插值起点
        // 若FK正解已执行则取真实数据，否则用传入目标暂代
        if (end_effector_pos.x != 0 || end_effector_pos.y != 0 || end_effector_pos.z != 0) {
            current_cmd_x = end_effector_pos.x;
            current_cmd_y = end_effector_pos.y;
            current_cmd_z = end_effector_pos.z;
        } else {
            current_cmd_x = target_tx;
            current_cmd_y = target_ty;
            current_cmd_z = target_tz;
        }

        is_init = 1;
        return;
    }

    // 笛卡尔空间匀速直线插补 + 末端柔顺滤波
    float max_linear_speed = 0.26f; // 设定的最大线速度：0.26米/秒 (26cm/s)
    if (arm_loaded_mode) {
        max_linear_speed = 0.15f;
    }
    float max_step = max_linear_speed * dt; // 每个控制周期(5ms)允许移动的最大物理距离

    float diff_x = target_tx - current_cmd_x;
    float diff_y = target_ty - current_cmd_y;
    float diff_z = target_tz - current_cmd_z;
    float dist = sqrtf(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    if (dist > max_step) {
        // 1. 长距离移动：执行【三维空间匀速直线插补】
        current_cmd_x += (diff_x / dist) * max_step;
        current_cmd_y += (diff_y / dist) * max_step;
        current_cmd_z += (diff_z / dist) * max_step;
    } else {
        // 2. 距离极近时：切回【一阶低通滤波】消除震荡，使停稳
        // 【调参一】：提高收敛系数，避免收尾时缓慢拖泥带水
        float alpha = 0.10f; // 原 0.05f
        if (arm_loaded_mode) {
            alpha = 0.08f;   // 原 0.08f
        }
        current_cmd_x = (1.0f - alpha) * current_cmd_x + alpha * target_tx;
        current_cmd_y = (1.0f - alpha) * current_cmd_y + alpha * target_ty;
        current_cmd_z = (1.0f - alpha) * current_cmd_z + alpha * target_tz;
    }

    End_Pos_t smooth_target_pos = {current_cmd_x, current_cmd_y, current_cmd_z};
    Joint_Angles_t target_angles;
    int ik_status = Arm_IK(&arm_params, &smooth_target_pos, &target_angles);

    float dir_m1 = 0.0f, dir_m2 = 0.0f, dir_m3 = 0.0f, dir_m4 = 0.0f;
    float max_step_safe = 0.00500f; // 用于记录符号趋势

    if (ik_status == 0) {
        float target_m1_pos = (target_angles.gamma / M1_DIR) + M1_ZERO_OFFSET;
        float target_m2_pos = (target_angles.alpha / M2_DIR) + M2_ZERO_OFFSET;
        float target_m3_pos = (target_angles.beta  / M3_DIR) + M3_ZERO_OFFSET;
        float target_m4_pos = (target_angles.omega / M4_DIR) + M4_ZERO_OFFSET;

        // 吸盘最短路径防缠绕保护
        while (target_m4_pos - current_m4_cmd > M_PI) target_m4_pos -= 2.0f * M_PI;
        while (target_m4_pos - current_m4_cmd < -M_PI) target_m4_pos += 2.0f * M_PI;
        if (target_m4_pos > 3.14159f) target_m4_pos = 3.14159f;
        if (target_m4_pos < -3.14159f) target_m4_pos = -3.14159f;

        // 提取 M1-M3 运动趋势，保留原有的误差积分抗饱和逻辑
        float step_m1 = target_m1_pos - current_m1_cmd;
        if (step_m1 > max_step_safe) dir_m1 = 1.0f; else if (step_m1 < -max_step_safe) dir_m1 = -1.0f; else dir_m1 = 0.0f;

        float step_m2 = target_m2_pos - current_m2_cmd;
        if (step_m2 > max_step_safe) dir_m2 = 1.0f; else if (step_m2 < -max_step_safe) dir_m2 = -1.0f; else dir_m2 = 0.0f;

        float step_m3 = target_m3_pos - current_m3_cmd;
        if (step_m3 > max_step_safe) dir_m3 = 1.0f; else if (step_m3 < -max_step_safe) dir_m3 = -1.0f; else dir_m3 = 0.0f;

        // 【针对 M4 吸盘的独立平滑优化】：
        // 吸盘姿态直接关系到箱体是否“低头”或被翻到朝天，因此仍然保留独立限速。
        float max_m4_speed = 1.5f;
        if (arm_loaded_mode) {
            max_m4_speed = 0.75f;
        }
        float max_m4_step = max_m4_speed * dt;
        float step_m4 = target_m4_pos - current_m4_cmd;

        if (step_m4 > max_m4_step) {
            current_m4_cmd += max_m4_step;
            dir_m4 = 1.0f;
        } else if (step_m4 < -max_m4_step) {
            current_m4_cmd -= max_m4_step;
            dir_m4 = -1.0f;
        } else {
            current_m4_cmd = target_m4_pos;
            dir_m4 = 0.0f;
        }

        // M1, M2, M3 因为已有 XYZ 笛卡尔平滑，直接全盘跟随
        // 运输时真正的大范围姿态约束由 IK 候选解筛选完成，这里不再额外硬锁底座策略
        current_m1_cmd = target_m1_pos;
        current_m2_cmd = target_m2_pos;
        current_m3_cmd = target_m3_pos;

    } else {
        // 遇到超限或空气墙时，靠 static 特性让关节指令原地锁死
        current_cmd_x = end_effector_pos.x;
        current_cmd_y = end_effector_pos.y;
        current_cmd_z = end_effector_pos.z;
    }

    // 推导期望速度
    float v_des_m1 = (current_m1_cmd - last_m1_cmd) / dt;
    float v_des_m2 = (current_m2_cmd - last_m2_cmd) / dt;
    float v_des_m3 = (current_m3_cmd - last_m3_cmd) / dt;
    float v_des_m4 = (current_m4_cmd - last_m4_cmd) / dt;

    last_m1_cmd = current_m1_cmd;
    last_m2_cmd = current_m2_cmd;
    last_m3_cmd = current_m3_cmd;
    last_m4_cmd = current_m4_cmd;

    // 动力学补偿
    float tau_gravity_m2 = 0.0f, tau_gravity_m3 = 0.0f, tau_gravity_m4 = 0.0f;
    Arm_Calc_Gravity_Torque(current_m2_cmd, current_m3_cmd, &tau_gravity_m2, &tau_gravity_m3, &tau_gravity_m4);

    // 刚度参数
    // 【调参二】：温和提升 Kp 刚性让跟随更紧凑，小幅提升 Kd 避免震荡
    float Kp_m1 = 45.0f, Kd_m1 = 1.5f; // Kp 原 40.0
    float Kp_m2 = 50.0f, Kd_m2 = 1.2f; // Kp 原 40.0, Kd 原 1.0
    float Kp_m3 = 50.0f, Kd_m3 = 1.2f; // Kp 原 40.0, Kd 原 1.0
    float Kp_m4 = 40.0f, Kd_m4 = 1.0f; // M4 维持不变

    // 摩擦力补偿
    float coulomb_fric_m1 = 0.20f;
    float coulomb_fric_m2 = 0.30f;
    float coulomb_fric_m3 = 0.20f;
    float coulomb_fric_m4 = 0.10f;

    if (arm_loaded_mode) {
        Kp_m1 = 55.0f; Kd_m1 = 2.0f; // Kp 原 48.0
        Kp_m2 = 65.0f; Kd_m2 = 2.2f; // Kp 原 58.0
        Kp_m3 = 60.0f; Kd_m3 = 1.8f; // Kp 原 55.0
        Kp_m4 = 48.0f; Kd_m4 = 1.5f;

        coulomb_fric_m2 = 0.34f;
        coulomb_fric_m3 = 0.24f;
        coulomb_fric_m4 = 0.14f;
    }

    float max_i_m1 = 1.000f;
    float max_i_m2 = 1.600f, max_i_m3 = 2.600f;
    float max_i_m4 = 0.500f;

    // 【调参三】：压缩死区逼迫积分器在小误差域内继续干活；提高 Ki 增强消除残差的推力
    float dead_zone = 0.003f; // 原 0.005f
    float Ki = 65.0f;         // 原 50.0f

    if (arm_loaded_mode) {
        Ki = 75.0f;           // 原 60.0f
    }

    // 积分器消除稳态误差
    static float err_i_m1 = 0.0f, err_i_m2 = 0.0f, err_i_m3 = 0.0f, err_i_m4 = 0.0f;
    float actual_err_m1 = current_m1_cmd - Arm_Motors[0].POS;
    float actual_err_m2 = current_m2_cmd - Arm_Motors[1].POS;
    float actual_err_m3 = current_m3_cmd - Arm_Motors[2].POS;
    float actual_err_m4 = current_m4_cmd - Arm_Motors[3].POS;

    if (dir_m1 == 0.0f) {
        if (fabsf(actual_err_m1) > dead_zone) {
            err_i_m1 += actual_err_m1 * Ki * dt;
            if (err_i_m1 > max_i_m1) err_i_m1 = max_i_m1; else if (err_i_m1 < -max_i_m1) err_i_m1 = -max_i_m1;
        }
    } else { err_i_m1 *= 0.95f; }

    if (dir_m2 == 0.0f) {
        if (fabsf(actual_err_m2) > dead_zone) {
            err_i_m2 += actual_err_m2 * Ki * dt;
            if (err_i_m2 > max_i_m2) err_i_m2 = max_i_m2; else if (err_i_m2 < -max_i_m2) err_i_m2 = -max_i_m2;
        }
    } else { err_i_m2 *= 0.95f; }

    if (dir_m3 == 0.0f) {
        if (fabsf(actual_err_m3) > dead_zone) {
            err_i_m3 += actual_err_m3 * Ki * dt;
            if (err_i_m3 > max_i_m3) err_i_m3 = max_i_m3; else if (err_i_m3 < -max_i_m3) err_i_m3 = -max_i_m3;
        }
    } else { err_i_m3 *= 0.95f; }

    if (dir_m4 == 0.0f) {
        if (fabsf(actual_err_m4) > dead_zone) {
            err_i_m4 += actual_err_m4 * Ki * dt;
            if (err_i_m4 > max_i_m4) err_i_m4 = max_i_m4; else if (err_i_m4 < -max_i_m4) err_i_m4 = -max_i_m4;
        }
    } else { err_i_m4 *= 0.95f; }

    // 连续化摩擦力符号
    float smooth_dir_m1 = v_des_m1 / (fabsf(v_des_m1) + 0.1f);
    float smooth_dir_m2 = v_des_m2 / (fabsf(v_des_m2) + 0.1f);
    float smooth_dir_m3 = v_des_m3 / (fabsf(v_des_m3) + 0.1f);
    float smooth_dir_m4 = v_des_m4 / (fabsf(v_des_m4) + 0.1f);

    // 静止小误差保持摩擦补偿
    // 当期望速度接近 0 时，原来的 smooth_dir 摩擦项会接近 0；
    // 但实际关节如果还差一点，会被静摩擦卡住，所以这里按位置误差方向补一点力。
    float hold_fric_m2 = 0.0f;
    float hold_fric_m3 = 0.0f;

    if (dir_m2 == 0.0f && fabsf(actual_err_m2) > dead_zone) {
        hold_fric_m2 = (actual_err_m2 > 0.0f) ? 0.18f : -0.18f;
    }

    if (dir_m3 == 0.0f && fabsf(actual_err_m3) > dead_zone) {
        hold_fric_m3 = (actual_err_m3 > 0.0f) ? 0.18f : -0.18f;
    }

    // 计算最终前馈扭矩
    // M4 这里保留重力前馈，确保挂载物资箱后吸盘不会自然“低头”
    float final_tau_m1 = (smooth_dir_m1 * coulomb_fric_m1) + err_i_m1;
    float final_tau_m2 = tau_gravity_m2 + (smooth_dir_m2 * coulomb_fric_m2) + hold_fric_m2 + err_i_m2;
    float final_tau_m3 = tau_gravity_m3 + (smooth_dir_m3 * coulomb_fric_m3) + hold_fric_m3 + err_i_m3;
    float final_tau_m4 = tau_gravity_m4 + (smooth_dir_m4 * coulomb_fric_m4) + err_i_m4;

    arm_debug.cmd_m2_pos = current_m2_cmd;
    arm_debug.cmd_m3_pos = current_m3_cmd;
    arm_debug.actual_m2_pos = Arm_Motors[1].POS;
    arm_debug.actual_m3_pos = Arm_Motors[2].POS;
    arm_debug.err_i_m2 = err_i_m2;
    arm_debug.err_i_m3 = err_i_m3;
    arm_debug.tau_ff_m2 = final_tau_m2;
    arm_debug.tau_ff_m3 = final_tau_m3;

    // 发送指令到CAN总线
    DM_Send_Ctrl(0x01, current_m1_cmd, v_des_m1, Kp_m1, Kd_m1, final_tau_m1);
    DM_Send_Ctrl(0x02, current_m2_cmd, v_des_m2, Kp_m2, Kd_m2, final_tau_m2);
    DM_Send_Ctrl(0x03, current_m3_cmd, v_des_m3, Kp_m3, Kd_m3, final_tau_m3);
    DM_Send_Ctrl(0x04, current_m4_cmd, v_des_m4, Kp_m4, Kd_m4, final_tau_m4);
}
