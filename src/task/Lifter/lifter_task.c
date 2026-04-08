//
// Created by w on 25-11-6.
//


#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "robot.h"
// #include "DM_motor.h"
#include "lifter_task.h"
#define LF 0
#define RF 1
#define LB 2
#define RB 3
#define PITCH_OFFSET (-0.957)
#define ROLL_OFFSET (-1.875)
#define IMU_FILTER_ALPHA_PIT  0.20f
#define IMU_FILTER_ALPHA_ROLL  0.1f
#define IMU_FILTER_ALPHA_Z  0.4f
/*
                  _ooOoo_
                 o8888888o
                 88" . "88
                 (| -_- |)
                 O\ = /O
              ____/`---'\____
            .' \\| |// `
           / \\||| : |||// \
          / _||||| -:- |||||- \
          | | \\\ - /// | |   |
          | \_| ''\---/'' | |
          \ .-\__ `-` ___/-. /
        ___`. .' /--.--\ `. . __
     ."" '< `.___\_<|>_/___.' >'""
    | | : `- \`.;`\ _ /`;.`/ - ` : | |
    \ \ `-. \_ __\ /__ _/ .-` / /
======`-.____`-.___\_____/___.-`____.-'======
                  `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
           佛祖保佑 永无BUG
*/
//文总是全栈工程师
// ========== 全局变量 ==========
static LifterController_t g_lifter_ctrl;
static struct ins_msg ins_data;
static struct lifter_cmd_msg lifter_cmd;
static struct lifter_fdb_msg lifter_fdb;

MCN_DECLARE(lifter_cmd_topic);
MCN_DECLARE(lifter_fdb_topic);
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static McnNode_t lifter_cmd_node;
static float imu_pitch_filt = 0.0f;
static float imu_roll_filt = 0.0f;
static float imu_z_filt  = 0.0f;
 unitree_motor_object_t *lifter_motor[4];  // 4个Unitree升降电机
static struct lifter_controller_t
{
    pid_obj_t *speed_pid;
}lifter_pid_controller [4];
static struct lifter_angle_t
{
    pid_obj_t *pitch_pid ;
    pid_obj_t *roll_pid ;
}lifter_angle_controller[4] ;
pid_obj_t *z_accel_controller  ;
// ========== 卡尔曼滤波器常量 ==========
#define HEIGHT_KF_STATE_DIM 2  // 状态维度：[h, dh]
#define HEIGHT_KF_CTRL_DIM  0  // 无控制输入
#define HEIGHT_KF_MEAS_DIM  1  // 测量维度：[h]
#define HEIGHT_KF_DT        0.001f  // 控制周期 1ms
#define LIFTER0_MIN_ANGLE 0.87916475f
#define LIFTER0_MAX_ANGLE (-4.8182f)
#define LIFTER1_MIN_ANGLE 3.56248f
#define LIFTER1_MAX_ANGLE 9.38184834f
#define LIFTER2_MIN_ANGLE 4.41020489f
#define LIFTER2_MAX_ANGLE 9.55365467f
#define LIFTER3_MIN_ANGLE 2.20376015f
#define LIFTER3_MAX_ANGLE (-3.0234834f)
float MOTOR_POS_OFFSET[4] = {12.1740828f, -3.12683535f, -5.81763554f, 14.8328609f}; // 电机位置零点偏移 (根据实际调试设置)

// ========函数的声明========

void SetRefState(LifterController_t *lifter,struct lifter_cmd_msg cmd);
static void LifterCtrl_StateHander();
static void lifter_pub_push(void);
static void lifter_sub_init(void);
static void lifter_sub_pull(void);

void GetTargetHeight(LifterController_t *lifter,struct lifter_cmd_msg lifter_cmd_fuc );
void LifterInit(void)
{

    lifter_sub_init();
    lifter_motor_init();
    // 初始化升降控制器
    // 初始化并注册电机
    Lifter_Init(&g_lifter_ctrl);
}

int lifter_ = 0;

void lifter_control_task(void)
{
    lifter_sub_pull();
    // lifter_++;
    // 2. 更新底盘状态（从传感器读取）
    Lifter_UpdateState(&g_lifter_ctrl);
    LifterCtrl_StateHander();
    // 3. LQR控制计算（计算关节力矩）
    LQR_Control(&g_lifter_ctrl);

    // 4. 调用Unitree电机控制（自动调用所有电机的控制回调函数并发送指令）
    unitree_motor_control();

    // 5. 更新发布者数据
    lifter_pub_push();


}

/**
 * @brief 底盘状态机处理
 */
static void LifterCtrl_StateHander()
{
    if ( lifter_cmd.ctrl_mode != LIFTER_RELAX)
    {
        Lifter_Enable();
        for (int i = 0; i < 4; i++)
        {
            unitree_motor_enable(lifter_motor[i]);

        }
    }
    SetRefState(&g_lifter_ctrl, lifter_cmd);
    switch (lifter_cmd.ctrl_mode)
    {
        case LIFTER_RELAX: //失能时需要把底盘高度下降，达到最低点，才能彻底给电机失能

            if (fabs(g_lifter_ctrl.state.h - g_lifter_ctrl.state.h_ref <= 0.2f))
            {
                lifter_fdb.back_mode = LIFTER_BACK_IS_OK;
                Lifter_Disable();
                for (int i = 0; i < 4; i++)
                {
                    unitree_motor_disable(lifter_motor[i]);
                }
            } else
            {
                lifter_fdb.back_mode = LIFTER_BACK_STEP;
            }
            break;
        case LIFTER_FLY:
            // SetRefState(&g_lifter_ctrl,lifter_cmd);
            break;
        case LIFTER_HEIGHT_CHANGE:

            // SetRefState(&g_lifter_ctrl,lifter_cmd);
            break;
        case LIFTER_SPIN:
            // SetRefState(&g_lifter_ctrl,lifter_cmd);
            break;
        case LIFTER_HEIGHT_KEEP:
            // SetRefState(&g_lifter_ctrl,lifter_cmd);
            break;
        case LIFTER_HEIGHT_INIT:
            if (fabs(g_lifter_ctrl.state.h - g_lifter_ctrl.state.h_ref <= 0.01f)) //小于等于5cm
            {
                lifter_fdb.back_mode = LIFTER_BACK_IS_OK; //完成归中
            } else
            {
                lifter_fdb.back_mode = LIFTER_BACK_STEP; //正在归中
            }
            break;

        case LIFTER_CLIMB:
            g_lifter_ctrl.leg.ref_joint_angle[0]=30.0F;
            g_lifter_ctrl.leg.ref_joint_angle[1]=80.0f;
            g_lifter_ctrl.leg.ref_joint_angle[2]=75.0f;
            g_lifter_ctrl.leg.ref_joint_angle[3]=0.0f;
            // g_lifter_ctrl.leg.ref_joint_angle[0]=80.0F;
            // g_lifter_ctrl.leg.ref_joint_angle[1]=80.0f;
            // g_lifter_ctrl.leg.ref_joint_angle[2]=0.0f;
            // g_lifter_ctrl.leg.ref_joint_angle[3]=70.02f;
            break;
            case LIFTER_BACK_UP:
            g_lifter_ctrl.leg.ref_joint_angle[3]=lifter_cmd.motor3angel;
            break;
    }
}
// alpha 越小，滤波越平滑，但响应越慢




void low_filter_angle(float imu_pitch_meas,float imu_roll_meas,float imu_z_meas)
{
    imu_pitch_filt = imu_pitch_filt +
                     IMU_FILTER_ALPHA_PIT * (imu_pitch_meas - imu_pitch_filt);
    imu_roll_filt = imu_roll_filt +IMU_FILTER_ALPHA_ROLL * (imu_roll_meas - imu_roll_filt);
    imu_z_filt=imu_z_filt+IMU_FILTER_ALPHA_Z*(imu_z_meas-imu_z_filt);
}
// ========== 多项式求值函数 ==========
/**
 * @brief 计算多项式的值
 * @param coeff 多项式系数数组 [c0, c1, c2, ..., cn]
 * @param order 多项式阶数
 * @param x 自变量
 * @return 多项式值 = c0 + c1*x + c2*x^2 + ... + cn*x^n
 */
float Poly_Eval(const float *coeff, int order, float x)
{
    // 使用 Horner 法则提高计算效率
    // p(x) = c0 + x*(c1 + x*(c2 + x*(c3 + ...)))
    float result = coeff[order];
    for (int i = order - 1; i >= 0; i--)
    {
        result = result * x + coeff[i];
    }
    return result;
}

// ========== 正运动学 ==========
/**
 * @brief 根据关节角度计算足端位置（正运动学）
 * @param mech 机构参数
 * @param theta_deg 关节电机角度 (度)
 * @param x 输出X坐标 (mm)
 * @param y 输出Y坐标 (mm)
 */
void FK_FootPosition(const LegMechanism_t *mech, float theta_deg, float *x, float *y)
{
    *x = Poly_Eval(mech->poly.px, POLY_ORDER, theta_deg);
    *y = -Poly_Eval(mech->poly.py, POLY_ORDER, theta_deg);
}

// ========== 雅可比矩阵计算 ==========
/**
 * @brief 计算雅可比矩阵（使用预计算的导数系数）
 * @param mech 机构参数
 * @param theta_deg 关节电机角度 (度)
 * @param J11 输出 dx/dtheta
 * @param J21 输出 dy/dtheta
 * 
 * 雅可比矩阵: J = [dx/dtheta]
 *                 [dy/dtheta]
 * 
 * 使用从MATLAB获得的导数系数dpx和dpy，避免在线求导
 */
void Jacobian_Compute(const LegMechanism_t *mech, float theta_deg, float *J11, float *J21)
{
    // 使用预计算的导数系数（POLY_ORDER-1阶）
    *J11 = Poly_Eval(mech->poly.dpx, POLY_ORDER - 1, theta_deg);
    *J21 = Poly_Eval(mech->poly.dpy, POLY_ORDER - 1, theta_deg);
}

// ========== 力矩转换 ==========
/**
 * @brief 足端力转换为关节力矩
 * @param mech 机构参数
 * @param leg_idx 腿编号 (0-3)
 * @param Fx 足端X方向力 (N)
 * @param Fy 足端Y方向力 (N)
 * @param torque 输出关节力矩 (N·m)
 * 
 * 公式: tau = J^T * F
 *       tau = dx/dtheta * Fx + dy/dtheta * Fy
 * 
 * 单位转换说明:
 *   - 雅可比矩阵 J 的单位: mm/deg (对度求导)
 *   - 力 F 的单位: N
 *   - 需要两个单位转换:
 *     (1) mm -> m: 除以1000
 *     (2) deg -> rad: 乘以 π/180 ≈ 0.017453293
 *   - 完整公式: τ = (J^T * F) *180/3.14 / 1000
 */
void Force_To_Torque(const LegMechanism_t *mech, int leg_idx, 
                     float Fx, float Fy, float *torque)
{
    // 获取当前关节角度
    float theta_deg = g_lifter_ctrl.leg.joint_angle[leg_idx];
    
    // 计算雅可比矩阵 (单位: mm/deg)
   static float J11, J21;
    Jacobian_Compute(mech, theta_deg, &J11, &J21);
    
    // 力矩计算 (单位转换: mm->m, deg->rad)
    // 180/pi / 1000
    float torque_Nmm_per_deg = (J11*J11+J21*J21)/J21*Fy;  // N·mm/deg
    *torque = torque_Nmm_per_deg * 0.0572957795f;     // N·m
}

// ========== 电机控制回调函数 ==========
/**
 * @brief 电机0控制回调函数（前左）
 * @param motor 电机对象指针
 */
float torqueaa=0;
static void motor_control_0(unitree_motor_object_t *motor)
{
    if (!g_lifter_ctrl.enable) {
        // 未使能时，清零控制参数
        unitree_motor_clear_ctrl(motor);
        return;
    }
    
    // 从LQR控制器获取计算好的关节力矩
    float torque = g_lifter_ctrl.leg.joint_torque[0]/6.33000f;

    float pid_out=-pid_calculate(lifter_pid_controller[0].speed_pid,g_lifter_ctrl.leg.joint_angle[0],g_lifter_ctrl.leg.ref_joint_angle[0]);
    torque=torque+pid_out;
    torqueaa=torque;
    VAL_LIMIT(torque,-2.5f,2.5f);
    // 设置电机控制参数（纯力矩控制模式：kp=0, kd=0）
    // pos=0, vel=0, tor=目标力矩, kp=0, kd=0
    unitree_motor_set_control(motor, 0.0f, 0.0f, torque, 0.0f, lifter_cmd.Kd);
}

/**
 * @brief 电机1控制回调函数（前右）
 * @param motor 电机对象指针
 */
static void motor_control_1(unitree_motor_object_t *motor)
{
    if (!g_lifter_ctrl.enable) {
        unitree_motor_clear_ctrl(motor);
        return;
    }
    
    float torque = -g_lifter_ctrl.leg.joint_torque[1]/6.33000f;
    float pid_out=pid_calculate(lifter_pid_controller[1].speed_pid,g_lifter_ctrl.leg.joint_angle[1],g_lifter_ctrl.leg.ref_joint_angle[1]);
    torque=torque+pid_out;
    VAL_LIMIT(torque,-2.5f,2.5f);
    // 设置电机控制参数（纯力矩控制模式：kp=0, kd=0）
    // pos=0, vel=0, tor=目标力矩, kp=0, kd=0
    unitree_motor_set_control(motor, 0.0f, 0.0f, torque, 0.0f, lifter_cmd.Kd);
}

/**
 * @brief 电机2控制回调函数（后左）
 * @param motor 电机对象指针
 */
static void motor_control_2(unitree_motor_object_t *motor)
{
    if (!g_lifter_ctrl.enable) {
        unitree_motor_clear_ctrl(motor);
        return;
    }
    
    float torque = -g_lifter_ctrl.leg.joint_torque[2]/6.33000f;
    float pid_out=pid_calculate(lifter_pid_controller[2].speed_pid,g_lifter_ctrl.leg.joint_angle[2],g_lifter_ctrl.leg.ref_joint_angle[2]);
    torque=torque+pid_out;
    VAL_LIMIT(torque,-2.5f,2.5f);
    // 设置电机控制参数（纯力矩控制模式：kp=0, kd=0）
    // pos=0, vel=0, tor=目标力矩, kp=0, kd=0
    unitree_motor_set_control(motor, 0.0f, 0.0f, torque, 0.0f, lifter_cmd.Kd);
}

/**
 * @brief 电机3控制回调函数（后右）
 * @param motor 电机对象指针
 */
static void motor_control_3(unitree_motor_object_t *motor)
{
    if (!g_lifter_ctrl.enable) {
        unitree_motor_clear_ctrl(motor);
        return;
    }
    
    float torque = g_lifter_ctrl.leg.joint_torque[3]/6.33000f;
    float pid_out=-pid_calculate(lifter_pid_controller[3].speed_pid,g_lifter_ctrl.leg.joint_angle[3],g_lifter_ctrl.leg.ref_joint_angle[3]);
    torque=torque+pid_out;
    VAL_LIMIT(torque,-2.5f,2.5f);
    // 设置电机控制参数（纯力矩控制模式：kp=0, kd=0）
    // pos=0, vel=0, tor=目标力矩, kp=0, kd=0
    unitree_motor_set_control(motor, 0.0f, 0.0f, torque, 0.0f, lifter_cmd.Kd);
}

/**
 * @brief 电机控制回调函数数组
 */
static void (*motor_control[4])(unitree_motor_object_t *) = {
    motor_control_0,
    motor_control_1,
    motor_control_2,
    motor_control_3
};

// ========== 电机配置 ==========
/**
 * @brief Unitree电机配置数组
 * 
 * 电机布局：
 * [0] 前左腿 (Front Left)
 * [1] 前右腿 (Front Right)
 * [2] 后左腿 (Rear Left)
 * [3] 后右腿 (Rear Right)
 * 
 * 通信方案：单路 RS485（4个电机共用 USART2）
 * 带宽占用：132µs / 1ms = 13.2%，充足！
 * 
 * 注意：
 * 1. 确保每个电机的拨码开关ID设置正确（0、1、2、3）
 * 2. RS485总线使用菊花链拓扑，两端接120Ω终端电阻
 * 3. 如需改回双路方案，将电机2、3的channel改为usart3_485
 */
 unitree_motor_config_t lifter_motor_config[4] = {
    {
        .motor_id = 0,              // 电机ID：0（确保与拨码开关一致）
        .channel = usart2_485,      // 通信通道：USART2
        .mode = UNITREE_MODE_FOC,   // 工作模式：FOC闭环控制
    },
    {
        .motor_id = 1,              // 电机ID：1
        .channel = usart2_485,      // 通信通道：USART2
        .mode = UNITREE_MODE_FOC,
    },
    {
        .motor_id =2 ,              // 电机ID：2
        .channel = usart2_485,      // 通信通道：USART2（单路方案）
        .mode = UNITREE_MODE_FOC,
    },
    {
        .motor_id = 3,              // 电机ID：3
        .channel = usart2_485,      // 通信通道：USART2（单路方案）
        .mode = UNITREE_MODE_FOC,
    }
};

// ========== 电机初始化函数 ==========
/**
 * @brief 注册升降电机及其控制器初始化
 * 
 * 该函数注册4个Unitree电机并设置控制回调函数
 */
 void lifter_motor_init(void)
{
     lifter_motor[0] = unitree_motor_register(&lifter_motor_config[0], motor_control_0);
     lifter_motor[1] = unitree_motor_register(&lifter_motor_config[1], motor_control_1);
     lifter_motor[2] = unitree_motor_register(&lifter_motor_config[2], motor_control_2);
     lifter_motor[3] = unitree_motor_register(&lifter_motor_config[3], motor_control_3);
    for (uint8_t i = 0; i < 4; i++)
    {
        // 注册电机实例


        // 检查注册是否成功
        if (lifter_motor[i] == NULL) {
            // TODO: 添加错误处理（如LED指示、日志记录等）
            continue;
        }

        // 使能电机（进入FOC模式）
        unitree_motor_enable(lifter_motor[i]);



    }
     pid_config_t lifter_pid_config  = INIT_PID_CONFIG(LIFTER_KP_V_MOTOR, LIFTER_KI_V_MOTOR, LIFTER_KD_V_MOTOR, LIFTER_INTEGRAL_V_MOTOR, LIFTER_MAX_V_MOTOR,
                                                      (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
     pid_config_t lifter_pitch_config =INIT_PID_CONFIG(LIFTER_KP_PA_MOTOR, LIFTER_KI_PA_MOTOR, LIFTER_KD_PA_MOTOR, LIFTER_INTEGRAL_PA_MOTOR, LIFTER_MAX_PA_MOTOR,
                                                      (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
     pid_config_t lifter_roll_config =INIT_PID_CONFIG(LIFTER_KP_RA_MOTOR, LIFTER_KI_RA_MOTOR, LIFTER_KD_RA_MOTOR, LIFTER_INTEGRAL_RA_MOTOR, LIFTER_MAX_RA_MOTOR,
                                                      (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
     pid_config_t lifter_z_config=INIT_PID_CONFIG(LIFTER_KP_Z_MOTOR, LIFTER_KI_Z_MOTOR, LIFTER_KD_Z_MOTOR, LIFTER_INTEGRAL_Z_MOTOR, LIFTER_MAX_Z_MOTOR,
                                                      (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
     z_accel_controller=pid_register(&lifter_z_config);
     for (uint8_t j = 0; j < 4; j++)
     {
         lifter_pid_controller[j].speed_pid = pid_register(&lifter_pid_config );
         lifter_angle_controller[j].pitch_pid = pid_register(&lifter_pitch_config  );
         lifter_angle_controller[j].roll_pid = pid_register(&lifter_roll_config  );
     }
}
////TODO 为后续添加控制底盘倾角的功能 ，目前只能改变底盘高度和速度
/**/


int shuipin=1;
void SetRefState(LifterController_t *lifter,struct lifter_cmd_msg cmd)
{
     FK_FootPosition(&lifter->mechanism,
                         cmd.target_angle,
                         &lifter->state.x_ref,
                         &lifter->state.h_ref);
     lifter->state.h_ref=lifter->state.h_ref/1000;

     float J11, J21;

     Jacobian_Compute(&lifter->mechanism, cmd.dTarget_angle, &J11, &J21);
     lifter->state.dh_ref = J21*cmd.dTarget_angle;
     for (int i = 0; i < 4; i++)
     {
         lifter->leg.ref_joint_angle[i] = cmd.target_angle;
     }
    if (shuipin == 1)
    {
        low_filter_angle(ins_data.pitch,ins_data.roll,ins_data.motion_accel_n[2]);
        lifter->leg.ref_joint_angle[0]+=pid_calculate(lifter_angle_controller[0].pitch_pid,imu_pitch_filt,PITCH_OFFSET)
                                    -pid_calculate(lifter_angle_controller[0].roll_pid,imu_roll_filt,ROLL_OFFSET);

        lifter->leg.ref_joint_angle[1]+=pid_calculate(lifter_angle_controller[1].pitch_pid,imu_pitch_filt,PITCH_OFFSET)
                                       +pid_calculate(lifter_angle_controller[1].roll_pid,imu_roll_filt,ROLL_OFFSET);

        lifter->leg.ref_joint_angle[2]+=-pid_calculate(lifter_angle_controller[2].pitch_pid,imu_pitch_filt,PITCH_OFFSET)
                                       -pid_calculate(lifter_angle_controller[2].roll_pid,imu_roll_filt,ROLL_OFFSET);

        lifter->leg.ref_joint_angle[3]+=-pid_calculate(lifter_angle_controller[3].pitch_pid,imu_pitch_filt,PITCH_OFFSET)
                                       +pid_calculate(lifter_angle_controller[3].roll_pid,imu_roll_filt,ROLL_OFFSET);

    }
    for (int i = 0; i < 4; i++)
    {
        // lifter->leg.ref_joint_angle[i]-=pid_calculate(z_accel_controller,imu_z_filt,0);
    }
     for (int i = 0; i < 4; i++)
     {
         VAL_LIMIT(lifter->leg.ref_joint_angle[i],15.0f,80.0f);
     }
     lifter->enable=cmd.enable;
    // lifter->state.h_ref=cmd.height;
    // lifter->state.dh_ref=cmd.d_height;
     // lifter->leg.ref_joint_angle
    lifter->state.roll_ref=0;
    lifter->state.pitch_ref=0;
    lifter->state.dpitch_ref=0;
    lifter->state.droll_ref=0;
}
// ========== LQR控制器 ==========
/**
 * @brief LQR状态反馈控制（使用算法层）
 * @param lifter 升降控制器
 *
 *
 * 控制律: u = u_ff - K * (x - x_ref)
 * 其中 u_ff = [mg/4, mg/4, mg/4, mg/4]^T 是重力补偿前馈
 */
void LQR_Control(LifterController_t *lifter)
{
    if (!lifter->enable || lifter->lqr_controller == NULL) {
        memset(lifter->leg.foot_force_y,0,sizeof(lifter->leg.foot_force_y));
        memset(lifter->leg.joint_torque,0,sizeof(lifter->leg.joint_torque));
        return;
    }
    
    lqr_object_t *lqr = (lqr_object_t *)lifter->lqr_controller;
    
    // ===== 组装状态向量 =====
    float state[LIFTER_STATE_DIM] = {
        lifter->state.h,
        lifter->state.roll,
        lifter->state.pitch,
        lifter->state.dh,
        lifter->state.droll,
        lifter->state.dpitch
    };
    
    float state_ref[LIFTER_STATE_DIM] = {
        lifter->state.h_ref,
        lifter->state.roll_ref,
        lifter->state.pitch_ref,
        lifter->state.dh_ref,
        lifter->state.droll_ref,
        lifter->state.dpitch_ref
    };
    
    // ===== 调用LQR算法层计算控制输出 =====
    float *u = lqr_update(lqr, state, state_ref);
    
    // ===== 保存虚拟足端力 =====
    for (int i = 0; i < LIFTER_CONTROL_DIM; i++) {
        lifter->leg.foot_force_x[i] = 0.0f;  // 假设仅在Y方向施力
        lifter->leg.foot_force_y[i] = u[i];
    }
    
    // ===== 足端力转换为关节力矩 =====
    for (int i = 0; i < 4; i++) {
        Force_To_Torque(&lifter->mechanism, i, 
                        lifter->leg.foot_force_x[i],
                        lifter->leg.foot_force_y[i],
                        &lifter->leg.joint_torque[i]);
        
        // 力矩限幅
        if (fabsf(lifter->leg.joint_torque[i]) > lifter->max_torque) {
            lifter->leg.joint_torque[i] = copysignf(lifter->max_torque,
                                                     lifter->leg.joint_torque[i]);
        }
    }
}
void GetTargetHeight(LifterController_t *lifter,struct lifter_cmd_msg lifter_cmd_fuc )
 {


        FK_FootPosition(&lifter->mechanism,
                         lifter_cmd_fuc.target_angle,
                         &lifter->state.x_ref,
                         &lifter->state.h_ref);
     lifter->state.h_ref=lifter->state.h_ref/1000;

     float J11, J21;

     Jacobian_Compute(&lifter->mechanism, lifter_cmd_fuc.dTarget_angle, &J11, &J21);
     lifter->state.dh_ref = J21*lifter_cmd_fuc.dTarget_angle;
     for (int i = 0; i < 4; i++)
     {
         lifter->leg.ref_joint_angle[i] = lifter_cmd_fuc.target_angle;
     }

 }

// ========== 状态更新 ==========
/**
 * @brief 更新底盘状态（从传感器读取）
 * @param lifter 升降控制器
 */
void Lifter_UpdateState(LifterController_t *lifter)
{
    // ===== 从IMU读取姿态角和角速度 =====
    // 注意：BMI088损坏，暂时将姿态角和角速度置0
    lifter->state.roll = ins_data.roll * 0.017453293f;      // 度 -> 弧度
    lifter->state.pitch = ins_data.pitch * 0.017453293f;    // 度 -> 弧度
    lifter->state.droll = ins_data.roll_gyro;                 // rad/s (无需转换)
    lifter->state.dpitch = -ins_data.pitch_gyro;                // rad/s (无需转换)


    lifter->leg.joint_angle[LF]= -(lifter_motor[LF]->measure.position - MOTOR_POS_OFFSET[LF]) * 57.2957795f/6.33f;
    lifter->leg.joint_angle[RF]= (lifter_motor[RF]->measure.position - MOTOR_POS_OFFSET[RF]) * 57.2957795f/6.33f;
    lifter->leg.joint_angle[LB]= (lifter_motor[LB]->measure.position - MOTOR_POS_OFFSET[LB]) * 57.2957795f/6.33f;
    lifter->leg.joint_angle[RB]= -(lifter_motor[RB]->measure.position - MOTOR_POS_OFFSET[RB]) * 57.2957795f/6.33f;
    // ===== 更新足端位置（正运动学）=====
    for (int i = 0; i < 4; i++) {
        FK_FootPosition(&lifter->mechanism, 
                        lifter->leg.joint_angle[i],
                        &lifter->leg.foot_pos_x[i],
                        &lifter->leg.foot_pos_y[i]);
    }
    
    // ===== 使用卡尔曼滤波器估计高度和速度 =====
    // 1. 从足端位置计算原始高度测量值
    float h_measured = Estimate_Height_From_Legs(lifter);
    
    // 2. 使用卡尔曼滤波器进行滤波和速度估计
    //    - 输入: 原始高度测量值（含噪声）
    //    - 输出: 滤波后的高度 + 通过微分估计的速度
    Height_KF_Update(lifter, h_measured);

    lifter_fdb.real_height = lifter->state.h;
    lifter_fdb.real_dheight= lifter->state.dh;

}

// ========== 初始化函数 ==========
/**
 * @brief 初始化升降控制器
 * @param lifter 升降控制器
 */
static void Lifter_Init(LifterController_t *lifter)
{
    memset(lifter, 0, sizeof(LifterController_t));
    
    // ===== 机构参数 (从MATLAB仿真获得) =====
    lifter->mechanism.L1 = 70.0f;
    lifter->mechanism.L2 = 100.0f;
    lifter->mechanism.L3 = 135.0f;
    lifter->mechanism.L4 = 70.0f;
    lifter->mechanism.L_small = 50.0f;
    lifter->mechanism.L3_offset = 65.0f;
    
    // // px[0] + px[1]*θ + px[2]*θ² + px[3]*θ³ + px[4]*θ⁴
    // float px_temp[POLY_ORDER + 1] = {
    //     1.0606676908e+02f,
    //     3.8015609946e+00f,
    //     -5.8677986815e-02f,
    //     4.3021717530e-04f,
    //     -1.6492643775e-06f
    // };
    //
    // // py[0] + py[1]*θ + py[2]*θ² + py[3]*θ³ + py[4]*θ⁴
    // float py_temp[POLY_ORDER + 1] = {
    //     -1.6411796029e+02f,
    //     1.6630830586e+00f,
    //     1.8173751654e-02f,
    //     -1.9681505558e-04f,
    //     5.4781180031e-07f
    // };
    //
    // // 导数多项式系数（3阶，雅可比矩阵，常数项在前）
    // // dpx[0] + dpx[1]*θ + dpx[2]*θ² + dpx[3]*θ³
    // float dpx_temp[POLY_ORDER] = {
    //     3.8015609946e+00f,
    //     -1.1735597363e-01f,
    //     1.2906515259e-03f,
    //     -6.5970575100e-06f
    // };
    //
    // // dpy[0] + dpy[1]*θ + dpy[2]*θ² + dpy[3]*θ³
    // float dpy_temp[POLY_ORDER] = {
    //     1.6630830586e+00f,
    //     3.6347503308e-02f,
    //     -5.9044516675e-04f,
    //     2.1912472012e-06f
    // };
    // 位置多项式系数（4阶，常数项在前）
    // px[0] + px[1]*θ + px[2]*θ² + px[3]*θ³ + px[4]*θ⁴
    float px_temp[POLY_ORDER + 1] = {
        1.0595894884e+02f,
        3.8348864509e+00f,
        -6.0839910899e-02f,
        4.7725296443e-04f,
        -1.9718857338e-06f
    };

    // py[0] + py[1]*θ + py[2]*θ² + py[3]*θ³ + py[4]*θ⁴
    float py_temp[POLY_ORDER + 1] = {
        -1.6409350461e+02f,
        1.6552915183e+00f,
        1.8684052126e-02f,
        -2.0797163114e-04f,
        6.2457216113e-07f
    };

    // 导数多项式系数（3阶，雅可比矩阵，常数项在前）
    // dpx[0] + dpx[1]*θ + dpx[2]*θ² + dpx[3]*θ³
    float dpx_temp[POLY_ORDER] = {
        3.8348864509e+00f,
        -1.2167982180e-01f,
        1.4317588933e-03f,
        -7.8875429353e-06f
    };

    // dpy[0] + dpy[1]*θ + dpy[2]*θ² + dpy[3]*θ³
    float dpy_temp[POLY_ORDER] = {
        1.6552915183e+00f,
        3.7368104253e-02f,
        -6.2391489343e-04f,
        2.4982886445e-06f
    };

    memcpy(lifter->mechanism.poly.px, px_temp, sizeof(px_temp));
    memcpy(lifter->mechanism.poly.py, py_temp, sizeof(py_temp));
    

    memcpy(lifter->mechanism.poly.dpx, dpx_temp, sizeof(dpx_temp));
    memcpy(lifter->mechanism.poly.dpy, dpy_temp, sizeof(dpy_temp));
    
    // ===== 系统参数 (从LQR.m获得) =====
    lifter->mass = 4.50f;       // kg
    lifter->I_xx = 0.5f;        // kg·m²
    lifter->I_yy = 0.6f;        // kg·m²
    lifter->gravity = 9.786f;   // m/s²
    lifter->L_x = 0.20f;        // m
    lifter->L_y = 0.15f;        // m
    
    // ===== 控制限制 =====
    lifter->max_force = lifter->mass * lifter->gravity * 0.9f;  // 最大支撑力
    lifter->min_force = 10.0f;  // 最小支撑力 (N)
    lifter->max_torque = 10.0f;  // 最大关节力矩 (N·m)
    
    // ===== 在 Lifter_Init() 函数中替换 K_matrix =====

     static float K_matrix[LIFTER_CONTROL_DIM * LIFTER_STATE_DIM] = {
         // 行1: F1的增益
         // 3.5355339059e+01f, 3.5355339059e+01f, -3.5355339059e+01f, 7.4311069083e+00f, 1.4418212030e+01f, -1.4326255431e+01f,
         // // 行2: F2的增益
         // 3.5355339059e+01f, -3.5355339059e+01f, -3.5355339059e+01f, 7.4311069083e+00f, -1.4418212030e+01f, -1.4326255431e+01f,
         // // 行3: F3的增益
         // 3.5355339059e+01f, 3.5355339059e+01f, 3.5355339059e+01f, 7.4311069083e+00f, 1.4418212030e+01f, 1.4326255431e+01f,
         // // 行4: F4的增益
         // 3.5355339059e+01f, -3.5355339059e+01f, 3.5355339059e+01f, 7.4311069083e+00f, -1.4418212030e+01f, 1.4326255431e+01f
     };


    
    // 前馈控制（重力补偿）
    static float feedforward[LIFTER_CONTROL_DIM];
    float F_gravity = lifter->mass * lifter->gravity / 4.0f;
    for (int i = 0; i < LIFTER_CONTROL_DIM; i++) {
        feedforward[i] = F_gravity;
    }
    
    // 控制限幅
    static float control_min[LIFTER_CONTROL_DIM];
    static float control_max[LIFTER_CONTROL_DIM];
    for (int i = 0; i < LIFTER_CONTROL_DIM; i++) {
        control_min[i] = lifter->min_force;
        control_max[i] = lifter->max_force;
    }
    
    // LQR配置
    lqr_config_t lqr_cfg = {
        .state_dim = LIFTER_STATE_DIM,
        .control_dim = LIFTER_CONTROL_DIM,
        .k_type = LQR_K_FIXED,
        .K_data      = K_matrix,
        .feedforward = feedforward,
        
        
    };
    
    // 注册LQR控制器
    lifter->lqr_controller = (void *)lqr_register(&lqr_cfg,NULL);
    
    if (lifter->lqr_controller != NULL) {
        lqr_Enable((lqr_object_t *)lifter->lqr_controller);
    }
    
    // ===== 初始化卡尔曼滤波器（高度-速度估计）=====
    Height_KF_Init(lifter);
    
    // ===== 初始状态 =====
    lifter->state.h_ref = 0.042f;      // 期望高度 0.3m
    lifter->state.roll_ref = 0.0f;     // 期望Roll角 0°
    lifter->state.pitch_ref = 0.0f;   // 期望Pitch角 0°
    lifter->state.dh_ref = 0.0f;      // 期望竖直速度 0
    lifter->state.droll_ref = 0.0f;    // 期望Roll角速度 0
    lifter->state.dpitch_ref = 0.0f;  // 期望Pitch角速度 0
    
    lifter->enable = 0;  // 默认不使能
}

// ========== 辅助函数：使能/失能控制 ==========
void Lifter_Enable(void)
{
    g_lifter_ctrl.enable = 1;
    
    // 使能所有电机（设置为FOC模式）
    for (uint8_t i = 0; i < 4; i++) {
        if (lifter_motor[i] != NULL) {
            unitree_motor_enable(lifter_motor[i]);
        }
    }
}

void Lifter_Disable(void)
{
    g_lifter_ctrl.enable = 0;
    // 清零所有力矩输出
    memset(g_lifter_ctrl.leg.joint_torque, 0, sizeof(g_lifter_ctrl.leg.joint_torque));
    
    // 失能所有电机（设置为IDLE模式）
    for (uint8_t i = 0; i < 4; i++) {
        if (lifter_motor[i] != NULL) {
            unitree_motor_disable(lifter_motor[i]);
        }
    }
}



/**
 * @brief 高度估计（从四条腿足端位置）
 * @param lifter 升降控制器
 * @return 估计的车身高度 (m)
 */
float Estimate_Height_From_Legs(LifterController_t *lifter)
{
    // 方法：取四条腿足端Y坐标（竖直方向）的平均值
    float y_avg = 0.0f;
    for (int i = 0; i < 4; i++) {
        y_avg += lifter->leg.foot_pos_y[i];
    }
    y_avg /= 4.0f;
    
    // 转换为米（足端坐标单位是mm）
    float height_m = y_avg / 1000.0f;
    
    return height_m;//因为计算出来的高度是负值 正负号
}

// ========== 卡尔曼滤波器（高度-速度估计）==========
/**
 * @brief 初始化高度-速度卡尔曼滤波器
 * @param lifter 升降控制器
 * 
 * 状态向量: x = [h, dh]^T
 *   h  - 高度 (m)
 *   dh - 速度 (m/s)
 * 
 * 测量向量: z = [h_measured]
 *   h_measured - 从足端位置计算的高度 (m)
 * 
 * 状态转移方程:
 *   h(k)  = h(k-1) + dh(k-1)*dt
 *   dh(k) = dh(k-1)
 * 
 * 测量方程:
 *   z(k) = h(k)
 */

void Height_KF_Init(LifterController_t *lifter)
{
    // 分配卡尔曼滤波器内存
    KalmanFilter_t *kf = (KalmanFilter_t *)user_malloc(sizeof(KalmanFilter_t));
    if (kf == NULL) {
        return;  // 内存分配失败
    }
    
    // 清零卡尔曼滤波器结构体
    memset(kf, 0, sizeof(KalmanFilter_t));

    // 初始化卡尔曼滤波器
    Kalman_Filter_Init(kf, HEIGHT_KF_STATE_DIM, HEIGHT_KF_CTRL_DIM, HEIGHT_KF_MEAS_DIM);
    
    // ===== 初始协方差矩阵 P =====
    // P = |10  0 |  (初始不确定度)
    //     | 0  5 |
    static float P_Init[HEIGHT_KF_STATE_DIM * HEIGHT_KF_STATE_DIM] = {
        10.0f,  0.0f,
        0.0f,   5.0f
    };
    memcpy(kf->P_data, P_Init, sizeof(P_Init));
    
    // ===== 状态转移矩阵 F =====
    // F = |1  dt|  (状态转移：h(k) = h(k-1) + dh(k-1)*dt)
    //     |0  1 |
    static float F_Init[HEIGHT_KF_STATE_DIM * HEIGHT_KF_STATE_DIM] = {
        1.0f,           HEIGHT_KF_DT,
        0.0f,           1.0f
    };
    memcpy(kf->F_data, F_Init, sizeof(F_Init));
    
    // ===== 过程噪声协方差矩阵 Q =====
    // 假设速度存在随机变化，加速度噪声标准差 sigma_a = 0.5 m/s²
    // Q = |dt^4/4  dt^3/2| * sigma_a^2
    //     |dt^3/2  dt^2  |
    float dt = HEIGHT_KF_DT;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    float sigma_a = 0.5f;  // 加速度噪声标准差
    float sigma_a2 = sigma_a * sigma_a;
    
    static float Q_Init[HEIGHT_KF_STATE_DIM * HEIGHT_KF_STATE_DIM];
    Q_Init[0] = 0.25f * dt4 * sigma_a2;  // Q(1,1)
    Q_Init[1] = 0.5f * dt3 * sigma_a2;   // Q(1,2)
    Q_Init[2] = 0.5f * dt3 * sigma_a2;   // Q(2,1)
    Q_Init[3] = dt2 * sigma_a2;          // Q(2,2)
    memcpy(kf->Q_data, Q_Init, sizeof(Q_Init));
    
    // ===== 测量矩阵 H =====
    // H = |1  0|  (只测量高度)
    static float H_Init[HEIGHT_KF_MEAS_DIM * HEIGHT_KF_STATE_DIM] = {
        1.0f,  0.0f
    };
    memcpy(kf->H_data, H_Init, sizeof(H_Init));
    
    // ===== 测量噪声协方差矩阵 R =====
    // R = |sigma_h^2|  (高度测量噪声方差)
    // 假设高度测量标准差为 5mm = 0.005m
    float sigma_h = 0.005f;  // 高度测量噪声标准差
    static float R_Init[HEIGHT_KF_MEAS_DIM * HEIGHT_KF_MEAS_DIM];
    R_Init[0] = sigma_h * sigma_h;
    memcpy(kf->R_data, R_Init, sizeof(R_Init));
    
    // ===== 状态最小方差（防止过度收敛）=====
    static float state_min_variance[HEIGHT_KF_STATE_DIM] = {
        0.0001f,  // 高度最小方差 (0.01mm)
        0.001f    // 速度最小方差 (1mm/s)
    };
    memcpy(kf->StateMinVariance, state_min_variance, sizeof(state_min_variance));
    
    // ===== 初始状态 =====
    kf->xhat_data[0] = 0.3f;   // 初始高度 0.3m
    kf->xhat_data[1] = 0.0f;   // 初始速度 0 m/s
    
    // 不使用自动调整（简单模式）
    kf->UseAutoAdjustment = 0;
    
    // 保存卡尔曼滤波器指针
    lifter->height_kf = (void *)kf;
}

/**
 * @brief 更新高度-速度卡尔曼滤波器
 * @param lifter 升降控制器
 * @param h_measured 从足端位置测量的高度 (m)
 * 
 * 功能：
 *   1. 将测量值传入卡尔曼滤波器
 *   2. 执行卡尔曼滤波更新
 *   3. 提取滤波后的高度和速度
 */
void Height_KF_Update(LifterController_t *lifter, float h_measured)
{
    if (lifter->height_kf == NULL) {
        return;  // 卡尔曼滤波器未初始化
    }
    
    KalmanFilter_t *kf = (KalmanFilter_t *)lifter->height_kf;
    
    // ===== 输入测量值 =====
    kf->MeasuredVector[0] = h_measured;
    
    // ===== 执行卡尔曼滤波 =====
    float *filtered = Kalman_Filter_Update(kf);
    
    // ===== 提取滤波结果 =====
    lifter->state.h = filtered[0];   // 滤波后的高度
    lifter->state.dh = filtered[1];  // 滤波后的速度
}

/**
 * @brief gimbal 线程中所有订阅者初始化
 */
static void lifter_sub_init(void)

{
    // sub_cmd = sub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    ins_topic_node=mcn_subscribe(MCN_HUB(ins_topic),NULL,NULL);
    lifter_cmd_node=mcn_subscribe(MCN_HUB(lifter_cmd_topic),NULL,NULL);

}

/**
 * @brief gimbal 线程中所有发布者推送更新话题
 */
static void lifter_pub_push(void)
{
     // pub_push_msg(, &gim_fdb);
    // pub_push_msg(pub_lifter, &lifter_fdb);
    mcn_publish(MCN_HUB(lifter_fdb_topic),&lifter_fdb);
}

/**
 * @brief gimbal 线程中所有订阅者获取更新话题
 */
static void lifter_sub_pull(void)
{
    // sub_get_msg(sub_cmd, &gim_cmd);
    // sub_get_msg(sub_ins, &ins_data);
    // sub_get_msg(sub_cmd, &lifter_fdb);
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic),ins_topic_node,&ins_data);
    }
    if (mcn_poll(lifter_cmd_node))
    {
        mcn_copy(MCN_HUB(lifter_cmd_topic), lifter_cmd_node, &lifter_cmd);
    }
}
// 每个周期调用的滤波
