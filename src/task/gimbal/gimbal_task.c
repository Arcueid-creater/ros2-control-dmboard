//
// Created by Gleam on 25-8-22.
//

#include <stdlib.h>
#include <math.h>
#include "gimbal_task.h"
#include "rm_config.h"
#include "rm_module.h"
#include "rm_algorithm.h"
#define GIM_PITCH_MOTOR_NUM 1
#define GIM_YAW_MOTOR_NUM 3
#define GIM_MOTOR_NUM 3
#define UP_PITCH_REDUCTION 0.6f
#define yaw_motor 0
#define dowm_pitch_motor 1
#define up_pitch_motor 2
// #define CLOSE_DM_MOTOR
static float control_dt[4];
static float control_start[4];
static GimbalController_t g_gimbal_ctrl;

float last_pitch=0.0f;
float lastoutput=2.0f;
static float pitch_filtered = 0;
float dm_obs[4];
float yaw_filtered=0;
#define DM_RATIO 1.0f
#define DM_OUTPUT_LIMIT 7.0f
/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */

// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chass_cmd;
MCN_DECLARE(gimbal_cmd);
static McnNode_t gimbal_cmd_node;
static struct gimbal_cmd_msg gim_cmd;
MCN_DECLARE(gimbal_ins_topic);
static McnNode_t gimbal_ins_node;
static struct dm_imu_t gim_ins;

// 发布
MCN_DECLARE(gimbal_fdb_topic);
struct gimbal_fdb_msg gimbal_fdb_data;

static void gimbal_pub_push(void);
static void gimbal_sub_init(void);
static void gimbal_sub_pull(void);
float angle_normalize(float angle_deg);
float pitch_calc_motor_angle(float ref_angle);
float get_pitch_form_motor();
static void gimbal_pid_init();
float yaw_filter_angle();
//事先需要在正方向矫正0度

static struct gimbal_controller_t{
    /* 基于imu数据闭环，主要用于手动模式 */
    pid_obj_t *pid_speed_imu;
    pid_obj_t *pid_angle_imu;
    /* 基于imu数据闭环，主要用于自动模式 */
    pid_obj_t *pid_speed_auto;
    pid_obj_t *pid_angle_auto;
}gim_controller[GIM_MOTOR_NUM];

motor_config_t gimbal_motor_config[GIM_MOTOR_NUM] = {
        {
                .motor_type = DM4310,
                .can_id = 2,
                .rx_id = 0x10,
                .tx_id = 0x03,
                .ctrl_mode = MIT_CFG
        },
        {
                .motor_type = DM4310,   //英雄pitch轴改用丝杆结构，换用3508电机
                .can_id = CAN_ID_GIMBAL_MOTOR,
                .rx_id = 0x10,   //电机ID待定
                .ctrl_mode = MIT_CFG,
                .tx_id = 0x02,

        },
        {
            .motor_type = DM4310,
            .can_id = CAN_ID_GIMBAL_MOTOR,
            .rx_id = 0x10,
            .tx_id = 0x01,
            .ctrl_mode = MIT_CFG
        }
};

/* ------------------------------------------------ 云台控制相关 ----------------------------------------------------- */
/* gyro三轴：[0]为X，[1]为Y，[2]为Z */
#define X 0
#define Y 1
#define Z 2
static int16_t yaw_motor_relive, pitch_motor_relive;  // 电机相对于归中值的角度

static dm_motor_object_t *gim_motor[GIM_MOTOR_NUM];  // 底盘电机实例
static float gim_motor_ref[GIM_MOTOR_NUM]; // 电机控制期望值

static void gimbal_motor_init();

/*自瞄相对角传参反馈*/
auto_relative_angle_status_e auto_relative_angle_status=RELATIVE_ANGLE_TRANS;

/* ------------------------------------------------ 云台线程入口 ----------------------------------------------------- */


void gimbal_task_init(void){
    gimbal_sub_init();
    gimbal_motor_init();
    gimbal_pid_init();
    // dm_motor_enable_all();
}

uint8_t data[8];

/* -------------------------------------------- 云台姿态与校准相关 -------------------------------------------- */

/* 云台状态机：上电先校准，然后正常运行 */
typedef enum
{
    GIMBAL_STATE_INIT = 0,      // 等待IMU与电机上电稳定
    GIMBAL_STATE_CALIB_UP_PITCH,// 上pitch 利用IMU找水平并零点关节编码器
    GIMBAL_STATE_CALIB_YAW,     // yaw 归中
    GIMBAL_STATE_SET_IMU_YAW0,  // 记录当前位置为IMU yaw = 0
    GIMBAL_STATE_RUN            // 正常运行，互补滤波
} gimbal_run_state_e;

static gimbal_run_state_e gimbal_state = GIMBAL_STATE_INIT;

/* 简单延时计数，用于 INIT -> CALIB 状态切换（单位：控制周期次数） */
static uint32_t gimbal_init_cnt = 0;

/* 上pitch 编码器零点设置标志，防止重复发送清零命令 */
static uint8_t up_pitch_zero_done = 0;

/* 从 dm_motor 对象读取当前关节角度（弧度，多圈 total_angle 已是rad） */
static float get_yaw_motor_angle(void)
{
    if (gim_motor[yaw_motor] == NULL)
    {
        return 0.0f;
    }
    return angle_normalize(gim_motor[yaw_motor]->measure.total_angle);
}

static float get_down_pitch_motor_angle(void)
{
    if (gim_motor[dowm_pitch_motor] == NULL)
    {
        return 0.0f;
    }
    return angle_normalize(gim_motor[dowm_pitch_motor]->measure.total_angle);
}
static float get_up_pitch_motor_angle(void)
{
    if (gim_motor[up_pitch_motor] == NULL)
    {
        return 0.0f;
    }
    return angle_normalize(gim_motor[up_pitch_motor]->measure.total_angle*UP_PITCH_REDUCTION);
}


static void GimbalCtrl_StateHandler(void)
{
    gimbal_fdb_data.yaw_relative_angle = get_yaw_motor_angle();
    yaw_filter_angle();
    gimbal_fdb_data.pit_offset_angle=gim_ins.pitch;
    switch (gim_cmd.ctrl_mode)
    {

        case GIMBAL_RELAX:
            dm_motor_disable_all();
            gimbal_fdb_data.back_mode = BACK_STEP;
            break;

        case GIMBAL_INIT:
        {
            /* 云台初始化 / 归中过程：允许电机输出，使能 DM，调用内部状态机 */
            dm_motor_enable_all();
            // gim_motor_ref[dowm_pitch_motor]=PI/2.0f;//下pitch抬起一半，不要全部收起
            gim_motor_ref[dowm_pitch_motor]=1.0f;
            gim_motor_ref[up_pitch_motor]=pitch_calc_motor_angle(0);
            gim_motor_ref[yaw_motor]=0.0f;//·
            if (fabsf(gim_motor[dowm_pitch_motor]->measure.total_angle-1.0)<6.0f)
            {

                gim_motor_ref[up_pitch_motor]=pitch_calc_motor_angle(0);
                if (fabs(gim_ins.pitch*DEGREE_2_RAD) <0.8f&&fabs(angle_normalize(gim_motor[yaw_motor]->measure.total_angle))<0.1f)//通过云台是否水平判断是否完成归中，然后再
                {
                        // gim_motor[up_pitch_motor]->set_mode(gim_motor[up_pitch_motor], DM_CMD_MOTOR_MODE);
                        gimbal_fdb_data.back_mode = BACK_IS_OK;
                        gimbal_fdb_data.yaw_offset_angle_total=yaw_filtered;
                        gimbal_fdb_data.pit_offset_angle=gim_ins.pitch;
                        gimbal_fdb_data.yaw_offset_angle=gim_ins.yaw;
                        gimbal_fdb_data.yaw_auto_offset_total_angle=yaw_filtered;
                }
                else
                {
                    gimbal_fdb_data.back_mode = BACK_STEP;
                }
            }
            else
            {
                gimbal_fdb_data.back_mode = BACK_STEP;
            }
            //三层if嵌套归中
            //第一层，将下pitch抬起到一半，防止其他电机归中时发生干涉，导致疯车
            //第二层将yaw轴归中和把上pitch电机归中
            //第三层如果imu显示枪管已经到达水平状态，就矫正上pitch电机，以便进行互补滤波
        }

            break;

        case GIMBAL_GYRO:
            // dm_motor_enable_all();
            gim_motor_ref[yaw_motor]=gim_cmd.yaw*DEGREE_2_RAD;
            gim_motor_ref[up_pitch_motor]=pitch_calc_motor_angle(gim_cmd.pitch);
            gimbal_fdb_data.yaw_relative_angle=get_yaw_motor_angle();
            // gimbal_fdb_data.yaw_relative_angle = angle_normalize(gim_motor[yaw_motor]->measure.total_angle);
            gimbal_fdb_data.yaw_offset_angle=gim_ins.yaw;
            gimbal_fdb_data.pit_offset_angle=gim_ins.pitch;
            gimbal_fdb_data.yaw_auto_offset_total_angle=yaw_filtered;
            break;
        case GIMBAL_AUTO:
            gim_motor_ref[yaw_motor]=gim_cmd.yaw*DEGREE_2_RAD;
            gim_motor_ref[up_pitch_motor]=pitch_calc_motor_angle(gim_cmd.pitch);
            gimbal_fdb_data.yaw_relative_angle=get_yaw_motor_angle();
            gimbal_fdb_data.pit_offset_angle=gim_ins.pitch;
            gimbal_fdb_data.yaw_offset_angle=gim_ins.yaw;
            // gimbal_fdb_data.back_mode = BACK_STEP;
            gimbal_fdb_data.yaw_offset_angle_total=yaw_filtered;
            break;
        case GIMBAL_NO_FOLLOW:

            break;
        case GIMBAL_RESET:
            // gim_motor[up_pitch_motor]->set_mode(gim_motor[up_pitch_motor], DM_CMD_ZERO_POSITION);
            // gim_motor[dowm_pitch_motor]->set_mode(gim_motor[dowm_pitch_motor], DM_CMD_ZERO_POSITION);
            // gim_motor[yaw_motor]->set_mode(gim_motor[yaw_motor], DM_CMD_ZERO_POSITION);
            // if ( gim_motor[up_pitch_motor]->ctrl_mode!=DM_CMD_ZERO_POSITION||gim_motor[dowm_pitch_motor]->ctrl_mode!=DM_CMD_ZERO_POSITION)
            // {
            //     // gim_motor[up_pitch_motor]->set_mode(gim_motor[up_pitch_motor], DM_CMD_ZERO_POSITION);
            //     gim_motor[dowm_pitch_motor]->set_mode(gim_motor[up_pitch_motor], DM_CMD_ZERO_POSITION);
            // }
            break;
        case GIMBAL_DOGHOLE:
            gim_motor_ref[up_pitch_motor]=pitch_calc_motor_angle(gim_cmd.pitch);
            gim_motor_ref[dowm_pitch_motor]=gim_cmd.down_pitch*DEGREE_2_RAD;
            gim_motor_ref[yaw_motor]=gim_cmd.yaw*DEGREE_2_RAD;
            break;
        default:
        {
            /* 默认情况下，不做特殊处理，仅保持当前状态 */
        }
            break;
    }
}
uint16_t pwmaaa=500;
int count_gimbal=0;
void gimbal_control_task(){

    gimbal_sub_pull();

    /* 云台模式状态机：根据 gim_cmd.ctrl_mode 进行归中/互补滤波/失能控制 */
    GimbalCtrl_StateHandler();


    /* 保留原有调试 CAN 发送 */
    // data[0]=0;
    // data[1]=(pwmaaa>>8)&0xff;
    // data[2]=pwmaaa&0xff;
    // // CAN_send(&hfdcan3,0x12,data);
    // if (count_gimbal%4==0)
    // {
    //     CAN_send(&hfdcan3,0x12,data);
    // }
    // count_gimbal++;
    gimbal_pub_push();

}


static void motor_enable()
{
    dm_motor_enable_all();  // 所有电机进入 motor 模式
}


/* 1 号电机 */
float target_speed1=1.0f;
float yaw_get_angle=0;
float yaw_get_angle_degree=0;
static dm_motor_para_t dm_yaw_control(dm_motor_measure_t measure)
{
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // 闭环反馈量
    static float pid_out_angle;         // 角度环输出
    static float send_data;        // 最终发送给电调的数据
    static float set_kd;
    switch (gim_cmd.ctrl_mode)
    {
        // TODO: 云台初始化模式加入斜坡算法，可以控制归中时间
        case GIMBAL_INIT:
            // pid_speed = gim_controller[yaw_motor].pid_speed_imu;
            pid_speed = gim_controller[yaw_motor].pid_speed_imu;
            pid_angle = gim_controller[yaw_motor].pid_angle_imu;
            get_speed = gim_motor[yaw_motor]->measure.speed_rads;
            // get_speed = gim_ins.gyro[2];
            // pid_speed = gim_controller[yaw_motor].pid_speed_imu;
            get_angle = angle_normalize(gim_motor[yaw_motor]->measure.total_angle);
            yaw_get_angle=get_angle;
            set_kd=0.01f;
            // send_data=0;
            break;
        case GIMBAL_GYRO:
            pid_speed = gim_controller[yaw_motor].pid_speed_imu;
            pid_angle = gim_controller[yaw_motor].pid_angle_imu;
            // get_speed = gim_motor[yaw_motor]->measure.speed_rads;
            get_speed = gim_ins.gyro[2];
            get_angle = (yaw_filtered - gimbal_fdb_data.yaw_offset_angle_total)*DEGREE_2_RAD;

            yaw_get_angle=get_angle;
            yaw_get_angle_degree=get_angle*RAD_2_DEGREE;
            set_kd=0.01f;
            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[yaw_motor].pid_speed_auto;
            pid_angle = gim_controller[yaw_motor].pid_angle_auto;
            // get_speed = gim_motor[yaw_motor]->measure.speed_rads;
            get_speed = gim_motor[yaw_motor]->measure.speed_rads;
            // get_speed = gim_ins.gyro[2];
            // pid_speed = gim_controller[yaw_motor].pid_speed_imu;
            get_angle = angle_normalize(gim_motor[yaw_motor]->measure.total_angle);
            // get_speed = gim_motor[yaw_motor]->measure.speed_rads;
            //
            // get_angle = angle_normalize(gim_motor[yaw_motor]->measure.total_angle);
            yaw_get_angle=get_angle;
            yaw_get_angle_degree=get_angle*RAD_2_DEGREE;
            set_kd=0.1f;
            break;
        case GIMBAL_DOGHOLE:
            pid_speed = gim_controller[yaw_motor].pid_speed_imu;
            pid_angle = gim_controller[yaw_motor].pid_angle_imu;
            // get_speed = gim_motor[yaw_motor]->measure.speed_rads;
            get_speed = gim_ins.gyro[2];
            get_angle = (yaw_filtered - gimbal_fdb_data.yaw_offset_angle_total)*DEGREE_2_RAD;

            yaw_get_angle=get_angle;
            yaw_get_angle_degree=get_angle*RAD_2_DEGREE;
            set_kd=0.01f;
            break;
        default:

            break;
    }
    /* 切换模式需要清空控制器历史状态 */
    if(gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
    }

    pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[yaw_motor]);
    // pid_out_angle = pid_calculate(pid_angle, get_angle, target_angle);
    send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);
    float t_ff=0.0f;
    send_data=send_data+t_ff;
    //云台yaw轴角度需要纠正


    static dm_motor_para_t set;

#ifdef CLOSE_DM_MOTOR
    send_data=0;
#endif
    // send_data=0;
    LIMIT_MIN_MAX(send_data, -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p =0;
        set.kp = 0;
        set.v = 0;
        set.kd = set_kd;
        set.t = send_data;
    }
    return set;
}
/* 2 号电机 */
float target_speed2=1.5;
float target_angle=1.5f;
static dm_motor_para_t dm_dn_pitch_control(dm_motor_measure_t measure)
{
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // 闭环反馈量
    static float pid_out_angle;         // 角度环输出
    static float send_data;        // 最终发送给电调的数据

    switch (gim_cmd.ctrl_mode)
    {
        // TODO: 云台初始化模式加入斜坡算法，可以控制归中时间
        case GIMBAL_INIT:
            pid_speed = gim_controller[dowm_pitch_motor].pid_speed_imu;
            pid_angle = gim_controller[dowm_pitch_motor].pid_angle_imu;
            get_speed = gim_motor[1]->measure.speed_rads;
            get_angle = gim_motor[1]->measure.total_angle;
            send_data=0;
            break;
        case GIMBAL_GYRO:
            pid_speed = gim_controller[dowm_pitch_motor].pid_speed_imu;
            pid_angle = gim_controller[dowm_pitch_motor].pid_angle_imu;
            get_speed = gim_motor[1]->measure.speed_rads;
            get_angle = gim_motor[1]->measure.total_angle;
            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[dowm_pitch_motor].pid_speed_auto;
            pid_angle = gim_controller[dowm_pitch_motor].pid_angle_auto;
            get_speed = gim_motor[1]->measure.speed_rads;
            get_angle = gim_motor[1]->measure.total_angle;
            break;

        case GIMBAL_DOGHOLE:
            pid_speed = gim_controller[dowm_pitch_motor].pid_speed_imu;
            pid_angle = gim_controller[dowm_pitch_motor].pid_angle_imu;
            get_speed = gim_motor[1]->measure.speed_rads;
            get_angle = gim_motor[1]->measure.total_angle;
            break;

        default:
            break;
    }
    /* 切换模式需要清空控制器历史状态 */
    if(gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
    }

    pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[dowm_pitch_motor]);
    // pid_out_angle = pid_calculate(pid_angle, get_angle, target_angle);
    send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);
    float t_ff=40.0f*arm_cos_f32(get_angle*DEGREE_2_RAD)*0.012f;
    send_data=send_data+t_ff;
    //云台yaw轴角度需要纠正


    static dm_motor_para_t set;

#ifdef CLOSE_DM_MOTOR
    send_data=0;
#endif
    send_data=0.0f;
    if(gim_cmd.ctrl_mode==GIMBAL_DOGHOLE)
    {
        // send_data=-0.8F;
    }
    LIMIT_MIN_MAX(send_data, -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p =0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0.1f;
        set.t = send_data;
    }
    return set;
}
/* 3 号电机 */

float target_speed3=0.0;
float target_angle3=1.5f;
static dm_motor_para_t dm_up_control(dm_motor_measure_t measure)
{
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // 闭环反馈量
    static float pid_out_angle;         // 角度环输出
    static float send_data;        // 最终发送给电调的数据

    switch (gim_cmd.ctrl_mode)
    {
        // TODO: 云台初始化模式加入斜坡算法，可以控制归中时间
        case GIMBAL_INIT:
            pid_speed = gim_controller[up_pitch_motor].pid_speed_imu;
            pid_angle = gim_controller[up_pitch_motor].pid_angle_imu;
            get_speed = gim_motor[up_pitch_motor]->measure.speed_rads;
            get_angle = gim_motor[up_pitch_motor]->measure.total_angle;
            send_data=0;
            break;
        case GIMBAL_GYRO:
            pid_speed = gim_controller[up_pitch_motor].pid_speed_imu;
            pid_angle = gim_controller[up_pitch_motor].pid_angle_imu;
            get_speed = gim_motor[up_pitch_motor]->measure.speed_rads;
            get_angle = gim_motor[up_pitch_motor]->measure.total_angle;
            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[up_pitch_motor].pid_speed_auto;
            pid_angle = gim_controller[up_pitch_motor].pid_angle_auto;
            get_speed = gim_motor[up_pitch_motor]->measure.speed_rads;
            get_angle = gim_motor[up_pitch_motor]->measure.total_angle;
            break;

        case GIMBAL_DOGHOLE:
            pid_speed = gim_controller[up_pitch_motor].pid_speed_imu;
            pid_angle = gim_controller[up_pitch_motor].pid_angle_imu;
            get_speed = gim_motor[up_pitch_motor]->measure.speed_rads;
            get_angle = gim_motor[up_pitch_motor]->measure.total_angle;
            break;
        default:
            pid_speed = gim_controller[up_pitch_motor].pid_speed_imu;
            pid_angle = gim_controller[up_pitch_motor].pid_angle_imu;
            get_speed = gim_motor[up_pitch_motor]->measure.speed_rads;
            get_angle = gim_motor[up_pitch_motor]->measure.total_angle;
            break;
    }
    /* 切换模式需要清空控制器历史状态 */
    if(gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
    }

    pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[up_pitch_motor]);
    // pid_out_angle = pid_calculate(pid_angle, get_angle, target_angle3);
    send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);
    float t_ff=30.0f*arm_cos_f32(pitch_filtered*DEGREE_2_RAD)*0.005f;
    // float t_ff=0.15f;
    send_data=send_data+t_ff;
    //云台yaw轴角度需要纠正


    static dm_motor_para_t set;

#ifdef CLOSE_DM_MOTOR
    send_data=0;
#endif
    // send_data=0;
    if(gim_cmd.ctrl_mode==GIMBAL_DOGHOLE)
    {
        send_data=-0.50F;
    }
    LIMIT_MIN_MAX(send_data, -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p =0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0.05f;
        set.t = send_data;
    }
    return set;
}

/* 底盘每个电机对应的控制函数 */
static void *dm_control[3] =
        {
                dm_yaw_control,
                dm_dn_pitch_control,
                dm_up_control,
        };
/* ------------------------------------------------ 云台控制相关 ----------------------------------------------------- */
/**
 * @brief 注册云台电机及其控制器初始化
 */
static void gimbal_motor_init()
{
/* ----------------------------------- yaw ---------------------------------- */
    for (int i=0; i<3; i++)
    {
        gim_motor[i]=dm_motor_register(&gimbal_motor_config[i],dm_control[i]);
    }
    // dm_motor_enable_all();  // 所有电机进入 motor 模式
    motor_enable();
}
static void gimbal_pid_init()
{
      pid_config_t yaw_speed_imu_config = INIT_PID_CONFIG(YAW_KP_V_IMU, YAW_KI_V_IMU, YAW_KD_V_IMU, YAW_INTEGRAL_V_IMU, YAW_MAX_V_IMU,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_angle_imu_config = INIT_PID_CONFIG(YAW_KP_A_IMU, YAW_KI_A_IMU, YAW_KD_A_IMU, YAW_INTEGRAL_A_IMU, YAW_MAX_A_IMU,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    // TODO: 自瞄模式参数待调
    pid_config_t yaw_speed_auto_config = INIT_PID_CONFIG(YAW_KP_V_AUTO, YAW_KI_V_AUTO, YAW_KD_V_AUTO, YAW_INTEGRAL_V_AUTO, YAW_MAX_V_AUTO,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_angle_auto_config = INIT_PID_CONFIG(YAW_KP_A_AUTO, YAW_KI_A_AUTO, YAW_KD_A_AUTO, YAW_INTEGRAL_A_AUTO, YAW_MAX_A_AUTO,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    gim_controller[yaw_motor].pid_speed_imu = pid_register(&yaw_speed_imu_config);
    gim_controller[yaw_motor].pid_angle_imu = pid_register(&yaw_angle_imu_config);
    gim_controller[yaw_motor].pid_speed_auto = pid_register(&yaw_speed_auto_config);
    gim_controller[yaw_motor].pid_angle_auto = pid_register(&yaw_angle_auto_config);

/* ---------------------------------- pitch --------------------------------- */
    pid_config_t up_pitch_speed_imu_config = INIT_PID_CONFIG(UP_PITCH_KP_V_IMU, UP_PITCH_KI_V_IMU, UP_PITCH_KD_V_IMU, UP_PITCH_INTEGRAL_V_IMU, UP_PITCH_MAX_V_IMU,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t up_pitch_angle_imu_config = INIT_PID_CONFIG(UP_PITCH_KP_A_IMU, UP_PITCH_KI_A_IMU, UP_PITCH_KD_A_IMU, UP_PITCH_INTEGRAL_A_IMU, UP_PITCH_MAX_A_IMU,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    // TODO: 自瞄模式参数待调
    pid_config_t up_pitch_speed_auto_config = INIT_PID_CONFIG(UP_PITCH_KP_V_AUTO, UP_PITCH_KI_V_AUTO, UP_PITCH_KD_V_AUTO, UP_PITCH_INTEGRAL_V_AUTO, UP_PITCH_MAX_V_AUTO,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t up_pitch_angle_auto_config = INIT_PID_CONFIG(UP_PITCH_KP_A_AUTO, UP_PITCH_KI_A_AUTO, UP_PITCH_KD_A_AUTO, UP_PITCH_INTEGRAL_A_AUTO, UP_PITCH_MAX_A_AUTO,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    gim_controller[up_pitch_motor].pid_speed_imu = pid_register(&up_pitch_speed_imu_config);
    gim_controller[up_pitch_motor].pid_angle_imu = pid_register(&up_pitch_angle_imu_config);
    gim_controller[up_pitch_motor].pid_speed_auto = pid_register(&up_pitch_speed_auto_config);
    gim_controller[up_pitch_motor].pid_angle_auto = pid_register(&up_pitch_angle_auto_config);

    pid_config_t dn_pitch_speed_imu_config = INIT_PID_CONFIG(DN_PITCH_KP_V_IMU, DN_PITCH_KI_V_IMU, DN_PITCH_KD_V_IMU, DN_PITCH_INTEGRAL_V_IMU, DN_PITCH_MAX_V_IMU,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t dn_pitch_angle_imu_config = INIT_PID_CONFIG(DN_PITCH_KP_A_IMU, DN_PITCH_KI_A_IMU, DN_PITCH_KD_A_IMU, DN_PITCH_INTEGRAL_A_IMU, DN_PITCH_MAX_A_IMU,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    // TODO: 自瞄模式参数待调
    pid_config_t dn_pitch_speed_auto_config = INIT_PID_CONFIG(DN_PITCH_KP_V_AUTO, DN_PITCH_KI_V_AUTO, DN_PITCH_KD_V_AUTO, DN_PITCH_INTEGRAL_V_AUTO, DN_PITCH_MAX_V_AUTO,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t dn_pitch_angle_auto_config = INIT_PID_CONFIG(DN_PITCH_KP_A_AUTO, DN_PITCH_KI_A_AUTO, DN_PITCH_KD_A_AUTO, DN_PITCH_INTEGRAL_A_AUTO, DN_PITCH_MAX_A_AUTO,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    gim_controller[dowm_pitch_motor].pid_speed_imu = pid_register(&dn_pitch_speed_imu_config);
    gim_controller[dowm_pitch_motor].pid_angle_imu = pid_register(&dn_pitch_angle_imu_config);
    gim_controller[dowm_pitch_motor].pid_speed_auto = pid_register(&dn_pitch_speed_auto_config);
    gim_controller[dowm_pitch_motor].pid_angle_auto = pid_register(&dn_pitch_angle_auto_config);



}

static void gimbal_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(gimbal_fdb_topic), &gimbal_fdb_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void gimbal_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
    gimbal_cmd_node = mcn_subscribe(MCN_HUB(gimbal_cmd), NULL, NULL);
    gimbal_ins_node = mcn_subscribe(MCN_HUB(gimbal_ins_topic), NULL, NULL);
}


/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void gimbal_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }

    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chass_cmd);
    }

    if (mcn_poll(gimbal_cmd_node))
    {
        mcn_copy(MCN_HUB(gimbal_cmd), gimbal_cmd_node, &gim_cmd);
    }

    if (mcn_poll(gimbal_ins_node))
    {
        mcn_copy(MCN_HUB(gimbal_ins_topic), gimbal_ins_node, &gim_ins);
    }
}
//规定向左转，为正方向
//仅使用弧度制，计算sin cos时也是使用弧度制，
//云台的所有角度均使用弧度制
//将云台的关节电机角度范围设置成2PI，不管是反馈得到的角度是多少，都可以正常进行归中
float angle_normalize(float angle_deg)
{
    if (angle_deg > PI)
    {
        angle_deg -= PI*2.000f;
    }

    else if (angle_deg < -PI)
    {
        angle_deg += PI*2.000f;
    }
    return angle_deg;
}

/**
 *
 * @param ref_angle pitch期望角度
 * @param ctrl_angle 发送给电机的控制角度
 * @brief imu pitch以抬头为正方向， yaw向左转为正方向
 * @return 返回弧度制
 */
////TODO互补滤波完成之后，将IMU替换掉，换成滤波之后的东西
////TODO 之后做好软件限位，不要开机的时候，角度超限
///TODO

float pitch_calc_motor_angle(float ref_angle)
{
    if (fabs(last_pitch-gim_ins.pitch)>0.0010f)
    // if (last_pitch!=gim_ins.pitch)
    {

        pitch_filtered = 0.05f * pitch_filtered + 0.95f * gim_ins.pitch;
        last_pitch=gim_ins.pitch;
        // ref_angle = angle_normalize(ref_angle*DEGREE_2_RAD);
        ref_angle = ref_angle*DEGREE_2_RAD;
        float error=ref_angle-pitch_filtered*DEGREE_2_RAD;
        float ctrl_angle=gim_motor[up_pitch_motor]->measure.total_angle-error/UP_PITCH_REDUCTION;
        lastoutput=ctrl_angle;
        return ctrl_angle;
    }
    else
    {
        return lastoutput;
    }
}


float yaw_filter_angle()
{
    yaw_filtered=0.01f * yaw_filtered + 0.99f * gim_ins.yaw_total_angle;
    return yaw_filtered;
}
/**
 *
 * @return 返回单纯由关节电机得到的pitch轴角度
 * @brief 在事先校准，使枪管水平的时候，关节电机位置为0，这样能够正常反馈得到由关节电机反馈得到的pitch角度
 */
float get_pitch_form_motor()
{
    float pitch_angle=gim_motor[up_pitch_motor]->measure.total_angle*UP_PITCH_REDUCTION;
    return pitch_angle;
}
