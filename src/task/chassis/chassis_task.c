//
// Created by Gleam on 25-8-22.
//

#include "chassis_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "robot.h"
#include "DM_motor.h"


/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
MCN_DECLARE(chassis_fdb);
// static struct ins_msg ins;
MCN_DECLARE(chassis_motor_trans_topic);
static McnNode_t chassis_motor_trans_node ;
// static struct chassis_cmd_msg chassis_cmd;
// 发布

// static struct chassis_fdb_msg chassis_fdb_data;
static struct chassis_cmd_msg chassis_cmd;
static struct chassis_fdb_msg chassis_fdb;
static struct chassis_motor_msg chassis_motor_msg;
static struct ins_msg ins_data;



static void chassis_pub_init(void);
static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);
extern struct referee_fdb_msg referee_fdb;
/* ------------------------------------------------- 电机控制相关 ------------------------------------------------------ */
static pid_obj_t *follow_pid; // 用于底盘跟随云台计算vw
static pid_config_t chassis_follow_config = INIT_PID_CONFIG(CHASSIS_KP_V_FOLLOW, CHASSIS_KI_V_FOLLOW, CHASSIS_KD_V_FOLLOW, CHASSIS_INTEGRAL_V_FOLLOW, CHASSIS_MAX_V_FOLLOW,
                                                            (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

static struct chassis_controller_t
{
    pid_obj_t *speed_pid;
}chassis_controller[4];

static dji_motor_object_t *chassis_motor[4];  // 底盘电机实例
static int16_t motor_ref[4]; // 电机控制期望值
/* -------------------------------------------------- 里程计相关 ------------------------------------------------------ */
/*定时器初始化*/
static int TIM_Init(void);
/*里程计所需数据*/
static float x_ch, y_ch, w_ch, x_gim, y_gim,vw_ch,vy_ch,vx_ch,vx_gim,vy_gim,x_sin_w,x_cos_w,y_sin_w,y_cos_w;
static struct chassis_real_speed_t
{
    float chassis_vx_ch;
    float chassis_vy_ch;
    float chassis_vw_ch;
    float chassis_vx_gim;
    float chassis_vy_gim;

}chassis_real_speed;


/* ------------------------------------------------ 底盘运动学解算 ----------------------------------------------------- */
/* 根据宏定义选择的底盘类型使能对应的解算函数 */
#ifdef BSP_CHASSIS_OMNI_MODE
static void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
static void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = mecanum_calc;
static struct chassis_real_speed_t omni_get_speed(dji_motor_object_t *chassis_motor[4]);
#endif /* BSP_CHASSIS_OMNI_MODE */
#ifdef BSP_CHASSIS_MECANUM_MODE
static void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
static void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = mecanum_calc;
#endif /* BSP_CHASSIS_MECANUM_MODE */


/* ---------------------------------------------- 其余变量与函数声明 ---------------------------------------------------- */
float follow_err,vw;
 void chassis_motor_init();
static void absolute_cal(struct chassis_cmd_msg *cmd, float angle);
static float chassis_dt;
// dji_motor_object_t *return_chassis_motor()
// {
//     return chassis_motor;
// }
void chassis_task_init()
{
    chassis_pub_init();
    chassis_sub_init();
    chassis_motor_init();
}
int chass_a=0;
void chassis_control_task(void)
{
            chassis_sub_pull();
            /* 计算实际速度 */
            //omni_get_speed(chassis_motor);
            /* 更新该线程所有的订阅者 */
            // chassis_sub_pull();

            for (uint8_t i = 0; i < 4; i++) {
                dji_motor_enable(chassis_motor[i]);
            }

            switch (chassis_cmd.ctrl_mode) {
                case CHASSIS_RELAX:
                    for (uint8_t i = 0; i < 4; i++) {
                        dji_motor_relax(chassis_motor[i]);
                    }
                    break;
                case CHASSIS_FOLLOW_GIMBAL:
                    follow_err = chassis_cmd.offset_angle;
                    if (follow_err < 0.01f && follow_err >= 0) {
                        // chassis_cmd.offset_angle = follow_err * follow_err / 5;
                        chassis_cmd.offset_angle=0;
                    } else if (follow_err < 0 && follow_err > -0.01f) {
                        // chassis_cmd.offset_angle = -follow_err * follow_err / 5;
                        chassis_cmd.offset_angle=0;
                    }

                    vw = pid_calculate(follow_pid, chassis_cmd.offset_angle, SIDEWAYS_ANGLE);
                    chassis_cmd.vw = vw;

                    /* 底盘运动学解算 */
                    absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
                    chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                    break;
                case CHASSIS_SPIN:
                    if (chassis_fdb.spin_flag==0)
                    {
                        chassis_fdb.spin_yaw_offset=ins_data.yaw;
                        chassis_fdb.spin_flag=1;
                    }
                    absolute_cal(&chassis_cmd, ins_data.yaw-chassis_fdb.spin_yaw_offset);
                    chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                    break;
                case CHASSIS_OPEN_LOOP:
                    chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                    break;
                case CHASSIS_STOP:
                    memset(motor_ref, 0, sizeof(motor_ref));
                    break;
                case CHASSIS_FLY:
                    break;
                case CHASSIS_AUTO:
                    break;
                case CHASSIS_NO_GIMBAL:
                    // absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
                    /* 底盘不跟随云台模式：底盘速度直接受chassis_cmd控制，不进行坐标转换 */
                    chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                case CHASSIS_RETURN:
                    if (fabs(ins_data.yaw-chassis_fdb.spin_yaw_offset) < 0.5)
                    {
                        chassis_fdb.spin_flag=0;
                    }
                    vw = -pid_calculate(follow_pid, ins_data.yaw-chassis_fdb.spin_yaw_offset, SIDEWAYS_ANGLE);
                    // // chassis_cmd.vw = vw;
                    //
                    // /* 底盘运动学解算 */
                    // absolute_cal(&chassis_cmd, ins_data.yaw-chassis_fdb.spin_yaw_offset);
                    chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                    break;
                default:
                    for (uint8_t i = 0; i < 4; i++) {
                        dji_motor_relax(chassis_motor[i]);
                    }
                    break;
            }

            /* 更新发布该线程的msg */
            chassis_pub_push();
}
/* ------------------------------------------------- 底盘线程入口 ------------------------------------------------------ */
void ChassisTask_Entry(void const * argument)
{
    static float chassis_start;

    chassis_pub_init();
    chassis_sub_init();
    chassis_motor_init();
    // bsp_can_init();
    // can_filter_init();

    for(;;)
    {
        /* 更新该线程所有的订阅者 */
        chassis_sub_pull();

        for (;;) {
            chassis_start = dwt_get_time_ms();
            /* 计算实际速度 */
            //omni_get_speed(chassis_motor);
            /* 更新该线程所有的订阅者 */
            chassis_sub_pull();

            for (uint8_t i = 0; i < 4; i++) {
                dji_motor_enable(chassis_motor[i]);
            }

            switch (chassis_cmd.ctrl_mode) {
                case CHASSIS_RELAX:
                    for (uint8_t i = 0; i < 4; i++) {
                        dji_motor_relax(chassis_motor[i]);
                    }
                    break;
                case CHASSIS_FOLLOW_GIMBAL:
                    follow_err = chassis_cmd.offset_angle;
                    if (follow_err < 5 && follow_err >= 0) {
                        chassis_cmd.offset_angle = follow_err * follow_err / 5;
                    } else if (follow_err < 0 && follow_err > -5) {
                        chassis_cmd.offset_angle = -follow_err * follow_err / 5;
                    }

                    vw = -pid_calculate(follow_pid, chassis_cmd.offset_angle, SIDEWAYS_ANGLE);
                    chassis_cmd.vw = vw;

                    /* 底盘运动学解算 */
                    absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
                    chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                    break;
                case CHASSIS_SPIN:
                    if (chassis_fdb.spin_flag==0)
                    {
                        chassis_fdb.spin_yaw_offset=ins_data.yaw;
                        chassis_fdb.spin_flag=1;
                    }
                    absolute_cal(&chassis_cmd, ins_data.yaw-chassis_fdb.spin_yaw_offset);
                    chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                    break;
                case CHASSIS_OPEN_LOOP:
                    chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                    break;
                case CHASSIS_STOP:
                    memset(motor_ref, 0, sizeof(motor_ref));
                    break;
                case CHASSIS_FLY:
                    break;
                case CHASSIS_AUTO:
                    break;
                case CHASSIS_NO_GIMBAL:
                    // absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
                    /* 底盘不跟随云台模式：底盘速度直接受chassis_cmd控制，不进行坐标转换 */
                    chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                case CHASSIS_RETURN:
                    if (fabs(ins_data.yaw-chassis_fdb.spin_yaw_offset) < 0.5)
                    {
                        chassis_fdb.spin_flag=0;
                    }
                    vw = -pid_calculate(follow_pid, ins_data.yaw-chassis_fdb.spin_yaw_offset, SIDEWAYS_ANGLE);
                    chassis_cmd.vw = vw;
                    //文总是全栈工程师
                    /* 底盘运动学解算 */
                    absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
                    chassis_calc_moto_speed(&chassis_cmd, motor_ref);
                    break;
                default:
                    for (uint8_t i = 0; i < 4; i++) {
                        dji_motor_relax(chassis_motor[i]);
                    }
                    break;
            }

            /* 更新发布该线程的msg */
            chassis_pub_push();
            /* 用于调试监测线程调度使用 */
            chassis_dt = dwt_get_time_ms() - chassis_start;
            vTaskDelay(1);
        }
    }
    /* USER CODE END ChassisTask_Entry */
}
/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
/**
 * @brief chassis 线程中所有发布者初始化
 */
static void chassis_pub_init(void)
{
    // pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
}

/**
 * @brief chassis 线程中所有订阅者初始化
 */
static void chassis_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void chassis_pub_push(void)
{
    mcn_publish(MCN_HUB(chassis_fdb), &chassis_fdb);
    for (int i=0;i<4;i++)
    {
        dji_motor_measure_t measure=chassis_motor[i]->measure;
        chassis_motor_msg.motor_tran_msg[i].motor_id=i;
        chassis_motor_msg.motor_tran_msg[i].ecd=measure.ecd;
        chassis_motor_msg.motor_tran_msg[i].real_current=measure.real_current;
        chassis_motor_msg.motor_tran_msg[i].speed_rpm=measure.speed_rpm;
        chassis_motor_msg.motor_tran_msg[i].temperature=measure.temperature;
        chassis_motor_msg.motor_tran_msg[i].total_angle=measure.total_angle;
    }
    mcn_publish(MCN_HUB(chassis_motor_trans_topic), &chassis_motor_msg);
}

/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void chassis_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins_data);
    }

    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chassis_cmd);
    }
}



/* ----------------------------------------------- 电机控制相关 ------------------------------------------------------- */

static int16_t motor_control_0(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set = pid_calculate(chassis_controller[0].speed_pid, measure.speed_rpm, motor_ref[0]);
    return set;
}

static int16_t motor_control_1(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set = pid_calculate(chassis_controller[1].speed_pid, measure.speed_rpm, motor_ref[1]);
    return set;
}

static int16_t motor_control_2(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set = pid_calculate(chassis_controller[2].speed_pid, measure.speed_rpm, motor_ref[2]);
    return set;
}

static int16_t motor_control_3(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set = pid_calculate(chassis_controller[3].speed_pid, measure.speed_rpm, motor_ref[3]);
    return set;
}


/* 底盘每个电机对应的控制函数 */
static void *motor_control[4] =
        {
                motor_control_0,
                motor_control_1,
                motor_control_2,
                motor_control_3,
        };

// TODO：将参数都放到配置文件中，通过宏定义进行替换
motor_config_t chassis_motor_config[4] =
        {
                {
                        .motor_type = M3508,
                        .can_id = CAN_ID_CHASSIS_MOTOR,
                        .rx_id = 0x201,
                        .tx_id = 0x201,
                        .controller = &chassis_controller[0],
                },
                {
                        .motor_type = M3508,
                        .can_id = CAN_ID_CHASSIS_MOTOR,
                        .rx_id = 0x202,
                        .tx_id = 0x202,
                        .controller = &chassis_controller[1],
                },
                {
                        .motor_type = M3508,
                        .can_id = CAN_ID_CHASSIS_MOTOR,
                        .rx_id = 0x203,
                        .tx_id = 0x203,
                        .controller = &chassis_controller[2],
                },
                {
                        .motor_type = M3508,
                        .can_id =CAN_ID_CHASSIS_MOTOR,
                        .rx_id = 0x204,
                        .tx_id = 0x204,
                        .controller = &chassis_controller[3],
                }
        };



/* ------------------------------------------------ 底盘运动学解算 ----------------------------------------------------- */
static void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * CHASSIS_DECELE_RATIO);

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //rad/s

    wheel_rpm[0] = ( cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//left//x，y方向速度,w底盘转动速度
    wheel_rpm[1] = ( cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//forward
    wheel_rpm[2] = (-cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//right
    wheel_rpm[3] = (-cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//back
    chassis_fdb.vw_ch = (chassis_motor[0]->measure.speed_rpm + chassis_motor[1]->measure.speed_rpm
                         + chassis_motor[2]->measure.speed_rpm  + chassis_motor[3]->measure.speed_rpm )
                        / (4.0f * wheel_rpm_ratio * LENGTH_RADIUS);

    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));//copy the rpm to out_speed
}


static struct chassis_real_speed_t omni_get_speed(dji_motor_object_t *chassis_motor[4])//里程计计算函数。
{

    float angle_hd = -(chassis_cmd.offset_angle/RADIAN_COEF);
    float wheel_rpm_ratio = 60.0f/(WHEEL_PERIMETER*CHASSIS_DECELE_RATIO);

    int16_t wheel_rpm[4];
    for (int i = 0; i < 4; ++i)
    {
        wheel_rpm[i]=chassis_motor[i]->measure.speed_rpm;
    }
    struct chassis_real_speed_t real_speed;
    vw_ch =real_speed.chassis_vw_ch = (wheel_rpm[0] + wheel_rpm[1] + wheel_rpm[2] + wheel_rpm[3]) / (4.0f * wheel_rpm_ratio * LENGTH_RADIUS);
    vy_ch =real_speed.chassis_vy_ch = (wheel_rpm[0] + wheel_rpm[3] - wheel_rpm[1] - wheel_rpm[2]) / (4.0f * 0.7071f * wheel_rpm_ratio);
    vx_ch =real_speed.chassis_vx_ch = (wheel_rpm[0] + wheel_rpm[1] - wheel_rpm[2] - wheel_rpm[3]) / (4.0f * 0.7071f * wheel_rpm_ratio) ;
    vx_gim =real_speed.chassis_vx_gim =  real_speed.chassis_vx_ch* cos(angle_hd) - real_speed.chassis_vy_ch* sin(angle_hd);
    vy_gim =real_speed.chassis_vy_gim =  real_speed.chassis_vx_ch* sin(angle_hd) + real_speed.chassis_vy_ch* cos(angle_hd);

    return real_speed;
}

/* ---------------------------------------------- 其余变量与函数声明 ---------------------------------------------------- */
/**
* @brief 注册底盘电机及其控制器初始化
*/
int motoridx=0;
 void chassis_motor_init()
{
    pid_config_t chassis_speed_config = INIT_PID_CONFIG(CHASSIS_KP_V_MOTOR, CHASSIS_KI_V_MOTOR, CHASSIS_KD_V_MOTOR, CHASSIS_INTEGRAL_V_MOTOR, CHASSIS_MAX_V_MOTOR,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_controller[i].speed_pid = pid_register(&chassis_speed_config);
        chassis_motor[i] = dji_motor_register(&chassis_motor_config[i], motor_control[i]);

    }

    follow_pid = pid_register(&chassis_follow_config);
}

/**
  * @brief  将云台坐标转换为底盘坐标
  * @param  cmd 底盘指令值，使用其中的速度
  * @param  angle 云台相对于底盘的角度
  */
static void  absolute_cal(struct chassis_cmd_msg *cmd, float angle)
{
    float angle_hd = -(angle/RADIAN_COEF);
    float vx = cmd->vx;
    float vy = cmd->vy;

    //保证底盘是相对摄像头做移动
    cmd->vx = vx * cos(angle_hd) - vy * sin(angle_hd);
    cmd->vy = vx * sin(angle_hd) + vy * cos(angle_hd);
}