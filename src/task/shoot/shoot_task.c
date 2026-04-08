//
// Created by Gleam on 25-8-22.
//

#include "rm_config.h"
#include "motor_def.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "shoot_task.h"

//TODO: ???????????????cmd????о???

/* ----------------------------------------------- ????????????? --------------------------------------------------- */
// ????
MCN_DECLARE(shoot_fdb_topic);
static struct shoot_fdb_msg shoot_fdb_data;

// ????
MCN_DECLARE(shoot_cmd);
static McnNode_t shoot_cmd_node;
static struct shoot_cmd_msg fire_cmd;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chass_cmd;

static void shoot_pub_push(void);
static void shoot_sub_init(void);
static void shoot_sub_pull(void);

/* ------------------------------------------------- ?????????? ----------------------------------------------------- */
/*????????????????*/
//#define SHT_MOTOR_NUM 3
#define SHT_MOTOR_NUM 1

/*?????????????????????????? ????????? ???????*/
#define RIGHT_FRICTION 0
#define LEFT_FRICTION 1
#define TRIGGER_MOTOR 0

/*pid????????*/
static struct shoot_controller_t{
    pid_obj_t *pid_speed;
    pid_obj_t *pid_angle;
}sht_controller[SHT_MOTOR_NUM];

/*??????????????*/
motor_config_t shoot_motor_config[SHT_MOTOR_NUM] ={
        {
                .motor_type = M2006,
                .can_id = CAN_ID_CHASSIS_MOTOR,
                .rx_id = TRIGGER_MOTOR_ID,
                .controller = &sht_controller[RIGHT_FRICTION],
        },

};

static dji_motor_object_t *sht_motor[SHT_MOTOR_NUM];  // ????????????
static float shoot_motor_ref[SHT_MOTOR_NUM]; // ????????????
/* ------------------------------------------------- ?????????? ----------------------------------------------------- */
//???????λ??????л?????????????????
static int total_angle_flag=SHOOT_ANGLE_CONTINUE;
/*????????*/
static void shoot_motor_init();
static int shoot_one_flag=1;
static int16_t motor_control_trigger(dji_motor_measure_t measure);

//??????????????
static int reverse_ref;

/* ----------------------------------------------- ????????? --------------------------------------------------- */


static float sht_dt;
static int flag;
static float sht_start;
static int servo_cvt_num;
static float sht_gap_time;
static float sht_gap_start_time;
static int reverse_cnt=0;
void shoot_task_init(){
    shoot_sub_init();
    shoot_motor_init();
    /*----------------------??????????----------------------------------*/
    fire_cmd.ctrl_mode=SHOOT_STOP;
    fire_cmd.trigger_status=TRIGGER_OFF;
    shoot_motor_ref[TRIGGER_MOTOR]=0;
}

uint8_t tx_data[8];
uint16_t friction_speed;
static int count_shoot=0;
void shoot_control(void)
{

    sht_start = dwt_get_time_ms();
    /* ???????????е?????? */
    shoot_sub_pull();
    /* ??????????? */


    /* ??????????? */
    for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
    {
        dji_motor_enable(sht_motor[i]);
    }



    shoot_fdb_data.trigger_motor_current=sht_motor[TRIGGER_MOTOR]->measure.real_current;
    if (fire_cmd.friction_on_flag==1)
    {
        tx_data[0]=1;
        friction_speed=600;
        tx_data[1]=(friction_speed>>8)& 0xff;
        tx_data[2]=friction_speed & 0xff;
    }
    else
    {
        tx_data[0]=1;
        friction_speed=10;
        tx_data[1]=(friction_speed>>8)& 0xff;
        tx_data[2]=friction_speed & 0xff;
    }
    switch (fire_cmd.ctrl_mode)
    {

        case SHOOT_STOP:
            shoot_motor_ref[TRIGGER_MOTOR] = sht_motor[TRIGGER_MOTOR]->measure.total_angle;
            total_angle_flag=0;
            shoot_fdb_data.trigger_status=SHOOT_WAITING;

            break;

        case SHOOT_ONE:

            if (fire_cmd.trigger_status == TRIGGER_ON)
            {
                //
                if (shoot_one_flag == 1)
                {
                    shoot_motor_ref[TRIGGER_MOTOR] = sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                    shoot_motor_ref[TRIGGER_MOTOR] = sht_motor[TRIGGER_MOTOR]->measure.total_angle + 360 / 9 * 2.5 * 36;
                    shoot_one_flag = 0;
                    // shoot_fdb_data.trigger_status=SHOOT_WAITING;
                }
                if ((fabs(sht_motor[TRIGGER_MOTOR]->measure.total_angle - shoot_motor_ref[TRIGGER_MOTOR]) < 5.0f)&&shoot_one_flag ==0)
                {
                    shoot_one_flag = 1;
                    // shoot_fdb_data.trigger_status=SHOOT_OK;
                }
            }


            break;

        case SHOOT_THREE:
            // /*????????????л?????????????????????????????*/
            if(total_angle_flag == SHOOT_ANGLE_CONTINUE)
            {
                shoot_motor_ref[TRIGGER_MOTOR]= sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                total_angle_flag = SHOOT_ANGLE_SINGLE;
            }
            if (fire_cmd.trigger_status == TRIGGER_ON)
            {

                shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] + TRIGGER_MOTOR_51_TO_ANGLE * 19;//M3508??????? 19:1???????????51.43????????????????19??
                fire_cmd.trigger_status=TRIGGER_OFF;//???????
                shoot_fdb_data.trigger_status=SHOOT_OK;
            }

            break;

        case SHOOT_COUNTINUE:
            // tx_data[0]=1;
            if (fire_cmd.trigger_status == TRIGGER_ING)
            {
                shoot_motor_ref[TRIGGER_MOTOR] = fire_cmd.shoot_freq*36*2.5/9*60;//???????????????????????????
                total_angle_flag = SHOOT_ANGLE_CONTINUE;
                // tx_data[0]=1;

                // CAN_send(&hfdcan3,0x110,tx_data);
                // tx_data[3]=(friction_speed >> 8) & 0xff;
                // shoot_fdb_data.trigger_status= SHOOT_OK;
            }
            if (fire_cmd.trigger_status == TRIGGER_OFF)
            {
                shoot_motor_ref[TRIGGER_MOTOR]=0;
            }
            break;

        case SHOOT_REVERSE:
            shoot_motor_ref[TRIGGER_MOTOR]=-4000;
            // shoot_motor_ref[TRIGGER_MOTOR]=4000;

            break;

        default:
            for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
            {
                dji_motor_relax(sht_motor[i]); // ????????????????
            }
            shoot_fdb_data.trigger_status=SHOOT_ERR;
            break;
    }

    if (count_shoot%2==0)
    {
        CAN_send(&hfdcan3,0x12,tx_data);
    }
    count_shoot++;
    if (count_shoot==1000)
    {
        count_shoot=0;
    }
    shoot_pub_push();


    // osDelay(1);


}
/**
 * @brief shoot?????????
 */
void shoot_control_task(){
    shoot_sub_pull();//???????
    shoot_control();
    shoot_pub_push();//????????
};

/**
 * @brief shoot ??????????
 */
static void shoot_motor_init(){
    /* -------------------------------------- right_friction ????????? ----------------------------------------- */

/* ------------------------------------------------  ???????------------------------------------------------------------------------- */
    pid_config_t toggle_speed_config = INIT_PID_CONFIG(TRIGGER_KP_V  , TRIGGER_KI_V , TRIGGER_KD_V  , TRIGGER_INTEGRAL_V, TRIGGER_MAX_V ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t toggle_angle_config = INIT_PID_CONFIG(TRIGGER_KP_A, TRIGGER_KI_A, TRIGGER_KD_A, TRIGGER_INTEGRAL_A , TRIGGER_MAX_A ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[TRIGGER_MOTOR].pid_speed = pid_register(&toggle_speed_config);
    sht_controller[TRIGGER_MOTOR].pid_angle = pid_register(&toggle_angle_config);

/* ---------------------------------- shoot????????---------------------------------------------------------------------------------------- */
    sht_motor[TRIGGER_MOTOR] = dji_motor_register(&shoot_motor_config[TRIGGER_MOTOR], motor_control_trigger);

}




/*?????????????*/
static int16_t motor_control_trigger(dji_motor_measure_t measure)
{
    /* PID???????л????????PID?????? */
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // ?????????
    static float pid_out_angle;         // ???????
    static int16_t send_data;        // ?????????????????

    /*??????????????pid??????????????????*/
    pid_speed = sht_controller[TRIGGER_MOTOR].pid_speed;
    pid_angle = sht_controller[TRIGGER_MOTOR].pid_angle;
    get_angle=measure.total_angle;
    get_speed=measure.speed_rpm;

    /* ?л??????????????????? */
    if(fire_cmd.ctrl_mode != fire_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
    }
    if (fire_cmd.ctrl_mode==SHOOT_ONE||fire_cmd.ctrl_mode==SHOOT_THREE) //非连发模式的时候，用双环pid控制拨弹电机
    {
        pid_out_angle = (int16_t) pid_calculate(pid_angle, get_angle, shoot_motor_ref[TRIGGER_MOTOR]);  // 编码器增长方向与imu相反
        send_data = (int16_t) pid_calculate(pid_speed, get_speed, pid_out_angle);     // 电机转动正方向与imu相反
    }
    /*pid计算输出*/
    else if(fire_cmd.ctrl_mode==SHOOT_COUNTINUE||fire_cmd.ctrl_mode==SHOOT_REVERSE)//自动模式的时候，只用速度环控制拨弹电机
    {
        send_data = (int16_t) pid_calculate(pid_speed, get_speed, shoot_motor_ref[TRIGGER_MOTOR] );
    }
    if (fire_cmd.ctrl_mode==SHOOT_STOP)
    {
        send_data=0;
    }
        return send_data;


}


/******************************************************???????*************************************************************************/
static void shoot_pub_push(void){
    mcn_publish(MCN_HUB(shoot_fdb_topic), &shoot_fdb_data);
}
static void shoot_sub_init(void){
    shoot_cmd_node = mcn_subscribe(MCN_HUB(shoot_cmd), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);

}
static void shoot_sub_pull(void){
    if (mcn_poll(shoot_cmd_node))
    {
        mcn_copy(MCN_HUB(shoot_cmd), shoot_cmd_node, &fire_cmd);
    }

    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chass_cmd);
    }

}