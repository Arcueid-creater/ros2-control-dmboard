#ifndef ROBOT_H
#define ROBOT_H

#include "dji_motor.h"
#include "stdbool.h"
#include "chassis_task.h"



/**
 * @brief 机器人初始化,请在开启rtos之前调用
 * 
 */
void robot_init();

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 * 
 */
void robot_task();

/* ------------------------------- ipc uMCN 相关 ------------------------------ */
struct ins_msg
{
    // IMU量测值
    float gyro[3];  // 角速度,°/s
    float accel[3]; // 加速度
    float motion_accel_b[3]; // 机体坐标加速度
    float motion_accel_n[3]; // 绝对系加速度
    // 位姿
    float roll;  /*yaw,pitch,roll都为°*/
    float pitch;
    float yaw;
    float roll_gyro;
    float pitch_gyro;
    float yaw_gyro;
    float yaw_total_angle;
};

/**
  * @brief     云台回中状态枚举
  */
typedef enum
{
    BACK_STEP = 0,             //云台正在回中
    BACK_IS_OK = 1,            //云台回中完毕
} gimbal_back_e;

/**
 * @brief 云台真实反馈状态数据,由gimbal发布
 */
struct gimbal_fdb_msg
{
    gimbal_back_e back_mode;  // 云台归中情况

    float yaw_offset_angle_total;    //云台初始 yaw 轴角度 （由imu得）
    float yaw_offset_angle;    //云台初始 yaw 轴角度 （由imu得）
    float pit_offset_angle;    //云台初始 pit 轴角度 （由imu得）
    float yaw_relative_angle;  //云台相对于初始位置的yaw轴角度
    float yaw_auto_offset_total_angle;
};

/**
 * @brief 云台模式
 */
typedef enum
{
    GIMBAL_RELAX = 0,        //云台断电
    GIMBAL_INIT = 1,         //云台初始化
    GIMBAL_GYRO = 2,         //云台跟随imu闭环
    GIMBAL_AUTO = 3 ,         //云台自瞄
    GIMBAL_NO_FOLLOW=4,
    GIMBAL_LIFTER=5,
    GIMBAL_RESET=6,
    GIMBAL_DOGHOLE=7,
} gimbal_mode_e;

struct gimbal_cmd_msg
{ // 云台期望角度控制
    float yaw;
    float pitch;
    gimbal_mode_e ctrl_mode;  // 当前云台控制模式
    gimbal_mode_e last_mode;  // 上一次云台控制模式
    float gimbal_height;
    float gimbal_angle;
    float down_pitch;
};

typedef struct
{
    float height;
    float dheight;
}height_ref_t;
//// TODO：后续优化启用，目前时间紧急，使用extern
//struct referee_msg
//{
//    robot_status_t robot_status;
//    ext_power_heat_data_t power_heat_data_t;
//};
typedef struct
{
    uint8_t motor_id;
    int16_t speed_rpm;
    float total_angle;
    uint16_t ecd;
    int16_t real_current;
    uint8_t temperature;
}chassis_motor_trans;

struct chassis_motor_msg
{
    chassis_motor_trans motor_tran_msg[4];
};

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief cmd发布的底盘控制数据,由chassis订阅
 */
struct chassis_cmd_msg
{
    float vx;                  // 前进方向速度
    float vy;                  // 横移方向速度
    float vw;                  // 旋转速度
    float offset_angle;        // 底盘和归中位置的夹角
    chassis_mode_e ctrl_mode;  // 当前底盘控制模式
    chassis_mode_e last_mode;  // 上一次底盘控制模式

};
/* ------------------------------ chassis反馈状态数据 ------------------------------ */
/**
 * @brief 底盘真实反馈状态数据,由chassis发布
 */
struct chassis_fdb_msg
{
    float x_pos_gim;
    float y_pos_gim;
    float spin_flag;
    float spin_yaw_offset;
    float vw_ch;  // 底盘旋转速度
};
typedef enum
{
    LIFTER_BACK_STEP = 0,
    LIFTER_BACK_IS_OK  = 1,
}lifter_back_e;
/**
 * @brief 升降真实反馈状态数据,由lifter发布
 */
struct lifter_fdb_msg
{
    float real_height;
    float real_dheight;
    float real_angle;
    lifter_back_e back_mode;
};
/**
 * @brief cmd发布的底盘控制数据,由lifter订阅
 */
typedef enum
{
    LIFTER_RELAX ,//失能
    // LIFTER_STOP,
    // LIFTER_OPEN_LOOP,
    // LIFTER_FOLLOW_GIMBAL
    LIFTER_SPIN,//小陀螺
    LIFTER_FLY,//飞坡
    LIFTER_HEIGHT_CHANGE,//底盘高度变化
    LIFTER_HEIGHT_KEEP, //底盘高度保持不变
    LIFTER_HEIGHT_INIT,//归中模式的底盘高度作为最适合的运动模式
    LIFTER_CLIMB,
    LIFTER_BACK_UP,
    LIFTER_DOGHOLE,
} lifter_mode_e ;

struct lifter_cmd_msg
{
    float height;//控制高度
    float d_height ;//控制速度
    float target_angle;
    float dTarget_angle ;
    float motor3angel;
    int enable;
    float Kd;
    lifter_mode_e ctrl_mode;
    lifter_mode_e last_mode;
};

/**
 * @brief 发射器模式
 */
typedef enum
{
    /*发射模式*/
    SHOOT_STOP=0        ,     //射击关闭
    SHOOT_ONE=1         ,     //单发模式
    SHOOT_THREE=2       ,     //三连发模式
    SHOOT_COUNTINUE=3   ,     //自动射击
    SHOOT_REVERSE=4     ,     //堵弹反转
    SHOOT_AUTO=5        ,     //自动发射模式
} shoot_mode_e;

/**
 * @brief 扳机模式
 */
typedef enum
{

    /*扳机状态*/
    TRIGGER_ON=1      ,     //扳机开火状态
    TRIGGER_OFF=0     ,     //扳机闭火状态
    TRIGGER_ING=2     ,     //扳机持续状态

} trigger_mode_e;

/**
 * @brief cmd发布的云台控制数据,由shoot订阅
 */
struct shoot_cmd_msg
{ // 发射器
    shoot_mode_e ctrl_mode;  // 当前发射器控制模式
    shoot_mode_e last_mode;  // 上一次发射器控制模式
    trigger_mode_e trigger_status;
    int16_t shoot_freq;      // 发射弹频
    // TODO: 添加发射弹速控制
    int16_t shoot_speed;     // 发射弹速
    uint8_t cover_open;      // 弹仓盖开关
    uint8_t mirror_enable;     // 倍镜使能开关
    bool friction_status;
    int friction_on_flag;
    int shoot_flag;
};

/**
  * @brief   发射器状态回馈
  */
//TODO:具体回馈设置待讨论
typedef enum
{
    SHOOT_OK=1,   //发射正常
    SHOOT_ERR=0,  //发射异常
    SHOOT_WAITING=2, //发射异常
    SHOOT_REVERSE_ING=3,
} shoot_back_e;

/**
 * @brief 发射机真实反馈状态数据,由shoot发布
 */
struct shoot_fdb_msg
{
    shoot_back_e trigger_status;  // shoot状态反馈
    int16_t trigger_motor_current; //拨弹电机电流，传给cmd控制反转
    int shoot_cnt;
    int reverse_cnt;
};

/* ------------------------------ trans解析自瞄数据 ------------------------------ */
/**
 * @brief 上位机自瞄数据,由trans发布
 */
struct trans_fdb_msg
{
    float yaw;
    float pitch;
    float roll;
    float yaw_filtered;
    float pitch_filtered;
    uint8_t heartbeat;
};

#endif