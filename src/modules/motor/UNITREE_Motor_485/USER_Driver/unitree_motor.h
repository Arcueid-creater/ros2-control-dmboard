/**
 * @file unitree_motor.h
 * @author Generated based on DJ_Motor and DM_Motor driver architecture
 * @brief Unitree GO-M8010-6 Motor Driver (RS485)
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __UNITREE_MOTOR_H
#define __UNITREE_MOTOR_H

#include "main.h"
#include "motor_control.h"
#include "usart.h"
#include <stdint.h>

/* 最大支持的Unitree电机数量 */
#define UNITREE_MOTOR_CNT 8

/* 电机工作模式定义 */
typedef enum
{
    UNITREE_MODE_IDLE = 0,      // 空闲模式（电机锁定）
    UNITREE_MODE_FOC = 1,       // FOC闭环控制模式
    UNITREE_MODE_CALIBRATE = 2, // 编码器校准模式
} unitree_mode_e;

/* 电机工作状态 */
typedef enum
{
    UNITREE_MOTOR_STOP = 0,     // 电机停止
    UNITREE_MOTOR_ENABLED = 1,  // 电机使能
} unitree_working_state_e;
void unitree_motor_rs485_reset(void);
/* 电机反馈数据结构体 */
typedef struct
{
    /* 原始反馈数据 */
    uint8_t motor_id;           // 电机ID                     1
    uint8_t mode;               // 当前工作模式
    int8_t  temperature;        // 电机温度 (°C)               1
    uint8_t error;              // 错误码: 0-正常 1-过热 2-过流 3-过压 4-编码器故障         1
    
    /* 处理后的数据 */
    float position;             // 当前位置 (rad)               1
    float velocity;             // 当前速度 (rad/s)             1
    float torque;               // 当前扭矩 (N.m)               1
    float foot_force;           // 足端传感器数据 (0-4095)
    
    /* 目标值 */
    float target_pos;           // 目标位置 (rad)
    float target_vel;           // 目标速度 (rad/s)
    float target_tor;           // 目标扭矩 (N.m)
    
    /* 通信状态 */
    uint8_t correct;            // 数据是否完整: 1-完整 0-不完整
    uint32_t update_cnt;        // 数据更新计数
    uint8_t error_cnt;         // 通信错误计数                    1

} unitree_motor_measure_t;
void Software_Reset(void);
/* 电机控制参数结构体 */
typedef struct
{
    float kp;                   // 位置刚度系数 (0-25.6)
    float kd;                   // 速度阻尼系数 (0-25.6)
    float pos_des;              // 期望位置 (rad)
    float vel_des;              // 期望速度 (rad/s)
    float tor_des;              // 期望扭矩 (N.m)
} unitree_motor_ctrl_t;

/* 电机配置结构体 */
typedef struct
{
    uint8_t motor_id;           // 电机ID (0-14, 15为广播)
    Channel_t channel;          // 通信通道 (usart2_485 或 usart3_485)
    unitree_mode_e mode;        // 工作模式
} unitree_motor_config_t;

/* Unitree电机对象结构体 */
typedef struct unitree_motor_object
{
    /* 基本配置 */
    uint8_t motor_id;           // 电机ID
    Channel_t channel;          // 通信通道
    unitree_mode_e mode;        // 工作模式
    unitree_working_state_e state; // 工作状态
    
    /* 测量数据 */
    unitree_motor_measure_t measure;
    
    /* 控制参数 */
    unitree_motor_ctrl_t ctrl;
    
    /* 底层通信结构体 */
    MOTOR_send send_data;
    MOTOR_recv recv_data;
    
    /* 控制接口 */
    void (*control_callback)(struct unitree_motor_object *motor);
    
} unitree_motor_object_t;

/* ==================== 函数声明 ==================== */
typedef struct
{
    Channel_t channel;
    MotorData_t motor_data;
} MotorRxMessage_t;

/**
 * @brief 注册一个Unitree电机实例
 * 
 * @param config 电机配置结构体
 * @param control_callback 电机控制回调函数（可选，传NULL则需手动设置控制参数）
 * @return unitree_motor_object_t* 返回电机对象指针，注册失败返回NULL
 */
unitree_motor_object_t* unitree_motor_register(unitree_motor_config_t *config, 
                                               void (*control_callback)(unitree_motor_object_t *motor));

/**
 * @brief 使能电机（设置为FOC模式）
 * 
 * @param motor 电机对象指针
 */
void unitree_motor_enable(unitree_motor_object_t *motor);

/**
 * @brief 失能电机（设置为IDLE模式）
 * 
 * @param motor 电机对象指针
 */
void unitree_motor_disable(unitree_motor_object_t *motor);

/**
 * @brief 电机控制函数，应该在周期性任务中调用
 *        该函数会调用所有已注册电机的控制回调函数，然后发送控制指令
 * 
 * @note 建议调用频率：500Hz-1000Hz
 */
void unitree_motor_control(void);

/**
 * @brief 设置电机控制参数（位置、速度、扭矩、刚度、阻尼）
 * 
 * @param motor 电机对象指针
 * @param pos 期望位置 (rad)
 * @param vel 期望速度 (rad/s)
 * @param tor 期望扭矩 (N.m)
 * @param kp 位置刚度系数 (0-25.6)
 * @param kd 速度阻尼系数 (0-25.6)
 */
void unitree_motor_set_control(unitree_motor_object_t *motor, 
                               float pos, float vel, float tor, 
                               float kp, float kd);

/**
 * @brief 发送单个电机的控制指令（立即发送）
 * 
 * @param motor 电机对象指针
 * @return int 0-成功 -1-失败
 */
int unitree_motor_send(unitree_motor_object_t *motor);

/**
 * @brief 清除电机控制参数
 * 
 * @param motor 电机对象指针
 */
void unitree_motor_clear_ctrl(unitree_motor_object_t *motor);

/**
 * @brief 获取指定ID的电机对象
 * 
 * @param motor_id 电机ID
 * @param channel 通信通道
 * @return unitree_motor_object_t* 返回电机对象指针，未找到返回NULL
 */
unitree_motor_object_t* unitree_motor_get_by_id(uint8_t motor_id, Channel_t channel);

/**
 * @brief 打印电机信息（用于调试）
 * 
 * @param motor 电机对象指针
 */
void unitree_motor_print_info(unitree_motor_object_t *motor);

void unitree_motor_rs485_init(void);

#endif /* __UNITREE_MOTOR_H */

