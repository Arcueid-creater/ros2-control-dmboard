/**
 * @file unitree_motor.c
 * @author Generated based on DJ_Motor and DM_Motor driver architecture
 * @brief Unitree GO-M8010-6 Motor Driver Implementation (RS485)
 * @version 1.0
 * @date 2025-11-07
 * 
 */

#include "unitree_motor.h"
#include "motor_control.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmsis_os.h>
/* 全局电机对象数组 */
static unitree_motor_object_t *unitree_motor_obj[UNITREE_MOTOR_CNT] = {NULL};
static uint8_t motor_idx = 0; // 已注册的电机数量

/* ==================== 内部辅助函数 ==================== */

/**
 * @brief 根据HAL返回状态更新电机统计信息
 */
static void update_motor_statistics(unitree_motor_object_t *motor, HAL_StatusTypeDef status)
{
    if (status == HAL_OK && motor->recv_data.correct == 1) {
        motor->measure.update_cnt++;
    } else {
        motor->measure.error_cnt++;
    }
}

/**
 * @brief 将底层接收数据解析到测量结构体
 */
static void parse_recv_data(unitree_motor_object_t *motor)
{
    MOTOR_recv *recv = &motor->recv_data;
    unitree_motor_measure_t *measure = &motor->measure;
    
    // 复制基本信息
    measure->motor_id = recv->motor_id;
    measure->mode = recv->mode;
    measure->temperature = recv->Temp;
    measure->error = recv->MError;
    
    // 复制物理量
    measure->position = recv->Pos;
    measure->velocity = recv->W;
    measure->torque = recv->T;
    measure->foot_force = recv->footForce;
    
    // 更新数据完整性标志
    measure->correct = recv->correct;
}

/**
 * @brief 将控制参数填充到底层发送结构体
 */
static void fill_send_data(unitree_motor_object_t *motor)
{
    MOTOR_send *send = &motor->send_data;
    unitree_motor_ctrl_t *ctrl = &motor->ctrl;
    
    // 设置基本参数
    send->id = motor->motor_id;
    send->mode = (uint16_t)motor->mode;
    send->channel = motor->channel;
    
    // 设置控制参数
    send->Pos = ctrl->pos_des;
    send->W = ctrl->vel_des;
    send->T = ctrl->tor_des;
    send->K_P = ctrl->kp;
    send->K_W = ctrl->kd;
}

/**
 * @brief 检查电机ID和通道是否冲突
 * @return 1-冲突 0-无冲突
 */
static int check_id_conflict(uint8_t motor_id, Channel_t channel)
{
    for (size_t i = 0; i < motor_idx; i++) {
        if (unitree_motor_obj[i]->motor_id == motor_id && 
            unitree_motor_obj[i]->channel == channel) {
            return 1; // 冲突
        }
    }
    return 0; // 无冲突
}

/* ==================== 外部接口函数 ==================== */

/**
 * @brief 注册一个Unitree电机实例
 */
unitree_motor_object_t* unitree_motor_register(unitree_motor_config_t *config, 
                                               void (*control_callback)(unitree_motor_object_t *motor))
{
    // 检查是否超过最大数量
    if (motor_idx >= UNITREE_MOTOR_CNT) {
        printf("[UNITREE_MOTOR] Error: Motor count exceeds maximum (%d)\r\n", UNITREE_MOTOR_CNT);
        return NULL;
    }
    
    // 检查ID冲突
    if (check_id_conflict(config->motor_id, config->channel)) {
        printf("[UNITREE_MOTOR] Error: Motor ID %d on channel %d already exists!\r\n", 
               config->motor_id, config->channel);
        return NULL;
    }
    
    // 分配内存
    unitree_motor_object_t *motor = (unitree_motor_object_t *)pvPortMalloc(sizeof(unitree_motor_object_t));
    if (motor == NULL) {
        printf("[UNITREE_MOTOR] Error: Memory allocation failed!\r\n");
        return NULL;
    }
    
    // 清零
    memset(motor, 0, sizeof(unitree_motor_object_t));
    
    // 配置基本参数
    motor->motor_id = config->motor_id;
    motor->channel = config->channel;
    motor->mode = config->mode;
    motor->state = UNITREE_MOTOR_STOP; // 默认停止状态
    
    // 设置控制回调
    motor->control_callback = control_callback;
    
    // 初始化控制参数（安全的默认值）
    motor->ctrl.kp = 0.0f;
    motor->ctrl.kd = 0.0f;
    motor->ctrl.pos_des = 0.0f;
    motor->ctrl.vel_des = 0.0f;
    motor->ctrl.tor_des = 0.0f;
    
    // 立即初始化send_data结构体，确保channel等字段被正确设置
    fill_send_data(motor);
    
    // 注册到全局数组
    unitree_motor_obj[motor_idx++] = motor;
    
    printf("[UNITREE_MOTOR] Motor ID %d registered on channel %d (index: %d)\r\n", 
           config->motor_id, config->channel, motor_idx - 1);
    
    return motor;
}

/**
 * @brief 使能电机
 */
void unitree_motor_enable(unitree_motor_object_t *motor)
{
    if (motor == NULL) return;
    
    motor->mode = UNITREE_MODE_FOC;
    motor->state = UNITREE_MOTOR_ENABLED;
    
    // 立即同步到send_data
    fill_send_data(motor);
    
    printf("[UNITREE_MOTOR] Motor ID %d enabled (FOC mode)\r\n", motor->motor_id);
}

/**
 * @brief 失能电机
 */
void unitree_motor_disable(unitree_motor_object_t *motor)
{
    if (motor == NULL) return;
    
    motor->mode = UNITREE_MODE_IDLE;
    motor->state = UNITREE_MOTOR_STOP;
    
    // 清除控制参数
    unitree_motor_clear_ctrl(motor);
    
    // 发送停止指令
    unitree_motor_send(motor);
    
    printf("[UNITREE_MOTOR] Motor ID %d disabled (IDLE mode)\r\n", motor->motor_id);
}

/**
 * @brief 设置电机控制参数
 */
void unitree_motor_set_control(unitree_motor_object_t *motor, 
                               float pos, float vel, float tor, 
                               float kp, float kd)
{
    if (motor == NULL) return;
    
    motor->ctrl.pos_des = pos;
    motor->ctrl.vel_des = vel;
    motor->ctrl.tor_des = tor;
    motor->ctrl.kp = kp;
    motor->ctrl.kd = kd;
    
    // 同时更新测量结构体中的目标值（用于调试）
    motor->measure.target_pos = pos;
    motor->measure.target_vel = vel;
    motor->measure.target_tor = tor;
    
    // 立即同步到send_data
    fill_send_data(motor);
}

/**
 * @brief 清除电机控制参数
 */
void unitree_motor_clear_ctrl(unitree_motor_object_t *motor)
{
    if (motor == NULL) return;
    
    motor->ctrl.kp = 0.0f;
    motor->ctrl.kd = 0.0f;
    motor->ctrl.pos_des = 0.0f;
    motor->ctrl.vel_des = 0.0f;
    motor->ctrl.tor_des = 0.0f;
    
    // 立即同步到send_data
    fill_send_data(motor);
}

/**
 * @brief 发送单个电机的控制指令
 */
int unitree_motor_send(unitree_motor_object_t *motor)
{
    if (motor == NULL) return -1;
    
    HAL_StatusTypeDef status;
    
    // 如果电机处于停止状态，清除控制参数
    if (motor->state == UNITREE_MOTOR_STOP) {
        unitree_motor_clear_ctrl(motor);
    }
    
    // 填充发送数据
    fill_send_data(motor);
    
    // 调用底层发送接收函数
    status = SERVO_Send_recv(&motor->send_data, &motor->recv_data);
    
    // 更新统计信息
    update_motor_statistics(motor, status);
    
    // 如果接收成功，解析数据
    if (status == HAL_OK && motor->recv_data.correct == 1) {
        parse_recv_data(motor);
        return 0;
    }
    
    // 通信失败
    return -1;
}

/**
 * @brief 电机控制函数（周期性调用）
 */
int callbackaaa=0;
void unitree_motor_control(void)
{
    unitree_motor_object_t *motor;
    
    // 遍历所有已注册的电机
    // static uint8_t i;
        for (int i=0;i<motor_idx;i++)
        {
            motor = unitree_motor_obj[i];

            if (motor == NULL) return;

            // 如果有控制回调函数，先调用回调
            if (motor->control_callback != NULL && motor->state == UNITREE_MOTOR_ENABLED) {
                callbackaaa++;
                motor->control_callback(motor);

            }

            // 发送控制指令并接收反馈
            unitree_motor_send(motor);
            //RS485
            if (motor->measure.error_cnt>=100)
            {

                // unitree_motor_rs485_reset();
                motor->measure.error_cnt=0;
                memset(motor, 0, sizeof(unitree_motor_object_t));
                Software_Reset();
                // unitree_motor_rs485_init();
            }
        }
}

/**
 * @brief 获取指定ID的电机对象
 */
unitree_motor_object_t* unitree_motor_get_by_id(uint8_t motor_id, Channel_t channel)
{
    for (size_t i = 0; i < motor_idx; i++) {
        if (unitree_motor_obj[i]->motor_id == motor_id && 
            unitree_motor_obj[i]->channel == channel) {
            return unitree_motor_obj[i];
        }
    }
    return NULL;
}


void Software_Reset(void)
{
    // 可选：关闭全局中断，避免复位过程中被中断干扰（推荐添加）
    __disable_irq();

    // 关键：调用NVIC_SystemReset()触发软件复位
    NVIC_SystemReset();

    // 以下代码永远不会执行，因为复位后程序会从头开始运行
    while(1);
}

