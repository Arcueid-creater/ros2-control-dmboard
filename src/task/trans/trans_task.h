/**
 * @file trans_task.h
 * @brief 上下位机通讯线程 - 新协议（0xFF 0xAA + CRC32）
 * @date 2025-02-08
 */
#ifndef TRANS_TASK_H
#define TRANS_TASK_H

#include <stdint.h>

/* ==================== 新通信协议配置 ==================== */
#define MOTOR_FRAME_HEADER_1    0xFF
#define MOTOR_FRAME_HEADER_2    0xAA
#define MOTOR_MAX_DATA_LEN      1024    // 最大数据长度（可根据需要调整：512/1024/2048）

/* ==================== 数据包结构 ==================== */

/**
 * @brief 电机控制/反馈数据包（灵活长度）
 * 格式：[0xFF][0xAA][长度(2)][数据(N)][CRC32(4)]
 */
typedef struct {
    uint8_t header[2];      // 0xFF, 0xAA
    uint16_t data_length;   // 数据段长度
    uint8_t data[MOTOR_MAX_DATA_LEN];  // 实际数据
    uint32_t crc32;         // CRC32校验（发送时自动计算）
} __attribute__((packed)) MotorPacket_t;



/* ==================== 颜色定义 ==================== */
typedef enum {
    RED = 1,
    BLUE = 0,
    UNKNOWN = -1
} TeamColor;

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化通信任务
 */
void trans_task_init(void);

/**
 * @brief 通信控制任务
 */
void trans_control_task(void);

/**
 * @brief 发送单个电机数据
 * @param motor_id 电机ID
 * @param position 位置
 * @param velocity 速度
 * @param current 电流
 * @param temperature 温度
 */
void send_single_motor(uint8_t motor_id, float position, float velocity, 
                       float current, float temperature);

/**
 * @brief 发送多个电机数据（批量）
 * @param motors 电机数据数组
 * @param count 电机数量
 */


/**
 * @brief 发送自定义数据
 * @param data 数据指针
 * @param length 数据长度
 */
void send_custom_data(uint8_t *data, uint16_t length);

/**
 * @brief 发送浮点数数组（最简单）
 * @param floats 浮点数数组
 * @param count 浮点数数量
 */
void send_float_array(float *floats, uint8_t count);

/**
 * @brief USB数据接收处理
 */
void process_usb_data(uint8_t* Buf, uint32_t *Len);

/**
 * @brief 获取接收到的电机数据
 */
struct trans_fdb_msg* get_trans_fdb(void);

/**
 * @brief 获取接收到的原始数据
 * @param data 输出缓冲区
 * @param length 输出数据长度
 * @return 1=有新数据, 0=无新数据
 */
uint8_t get_received_data(uint8_t *data, uint16_t *length);

/**
 * @brief 检查是否有新数据
 * @return 1=有新数据, 0=无新数据
 */
uint8_t has_new_data(void);

/* ==================== 线程间通信相关 ==================== */


/**
 * @brief INS消息结构体（根据你的系统定义）
 */

/**
 * @brief 底盘命令消息结构体
 */


/**
 * @brief 云台命令消息结构体
 */

/**
 * @brief 云台反馈消息结构体

/**
 * @brief 云台IMU消息结构体
 */


#endif // TRANS_TASK_H
