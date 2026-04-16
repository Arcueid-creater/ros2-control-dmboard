#include <stdio.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "rm_module.h"
#include "usbd_cdc_if.h"
#include "trans_task.h"
#include "gimbal_task.h"
#include "rc_dbus.h"

#define HEART_BEAT 500 // ms

/* ==================== USB接收：队列传输（新增）==================== */
#define USB_RX_MSG_LEN          512
#define USB_RX_MSG_COUNT        8

typedef struct
{
    uint16_t len;
    uint8_t data[USB_RX_MSG_LEN];
} usb_rx_msg_t;

static QueueHandle_t usb_rx_queue = NULL;
static usb_rx_msg_t usb_rx_msg_pool[USB_RX_MSG_COUNT];
static uint8_t usb_rx_msg_idx = 0;
static void process_usb_bytes(uint8_t* Buf, uint16_t Len);

/* ==================== 线程间通信相关 ==================== */
static rc_dbus_obj_t *rc_now, *rc_last;
// 发布
MCN_DECLARE(transmission_fdb_topic);
static struct trans_fdb_msg trans_fdb_data;

// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;

MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chass_cmd;

MCN_DECLARE(gimbal_cmd);
static McnNode_t gimbal_cmd_node;
static struct gimbal_cmd_msg gimbal_cmd;

MCN_DECLARE(gimbal_fdb_topic);
static McnNode_t gimbal_fdb_node;
static struct gimbal_fdb_msg gimbal_fdb;

MCN_DECLARE(gimbal_ins_topic);
static McnNode_t gimbal_ins_node;
static struct dm_imu_t gim_ins;
MCN_DECLARE(chassis_motor_trans_topic);
static McnNode_t chassis_motor_trans_node;
static struct chassis_motor_msg chassis_motor_msg;
// 内部函数声明
static void trans_pub_push(void);
static void trans_sub_init(void);
static void trans_sub_pull(void);

static void send_rc_dbus_data(const rc_dbus_obj_t* rc);

/* ==================== 全局变量 ==================== */
static uint32_t heart_dt;
static float trans_dt;
static float trans_start;
static float yaw_filtered = 0;
static float pitch_filtered = 0;
static float yaw_obs = 0;
TeamColor team_color = UNKNOWN;

/* ==================== 接收到的底盘命令存储 ==================== */
static chassis_cmd_received_t chassis_cmd_received = {0};

/**
 * @brief 获取接收到的底盘速度命令
 */
chassis_cmd_received_t* get_chassis_cmd_received(void)
{
    return &chassis_cmd_received;
}

/* ==================== 接收数据缓冲区（新增）==================== */
// 存储接收到的原始数据
static uint8_t received_data_buffer[MOTOR_MAX_DATA_LEN];
static uint16_t received_data_length = 0;
static uint8_t data_ready_flag = 0;  // 数据就绪标志

/**
 * @brief 获取接收到的数据
 * @param data 输出缓冲区
 * @param length 输出数据长度
 * @return 1=有新数据, 0=无新数据
 */
uint8_t get_received_data(uint8_t *data, uint16_t *length)
{
    if (data_ready_flag)
    {
        memcpy(data, received_data_buffer, received_data_length);
        *length = received_data_length;
        data_ready_flag = 0;  // 清除标志
        return 1;
    }
    return 0;
}

/**
 * @brief 检查是否有新数据
 */
uint8_t has_new_data(void)
{
    return data_ready_flag;
}

/* ==================== CRC32查找表 ==================== */
static const uint32_t CRC32_TABLE[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

/**
 * @brief 计算CRC32
 */
static uint32_t calculate_crc32(const uint8_t* data, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ CRC32_TABLE[index];
    }
    return ~crc;
}

/* ==================== 接收相关 ==================== */
static uint8_t rx_buffer[512];
static uint16_t rx_index = 0;

typedef enum {
    WAIT_FOR_HEADER1,
    WAIT_FOR_HEADER2,
    RECEIVING_LENGTH,
    RECEIVING_DATA
} RxState_e;

static RxState_e rx_state = WAIT_FOR_HEADER1;
static uint16_t expected_data_length = 0;

/* ==================== 上发数据类型定义 ==================== */

typedef enum
{
    TRANS_MSG_ID_RC_DBUS = 0x10,
    TRANS_MSG_ID_CHASSIS_CMD = 0x20,  // 底盘速度命令
} trans_msg_id_e;

static void send_rc_dbus_data(const rc_dbus_obj_t* rc)
{
    uint8_t data[256];
    size_t offset = 0;

    if (rc == NULL)
    {
        return;
    }

    data[offset++] = (uint8_t)TRANS_MSG_ID_RC_DBUS;

    memcpy(data + offset, &rc->ch1, sizeof(rc->ch1)); offset += sizeof(rc->ch1);
    memcpy(data + offset, &rc->ch2, sizeof(rc->ch2)); offset += sizeof(rc->ch2);
    memcpy(data + offset, &rc->ch3, sizeof(rc->ch3)); offset += sizeof(rc->ch3);
    memcpy(data + offset, &rc->ch4, sizeof(rc->ch4)); offset += sizeof(rc->ch4);

    data[offset++] = rc->sw1;
    data[offset++] = rc->sw2;

    memcpy(data + offset, &rc->mouse.x, sizeof(rc->mouse.x)); offset += sizeof(rc->mouse.x);
    memcpy(data + offset, &rc->mouse.y, sizeof(rc->mouse.y)); offset += sizeof(rc->mouse.y);
    data[offset++] = rc->mouse.l;
    data[offset++] = rc->mouse.r;

    memcpy(data + offset, &rc->kb.key_code, sizeof(rc->kb.key_code)); offset += sizeof(rc->kb.key_code);
    memcpy(data + offset, &rc->wheel, sizeof(rc->wheel)); offset += sizeof(rc->wheel);
    for (int i = 0;i<4; i++)
    {
        chassis_motor_trans msgs=chassis_motor_msg.motor_tran_msg[i];
        memcpy(data + offset, &msgs.motor_id, sizeof(msgs.motor_id)); offset += sizeof(msgs.motor_id);          //1
        memcpy(data + offset, &msgs.speed_rpm, sizeof(msgs.speed_rpm)); offset += sizeof(msgs.speed_rpm);       //2
        memcpy(data + offset, &msgs.total_angle, sizeof(msgs.total_angle)); offset += sizeof(msgs.total_angle);//4
        memcpy(data + offset, &msgs.ecd, sizeof(msgs.ecd)); offset += sizeof(msgs.ecd);                         //2
        memcpy(data + offset, &msgs.real_current, sizeof(msgs.real_current)); offset += sizeof(msgs.real_current);//2
        memcpy(data + offset, &msgs.temperature, sizeof(msgs.temperature)); offset += sizeof(msgs.temperature);//1
    }

    send_custom_data(data, (uint16_t)offset);
}

/* ==================== 发送函数 ==================== */

/**
 * @brief 通用发送函数（内部使用）
 */
static void send_packet(uint8_t *data, uint16_t length)
{
    static uint8_t tx_buffer[256];
    size_t offset = 0;
    
    // 1. 帧头
    tx_buffer[offset++] = MOTOR_FRAME_HEADER_1;
    tx_buffer[offset++] = MOTOR_FRAME_HEADER_2;
    
    // 2. 数据长度
    memcpy(tx_buffer + offset, &length, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    
    // 3. 数据
    memcpy(tx_buffer + offset, data, length);
    offset += length;
    
    // 4. 计算CRC32（从帧头到数据结束）
    uint32_t crc = calculate_crc32(tx_buffer, offset);
    memcpy(tx_buffer + offset, &crc, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    // 5. 发送
    CDC_Transmit_HS(tx_buffer, offset);
}

/**
 * @brief 发送单个电机数据
 */
void send_single_motor(uint8_t motor_id, float position, float velocity, 
                       float current, float temperature)
{
    uint8_t data[17];  // 1 + 4*4 = 17字节
    size_t offset = 0;
    
    // 电机ID
    data[offset++] = motor_id;
    
    // 位置
    memcpy(data + offset, &position, sizeof(float));
    offset += sizeof(float);
    
    // 速度
    memcpy(data + offset, &velocity, sizeof(float));
    offset += sizeof(float);
    
    // 电流
    memcpy(data + offset, &current, sizeof(float));
    offset += sizeof(float);
    
    // 温度
    memcpy(data + offset, &temperature, sizeof(float));
    offset += sizeof(float);
    
    send_packet(data, offset);
}


/**
 * @brief 发送自定义数据
 */
void send_custom_data(uint8_t *data, uint16_t length)
{
    send_packet(data, length);
}

/**
 * @brief 发送浮点数数组（最简单）
 */
void send_float_array(float *floats, uint8_t count)
{
    uint8_t data[256];
    size_t offset = 0;
    
    for (uint8_t i = 0; i < count; i++)
    {
        memcpy(data + offset, &floats[i], sizeof(float));
        offset += sizeof(float);
    }
    
    send_packet(data, offset);
}

/* ==================== 接收函数 ==================== */

/**
 * @brief USB数据接收处理
 */
void process_usb_data(uint8_t* Buf, uint32_t *Len)
{
    if (usb_rx_queue == NULL || Buf == NULL || Len == NULL || *Len == 0)
    {
        return;
    }

    uint32_t in_len = *Len;
    if (in_len > USB_RX_MSG_LEN)
    {
        in_len = USB_RX_MSG_LEN;
    }

    // 从静态池中获取一个消息对象
    usb_rx_msg_t *msg = &usb_rx_msg_pool[usb_rx_msg_idx];
    msg->len = (uint16_t)in_len;
    memcpy(msg->data, Buf, in_len);

    // 发送消息指针。注意：这里发送的是池中对象的指针。
    // usb_rx_msg_idx 简单自增实现轮转（Circular Buffer 思想）
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xQueueSendFromISR(usb_rx_queue, &msg, &xHigherPriorityTaskWoken) == pdPASS)
        {
            usb_rx_msg_idx = (usb_rx_msg_idx + 1) % USB_RX_MSG_COUNT;
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void process_usb_bytes(uint8_t* Buf, uint16_t Len)
{
    for (uint16_t i = 0; i < Len; i++)
    {
        uint8_t byte = Buf[i];

        switch (rx_state)
        {
            case WAIT_FOR_HEADER1:
                if (byte == MOTOR_FRAME_HEADER_1)
                {
                    rx_index = 0;
                    rx_buffer[rx_index++] = byte;
                    rx_state = WAIT_FOR_HEADER2;
                }
                break;

            case WAIT_FOR_HEADER2:
                if (byte == MOTOR_FRAME_HEADER_2)
                {
                    rx_buffer[rx_index++] = byte;
                    rx_state = RECEIVING_LENGTH;
                }
                else
                {
                    rx_state = WAIT_FOR_HEADER1;
                }
                break;

            case RECEIVING_LENGTH:
                rx_buffer[rx_index++] = byte;
                if (rx_index >= 4)  // 帧头(2) + 长度(2)
                {
                    memcpy(&expected_data_length, rx_buffer + 2, sizeof(uint16_t));
                    rx_state = RECEIVING_DATA;
                }
                break;

            case RECEIVING_DATA:
                rx_buffer[rx_index++] = byte;

                // 检查是否接收完整：帧头(2) + 长度(2) + 数据(N) + CRC32(4)
                if (rx_index >= 4 + expected_data_length + 4)
                {
                    // 验证CRC32
                    size_t crc_offset = 4 + expected_data_length;
                    uint32_t received_crc;
                    memcpy(&received_crc, rx_buffer + crc_offset, sizeof(uint32_t));

                    uint32_t calculated_crc = calculate_crc32(rx_buffer, crc_offset);

                    if (received_crc == calculated_crc)
                    {
                        // CRC校验通过，解析数据
                        uint8_t *data = rx_buffer + 4;  // 跳过帧头和长度

                        // ========== 保存接收到的数据（新增）==========
                        // 复制数据到全局缓冲区供外部使用
                        memcpy(received_data_buffer, data, expected_data_length);
                        received_data_length = expected_data_length;
                        data_ready_flag = 1;  // 设置数据就绪标志

                        // ========== 解析数据 ==========
                        if (expected_data_length >= 1)
                        {
                            size_t offset = 0;
                            uint8_t msg_id = data[offset++];

                            switch (msg_id)
                            {
                                case TRANS_MSG_ID_CHASSIS_CMD:
                                {
                                    // 解析底盘速度命令：3个float（linear_x, linear_y, angular_z）
                                    if (expected_data_length >= 1 + 3 * sizeof(float))
                                    {
                                        float linear_x, linear_y, angular_z;
                                        memcpy(&linear_x, data + offset, sizeof(float)); offset += sizeof(float);
                                        memcpy(&linear_y, data + offset, sizeof(float)); offset += sizeof(float);
                                        memcpy(&angular_z, data + offset, sizeof(float)); offset += sizeof(float);

                                        // 存储到底盘命令结构体
                                        chassis_cmd_received.linear_x = linear_x;
                                        chassis_cmd_received.linear_y = linear_y;
                                        chassis_cmd_received.angular_z = angular_z;
                                        chassis_cmd_received.updated = 1;  // 标记数据已更新

                                        // TODO: 在这里添加你的底盘控制逻辑
                                        // 例如：将速度命令转换为电机控制指令
                                        // 或者发布到底盘控制topic
                                    }
                                    break;
                                }

                                case TRANS_MSG_ID_RC_DBUS:
                                {
                                    // 如果需要接收遥控器数据的话
                                    // 添加解析逻辑
                                    break;
                                }

                                default:
                                {
                                    // 未知消息ID
                                    break;
                                }
                            }
                        }
                    }

                    // 重置状态
                    rx_state = WAIT_FOR_HEADER1;
                    rx_index = 0;
                }
                break;
        }
    }
}

/**
 * @brief 获取接收到的数据
 */
struct trans_fdb_msg* get_trans_fdb(void)
{
    return &trans_fdb_data;
}

/* ==================== 任务函数 ==================== */

void trans_task_init(void)
{
    memset(&trans_fdb_data, 0, sizeof(trans_fdb_data));
    rx_state = WAIT_FOR_HEADER1;
    rx_index = 0;

    if (usb_rx_queue == NULL)
    {
        usb_rx_queue = xQueueCreate(USB_RX_MSG_COUNT, sizeof(usb_rx_msg_t *));
    }

    rc_now = dbus_rc_init();
    // 初始化订阅
    trans_sub_init();
    rc_last = (rc_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST
    heart_dt = dwt_get_time_ms();
}

void trans_control_task(void)
{
    trans_start = dwt_get_time_ms();
    
    // 订阅数据更新
    trans_sub_pull();

    // 处理消息队列中的所有消息
    usb_rx_msg_t *msg = NULL;
    while (xQueueReceive(usb_rx_queue, &msg, 0) == pdPASS)
    {
        if (msg != NULL && msg->len > 0)
        {
            process_usb_bytes(msg->data, msg->len);
        }
    }
    
    /* ==================== 心跳检测 ==================== */
    if ((dwt_get_time_ms() - heart_dt) >= HEART_BEAT)
    {
        heart_dt = dwt_get_time_ms();
        // 可以在这里发送心跳包
    }
    
    /* ==================== 发送数据示例 ==================== */

    /* 上发：已接收到的遥控器数据（DBUS decode 后的 rc_now） */
    send_rc_dbus_data(rc_now);

    /* ==================== 发布数据更新 ==================== */
    trans_pub_push();
    
    /* ==================== 性能监测 ==================== */
    trans_dt = dwt_get_time_ms() - trans_start;
    if (trans_dt > 1)
    {
        // LOGINFO("Transmission Task is being DELAY! dt = [%f]\r\n", &trans_dt);
    }
}

/* ==================== 消息订阅发布 ==================== */

static void trans_pub_push(void)
{
    mcn_publish(MCN_HUB(transmission_fdb_topic), &trans_fdb_data);
}

static void trans_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
    gimbal_cmd_node = mcn_subscribe(MCN_HUB(gimbal_cmd), NULL, NULL);
    gimbal_fdb_node = mcn_subscribe(MCN_HUB(gimbal_fdb_topic), NULL, NULL);
    gimbal_ins_node = mcn_subscribe(MCN_HUB(gimbal_ins_topic), NULL, NULL);
    chassis_motor_trans_node= mcn_subscribe(MCN_HUB(chassis_motor_trans_topic), NULL, NULL);
}

static void trans_sub_pull(void)
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
        mcn_copy(MCN_HUB(gimbal_cmd), gimbal_cmd_node, &gimbal_cmd);
    }
    if (mcn_poll(gimbal_fdb_node))
    {
        mcn_copy(MCN_HUB(gimbal_fdb_topic), gimbal_fdb_node, &gimbal_fdb);
    }
    if (mcn_poll(gimbal_ins_node))
    {
        mcn_copy(MCN_HUB(gimbal_ins_topic), gimbal_ins_node, &gim_ins);
    }
    if (mcn_poll(chassis_motor_trans_node))
    {
        mcn_copy(MCN_HUB(chassis_motor_trans_topic), chassis_motor_trans_node, &chassis_motor_msg);
    }
}
