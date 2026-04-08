// Tip: 遥控器接收模块
#include "stm32h7xx_hal.h"
#include "rc_dbus.h"
#include "rm_config.h"
#include <string.h>
#include "cmsis_os.h"
#include "stdlib.h"

extern UART_HandleTypeDef huart5;

/* 数据有效性检查 */
#define VALID_CHANNEL(val) (abs(val) <= RC_MAX_VALUE)


rc_dbus_obj_t rc_dbus_obj[2];   // [0]:当前数据NOW,[1]:上一次的数据LAST


/**
 * @brief 初始化sbus_rc
 *
 * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
rc_dbus_obj_t *dbus_rc_init(void)
{
    // USART_DMAEx_MultiBuffer_Init(&huart5,SBUS_MultiRx_Buf[0], SBUS_MultiRx_Buf[1],36);
    // 遥控器离线检测定时器相关
    return rc_dbus_obj;
}

/**
 * @brief 遥控器dbus数据解析
 *
 * @param rc_dbus_obj 指向dbus_rc实例的指针
 */
int dbus_rc_decode(uint8_t *buff)
{
    /* 下面是正常遥控器数据的处理 */
    rc_dbus_obj[NOW].ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc_dbus_obj[NOW].ch1 -= 1024;
    rc_dbus_obj[NOW].ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc_dbus_obj[NOW].ch2 -= 1024;
    rc_dbus_obj[NOW].ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc_dbus_obj[NOW].ch3 -= 1024;
    rc_dbus_obj[NOW].ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc_dbus_obj[NOW].ch4 -= 1024;

    /* 防止遥控器零点有偏差 */
    if(rc_dbus_obj[NOW].ch1 <= 5 && rc_dbus_obj[NOW].ch1 >= -5)
        rc_dbus_obj[NOW].ch1 = 0;
    if(rc_dbus_obj[NOW].ch2 <= 5 && rc_dbus_obj[NOW].ch2 >= -5)
        rc_dbus_obj[NOW].ch2 = 0;
    if(rc_dbus_obj[NOW].ch3 <= 5 && rc_dbus_obj[NOW].ch3 >= -5)
        rc_dbus_obj[NOW].ch3 = 0;
    if(rc_dbus_obj[NOW].ch4 <= 5 && rc_dbus_obj[NOW].ch4 >= -5)
        rc_dbus_obj[NOW].ch4 = 0;

    /* 拨杆值获取 */
    rc_dbus_obj[NOW].sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc_dbus_obj[NOW].sw2 = (buff[5] >> 4) & 0x0003;

    /* 遥控器异常值处理，函数直接返回 */
    if ((abs(rc_dbus_obj[NOW].ch1) > RC_DBUS_MAX_VALUE) || \
      (abs(rc_dbus_obj[NOW].ch2) > RC_DBUS_MAX_VALUE) || \
      (abs(rc_dbus_obj[NOW].ch3) > RC_DBUS_MAX_VALUE) || \
      (abs(rc_dbus_obj[NOW].ch4) > RC_DBUS_MAX_VALUE))
    {
        memset(&rc_dbus_obj[NOW], 0, sizeof(rc_dbus_obj_t));
        return -1;
    }

    /* 鼠标移动速度获取 */
    rc_dbus_obj[NOW].mouse.x = buff[6] | (buff[7] << 8);
    rc_dbus_obj[NOW].mouse.y = buff[8] | (buff[9] << 8);

    /* 鼠标左右按键键值获取 */
    rc_dbus_obj[NOW].mouse.l = buff[12];
    rc_dbus_obj[NOW].mouse.r = buff[13];

    /* 键盘按键键值获取 */
    rc_dbus_obj[NOW].kb.key_code = buff[14] | buff[15] << 8;

    /* 遥控器左侧上方拨轮数据获取，和遥控器版本有关，有的无法回传此项数据 */
    rc_dbus_obj[NOW].wheel = buff[16] | buff[17] << 8;
    rc_dbus_obj[NOW].wheel -= 1024;

    rc_dbus_obj[LAST] = rc_dbus_obj[NOW];
}


