#include "usart.h"
#include "motor_control.h"
#include "crc_ccitt.h"
#include "stdio.h"
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "string.h"
#include "unitree_motor.h"
#define huart2_DE_Pin GPIO_PIN_4
#define huart2_DE_GPIO_Port GPIOD

#define huart3_DE_Pin GPIO_PIN_14
#define huart3_DE_GPIO_Port GPIOB
#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 


extern  QueueHandle_t motor_rx_queue;
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     MotorRxMessage_t msg;
//
//     if (motor_rx_queue == NULL)
//     {
//         return;
//     }
//
//     if (huart->Instance == USART2)
//     {
//         if (Size == sizeof(MotorData_t))
//         {
//             msg.channel = usart2_485;
//             memcpy(&msg.motor_data, usart2_rx_buffer, sizeof(MotorData_t));
//             xQueueSendFromISR(motor_rx_queue, &msg, &xHigherPriorityTaskWoken);
//         }
//         HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer, sizeof(usart2_rx_buffer));
//     }
//     else if (huart->Instance == USART3)
//     {
//         if (Size == sizeof(MotorData_t))
//         {
//             msg.channel = usart3_485;
//             memcpy(&msg.motor_data, usart3_rx_buffer, sizeof(MotorData_t));
//             xQueueSendFromISR(motor_rx_queue, &msg, &xHigherPriorityTaskWoken);
//         }
//         HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_buffer, sizeof(usart3_rx_buffer));
//     }
//
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }

int modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
//		SATURATE(motor_s->id,   0,    15);
//		SATURATE(motor_s->mode, 0,    7);
		SATURATE(motor_s->K_P,  0.0f,   25.599f);
		SATURATE(motor_s->K_W,  0.0f,   25.599f);
		SATURATE(motor_s->T,   -127.99f,  127.99f);
		SATURATE(motor_s->W,   -804.00f,  804.00f);
		SATURATE(motor_s->Pos, -411774.0f,  411774.0f);

    motor_s->motor_send_data.mode.id   = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos  = motor_s->K_P/25.6f*32768;
    motor_s->motor_send_data.comd.k_spd  = motor_s->K_W/25.6f*32768;
    motor_s->motor_send_data.comd.pos_des  = motor_s->Pos/6.2832f*32768;
    motor_s->motor_send_data.comd.spd_des  = motor_s->W/6.2832f*256;
    motor_s->motor_send_data.comd.tor_des  = motor_s->T*256;
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

int extract_data(MOTOR_recv *motor_r)
{
    if(motor_r->motor_recv_data.CRC16 !=
        crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14)){
        // printf("[WARNING] Receive data CRC error");
        motor_r->correct = 0;
        return motor_r->correct;
    }
    else
		{
        motor_r->motor_id = motor_r->motor_recv_data.mode.id;
        motor_r->mode = motor_r->motor_recv_data.mode.status;
        motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError = motor_r->motor_recv_data.fbk.MError;
        motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f ;
        motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
        motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768;
				motor_r->footForce = motor_r->motor_recv_data.fbk.force;
				motor_r->correct = 1;
        return motor_r->correct;
    }
}
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
// 	if (huart == &huart2) {
// 		SET_huart2_DE_DOWN();
// 		SET_huart3_DE_DOWN();
// 		// 如有需要，也可以在这里给任务发通知/信号量
// 	} else if (huart == &huart3) {
// 		SET_huart2_DE_DOWN();
// 		SET_huart3_DE_DOWN();
// 	}
// }
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData)
{
	HAL_StatusTypeDef txStatus = HAL_ERROR;
	MotorRxMessage_t msg;
	BaseType_t recv_res;
	TickType_t ticks_to_wait;
	uint8_t *rp = (uint8_t *)&rData->motor_recv_data;

	if (motor_rx_queue == NULL)
	{
		return HAL_ERROR;
	}

	modify_data(pData);

	if (pData->channel==usart2_485)
	{
		SET_huart2_DE_UP();
		SET_huart3_DE_UP();
		txStatus = HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&(pData->motor_send_data), sizeof(pData->motor_send_data));
	}
	else if (pData->channel==usart3_485)
	{
		SET_huart2_DE_UP();
		SET_huart3_DE_UP();
		txStatus = HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&(pData->motor_send_data), sizeof(pData->motor_send_data));
		//RS485
	}

	if (txStatus != HAL_OK)
	{
		return txStatus;
	}

	// if (txStatus != HAL_OK)
	// {
	// 	return txStatus;
	// }

	ticks_to_wait = pdMS_TO_TICKS(10);

	for (;;)
	{
		recv_res = xQueueReceive(motor_rx_queue, &msg, ticks_to_wait);
		if (recv_res != pdTRUE)
		{
			return HAL_TIMEOUT;
		}

		if ((pData->channel == usart2_485 && msg.channel == usart2_485) ||
		    (pData->channel == usart3_485 && msg.channel == usart3_485))
		{
			memcpy(&rData->motor_recv_data, &msg.motor_data, sizeof(MotorData_t));
			break;
		}
	}
	if((rp[0] == 0xFE || rp[0] == 0xFD) && rp[1] == 0xEE)
	{
			rData->correct = 1;
        extract_data(rData);
        return HAL_OK;
    }

    return HAL_ERROR;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		SET_huart2_DE_DOWN();
		SET_huart3_DE_DOWN();
	}
	else if (huart == &huart3)
	{
		SET_huart2_DE_DOWN();
		SET_huart3_DE_DOWN();
	}
}
