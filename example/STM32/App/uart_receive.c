#include "uart_receive.h"
#include "robot_communication.h"

uart_data_t uart_data;

uint8_t motor_receive_temp;
uint8_t motor_receive_data[UART_DATA_SIZE_MAX];
uint8_t motor_receive_cnt;

uint8_t lift_receive_temp;
uint8_t lift_receive_data[UART_DATA_SIZE_MAX];
uint8_t lift_receive_cnt;

void Motor_UART_Init(void)
{
	HAL_UART_Init(&MOTOR_UART);
	HAL_UART_Receive_IT(&MOTOR_UART, &motor_receive_temp, 1);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == MOTOR_UART_I)
	{
		motor_receive_data[motor_receive_cnt] = motor_receive_temp;
		motor_receive_cnt++;
		if(motor_receive_temp == 0x6B)
		{
			uart_data.motor_data_size = motor_receive_cnt-1;
			memcpy(&uart_data.Motor_data, &motor_receive_data, UART_DATA_SIZE_MAX);
			memset(&motor_receive_data, 0, UART_DATA_SIZE_MAX);
			motor_receive_cnt = 0;
		}
		HAL_UART_Receive_IT(&MOTOR_UART, &motor_receive_temp, 1);
	}
	else if(huart->Instance == ROBOT_COM_UART_I)
	{
		Robot_Communication_Callback();
	}
	else if(huart->Instance == OPS9_UART_I)
	{
		
//		INS_Communication_Callback();
	}
	else if(huart->Instance == SERIAL_SCREEN_I)
	{
		Serial_Screen_Communication_Callback();
	}
}

