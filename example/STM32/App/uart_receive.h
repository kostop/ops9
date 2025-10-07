#ifndef _UART_RECEIVE_H
#define _UART_RECEIVE_H

#include "main.h"
#include "usart.h"
#include "INS_task.h"
#include "serial_screen.h"

//#define REMOTE_UART			huart1
//#define REMOTE_UART_I		USART1
#define ROBOT_COM_UART		huart1
#define ROBOT_COM_UART_I	USART1
#define CAR_COM_UART		huart4
#define CAR_COM_UART_I		UART4
#define SERIAL_SCREEN		huart5
#define SERIAL_SCREEN_I		UART5
#define OPS9_UART			huart2
#define OPS9_UART_I			USART2
#define MOTOR_UART 			huart3
#define MOTOR_UART_I		USART3

#define UART_DATA_SIZE_MAX		128


typedef struct
{
	uint8_t motor_data_size;
	uint8_t Motor_data[UART_DATA_SIZE_MAX];
}uart_data_t;

extern uart_data_t uart_data;

void Motor_UART_Init(void);
const uart_data_t* get_UART_Receive_data_point(void);

#endif
