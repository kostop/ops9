#include "robot_com.h"
#include "usart.h"
#include <string.h>
#include "crc8_crc16.h"

#define ROBOT_TX_DATA_LEN		14
#define ROBOT_RX_DATA_LEN		2

robot_com_para_t robot_com_para;

uint16_t robot_com_receive_count;
uint8_t robot_com_receive_state;
uint8_t robot_com_receive_buffer;

uint8_t robot_tx_data[ROBOT_TX_DATA_LEN];
uint8_t robot_rx_data[ROBOT_RX_DATA_LEN];

void Robot_com_init(void)
{
	HAL_UART_Receive_IT(&huart2, &robot_com_receive_buffer, 1);
	robot_com_para.robot_com_control_point = get_control_para_point();
	robot_com_para.robot_com_imu_point = get_imu_para_point();
	robot_com_para.Robot_TX_data.header = 0x5C;
}

void Robot_com_feed_back(void)
{
	robot_com_para.Robot_TX_data.x = robot_com_para.robot_com_control_point->chassis_pos.x;
	robot_com_para.Robot_TX_data.y = robot_com_para.robot_com_control_point->chassis_pos.y;
	robot_com_para.Robot_TX_data.z = robot_com_para.robot_com_control_point->chassis_pos.z;
	memcpy(&robot_tx_data, &robot_com_para.Robot_TX_data, ROBOT_TX_DATA_LEN);
	append_CRC8_check_sum(robot_tx_data, ROBOT_TX_DATA_LEN);
}

void Robot_com_send_data(void)
{
	Robot_com_feed_back();

	HAL_UART_Transmit_IT(&huart2, robot_tx_data, ROBOT_TX_DATA_LEN);
}

void Robot_com_call_back(void)
{
	switch(robot_com_receive_state)
	{
		case 0:
			if(robot_com_receive_buffer == 0xC5)
			{
				robot_rx_data[robot_com_receive_count] = robot_com_receive_buffer;
				robot_com_receive_count++;
				robot_com_receive_state = 1;
			}
			break;
		case 1:
			if(robot_com_receive_buffer == 0xC5)
			{
				robot_com_receive_count = 0;
				robot_com_receive_state = 0;
			}
			else
			{
				robot_rx_data[robot_com_receive_count] = robot_com_receive_buffer;
				robot_com_receive_count++;
				robot_com_receive_state = 2;
			}
			break;
		case 2:
			memcpy(&robot_com_para.Robot_RX_data, &robot_rx_data, ROBOT_RX_DATA_LEN);
			robot_com_receive_state = 0;
			robot_com_receive_count = 0;
			break;
			
	}
	HAL_UART_Receive_IT(&huart2, &robot_com_receive_buffer, 1);
}

uint8_t get_ops_cmd_data(void)
{
	return robot_com_para.Robot_RX_data.data;
}

