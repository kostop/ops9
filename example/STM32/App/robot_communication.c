#include "robot_communication.h"
#include "vision_task.h"

#define ROBOT_COM_TASK_DELAY	1

//0.16*0.12		0.185
//640*480

robot_communication_t robot_communication;

uint8_t robot_com_receive_buffer;
uint8_t robot_com_data_cnt;
uint8_t robot_com_receive_state;


void robot_data_send(robot_communication_t* robot_communication)
{
	robot_communication->Robot_Tx_Data.header = 0x5A;
	append_CRC16_check_sum((uint8_t*)&robot_communication->Robot_Tx_Data, robot_communication->tx_data_size);
	memcpy(robot_communication->tx_data, &robot_communication->Robot_Tx_Data, robot_communication->tx_data_size);
	HAL_UART_Transmit_IT(&ROBOT_COM_UART, robot_communication->tx_data, robot_communication->tx_data_size);
}

void Robot_Communication_Callback(void)
{
	robot_communication.rx_data_size = sizeof(robot_communication.Robot_Rx_Data);
	robot_communication.tx_data_size = sizeof(robot_communication.Robot_Tx_Data);
	HAL_UART_Receive_IT(&ROBOT_COM_UART, &robot_com_receive_buffer, 1);
	switch(robot_com_receive_state)
	{
		case 0:
		{
			if(robot_com_receive_buffer == 0xA5)   //收到
			{
				robot_com_data_cnt = 0; // 
				robot_com_receive_state = 1;
				robot_communication.rx_data[robot_com_data_cnt]=robot_com_receive_buffer;
				robot_com_data_cnt++;
			}
			break;
		}
		case 1:
		{
			if(robot_com_receive_buffer == 0xA5)   //如果再次收到0xA5，重新开始接收
			{
				robot_com_receive_state = 0;
				break;
			}
			robot_communication.rx_data[robot_com_data_cnt]=robot_com_receive_buffer;
			robot_com_data_cnt++;
			if(robot_com_data_cnt==robot_communication.rx_data_size)  //收满
				robot_com_receive_state = 2;
			break;
		}
	}
	if(robot_com_receive_state == 2)
	{
		if(verify_CRC16_check_sum((uint8_t *)&robot_communication.rx_data, robot_communication.rx_data_size))
		{
		  memcpy(&robot_communication.Robot_Rx_Data, robot_communication.rx_data, robot_communication.rx_data_size); //传出接收到的数据
//		  robot_data_send(&robot_communication);
//		  vision_get_data();
		}
		robot_com_receive_state = 0;
	}
}

void robot_com_task_init(void)
{
	robot_com_receive_buffer = 0;
	robot_com_data_cnt = 0;
	robot_com_receive_state = 0;
	
	robot_communication.Robot_Rx_Data.order1 = 123;
	robot_communication.Robot_Rx_Data.order2 = 123;
	
//	memset(&robot_communication, 0, sizeof(robot_communication));
	
	HAL_UART_Receive_IT(&ROBOT_COM_UART, &robot_com_receive_buffer, 1);
}

void Robot_Data_get(chassis_state_t *chassis_state)
{
	robot_communication.Robot_Tx_Data.x = chassis_state->world_x;
	robot_communication.Robot_Tx_Data.y = chassis_state->world_y;
	robot_communication.Robot_Tx_Data.z = chassis_state->world_z;
	
	robot_communication.Robot_Tx_Data.vx = chassis_state->vx;
	robot_communication.Robot_Tx_Data.vy = chassis_state->vy;
	robot_communication.Robot_Tx_Data.wz = chassis_state->wz;
}



const robot_communication_t* get_robot_com_data_point(void)
{
	return &robot_communication;
}

