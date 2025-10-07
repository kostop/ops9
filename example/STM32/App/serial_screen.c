#include "serial_screen.h"
#include "auto_screen_task.h"
#include "chassis_task.h"

serial_screen_t	serial_screen;
uint8_t car_id[3];
fp32 move[2];

uint8_t serial_screen_receive_buffer;
uint8_t serial_screen_data_cnt;
uint8_t serial_screen_receive_state;


void serial_screen_init(void)
{	
	serial_screen_receive_buffer = 0;
	serial_screen_data_cnt = 0;
	serial_screen_receive_state = 0;
	
	
	// 0x74 0x30 0x2e 0x74 0x78 0x74 0x3d 0x22
	serial_screen.Serial_Screen_Tx_Data.hander[0] = 0x74;
	serial_screen.Serial_Screen_Tx_Data.hander[1] = 0x30;
	serial_screen.Serial_Screen_Tx_Data.hander[2] = 0x2e;
	serial_screen.Serial_Screen_Tx_Data.hander[3] = 0x74;
	serial_screen.Serial_Screen_Tx_Data.hander[4] = 0x78;
	serial_screen.Serial_Screen_Tx_Data.hander[5] = 0x74;
	serial_screen.Serial_Screen_Tx_Data.hander[6] = 0x3d;
	serial_screen.Serial_Screen_Tx_Data.hander[7] = 0x22;
	// 0x2b
	serial_screen.Serial_Screen_Tx_Data.sum_buffer = 0x2b;
	// 0x22 0xff 0xff 0xff
	serial_screen.Serial_Screen_Tx_Data.ender[0] = 0x22;
	serial_screen.Serial_Screen_Tx_Data.ender[1] = 0xff;
	serial_screen.Serial_Screen_Tx_Data.ender[2] = 0xff;
	serial_screen.Serial_Screen_Tx_Data.ender[3] = 0xff;
	
	HAL_UART_Receive_IT(&SERIAL_SCREEN, &serial_screen_receive_buffer, 1);
}



//发送、记录数值
void serial_screen_send(uint8_t data[6])
{
	for(int i=0; i<3; i++)
	{
		serial_screen.Serial_Screen_Tx_Data.data1[i] = data[i]+48;
		serial_screen.Serial_Screen_Tx_Data.data2[i] = data[i+3]+48;
	}
	
	
	memcpy(serial_screen.tx_data, &serial_screen.Serial_Screen_Tx_Data, SERIAL_SCREEN_TX_LEN);
	HAL_UART_Transmit_IT(&SERIAL_SCREEN, serial_screen.tx_data, SERIAL_SCREEN_TX_LEN);
}







void Serial_Screen_Communication_Callback(void)
{
	switch(serial_screen_receive_state)
	{
		case 0:
		{
			if(serial_screen_receive_buffer == 0x5d)   //收到
			{
				serial_screen_data_cnt = 0; // 
				serial_screen_receive_state = 1;
				serial_screen.rx_data[serial_screen_data_cnt]=serial_screen_receive_buffer;
				serial_screen_data_cnt++;
			}
			break;
		}
		case 1:
		{
			if(serial_screen_receive_buffer == 0x5d)   //如果再次收到0xA5，重新开始接收
			{
				serial_screen_receive_state = 0;
				break;
			}
			serial_screen.rx_data[serial_screen_data_cnt]=serial_screen_receive_buffer;
			serial_screen_data_cnt++;
			if(serial_screen_data_cnt==SERIAL_SCREEN_RX_LEN)  //收满
				serial_screen_receive_state = 2;
			break;
		}
	}
	if(serial_screen_receive_state == 2)
	{
		if(serial_screen.rx_data[SERIAL_SCREEN_RX_LEN-1] == 0x6d)
		{
			memcpy(&serial_screen.Serial_Screen_Rx_Data, serial_screen.rx_data, SERIAL_SCREEN_RX_LEN); //传出接收到的数据
			switch(serial_screen.Serial_Screen_Rx_Data.type)
			{
				case 0x01:
					car_id[0] = serial_screen.Serial_Screen_Rx_Data.data1;
					car_id[1] = serial_screen.Serial_Screen_Rx_Data.data2;
					car_id[2] = serial_screen.Serial_Screen_Rx_Data.data3;
					get_car_id(car_id);
					break;
				case 0x02:
					move[0] = serial_screen.Serial_Screen_Rx_Data.data1;
					move[1] = serial_screen.Serial_Screen_Rx_Data.data2;
					get_chassis_move_set_para(move);
					break;
			}
		}
	serial_screen_receive_state = 0;
	}
	HAL_UART_Receive_IT(&SERIAL_SCREEN, &serial_screen_receive_buffer, 1);
}



