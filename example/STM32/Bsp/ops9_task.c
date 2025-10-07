#include "ops9_task.h"
#include "usart.h"
#include "crc8_crc16.h"

ops9_para_t ops9_para;

#define OPS9_RX_DATA_LEN	14
#define OPS9_TX_DATA_LEN	2

uint8_t ops9_transmit_data[OPS9_TX_DATA_LEN];
uint8_t ops9_receive_data[OPS9_RX_DATA_LEN];


void OPS9_init(void)
{
	HAL_Delay(500);
	ops9_transmit_data[0] = 0xC5;
	ops9_transmit_data[1] = 0x22;
	HAL_UART_Transmit(&huart2, ops9_transmit_data, OPS9_TX_DATA_LEN, 50);
	HAL_Delay(20);
	ops9_transmit_data[0] = 0xC5;
	ops9_transmit_data[1] = 0x30;
	HAL_UART_Transmit(&huart2, ops9_transmit_data, OPS9_TX_DATA_LEN, 50);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ops9_receive_data, OPS9_RX_DATA_LEN);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART2)
	{
		if(verify_CRC8_check_sum(ops9_receive_data, OPS9_RX_DATA_LEN))
		{
			memcpy(&ops9_para.OPS9_Rx_Data, &ops9_receive_data, OPS9_RX_DATA_LEN);
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ops9_receive_data, OPS9_RX_DATA_LEN);
	}
}

ops9_para_t *get_ops9_data_point(void)
{
	return &ops9_para;
}
