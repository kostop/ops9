#include "imu.h"
#include "usart.h"
#include "delay.h"

#define IMU_DATA_LEN	11

IMU_RX_Data_t	IMU_RX_Data;
IMU_para_t		IMU_para;

uint8_t imu_buffer;
uint8_t imu_receive_data[IMU_DATA_LEN];
uint8_t imu_data_cnt;
uint8_t imu_receive_state;
uint8_t imu_data_sum;

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_receive_data, IMU_DATA_LEN); 
  
  /* USER CODE END USART1_IRQn 1 */
}

void IMU_dma_init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_receive_data, IMU_DATA_LEN);
}

void IMU_zero_angle(void)
{
	uint8_t reset_z_axis[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
		delay_ms(20);
	while(IMU_para.yaw != 0)
	{
		HAL_UART_Transmit(&huart1, reset_z_axis, sizeof(reset_z_axis), 10);
		delay_ms(20);
	}
}



void IMU_check_task(void)
{
	if(IMU_para.get_data_flag == 0)
	{
		IMU_dma_init();
	}
}

void IMU_init(void)
{
	IMU_dma_init();
	IMU_zero_angle();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) 
    {
		if(imu_receive_data[1] == 0x53)
		{
			memcpy(&IMU_RX_Data, imu_receive_data, Size);
			IMU_para.yaw = (float)IMU_RX_Data.yaw/32768*PI;
			IMU_para.get_data_flag = 1;
		}
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_receive_data, IMU_DATA_LEN); 
	}
}


const IMU_para_t *get_imu_para_point(void)
{
	return &IMU_para;
}


