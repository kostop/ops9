#include "encoder.h"
#include "tim.h"

#define ENCODER_PULSE			1024.0f
#define ENCODER_TASK_TIME_OUT	0.001f		//s


encoder_para_t encoder_para[2];

void Encoder_init(void)
{
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
}

void Encoder_task(void)
{
	encoder_para[0].dirt = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)*2-1;
	encoder_para[0].count = __HAL_TIM_GetCounter(&htim1)*encoder_para[0].dirt;
	__HAL_TIM_SetCounter(&htim1, 0);
	encoder_para[0].rate = (float)encoder_para[0].count/ENCODER_PULSE/ENCODER_TASK_TIME_OUT*2*PI;
	encoder_para[0].total_count += encoder_para[0].count;
	
	encoder_para[1].dirt = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)*2-1;
	encoder_para[1].count = __HAL_TIM_GetCounter(&htim2)*encoder_para[1].dirt;
	__HAL_TIM_SetCounter(&htim2, 0);
	encoder_para[1].rate = (float)encoder_para[1].count/ENCODER_PULSE/ENCODER_TASK_TIME_OUT*2*PI;
	encoder_para[1].total_count += encoder_para[1].count;
}

const encoder_para_t *get_encoder_para_point(uint8_t num)
{
	return &encoder_para[num];
}

