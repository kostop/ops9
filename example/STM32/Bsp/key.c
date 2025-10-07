#include "key.h"

#define START_KEY_PORT	GPIOF
#define START_KEY_PIN	GPIO_PIN_1
#define GIMBAL_CALI_PORT	GPIOE
#define GIMBAL_CALI_PIN		GPIO_PIN_5

chassis_mode_e start_key_state = CHASSIS_NORMAL_MOVE;
uint8_t gimbal_cali_state = 0;

void KEY_Init(void)
{
	gimbal_cali_state = !HAL_GPIO_ReadPin(GIMBAL_CALI_PORT, GIMBAL_CALI_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case START_KEY_PIN:
			if(HAL_GPIO_ReadPin(START_KEY_PORT, START_KEY_PIN)==0)
			{
				while(HAL_GPIO_ReadPin(START_KEY_PORT, START_KEY_PIN)==0)
				{
//					if(start_key_state == CHASSIS_POSITION_MOVE)
//						start_key_state = CHASSIS_NORMAL_MOVE;
//					else
						start_key_state = CHASSIS_POSITION_MOVE;
				}
			}
			break;
		case GIMBAL_CALI_PIN:
			if(HAL_GPIO_ReadPin(GIMBAL_CALI_PORT, GIMBAL_CALI_PIN)==0)
			{
				gimbal_cali_state = 1;
			}
			break;
	}
}

chassis_mode_e get_start_key_state(void)
{
	return start_key_state;
}

uint8_t get_gimbal_cali_key_state(void)
{
	return gimbal_cali_state;
}
