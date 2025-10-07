#include "steering.h"

#define PWM_OUT_CCR				2000

#define YAW_MAX_ANGLE			540.0f
#define HAND_MAX_ANGLE			180.0f
#define TURNTABLE_MAX_ANGLE		615.0f

#define HAND_START_ANGLE		0.0f
#define HAND_GET_ANGLE			25.0f

#define YAW_START_ANGLE		45.0f
#define YAW_PUT_ANGLE		132.0f
#define YAW_INIT_ANGLE		YAW_START_ANGLE+180.0f

#define TURNTABLE_START_ANGLE	133.0f

void steering_init(void)
{
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
	averageFilter_init(YAW_STEERING, YAW_INIT_ANGLE);
	averageFilter_init(HAND_STEERING, HAND_START_ANGLE);
}

bool_t steering_controller(uint8_t num, fp32 state)
{
	fp32 angle, before_fliter_angle;
	uint16_t compare;
	switch(num)
	{
		case YAW_STEERING:
			if(state == 3)
				angle = YAW_INIT_ANGLE;
			else
				angle = YAW_START_ANGLE+YAW_PUT_ANGLE*state;
			before_fliter_angle = averageFilter(YAW_STEERING, angle);
			compare = 250+(before_fliter_angle/YAW_MAX_ANGLE) * PWM_OUT_CCR;
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, compare);
			break;
		case HAND_STEERING:
			angle = HAND_START_ANGLE+state*HAND_GET_ANGLE/2;
			before_fliter_angle = averageFilter(HAND_STEERING, angle);
			compare = 500+(before_fliter_angle/HAND_MAX_ANGLE) * PWM_OUT_CCR;
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, compare);
			break;
		case TURNTABLE_STEERING:
			angle = TURNTABLE_START_ANGLE+(state-2)*TURNTABLE_START_ANGLE;
			compare = 185+(angle/TURNTABLE_MAX_ANGLE) * PWM_OUT_CCR;
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, compare);
			break;
	}
	if(fabs(before_fliter_angle - angle)<GIMBAL_THRESHOLD_ANGLE)
		return 1;
	else
		return 0;
}
