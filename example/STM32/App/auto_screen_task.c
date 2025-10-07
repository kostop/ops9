#include "auto_screen_task.h"
#include "serial_screen.h"

#define CAR_ID						2				//0 1 2

auto_screen_para_t auto_screen_para;

static void auto_screen_task_chassis_move_action(fp32 x, fp32 y, fp32 z);

static uint16_t count_init=0;

void auto_screen_task_init(void)
{
	auto_screen_para.count = START_COUNT_NUM;
	
	auto_screen_task_chassis_move_action(0.5f, 0.0f, 0);
	auto_screen_task_chassis_move_action(0.0f, 0.5f, 0);
	auto_screen_task_chassis_move_action(1.5f, 0.0f, 0);
	auto_screen_task_chassis_move_action(0.0f, 1.5f, 0);
	auto_screen_task_chassis_move_action(3.0f, 0.0f, 0);
	auto_screen_task_chassis_move_action(0.0f, 3.0f, 0);


	auto_screen_para.auto_task_chassis_point = get_chassis_para_point();
	auto_screen_para.count_num = count_init+1;
}

static void auto_screen_task_chassis_move_action(fp32 x, fp32 y, fp32 z)
{
	auto_screen_para.auto_task_return[count_init].chassis_pos.x	= x;
	auto_screen_para.auto_task_return[count_init].chassis_pos.y	= y;
	auto_screen_para.auto_task_return[count_init].chassis_pos.z	= z;
	auto_screen_para.auto_task_return[count_init].task_type		= TASK_TYPE_CHASSIS;
	count_init+=1;
}

uint8_t get_auto_screen_task_chassis_pos(geometry_twist_pos_t *get_chassis_pos_point)
{
	uint8_t now_task_type = auto_screen_para.now_task_type;
	auto_screen_para.chassis_pos_return = auto_screen_para.auto_task_return[CAR_ID+auto_screen_para.car_id[CAR_ID]+CAR_ID].chassis_pos;
	
	*get_chassis_pos_point = auto_screen_para.chassis_pos_return;
	return now_task_type;
}

void get_car_id(uint8_t *car_id)
{
	for(int i=0; i<3; i++)
		auto_screen_para.car_id[i] = car_id[i];
}
