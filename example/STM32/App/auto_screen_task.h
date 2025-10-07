#ifndef _AUTO_SCREEN_TASK_H
#define _AUTO_SCREEN_TASK_H

#include "main.h"
#include "auto_task.h"

#define TASK_SCREEN_NUM		6

typedef struct
{
	const chassis_para_t 		*auto_task_chassis_point;
	geometry_twist_pos_t 		chassis_pos;
	
	uint32_t count;				//现在的任务编号
	uint32_t count_num;			//计算现有的任务总数
	auto_task_return_t 			auto_task_return[TASK_SCREEN_NUM];			//所有任务总数
	
	geometry_twist_pos_t		chassis_pos_return;
	uint8_t						now_task_type;								//现在的任务是什么
	
	uint8_t						car_id[3];
}auto_screen_para_t;

void get_car_id(uint8_t *car_id);


void auto_screen_task_init(void);
uint8_t get_auto_screen_task_chassis_pos(geometry_twist_pos_t *get_chassis_pos_point);

#endif
