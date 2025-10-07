#ifndef _VISION_TASK_H
#define _VISION_TASK_H

#include "stm32f1xx_hal.h"
#include "struct_typedef.h"
#include "robot_communication.h"
#include "chassis_task.h"
#include "serial_screen.h"
#include "robot_kinematics.h"

#define TYPE_CIRCLE		1
#define TYPE_METERIAL	2

#define RED				0
#define GREEN			1
#define BLUE			2


typedef struct
{
	const robot_communication_t *vision_robot_communication_point;
	const chassis_para_t 		*vision_chassis_point;
	
	cam_pos_t meterial_circle_info[2];	//目标物料或者地标的信息
	uint8_t data[6];
	uint8_t type;
	uint8_t color;
	
	fp32 vx;
	fp32 vy;
	
	cam_pos_t	target_pos;
	geometry_twist_pos_t chassis_pos;
	
}vision_para_t;

void vision_init(void);
uint8_t vision_get_data(void);
cam_pos_t vision_cali_pos(uint8_t cali_type);
uint8_t vision_wait_meterial(uint8_t count, uint16_t task_time);
void get_order(uint8_t *data);

#endif
