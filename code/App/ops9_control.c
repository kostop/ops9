#include "ops9_control.h"
#include <math.h>

#define WHEEL_RADIUS			0.029f		//m
uint8_t ops9_direction = 0;

float Control_task_time_out;

control_para_t control_para;

static void feed_back(control_para_t *feed_back_point);
static void control_loop(control_para_t *control_loop_point);

float yaw_add_count(float yaw)
{
	static float yaw_last, yaw_add;
	float yaw_result;
	if(yaw - yaw_last < -PI)
		yaw_add += 2*PI;
	else if(yaw - yaw_last > PI)
		yaw_add -= 2*PI;
	
	yaw_last = yaw;
	yaw_result = yaw + yaw_add;
	return yaw_result;
}

void Control_init(uint16_t Task_time_out)
{
	ops9_direction = 2;
	for(int i=0; i<2; i++)
		control_para.control_encoder_point[i] = get_encoder_para_point(i);
	control_para.control_imu_point = get_imu_para_point();
	Control_task_time_out = 0.001f;
}

void Control_task(uint8_t dirt)
{
//	ops9_direction = dirt;
	feed_back(&control_para);
	control_loop(&control_para);
}

static void feed_back(control_para_t *feed_back_point)
{
	for(int i=0; i<2; i++)
	{
		feed_back_point->wheel_para[i].angle = (float)feed_back_point->control_encoder_point[i]->total_count/1024*2*PI;
		feed_back_point->wheel_para[i].len = feed_back_point->wheel_para[i].angle*WHEEL_RADIUS;
		feed_back_point->wheel_para[i].rate = feed_back_point->control_encoder_point[i]->rate;
		feed_back_point->wheel_para[i].speed = feed_back_point->wheel_para[i].rate*WHEEL_RADIUS;
	}

	feed_back_point->chassis_pos.z = yaw_add_count(feed_back_point->control_imu_point->yaw);
}

static void control_loop(control_para_t *control_loop_point)
{
	switch(ops9_direction)
	{
		case 0:
			control_loop_point->chassis_line.y = control_loop_point->wheel_para[0].speed*Control_task_time_out;
			control_loop_point->chassis_line.x = -control_loop_point->wheel_para[1].speed*Control_task_time_out;
			break;
		case 1:
			control_loop_point->chassis_line.x = control_loop_point->wheel_para[0].speed*Control_task_time_out;
			control_loop_point->chassis_line.y = control_loop_point->wheel_para[1].speed*Control_task_time_out;
			break;
		case 2:
			control_loop_point->chassis_line.y = -control_loop_point->wheel_para[0].speed*Control_task_time_out;
			control_loop_point->chassis_line.x = control_loop_point->wheel_para[1].speed*Control_task_time_out;
			break;
		case 3:
			control_loop_point->chassis_line.x = -control_loop_point->wheel_para[0].speed*Control_task_time_out;
			control_loop_point->chassis_line.y = -control_loop_point->wheel_para[1].speed*Control_task_time_out;
			break;
	}
	float sin_angle = sin(control_loop_point->chassis_pos.z);
	float cos_angle = cos(control_loop_point->chassis_pos.z);
	
	
	control_loop_point->chassis_pos.x += control_loop_point->chassis_line.x * cos_angle - control_loop_point->chassis_line.y * sin_angle;
	control_loop_point->chassis_pos.y += control_loop_point->chassis_line.x * sin_angle + control_loop_point->chassis_line.y * cos_angle;
}


const control_para_t *get_control_para_point(void)
{
	return &control_para;
}
