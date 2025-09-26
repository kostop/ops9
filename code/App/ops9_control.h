#ifndef _CONTROL_H
#define _CONTROL_H

#include "imu.h"
#include "encoder.h"


typedef struct
{
	float len;
	float angle;
	float rate;
	float speed;
}wheel_para_t;

typedef struct
{
	float x;
	float y;
	float z;
}chassis_geometry_t;


typedef struct
{
	const IMU_para_t 		*control_imu_point;
	const encoder_para_t 	*control_encoder_point[2];
	
	wheel_para_t	wheel_para[2];
	chassis_geometry_t	chassis_pos;
	chassis_geometry_t	chassis_line;
}control_para_t;

void Control_init(uint16_t Task_time_out);
void Control_task(uint8_t dirt);
const control_para_t *get_control_para_point(void);

#endif
