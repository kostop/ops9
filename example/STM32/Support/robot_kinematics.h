#ifndef _ROBOT_KINEMATICS_H
#define _ROBOT_KINEMATICS_H

#include "struct_typedef.h"
#include <string.h>

typedef struct
{
	fp32 x;
	fp32 y;
	fp32 z;
}geometry_twist_pos_t;


typedef struct
{
	fp32 vx;
	fp32 vy;
	fp32 wz;
}geometry_twist_line_t;

typedef struct
{
	fp32 roll;
	fp32 pitch;
	fp32 yaw;
}geometry_twist_attitude_t;

typedef struct
{
	fp32 angle;
	fp32 hight;
	bool_t hand;
}arm_attitude_t;


typedef struct
{
	fp32 x;
	fp32 y;
}cam_pos_t;

#endif
