#ifndef _INS_TASK_H
#define _INS_TASK_H

#include "struct_typedef.h"
#include <string.h>
#include "cmsis_os.h"
#include "robot_kinematics.h"
#include "usart.h"
#include "head_file.h"

#define PI			3.1415926535897932384626433832795f

typedef struct
{
	fp32 yaw_start;
	geometry_twist_attitude_t	INS_attitude;
	geometry_twist_attitude_t	INS_attitude_before;
}INS_para_t;


void INS_task(void const * argument);
void INS_init(void);
void INS_Communication_Callback(void);
void CmdProcess(void);

const INS_para_t *get_INS_point(void);

#endif
