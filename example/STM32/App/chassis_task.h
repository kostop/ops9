#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "head_file.h"
#include "remote_control.h"
#include "INS_task.h"


typedef struct
{
	fp32 motor_len;
	fp32 motor_time_len;
	fp32 motor_last_len;
	fp32 motor_angle;
	fp32 motor_rate;
}chassis_motor_para_t;

typedef struct
{
	fp32 speed_max;
	fp32 speed_min;
	fp32 acc;
}chassis_move_set_para_t;

typedef struct
{
//	all_encoder_para_t *chassis_encoder_para;
//	const INS_para_t	*chassis_INS_point;
//	const RC_ctrl_t 	*chassis_remote_point;
	const ops9_para_t	*chassis_ops9_point;
	
	chassis_mode_e 	chassis_mode;
	bool_t get_pos_state;
	bool_t get_angle_state;
	
	pid_type_def	chassis_pos_pid;
	pid_type_def	chassis_angle_pid;
	
	geometry_twist_pos_t	chassis_pos;		//绝对坐标
	geometry_twist_pos_t	chassis_set_pos;	//目标位置
	geometry_twist_line_t	chassis_line;		//实际底盘速度
	geometry_twist_line_t	chassis_set_line;	//设定底盘速度	
	
	cam_pos_t	chassis_pos_cali;
	
	chassis_motor_para_t chassis_motor[TOTAL_CHASSIS_NUM];
	fp32 motor_rate_set[TOTAL_CHASSIS_NUM];
	uint8_t motor_acc_set;
	fp32 chassis_speed_max;
	
	chassis_move_set_para_t	chassis_move_set_para;
}chassis_para_t;

void Chassis_task(void const * argument);
chassis_mode_e get_chassis_mode(void);
chassis_para_t *get_chassis_para_point(void);
bool_t get_chassis_state(uint8_t tag);
void get_chassis_move_set_para(fp32 *move_para);


#endif
