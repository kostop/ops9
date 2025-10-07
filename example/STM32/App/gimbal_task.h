#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "head_file.h"
#include "remote_control.h"

#define HIGHT_MAX			0.170f			//m
#define GIMBAL_CAM_HIGHT	0.182f
#define METERIAL_HIGHT		0.073f			//m	
#define SPIN_START_ANGLE			12.0f
#define PAT_HIGHT			0.090f

typedef struct
{
	fp32 motor_pos;
	fp32 motor_angle;
	fp32 motor_rate;
}gimbal_motor_para_t;

typedef struct
{
	const RC_ctrl_t *gimbal_remote_point;
	chassis_mode_e 	gimbal_mode;
	
	pid_type_def	gimbal_angle_pid;
	
	arm_attitude_t  gimbal_set_pos;
	arm_attitude_t	gimbal_real_pos;
	
	uint8_t turntable_num;			//1 red		2 green		3 blue
	
//	bool_t get_date_state;
	bool_t get_angle_state;
	bool_t get_hight_state;
	bool_t get_hand_state;
	
	gimbal_motor_para_t gimbal_motor[TOTAL_GIMBLE_NUM];
	fp32 motor_rate_set[TOTAL_GIMBLE_NUM];
}gimbal_para_t;

fp32 get_gimbal_cam_hight(void);
fp32 get_meterial_cam_hight(void);
bool_t get_gimbal_state(uint8_t tag);
void Gimbal_task(void const * argument);

#endif

