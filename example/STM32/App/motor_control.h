#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#include "head_file.h"


#define MOTOR_MAX_RATE		40.0f
#define Motor_Vel_Ask(addr)		Emm_V5_Read_Sys_Params(addr, S_VEL)
#define Motor_Pos_Ask(addr)		Emm_V5_Read_Sys_Params(addr, S_CPOS)
#define Motor_Stop(addr)		Emm_V5_Stop_Now(addr, 1)
#define Motor_Zero(addr)		Emm_V5_Reset_CurPos_To_Zero(addr);
#define Motor_Mode(addr, mode)	Emm_V5_Modify_Ctrl_Mode(addr, 0, mode);
#define Motor_Rate_Control(addr, dir, vel, acc)	Emm_V5_Vel_Control(addr, dir, vel, acc, 0);
//#define Motor_Pos_Control(addr, dir, vel, acc)	Emm_V5_Pos_Control(1, 0, 3000, 0, 1000, 1, 0);

//#define Motor_Vel_Ask(motor_id)		ZDT_X42_V2_Read_Sys_Params(motor_id, S_VEL)
//#define Motor_Pos_Ask(motor_id)		ZDT_X42_V2_Read_Sys_Params(motor_id, S_CPOS)
//#define Motor_Stop(motor_id)		motor_rate_total_control(motor_id,0);
//#define Motor_Zero(motor_id)		ZDT_X42_V2_Reset_CurPos_To_Zero(motor_id);
//#define Motor_Mode(motor_id, mode)	ZDT_X42_V2_Modify_Ctrl_Mode(motor_id, 0, mode);
//#define Motor_Rate_Control(motor_id, dir, vel)	ZDT_X42_V2_Velocity_Control(motor_id, dir, M_ACC, vel, 0);
 
#define	Read_Error			0
#define ANGLE_GET			0
#define RATE_GET			1
#define	ANGLE_GET_FLAG		1
#define	RATE_GET_FLAG		2



typedef struct
{
	pid_type_def	chassis_motor_rate_pid;
	pid_type_def	chassis_motor_angle_pid;
	pid_type_def	gimbal_motor_angle_pid;
	
	fp32 			Rate;		//n/s
	fp32			Angle;
	fp32			Angle_set;
	fp32 			angle_last_set;
	fp32			Rate_set;
	uint8_t 		Acc_set;
}motor_para_t;

extern uint8_t motor_init_flag;

uint8_t motor_data_flash(void);
void Motor_Task(void const * argument);
void chassis_rate_send(fp32 motor_rate[TOTAL_CHASSIS_NUM], uint8_t motor_acc);
void gimbal_rate_send(fp32 motor_rate[TOTAL_GIMBLE_NUM]);
//void Motor_Pid_Count(void);
void motor_rate_total_control(uint8_t motor_id, fp32 rate, fp32 acc);
fp32 Motor_Data_Get(uint8_t i, uint8_t data_type);

#endif
