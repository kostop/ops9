#ifndef _AUTO_TASK_H
#define _AUTO_TASK_H

#include "cmsis_os.h"
#include "struct_typedef.h"
#include "robot_kinematics.h"
#include "chassis_task.h"

#define START_COUNT_NUM			0		//设置开始任务起始点
//#define TASK_TYPE_CHASSIS		1
//#define TASK_TYPE_GIMBAL		2
//#define TASK_TYPE_CALIBRATE		3		//地标校准任务
//#define TASK_TYPE_CALIBRATE_TAMP	4	//物料校准任务
//#define TASK_TYPE_TRRNTABLE		5		//转盘任务
//#define TASK_TYPE_GROUND_SIGN	6		//地标任务
//#define TASK_TYPE_CHASSIS_SAVE		7	//存放位置信息
//#define TASK_TYPE_VISION_WAIT		8

typedef enum
{
	TASK_TYPE_CHASSIS = 1,
	TASK_TYPE_GIMBAL,
	TASK_TYPE_CALIBRATE,			//地标校准任务
	TASK_TYPE_CALIBRATE_TAMP,       //物料校准任务
	TASK_TYPE_TRRNTABLE,           	//转盘任务
	TASK_TYPE_GROUND_SIGN,          //地标任务
	TASK_TYPE_CHASSIS_SAVE,         //存放位置信息
	TASK_TYPE_VISION_WAIT,
}auto_task_type_e;



#define TASK_TOTAL_NUM				1000	//数组大小
#define GIMBAL_TASK_TOTAL_NUM		14		//空闲位置（识别、存放物料、准备（平地、转盘）、准备（存放抓取物料）、准备（物料上））
#define GIMBAL_TASK_HALF			GIMBAL_TASK_TOTAL_NUM/2

typedef struct
{
	geometry_twist_pos_t 	chassis_pos;
	uint8_t 				gimbal_type;
	uint8_t 				task_type;
}auto_task_return_t;


typedef struct
{
	uint8_t						turntable_num;
	//转盘任务CNT
	uint8_t 					turntable_count;
}turntable_para_t;

typedef struct
{
	uint8_t						ground_sign_num;
	//转盘任务CNT
	uint8_t 					ground_sign_count;
}ground_sign_para_t;

typedef struct
{
	const chassis_para_t 		*auto_task_chassis_point;
	geometry_twist_pos_t 		chassis_pos;
	
	uint32_t count;				//现在的任务编号
	uint32_t count_num;			//计算现有的任务总数
	arm_attitude_t 				gimbal_set_pos[GIMBAL_TASK_TOTAL_NUM];		//云台自动位置
	auto_task_return_t 			auto_task_return[TASK_TOTAL_NUM];			//所有任务总数
	
	bool_t						get_data_flag;								//云台和底盘公用的一个标志位，用来判断是不是获得了数据
	uint8_t						now_task_type;								//现在的任务是什么
	//返回的自动任务位置信息
	geometry_twist_pos_t		chassis_pos_return;
	arm_attitude_t				arm_attitude_return;
	turntable_para_t			turntable_para;
	ground_sign_para_t			ground_sign_para;
	//校准任务信息
	cam_pos_t					chassis_pos_cali;
	cam_pos_t					chassis_pos_cali_temp;
	
	//地标放置任务坐标
	cam_pos_t					chassis_pos_ground_temp;
	
	//存放坐标信息
	geometry_twist_pos_t		chassis_pos_save;
	
	//物料顺序信息
	uint8_t 					data[6];
	
}auto_task_para_t;


void Auto_task(void const * argument);
uint8_t get_auto_task_cali(cam_pos_t *get_cali_point);
uint8_t get_auto_task_chassis_pos(geometry_twist_pos_t *get_chassis_pos_point);
uint8_t get_auto_task_gimbal_pos(arm_attitude_t *get_gimbal_pos_point);
uint8_t get_auto_task_turntable_num(uint8_t *get_turntable_num_point);

#endif
