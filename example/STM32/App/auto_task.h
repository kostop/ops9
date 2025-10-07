#ifndef _AUTO_TASK_H
#define _AUTO_TASK_H

#include "cmsis_os.h"
#include "struct_typedef.h"
#include "robot_kinematics.h"
#include "chassis_task.h"

#define START_COUNT_NUM			0		//���ÿ�ʼ������ʼ��
//#define TASK_TYPE_CHASSIS		1
//#define TASK_TYPE_GIMBAL		2
//#define TASK_TYPE_CALIBRATE		3		//�ر�У׼����
//#define TASK_TYPE_CALIBRATE_TAMP	4	//����У׼����
//#define TASK_TYPE_TRRNTABLE		5		//ת������
//#define TASK_TYPE_GROUND_SIGN	6		//�ر�����
//#define TASK_TYPE_CHASSIS_SAVE		7	//���λ����Ϣ
//#define TASK_TYPE_VISION_WAIT		8

typedef enum
{
	TASK_TYPE_CHASSIS = 1,
	TASK_TYPE_GIMBAL,
	TASK_TYPE_CALIBRATE,			//�ر�У׼����
	TASK_TYPE_CALIBRATE_TAMP,       //����У׼����
	TASK_TYPE_TRRNTABLE,           	//ת������
	TASK_TYPE_GROUND_SIGN,          //�ر�����
	TASK_TYPE_CHASSIS_SAVE,         //���λ����Ϣ
	TASK_TYPE_VISION_WAIT,
}auto_task_type_e;



#define TASK_TOTAL_NUM				1000	//�����С
#define GIMBAL_TASK_TOTAL_NUM		14		//����λ�ã�ʶ�𡢴�����ϡ�׼����ƽ�ء�ת�̣���׼�������ץȡ���ϣ���׼���������ϣ���
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
	//ת������CNT
	uint8_t 					turntable_count;
}turntable_para_t;

typedef struct
{
	uint8_t						ground_sign_num;
	//ת������CNT
	uint8_t 					ground_sign_count;
}ground_sign_para_t;

typedef struct
{
	const chassis_para_t 		*auto_task_chassis_point;
	geometry_twist_pos_t 		chassis_pos;
	
	uint32_t count;				//���ڵ�������
	uint32_t count_num;			//�������е���������
	arm_attitude_t 				gimbal_set_pos[GIMBAL_TASK_TOTAL_NUM];		//��̨�Զ�λ��
	auto_task_return_t 			auto_task_return[TASK_TOTAL_NUM];			//������������
	
	bool_t						get_data_flag;								//��̨�͵��̹��õ�һ����־λ�������ж��ǲ��ǻ��������
	uint8_t						now_task_type;								//���ڵ�������ʲô
	//���ص��Զ�����λ����Ϣ
	geometry_twist_pos_t		chassis_pos_return;
	arm_attitude_t				arm_attitude_return;
	turntable_para_t			turntable_para;
	ground_sign_para_t			ground_sign_para;
	//У׼������Ϣ
	cam_pos_t					chassis_pos_cali;
	cam_pos_t					chassis_pos_cali_temp;
	
	//�ر������������
	cam_pos_t					chassis_pos_ground_temp;
	
	//���������Ϣ
	geometry_twist_pos_t		chassis_pos_save;
	
	//����˳����Ϣ
	uint8_t 					data[6];
	
}auto_task_para_t;


void Auto_task(void const * argument);
uint8_t get_auto_task_cali(cam_pos_t *get_cali_point);
uint8_t get_auto_task_chassis_pos(geometry_twist_pos_t *get_chassis_pos_point);
uint8_t get_auto_task_gimbal_pos(arm_attitude_t *get_gimbal_pos_point);
uint8_t get_auto_task_turntable_num(uint8_t *get_turntable_num_point);

#endif
