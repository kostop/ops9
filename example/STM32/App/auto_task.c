#include "auto_task.h"
#include "vision_task.h"

auto_task_para_t auto_task_para;
geometry_twist_pos_t target_pos;

//地图位置长度
#define HALF_START		0.15f
#define HALF_ROAD		0.225f
#define OUT_LINE		0.08f
#define HALF_CENTER_Y	0.635f

#define Y1_POS			HALF_ROAD-HALF_START+OUT_LINE
#define Y2_POS			Y1_POS+HALF_CENTER_Y+HALF_ROAD
#define Y3_POS			Y2_POS+HALF_CENTER_Y+HALF_ROAD
#define X1_POS			
#define X2_POS			1.2f-HALF_START
#define X3_POS			X2_POS+0.825f

#define QR_POS_X		0.8f-HALF_START
#define QR_POS_Y		Y1_POS
#define PET_POS_X		1.6f-HALF_START
#define PET_POS_Y		Y1_POS
#define GROUND_SIGN_DIS	0.15f

//云台高度定义
#define CALI_HIGHT			0.14f			//校准、安全高度
#define MAX_HIGHT			HIGHT_MAX
#define GROUND_HIGHT		0.00000f
#define SAVE_HIGHT			0.125f
#define MATE_HIGHT			METERIAL_HIGHT

#define AUTO_TASK_TIME_OUT	30

static void auto_task_chassis_move_action(fp32 x, fp32 y, fp32 z);


static void auto_task_init(auto_task_para_t *auto_task_init_point);
static void auto_task_feed_back(auto_task_para_t *auto_task_feed_back_point);
static void auto_task_set_control(auto_task_para_t *auto_task_type_count_point);

static uint16_t count_init=0;

void Auto_task(void const * argument)
{
	auto_task_init(&auto_task_para);
	while(1)
	{
		auto_task_feed_back(&auto_task_para);
		auto_task_set_control(&auto_task_para);
		vTaskDelay(AUTO_TASK_TIME_OUT);
	}
}


static void auto_task_init(auto_task_para_t *auto_task_init_point)
{
	auto_task_init_point->count = START_COUNT_NUM;
	
//	auto_task_chassis_move_action(QR_POS_X, QR_POS_Y, 0);
//	auto_task_chassis_move_action(PET_POS_X, PET_POS_Y, 0);
	
//	auto_task_chassis_move_action(0, 0, 0);


	auto_task_init_point->auto_task_chassis_point = get_chassis_para_point();
	auto_task_para.arm_attitude_return.hight = MAX_HIGHT;
	auto_task_init_point->get_data_flag = 1;
	auto_task_init_point->ground_sign_para.ground_sign_num = 2;
	auto_task_init_point->count_num = count_init+1;
}

static void auto_task_feed_back(auto_task_para_t *auto_task_feed_back_point)
{
	auto_task_feed_back_point->now_task_type = auto_task_feed_back_point->auto_task_return[auto_task_feed_back_point->count].task_type;
//	get_order(auto_task_feed_back_point->data);
	auto_task_feed_back_point->chassis_pos = auto_task_feed_back_point->auto_task_chassis_point->chassis_pos;
}

//自动任务自动累加
void auto_count(uint32_t *count, uint32_t count_num)
{
	if(*count<count_num)
		*count += 1;
}

//循环任务自动累加
uint8_t auto_count_order(uint8_t data[3], uint8_t *count)
{
	uint8_t color;
	if(*count >= 3)
		*count = 0;
	color = data[*count];
	*count += 1;
	return color;
}

static void auto_task_set_control(auto_task_para_t *auto_task_type_count_point)
{
	if(get_chassis_state(2)&&auto_task_type_count_point->get_data_flag==0)
	{
		switch(auto_task_type_count_point->now_task_type)
		{
			case TASK_TYPE_CHASSIS:
				auto_task_type_count_point->chassis_pos_return = auto_task_type_count_point->auto_task_return[auto_task_type_count_point->count].chassis_pos;
//				auto_task_type_count_point->chassis_pos_return.x += auto_task_type_count_point->chassis_pos_cali.x;
//				auto_task_type_count_point->chassis_pos_return.y += auto_task_type_count_point->chassis_pos_cali.y;
				auto_task_para.chassis_pos_cali_temp.x = 0;
				auto_task_para.chassis_pos_cali_temp.y = 0;
				auto_task_para.chassis_pos_ground_temp.x = 0;
				auto_task_para.chassis_pos_ground_temp.y = 0;
				break;
			case TASK_TYPE_GIMBAL:
				auto_task_type_count_point->arm_attitude_return = auto_task_type_count_point->gimbal_set_pos[auto_task_type_count_point->auto_task_return[auto_task_type_count_point->count].gimbal_type];
				break;
//			case TASK_TYPE_CALIBRATE:
//				auto_task_type_count_point->chassis_pos_cali = vision_cali_pos(TYPE_CIRCLE);
//				break;
//			case TASK_TYPE_CALIBRATE_TAMP:
//				auto_task_type_count_point->chassis_pos_cali_temp = vision_cali_pos(TYPE_METERIAL);
//				break;
//			case TASK_TYPE_VISION_WAIT:
//				while(vision_wait_meterial(auto_task_type_count_point->turntable_para.turntable_count, AUTO_TASK_TIME_OUT)==0);
//				break;
			case TASK_TYPE_TRRNTABLE:
				auto_task_type_count_point->turntable_para.turntable_num = auto_count_order(auto_task_type_count_point->data, &auto_task_type_count_point->turntable_para.turntable_count);
				break;
			case TASK_TYPE_CHASSIS_SAVE:
				auto_task_type_count_point->chassis_pos_save = auto_task_type_count_point->chassis_pos_return;
				break;
			case TASK_TYPE_GROUND_SIGN:
				auto_task_type_count_point->ground_sign_para.ground_sign_num = auto_count_order(auto_task_type_count_point->data, &auto_task_type_count_point->ground_sign_para.ground_sign_count);
				fp32 cos_angle = cos(auto_task_type_count_point->chassis_pos.z);
				fp32 sin_angle = sin(auto_task_type_count_point->chassis_pos.z);
				auto_task_type_count_point->chassis_pos_ground_temp.x = cos_angle*(auto_task_type_count_point->ground_sign_para.ground_sign_num-2)*GROUND_SIGN_DIS;
				auto_task_type_count_point->chassis_pos_ground_temp.y = sin_angle*(auto_task_type_count_point->ground_sign_para.ground_sign_num-2)*GROUND_SIGN_DIS;
				break;
		}
		auto_task_type_count_point->get_data_flag = 1;
		auto_count(&auto_task_type_count_point->count, auto_task_type_count_point->count_num);
	}
}

//返回底盘自动任务回传坐标
uint8_t get_auto_task_chassis_pos(geometry_twist_pos_t *get_chassis_pos_point)
{
	uint8_t now_task_type = auto_task_para.now_task_type;
	if(auto_task_para.now_task_type == TASK_TYPE_GROUND_SIGN||auto_task_para.now_task_type == TASK_TYPE_CALIBRATE||auto_task_para.now_task_type == TASK_TYPE_CALIBRATE_TAMP)
	{
		auto_task_para.chassis_pos_return.x = auto_task_para.chassis_pos_save.x + auto_task_para.chassis_pos_ground_temp.x+auto_task_para.chassis_pos_cali_temp.x;
		auto_task_para.chassis_pos_return.y = auto_task_para.chassis_pos_save.y + auto_task_para.chassis_pos_ground_temp.y+auto_task_para.chassis_pos_cali_temp.y;
	}

	*get_chassis_pos_point = auto_task_para.chassis_pos_return;
	auto_task_para.get_data_flag = 0;
	return now_task_type;
}

//返回云台自动任务回传姿态
uint8_t get_auto_task_gimbal_pos(arm_attitude_t *get_gimbal_pos_point)
{
	*get_gimbal_pos_point = auto_task_para.arm_attitude_return;
	auto_task_para.get_data_flag = 0;
	return auto_task_para.now_task_type;
}
//返回转盘自动任务位置
uint8_t get_auto_task_turntable_num(uint8_t *get_turntable_num_point)
{
	*get_turntable_num_point = auto_task_para.turntable_para.turntable_num;
	return auto_task_para.now_task_type;
}

uint8_t get_auto_task_cali(cam_pos_t *get_cali_point)
{
	*get_cali_point = auto_task_para.chassis_pos_cali;
	auto_task_para.chassis_pos_cali.x = 0;
	auto_task_para.chassis_pos_cali.y=0;
	return auto_task_para.now_task_type;
}

//底盘自动动作任务
static void auto_task_chassis_move_action(fp32 x, fp32 y, fp32 z)
{
	auto_task_para.auto_task_return[count_init].chassis_pos.x	= x;
	auto_task_para.auto_task_return[count_init].chassis_pos.y	= y;
	auto_task_para.auto_task_return[count_init].chassis_pos.z	= z;
	auto_task_para.auto_task_return[count_init].task_type		= TASK_TYPE_CHASSIS;
	count_init+=1;
}
