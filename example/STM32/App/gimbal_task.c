#include "gimbal_task.h"
#include "chassis_task.h"
#include "vision_task.h"
#include "auto_task.h"

gimbal_para_t gimbal_para;
arm_attitude_t gimbal_zero_pos = {0};

#define GIMBAL_ANGLE_KP				800.0f
#define GIMBAL_ANGLE_KI				0.0f
#define GIMBAL_ANGLE_KD				0.0f
#define GIMBAL_ANGLE_MAX_OUT		50.0f
#define GIMBAL_ANGLE_MAX_IOUT		50.0f
 
#define PULLEY_RADIUS		0.006f		//m

#define GIMBAL_TASK_TIME_OUT			5

static void gimbal_init(gimbal_para_t *gimbal_init_para);
static void gimbal_feed_back(gimbal_para_t *gimbal_feed_back_para);
static void gimbal_set_mode(gimbal_para_t *gimbal_set_mode_point);
static void gimbal_set_control(gimbal_para_t *gimbal_set_control_para);
static void gimbal_control_loop(gimbal_para_t *gimbal_control_loop_para);
static bool_t get_gimbal_hight_state(void);

void Gimbal_task(void const * argument)
{
	gimbal_init(&gimbal_para);
	while(1)
	{
		gimbal_feed_back(&gimbal_para);
		gimbal_set_mode(&gimbal_para);
		gimbal_set_control(&gimbal_para);
		gimbal_control_loop(&gimbal_para);
		
		
		//发送控制量
		//yaw轴舵机
		if(get_gimbal_state(1))
			gimbal_para.get_angle_state = steering_controller(YAW_STEERING, gimbal_para.gimbal_set_pos.angle);
		//夹爪舵机
		gimbal_para.get_hand_state = steering_controller(HAND_STEERING, gimbal_para.gimbal_set_pos.hand);
		//转盘舵机
		steering_controller(TURNTABLE_STEERING, gimbal_para.turntable_num);
		//抬升电机
		gimbal_para.get_hight_state = get_gimbal_hight_state();
		vTaskDelay(GIMBAL_TASK_TIME_OUT);
	}
}

static void gimbal_init(gimbal_para_t *gimbal_init_para)
{
	gimbal_init_para->gimbal_remote_point = get_remote_control_point();
	
//	gimbal_init_para->gimbal_set_pos.angle = 120;					//averageFilter(((float)(gimbal_para.gimbal_remote_point->rc.ch[4]+660)/1320.0f)*180.0f);
	gimbal_init_para->gimbal_set_pos.hight = HIGHT_MAX;
	gimbal_init_para->turntable_num = 1;
	gimbal_init_para->gimbal_set_pos.angle = 3;
	float gimbal_angle_pid[] = {GIMBAL_ANGLE_KP, GIMBAL_ANGLE_KI, GIMBAL_ANGLE_KD};
	PID_init(&gimbal_init_para->gimbal_angle_pid, PID_POSITION, gimbal_angle_pid, GIMBAL_ANGLE_MAX_OUT, GIMBAL_ANGLE_MAX_IOUT);
}


static void gimbal_feed_back(gimbal_para_t *gimbal_feed_back_para)
{
	for(int i=0; i<TOTAL_GIMBLE_NUM; i++)
	{
		gimbal_feed_back_para->gimbal_motor[i].motor_angle = Motor_Data_Get(i+TOTAL_CHASSIS_NUM, ANGLE_GET);
		gimbal_feed_back_para->gimbal_motor[i].motor_rate = Motor_Data_Get(i+TOTAL_CHASSIS_NUM, RATE_GET);
	}
}

static void gimbal_set_mode(gimbal_para_t *gimbal_set_mode_point)
{
	gimbal_set_mode_point->gimbal_mode = get_chassis_mode();
}

//获得自动任务目标位置
void gimbal_get_auto_task_pos(arm_attitude_t *gimbal_set_pos_point)
{
	arm_attitude_t gimbal_temp_pos;
	uint8_t now_task_type;
	now_task_type = get_auto_task_gimbal_pos(&gimbal_temp_pos);
	if(now_task_type == TASK_TYPE_GIMBAL)
		*gimbal_set_pos_point = gimbal_temp_pos;
}
//获得自动任务转盘位置
void turntalbe_get_auto_task_num(uint8_t *turntable_num)
{
	uint8_t turntable_temp_num;
	uint8_t now_task_type;
	now_task_type = get_auto_task_turntable_num(&turntable_temp_num);
	if(now_task_type == TASK_TYPE_TRRNTABLE)
		*turntable_num = turntable_temp_num;
}

//遥控器控制
fp32 hight_normal_move;
void gimbal_normal_move_get_pos(arm_attitude_t *arm_attitude_point, const RC_ctrl_t *remote_point)
{
	static fp32 hight_normal_move_limit=HIGHT_MAX;
	hight_normal_move_limit += (float)remote_point->rc.ch[1]/200000;
	hight_normal_move_limit = limit_Data(hight_normal_move_limit, HIGHT_MAX, 0);
	if(switch_is_up(remote_point->rc.s[CHASSIS_MODE_CHANNEL_2]))
		hight_normal_move += (float)remote_point->rc.ch[4]/5000000;
	arm_attitude_point->hight = hight_normal_move_limit + hight_normal_move;
	arm_attitude_point->hand = switch_is_mid(remote_point->rc.s[CHASSIS_MODE_CHANNEL_2])*2;
}

static void gimbal_set_control(gimbal_para_t *gimbal_set_control_para)
{
	if(gimbal_set_control_para->gimbal_mode == CHASSIS_NORMAL_MOVE)
		//遥控器控制
		gimbal_normal_move_get_pos(&gimbal_set_control_para->gimbal_set_pos, gimbal_set_control_para->gimbal_remote_point);
	else if(gimbal_set_control_para->gimbal_mode == CHASSIS_POSITION_MOVE)
	{
		//获得自动任务云台位置
		gimbal_get_auto_task_pos(&gimbal_set_control_para->gimbal_set_pos);
		//获得自动任务转盘位置
		turntalbe_get_auto_task_num(&gimbal_set_control_para->turntable_num);
	}
	else if(gimbal_set_control_para->gimbal_mode == CHASSIS_NO_MOVE)
		//云台回位
		gimbal_set_control_para->gimbal_set_pos.hight = HIGHT_MAX+hight_normal_move;
}

//高度逆解算
fp32 gimbal_move_back_decomposition(fp32 angle)
{
	fp32 hight = angle * PULLEY_RADIUS + HIGHT_MAX;
	return hight;
}

static void gimbal_control_loop(gimbal_para_t *gimbal_control_loop_para)
{
	//高度逆解算
	gimbal_control_loop_para->gimbal_real_pos.hight = gimbal_move_back_decomposition(gimbal_control_loop_para->gimbal_motor[0].motor_angle);
	gimbal_control_loop_para->motor_rate_set[0] = PID_calc(&gimbal_control_loop_para->gimbal_angle_pid, gimbal_control_loop_para->gimbal_real_pos.hight, gimbal_control_loop_para->gimbal_set_pos.hight);

}

//云台高度到位
static bool_t get_gimbal_hight_state(void)
{
	gimbal_rate_send(gimbal_para.motor_rate_set);
	if(fabs(gimbal_para.gimbal_set_pos.hight - gimbal_para.gimbal_real_pos.hight)<GIMBAL_THRESHOLD_HIGHT)
		return 1;
	else
		return 0;
}

//获得相机高度
fp32 get_gimbal_cam_hight(void)
{
	return gimbal_para.gimbal_real_pos.hight+GIMBAL_CAM_HIGHT;
}

//获得物料相对相机的高度
fp32 get_meterial_cam_hight(void)
{
	return gimbal_para.gimbal_real_pos.hight+GIMBAL_CAM_HIGHT-METERIAL_HIGHT-PAT_HIGHT;
}


//0		角度
//1		高度
//2		所有
//3		其一
bool_t get_gimbal_state(uint8_t tag)
{
	switch(tag)
	{
		case 0:
			return  gimbal_para.get_angle_state;
		case 1:	
			return 	gimbal_para.get_hight_state;
		case 2:
			return	gimbal_para.get_angle_state&gimbal_para.get_hight_state&gimbal_para.get_hand_state;
		case 3:
			return 	gimbal_para.get_angle_state|gimbal_para.get_hight_state|gimbal_para.get_hand_state;
	}
	return 0;
}





