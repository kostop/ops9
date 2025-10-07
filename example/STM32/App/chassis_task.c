#include "chassis_task.h"
#include "auto_task.h"
#include "auto_screen_task.h"

chassis_para_t chassis_para;
geometry_twist_pos_t chassis_zero_pos = {0};
extern uint8_t motor_init_flag;

#define CHASSIS_POS_KP				3.0f//10
#define CHASSIS_POS_KI				0.0f
#define CHASSIS_POS_KD				0.0f
#define CHASSIS_POS_MAX_OUT			CHASSIS_MAX_SPEED_POS
#define CHASSIS_POS_MAX_IOUT		0.0f

#define CHASSIS_ANGLE_KP			0.9f		//450.0f	编码器
#define CHASSIS_ANGLE_KI			0.0f
#define CHASSIS_ANGLE_KD			0.0f
#define CHASSIS_ANGLE_MAX_OUT		CHASSIS_MAX_SPEED_POS
#define CHASSIS_ANGLE_MAX_IOUT		50.0f


#define FRE_Z_MODE					1			//0	编码器
												//1	陀螺仪
#define SET_Z_MODE					0			//0	移动Z轴速度环
												//1	固定Z轴速度环

#define CHASSIS_TASK_TIMEOUT		4
#define SYSTEM_RESET_TIMEOUT		500


#define CHASSIS_RADIUS				0.21474430328486805953967629161934f				//底盘半径/m
#define WHEEL_RADIUS				0.0385f				//轮子半径/m
#define CHASSIS_W					0.24900f			//轮距
#define CHASSIS_H					0.19800f			//轴距

#define CHASSIS_ACC					0
#define CHASSIS_MAX_SPEED			0.5f		//m/s
#define CHASSIS_MAX_SPEED_POS		0.5f
#define CHASSIS_MIN_SPEED_POS		0.1f
#define CHASSIS_ACC_POS				0.5f

static uint16_t reset_count;

static void chassis_init(chassis_para_t *chassis_para_init);
static void chassis_set_mode(chassis_para_t *chassis_set_mode_para);
static void chassis_set_control(chassis_para_t *chassis_set_control_point);
static void chassis_feed_back(chassis_para_t *chassis_para_feedback);
static void chassis_control_loop(chassis_para_t *chassis_control_loop_para);
static void chassis_state_load(chassis_para_t *chassis_state_load_para);

void Chassis_task(void const * argument)
{
	vTaskDelay(500);
	chassis_init(&chassis_para);
	while(1)
	{
//		Motor_Data_Get(1, 0);
		chassis_feed_back(&chassis_para);
		chassis_set_mode(&chassis_para);
		chassis_set_control(&chassis_para);
		chassis_control_loop(&chassis_para);
		
		//底盘状态切换切换
		chassis_state_load(&chassis_para);
		
		
		//发送速度数据
		chassis_rate_send(chassis_para.motor_rate_set, 0);
		
		if(chassis_para.chassis_mode == CHASSIS_NO_MOVE)				//重启
		{
			reset_count ++;
			memset(&chassis_para.chassis_set_line, 0, sizeof(geometry_twist_line_t));
			if(reset_count > SYSTEM_RESET_TIMEOUT/CHASSIS_TASK_TIMEOUT)
			{
				motor_init_flag = 0;
				if(reset_count > (SYSTEM_RESET_TIMEOUT/CHASSIS_TASK_TIMEOUT)*2)
					NVIC_SystemReset();
			}
		}
		else
			reset_count = 0;
			
		vTaskDelay(CHASSIS_TASK_TIMEOUT);
	}
}


static void chassis_init(chassis_para_t *chassis_para_init)
{
//	chassis_para_init->chassis_INS_point = get_INS_point();
//	chassis_para_init->chassis_remote_point = get_remote_control_point();
	chassis_para_init->chassis_ops9_point = get_ops9_data_point();
	
	float chassis_pos_pid[] = {CHASSIS_POS_KP, CHASSIS_POS_KI, CHASSIS_POS_KD};
	float chassis_angle_pid[] = {CHASSIS_ANGLE_KP, CHASSIS_ANGLE_KI, CHASSIS_ANGLE_KD};
	
	chassis_para_init->chassis_mode = CHASSIS_POSITION_MOVE;
	
	PID_init(&chassis_para_init->chassis_pos_pid, PID_POSITION, chassis_pos_pid, CHASSIS_POS_MAX_OUT, CHASSIS_POS_MAX_IOUT);
	PID_init(&chassis_para_init->chassis_angle_pid, PID_POSITION, chassis_angle_pid, CHASSIS_ANGLE_MAX_OUT, CHASSIS_ANGLE_MAX_IOUT);
	
	chassis_para_init->chassis_move_set_para.speed_max = CHASSIS_MAX_SPEED_POS;
	chassis_para_init->chassis_move_set_para.speed_min = CHASSIS_MIN_SPEED_POS;
	chassis_para_init->chassis_move_set_para.acc = CHASSIS_ACC_POS;
}


//运动逆解算
void chassis_move_back_decomposition(chassis_motor_para_t chassis_motor_para[4], geometry_twist_pos_t *chassis_pos, geometry_twist_line_t *chassis_line, uint16_t task_time_out)
{
	for(int i=0; i<4; i++)
	{
		chassis_motor_para[i].motor_len = chassis_motor_para[i].motor_angle * WHEEL_RADIUS;
		chassis_motor_para[i].motor_time_len = chassis_motor_para[i].motor_len - chassis_motor_para[i].motor_last_len;
		chassis_motor_para[i].motor_last_len = chassis_motor_para[i].motor_len;
	}
	
	fp32 vx = (-chassis_motor_para[0].motor_time_len + chassis_motor_para[1].motor_time_len + chassis_motor_para[2].motor_time_len - chassis_motor_para[3].motor_time_len)/4.0f;
	fp32 vy = (-chassis_motor_para[0].motor_time_len - chassis_motor_para[1].motor_time_len + chassis_motor_para[2].motor_time_len + chassis_motor_para[3].motor_time_len)/4.0f;
	
	chassis_line->vx = (vx / task_time_out)*1000.0f;
	chassis_line->vy = (vy / task_time_out)*1000.0f;
	
	fp32 z_angle;
	switch(FRE_Z_MODE)
	{
		case 0:
			z_angle = ((chassis_motor_para[0].motor_len+chassis_motor_para[3].motor_len + chassis_motor_para[1].motor_len+chassis_motor_para[2].motor_len)/4.0f)/CHASSIS_RADIUS;
			chassis_pos->z = z_angle;
			break;
		case 1:
			z_angle = chassis_pos->z;
			break;
	}
	fp32 sin_angle = sin(z_angle);
	fp32 cos_angle = cos(z_angle);
	
	
	chassis_pos->x += vx * cos_angle + vy * sin_angle;
	chassis_pos->y -= -vx * sin_angle + vy * cos_angle;
}

static void chassis_feed_back(chassis_para_t *chassis_para_feedback)
{
	chassis_para_feedback->chassis_pos.x = -chassis_para_feedback->chassis_ops9_point->OPS9_Rx_Data.x + chassis_para_feedback->chassis_pos_cali.x;
	chassis_para_feedback->chassis_pos.y = -chassis_para_feedback->chassis_ops9_point->OPS9_Rx_Data.y + chassis_para_feedback->chassis_pos_cali.y;
	
	for(int i=0; i<TOTAL_CHASSIS_NUM; i++)
	{
		chassis_para_feedback->chassis_motor[i].motor_angle = Motor_Data_Get(i, ANGLE_GET);
//		chassis_para_feedback->chassis_motor[i].motor_rate = Motor_Data_Get(i, RATE_GET);
	}
	if(FRE_Z_MODE == 1)
		chassis_para_feedback->chassis_pos.z = chassis_para_feedback->chassis_ops9_point->OPS9_Rx_Data.z;
	else
		//运动学逆解算，实时记录整车位置
		chassis_move_back_decomposition(chassis_para_feedback->chassis_motor, &chassis_para_feedback->chassis_pos, &chassis_para_feedback->chassis_line, CHASSIS_TASK_TIMEOUT);
}

//底盘模式切换
static void chassis_set_mode(chassis_para_t *chassis_set_mode_para)
{
//	if(switch_is_down(chassis_set_mode_para->chassis_remote_point->rc.s[CHASSIS_MODE_CHANNEL_1]))
//		//底盘无力
//		chassis_set_mode_para->chassis_mode = CHASSIS_NO_MOVE;
//	else if(switch_is_mid(chassis_set_mode_para->chassis_remote_point->rc.s[CHASSIS_MODE_CHANNEL_1]))
//		chassis_set_mode_para->chassis_mode = CHASSIS_NORMAL_MOVE;
//	else if(switch_is_up(chassis_set_mode_para->chassis_remote_point->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_down(chassis_set_mode_para->chassis_remote_point->rc.s[CHASSIS_MODE_CHANNEL_2]))
//		chassis_set_mode_para->chassis_mode = CHASSIS_POSITION_MOVE;
//	else if(switch_is_up(chassis_set_mode_para->chassis_remote_point->rc.s[CHASSIS_MODE_CHANNEL_1])&&switch_is_up(chassis_set_mode_para->chassis_remote_point->rc.s[CHASSIS_MODE_CHANNEL_2]))
//		chassis_set_mode_para->chassis_mode = CHASSIS_CALIBRATE;
//	else
//	{
//		chassis_set_mode_para->chassis_mode = get_start_key_state();
//	}
}

//获得自动任务目标位置
void chassis_get_auto_task_pos(geometry_twist_pos_t *chassis_set_pos_point, cam_pos_t *chassis_pos_cali_point)
{
	geometry_twist_pos_t chassis_temp_pos;
	cam_pos_t chassis_pos_cali;
	uint8_t now_task_type;
	now_task_type = get_auto_screen_task_chassis_pos(&chassis_temp_pos);
	get_auto_task_cali(&chassis_pos_cali);
	if(now_task_type != TASK_TYPE_GIMBAL)
	{
		if(now_task_type == TASK_TYPE_CALIBRATE)
		{
			chassis_pos_cali_point->x -= chassis_pos_cali.x;
			chassis_pos_cali_point->y -= chassis_pos_cali.y;
		}
		else
			*chassis_set_pos_point = chassis_temp_pos;
	}
}


//获得速度环遥控器速度设定
void chassis_normal_move_get_line(chassis_para_t *chassis_normal_move_para)
{
	fp32 vy_set = 0;
	fp32 vx_set = 0;
	
//	if(vx_set > chassis_rate)
	
	fp32 cos_angle = cos(chassis_normal_move_para->chassis_pos.z);
	fp32 sin_angle = sin(chassis_normal_move_para->chassis_pos.z); 
	if(SET_Z_MODE)
	{
		chassis_normal_move_para->chassis_set_line.vx = cos_angle * vx_set + sin_angle * vy_set;
		chassis_normal_move_para->chassis_set_line.vy = -sin_angle * vx_set + cos_angle * vy_set;
	}
	else
	{
		chassis_normal_move_para->chassis_set_line.vx = vx_set;
		chassis_normal_move_para->chassis_set_line.vy = vy_set;
	}
	chassis_normal_move_para->chassis_set_line.wz = 0;
}


void get_chassis_move_set_para(fp32 *move_para)
{
	if(move_para[0]!=0&&move_para[1]!=0)
	{
		chassis_para.chassis_move_set_para.speed_max = move_para[0]/100.0f;
		chassis_para.chassis_move_set_para.acc = move_para[1]/100.0f;
	}
}

static void chassis_set_control(chassis_para_t *chassis_set_control_point)
{
	if(chassis_set_control_point->chassis_mode == CHASSIS_POSITION_MOVE)
		//获得自动任务
		chassis_get_auto_task_pos(&chassis_set_control_point->chassis_set_pos, &chassis_set_control_point->chassis_pos_cali);
	else if(chassis_set_control_point->chassis_mode == CHASSIS_NORMAL_MOVE)
		chassis_normal_move_get_line(chassis_set_control_point);
	Trapezoidal_deceleration_algorithm(&chassis_set_control_point->chassis_speed_max, chassis_set_control_point->chassis_set_pos, chassis_set_control_point->chassis_pos, chassis_set_control_point->chassis_move_set_para.acc, chassis_set_control_point->chassis_move_set_para.speed_max, chassis_set_control_point->chassis_move_set_para.speed_min);
}

//运动正解算
void chassis_move_forward_decomposition(fp32 motor_rate_set[4], geometry_twist_line_t chassis_line_set)
{
   motor_rate_set[0] = (-chassis_line_set.vx + chassis_line_set.vy + chassis_line_set.wz)/WHEEL_RADIUS;
   motor_rate_set[1] = (chassis_line_set.vx + chassis_line_set.vy + chassis_line_set.wz)/WHEEL_RADIUS;
   motor_rate_set[2] = (chassis_line_set.vx - chassis_line_set.vy + chassis_line_set.wz)/WHEEL_RADIUS;
   motor_rate_set[3] = (-chassis_line_set.vx - chassis_line_set.vy + chassis_line_set.wz)/WHEEL_RADIUS;
}


//位置环计算
void chassis_position_move_task(chassis_para_t *chassis_para_position_count)
{
	fp32 vx_map_set = PID_calc(&chassis_para_position_count->chassis_pos_pid, chassis_para_position_count->chassis_pos.x, chassis_para_position_count->chassis_set_pos.x);
	fp32 vy_map_set = PID_calc(&chassis_para_position_count->chassis_pos_pid, chassis_para_position_count->chassis_pos.y, chassis_para_position_count->chassis_set_pos.y);
	
	fp32 cos_angle = cos(chassis_para_position_count->chassis_pos.z);
	fp32 sin_angle = sin(chassis_para_position_count->chassis_pos.z);
	
	fp32 before_filter_vx = cos_angle * vx_map_set + sin_angle * vy_map_set;
	fp32 before_filter_vy = -sin_angle * vx_map_set + cos_angle * vy_map_set;
	fp32 before_filter_wz = PID_calc(&chassis_para_position_count->chassis_angle_pid, chassis_para_position_count->chassis_pos.z, chassis_para_position_count->chassis_set_pos.z);
	
	
	chassis_para_position_count->chassis_set_line.vx = limit_Data(before_filter_vx, chassis_para_position_count->chassis_speed_max, -chassis_para_position_count->chassis_speed_max);
	chassis_para_position_count->chassis_set_line.vy = limit_Data(before_filter_vy, chassis_para_position_count->chassis_speed_max, -chassis_para_position_count->chassis_speed_max);
	chassis_para_position_count->chassis_set_line.wz = limit_Data(before_filter_wz, chassis_para_position_count->chassis_speed_max, -chassis_para_position_count->chassis_speed_max);
}

static void chassis_control_loop(chassis_para_t *chassis_control_loop_para)
{

	//位置环模式
	if(chassis_control_loop_para->chassis_mode == CHASSIS_POSITION_MOVE)
		chassis_position_move_task(chassis_control_loop_para);
	//运动学正解算，获得四个电机实时转速
	chassis_move_forward_decomposition(chassis_control_loop_para->motor_rate_set, chassis_control_loop_para->chassis_set_line);
}


chassis_mode_e get_chassis_mode(void)
{
	return chassis_para.chassis_mode;
}

//0		位置
//1		角度
//2		所有
//3		其一
uint8_t get_chassis_state(uint8_t tag)
{
	switch(tag)
	{
		case 0:		//pos_state
			return  chassis_para.get_pos_state;
		case 1:		//angle_state	
			return 	chassis_para.get_angle_state;
		case 2:
			return	chassis_para.get_pos_state&chassis_para.get_angle_state;
		case 3:
			return 	chassis_para.get_pos_state|chassis_para.get_angle_state;
	}
	return 0;
}

//底盘状态切换获得
static void chassis_state_load(chassis_para_t *chassis_state_load_para)
{
	fp32 x_err = chassis_state_load_para->chassis_pos.x - chassis_state_load_para->chassis_set_pos.x;
	fp32 y_err = chassis_state_load_para->chassis_pos.y - chassis_state_load_para->chassis_set_pos.y;
	fp32 z_err = chassis_state_load_para->chassis_pos.z - chassis_state_load_para->chassis_set_pos.z;
	
	if(fabs(x_err) < POS_THRESHOLD_XY && fabs(y_err) < POS_THRESHOLD_XY)
		chassis_state_load_para->get_pos_state = 1;
	else
		chassis_state_load_para->get_pos_state = 0;
	if(fabs(z_err) < POS_THRESHOLD_Z)
		chassis_state_load_para->get_angle_state = 1;
	else
		chassis_state_load_para->get_angle_state = 0;
	
}

chassis_para_t *get_chassis_para_point(void)
{
	return &chassis_para;
}

