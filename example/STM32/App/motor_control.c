#include "motor_control.h"
#include "key.h"

motor_para_t motor_para[TOTAL_MOTOR_NUM];



#define CHASSIS_ANGLE_KP			35.0f
#define CHASSIS_ANGLE_KI			0.00f
#define CHASSIS_ANGLE_KD			0.0f
#define CHASSIS_ANGLE_MAX_OUT		200.0f
#define CHASSIS_ANGLE_MAX_IOUT		50.0f

#define CHASSIS_RATE_KP				0.2f
#define CHASSIS_RATE_KI				0.00f
#define CHASSIS_RATE_KD				0.0f
#define CHASSIS_RATE_MAX_OUT		MOTOR_MAX_RATE
#define CHASSIS_RATE_MAX_IOUT		10.0f

#define GIMBAL_RATE_KP				0.15f
#define GIMBAL_RATE_KI				0.00f
#define GIMBAL_RATE_KD				0.0f
#define GIMBAL_RATE_MAX_OUT			MOTOR_MAX_RATE
#define GIMBAL_RATE_MAX_IOUT		10.0f

#define SEND_TIME_OUT	5

uint8_t motor_init_flag = 0;
uint8_t zero_flag[4]={0,0,0,0};

void Motor_Task(void const * argument)
{
	static uint8_t motor_total_id=1;
	Motor_UART_Init();
	vTaskDelay(500);
	
	while(1)
	{
		if(motor_init_flag==0)
		{
			for(int i=0; i<TOTAL_MOTOR_NUM; i++)
				memset(&motor_para[i], 0, sizeof(motor_para_t));
			Motor_Zero(motor_total_id);
			vTaskDelay(20);
			Motor_Stop(motor_total_id);
			vTaskDelay(20);
			Motor_Mode(motor_total_id, 2);
			vTaskDelay(20);
		}
		else
		{
			Motor_Pos_Ask(motor_total_id);
			vTaskDelay(SEND_TIME_OUT);
//			Motor_Vel_Ask(motor_total_id);
//			vTaskDelay(SEND_TIME_OUT);
//			if(motor_total_id != 5)
//				motor_rate_total_control(motor_total_id, motor_para[motor_total_id-1].Rate_set, 0);
//			else
				motor_rate_total_control(motor_total_id, motor_para[motor_total_id-1].Rate_set, 0);
		}
		motor_total_id++;
		if(motor_total_id > TOTAL_MOTOR_NUM)
		{
			motor_total_id = 1;
			if(motor_init_flag==0)
				motor_init_flag = 1;
		}
//		motor_data_flash();
		vTaskDelay(SEND_TIME_OUT);
	}
	
}



//Vel		转/s
//rate		rad/s
//radius	0.4m
//motor_id	1~..
void motor_rate_total_control(uint8_t motor_id, fp32 rate, fp32 acc)
{
	uint8_t dir;
	uint16_t vel = (uint16_t)fabs((rate/(2.0f*PI))*60);
	if(rate>=0)		dir=0;
	else if(rate<0)	dir=1;
	Motor_Rate_Control(motor_id, dir, vel, acc);
//	Motor_Pos_Control(motor_id, dir, vel, acc);
}

void chassis_rate_send(fp32 motor_rate[TOTAL_CHASSIS_NUM], uint8_t motor_acc)
{
	for(int i=0; i<TOTAL_CHASSIS_NUM; i++)
	{
		motor_para[i].Rate_set = motor_rate[i];
		motor_para[i].Acc_set = motor_acc;
	}
}

void gimbal_rate_send(fp32 motor_rate[TOTAL_GIMBLE_NUM])
{
	for(int i=0; i<TOTAL_GIMBLE_NUM; i++)
		motor_para[i+TOTAL_CHASSIS_NUM].Rate_set = motor_rate[i];
}

///**
//  * @brief    电机速度获取
//  * @param    addr：电机地址
//  */
//void motor_vel_read(uint8_t motor_id)
//{
//	uint16_t Vel;
//	fp32 Rate;
//	    // 获取电机实时转速返回值（电机实时转速返回值放大了10倍返回的，因为要保留1位小数）
//    Vel = ((uint16_t)uart_data.Motor_data[3] << 8) | (uint16_t)uart_data.Motor_data[4];

//    // 缩小10倍，并判断符号，得到真正的实时转速，存放在浮点数变量Motor_Vel中
//    Rate = (fp32)Vel * 0.1f/(2.0f*PI); 
//	if(uart_data.Motor_data[2])
//		Rate = -Rate; 
//	if(uart_data.Motor_data[2])	motor_para[motor_id-1].Rate = -Rate;
//	else						motor_para[motor_id-1].Rate = Rate;
//	
//}
///**
//  * @brief    电机位置获取
//  * @param    addr：电机地址
//  */
//void motor_position_read(uint8_t motor_id)
//{
//	uint32_t pos;
//	fp32 angle;
//	// 获取电机实时角度返回值（电机实时角度返回值放大了10倍返回的，因为要保留1位小数）
//    pos = ((uint32_t)uart_data.Motor_data[3] << 24) | ((uint32_t)uart_data.Motor_data[4] << 16) | ((uint32_t)uart_data.Motor_data[5] << 8) | (uint32_t)uart_data.Motor_data[6];

//    // 缩小10倍，并判断符号，得到真正的实时角度，存放在浮点数变量Motor_Cur_Pos中
//    angle = (fp32)pos * 0.1f * 2.0f * PI/360.0f; 
//	if(uart_data.Motor_data[2]) 
//		angle = -angle;
//	
//	motor_para[motor_id-1].Angle = angle;
//}

/**
  * @brief    电机速度获取
  * @param    addr：电机地址
  */
void motor_vel_read(uint8_t addr)
{
	uint16_t Vel;
	fp32 Rate;
	Vel = 	(uint16_t)(((uint16_t)uart_data.Motor_data[3] << 8)   |
					   ((uint16_t)uart_data.Motor_data[4] << 0));
	Rate = Vel*2.0f*PI/60.0f;
	if(uart_data.Motor_data[2])	motor_para[addr-1].Rate = -Rate;
	else						motor_para[addr-1].Rate = Rate;
	
}
/**
  * @brief    电机位置获取
  * @param    addr：电机地址
  */
void motor_position_read(uint8_t addr)
{
	uint32_t pos;
    pos = (uint32_t)(((uint32_t)uart_data.Motor_data[3] << 24)    |
                     ((uint32_t)uart_data.Motor_data[4] << 16)    |
                     ((uint32_t)uart_data.Motor_data[5] << 8)     |
                     ((uint32_t)uart_data.Motor_data[6] << 0));
	motor_para[addr-1].Angle = (fp32)pos * 2.0f * PI / 65536.0f;
	if(uart_data.Motor_data[2]) { motor_para[addr-1].Angle = -motor_para[addr-1].Angle; }
//	//处理另外两个电机的角度加个负号
//	if(addr == 1||addr == 4) { motor_para[addr-1].Angle = -motor_para[addr-1].Angle; }
}


/**
  * @brief    电机数据获取
  * @param    addr：电机地址
  */
uint8_t motor_data_flash(void)
{
	uint8_t motor_addr = uart_data.Motor_data[0];
	uint8_t check_num = uart_data.Motor_data[1];
	if(motor_addr!=0)
		switch(check_num)
		{
			case 0x35:
				motor_vel_read(motor_addr);	
				return RATE_GET_FLAG;
			case 0x36:
				motor_position_read(motor_addr);
				return ANGLE_GET_FLAG;
		}
	return Read_Error;
}


fp32 Motor_Data_Get(uint8_t i, uint8_t data_type)
{
	motor_data_flash();
	switch(data_type)
	{
		case ANGLE_GET:
			return motor_para[i].Angle;
		case RATE_GET:
			return motor_para[i].Rate;
	}
	return 0;
}

