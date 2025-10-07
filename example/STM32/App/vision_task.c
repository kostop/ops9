#include "vision_task.h"

#define PIXEL_DISTANCE_CONVERSION	0.00135135135135135135135135135135f		//cx*宽度*实际高度/（像素宽度*高度）	cx*h*0.08f/(320*0.185f)
#define PIXEL_COL	640
#define PIXEL_ROW	480
#define HALF_COL	320
#define HALF_ROW	240

#define METERIAL_SPEED_THRESHOLD	0.05f

#define VISION_OFFSET_X				-0.008f
#define VISION_OFFSET_Y				-0.003f

vision_para_t vision_para;

void vision_init(void)
{
	vision_para.vision_robot_communication_point = get_robot_com_data_point();
	vision_para.vision_chassis_point = get_chassis_para_point();
	vision_para.data[0] = 1;
	vision_para.data[1] = 2;
	vision_para.data[2] = 3;
	vision_para.data[3] = 1;
	vision_para.data[4] = 2;
	vision_para.data[5] = 3;
}

//返回测定坐标
cam_pos_t vision_cali_pos(uint8_t cali_type)
{
	//获得底盘绝对Z轴旋转角度做坐标象限变换
	vision_para.chassis_pos = vision_para.vision_chassis_point->chassis_pos;
	fp32 cos_angle = cos(vision_para.chassis_pos.z);
	fp32 sin_angle = sin(vision_para.chassis_pos.z);
	
	vision_para.target_pos.x = cos_angle * (vision_para.meterial_circle_info[cali_type-1].x + VISION_OFFSET_X) - sin_angle * (vision_para.meterial_circle_info[cali_type-1].y + VISION_OFFSET_Y);
	vision_para.target_pos.y = sin_angle * (vision_para.meterial_circle_info[cali_type-1].x + VISION_OFFSET_X) + cos_angle * (vision_para.meterial_circle_info[cali_type-1].y + VISION_OFFSET_Y);
	return vision_para.target_pos;
}

void get_order(uint8_t *data)
{
	memcpy(data, vision_para.data, 6);
}

//测算物料移动速度返回
uint8_t vision_wait_meterial(uint8_t count, uint16_t task_time)
{
	static fp32 last_x, last_y;
	vision_para.vx = (vision_para.meterial_circle_info[TYPE_METERIAL-1].x - last_x)*1000/task_time;
	vision_para.vy = (vision_para.meterial_circle_info[TYPE_METERIAL-1].y - last_y)*1000/task_time;
	last_x = vision_para.meterial_circle_info[TYPE_METERIAL-1].x;
	last_y = vision_para.meterial_circle_info[TYPE_METERIAL-1].y;
	
	if(vision_para.data[count] == vision_para.color && vision_para.type == TYPE_METERIAL)
	{
		if(fabs(vision_para.meterial_circle_info[TYPE_METERIAL-1].x)<METERIAL_SPEED_THRESHOLD&&fabs(vision_para.meterial_circle_info[TYPE_METERIAL-1].y)<METERIAL_SPEED_THRESHOLD)
		{
			return 1;
		}
	}
	return 0;
}

//视觉任务获得坐标数据
//返回识别类型
uint8_t vision_get_data(void)
{
	vision_para.type = vision_para.vision_robot_communication_point->Robot_Rx_Data.type;
	vision_para.color = vision_para.vision_robot_communication_point->Robot_Rx_Data.color+1;
	if(vision_para.vision_robot_communication_point->Robot_Rx_Data.order1!=0&&vision_para.vision_robot_communication_point->Robot_Rx_Data.order2!=0)
	{
		vision_para.data[0] = vision_para.vision_robot_communication_point->Robot_Rx_Data.order1/100;
		vision_para.data[1] = vision_para.vision_robot_communication_point->Robot_Rx_Data.order1/10%10;
		vision_para.data[2] = vision_para.vision_robot_communication_point->Robot_Rx_Data.order1%10;
		vision_para.data[3] = vision_para.vision_robot_communication_point->Robot_Rx_Data.order2/100;
		vision_para.data[4] = vision_para.vision_robot_communication_point->Robot_Rx_Data.order2/10%10;
		vision_para.data[5] = vision_para.vision_robot_communication_point->Robot_Rx_Data.order2%10;
	}
	
	switch(vision_para.type)
	{
		case TYPE_CIRCLE:
			vision_para.meterial_circle_info[TYPE_CIRCLE-1].x = -(vision_para.vision_robot_communication_point->Robot_Rx_Data.cx - HALF_COL)* PIXEL_DISTANCE_CONVERSION * get_gimbal_cam_hight();
			vision_para.meterial_circle_info[TYPE_CIRCLE-1].y = (vision_para.vision_robot_communication_point->Robot_Rx_Data.cy - HALF_ROW)* PIXEL_DISTANCE_CONVERSION * get_gimbal_cam_hight();
			break;
		case TYPE_METERIAL:
			vision_para.meterial_circle_info[TYPE_METERIAL-1].x = -(vision_para.vision_robot_communication_point->Robot_Rx_Data.cx - HALF_COL)* PIXEL_DISTANCE_CONVERSION * get_meterial_cam_hight();
			vision_para.meterial_circle_info[TYPE_METERIAL-1].y = (vision_para.vision_robot_communication_point->Robot_Rx_Data.cy - HALF_ROW)* PIXEL_DISTANCE_CONVERSION * get_meterial_cam_hight();
			break;
	}
	return vision_para.type;
}

