#ifndef HEAD_FILE_H
#define HEAD_FILE_H

#define PI		3.1415926535897932384626433832795f
#define F_LIMIT(a) (a>=0?1:-1) //取一个数的正负号


#define TOTAL_CHASSIS_NUM	4
#define TOTAL_GIMBLE_NUM	1
#define TOTAL_MOTOR_NUM		TOTAL_CHASSIS_NUM

#define GIMBAL_THRESHOLD_HIGHT		0.003f		//m
#define GIMBAL_THRESHOLD_ANGLE		0.0005f		//rad

#define POS_THRESHOLD_XY			0.007f		//m
#define POS_THRESHOLD_Z				0.005f		//rad

typedef enum
{
	CHASSIS_NO_MOVE = 1,
	CHASSIS_NORMAL_MOVE,
	CHASSIS_POSITION_MOVE,
	CHASSIS_POSITION_SINGLE,
	CHASSIS_CALIBRATE,
}chassis_mode_e;

//User
#include "struct_typedef.h"
#include <string.h>
#include "math.h"
#include "cmsis_os.h"

//Support
#include "robot_kinematics.h"
#include "low_pass_filter.h"
#include "average_filter.h"
#include "smooth_speed_filter.h"
#include "limit.h"

//Official
#include "usart.h"
#include "main.h"

//Bsp
#include "Emm_V5.h"
#include "steering.h"
#include "ops9_task.h"

//App
#include "uart_receive.h"
#include "pid.h"
#include "motor_control.h"
#include "calibrate_task.h"
#include "gimbal_task.h"
#include "serial_screen.h"

#endif
