#ifndef _ROBOT_COM_H
#define _ROBOT_COM_H

#include "imu.h"
#include "ops9_control.h"
#include "stm32f1xx_hal.h"

#define ROBOT_CMD_RESTART		0x22U

typedef __packed struct
{
	uint8_t header;
	float x;
	float y;
	float z;
	uint8_t check_sum;
}Robot_TX_data_t;

typedef __packed struct
{
	uint8_t header;
	uint8_t data;
}Robot_RX_data_t;

typedef struct
{
	const IMU_para_t		*robot_com_imu_point;
	const control_para_t 	*robot_com_control_point;
	Robot_RX_data_t			Robot_RX_data;
	Robot_TX_data_t			Robot_TX_data;
}robot_com_para_t;

void Robot_com_init(void);
void Robot_com_call_back(void);
void Robot_com_send_data(void);
void Robot_com_tx_complete_callback(void);
void Robot_com_error_callback(void);
void Robot_com_prepare_reset(void);

uint8_t get_ops_cmd_data(void);

#endif
