#ifndef _IMU_H
#define _IMU_H

#include "stm32f1xx_hal.h"
#include <string.h>

#define PI		3.1415926535897932384626433832795f


typedef __packed struct
{
	uint8_t 	header;
	uint8_t 	check;
	uint32_t 	reserve1;
	int16_t 	yaw;
	uint16_t 	reserve2;
	uint8_t 	sum;
}IMU_RX_Data_t;

typedef struct
{
	uint8_t get_data_flag;
	float yaw;
}IMU_para_t;

const IMU_para_t *get_imu_para_point(void);
void IMU_dma_init(void);
void IMU_zero_angle(void);
void IMU_check_task(void);
void IMU_init(void);

#endif

