#ifndef _ROBOT_COMMUNICATION_H
#define _ROBOT_COMMUNICATION_H

#include <stdint.h>
#include <string.h>


#include "uart_receive.h"
#include "usart.h"
#include "crc8_crc16.h"

#define DATA_MAX_SIZE	30


typedef struct
{
	float world_x;
	float world_y;
	float world_z;
	
	float vx;
	float vy;
	float wz;
	
}chassis_state_t;

typedef __packed struct
{
    uint8_t header;
    float x;
    float y;
    float z;

    float vx;
    float vy;
    float wz;
    uint16_t checksum;
}Robot_Tx_Data_t;


typedef __packed struct
{
    uint8_t header;
	uint8_t type;			//圆环、物料
	uint8_t color;			//1 红色	2 绿色	3 蓝色
	uint16_t order1;
	uint16_t order2;
    uint16_t cx;
    uint16_t cy;
    uint16_t checksum;
}Robot_Rx_Data_t;

typedef struct
{
	uint8_t rx_data_size;
	uint8_t tx_data_size;
	uint8_t rx_data[DATA_MAX_SIZE];
	uint8_t tx_data[DATA_MAX_SIZE];
	
	Robot_Tx_Data_t Robot_Tx_Data;
	Robot_Rx_Data_t Robot_Rx_Data;
}robot_communication_t;

void Robot_Data_get(chassis_state_t *chassis_state);
void robot_com_task_init(void);
void robot_data_send(robot_communication_t* robot_communication);
void Robot_Communication_Callback(void);
void Robot_Com_Task(void const *pvParameters);
const robot_communication_t* get_robot_com_data_point(void);

#endif

