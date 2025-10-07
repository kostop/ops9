#ifndef _OPS9_TASK_H
#define _OPS9_TASK_H

#include "head_file.h"

typedef __packed struct
{
	uint8_t header;
	fp32 x;
	fp32 y;
	fp32 z;
	uint8_t check_sum;
}OPS9_Rx_Data_t;

typedef struct
{
	OPS9_Rx_Data_t OPS9_Rx_Data;
}ops9_para_t;

void OPS9_init(void);
ops9_para_t *get_ops9_data_point(void);

#endif
