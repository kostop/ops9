#ifndef _ENCODER_H
#define _ENCODER_H

#include "stm32f1xx_hal.h"

#define PI		3.1415926535897932384626433832795f

typedef struct
{
	float	rate;
	int8_t dirt;
	int32_t count;
	int32_t total_count;
}encoder_para_t;

void Encoder_init(void);
void Encoder_task(void);

const encoder_para_t *get_encoder_para_point(uint8_t num);

#endif
