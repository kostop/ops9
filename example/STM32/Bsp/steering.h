#ifndef _STEERING_H
#define _STEERING_H

#include "head_file.h"

#define YAW_STEERING		0
#define HAND_STEERING		1
#define TURNTABLE_STEERING		2

void steering_init(void);
bool_t steering_controller(uint8_t num, fp32 angle);

#endif
