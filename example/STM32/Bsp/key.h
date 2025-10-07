#ifndef _KEY_H
#define _KEY_H

#include "head_file.h"

chassis_mode_e get_start_key_state(void);
uint8_t get_gimbal_cali_key_state(void);
void KEY_Init(void);

#endif
