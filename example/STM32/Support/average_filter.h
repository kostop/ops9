#ifndef _AVERAGE_FILTER_H
#define _AVERAGE_FILTER_H

#include <stdio.h>
#include <string.h>
#include "struct_typedef.h"
#include "math.h"
#include "cmsis_os.h"
#include "robot_kinematics.h"

void averageFilter_init(uint8_t num, float in_data);
float averageFilter(uint8_t num, float in_data);
void Trapezoidal_deceleration_algorithm(fp32 *V, geometry_twist_pos_t target_pos, geometry_twist_pos_t pos, fp32 acc, fp32 V_max, fp32 V_min);

#endif
