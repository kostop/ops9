#ifndef _SMOOTH_SPEED_FILTER_H
#define _SMOOTH_SPEED_FILTER_H

#include <stdio.h>
#include <string.h>
#include <math.h>

double smooth_speed_realtime(double current_speed, double *history_speed, int size);
//void smooth_speed(double *speed, int size, int window_size);

#endif
