#include "average_filter.h"

#define TOTAL_NUM		150

float data[3][TOTAL_NUM];
 
void averageFilter_init(uint8_t num, float in_data)
{
	for(int i=0; i<TOTAL_NUM; i++)
	{
		data[num][i] = in_data;
	}
}
float averageFilter(uint8_t num, float in_data)
{
	float sum = 0;
	for(int i=0; i<TOTAL_NUM-1; i++)
	{
		data[num][i]=data[num][i+1];
		sum = sum + data[num][i];
	}
	data[num][TOTAL_NUM-1] = in_data;
	sum = sum + data[num][TOTAL_NUM-1];
	
	return(sum/TOTAL_NUM);
}

#include <math.h>
#include "limit.h"
fp32 len_total,len,acc_dis;
geometry_twist_pos_t last_target_pos, last_target_pos_temp;
void Trapezoidal_deceleration_algorithm(fp32 *V, geometry_twist_pos_t target_pos, geometry_twist_pos_t pos, fp32 acc, fp32 V_max, fp32 V_min)
{
	if(last_target_pos_temp.x!=target_pos.x||last_target_pos_temp.y!=target_pos.y)
	{
		last_target_pos = last_target_pos_temp;
		last_target_pos_temp = target_pos;
	}
	
	len_total = sqrt(pow(target_pos.x - last_target_pos.x, 2) + pow(target_pos.y - last_target_pos.y, 2));
	len = sqrt(pow(target_pos.x - pos.x, 2) + pow(target_pos.y - pos.y, 2));
	acc_dis = 0.5f*pow(V_max-V_min,2)/acc;
	if(acc_dis>len_total/2)
		acc_dis = len_total/2;
	
	
	if(len_total - len<acc_dis)
	{
		*V = 2.0f*(len_total-len)*acc/(V_max-V_min);		//(V_max-V_min)*(len_total-len)/(0.5f*pow(V_max-V_min,2)/acc);
	}
	else if(len<acc_dis)
	{
		*V = (V_max-V_min)*(len)/(0.5f*pow(V_max-V_min,2)/acc);
	}
	*V=limit_Data(*V, V_max, V_min);
	
}
