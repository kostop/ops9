#include "limit.h"

/*
* 数据限幅函数
*/
float limit_Data(float val, float hight, float low)
{
	if(val<low) val = low;
	else val = val;
	if(val>hight) val = hight;
	else val = val;
	return val;
}

