#include "low_pass_filter.h"


float fc = 2.0f;     //截止频率
float Ts = 0.02f;    //采样周期
float pi = 3.14159f; //π
float alpha = 0;     //滤波系数

/************************ 滤波器初始化 alpha *****************************/
void low_pass_filter_init(void)
{
  double b = 2.0 * pi * fc * Ts;
  alpha = 0.8;
}

float low_pass_filter(float value)
{
  static float out_last = 0; //上一次滤波值
  float out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}



