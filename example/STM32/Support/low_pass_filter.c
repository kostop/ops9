#include "low_pass_filter.h"


float fc = 2.0f;     //��ֹƵ��
float Ts = 0.02f;    //��������
float pi = 3.14159f; //��
float alpha = 0;     //�˲�ϵ��

/************************ �˲�����ʼ�� alpha *****************************/
void low_pass_filter_init(void)
{
  double b = 2.0 * pi * fc * Ts;
  alpha = 0.8;
}

float low_pass_filter(float value)
{
  static float out_last = 0; //��һ���˲�ֵ
  float out;

  /***************** �����һ�ν��룬��� out_last ��ֵ ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** һ���˲� *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}



