#ifndef __CONTROL_H__
#define __CONTROL_H__


#define POWER_MAX 400  //����
#define Median_of_rudder 1485 //�����ֵ ��1215  �У�1485  �ң�1795


extern int Velocity_max;  //����ٶ�
extern int Velocity_min;//��С����ٶ�
void PIT0_IRQHandler(void);
void PIT2_IRQHandler(void);
void control(void);
void oled_show(void);
void Set_Velocity_votor(void);
#endif