#ifndef __CONTROL_H__
#define __CONTROL_H__


#define POWER_MAX 400  //？？
#define Median_of_rudder 1485 //舵机中值 左：1215  中：1485  右：1795


extern int Velocity_max;  //最大速度
extern int Velocity_min;//最小弯道速度
void PIT0_IRQHandler(void);
void PIT2_IRQHandler(void);
void control(void);
void oled_show(void);
void Set_Velocity_votor(void);
#endif