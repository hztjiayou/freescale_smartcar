#ifndef __IMAGE_H__
#define __IMAGE_H__

#define ERR2STAR 20
#define ERR2LONG 30

#define MAX_N 100       //(x_i,y_i)的最大维数
#define filter_column 3//过滤列数(防止摄像头的边侧漏光)

typedef struct tagPOINT //结构是算斜率用的
{
	double x;
	double y;
} POINT;



typedef struct Fuzzy_line_finding //结构体里面是模糊寻线的数据
{
  unsigned int Number_Ahead;//记录前几行的数值，最后求平均
  char Track_width[60];//储存赛道半宽度

  char Left_Black[60];//存储左边线（一维数组） 默认第一个值为0
  char Left_Black_flag; //左边线中断位

  char Right_Black[60];//存储右边线（一维数组）默认第一个值为80
  char Right_Black_flag;  //右边线中断位

  int MiddleLine_old;  //储存上一次的中间线
  int MiddleLine[60];//存储中间线（一维数组）默认第一个值为40

  char Scan_interrupt_flag;//扫描中断位
  char Scan_interrupt_flag_flag;//扫描中断位的标志位（与速度的控制有关系）
  char Average_flag;       //平均值标志位（这一帧）
  char Average_line_number_flag;    //平均值行数的标志（这一帧）
  char Average_line_number_flag_old;//平均值行数的标志（上一帧）
  char crossroads_flag;    //十字路口标志位(这一帧)
  char crossroads_flag_old;//十字路口标志位(上一帧)
  char crossroads_Left_flag; //十字路口左标志位(这一帧)
  char crossroads_Left_flag_old; //十字路口左标志位(上一帧)
  char crossroads_Right_flag;//十字路口右标志位(这一帧)
  char crossroads_Right_flag_old;//十字路口右标志位(上一帧)
  char crossroads_Left_Right_flag;    //十字路口左右标志位
  char crossroads_Left_Right_flag_old;//十字路口左右标志位（上一帧）
  int Direction_flag;    //方向标志位
  int Direction_flag_old;//方向标志位(上一帧)

  char Left_black_line_flag;//左黑线标志位(这一帧)
  char Left_black_line_flag_old;//左黑线标志位(上一帧)
  char Right_black_line_flag;//右黑线标志位(这一帧)
  char Right_black_line_flag_old;//右黑线标志位(上一帧)
}Fuzzy_line_finding;

void Track_width_init(Fuzzy_line_finding *line);//初始化赛道宽度
void Left_and_right_scanning(int16 i,Fuzzy_line_finding *line);//左右扫描边线
void Addition_and_subtraction_judgment(int16 i,Fuzzy_line_finding *line);//边线加减法
void Follow_the_judgment(int16 i,Fuzzy_line_finding *line);//边线跟随法
void Fuzzy_line_finding_method(Fuzzy_line_finding *line);//模糊寻线法

void Weight_init(int Weight_Max);//初始化各行的权重
void Error_Handle_Jiaquan(Fuzzy_line_finding *line);//差值加权相加法
double GetRoadErr3(int *line);//最小二乘法算差值

void Image_scanning();
void Show_TestData();//显示测试的数据

char Absolute(char a,char b);

extern int Error_middleline;

#endif