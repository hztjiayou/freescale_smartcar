#include "common.h"
#include "include.h"
extern uint8 img[CAMERA_H][CAMERA_W];
extern PID   PID_DuoJi;//舵机
extern PID   PID_Votor;//电机
extern int PWM_DuoJi; //舵机的PWM值
extern int Velocity_max;
extern int Velocity_min;
extern KEY_STATUS_e KEY_Data_B1;
extern KEY_STATUS_e KEY_Data_B2;
extern KEY_STATUS_e KEY_Data_B3;
extern int Velocity_votor;
extern unsigned char Option_Mode_1;//模式一选项
extern unsigned char Option_Mode_2;//模式二选项
extern unsigned char Option_Mode_3;//模式三选项
extern double Stop_Stop_accelerating_Time;//超减速时间
extern int Stop_Stop_accelerating_Velocity;//超减速速度差
extern double Time;
extern char Over_flag;
extern char Over_duoji_flag;
extern double Start_Time;
extern uint8 Guanxian;

int Fps=0;
char Fps_flag=0;
int Error_middleline=0;      //差值加权，有正负
//int Weight[60]={0};    //储存权重值
/*int Weight[60]={10,10,10,10,10,10,11,11,11,11,
                11,11,13,13,13,13,13,13,15,15,
                15,15,15,16,16,16,16,16,16,16,
                16,16,16,16,18,18,18,18,18,18,
                18,18,18,18,18,20,20,20,20,20,
                20,20,16,16,16,16,15,15,11,11};*///0.01,0.01
/*int Weight[60]={7,7,7,7,7,7,8,8,8,8,
                8,11,13,13,13,13,13,13,15,15,
                15,15,15,16,16,16,16,16,16,16,
                16,16,16,16,21,21,21,21,21,21,
                21,21,21,21,21,20,20,20,20,20,
                20,20,16,16,16,16,15,15,11,11};*/
int Weight[60]={0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,
                16,16,16,16,18,18,18,18,18,18,
                18,18,18,18,18,20,20,20,20,20,
                20,20,16,16,16,16,15,15,11,11,
                0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0};//0.0130,0.0130
int Weight_Sum=0; //储存所有权重的总值
int Weight_max=30;//储存权重最大的行号
double Error_k=0;            //差值求k值

//Fuzzy_line_finding Fuzzy_line={0,{0},  {0},0,  {80},0,  40,{40},  1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
Fuzzy_line_finding Fuzzy_line;
/*unsigned int Number_Ahead=0;//记录前几行的数值，最后求平均
char Track_width[60]={0};//储存赛道半宽度

char Left_Black[60]={0};//存储左边线（一维数组） 默认第一个值为0
char Left_Black_flag=0; //左边线中断位

char Right_Black[60]={80};//存储右边线（一维数组）默认第一个值为80
char Right_Black_flag=0;  //右边线中断位

char MiddleLine_old=40;  //储存上一次的中间线
char MiddleLine[60]={40};//存储中间线（一维数组）默认第一个值为40

char Scan_interrupt_flag=1;//扫描中断位
char Scan_interrupt_flag_flag=0;//扫描中断位的标志位（与速度的控制有关系）
char Average_flag=0;       //平均值标志位（这一帧）
char Average_line_number_flag=0;    //平均值行数的标志（这一帧）
char Average_line_number_flag_old=0;//平均值行数的标志（上一帧）
char crossroads_flag=0;    //十字路口标志位(这一帧)
char crossroads_flag_old=0;//十字路口标志位(上一帧)
char crossroads_Left_flag=0; //十字路口左标志位(这一帧)
char crossroads_Left_flag_old=0; //十字路口左标志位(上一帧)
char crossroads_Right_flag=0;//十字路口右标志位(这一帧)
char crossroads_Right_flag_old=0;//十字路口右标志位(上一帧)
char crossroads_Left_Right_flag=0;    //十字路口左右标志位
char crossroads_Left_Right_flag_old=0;//十字路口左右标志位（上一帧）
int Direction_flag=0;    //方向标志位
int Direction_flag_old=0;//方向标志位(上一帧)

char Left_black_line_flag=0;//左黑线标志位(这一帧)
char Left_black_line_flag_old=0;//左黑线标志位(上一帧)
char Right_black_line_flag=0;//右黑线标志位(这一帧)
char Right_black_line_flag_old=0;//右黑线标志位(上一帧)*/

///////////////////////////////////////////
int Road_length=0;

void Image_scanning()
{
  //ZuoHuanDaoIn();
  //YouHuanDaoIn();
  
  
  ///////////帧率检测///////////
  Fps++;
  //////////////////////////////
  if(zhaohao_banma_state == 1)
  {
    if(Time>=Start_Time)
    {
      //printf("Time:%0.3fs\r\n",Time);
      ClearState();
      Over_flag=1;
    }
  }
  if(Time < Start_Time)
  {
    ClearState();
    StartAndFinish();
  }
  //huandao(img);
  SDYS();
  ShiZiWan();
  //////////////////////////////
  //GetRoadBase2(img,Left_Black,Right_Black,MiddleLine,&Road_length);
  Fuzzy_line_finding_method(&Fuzzy_line);//模糊寻线法
  //////////////////////////////
  Error_Handle_Jiaquan(&Fuzzy_line);//加权处理误差
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
}

void Fuzzy_line_finding_method(Fuzzy_line_finding *line)//模糊寻线法
{
  int16 i;
  int16 j;
  line->Number_Ahead=0;
  line->Scan_interrupt_flag=0;
  line->MiddleLine[0]=40;
  line->Left_Black[0]=0;
  line->Right_Black[0]=80;
  line->Left_Black_flag=0;
  line->Right_Black_flag=0;
  line->MiddleLine_old=40;
  line->Average_flag=0;
  line->Average_line_number_flag=0;
  line->Direction_flag=0;
  line->crossroads_flag=0;
  line->crossroads_Left_flag=0;
  line->crossroads_Right_flag=0;
  line->crossroads_Left_Right_flag=0;
  line->Left_black_line_flag=0;
  line->Right_black_line_flag=0;
  for(i=CAMERA_H-1;i>=0;i--)
  {
    //break;//////////////////////////////////////////////////////////////////////////////////////////////////用自己的程序的时候要删掉
    if(i>CAMERA_H-4) //前三行求平均 确定起始中点
    {
      Addition_and_subtraction_judgment(i,line);//边线加减法
      line->Number_Ahead=line->MiddleLine[CAMERA_H-i-1]+line->Number_Ahead;
      if(i == CAMERA_H-3)
      {
        line->Number_Ahead=line->Number_Ahead/3;
        line->MiddleLine_old=line->Number_Ahead;
      }
      continue;
    }
    Follow_the_judgment(i,line);//边线跟随法
    /******************************过滤函数*************************************/ 
       
    if(line->MiddleLine[CAMERA_H-i-1]>75-filter_column)   //过滤没有赛道时仍有显示的中心线和加扫描中断标志
    {
      line->Scan_interrupt_flag=CAMERA_H-i-1;
      j=0;
      for(i=i;i>=0;i--)
      {
        j++;
        line->Direction_flag++;
        line->MiddleLine[CAMERA_H-i-1]=80+j;
      }
      line->Direction_flag_old=line->Direction_flag;//给本帧一个方向
      break;
    }              
    else {if((CAMERA_H-i-1)==59) line->Scan_interrupt_flag=CAMERA_H-i-1;}
    
    if(line->MiddleLine[CAMERA_H-i-1]<4+filter_column)   //过滤没有赛道时仍有显示的中心线和加扫描中断标志
    {
      line->Scan_interrupt_flag=CAMERA_H-i-1;
      j=0;
      for(i=i;i>=0;i--)
      {
        j++;
        line->Direction_flag--;
        line->MiddleLine[CAMERA_H-i-1]=0-j;
      }
      line->Direction_flag_old=line->Direction_flag;//给本帧一个方向
      break;
    }
    else {if((CAMERA_H-i-1)==59) line->Scan_interrupt_flag=CAMERA_H-i-1;}
    
    if(Absolute(line->Left_Black[CAMERA_H-i-1],line->Right_Black[CAMERA_H-i-1])<10)//过滤中线冲出赛道的情况及过滤掉检测终点线的情况//&& ((MiddleLine[CAMERA_H-i-1]>50) || (MiddleLine[CAMERA_H-i-1]<30))
    {
      if(line->Right_Black[CAMERA_H-i-1]>40)
      {
        line->Scan_interrupt_flag=CAMERA_H-i-1;
        j=0;
        for(i=i;i>=0;i--)
        {
          j++;
          line->Direction_flag++;
          line->MiddleLine[CAMERA_H-i-1]=80+j;
        }
        line->Direction_flag_old=line->Direction_flag;//给本帧一个方向
        break;
      }
      if(line->Left_Black[CAMERA_H-i-1]<40)
      {
        line->Scan_interrupt_flag=CAMERA_H-i-1;
        j=0;
        for(i=i;i>=0;i--)
        {
          j++;
          line->Direction_flag--;
          line->MiddleLine[CAMERA_H-i-1]=0-j;
        }
        line->Direction_flag_old=line->Direction_flag;//给本帧一个方向
        break;
      }
    }
    else {if((CAMERA_H-i-1)==CAMERA_H-1) line->Scan_interrupt_flag=CAMERA_H-i-1;}
      //if(Absolute(Left_Black[CAMERA_H-i-1],Right_Black[CAMERA_H-i-1])>60 && (CAMERA_H-i-1)>30 && crossroads_flag_old<=5)
        //crossroads_Left_Right_flag=crossroads_Left_Right_flag+1;//////////////////有问题--前方扫描行数不大于60过滤
    
      //if(crossroads_flag_old > 0 || crossroads_Left_Right_flag_old >4)//纠正方向处理
    
    if((line->crossroads_flag_old > 0 || line->crossroads_Left_Right_flag_old >5))
      {
        if((line->MiddleLine[CAMERA_H-i-1]-line->MiddleLine[CAMERA_H-i-2])>=0 && line->Direction_flag_old< 0 && line->Average_flag == 0) /*&& Average_flag == 0*/
          line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine[CAMERA_H-i-2];
        if((line->MiddleLine[CAMERA_H-i-1]-line->MiddleLine[CAMERA_H-i-2])>=0 && line->Direction_flag_old> 0 && line->Average_flag == 0 && Absolute(line->MiddleLine[CAMERA_H-i-1],line->MiddleLine[CAMERA_H-i-2])>3)
          line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine[CAMERA_H-i-2]+2;
        if((line->MiddleLine[CAMERA_H-i-1]-line->MiddleLine[CAMERA_H-i-2])<=0 && line->Direction_flag_old> 0 && line->Average_flag == 0) /*&& Average_flag == 0*/
          line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine[CAMERA_H-i-2];
        if((line->MiddleLine[CAMERA_H-i-1]-line->MiddleLine[CAMERA_H-i-2])<=0 && line->Direction_flag_old< 0 && line->Average_flag == 0 && Absolute(line->MiddleLine[CAMERA_H-i-1],line->MiddleLine[CAMERA_H-i-2])>3)
          line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine[CAMERA_H-i-2]-2;
        if(line->Direction_flag_old == 0 && line->Average_flag == 0)
          line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine[CAMERA_H-i-2];
      }
    line->MiddleLine_old=line->MiddleLine[CAMERA_H-i-1];//每次更新中线后必须要给MiddleLine_old值  绝对不能注释掉
  }///////////////////////////////////////////大FOR循环结束//然后把标志位给下一帧用于判断
  line->Average_line_number_flag_old=line->Average_line_number_flag;
  line->Left_black_line_flag_old=line->Left_black_line_flag;
  line->Right_black_line_flag_old=line->Right_black_line_flag;
  line->crossroads_Left_flag_old=line->crossroads_Left_flag;
  line->crossroads_Right_flag_old=line->crossroads_Right_flag;
  if((line->crossroads_Left_flag>5 && line->crossroads_Right_flag>5) || line->crossroads_Left_flag>20 || line->crossroads_Right_flag>20)
    line->crossroads_Left_Right_flag_old=(line->crossroads_Left_flag+line->crossroads_Right_flag)/2;
  else
    line->crossroads_Left_Right_flag_old=0;
  line->crossroads_flag_old=line->crossroads_flag;//检测十字路口标志位
  line->Direction_flag_old=line->Direction_flag;//给本帧一个方向
}

/****************************左右扫描函数**************************************/
void Left_and_right_scanning(int16 i,Fuzzy_line_finding *line)
{
    int16 j;
    for(j=line->MiddleLine_old-1;j>0+filter_column;j--)        // 从中间向左边搜索，寻找黑点
    {
      if((img[i][j]==1 && img[i][j-1]==0) || (img[i][j]==0 && img[i][j-1]==0))
      {
        line->Left_Black_flag=1;
        line->Left_Black[CAMERA_H-i-1]=j-1;   
        if(CAMERA_H-i-1>=10)
          line->Left_black_line_flag++;
        break;
      }
      else
      {
        if(j == filter_column+1)
        {
          if(CAMERA_H-i-1>=10)
          line->crossroads_Left_flag++;
          line->Left_Black_flag=0;
          line->Left_Black[CAMERA_H-i-1]=0;  // 未找到左边黑点
        }
      }
    }
    
    for(j=line->MiddleLine_old;j<CAMERA_W-1-filter_column;j++)  // 从中间向右边搜索，寻找黑点
    {
      if((img[i][j]==1 && img[i][j+1]==0) || (img[i][j]==0 && img[i][j+1]==0))
      {
        line->Right_Black_flag=1;
        line->Right_Black[CAMERA_H-i-1]=j+1;  
        if(CAMERA_H-i-1>=10)
          line->Right_black_line_flag++;
        break; 
      }
      else
      {
        if(j == CAMERA_W-1-filter_column-1)
        {
          if(CAMERA_H-i-1>10)
          line->crossroads_Right_flag++;
          line->Right_Black_flag=0;
          line->Right_Black[CAMERA_H-i-1]=80;  //未找到右边黑点
        }
      }
    }
    ////////////////////////////////////////////////////////////////////////////////////
      /*if(Left_Black_flag == 1 && Right_Black_flag == 0 && Left_black_line_flag_old >= 49 && Right_black_line_flag_old < 49)
      {
        Right_Black[CAMERA_H-i-1]=Left_Black[CAMERA_H-i-1]+2*Track_width[CAMERA_H-i-1];
        Right_Black_flag=1;
      }
      if(Left_Black_flag == 0 && Right_Black_flag == 1 && Right_black_line_flag_old >= 49 && Left_black_line_flag_old < 49)
      {
        Left_Black[CAMERA_H-i-1]=Right_Black[CAMERA_H-i-1]-2*Track_width[CAMERA_H-i-1];
        Left_Black_flag=1;
      }*/
    ////////////////////////////////////////////////////////////////////////////////////
}

/****************************加减判断******************************************/
void Addition_and_subtraction_judgment(int16 i,Fuzzy_line_finding *line)                                 
{
    Left_and_right_scanning(i,line);
    if(line->Left_Black_flag!=0 && line->Right_Black_flag!=0)//左右边线同时存在的情况
    {
      line->Average_flag=1;
      line->Average_line_number_flag++;
      line->MiddleLine[CAMERA_H-i-1]=(line->Left_Black[CAMERA_H-i-1] + line->Right_Black[CAMERA_H-i-1])/2;
    }
    
    if(line->Left_Black_flag==0 && line->Right_Black_flag==0)//在这一行检测不到边线的情况    
    {
      line->Average_flag=0;
      if(CAMERA_H-i-1<45)
        line->crossroads_flag++;
      line->MiddleLine[CAMERA_H-i-1]=(line->Left_Black[CAMERA_H-i-1]+line->Right_Black[CAMERA_H-i-1])/2;
      //MiddleLine[CAMERA_H-i-1]=40;
    }
    
    if(line->Left_Black_flag==0 && line->Right_Black_flag!=0)//只有右边线的情况
    {
      line->Average_flag=0;
      line->MiddleLine[CAMERA_H-i-1]=line->Right_Black[CAMERA_H-i-1]-line->Track_width[CAMERA_H-i-1];//减去赛道半宽度
    }
    
    if(line->Left_Black_flag!=0 && line->Right_Black_flag==0)//只有左边线的情况
    {
      line->Average_flag=0;
      line->MiddleLine[CAMERA_H-i-1]=line->Left_Black[CAMERA_H-i-1]+line->Track_width[CAMERA_H-i-1];//加上赛道半宽度
    }
    
    line->MiddleLine_old=line->MiddleLine[CAMERA_H-i-1];
}

/****************************跟随线判断****************************************/

void Follow_the_judgment(int16 i,Fuzzy_line_finding *line)
{
    Left_and_right_scanning(i,line);
    if(line->Left_Black_flag!=0 && line->Right_Black_flag!=0)//左右边线同时存在的情况
    {
      line->Average_flag=1;
      line->Average_line_number_flag++;
      line->MiddleLine[CAMERA_H-i-1]=(line->Left_Black[CAMERA_H-i-1] + line->Right_Black[CAMERA_H-i-1])/2;
    }
    
    if(line->Left_Black_flag==0 && line->Right_Black_flag==0)//在这一行检测不到边线的情况    
    {                
      line->Average_flag=0;
      if(CAMERA_H-i-1<45)
        line->crossroads_flag++;
      line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine_old;
      //MiddleLine[CAMERA_H-i-1]=40;
    }
    
    if(line->Left_Black_flag==0 && line->Right_Black_flag!=0)//只有右边线的情况
    {
      line->Average_flag=0;
      if(CAMERA_H-i-1<30)
        line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine_old+line->Right_Black[CAMERA_H-i-1] - line->Right_Black[CAMERA_H-i-2]+line->Track_width[CAMERA_H-i-2]-line->Track_width[CAMERA_H-i-1];//中线跟随右边界
      else
        line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine_old+line->Right_Black[CAMERA_H-i-1] - line->Right_Black[CAMERA_H-i-2];//中线跟随右边界
    }
    
    if(line->Left_Black_flag!=0 && line->Right_Black_flag==0)//只有左边线的情况
    {
      line->Average_flag=0;
      if(CAMERA_H-i-1<30)
        line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine_old+line->Left_Black[CAMERA_H-i-1] - line->Left_Black[CAMERA_H-i-2]-line->Track_width[CAMERA_H-i-2]+line->Track_width[CAMERA_H-i-1];//中线跟随左边界
      else
        line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine_old+line->Left_Black[CAMERA_H-i-1] - line->Left_Black[CAMERA_H-i-2];//中线跟随左边界
    }
    
    if(line->MiddleLine[CAMERA_H-i-1]>line->MiddleLine_old && Absolute(line->MiddleLine[CAMERA_H-i-1],line->MiddleLine_old)<5 && (CAMERA_H-i-1)<20 && (CAMERA_H-i-1)>4)
    {
      line->Direction_flag=line->Direction_flag+(20-(CAMERA_H-i-1));//加权重处理
      if(line->Average_flag==1)
        line->Direction_flag=line->Direction_flag+(20-(CAMERA_H-i-1));//加权重处理
    }
    if(line->MiddleLine[CAMERA_H-i-1]<line->MiddleLine_old && Absolute(line->MiddleLine[CAMERA_H-i-1],line->MiddleLine_old)<5 && (CAMERA_H-i-1)<20 && (CAMERA_H-i-1)>4)
    {
      line->Direction_flag=line->Direction_flag-(20-(CAMERA_H-i-1));//加权重处理
      if(line->Average_flag==1)
        line->Direction_flag=line->Direction_flag-(20-(CAMERA_H-i-1));//加权重处理
    }

    line->MiddleLine_old=line->MiddleLine[CAMERA_H-i-1];
}

///////////////////////////////////////////////////////////////////////////////////////
void Error_Handle_Jiaquan(Fuzzy_line_finding *line)//用加权的方式处理误差
{
  int i;
  Error_middleline=0;
  for(i=0;i<=CAMERA_H-1;i++)
  {
    Error_middleline=((int)line->MiddleLine[i]-39)*Weight[i]*2+Error_middleline;
  }
}

double GetRoadErr3(int *line)//用斜率的方法处理误差
{
	int i;
	double sumX = 0;
	double sumY = 0;
	double sumXY = 0;
	double sumX2 = 0;
	double aveX;
	double aveY;
	double a, b, err;
	for (i = ERR2STAR; i < ERR2STAR+ERR2LONG; i++)
	{
		sumX += i - ERR2STAR;
		sumY += line[i];
		sumXY += (i - ERR2STAR) * line[i];
		sumX2 += (i - ERR2STAR)*(i - ERR2STAR);
	}
	aveX = sumX / ERR2LONG;
	aveY = sumY / ERR2LONG;
	a = (sumXY - sumX * sumY / ERR2LONG) / (sumX2 - sumX * sumX / ERR2LONG);
	b = aveY - a * aveX;
	err = 3000 * atan(a);
	err += 250 * (b - 39);
	return atan(a)*57.3;
}
///////////////////////////////////////////////////////////////////////////////////////
char Absolute(char a,char b)//判断差值
{
  char c;
  if(a>=b)
    c=a-b;
  else
    c=b-a;
  return c;
}

void Track_width_init(Fuzzy_line_finding *line)//初始化赛道半宽度
{
  int16 i;
  float  Width=30;
  for(i=0;i<CAMERA_H;i++)
  {
    Width=Width-0.33;
    line->Track_width[i]=(int)Width;
  }
}

void Weight_init(int Weight_Max)//初始化各行的权重  Weight_Max是权重最重的行号，不是数组里面的数
{
  int16 i;
  Weight_Sum=0;
  for(i=0;i<CAMERA_H;i++)
  {
    if(i<=(Weight_Max-1))
      Weight[i]=60-(Weight_Max-i);
    else
      Weight[i]=60-(i-Weight_Max);
    Weight_Sum=Weight_Sum+Weight[i];
  }
}
void Show_TestData()//显示测试的数据
{
    if(KEY_Data_B1 == KEY_DOWN && KEY_Data_B2 == KEY_DOWN)
    {
      //OLED_ShowStr_Number_float(80,0,"SST ",Stop_Stop_accelerating_Time,1);
      OLED_ShowStr_Number(80,0,"G ",Guanxian,1);
      OLED_ShowStr_Number(80,1,"SSV ",Stop_Stop_accelerating_Velocity,1);
      OLED_ShowStr_Number_float(80,2,"P ",PID_DuoJi.Proportion,1);
      OLED_ShowStr_Number_float(80,3,"D ",PID_DuoJi.Derivative,1);
      OLED_ShowStr_Number(80,4,"V ",Velocity_max,1);
      OLED_ShowStr_Number(80,5,"V ",Velocity_min,1);
      OLED_ShowStr_Number(80,6,"H:",Fuzzy_line.Scan_interrupt_flag,1);
      OLED_ShowStr_Number(80,7,"E:",Error_middleline,1);
      if(Option_Mode_1 == 0) OLED_ShowStr(86,0,":",1);
      if(Option_Mode_1 == 1) OLED_ShowStr(98,1,":",1);
      if(Option_Mode_1 == 2) OLED_ShowStr(86,2,":",1);
      if(Option_Mode_1 == 3) OLED_ShowStr(86,3,":",1);
      if(Option_Mode_1 == 4) OLED_ShowStr(86,4,":",1);
      if(Option_Mode_1 == 5) OLED_ShowStr(86,5,":",1);
      if(Option_Mode_1 == 6) OLED_ShowStr(86,6,":",1);
    }
    
    if(KEY_Data_B1 == KEY_UP && KEY_Data_B2 == KEY_DOWN)
    {
      //OLED_ShowStr_Number(80,0,"",zhaohao_huandao_state,1);
      OLED_ShowStr_Number(80,1,"DI:",Fuzzy_line.Direction_flag_old,1);
      OLED_ShowStr_Number(80,2,"AV:",Fuzzy_line.Average_line_number_flag_old,1);
      OLED_ShowStr_Number(80,3,"CL:",Fuzzy_line.crossroads_Left_flag,1);
      OLED_ShowStr_Number(80,4,"CR:",Fuzzy_line.crossroads_Right_flag,1);
      OLED_ShowStr_Number(80,5,"CLR:",Fuzzy_line.crossroads_Left_Right_flag_old,1);
      OLED_ShowStr_Number(80,6,"LB:",Fuzzy_line.Left_black_line_flag_old,1);
      OLED_ShowStr_Number(80,7,"RB:",Fuzzy_line.Right_black_line_flag_old,1);
    }
    
    if(KEY_Data_B1 == KEY_DOWN && KEY_Data_B2 == KEY_UP)
    {
      OLED_ShowStr_Number(80,0,"V:",Velocity_votor,1);
      OLED_ShowStr_Number_float(80,1,"P ",PID_Votor.Proportion,1);
      OLED_ShowStr_Number_float(80,2,"I ",PID_Votor.Integral,1);
      OLED_ShowStr_Number_float(80,3,"D ",PID_Votor.Derivative,1);
      OLED_ShowStr_Number(80,4,"V ",PID_Votor.SetPoint,1);
      OLED_ShowStr_Number(80,6,"E:",Error_middleline,1);//差值Error_middleline
      OLED_ShowStr_Number(80,7,"Pwm:",PWM_DuoJi,1);
      if(Option_Mode_2 == 0) OLED_ShowStr(86,1,":",1);
      if(Option_Mode_2 == 1) OLED_ShowStr(86,2,":",1);
      if(Option_Mode_2 == 2) OLED_ShowStr(86,3,":",1);
      if(Option_Mode_2 == 3) OLED_ShowStr(86,4,":",1);
    }
    
    if(KEY_Data_B1 == KEY_UP && KEY_Data_B2 == KEY_UP)
    {
      OLED_ShowStr(80,0,"  RC_off",1);
      OLED_ShowStr(80,1,"  RC_on",1);
      OLED_ShowStr(80,2,"  RC_cls",1);
      OLED_ShowStr(80,3,"  v_play",1);
      OLED_ShowStr(80,4,"  ",1);
      if(Option_Mode_3 == 0) OLED_ShowStr(80,0,"->",1);
      if(Option_Mode_3 == 1) OLED_ShowStr(80,1,"->",1);
      if(Option_Mode_3 == 2) OLED_ShowStr(80,2,"->",1);
      if(Option_Mode_3 == 3) OLED_ShowStr(80,3,"->",1);
      if(Option_Mode_3 == 4) OLED_ShowStr(80,4,"->",1);
    }
}
