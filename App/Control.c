#include "common.h"
#include "include.h"
extern PID   PID_DuoJi;//舵机PID
extern PID   PID_Votor;//电机PID
extern PID_INIT PID_Init;
extern Fuzzy_line_finding Fuzzy_line;
volatile uint32 irqflag = 0;     //标志位定义，非0 表示进入中断
extern uint8 img[CAMERA_H][CAMERA_W];
extern int Weight_max;//储存权重最大的行号

//extern unsigned char Scan_interrupt_flag;//扫描停止位 错误的程序！！
extern int Error_middleline;      //加权后的差值
extern int Error_k;               //中线的k值
extern char Fps;                  //记录的fps值
extern char Fps_flag;             //Fps的标志位

uint8 Guanxian = 60;      //调图像二值化的阈值
char Over_flag=0;         //总结束标志位
char Start_flag=1;        //开始跑标志位

unsigned char Option_Mode_1=0;//模式一选项
unsigned char Option_Mode_2=0;//模式二选项
unsigned char Option_Mode_3=0;//模式三选项
int PWM_DuoJi=Median_of_rudder;  //定义舵机的PWM
int Speed=0;
int Speed_Max=800;   //电机的最大限幅

int Velocity_max=0;  //最大速度
int Velocity_min=180;//最小弯道速度

double Start_Time=1.2;//开始加速时间
int Start_to_speed_up=17000;//开始加速 7000
int Stop_accelerating=20000;//停止加速 25000
int Stop_Stop_accelerating=17000;//超停止加速
char Stop_Stop_accelerating_flag=0;//超减速标志位
double Stop_Stop_accelerating_Time=0.2;//超减速时间
int Stop_Stop_accelerating_Velocity=45;//超减速速度差
int Speed_up_coefficient=0; //系数(第一代的控制算法的参数)

char Velocity_mode=0;//速度策略的模式
char Velocity_mode_last=0;//速度策略的模式(上一次的)
char Velocity_mode_2to3_flag=0;//过渡高速的标志
int Bang_Pwm=200;
int Bang_Error=20000;

int Velocity_votor=0;//正交解码的值（速度）


double Time=0;                        //总时间
double Time_Fps=0;                    //检测FPS的时间
double Time_Stop_Stop_accelerating=0; //超减速用到的时间
double Time_over=0;                   //用于结束的时间
double Time_Velocity_mode_2to3=0;     //用于是否过渡高速的判断

char Show_Time_flag=0; //显示结束时间标志位

char DuoJi_Flag=1; //可能有用的舵机开关标志
KEY_e KEY_Data;
KEY_STATUS_e KEY_Data_B1;
KEY_STATUS_e KEY_Data_B2;
KEY_STATUS_e KEY_Data_B3;
KEY_STATUS_e KEY_Data_B4;

int16 buff[3] = {0xfc03,0,0x03fc};//上位机接受数据的协议
void PIT2_IRQHandler(void)
{
    /////////////计算帧率用///////////////
    if(Fps_flag == 1)
    {
      Time_Fps=Time_Fps+0.01;
      if(Time_Fps >= 1)
      {
        printf("Fps=%d\r\n",Fps);
        Fps=0;
        Time_Fps=0;
      }
    }
    else 
    {
      Time_Fps=0;
      Fps=0;
    }
    //////////////////////////////////////
    
    //////////计时器，用于输出时间////////
    if(Velocity_max!=0)
    { 
      Show_Time_flag=1;
      Time=Time+0.01;
    }
    else
    {
      PID_Votor.SumError=0;//停止时积分清零
      if(Show_Time_flag==1)
      {
        printf("Time:%0.3fs\r\n",Time);
        Time=0;
        Show_Time_flag=0;
      }
    }
    //////////////////////////////////////
    
    PIT_Flag_Clear(PIT2);  //清中断标志位
}

void PIT0_IRQHandler(void)
{
    //////////////用于结束计时/////////////
    if(Over_flag==1)
    {
      StartAndFinish();
      Time_over=Time_over+0.01;
      if(Time_over>=0.2)
      {
        Time_over=0;
        Over_flag=0;
        Velocity_max=0;
        DEBUG_PRINTF("\nSTOP for Banmaxian");//输出停车信息
      }
    }
    //////////////////////////////////////
    
    /////////////超停止加速///////////////
    if(Stop_Stop_accelerating_flag == 1)
    {
      Time_Stop_Stop_accelerating=Time_Stop_Stop_accelerating+0.01;
      if(Time_Stop_Stop_accelerating >= Stop_Stop_accelerating_Time)
      {
        Time_Stop_Stop_accelerating=0;
        Stop_Stop_accelerating_flag=0;
      }
    }
    else
    {
      Time_Stop_Stop_accelerating=0;
    }
    //////////////////////////////////////
    
    //////////////////////////////////////
    if(Velocity_mode_2to3_flag==1)
    {
      Time_Velocity_mode_2to3=Time_Velocity_mode_2to3+0.01;
    }
    else
      Time_Velocity_mode_2to3=0;
    //////////////////////////////////////
    
    //////////用于电机控制////////////////
    Velocity_votor = -FTM_QUAD_get(FTM1);   //获取FTM 正交解码 的脉冲数(负数表示反方向)
    buff[1] = Velocity_votor;
    //uart_putbuff(UART3, (uint8*)(buff),6);//给上位机发送数据
    if(KEY_Data_B3 == KEY_UP)
    {
      Set_Velocity_votor();
    }
    else
    {
      PID_Votor.SumError=0;//停止时积分清零
      FTM_PWM_Duty(FTM0, FTM_CH1,0);
      FTM_PWM_Duty(FTM0, FTM_CH2,0);
    }//用于电机控制
    //////////////////////////////////////
    
    FTM_QUAD_clean(FTM1);
    PIT_Flag_Clear(PIT0);  //清中断标志位
}

void Set_Velocity_votor(void)
{
  if(Error_middleline >= Bang_Error && Velocity_mode==1)
  {
    FTM_PWM_Duty(FTM0, FTM_CH1,Bang_Pwm);
    FTM_PWM_Duty(FTM0, FTM_CH2,0);  
  }
  {
    Speed += LocPID_Votor(Velocity_votor);  
    if(Speed>Speed_Max) Speed=Speed_Max;
    if(Speed<-Speed_Max) Speed=-Speed_Max;
    if(Speed>0)
    {
      FTM_PWM_Duty(FTM0, FTM_CH1,0);
      FTM_PWM_Duty(FTM0, FTM_CH2,Speed);
    }
    if(Speed<0)
    {
      FTM_PWM_Duty(FTM0, FTM_CH1,-Speed);
      FTM_PWM_Duty(FTM0, FTM_CH2,0);        
    }
  }
}


void control(void)
{
  KEY_Data_B3=key_get(KEY_B3);
  KEY_Data_B4=key_get(KEY_B4);
  if(KEY_Data_B4 == KEY_DOWN)
  {
    KEY_Data=KEY_Scan(0);
    KEY_Data_B1=key_get(KEY_B1);
    KEY_Data_B2=key_get(KEY_B2);
    
    if(KEY_Data == KEY_D && KEY_Data_B1 == KEY_DOWN && KEY_Data_B2 == KEY_DOWN)
    {
      Option_Mode_1++;
      if(Option_Mode_1 == 6)
        Option_Mode_1=0;
    }
    
    if(KEY_Data == KEY_D && KEY_Data_B1 == KEY_DOWN && KEY_Data_B2 == KEY_UP)
    {
      Option_Mode_2++;
      if(Option_Mode_2 == 4)
        Option_Mode_2=0;
    }
    
    if(KEY_Data == KEY_D && KEY_Data_B1 == KEY_UP && KEY_Data_B2 == KEY_UP)
    {
      Option_Mode_3++;
      if(Option_Mode_3 == 5)
        Option_Mode_3=0;
    }
    
    if(KEY_Data_B1 == KEY_DOWN && KEY_Data_B2 == KEY_DOWN)
    {
      //if(Option_Mode_1 == 0){if(KEY_Data ==  KEY_L) Stop_Stop_accelerating_Time=Stop_Stop_accelerating_Time+0.02;if(KEY_Data ==  KEY_R) Stop_Stop_accelerating_Time=Stop_Stop_accelerating_Time-0.02;}
      if(Option_Mode_1 == 0){if(KEY_Data ==  KEY_L) Guanxian=Guanxian+10;if(KEY_Data ==  KEY_R) Guanxian=Guanxian-10;}
      if(Option_Mode_1 == 1){if(KEY_Data ==  KEY_L) Stop_Stop_accelerating_Velocity=Stop_Stop_accelerating_Velocity+5;if(KEY_Data ==  KEY_R) Stop_Stop_accelerating_Velocity=Stop_Stop_accelerating_Velocity-5;}
      if(Option_Mode_1 == 2){if(KEY_Data ==  KEY_L) PID_Init.DuoJi_Proportion=PID_Init.DuoJi_Proportion+0.0002;if(KEY_Data ==  KEY_R) PID_Init.DuoJi_Proportion=PID_Init.DuoJi_Proportion-0.0002;}  
      if(Option_Mode_1 == 3){if(KEY_Data ==  KEY_L) PID_Init.DuoJi_Derivative=PID_Init.DuoJi_Derivative+0.0002;if(KEY_Data ==  KEY_R) PID_Init.DuoJi_Derivative=PID_Init.DuoJi_Derivative-0.0002;}
      if(Option_Mode_1 == 4){if(KEY_Data ==  KEY_L) {Velocity_max=Velocity_max+10;Speed_up_coefficient=(Velocity_max-Velocity_min)/(Stop_accelerating-Start_to_speed_up);}if(KEY_Data ==  KEY_R) {Velocity_max=Velocity_max-10;Speed_up_coefficient=(Velocity_max-Velocity_min)/(Stop_accelerating-Start_to_speed_up);}}
      if(Option_Mode_1 == 5){if(KEY_Data ==  KEY_L) {Velocity_min=Velocity_min+5;Speed_up_coefficient=(Velocity_max-Velocity_min)/(Stop_accelerating-Start_to_speed_up);}if(KEY_Data ==  KEY_R) {Velocity_min=Velocity_min-5;Speed_up_coefficient=(Velocity_max-Velocity_min)/(Stop_accelerating-Start_to_speed_up);}}
    }
    if(KEY_Data_B1 == KEY_DOWN && KEY_Data_B2 == KEY_UP)
    {
      if(Option_Mode_2 == 0){if(KEY_Data ==  KEY_L) PID_Init.Votor_Proportion=PID_Init.Votor_Proportion+0.1;if(KEY_Data ==  KEY_R) PID_Init.Votor_Proportion=PID_Init.Votor_Proportion-0.1;}//0.01
      if(Option_Mode_2 == 1){if(KEY_Data ==  KEY_L) PID_Init.Votor_Integral=PID_Init.Votor_Integral+1;if(KEY_Data ==  KEY_R) PID_Init.Votor_Integral=PID_Init.Votor_Integral-1;}//0.001
      if(Option_Mode_2 == 2){if(KEY_Data ==  KEY_L) PID_Init.Votor_Derivative=PID_Init.Votor_Derivative+0.1;if(KEY_Data ==  KEY_R) PID_Init.Votor_Derivative=PID_Init.Votor_Derivative-0.1;}//0.01
      if(Option_Mode_2 == 3){if(KEY_Data ==  KEY_L) {Velocity_max=Velocity_max+10;Speed_up_coefficient=(Velocity_max-Velocity_min)/(Stop_accelerating-Start_to_speed_up);}if(KEY_Data ==  KEY_R) {Velocity_max=Velocity_max-10;Speed_up_coefficient=(Velocity_max-Velocity_min)/(Stop_accelerating-Start_to_speed_up);}}
    }
    if(KEY_Data_B1 == KEY_UP && KEY_Data_B2 == KEY_UP)
    {
      if(Option_Mode_3 == 0){if(KEY_Data ==  KEY_L) ;if(KEY_Data ==  KEY_R) REC_off();}
      if(Option_Mode_3 == 1){if(KEY_Data ==  KEY_L) ;if(KEY_Data ==  KEY_R) REC_on();}
      if(Option_Mode_3 == 2){if(KEY_Data ==  KEY_L) ;if(KEY_Data ==  KEY_R) REC_cls();}
      if(Option_Mode_3 == 3){if(KEY_Data ==  KEY_L) ;if(KEY_Data ==  KEY_R) video_play();}
      if(Option_Mode_3 == 4){if(KEY_Data ==  KEY_L) ;if(KEY_Data ==  KEY_R) ;}
    }
  }
  
  //PID_DuoJi.Proportion = 0.0001*((abs(Error_middleline)/1000) * (abs(Error_middleline)/1000))/((float)1156/10) + (PID_Init.DuoJi_Proportion*10000-10)*0.0001;
  //if(PID_DuoJi.Proportion >= PID_Init.DuoJi_Proportion) PID_DuoJi.Proportion = 0.009;
  Pid_refresh();//刷新PID参数(每次更改PID参数要用)
  

  //////////////////////////控制舵机的值///////////////////////////
  if(KEY_Data_B3 == KEY_UP && DuoJi_Flag==1)
  {
    //if(Error_middleline > 0) Error_middleline = (int)((float)Error_middleline *1.148);
    PWM_DuoJi = Median_of_rudder-LocPID_DuoJi(Error_middleline); 
    
    //if(PWM_DuoJi > 1485)  PWM_DuoJi = (int)((float)PWM_DuoJi *1.148);
    if(PWM_DuoJi > 1795)  PWM_DuoJi = 1795;
    if(PWM_DuoJi < 1215)  PWM_DuoJi = 1215;
    
    FTM_PWM_Duty(FTM2,FTM_CH0,PWM_DuoJi);
  }
  /////////////////////////////////////////////////////////////////
  
  if(Velocity_max!=0)
  {
    if(Fuzzy_line.Scan_interrupt_flag<=48)
    {
      Velocity_mode=1;
    }
    else Velocity_mode=0;
    if(Velocity_mode==1 && Fuzzy_line.Scan_interrupt_flag<=39) //mode == 1  20000 bangbang
    {
      Velocity_mode=2;
    }
    if(Velocity_mode==2 && Fuzzy_line.Scan_interrupt_flag>=55)
    {
      Velocity_mode_2to3_flag=1;
    }
    else
      Velocity_mode_2to3_flag=0;
    if(Time_Velocity_mode_2to3>=0.3)
      Velocity_mode=0;
    /////////////////输出模式信息/////////////////
    if(Velocity_mode_last != Velocity_mode)
    {
      Velocity_mode_last=Velocity_mode;
      DEBUG_PRINTF("          MD=%d\n",Velocity_mode);//输出停车信息
    }
    //////////////////////////////////////////////
  }
  
  if(Time<=Start_Time && Velocity_mode==0)
    PID_Votor.SetPoint=(int)(Time*Velocity_max/Start_Time);//起步定时加速
  else
  {
    if(Velocity_mode==1 && Velocity_max!=0 && Velocity_max > Velocity_min)
    {
      if(abs(Error_middleline)>=22000) PID_Votor.SetPoint=Velocity_min-Stop_Stop_accelerating_Velocity;
      if(abs(Error_middleline)<=4000) PID_Votor.SetPoint=Velocity_max;
      if(abs(Error_middleline)>4000 && abs(Error_middleline)<22000) PID_Votor.SetPoint=Velocity_max-(Velocity_max-Velocity_min+Stop_Stop_accelerating_Velocity)/(22000-4000)*(Error_middleline-4000);
    }
    if(Velocity_mode==2 && Velocity_max!=0 && Velocity_max > Velocity_min) PID_Votor.SetPoint=Velocity_min;
    if(Velocity_mode==0 && Velocity_max!=0 && Velocity_max > Velocity_min) 
    {
      if(abs(Error_middleline)>=17000) PID_Votor.SetPoint=Velocity_min;
      if(abs(Error_middleline)<=4000) PID_Votor.SetPoint=Velocity_max;
      if(abs(Error_middleline)>4000 && abs(Error_middleline)<17000) PID_Votor.SetPoint=Velocity_max-(Velocity_max-Velocity_min)/(17000-4000)*(Error_middleline-4000);
    }
    if(Velocity_max <= Velocity_min) PID_Votor.SetPoint=Velocity_max;
  }
  //if(Time>Start_Time && (Velocity_max<=Velocity_min && Velocity_votor<=Velocity_max-50) || (Velocity_max>Velocity_min && Velocity_votor<=Velocity_min-50))//防止冲撞的程序，比赛的时候去掉(考虑到要装挡板)
  if(Time>Start_Time && Velocity_votor <= 30)//防止冲撞的程序，比赛的时候去掉(考虑到要装挡板)
  {
      Velocity_max=0;
      DEBUG_PRINTF("\nSTOP for Zhangai");//输出停车信息
  }
}


void oled_show(void)
{
  int16 i,j,k;
  static uint8 Flag=1;
  if(KEY_Data_B4 == KEY_DOWN)
  {
    if(KEY_Data_B1 == KEY_UP && KEY_Data_B2 == KEY_UP)
    {
      if(Flag == 1)
      {
        OLED_Fill(0x00);  //初始清屏
        Flag=0;
      }
      OLED_PrintImage((uint8*)img,CAMERA_H,CAMERA_W);
    }
    else
    {
      for(i=0;i<CAMERA_H;i++)      //显示图像
      {    
        for(j=0;j<CAMERA_W;j++)
        {
          k=img[i][j];
          OLED_DrawPoint(j,60-i,k);//oled显示点  x， y，亮/灭
        }       
      }
      for(i=CAMERA_H;i>0;i--)      //显示中线
      {
        OLED_DrawPoint(Fuzzy_line.MiddleLine[CAMERA_H-i],60-i,0);//Oled显示函数
      }
      OLED_Refresh_Gram();//oled刷新，要配合画点 
      Flag=1;
    }
    Show_TestData();    //显示测试的数据
  }
  else
  {
    //OLED_ShowStr(37,3,"Run",2);
    OLED_ShowStr_Number(40,4,"V:",Velocity_max,1);
  }
}

