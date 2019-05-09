#include "common.h"
#include "include.h"
extern PID   PID_DuoJi;//���PID
extern PID   PID_Votor;//���PID
extern PID_INIT PID_Init;
extern Fuzzy_line_finding Fuzzy_line;
volatile uint32 irqflag = 0;     //��־λ���壬��0 ��ʾ�����ж�
extern uint8 img[CAMERA_H][CAMERA_W];
extern int Weight_max;//����Ȩ�������к�

//extern unsigned char Scan_interrupt_flag;//ɨ��ֹͣλ ����ĳ��򣡣�
extern int Error_middleline;      //��Ȩ��Ĳ�ֵ
extern int Error_k;               //���ߵ�kֵ
extern char Fps;                  //��¼��fpsֵ
extern char Fps_flag;             //Fps�ı�־λ

uint8 Guanxian = 60;      //��ͼ���ֵ������ֵ
char Over_flag=0;         //�ܽ�����־λ
char Start_flag=1;        //��ʼ�ܱ�־λ

unsigned char Option_Mode_1=0;//ģʽһѡ��
unsigned char Option_Mode_2=0;//ģʽ��ѡ��
unsigned char Option_Mode_3=0;//ģʽ��ѡ��
int PWM_DuoJi=Median_of_rudder;  //��������PWM
int Speed=0;
int Speed_Max=800;   //���������޷�

int Velocity_max=0;  //����ٶ�
int Velocity_min=180;//��С����ٶ�

double Start_Time=1.2;//��ʼ����ʱ��
int Start_to_speed_up=17000;//��ʼ���� 7000
int Stop_accelerating=20000;//ֹͣ���� 25000
int Stop_Stop_accelerating=17000;//��ֹͣ����
char Stop_Stop_accelerating_flag=0;//�����ٱ�־λ
double Stop_Stop_accelerating_Time=0.2;//������ʱ��
int Stop_Stop_accelerating_Velocity=45;//�������ٶȲ�
int Speed_up_coefficient=0; //ϵ��(��һ���Ŀ����㷨�Ĳ���)

char Velocity_mode=0;//�ٶȲ��Ե�ģʽ
char Velocity_mode_last=0;//�ٶȲ��Ե�ģʽ(��һ�ε�)
char Velocity_mode_2to3_flag=0;//���ɸ��ٵı�־
int Bang_Pwm=200;
int Bang_Error=20000;

int Velocity_votor=0;//���������ֵ���ٶȣ�


double Time=0;                        //��ʱ��
double Time_Fps=0;                    //���FPS��ʱ��
double Time_Stop_Stop_accelerating=0; //�������õ���ʱ��
double Time_over=0;                   //���ڽ�����ʱ��
double Time_Velocity_mode_2to3=0;     //�����Ƿ���ɸ��ٵ��ж�

char Show_Time_flag=0; //��ʾ����ʱ���־λ

char DuoJi_Flag=1; //�������õĶ�����ر�־
KEY_e KEY_Data;
KEY_STATUS_e KEY_Data_B1;
KEY_STATUS_e KEY_Data_B2;
KEY_STATUS_e KEY_Data_B3;
KEY_STATUS_e KEY_Data_B4;

int16 buff[3] = {0xfc03,0,0x03fc};//��λ���������ݵ�Э��
void PIT2_IRQHandler(void)
{
    /////////////����֡����///////////////
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
    
    //////////��ʱ�����������ʱ��////////
    if(Velocity_max!=0)
    { 
      Show_Time_flag=1;
      Time=Time+0.01;
    }
    else
    {
      PID_Votor.SumError=0;//ֹͣʱ��������
      if(Show_Time_flag==1)
      {
        printf("Time:%0.3fs\r\n",Time);
        Time=0;
        Show_Time_flag=0;
      }
    }
    //////////////////////////////////////
    
    PIT_Flag_Clear(PIT2);  //���жϱ�־λ
}

void PIT0_IRQHandler(void)
{
    //////////////���ڽ�����ʱ/////////////
    if(Over_flag==1)
    {
      StartAndFinish();
      Time_over=Time_over+0.01;
      if(Time_over>=0.2)
      {
        Time_over=0;
        Over_flag=0;
        Velocity_max=0;
        DEBUG_PRINTF("\nSTOP for Banmaxian");//���ͣ����Ϣ
      }
    }
    //////////////////////////////////////
    
    /////////////��ֹͣ����///////////////
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
    
    //////////���ڵ������////////////////
    Velocity_votor = -FTM_QUAD_get(FTM1);   //��ȡFTM �������� ��������(������ʾ������)
    buff[1] = Velocity_votor;
    //uart_putbuff(UART3, (uint8*)(buff),6);//����λ����������
    if(KEY_Data_B3 == KEY_UP)
    {
      Set_Velocity_votor();
    }
    else
    {
      PID_Votor.SumError=0;//ֹͣʱ��������
      FTM_PWM_Duty(FTM0, FTM_CH1,0);
      FTM_PWM_Duty(FTM0, FTM_CH2,0);
    }//���ڵ������
    //////////////////////////////////////
    
    FTM_QUAD_clean(FTM1);
    PIT_Flag_Clear(PIT0);  //���жϱ�־λ
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
  Pid_refresh();//ˢ��PID����(ÿ�θ���PID����Ҫ��)
  

  //////////////////////////���ƶ����ֵ///////////////////////////
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
    /////////////////���ģʽ��Ϣ/////////////////
    if(Velocity_mode_last != Velocity_mode)
    {
      Velocity_mode_last=Velocity_mode;
      DEBUG_PRINTF("          MD=%d\n",Velocity_mode);//���ͣ����Ϣ
    }
    //////////////////////////////////////////////
  }
  
  if(Time<=Start_Time && Velocity_mode==0)
    PID_Votor.SetPoint=(int)(Time*Velocity_max/Start_Time);//�𲽶�ʱ����
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
  //if(Time>Start_Time && (Velocity_max<=Velocity_min && Velocity_votor<=Velocity_max-50) || (Velocity_max>Velocity_min && Velocity_votor<=Velocity_min-50))//��ֹ��ײ�ĳ��򣬱�����ʱ��ȥ��(���ǵ�Ҫװ����)
  if(Time>Start_Time && Velocity_votor <= 30)//��ֹ��ײ�ĳ��򣬱�����ʱ��ȥ��(���ǵ�Ҫװ����)
  {
      Velocity_max=0;
      DEBUG_PRINTF("\nSTOP for Zhangai");//���ͣ����Ϣ
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
        OLED_Fill(0x00);  //��ʼ����
        Flag=0;
      }
      OLED_PrintImage((uint8*)img,CAMERA_H,CAMERA_W);
    }
    else
    {
      for(i=0;i<CAMERA_H;i++)      //��ʾͼ��
      {    
        for(j=0;j<CAMERA_W;j++)
        {
          k=img[i][j];
          OLED_DrawPoint(j,60-i,k);//oled��ʾ��  x�� y����/��
        }       
      }
      for(i=CAMERA_H;i>0;i--)      //��ʾ����
      {
        OLED_DrawPoint(Fuzzy_line.MiddleLine[CAMERA_H-i],60-i,0);//Oled��ʾ����
      }
      OLED_Refresh_Gram();//oledˢ�£�Ҫ��ϻ��� 
      Flag=1;
    }
    Show_TestData();    //��ʾ���Ե�����
  }
  else
  {
    //OLED_ShowStr(37,3,"Run",2);
    OLED_ShowStr_Number(40,4,"V:",Velocity_max,1);
  }
}

