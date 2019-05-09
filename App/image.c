#include "common.h"
#include "include.h"
extern uint8 img[CAMERA_H][CAMERA_W];
extern PID   PID_DuoJi;//���
extern PID   PID_Votor;//���
extern int PWM_DuoJi; //�����PWMֵ
extern int Velocity_max;
extern int Velocity_min;
extern KEY_STATUS_e KEY_Data_B1;
extern KEY_STATUS_e KEY_Data_B2;
extern KEY_STATUS_e KEY_Data_B3;
extern int Velocity_votor;
extern unsigned char Option_Mode_1;//ģʽһѡ��
extern unsigned char Option_Mode_2;//ģʽ��ѡ��
extern unsigned char Option_Mode_3;//ģʽ��ѡ��
extern double Stop_Stop_accelerating_Time;//������ʱ��
extern int Stop_Stop_accelerating_Velocity;//�������ٶȲ�
extern double Time;
extern char Over_flag;
extern char Over_duoji_flag;
extern double Start_Time;
extern uint8 Guanxian;

int Fps=0;
char Fps_flag=0;
int Error_middleline=0;      //��ֵ��Ȩ��������
//int Weight[60]={0};    //����Ȩ��ֵ
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
int Weight_Sum=0; //��������Ȩ�ص���ֵ
int Weight_max=30;//����Ȩ�������к�
double Error_k=0;            //��ֵ��kֵ

//Fuzzy_line_finding Fuzzy_line={0,{0},  {0},0,  {80},0,  40,{40},  1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
Fuzzy_line_finding Fuzzy_line;
/*unsigned int Number_Ahead=0;//��¼ǰ���е���ֵ�������ƽ��
char Track_width[60]={0};//������������

char Left_Black[60]={0};//�洢����ߣ�һά���飩 Ĭ�ϵ�һ��ֵΪ0
char Left_Black_flag=0; //������ж�λ

char Right_Black[60]={80};//�洢�ұ��ߣ�һά���飩Ĭ�ϵ�һ��ֵΪ80
char Right_Black_flag=0;  //�ұ����ж�λ

char MiddleLine_old=40;  //������һ�ε��м���
char MiddleLine[60]={40};//�洢�м��ߣ�һά���飩Ĭ�ϵ�һ��ֵΪ40

char Scan_interrupt_flag=1;//ɨ���ж�λ
char Scan_interrupt_flag_flag=0;//ɨ���ж�λ�ı�־λ�����ٶȵĿ����й�ϵ��
char Average_flag=0;       //ƽ��ֵ��־λ����һ֡��
char Average_line_number_flag=0;    //ƽ��ֵ�����ı�־����һ֡��
char Average_line_number_flag_old=0;//ƽ��ֵ�����ı�־����һ֡��
char crossroads_flag=0;    //ʮ��·�ڱ�־λ(��һ֡)
char crossroads_flag_old=0;//ʮ��·�ڱ�־λ(��һ֡)
char crossroads_Left_flag=0; //ʮ��·�����־λ(��һ֡)
char crossroads_Left_flag_old=0; //ʮ��·�����־λ(��һ֡)
char crossroads_Right_flag=0;//ʮ��·���ұ�־λ(��һ֡)
char crossroads_Right_flag_old=0;//ʮ��·���ұ�־λ(��һ֡)
char crossroads_Left_Right_flag=0;    //ʮ��·�����ұ�־λ
char crossroads_Left_Right_flag_old=0;//ʮ��·�����ұ�־λ����һ֡��
int Direction_flag=0;    //�����־λ
int Direction_flag_old=0;//�����־λ(��һ֡)

char Left_black_line_flag=0;//����߱�־λ(��һ֡)
char Left_black_line_flag_old=0;//����߱�־λ(��һ֡)
char Right_black_line_flag=0;//�Һ��߱�־λ(��һ֡)
char Right_black_line_flag_old=0;//�Һ��߱�־λ(��һ֡)*/

///////////////////////////////////////////
int Road_length=0;

void Image_scanning()
{
  //ZuoHuanDaoIn();
  //YouHuanDaoIn();
  
  
  ///////////֡�ʼ��///////////
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
  Fuzzy_line_finding_method(&Fuzzy_line);//ģ��Ѱ�߷�
  //////////////////////////////
  Error_Handle_Jiaquan(&Fuzzy_line);//��Ȩ�������
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
}

void Fuzzy_line_finding_method(Fuzzy_line_finding *line)//ģ��Ѱ�߷�
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
    //break;//////////////////////////////////////////////////////////////////////////////////////////////////���Լ��ĳ����ʱ��Ҫɾ��
    if(i>CAMERA_H-4) //ǰ������ƽ�� ȷ����ʼ�е�
    {
      Addition_and_subtraction_judgment(i,line);//���߼Ӽ���
      line->Number_Ahead=line->MiddleLine[CAMERA_H-i-1]+line->Number_Ahead;
      if(i == CAMERA_H-3)
      {
        line->Number_Ahead=line->Number_Ahead/3;
        line->MiddleLine_old=line->Number_Ahead;
      }
      continue;
    }
    Follow_the_judgment(i,line);//���߸��淨
    /******************************���˺���*************************************/ 
       
    if(line->MiddleLine[CAMERA_H-i-1]>75-filter_column)   //����û������ʱ������ʾ�������ߺͼ�ɨ���жϱ�־
    {
      line->Scan_interrupt_flag=CAMERA_H-i-1;
      j=0;
      for(i=i;i>=0;i--)
      {
        j++;
        line->Direction_flag++;
        line->MiddleLine[CAMERA_H-i-1]=80+j;
      }
      line->Direction_flag_old=line->Direction_flag;//����֡һ������
      break;
    }              
    else {if((CAMERA_H-i-1)==59) line->Scan_interrupt_flag=CAMERA_H-i-1;}
    
    if(line->MiddleLine[CAMERA_H-i-1]<4+filter_column)   //����û������ʱ������ʾ�������ߺͼ�ɨ���жϱ�־
    {
      line->Scan_interrupt_flag=CAMERA_H-i-1;
      j=0;
      for(i=i;i>=0;i--)
      {
        j++;
        line->Direction_flag--;
        line->MiddleLine[CAMERA_H-i-1]=0-j;
      }
      line->Direction_flag_old=line->Direction_flag;//����֡һ������
      break;
    }
    else {if((CAMERA_H-i-1)==59) line->Scan_interrupt_flag=CAMERA_H-i-1;}
    
    if(Absolute(line->Left_Black[CAMERA_H-i-1],line->Right_Black[CAMERA_H-i-1])<10)//�������߳����������������˵�����յ��ߵ����//&& ((MiddleLine[CAMERA_H-i-1]>50) || (MiddleLine[CAMERA_H-i-1]<30))
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
        line->Direction_flag_old=line->Direction_flag;//����֡һ������
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
        line->Direction_flag_old=line->Direction_flag;//����֡һ������
        break;
      }
    }
    else {if((CAMERA_H-i-1)==CAMERA_H-1) line->Scan_interrupt_flag=CAMERA_H-i-1;}
      //if(Absolute(Left_Black[CAMERA_H-i-1],Right_Black[CAMERA_H-i-1])>60 && (CAMERA_H-i-1)>30 && crossroads_flag_old<=5)
        //crossroads_Left_Right_flag=crossroads_Left_Right_flag+1;//////////////////������--ǰ��ɨ������������60����
    
      //if(crossroads_flag_old > 0 || crossroads_Left_Right_flag_old >4)//����������
    
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
    line->MiddleLine_old=line->MiddleLine[CAMERA_H-i-1];//ÿ�θ������ߺ����Ҫ��MiddleLine_oldֵ  ���Բ���ע�͵�
  }///////////////////////////////////////////��FORѭ������//Ȼ��ѱ�־λ����һ֡�����ж�
  line->Average_line_number_flag_old=line->Average_line_number_flag;
  line->Left_black_line_flag_old=line->Left_black_line_flag;
  line->Right_black_line_flag_old=line->Right_black_line_flag;
  line->crossroads_Left_flag_old=line->crossroads_Left_flag;
  line->crossroads_Right_flag_old=line->crossroads_Right_flag;
  if((line->crossroads_Left_flag>5 && line->crossroads_Right_flag>5) || line->crossroads_Left_flag>20 || line->crossroads_Right_flag>20)
    line->crossroads_Left_Right_flag_old=(line->crossroads_Left_flag+line->crossroads_Right_flag)/2;
  else
    line->crossroads_Left_Right_flag_old=0;
  line->crossroads_flag_old=line->crossroads_flag;//���ʮ��·�ڱ�־λ
  line->Direction_flag_old=line->Direction_flag;//����֡һ������
}

/****************************����ɨ�躯��**************************************/
void Left_and_right_scanning(int16 i,Fuzzy_line_finding *line)
{
    int16 j;
    for(j=line->MiddleLine_old-1;j>0+filter_column;j--)        // ���м������������Ѱ�Һڵ�
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
          line->Left_Black[CAMERA_H-i-1]=0;  // δ�ҵ���ߺڵ�
        }
      }
    }
    
    for(j=line->MiddleLine_old;j<CAMERA_W-1-filter_column;j++)  // ���м����ұ�������Ѱ�Һڵ�
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
          line->Right_Black[CAMERA_H-i-1]=80;  //δ�ҵ��ұߺڵ�
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

/****************************�Ӽ��ж�******************************************/
void Addition_and_subtraction_judgment(int16 i,Fuzzy_line_finding *line)                                 
{
    Left_and_right_scanning(i,line);
    if(line->Left_Black_flag!=0 && line->Right_Black_flag!=0)//���ұ���ͬʱ���ڵ����
    {
      line->Average_flag=1;
      line->Average_line_number_flag++;
      line->MiddleLine[CAMERA_H-i-1]=(line->Left_Black[CAMERA_H-i-1] + line->Right_Black[CAMERA_H-i-1])/2;
    }
    
    if(line->Left_Black_flag==0 && line->Right_Black_flag==0)//����һ�м�ⲻ�����ߵ����    
    {
      line->Average_flag=0;
      if(CAMERA_H-i-1<45)
        line->crossroads_flag++;
      line->MiddleLine[CAMERA_H-i-1]=(line->Left_Black[CAMERA_H-i-1]+line->Right_Black[CAMERA_H-i-1])/2;
      //MiddleLine[CAMERA_H-i-1]=40;
    }
    
    if(line->Left_Black_flag==0 && line->Right_Black_flag!=0)//ֻ���ұ��ߵ����
    {
      line->Average_flag=0;
      line->MiddleLine[CAMERA_H-i-1]=line->Right_Black[CAMERA_H-i-1]-line->Track_width[CAMERA_H-i-1];//��ȥ��������
    }
    
    if(line->Left_Black_flag!=0 && line->Right_Black_flag==0)//ֻ������ߵ����
    {
      line->Average_flag=0;
      line->MiddleLine[CAMERA_H-i-1]=line->Left_Black[CAMERA_H-i-1]+line->Track_width[CAMERA_H-i-1];//������������
    }
    
    line->MiddleLine_old=line->MiddleLine[CAMERA_H-i-1];
}

/****************************�������ж�****************************************/

void Follow_the_judgment(int16 i,Fuzzy_line_finding *line)
{
    Left_and_right_scanning(i,line);
    if(line->Left_Black_flag!=0 && line->Right_Black_flag!=0)//���ұ���ͬʱ���ڵ����
    {
      line->Average_flag=1;
      line->Average_line_number_flag++;
      line->MiddleLine[CAMERA_H-i-1]=(line->Left_Black[CAMERA_H-i-1] + line->Right_Black[CAMERA_H-i-1])/2;
    }
    
    if(line->Left_Black_flag==0 && line->Right_Black_flag==0)//����һ�м�ⲻ�����ߵ����    
    {                
      line->Average_flag=0;
      if(CAMERA_H-i-1<45)
        line->crossroads_flag++;
      line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine_old;
      //MiddleLine[CAMERA_H-i-1]=40;
    }
    
    if(line->Left_Black_flag==0 && line->Right_Black_flag!=0)//ֻ���ұ��ߵ����
    {
      line->Average_flag=0;
      if(CAMERA_H-i-1<30)
        line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine_old+line->Right_Black[CAMERA_H-i-1] - line->Right_Black[CAMERA_H-i-2]+line->Track_width[CAMERA_H-i-2]-line->Track_width[CAMERA_H-i-1];//���߸����ұ߽�
      else
        line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine_old+line->Right_Black[CAMERA_H-i-1] - line->Right_Black[CAMERA_H-i-2];//���߸����ұ߽�
    }
    
    if(line->Left_Black_flag!=0 && line->Right_Black_flag==0)//ֻ������ߵ����
    {
      line->Average_flag=0;
      if(CAMERA_H-i-1<30)
        line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine_old+line->Left_Black[CAMERA_H-i-1] - line->Left_Black[CAMERA_H-i-2]-line->Track_width[CAMERA_H-i-2]+line->Track_width[CAMERA_H-i-1];//���߸�����߽�
      else
        line->MiddleLine[CAMERA_H-i-1]=line->MiddleLine_old+line->Left_Black[CAMERA_H-i-1] - line->Left_Black[CAMERA_H-i-2];//���߸�����߽�
    }
    
    if(line->MiddleLine[CAMERA_H-i-1]>line->MiddleLine_old && Absolute(line->MiddleLine[CAMERA_H-i-1],line->MiddleLine_old)<5 && (CAMERA_H-i-1)<20 && (CAMERA_H-i-1)>4)
    {
      line->Direction_flag=line->Direction_flag+(20-(CAMERA_H-i-1));//��Ȩ�ش���
      if(line->Average_flag==1)
        line->Direction_flag=line->Direction_flag+(20-(CAMERA_H-i-1));//��Ȩ�ش���
    }
    if(line->MiddleLine[CAMERA_H-i-1]<line->MiddleLine_old && Absolute(line->MiddleLine[CAMERA_H-i-1],line->MiddleLine_old)<5 && (CAMERA_H-i-1)<20 && (CAMERA_H-i-1)>4)
    {
      line->Direction_flag=line->Direction_flag-(20-(CAMERA_H-i-1));//��Ȩ�ش���
      if(line->Average_flag==1)
        line->Direction_flag=line->Direction_flag-(20-(CAMERA_H-i-1));//��Ȩ�ش���
    }

    line->MiddleLine_old=line->MiddleLine[CAMERA_H-i-1];
}

///////////////////////////////////////////////////////////////////////////////////////
void Error_Handle_Jiaquan(Fuzzy_line_finding *line)//�ü�Ȩ�ķ�ʽ�������
{
  int i;
  Error_middleline=0;
  for(i=0;i<=CAMERA_H-1;i++)
  {
    Error_middleline=((int)line->MiddleLine[i]-39)*Weight[i]*2+Error_middleline;
  }
}

double GetRoadErr3(int *line)//��б�ʵķ����������
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
char Absolute(char a,char b)//�жϲ�ֵ
{
  char c;
  if(a>=b)
    c=a-b;
  else
    c=b-a;
  return c;
}

void Track_width_init(Fuzzy_line_finding *line)//��ʼ����������
{
  int16 i;
  float  Width=30;
  for(i=0;i<CAMERA_H;i++)
  {
    Width=Width-0.33;
    line->Track_width[i]=(int)Width;
  }
}

void Weight_init(int Weight_Max)//��ʼ�����е�Ȩ��  Weight_Max��Ȩ�����ص��кţ����������������
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
void Show_TestData()//��ʾ���Ե�����
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
      OLED_ShowStr_Number(80,6,"E:",Error_middleline,1);//��ֵError_middleline
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
