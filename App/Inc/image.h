#ifndef __IMAGE_H__
#define __IMAGE_H__

#define ERR2STAR 20
#define ERR2LONG 30

#define MAX_N 100       //(x_i,y_i)�����ά��
#define filter_column 3//��������(��ֹ����ͷ�ı߲�©��)

typedef struct tagPOINT //�ṹ����б���õ�
{
	double x;
	double y;
} POINT;



typedef struct Fuzzy_line_finding //�ṹ��������ģ��Ѱ�ߵ�����
{
  unsigned int Number_Ahead;//��¼ǰ���е���ֵ�������ƽ��
  char Track_width[60];//������������

  char Left_Black[60];//�洢����ߣ�һά���飩 Ĭ�ϵ�һ��ֵΪ0
  char Left_Black_flag; //������ж�λ

  char Right_Black[60];//�洢�ұ��ߣ�һά���飩Ĭ�ϵ�һ��ֵΪ80
  char Right_Black_flag;  //�ұ����ж�λ

  int MiddleLine_old;  //������һ�ε��м���
  int MiddleLine[60];//�洢�м��ߣ�һά���飩Ĭ�ϵ�һ��ֵΪ40

  char Scan_interrupt_flag;//ɨ���ж�λ
  char Scan_interrupt_flag_flag;//ɨ���ж�λ�ı�־λ�����ٶȵĿ����й�ϵ��
  char Average_flag;       //ƽ��ֵ��־λ����һ֡��
  char Average_line_number_flag;    //ƽ��ֵ�����ı�־����һ֡��
  char Average_line_number_flag_old;//ƽ��ֵ�����ı�־����һ֡��
  char crossroads_flag;    //ʮ��·�ڱ�־λ(��һ֡)
  char crossroads_flag_old;//ʮ��·�ڱ�־λ(��һ֡)
  char crossroads_Left_flag; //ʮ��·�����־λ(��һ֡)
  char crossroads_Left_flag_old; //ʮ��·�����־λ(��һ֡)
  char crossroads_Right_flag;//ʮ��·���ұ�־λ(��һ֡)
  char crossroads_Right_flag_old;//ʮ��·���ұ�־λ(��һ֡)
  char crossroads_Left_Right_flag;    //ʮ��·�����ұ�־λ
  char crossroads_Left_Right_flag_old;//ʮ��·�����ұ�־λ����һ֡��
  int Direction_flag;    //�����־λ
  int Direction_flag_old;//�����־λ(��һ֡)

  char Left_black_line_flag;//����߱�־λ(��һ֡)
  char Left_black_line_flag_old;//����߱�־λ(��һ֡)
  char Right_black_line_flag;//�Һ��߱�־λ(��һ֡)
  char Right_black_line_flag_old;//�Һ��߱�־λ(��һ֡)
}Fuzzy_line_finding;

void Track_width_init(Fuzzy_line_finding *line);//��ʼ���������
void Left_and_right_scanning(int16 i,Fuzzy_line_finding *line);//����ɨ�����
void Addition_and_subtraction_judgment(int16 i,Fuzzy_line_finding *line);//���߼Ӽ���
void Follow_the_judgment(int16 i,Fuzzy_line_finding *line);//���߸��淨
void Fuzzy_line_finding_method(Fuzzy_line_finding *line);//ģ��Ѱ�߷�

void Weight_init(int Weight_Max);//��ʼ�����е�Ȩ��
void Error_Handle_Jiaquan(Fuzzy_line_finding *line);//��ֵ��Ȩ��ӷ�
double GetRoadErr3(int *line);//��С���˷����ֵ

void Image_scanning();
void Show_TestData();//��ʾ���Ե�����

char Absolute(char a,char b);

extern int Error_middleline;

#endif