#ifndef __PID_H__
#define __PID_H__


typedef struct PID
{
    int  SetPoint;     //�趨Ŀ�� Desired Value
    long SumError;                //����ۼ�

    double  Proportion;         //�������� Proportional Cons
    double  Integral;           //���ֳ��� Integral Const
    double  Derivative;         //΢�ֳ��� Derivative Const

    int LastError;               //Error[-1]
    int PrevError;               //Error[-2]
} PID;

typedef struct PID_INIT
{
    double  DuoJi_Proportion;         //����ı������� Proportional Cons
    double  DuoJi_Integral;           //����Ļ��ֳ��� Integral Const
    double  DuoJi_Derivative;         //�����΢�ֳ��� Derivative Const

    double  DuoJi_R_Proportion;         //����ı������� Proportional Cons
    double  DuoJi_R_Integral;           //����Ļ��ֳ��� Integral Const
    double  DuoJi_R_Derivative;         //�����΢�ֳ��� Derivative Const
    
    double  Votor_Proportion;         //����ı������� Proportional Cons
    double  Votor_Integral;           //����Ļ��ֳ��� Integral Const
    double  Votor_Derivative;         //�����΢�ֳ��� Derivative Const
}PID_INIT;

void Pid_init();////PID��ʼ��
void Pid_refresh();//ˢ��PID����

void PID_DuoJi_Init(void);
int IncPIDCalc(int NextPoint);
int LocPID_DuoJi(int NextPoint);
void PIDSetPoint(int SetPoint);
int PIDGetPoint(void);//
void PIDSetKp(double Kp);
double PIDGetKp(void);//
void PIDSetKi(double Ki);
double PIDGetKi(void);//
void PIDSetKd(double Kd);
double PIDGetKd(void);//
int PIDGetLastError(void);

void PID_Votor_Init(void);
int LocPID_Votor(int NextPoint);
int IncPIDCalc_Votor(int NextPoint);
#endif

