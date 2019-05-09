#ifndef __PID_H__
#define __PID_H__


typedef struct PID
{
    int  SetPoint;     //设定目标 Desired Value
    long SumError;                //误差累计

    double  Proportion;         //比例常数 Proportional Cons
    double  Integral;           //积分常数 Integral Const
    double  Derivative;         //微分常数 Derivative Const

    int LastError;               //Error[-1]
    int PrevError;               //Error[-2]
} PID;

typedef struct PID_INIT
{
    double  DuoJi_Proportion;         //舵机的比例常数 Proportional Cons
    double  DuoJi_Integral;           //舵机的积分常数 Integral Const
    double  DuoJi_Derivative;         //舵机的微分常数 Derivative Const

    double  DuoJi_R_Proportion;         //舵机的比例常数 Proportional Cons
    double  DuoJi_R_Integral;           //舵机的积分常数 Integral Const
    double  DuoJi_R_Derivative;         //舵机的微分常数 Derivative Const
    
    double  Votor_Proportion;         //电机的比例常数 Proportional Cons
    double  Votor_Integral;           //电机的积分常数 Integral Const
    double  Votor_Derivative;         //电机的微分常数 Derivative Const
}PID_INIT;

void Pid_init();////PID初始化
void Pid_refresh();//刷新PID参数

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

