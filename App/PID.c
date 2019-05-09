#include "common.h"
#include "include.h"

//static PID   sPID;
//static PID   *sptr_DuoJi = &sPID;

PID   PID_DuoJi;//舵机（不可操作，只能刷新）
PID   *sptr_DuoJi = &PID_DuoJi;

PID   PID_Votor;//电机（不可操作，只能刷新）
PID   *sptr_Votor = &PID_Votor;

PID   PID_DuoJi_R;//舵机右
PID   *sptr_DuoJi_R = &PID_DuoJi_R;

PID_INIT PID_Init;//可操作的PID参数

/*******************************************************************************
* 函数名称 : Pid_init
* 函数描述 : PID 初始化
* 函数输入 : 无
* 函数输出 : 无
* 函数返回 : 无
*******************************************************************************/
void Pid_init()
{
  PID_DuoJi_Init();   //舵机的PID参数初始化(置零)
  PID_Votor_Init();   //电机的PID参数初始化(置零)
  
  PID_Init.DuoJi_Proportion=0.0090;//p: 0.0048 - 0.0090 起始到弯道 error :0 - 34000 0.0090 = 0.0001*((error/1000) * (error/1000))/27.5238095 + 0.0048
  PID_Init.DuoJi_Integral=0;
  PID_Init.DuoJi_Derivative=0.0105;
  
  PID_Init.DuoJi_R_Proportion=0.0090;//p: 0.0048 - 0.0090 起始到弯道 error :0 - 34000 0.0090 = 0.0001*((error/1000) * (error/1000))/27.5238095 + 0.0048
  PID_Init.DuoJi_R_Integral=0;
  PID_Init.DuoJi_R_Derivative=0.027;
  
  PID_Init.Votor_Proportion=0.8;
  PID_Init.Votor_Integral=0;    
  PID_Init.Votor_Derivative=5.0;
  
  Pid_refresh();//刷新PID参数
}

/*调试参数日志
2018.7.23 速度：170 P:0.009 D:0.027(D值 正在向上调试中)
          速度：170 P:0.009 D:0(P值 正在调试中)
2018.7.23 120速度的情况 ： 0.0448 0.0252 0.0189(两次抖动) 0.0141(一次抖动) 0.0105（不抖了） 有些切内道 考虑以后过高速的情况
          130速度的情况 ： 0.0105（稳定）有些切内道
          140速度的情况 ： 0.0105（稳定）不切内道 预测正确 是因为p提前给大了
          150速度的情况 ： 0.0105（稳定）小圆环切内道 大圆环正常稳定
          160速度的情况 ： 0.0105（稳定）小圆环意境不内切了
          170速度的情况 ： 0.0105（不稳）可能响应有点点慢 有些许弯道拐慢的现象
                           0.0115（不如0.0105稳）过小于50cm的弯道 出现响应慢 部分正常弯道 响应慢 无明显甩尾现象（可能和最小转弯半径<45cm>有关系）
          总结：可能和D值有关 弯道半径太小也有可能 。。
*/
/*******************************************************************************
* 函数名称 : Pid_refresh
* 函数描述 : PID 刷新值
* 函数输入 : 无
* 函数输出 : 无
* 函数返回 : 无
*******************************************************************************/
void Pid_refresh()
{
  PID_DuoJi.Proportion=PID_Init.DuoJi_Proportion;
  PID_DuoJi.Integral=PID_Init.DuoJi_Integral;
  PID_DuoJi.Derivative=PID_Init.DuoJi_Derivative;
  
  PID_DuoJi_R.Proportion=PID_Init.DuoJi_Proportion * 1.148;
  PID_DuoJi_R.Integral=PID_Init.DuoJi_Integral * 1.148;
  PID_DuoJi_R.Derivative=PID_Init.DuoJi_Derivative * 1.148;
  
  PID_Votor.Proportion=PID_Init.Votor_Proportion;
  PID_Votor.Integral=PID_Init.Votor_Integral;    
  PID_Votor.Derivative=PID_Init.Votor_Derivative;
}

/*******************************************************************************
* 函数名称 : IncPIDInit
* 函数描述 : PID 参数初始化
* 函数输入 : 无
* 函数输出 : 无
* 函数返回 : 无
*******************************************************************************/
void PID_DuoJi_Init(void)
{
    sptr_DuoJi->SumError  = 0;
    sptr_DuoJi->LastError = 0;    //Error[-1]
    sptr_DuoJi->PrevError = 0;    //Error[-2]

    sptr_DuoJi->Proportion = 0;    //比例常数 Proportional Const
    sptr_DuoJi->Integral   = 0;    //积分常数 Integral Const
    sptr_DuoJi->Derivative = 0;    //微分常数 Derivative Const
    sptr_DuoJi->SetPoint   = 0;
    
    sptr_DuoJi_R->SumError  = 0;
    sptr_DuoJi_R->LastError = 0;    //Error[-1]
    sptr_DuoJi_R->PrevError = 0;    //Error[-2]

    sptr_DuoJi_R->Proportion = 0;    //比例常数 Proportional Const
    sptr_DuoJi_R->Integral   = 0;    //积分常数 Integral Const
    sptr_DuoJi_R->Derivative = 0;    //微分常数 Derivative Const
    sptr_DuoJi_R->SetPoint   = 0;
}

/*******************************************************************************
* 函数名称 : IncPIDInit
* 函数描述 : PID 参数初始化
* 函数输入 : 无
* 函数输出 : 无
* 函数返回 : 无
*******************************************************************************/
void PID_Votor_Init(void)
{
    sptr_Votor->SumError  = 0;
    sptr_Votor->LastError = 0;    //Error[-1]
    sptr_Votor->PrevError = 0;    //Error[-2]

    sptr_Votor->Proportion = 0;    //比例常数 Proportional Const  1.2 0.006 1.2
    sptr_Votor->Integral   = 0;    //积分常数 Integral Const
    sptr_Votor->Derivative = 0;    //微分常数 Derivative Const
    sptr_Votor->SetPoint   = 0;
}



/*******************************************************************************
* 函数名称 : IncPIDCalc
* 函数描述 : 增量式 PID 控制计算
* 函数输入 : int 当前位置
* 函数输出 : 无
* 函数返回 : 增量式PID结果
*******************************************************************************/
int IncPIDCalc(int NextPoint)
{
    int iError, iIncpid;
    //当前误差
    iError = sptr_DuoJi->SetPoint - NextPoint;
    //增量计算
    /*if(Error_middleline > 0)
    {
      iIncpid = (int)(sptr_DuoJi_R->Proportion * iError               //E[k]项
                - sptr_DuoJi_R->Integral   * sptr_DuoJi->LastError     //E[k－1]项
                + sptr_DuoJi_R->Derivative * sptr_DuoJi->PrevError);   //E[k－2]项
    }
    else
    {
      iIncpid = (int)(sptr_DuoJi->Proportion * iError               //E[k]项
                - sptr_DuoJi->Integral   * sptr_DuoJi->LastError     //E[k－1]项
                + sptr_DuoJi->Derivative * sptr_DuoJi->PrevError);   //E[k－2]项 
    }*/

    iIncpid = (int)(sptr_DuoJi->Proportion * iError               //E[k]项
              - sptr_DuoJi->Integral   * sptr_DuoJi->LastError     //E[k－1]项
              + sptr_DuoJi->Derivative * sptr_DuoJi->PrevError);   //E[k－2]项 
    
    //存储误差，用于下次计算
    sptr_DuoJi->PrevError = sptr_DuoJi->LastError;
    sptr_DuoJi->LastError = iError;
    //返回增量值
    return(iIncpid);
}

/*******************************************************************************
* 函数名称 : LocPIDCalc
* 函数描述 : 位置式 PID 控制计算
* 函数输入 : int 当前位置
* 函数输出 : 无
* 函数返回 : 位置式PID结果
*******************************************************************************/
int LocPID_DuoJi(int NextPoint)
{
    int  iError,dError;

    iError = sptr_DuoJi->SetPoint - NextPoint;       //偏差
    sptr_DuoJi->SumError += iError;       //积分
    dError = iError - sptr_DuoJi->LastError;     //微分
    sptr_DuoJi->LastError = iError;

    return (int)(sptr_DuoJi->Proportion * iError            //比例项
           + sptr_DuoJi->Integral * sptr_DuoJi->SumError   //积分项
           + sptr_DuoJi->Derivative * dError);        //微分项
}
/*******************************************************************************
* 函数名称 : PIDSetPoint
* 函数描述 : 设置PID参数目标值
* 函数输入 : int
* 函数输出 : 无
* 函数返回 : 无
*******************************************************************************/
void PIDSetPoint(int SetPoint)
{
    sptr_DuoJi->SetPoint   = SetPoint;
}
/*******************************************************************************
* 函数名称 : PIDGetPoint
* 函数描述 : 获取PID参数目标值
* 函数输入 : 无
* 函数输出 : 无
* 函数返回 : int
*******************************************************************************/
int PIDGetPoint(void)
{
    return (sptr_DuoJi->SetPoint);
}
/*******************************************************************************
* 函数名称 : PIDSetKp
* 函数描述 : 设置PID参数Kp
* 函数输入 : int
* 函数输出 : 无
* 函数返回 : 无
*******************************************************************************/
void PIDSetKp(double Kp)
{
    sptr_DuoJi->Proportion = Kp;
}
/*******************************************************************************
* 函数名称 : PIDGetKp
* 函数描述 : 获取PID参数Kp
* 函数输入 : 无
* 函数输出 : 无
* 函数返回 : int
*******************************************************************************/
double PIDGetKp(void)
{
    return (sptr_DuoJi->Proportion);
}
/*******************************************************************************
* 函数名称 : PIDSetKi
* 函数描述 : 设置PID参数Ki
* 函数输入 : int
* 函数输出 : 无
* 函数返回 : 无
*******************************************************************************/
void PIDSetKi(double Ki)
{
    sptr_DuoJi->Integral = Ki;
}
/*******************************************************************************
* 函数名称 : PIDGetKi
* 函数描述 : 获取PID参数Ki
* 函数输入 : 无
* 函数输出 : 无
* 函数返回 : int
*******************************************************************************/
double PIDGetKi(void)
{
    return (sptr_DuoJi->Integral);
}
/*******************************************************************************
* 函数名称 : PIDSetKd
* 函数描述 : 设置PID参数Kd
* 函数输入 : int
* 函数输出 : 无
* 函数返回 : 无
*******************************************************************************/
void PIDSetKd(double Kd)
{
    sptr_DuoJi->Derivative = Kd;
}
/*******************************************************************************
* 函数名称 : PIDGetKd
* 函数描述 : 获取PID参数Ki
* 函数输入 : 无
* 函数输出 : 无
* 函数返回 : int
*******************************************************************************/
double PIDGetKd(void)
{
    return (sptr_DuoJi->Derivative);
}

/*******************************************************************************
* 函数名称 : PIDGetLastError
* 函数描述 : 获取PID参数 最后一次误差值
* 函数输入 : 无
* 函数输出 : 无
* 函数返回 : int
*******************************************************************************/
int PIDGetLastError(void)
{
    return (sptr_DuoJi->LastError);
}


/*******************************************************************************
* 函数名称 : LocPIDCalc
* 函数描述 : 位置式 PID 控制计算
* 函数输入 : int 当前位置
* 函数输出 : 无
* 函数返回 : 位置式PID结果
*******************************************************************************/
int LocPID_Votor(int NextPoint)
{
    int  iError,dError;

    iError = sptr_Votor->SetPoint - NextPoint;       //偏差
    sptr_Votor->SumError += iError;       //积分
    dError = iError - sptr_Votor->LastError;     //微分
    sptr_Votor->LastError = iError;

    return (int)(sptr_Votor->Proportion * iError            //比例项
           + sptr_Votor->Integral * sptr_Votor->SumError   //积分项
           + sptr_Votor->Derivative * dError);        //微分项
}

/*******************************************************************************
* 函数名称 : IncPIDCalc
* 函数描述 : 增量式 PID 控制计算
* 函数输入 : int 当前位置
* 函数输出 : 无
* 函数返回 : 增量式PID结果
*******************************************************************************/
int IncPIDCalc_Votor(int NextPoint)
{
    int iError, iIncpid;
    //当前误差
    iError = sptr_Votor->SetPoint - NextPoint;
    //增量计算
    iIncpid = (int)(sptr_Votor->Proportion * iError               //E[k]项
              - sptr_Votor->Integral   * sptr_Votor->LastError     //E[k－1]项
              + sptr_Votor->Derivative * sptr_Votor->PrevError);   //E[k－2]项
    //存储误差，用于下次计算
    sptr_Votor->PrevError = sptr_Votor->LastError;
    sptr_Votor->LastError = iError;
    //返回增量值
    return(iIncpid);
}