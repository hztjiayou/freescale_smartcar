#include "common.h"
#include "include.h"

//static PID   sPID;
//static PID   *sptr_DuoJi = &sPID;

PID   PID_DuoJi;//��������ɲ�����ֻ��ˢ�£�
PID   *sptr_DuoJi = &PID_DuoJi;

PID   PID_Votor;//��������ɲ�����ֻ��ˢ�£�
PID   *sptr_Votor = &PID_Votor;

PID   PID_DuoJi_R;//�����
PID   *sptr_DuoJi_R = &PID_DuoJi_R;

PID_INIT PID_Init;//�ɲ�����PID����

/*******************************************************************************
* �������� : Pid_init
* �������� : PID ��ʼ��
* �������� : ��
* ������� : ��
* �������� : ��
*******************************************************************************/
void Pid_init()
{
  PID_DuoJi_Init();   //�����PID������ʼ��(����)
  PID_Votor_Init();   //�����PID������ʼ��(����)
  
  PID_Init.DuoJi_Proportion=0.0090;//p: 0.0048 - 0.0090 ��ʼ����� error :0 - 34000 0.0090 = 0.0001*((error/1000) * (error/1000))/27.5238095 + 0.0048
  PID_Init.DuoJi_Integral=0;
  PID_Init.DuoJi_Derivative=0.0105;
  
  PID_Init.DuoJi_R_Proportion=0.0090;//p: 0.0048 - 0.0090 ��ʼ����� error :0 - 34000 0.0090 = 0.0001*((error/1000) * (error/1000))/27.5238095 + 0.0048
  PID_Init.DuoJi_R_Integral=0;
  PID_Init.DuoJi_R_Derivative=0.027;
  
  PID_Init.Votor_Proportion=0.8;
  PID_Init.Votor_Integral=0;    
  PID_Init.Votor_Derivative=5.0;
  
  Pid_refresh();//ˢ��PID����
}

/*���Բ�����־
2018.7.23 �ٶȣ�170 P:0.009 D:0.027(Dֵ �������ϵ�����)
          �ٶȣ�170 P:0.009 D:0(Pֵ ���ڵ�����)
2018.7.23 120�ٶȵ���� �� 0.0448 0.0252 0.0189(���ζ���) 0.0141(һ�ζ���) 0.0105�������ˣ� ��Щ���ڵ� �����Ժ�����ٵ����
          130�ٶȵ���� �� 0.0105���ȶ�����Щ���ڵ�
          140�ٶȵ���� �� 0.0105���ȶ��������ڵ� Ԥ����ȷ ����Ϊp��ǰ������
          150�ٶȵ���� �� 0.0105���ȶ���СԲ�����ڵ� ��Բ�������ȶ�
          160�ٶȵ���� �� 0.0105���ȶ���СԲ���⾳��������
          170�ٶȵ���� �� 0.0105�����ȣ�������Ӧ�е���� ��Щ���������������
                           0.0115������0.0105�ȣ���С��50cm����� ������Ӧ�� ����������� ��Ӧ�� ������˦β���󣨿��ܺ���Сת��뾶<45cm>�й�ϵ��
          �ܽ᣺���ܺ�Dֵ�й� ����뾶̫СҲ�п��� ����
*/
/*******************************************************************************
* �������� : Pid_refresh
* �������� : PID ˢ��ֵ
* �������� : ��
* ������� : ��
* �������� : ��
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
* �������� : IncPIDInit
* �������� : PID ������ʼ��
* �������� : ��
* ������� : ��
* �������� : ��
*******************************************************************************/
void PID_DuoJi_Init(void)
{
    sptr_DuoJi->SumError  = 0;
    sptr_DuoJi->LastError = 0;    //Error[-1]
    sptr_DuoJi->PrevError = 0;    //Error[-2]

    sptr_DuoJi->Proportion = 0;    //�������� Proportional Const
    sptr_DuoJi->Integral   = 0;    //���ֳ��� Integral Const
    sptr_DuoJi->Derivative = 0;    //΢�ֳ��� Derivative Const
    sptr_DuoJi->SetPoint   = 0;
    
    sptr_DuoJi_R->SumError  = 0;
    sptr_DuoJi_R->LastError = 0;    //Error[-1]
    sptr_DuoJi_R->PrevError = 0;    //Error[-2]

    sptr_DuoJi_R->Proportion = 0;    //�������� Proportional Const
    sptr_DuoJi_R->Integral   = 0;    //���ֳ��� Integral Const
    sptr_DuoJi_R->Derivative = 0;    //΢�ֳ��� Derivative Const
    sptr_DuoJi_R->SetPoint   = 0;
}

/*******************************************************************************
* �������� : IncPIDInit
* �������� : PID ������ʼ��
* �������� : ��
* ������� : ��
* �������� : ��
*******************************************************************************/
void PID_Votor_Init(void)
{
    sptr_Votor->SumError  = 0;
    sptr_Votor->LastError = 0;    //Error[-1]
    sptr_Votor->PrevError = 0;    //Error[-2]

    sptr_Votor->Proportion = 0;    //�������� Proportional Const  1.2 0.006 1.2
    sptr_Votor->Integral   = 0;    //���ֳ��� Integral Const
    sptr_Votor->Derivative = 0;    //΢�ֳ��� Derivative Const
    sptr_Votor->SetPoint   = 0;
}



/*******************************************************************************
* �������� : IncPIDCalc
* �������� : ����ʽ PID ���Ƽ���
* �������� : int ��ǰλ��
* ������� : ��
* �������� : ����ʽPID���
*******************************************************************************/
int IncPIDCalc(int NextPoint)
{
    int iError, iIncpid;
    //��ǰ���
    iError = sptr_DuoJi->SetPoint - NextPoint;
    //��������
    /*if(Error_middleline > 0)
    {
      iIncpid = (int)(sptr_DuoJi_R->Proportion * iError               //E[k]��
                - sptr_DuoJi_R->Integral   * sptr_DuoJi->LastError     //E[k��1]��
                + sptr_DuoJi_R->Derivative * sptr_DuoJi->PrevError);   //E[k��2]��
    }
    else
    {
      iIncpid = (int)(sptr_DuoJi->Proportion * iError               //E[k]��
                - sptr_DuoJi->Integral   * sptr_DuoJi->LastError     //E[k��1]��
                + sptr_DuoJi->Derivative * sptr_DuoJi->PrevError);   //E[k��2]�� 
    }*/

    iIncpid = (int)(sptr_DuoJi->Proportion * iError               //E[k]��
              - sptr_DuoJi->Integral   * sptr_DuoJi->LastError     //E[k��1]��
              + sptr_DuoJi->Derivative * sptr_DuoJi->PrevError);   //E[k��2]�� 
    
    //�洢�������´μ���
    sptr_DuoJi->PrevError = sptr_DuoJi->LastError;
    sptr_DuoJi->LastError = iError;
    //��������ֵ
    return(iIncpid);
}

/*******************************************************************************
* �������� : LocPIDCalc
* �������� : λ��ʽ PID ���Ƽ���
* �������� : int ��ǰλ��
* ������� : ��
* �������� : λ��ʽPID���
*******************************************************************************/
int LocPID_DuoJi(int NextPoint)
{
    int  iError,dError;

    iError = sptr_DuoJi->SetPoint - NextPoint;       //ƫ��
    sptr_DuoJi->SumError += iError;       //����
    dError = iError - sptr_DuoJi->LastError;     //΢��
    sptr_DuoJi->LastError = iError;

    return (int)(sptr_DuoJi->Proportion * iError            //������
           + sptr_DuoJi->Integral * sptr_DuoJi->SumError   //������
           + sptr_DuoJi->Derivative * dError);        //΢����
}
/*******************************************************************************
* �������� : PIDSetPoint
* �������� : ����PID����Ŀ��ֵ
* �������� : int
* ������� : ��
* �������� : ��
*******************************************************************************/
void PIDSetPoint(int SetPoint)
{
    sptr_DuoJi->SetPoint   = SetPoint;
}
/*******************************************************************************
* �������� : PIDGetPoint
* �������� : ��ȡPID����Ŀ��ֵ
* �������� : ��
* ������� : ��
* �������� : int
*******************************************************************************/
int PIDGetPoint(void)
{
    return (sptr_DuoJi->SetPoint);
}
/*******************************************************************************
* �������� : PIDSetKp
* �������� : ����PID����Kp
* �������� : int
* ������� : ��
* �������� : ��
*******************************************************************************/
void PIDSetKp(double Kp)
{
    sptr_DuoJi->Proportion = Kp;
}
/*******************************************************************************
* �������� : PIDGetKp
* �������� : ��ȡPID����Kp
* �������� : ��
* ������� : ��
* �������� : int
*******************************************************************************/
double PIDGetKp(void)
{
    return (sptr_DuoJi->Proportion);
}
/*******************************************************************************
* �������� : PIDSetKi
* �������� : ����PID����Ki
* �������� : int
* ������� : ��
* �������� : ��
*******************************************************************************/
void PIDSetKi(double Ki)
{
    sptr_DuoJi->Integral = Ki;
}
/*******************************************************************************
* �������� : PIDGetKi
* �������� : ��ȡPID����Ki
* �������� : ��
* ������� : ��
* �������� : int
*******************************************************************************/
double PIDGetKi(void)
{
    return (sptr_DuoJi->Integral);
}
/*******************************************************************************
* �������� : PIDSetKd
* �������� : ����PID����Kd
* �������� : int
* ������� : ��
* �������� : ��
*******************************************************************************/
void PIDSetKd(double Kd)
{
    sptr_DuoJi->Derivative = Kd;
}
/*******************************************************************************
* �������� : PIDGetKd
* �������� : ��ȡPID����Ki
* �������� : ��
* ������� : ��
* �������� : int
*******************************************************************************/
double PIDGetKd(void)
{
    return (sptr_DuoJi->Derivative);
}

/*******************************************************************************
* �������� : PIDGetLastError
* �������� : ��ȡPID���� ���һ�����ֵ
* �������� : ��
* ������� : ��
* �������� : int
*******************************************************************************/
int PIDGetLastError(void)
{
    return (sptr_DuoJi->LastError);
}


/*******************************************************************************
* �������� : LocPIDCalc
* �������� : λ��ʽ PID ���Ƽ���
* �������� : int ��ǰλ��
* ������� : ��
* �������� : λ��ʽPID���
*******************************************************************************/
int LocPID_Votor(int NextPoint)
{
    int  iError,dError;

    iError = sptr_Votor->SetPoint - NextPoint;       //ƫ��
    sptr_Votor->SumError += iError;       //����
    dError = iError - sptr_Votor->LastError;     //΢��
    sptr_Votor->LastError = iError;

    return (int)(sptr_Votor->Proportion * iError            //������
           + sptr_Votor->Integral * sptr_Votor->SumError   //������
           + sptr_Votor->Derivative * dError);        //΢����
}

/*******************************************************************************
* �������� : IncPIDCalc
* �������� : ����ʽ PID ���Ƽ���
* �������� : int ��ǰλ��
* ������� : ��
* �������� : ����ʽPID���
*******************************************************************************/
int IncPIDCalc_Votor(int NextPoint)
{
    int iError, iIncpid;
    //��ǰ���
    iError = sptr_Votor->SetPoint - NextPoint;
    //��������
    iIncpid = (int)(sptr_Votor->Proportion * iError               //E[k]��
              - sptr_Votor->Integral   * sptr_Votor->LastError     //E[k��1]��
              + sptr_Votor->Derivative * sptr_Votor->PrevError);   //E[k��2]��
    //�洢�������´μ���
    sptr_Votor->PrevError = sptr_Votor->LastError;
    sptr_Votor->LastError = iError;
    //��������ֵ
    return(iIncpid);
}