#ifndef __ZHAOHAO_SDJC_H__
#define __ZHAOHAO_SDJC_H__

typedef struct EdgePoint {//点坐标
	int x;
	int y;
	//int inflexion;
}EdgePoint;

typedef struct ZhiXianData{
    double a;//直线斜率
    double b;//直线截距
    double variance;//方差
    int flag;//直线状态 0有问题1正常
} ZhiXianData;

extern int zhaohao_huandao_state;//环岛状态
extern int zhaohao_zhangai_state;//障碍状态
extern int zhaohao_banma_state;//斑马线状态

//赛道元素
void SDYS();
void ShiZiWan();
void SetHuanDaoTime(int time);
void SetZhangAiTime(int time);
void SetBanMaChange(int change);
void ClearState();
void StartAndFinish();
#endif