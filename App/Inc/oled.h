
#ifndef _OLED_H
#define _OLED_H
#include "common.h"
#include "include.h"
#include "MK60_gpio.h"


#define OLED_SCL  PTC11
#define OLED_SDA  PTC12
#define OLED_RST  PTC13
#define OLED_DC   PTC14



#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xCF

#define X_WIDTH 128
#define Y_WIDTH 64

#define LED_IMAGE_WHITE       0x01
#define LED_IMAGE_BLACK       0

#define LED_MAX_ROW_NUM      64
#define LED_MAX_COLUMN_NUM  128


void OLED_Init(void);
void OLED_CLS(void);
void OLED_Fill(u8 bmp_data);

void OLED_PutPixel(u8 x,u8 y);							//绘制一个点（x,y）
void OLED_Rectangle(u8 x1,u8 y1,u8 x2,u8 y2,u8 gif);            //绘制一个实心矩形
void OLED_P6x8Str(u8 x,u8 y,u8 ch[]);				    //字符串
void OLED_P8x16Str(u8 x,u8 y,u8 ch[]);				    //字符串
void OLED_P14x16Str(u8 x,u8 y,u8 ch[]);			        //汉字串
void OLED_Print(u8 x, u8 y, u8 ch[]);		//显示汉字字符混合 OLED_Print(6,6,"直立Saaaa");


void Draw_BMP(u8 x0,u8 y0,u8 x1,u8 y1,u8 bmp[]);	    //显示图片

void OLED_Drawtu(void);       //显示图片-小车
void OLED_Picture(void);      //显示图片-大马

void OLED_PrintImage(u8 *pucTable, u16 usRowNum, u16 usColumnNum);     //图像采集显示

void OLED_Picture_sign(void);   //显示标记（测试中）



void OLED_nurbem(u8 x,u8 y,u8 a,u8 b);                      //数字
void OLED_nurbem_big(u8 x,u8 y,u8 a,u8 b);
void OLED_nurbem_int(u8 x,u8 y,u16 a);
void OLED_float(u8 x,u8 y,float a);     //浮点数

void OLED_Refresh_Gram(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);

void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[],unsigned char TextSize);
void OLED_ShowStr_Number(unsigned char x, unsigned char y, unsigned char ch[],int shuju, unsigned char TextSize);
void OLED_ShowStr_Number_float(unsigned char x, unsigned char y, unsigned char ch[],float shuju, unsigned char TextSize);



//OLED_Init();             //----OLED显示初始化
//OLED_Print(0,0,"OLED-CHECK-OK!");     //----测试OLED能否正常工作

#endif
