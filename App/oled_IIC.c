#include "common.h"
#include "include.h"
//#include  "codetab.h"   //字符库
uint8 OLED_GRAM_IIC[128][8];	 
extern const u8 F6x8[][6];
extern const u8 F8X16[];
const unsigned char  NUM16x16[] = 	  	 
{  
0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,/*"0",0*/
/* (8 X 16 , ?? )*/

0x00,0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,/*"1",1*/
/* (8 X 16 , ?? )*/

0x00,0x70,0x08,0x08,0x08,0x08,0xF0,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,/*"2",2*/
/* (8 X 16 , ?? )*/

0x00,0x30,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x18,0x20,0x21,0x21,0x22,0x1C,0x00,/*"3",3*/
/* (8 X 16 , ?? )*/

0x00,0x00,0x80,0x40,0x30,0xF8,0x00,0x00,0x00,0x06,0x05,0x24,0x24,0x3F,0x24,0x24,/*"4",4*/
/* (8 X 16 , ?? )*/

0x00,0xF8,0x88,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x20,0x20,0x20,0x11,0x0E,0x00,/*"5",5*/
/* (8 X 16 , ?? )*/

0x00,0xE0,0x10,0x88,0x88,0x90,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x20,0x1F,0x00,/*"6",6*/
/* (8 X 16 , ?? )*/

0x00,0x18,0x08,0x08,0x88,0x68,0x18,0x00,0x00,0x00,0x00,0x3E,0x01,0x00,0x00,0x00,/*"7",7*/
/* (8 X 16 , ?? )*/

0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,/*"8",8*/
/* (8 X 16 , ?? )*/

0x00,0xF0,0x08,0x08,0x08,0x10,0xE0,0x00,0x00,0x01,0x12,0x22,0x22,0x11,0x0F,0x00,/*"9",9*/
/* (8 X 16 , ?? )*/
	0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,/*"P",10*/
	/* (8 X 16 , ?? )*/

	0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,/*"I",11*/
	/* (8 X 16 , ?? )*/

	0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,/*"D",12*/
/* (8 X 16 , ?? )*/
	0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,/*"|",13*/
/* (8 X 16 , ?? )*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,/*".",14*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,


};
void IIC_Init(void)
{					     
	gpio_init (PTD8, GPO,0);    //初始化 PTA8 管脚为输入 
        gpio_init (PTD9, GPO,0);    //初始化 PTA8 管脚为输入 
	IIC_SCL_1;
	IIC_SDA_1;
}
void IIC_Start(void)
{
	SDA_OUT;     
	IIC_SDA_1;	  	  
	IIC_SCL_1;
	DELAY_US(4);
 	IIC_SDA_0;
	DELAY_US(4);
	IIC_SCL_0;
}	  
void IIC_Stop(void)
{
	SDA_OUT;//sda??ê?3?
	IIC_SCL_0;
	IIC_SDA_0;//STOP:when CLK is high DATA change form low to high
 	DELAY_US(4);
	IIC_SCL_1; 
	IIC_SDA_1;//·￠?íI2C×ü???áê?D?o?
	DELAY_US(4);							   	
}
uint8 IIC_Wait_Ack(void)
{
	uint8 ucErrTime=0;
        uint8 j;
        SDA_OUT;
	IIC_SDA_1;
        SDA_IN_1;
        DELAY_US(1);	   
	IIC_SCL_1;
        DELAY_US(1);
        j=READ_SDA;
	while(j)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_0; 
        return 0;  
} 
void IIC_Ack(void)
{
	IIC_SCL_0;
	SDA_OUT;
	IIC_SDA_0;
	DELAY_US(2);
	IIC_SCL_1;
        DELAY_US(2);
	IIC_SCL_0;
}
	    
void IIC_NAck(void)
{
	IIC_SCL_0;
	SDA_OUT;
	IIC_SDA_1;
	DELAY_US(2);
	IIC_SCL_1;
	DELAY_US(2);
	IIC_SCL_0;
}					 				     
	  
void IIC_Send_Byte(uint8 txd)
{                        
    uint8 t,j;   
    SDA_OUT; 	    
    IIC_SCL_0;
    for(t=0;t<8;t++)
    {    
        j=(txd&0x80)>>7;
        if(j){IIC_SDA_1;}
        else {IIC_SDA_0;}
        txd<<=1; 	  
		DELAY_US(2); 
		IIC_SCL_1;
		DELAY_US(2); 
		IIC_SCL_0;	
		DELAY_US(2);
    }	 
} 	       
uint8 IIC_Read_Byte(unsigned char ack)
{
        uint8 j;
	unsigned char i,receive=0;
	SDA_IN_1;//SDAéè???aê?è?
    for(i=0;i<8;i++ )
	{
        IIC_SCL_0; 
        DELAY_US(2);
		IIC_SCL_1;
        receive<<=1;
        j=gpio_get (PTD9);;
        if(j)receive++;   
		DELAY_US(1); 
    }					 
    if (!ack)
        IIC_NAck();//·￠?ínACK
    else
        IIC_Ack(); //·￠?íACK   
    return receive;
}
void OLED_Init_IIC(void)
{
	IIC_Init();
	DELAY_MS(500);//3?ê??ˉ???°μ??óê±oü??òa￡?
	OLED_WrCmd_IIC(0xae);//--turn off oled panel
	OLED_WrCmd_IIC(0x00);//---set low column address
	OLED_WrCmd_IIC(0x10);//---set high column address
	OLED_WrCmd_IIC(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WrCmd_IIC(0x81);//--set contrast control register
	OLED_WrCmd_IIC(Brightness); // Set SEG Output Current Brightness
	OLED_WrCmd_IIC(0xa1);//--Set SEG/Column Mapping     0xa0×óóò·′?? 0xa1?y3￡
	OLED_WrCmd_IIC(0xc8);//Set COM/Row Scan Direction   0xc0é???·′?? 0xc8?y3￡
	OLED_WrCmd_IIC(0xa6);//--set normal display
	OLED_WrCmd_IIC(0xa8);//--set multiplex ratio(1 to 64)
	OLED_WrCmd_IIC(0x3f);//--1/64 duty
	OLED_WrCmd_IIC(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WrCmd_IIC(0x00);//-not offset
	OLED_WrCmd_IIC(0xd5);//--set display clock divide ratio/oscillator frequency
	OLED_WrCmd_IIC(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WrCmd_IIC(0xd9);//--set pre-charge period
	OLED_WrCmd_IIC(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WrCmd_IIC(0xda);//--set com pins hardware configuration
	OLED_WrCmd_IIC(0x12);
	OLED_WrCmd_IIC(0xdb);//--set vcomh
	OLED_WrCmd_IIC(0x40);//Set VCOM Deselect Level
	OLED_WrCmd_IIC(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WrCmd_IIC(0x02);//
	OLED_WrCmd_IIC(0x8d);//--set Charge Pump enable/disable
	OLED_WrCmd_IIC(0x14);//--set(0x10) disable
	OLED_WrCmd_IIC(0xa4);// Disable Entire Display On (0xa4/0xa5)
	OLED_WrCmd_IIC(0xa6);// Disable Inverse Display On (0xa6/a7) 
	OLED_WrCmd_IIC(0xaf);//--turn on oled panel
	OLED_Fill_IIC(0x00); //3?ê????á//改了 可能有问题
	OLED_Set_Pos_IIC(0,0);
}



void OLED_Wrdat(u8 dat)				//D′êy?Y
{
	IIC_Start();
	IIC_Send_Byte(0x78);		//?÷?tμ??·
	IIC_Wait_Ack();
	IIC_Send_Byte(0x40);		//D′êy?Y
	IIC_Wait_Ack();
	IIC_Send_Byte(dat);
	IIC_Wait_Ack();
	IIC_Stop();
}

/******************/
void OLED_WrCmd_IIC(u8 Command)		//D′?üá?
{
	IIC_Start();
	IIC_Send_Byte(0x78);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x00);
	IIC_Wait_Ack();
	IIC_Send_Byte(Command);
	IIC_Wait_Ack();
	IIC_Stop();
}


void OLED_Set_Pos_IIC(u8 x,u8 y)		
{
	OLED_WrCmd_IIC(0xb0+y);
	OLED_WrCmd_IIC(((x&0xf0)>>4)|0x10);
	OLED_WrCmd_IIC((x&0x0f)|0x01);
}


void OLED_Fill_IIC(u8 bmp_dat)	
{
	u8 x,y;
	for(y = 0; y < 8; y++)
	{
		OLED_WrCmd_IIC(0xb0+y);
		OLED_WrCmd_IIC(0x01);
		OLED_WrCmd_IIC(0x10);
		for(x = 0; x < X_WIDTH; x++)
		{
			OLED_Wrdat(bmp_dat);
		}
	}
}

void OLED_RESET(void)					//?′??
{
	u8 y,x;
	for(y = 0; y < 8; y++)
	{
		OLED_WrCmd_IIC(0xb0 + y);
		OLED_WrCmd_IIC(0x01);
		OLED_WrCmd_IIC(0x10);
		for(x = 0; x < X_WIDTH; x++)
		OLED_Wrdat(0);
	}
}
void OLED_P16x16num(unsigned char x, unsigned char y,unsigned char N)
{
	unsigned char wm=0;
	unsigned int adder=16*N;
	OLED_Set_Pos_IIC(x , y);
	for(wm = 0;wm < 8;wm++)
	{
		OLED_Wrdat(NUM16x16[adder]);
		adder += 1;
	}
	OLED_Set_Pos_IIC(x,y + 1);
	for(wm = 0;wm < 8;wm++)
	{
		OLED_Wrdat(NUM16x16[adder]);
		adder += 1;
	} 	  	
}
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
	unsigned int j=0;
	unsigned char x,y;

  if(y1%8==0)
		y = y1/8;
  else
		y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos_IIC(x0,y);
    for(x=x0;x<x1;x++)
		{
			OLED_Wrdat(BMP[j++]);
		}
	}
}
void OLED_DrawPoint_IIC(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM_IIC[x][pos]|=temp;
	else OLED_GRAM_IIC[x][pos]&=~temp;	    
}
void OLED_Refresh_Gram_IIC(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WrCmd_IIC (0xb0+i);    //éè??ò3μ??·￡¨0~7￡?
		OLED_WrCmd_IIC (0x00);      //éè????ê??????aáDμíμ??·
		OLED_WrCmd_IIC (0x10);      //éè????ê??????aáD??μ??·   
		for(n=0;n<128;n++)OLED_Wrdat(OLED_GRAM_IIC[n][i]); 
	}   
}



//(x:0-127，y:0-7) ch[] 显示的字符串 shuju 显示的数字 int型 可自己设置   TextSize 字符大小 (1:6*8，2:8*16)
void OLED_ShowStr_Number_IIC(unsigned char x, unsigned char y, unsigned char ch[],int shuju, unsigned char TextSize) 
{
	unsigned char c = 0,i = 0,j = 0;u8 ShuZu[20];
        sprintf((char*)ShuZu,"%s%d    ",ch,shuju);
	switch(TextSize)
	{
		case 1:
		{
			while(ShuZu[j] != '\0')
			{
				c = ShuZu[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_Set_Pos_IIC(x,y);
				for(i=0;i<6;i++)
				OLED_Wrdat(F6x8[c][i]);
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ShuZu[j] != '\0')
			{
				c = ShuZu[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_Set_Pos_IIC(x,y);
				for(i=0;i<8;i++)
					OLED_Wrdat(F8X16[c*16+i]);
				OLED_Set_Pos_IIC(x,y+1);
				for(i=0;i<8;i++)
					OLED_Wrdat(F8X16[c*16+i+8]);
				x += 8;
				j++;
			}
		}break;
	}
}

//(x:0-127，y:0-7) ch[] 显示的字符串 shuju 显示的数字 int型 可自己设置   TextSize 字符大小 (1:6*8，2:8*16)
void OLED_ShowStr_IIC(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize) 
{
	unsigned char c = 0,i = 0,j = 0;u8 ShuZu[20];
        sprintf((char*)ShuZu,"%s",ch);
	switch(TextSize)
	{
		case 1:
		{
			while(ShuZu[j] != '\0')
			{
				c = ShuZu[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_Set_Pos_IIC(x,y);
				for(i=0;i<6;i++)
				OLED_Wrdat(F6x8[c][i]);
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ShuZu[j] != '\0')
			{
				c = ShuZu[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_Set_Pos_IIC(x,y);
				for(i=0;i<8;i++)
					OLED_Wrdat(F8X16[c*16+i]);
				OLED_Set_Pos_IIC(x,y+1);
				for(i=0;i<8;i++)
					OLED_Wrdat(F8X16[c*16+i+8]);
				x += 8;
				j++;
			}
		}break;
	}
}
