#ifndef __OLED_H__
#define __OLED_H__

#define IIC_SCL_1  gpio_set (PTD8, 1);     
#define IIC_SCL_0  gpio_set (PTD8, 0);    

#define IIC_SDA_1  gpio_set (PTD9, 1);    
#define IIC_SDA_0  gpio_set (PTD9, 0);    

#define SDA_OUT    gpio_init (PTD9, GPO,0);    
#define SDA_IN_1     gpio_init (PTD9, GPI,0);    
#define READ_SDA     gpio_get (PTD9);
#define u8 uint8
#define	Brightness	0xCF 
#define X_WIDTH 	128
#define Y_WIDTH 	64

void IIC_Init(void);                //3?��??��IIC��?IO?��				 
void IIC_Start(void);				//����?��IIC?a��?D?o?
void IIC_Stop(void);	  			//����?��IIC����?1D?o?
void IIC_Send_Byte(u8 txd);			//IIC����?����???��??��
u8 IIC_Read_Byte(unsigned char ack);//IIC?����?��???��??��
u8 IIC_Wait_Ack(void); 				//IIC�̨���yACKD?o?
void IIC_Ack(void);					//IIC����?��ACKD?o?
void IIC_NAck(void);				//IIC2?����?��ACKD?o?

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);
void OLED_Init_IIC(void);
void OLED_Wrdat(u8 dat);				//D�䨺y?Y
void OLED_WrCmd_IIC(u8 Command);		//D��?����?
void OLED_Set_Pos_IIC(u8 x,u8 y);		//����??��?����
void OLED_Fill_IIC(u8 bmp_dat);			//��??������
void OLED_RESET(void);					//?��??
void OLED_P16x16num(unsigned char x, unsigned char y,unsigned char N);
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);

void OLED_ShowStr_IIC(unsigned char x, unsigned char y, unsigned char ch[],unsigned char TextSize);
void OLED_ShowStr_Number_IIC(unsigned char x, unsigned char y, unsigned char ch[],int shuju, unsigned char TextSize);

void OLED_Refresh_Gram_IIC(void);
void OLED_DrawPoint_IIC(u8 x,u8 y,u8 t);
#endif