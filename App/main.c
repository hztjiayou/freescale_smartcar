/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外初学论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#include "math.h"
#include "core_cm4.h"
uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
//uint8 img[CAMERA_W*CAMERA_H];                         //由于鹰眼摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理
uint8 img[CAMERA_H][CAMERA_W];

//int ShuJu=0; //调试串口通信的时候用

//函数声明
void sendimg(uint8 *imgaddr, uint32 imgsize);          //发送图像到上位机
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void uart3_handler();
void uart0_handler();
void PIT1_IRQHandler(void);
extern Fuzzy_line_finding Fuzzy_line;
extern volatile IMG_STATUS_e      ov7725_eagle_img_flag;
extern KEY_STATUS_e KEY_Data_B1;
extern KEY_STATUS_e KEY_Data_B2;
extern int Weight_max;//储存权重最大的行号
extern int Velocity_max;
extern int Velocity_min;
extern int Start_to_speed_up;   //开始加速
extern int Stop_accelerating;   //停止加速
extern int Speed_up_coefficient;//系数
extern char Fps_flag;           //控制输出FPS数据的开关
extern char Video_flag;         //控制录制的开关
extern char Start_flag;         //控制开始设置的开关
extern KEY_STATUS_e KEY_Data_B4;
extern uint8 Guanxian;          //控制二值化的阈值
extern PID_INIT PID_Init;//可操作的PID参数
extern PID   PID_DuoJi;//舵机（不可操作，只能刷新）
void Start()
{
  int Velocity=100;  //速度
  KEY_e KEY_data;
  FTM_PWM_Duty(FTM2,FTM_CH0,Median_of_rudder);//控制舵机初始位置为中值
  while(1)
  {
    KEY_data=KEY_Scan(0);
    OLED_ShowStr_Number(40,4,"V:",Velocity,1);
    if(KEY_data ==  KEY_L)
      Velocity=Velocity+10;
    
    if(KEY_data ==  KEY_D)
      Velocity=Velocity-10;
    
    if(KEY_data ==  KEY_R)
    {
      DELAY_MS(2000);
      Velocity_max=Velocity;
      Speed_up_coefficient=(Velocity_max-Velocity_min)/(Stop_accelerating-Start_to_speed_up);
      break;
    }
  }
}
/*!
 *  @brief      main函数
 *  @since      v5.0
 *  @note       山外 DMA 采集摄像头 实验
                注意，此例程 bus频率设为100MHz(50MHz bus频率会太慢而导致没法及时采集图像)
 */
void  main(void)
{
    gpio_init (PTA17, GPO,1);              //初始化 PTA8 管脚为输入
    gpio_set (PTA17, 1);
    camera_init(imgbuff);   //初始化摄像头
    uart_init(UART3,115200);//串口初始化
    
    NVIC_SetPriorityGrouping((uint32)0x3);            //设置优先级分组,4bit 抢占优先级,没有亚优先级
    
    set_vector_handler(UART3_RX_TX_VECTORn,uart3_handler);  //设置中断服务函数到中断向量表里
    uart_rx_irq_en (UART3);                                 //开串口接收中断
    
    /******************配置中断复位函数******************/
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //设置LPTMR的中断复位函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //设置LPTMR的中断复位函数为 PORTA_IRQHandler
    /***************************************************/
   
    FTM_QUAD_Init(FTM1);                                    //FTM1 正交解码初始化（所用的管脚可查 vcan_port_cfg.h ）
    pit_init_ms(PIT0, 10);                                  //初始化PIT0，定时时间为： 10ms 100fps
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断复位函数为 PIT0_IRQHandler
    
    pit_init_ms(PIT1, 20);                                  //初始化PIT1，定时时间为： 20ms 50fps
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //设置PIT1的中断复位函数为 PIT1_IRQHandler
    
    pit_init_ms(PIT2, 10);                                  //初始化PIT2，定时时间为： 10ms 100fps
    set_vector_handler(PIT2_VECTORn ,PIT2_IRQHandler);      //设置PIT2的中断复位函数为 PIT2_IRQHandler
    
    NVIC_SetPriority(PIT2_IRQn, 0);
    NVIC_SetPriority(DMA0_IRQn, 1);
    NVIC_SetPriority(PORTA_IRQn, 2);
    NVIC_SetPriority(PIT0_IRQn, 3);
    NVIC_SetPriority(PIT1_IRQn, 4);
    
    
    /*NVIC_EnableIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_EnableIRQ(PIT1_IRQn);*/
    
    
    FTM_PWM_init(FTM2,FTM_CH0,100,Median_of_rudder);   //初始化 舵机 PWM PA10
    FTM_PWM_init(FTM0,FTM_CH1,10*1000,0);  // PA4 
    FTM_PWM_init(FTM0,FTM_CH2,10*1000,0);  // PA5（该处参数不为0时为正转）
    OLED_Init();        //oled初始化
    OLED_Refresh_Gram();//oled刷新，要配合画点
    Pid_init();//PID初始化
    Track_width_init(&Fuzzy_line); //初始化赛道半宽度
    //Weight_init(Weight_max);//线性初始化所有行的权重（暂时屏蔽）
    key_init(KEY_MAX);
    Speed_up_coefficient=(Velocity_max-Velocity_min)/(Stop_accelerating-Start_to_speed_up);//初始化加速过程的系数
    FTM_PWM_Duty(FTM0, FTM_CH1,0);
    FTM_PWM_Duty(FTM0, FTM_CH2,0);
    FTM_PWM_Duty(FTM2,FTM_CH0,Median_of_rudder);//控制舵机初始位置为中值
    
    video_init();//录制功能初始化
    camera_get_img();                                 //摄像头获取图像
    DEBUG_PRINTF("\nFreescaleB Config Success!");//显示B车配置成功
    gpio_set (PTA17, 0);//显示B车配置成功
    
    enable_irq (PIT0_IRQn);                                 //使能PIT0中断
    enable_irq (PIT1_IRQn);                                 //使能PIT1中断
    enable_irq (PIT2_IRQn);                                 //使能PIT2中断
    while(1)
    {
        //camera_get_img();                                 //摄像头获取图像
        //disable_irq(PIT1_IRQn);
        //sendimg(imgbuff, CAMERA_SIZE);                  //发送到上位机
    }
}


void PIT1_IRQHandler(void)
{
  SCCB_WriteByte(OV7725_CNST  ,Guanxian);//改阈值
  KEY_Data_B4=key_get(KEY_B4);
  if(KEY_Data_B4 == KEY_UP && Start_flag==1)
  {
    Start();//开始的时候设置速度
    Start_flag=0;
  }
  if(KEY_Data_B4 == KEY_DOWN)
  {
      Start_flag=1;
  }
  //获取图像*******************************************************************
  if(ov7725_eagle_img_flag != IMG_FINISH)
    camera_get_img();
  img_extract((uint8 *)img,(uint8 *)imgbuff, CAMERA_H*CAMERA_W/8);
  ov7725_eagle_img_flag = IMG_START;           //开始采集图像
  PORTA_ISFR = ~0;                //写1清中断标志位(必须的，不然回导致一开中断就马上触发中断)
  enable_irq(PORTA_IRQn);                 //允许PTA的中断
  /////////******************************************************************
  
  img_extract((uint8 *)img,(uint8 *)imgbuff, CAMERA_H*CAMERA_W/8);//解压为灰度图像，方便发送到上位机显示
  KEY_Data_B1=key_get(KEY_B1);
  KEY_Data_B2=key_get(KEY_B2);
  Image_scanning();//扫描图像 求出误差值
  control();       //控制舵机和电机
  oled_show();     //显示图像和数据
  REC(imgbuff,0,0,0);
  ///////////////////////////////////////
  PIT_Flag_Clear(PIT1);  //清中断标志位
}

/*!
 *  @brief      发送图像到eSmartCameraCar上位机显示
 *  @param      imgaddr         图像地址
 *  @param      imgsize         图像占用空间大小
 *  @since      v5.0
 *  @note       不同的上位机，不同的命令，这里使用 eSmartCameraCar软件，
                如果使用其他上位机，则需要修改代码。
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //发送到上位机
 */
void sendimg(uint8 *imgaddr, uint32 imgsize)
{
   #define CMD_IMG     1
    uint8 cmdf[2] = {CMD_IMG, ~CMD_IMG};    //山外上位机 使用的命令
    uint8 cmdr[2] = {~CMD_IMG, CMD_IMG};    //山外上位机 使用的命令

    //uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送命令
    nrf_tx(cmdf, sizeof(cmdf));
    //uart_putbuff(VCAN_PORT, imgaddr, imgsize);      //再发送图像
    nrf_tx(imgaddr, imgsize);
    //uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //先发送命令
    nrf_tx(cmdr, sizeof(cmdr));
}

/*!
 *  @brief      二值化图像解压（空间 换 时间 解压）
 *  @param      dst             图像解压目的地址
 *  @param      src             图像解压源地址
 *  @param      srclen          二值化图像的占用空间大小
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //发送到上位机
 */
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {1, 0}; //0 和 1 分别对应的颜色
    //注：山外的摄像头 0 表示 白色，1表示 黑色
    uint8 tmpsrc;
    while(srclen --)
    {
        tmpsrc = *src++;
        *dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
    }
}

/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */

void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if 0             //鹰眼直接全速采集，不需要行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}

void uart3_handler()
{
    static char Shuju[20];
    static char Shuju_wei=0;
    if(uart_query    (UART3) == 1)   //接收数据寄存器满
    {
        //用户需要处理接收数据
        uart_getchar   (UART3, &Shuju[Shuju_wei]);                    //无限等待接受1个字节
        //uart_putchar   (UART3 , Shuju[Shuju_wei]);                  //发送字符串（用于检测串口的发送和接受）
        Shuju_wei++;
        if(Shuju[Shuju_wei-1] == ';')
        {
           if(atoi(Shuju+6) == 1 || atoi(Shuju+6) == 2 || atoi(Shuju+6) == 3 || atoi(Shuju+6) == 4 || atoi(Shuju+6) == 5 || atoi(Shuju+6) == 6)
           {
             if(atoi(Shuju+6) == 1)
             {
               Fps_flag++;
               if(Fps_flag == 3)
                 Fps_flag=1;
             }
             if(atoi(Shuju+6) == 2)
             {
               Video_flag++;
               if(Video_flag == 3)
                 Video_flag=1;
             }
             //////////////////////
             if(Video_flag == 1) REC_on();
             if(Video_flag == 2) REC_off();
             if(atoi(Shuju+6) == 3) {PID_Init.DuoJi_Proportion = PID_Init.DuoJi_Proportion + 0.0002; Pid_refresh(); printf("P = %f\n",PID_DuoJi.Proportion);}
             if(atoi(Shuju+6) == 4) {PID_Init.DuoJi_Proportion = PID_Init.DuoJi_Proportion - 0.0002; Pid_refresh(); printf("P = %f\n",PID_DuoJi.Proportion);}
             if(atoi(Shuju+6) == 5) {PID_Init.DuoJi_Derivative = PID_Init.DuoJi_Derivative + 0.0002; Pid_refresh(); printf("D = %f\n",PID_DuoJi.Derivative);}
             if(atoi(Shuju+6) == 6) {PID_Init.DuoJi_Derivative = PID_Init.DuoJi_Derivative - 0.0002; Pid_refresh(); printf("D = %f\n",PID_DuoJi.Derivative);}
           }
           else
           {
             Fps_flag=0;
             Velocity_max=atoi(Shuju+6);
             Speed_up_coefficient=(Velocity_max-Velocity_min)/(Stop_accelerating-Start_to_speed_up);
           }
           Shuju_wei=0;        //数据位清零
           strcpy(Shuju,"");   //数据清零
        }
    }
}

/*void uart0_handler(void)
{
    static char Shuju[20];
    static char Shuju_wei=0;
    if(uart_query    (UART0) == 1)   //接收数据寄存器满
    {
        //用户需要处理接收数据
        uart_getchar   (UART0, &Shuju[Shuju_wei]);                    //无限等待接受1个字节
        uart_putchar   (UART0 , Shuju[Shuju_wei]);                    //发送字符串
        Shuju_wei++;
        if(Shuju[Shuju_wei-1] == '.')
        {
           ShuJu=atoi(Shuju);
           Shuju_wei=0;        //数据位清零
           strcpy(Shuju,"");   //数据清零
        }
    }
}*/