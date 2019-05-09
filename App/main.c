/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ���ѧ��̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��K60 ƽ̨������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#include "math.h"
#include "core_cm4.h"
uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������
//uint8 img[CAMERA_W*CAMERA_H];                         //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��
uint8 img[CAMERA_H][CAMERA_W];

//int ShuJu=0; //���Դ���ͨ�ŵ�ʱ����

//��������
void sendimg(uint8 *imgaddr, uint32 imgsize);          //����ͼ����λ��
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
extern int Weight_max;//����Ȩ�������к�
extern int Velocity_max;
extern int Velocity_min;
extern int Start_to_speed_up;   //��ʼ����
extern int Stop_accelerating;   //ֹͣ����
extern int Speed_up_coefficient;//ϵ��
extern char Fps_flag;           //�������FPS���ݵĿ���
extern char Video_flag;         //����¼�ƵĿ���
extern char Start_flag;         //���ƿ�ʼ���õĿ���
extern KEY_STATUS_e KEY_Data_B4;
extern uint8 Guanxian;          //���ƶ�ֵ������ֵ
extern PID_INIT PID_Init;//�ɲ�����PID����
extern PID   PID_DuoJi;//��������ɲ�����ֻ��ˢ�£�
void Start()
{
  int Velocity=100;  //�ٶ�
  KEY_e KEY_data;
  FTM_PWM_Duty(FTM2,FTM_CH0,Median_of_rudder);//���ƶ����ʼλ��Ϊ��ֵ
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
 *  @brief      main����
 *  @since      v5.0
 *  @note       ɽ�� DMA �ɼ�����ͷ ʵ��
                ע�⣬������ busƵ����Ϊ100MHz(50MHz busƵ�ʻ�̫��������û����ʱ�ɼ�ͼ��)
 */
void  main(void)
{
    gpio_init (PTA17, GPO,1);              //��ʼ�� PTA8 �ܽ�Ϊ����
    gpio_set (PTA17, 1);
    camera_init(imgbuff);   //��ʼ������ͷ
    uart_init(UART3,115200);//���ڳ�ʼ��
    
    NVIC_SetPriorityGrouping((uint32)0x3);            //�������ȼ�����,4bit ��ռ���ȼ�,û�������ȼ�
    
    set_vector_handler(UART3_RX_TX_VECTORn,uart3_handler);  //�����жϷ��������ж���������
    uart_rx_irq_en (UART3);                                 //�����ڽ����ж�
    
    /******************�����жϸ�λ����******************/
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //����LPTMR���жϸ�λ����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //����LPTMR���жϸ�λ����Ϊ PORTA_IRQHandler
    /***************************************************/
   
    FTM_QUAD_Init(FTM1);                                    //FTM1 ���������ʼ�������õĹܽſɲ� vcan_port_cfg.h ��
    pit_init_ms(PIT0, 10);                                  //��ʼ��PIT0����ʱʱ��Ϊ�� 10ms 100fps
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϸ�λ����Ϊ PIT0_IRQHandler
    
    pit_init_ms(PIT1, 20);                                  //��ʼ��PIT1����ʱʱ��Ϊ�� 20ms 50fps
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //����PIT1���жϸ�λ����Ϊ PIT1_IRQHandler
    
    pit_init_ms(PIT2, 10);                                  //��ʼ��PIT2����ʱʱ��Ϊ�� 10ms 100fps
    set_vector_handler(PIT2_VECTORn ,PIT2_IRQHandler);      //����PIT2���жϸ�λ����Ϊ PIT2_IRQHandler
    
    NVIC_SetPriority(PIT2_IRQn, 0);
    NVIC_SetPriority(DMA0_IRQn, 1);
    NVIC_SetPriority(PORTA_IRQn, 2);
    NVIC_SetPriority(PIT0_IRQn, 3);
    NVIC_SetPriority(PIT1_IRQn, 4);
    
    
    /*NVIC_EnableIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_EnableIRQ(PIT1_IRQn);*/
    
    
    FTM_PWM_init(FTM2,FTM_CH0,100,Median_of_rudder);   //��ʼ�� ��� PWM PA10
    FTM_PWM_init(FTM0,FTM_CH1,10*1000,0);  // PA4 
    FTM_PWM_init(FTM0,FTM_CH2,10*1000,0);  // PA5���ô�������Ϊ0ʱΪ��ת��
    OLED_Init();        //oled��ʼ��
    OLED_Refresh_Gram();//oledˢ�£�Ҫ��ϻ���
    Pid_init();//PID��ʼ��
    Track_width_init(&Fuzzy_line); //��ʼ����������
    //Weight_init(Weight_max);//���Գ�ʼ�������е�Ȩ�أ���ʱ���Σ�
    key_init(KEY_MAX);
    Speed_up_coefficient=(Velocity_max-Velocity_min)/(Stop_accelerating-Start_to_speed_up);//��ʼ�����ٹ��̵�ϵ��
    FTM_PWM_Duty(FTM0, FTM_CH1,0);
    FTM_PWM_Duty(FTM0, FTM_CH2,0);
    FTM_PWM_Duty(FTM2,FTM_CH0,Median_of_rudder);//���ƶ����ʼλ��Ϊ��ֵ
    
    video_init();//¼�ƹ��ܳ�ʼ��
    camera_get_img();                                 //����ͷ��ȡͼ��
    DEBUG_PRINTF("\nFreescaleB Config Success!");//��ʾB�����óɹ�
    gpio_set (PTA17, 0);//��ʾB�����óɹ�
    
    enable_irq (PIT0_IRQn);                                 //ʹ��PIT0�ж�
    enable_irq (PIT1_IRQn);                                 //ʹ��PIT1�ж�
    enable_irq (PIT2_IRQn);                                 //ʹ��PIT2�ж�
    while(1)
    {
        //camera_get_img();                                 //����ͷ��ȡͼ��
        //disable_irq(PIT1_IRQn);
        //sendimg(imgbuff, CAMERA_SIZE);                  //���͵���λ��
    }
}


void PIT1_IRQHandler(void)
{
  SCCB_WriteByte(OV7725_CNST  ,Guanxian);//����ֵ
  KEY_Data_B4=key_get(KEY_B4);
  if(KEY_Data_B4 == KEY_UP && Start_flag==1)
  {
    Start();//��ʼ��ʱ�������ٶ�
    Start_flag=0;
  }
  if(KEY_Data_B4 == KEY_DOWN)
  {
      Start_flag=1;
  }
  //��ȡͼ��*******************************************************************
  if(ov7725_eagle_img_flag != IMG_FINISH)
    camera_get_img();
  img_extract((uint8 *)img,(uint8 *)imgbuff, CAMERA_H*CAMERA_W/8);
  ov7725_eagle_img_flag = IMG_START;           //��ʼ�ɼ�ͼ��
  PORTA_ISFR = ~0;                //д1���жϱ�־λ(����ģ���Ȼ�ص���һ���жϾ����ϴ����ж�)
  enable_irq(PORTA_IRQn);                 //����PTA���ж�
  /////////******************************************************************
  
  img_extract((uint8 *)img,(uint8 *)imgbuff, CAMERA_H*CAMERA_W/8);//��ѹΪ�Ҷ�ͼ�񣬷��㷢�͵���λ����ʾ
  KEY_Data_B1=key_get(KEY_B1);
  KEY_Data_B2=key_get(KEY_B2);
  Image_scanning();//ɨ��ͼ�� ������ֵ
  control();       //���ƶ���͵��
  oled_show();     //��ʾͼ�������
  REC(imgbuff,0,0,0);
  ///////////////////////////////////////
  PIT_Flag_Clear(PIT1);  //���жϱ�־λ
}

/*!
 *  @brief      ����ͼ��eSmartCameraCar��λ����ʾ
 *  @param      imgaddr         ͼ���ַ
 *  @param      imgsize         ͼ��ռ�ÿռ��С
 *  @since      v5.0
 *  @note       ��ͬ����λ������ͬ���������ʹ�� eSmartCameraCar�����
                ���ʹ��������λ��������Ҫ�޸Ĵ��롣
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //���͵���λ��
 */
void sendimg(uint8 *imgaddr, uint32 imgsize)
{
   #define CMD_IMG     1
    uint8 cmdf[2] = {CMD_IMG, ~CMD_IMG};    //ɽ����λ�� ʹ�õ�����
    uint8 cmdr[2] = {~CMD_IMG, CMD_IMG};    //ɽ����λ�� ʹ�õ�����

    //uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //�ȷ�������
    nrf_tx(cmdf, sizeof(cmdf));
    //uart_putbuff(VCAN_PORT, imgaddr, imgsize);      //�ٷ���ͼ��
    nrf_tx(imgaddr, imgsize);
    //uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //�ȷ�������
    nrf_tx(cmdr, sizeof(cmdr));
}

/*!
 *  @brief      ��ֵ��ͼ���ѹ���ռ� �� ʱ�� ��ѹ��
 *  @param      dst             ͼ���ѹĿ�ĵ�ַ
 *  @param      src             ͼ���ѹԴ��ַ
 *  @param      srclen          ��ֵ��ͼ���ռ�ÿռ��С
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //���͵���λ��
 */
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {1, 0}; //0 �� 1 �ֱ��Ӧ����ɫ
    //ע��ɽ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
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
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */

void PORTA_IRQHandler()
{
    uint8  n = 0;    //���ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if 0             //ӥ��ֱ��ȫ�ٲɼ�������Ҫ���ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0�жϷ�����
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
    if(uart_query    (UART3) == 1)   //�������ݼĴ�����
    {
        //�û���Ҫ�����������
        uart_getchar   (UART3, &Shuju[Shuju_wei]);                    //���޵ȴ�����1���ֽ�
        //uart_putchar   (UART3 , Shuju[Shuju_wei]);                  //�����ַ��������ڼ�⴮�ڵķ��ͺͽ��ܣ�
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
           Shuju_wei=0;        //����λ����
           strcpy(Shuju,"");   //��������
        }
    }
}

/*void uart0_handler(void)
{
    static char Shuju[20];
    static char Shuju_wei=0;
    if(uart_query    (UART0) == 1)   //�������ݼĴ�����
    {
        //�û���Ҫ�����������
        uart_getchar   (UART0, &Shuju[Shuju_wei]);                    //���޵ȴ�����1���ֽ�
        uart_putchar   (UART0 , Shuju[Shuju_wei]);                    //�����ַ���
        Shuju_wei++;
        if(Shuju[Shuju_wei-1] == '.')
        {
           ShuJu=atoi(Shuju);
           Shuju_wei=0;        //����λ����
           strcpy(Shuju,"");   //��������
        }
    }
}*/