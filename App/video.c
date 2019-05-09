#include "common.h"
#include "include.h"
extern uint8 img[CAMERA_H][CAMERA_W];
extern char Fps;
extern void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
int REC_switch = 0;
int pian = 255;
int wei = 0;
char Video_flag=0;
Video video_buff;
#define MINPAIN 155

void video_init()
{
	flash_init();
}

void REC_on()
{
	pian = 255;
	wei = 0;
	REC_switch = 1;
}
void REC_off()
{
	REC_switch = 0;
}

void REC_cls()
{
	for(pian = 255; pian > MINPAIN; pian--)
		flash_erase_sector(pian);
}

void REC(uint8 *buff , int32 data1, int32 data2, int32 data3)
{
	int i;
	if(pian == MINPAIN)
		return;
	if(REC_switch)
	{
		for(i = 0; i < 600; i++)
		{
			video_buff.imgbuff[wei][i] = buff[i];
		}
		video_buff.data1[wei] = data1;
		video_buff.data2[wei] = data2;
		video_buff.data3[wei] = data3;
		wei++;
	}
	if(wei == 3)
	{
		wei = 0;
		if(1 != flash_write_buf(pian,0,sizeof(Video),(uint8*)(&video_buff)))
			printf("flash_err\n");
		pian--;
	}
}

void video_play()
{
	void video_pause();//ÔÝÍ£
	int video_back();//ºóÍË
	for(pian = 255; pian > MINPAIN; pian--)
	{
		video_buff = flash_read(pian, 0, Video);
		for(wei = 0; wei < 3; wei++)
		{
                        Fps++;
			img_extract((uint8 *)img,(uint8 *)&video_buff.imgbuff[wei], CAMERA_H*CAMERA_W/8);
			/*×¢ÒâÐÞ¸Ä*/
			OLED_PrintImage((uint8*)img,CAMERA_H,CAMERA_W);
			//OLED_ShowData(0,'e', video_buff.data1[wei]);
			//OLED_ShowData(1,'l', video_buff.data2[wei]);
			//OLED_ShowData(2,'s', video_buff.data3[wei]);
			//OLED_ShowData(3,'T', (255-pian)*3+wei+1);
			video_pause();//ÔÝÍ£
			video_back();//ºóÍË
			DELAY_MS(40);
		}
	}
}

int video_back()//ºóÍË
{
	if(gpio_get(PTD6) == 0)
	{
		DELAY_MS(10);
		if(gpio_get(PTD6) == 0)
		{
			pian += 3;
			wei = 0;
			if(pian > 255) pian = 255;
		}
		return 1;
	}
	else 
		return 0; 
}

void video_pause()//ÔÝÍ£
{
	static int pause = 0;
	if(gpio_get(PTD10) == 0)
	{
		DELAY_MS(10);
		if(gpio_get(PTD10) == 0)
		{
			if(pause == 0) pause = 1;
			else pause = 0;
		}
	}
	while(pause)
	{
		//sendimg((uint8*)img, 60*80);
		if(gpio_get(PTD8) == 0)
		{
			DELAY_MS(10);
			if(gpio_get(PTD8) == 0)
			{
				break;
			}
		}
		if(video_back()) break;
		if(gpio_get(PTD10) == 0)
		{
			DELAY_MS(10);
			if(gpio_get(PTD10) == 0)
			{
				pause = 0;
				while(gpio_get(PTD10) == 0);
				break;
			}
	}
	}
}
