#ifndef __VIDEO_H__
#define __VIDEO_H__

typedef struct Video
{
	uint8 imgbuff[3][600];
	int32 data1[3];
	int32 data2[3];
	int32 data3[3];

}Video;

void REC(uint8 *buff , int32 data1, int32 data2, int32 data3);//录制函数
void REC_off();//关闭录制
void REC_on();//打开录制
void REC_cls();//清除视频
void video_init();//初始化
void video_play();//播放
//开放性函数，需要自己修改；
void video_pause();//暂停
int video_back();//后退
#endif