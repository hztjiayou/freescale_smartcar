#ifndef __VIDEO_H__
#define __VIDEO_H__

typedef struct Video
{
	uint8 imgbuff[3][600];
	int32 data1[3];
	int32 data2[3];
	int32 data3[3];

}Video;

void REC(uint8 *buff , int32 data1, int32 data2, int32 data3);//¼�ƺ���
void REC_off();//�ر�¼��
void REC_on();//��¼��
void REC_cls();//�����Ƶ
void video_init();//��ʼ��
void video_play();//����
//�����Ժ�������Ҫ�Լ��޸ģ�
void video_pause();//��ͣ
int video_back();//����
#endif