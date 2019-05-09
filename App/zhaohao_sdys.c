#include "include.h"
#include "zhaohao_sdys.h"

extern uint8 img[CAMERA_H][CAMERA_W];
uint8 (*obj_img)[80]=img;//图片参数

int zhaohao_huandao_state = 0;//环岛状态
int zhaohao_zhangai_state = 0;//障碍状态
int zhaohao_banma_state = 0;//斑马线状态
int banmaxian_change = 18;//斑马线设置检测值
int huandaotime = 0;
int huandaosettime = 20;//环岛超时
int zhangaitime = 0;
int zhangaisettime = 100;//障碍超时
int zhangaiweight = 50;//避障权重
int startandfinish = 0;

/* 画线函数，在图像中画一条直线
* 入口参数 起始点x坐标、y坐标，结束点x坐标、y坐标，图像指针
* 返回值 无
*/
void HuaXian(EdgePoint p1, EdgePoint p2)
{
	unsigned int t;
	int xerr = 0, yerr = 0, delta_x, delta_y, distance;
	int incx, incy, uRow, uCol;
	if(p1.x>79)p1.x=79;
	if(p1.x<0)p1.x=0;
	if(p2.x>79)p2.x=79;
	if(p2.x<0)p2.x=0;
	if(p1.y>59)p1.y=59;
	if(p1.y<0)p1.y=0;
	if(p2.y>59)p2.y=59;
	if(p2.y<0)p2.y=0;
	delta_x = p2.x - p1.x; //计算坐标增量 
	delta_y = p2.y - p1.y;
	uRow = p1.x;
	uCol = p1.y;
	if (delta_x>0)incx = 1; //设置单步方向 
	else if (delta_x == 0)incx = 0;//垂直线 
	else { incx = -1; delta_x = -delta_x; }
	if (delta_y>0)incy = 1;
	else if (delta_y == 0)incy = 0;//水平线 
	else { incy = -1; delta_y = -delta_y; }
	if (delta_x>delta_y)distance = delta_x; //选取基本增量坐标轴 
	else distance = delta_y;
	for (t = 0; t <= distance + 1; t++)//画线输出 
	{
		obj_img[uCol][uRow] = 0x00;//画点 
		xerr += delta_x;
		yerr += delta_y;
		if (xerr>distance)
		{
			xerr -= distance;
			uRow += incx;
		}
		if (yerr>distance)
		{
			yerr -= distance;
			uCol += incy;
		}
	}
}

/*画曲线用的，效果自己测试
 *下面的点写在后面是外凸，否则内凸
 */
void QuXianHuaXian(EdgePoint p1, EdgePoint p2)
{
	EdgePoint p3, p4;
	p3.x = p1.x + (p2.x - p1.x) / 2;
	p3.y = p1.y + (p2.y - p1.y) / 4;
	p4.x = p2.x - (p2.x - p1.x) / 4;
	p4.y = p2.y - (p2.y - p1.y) / 2;
	HuaXian(p1, p3);
	HuaXian(p3, p4);
	HuaXian(p4, p2);
}

/*把部分图像抹白，清除斑马线用的*/
void img_clear_line(int yd,int yu)
{
	int temp;
	int i,j;
	int y1 = yd,y2 = yu;
	if(y1 > y2){temp = y1;y1=y2,y2=temp;}
	if(y1 < 0)y1 = 0;
	if(y2 > 59)y2 = 59;
	for(i = y1; i <= y2;i++)
	{
		for(j = 5; j < 75; j++)
			obj_img[i][j] = 0x01;
	}
}

/* 判断直线函数，通过最小二乘法判断是否是一条直线
 * 入口参数 直线指针，直线长度
 * 返回值 直线数据
 */
ZhiXianData ifzhixian(int* line, int linelong)
{
	int i;//for循环用
	double sumX = 0;//求x和
	double sumY = 0;//求y和
	double sumXY = 0;//求x*y和
	double sumX2 = 0;//求y^2和
	double aveX;//求x平均数
	double aveY;//求y平均数
	ZhiXianData zhixian;//记录直线参数
    zhixian.flag = 1;//假设直线没有超出图像边界；
	for (i = 0; i < linelong; i++)//统计四项参数和
	{
		if(line[i] < 2 || line[i] > 77)
			zhixian.flag = 0;
		sumX += i;
		sumY += line[i];
		sumXY += i*line[i];
		sumX2 += i*i;
	}
    //计算平均数
	aveX = sumX / linelong;
	aveY = sumY / linelong;
    //计算斜率
	zhixian.a = (sumXY - sumX * sumY / linelong) / (sumX2 - sumX * sumX / linelong);
	//计算偏差
    zhixian.b = aveY - zhixian.a * aveX;
	zhixian.variance = 0;
    //计算方差
	for (i = 0; i < linelong; i++)
	{
		zhixian.variance += (line[i] - (zhixian.a*i + zhixian.b))*(line[i] - (zhixian.a*i + zhixian.b));
	}
    zhixian.variance /= linelong;
    //返回直线数据
    return zhixian;
}

/*
*环岛检测，in
*带障碍检测
*/
void ZuoHuanDaoIn()
{
  	if(zhaohao_huandao_state < 0) return;
	int x=49, y;//寻线用
	int lastx,prex;
	EdgePoint p1=(EdgePoint){9,9},p2=(EdgePoint){59,59};//画线用，存有初始值
	int line[60];//存线用
	int miny;
	for (y = 59; y > 4; y--)//寻边界
	{
	 	x=x-10;
		if (x<1)x = 1;
	  	if (obj_img[y][x] == 0x00) break;
		for (;obj_img[y][x] > 0x00; x++);//边界寻找
		line[59-y]=x;
		miny=y;
	}
	if (miny<10 && zhaohao_huandao_state == 0)//判断环岛第一步，右边线是不是直线
	{
		ZhiXianData zhixiandata = ifzhixian(line, 30);
		if (zhixiandata.flag != 0 && zhixiandata.a < 0 && zhixiandata.variance < 1)
			zhaohao_huandao_state = -1;//-1标记 下一步环岛验证用
		else
			return;
	}
	//不管是不是环岛，在这里先把补线点p2找到
	p2.x=line[0];
	p2.y=59;
	//判断
	if (zhaohao_huandao_state == -1)//判断环岛第二步，左边是否有拐点
	{
	  	x = line[0]-10;
		if(x<1)x=1;
	 	for (;obj_img[59][x] > 0x00; x--);//边界寻找
		prex=x;
		x = line[1]-10;
		if(x<1)x=1;
		for (;obj_img[58][x] > 0x00; x--);//边界寻找
		lastx=x;
		for (y = 57; y > miny; y--)
		{
		  	x = line[59-y]-10;
			if(x<1)x=1;
			for (;obj_img[y][x] > 0x00; x--);//边界寻找
			//if里面是环岛的判断条件
			if(y < 30 && x>8 && lastx-prex>5 && x-lastx<2 && obj_img[y+2][x-4] != 0x00 && obj_img[y-2][x+3] != 0x00)
			{
			 	ZhiXianData zhixiandata = ifzhixian(line+38-y, 20);
				if (zhixiandata.flag == 0 || zhixiandata.a > 0 || zhixiandata.variance > 1)
				{
					zhaohao_huandao_state = 0;//-1标记 下一步环岛验证用
					return;
				}
			  	y = y+12;
			  	for (int i = 0; i < 10; i++,y--)
				{
				  	x = line[59-y]-10;
					if(x<1)x=1;
					for (;obj_img[y][x] > 0x00; x--);//边界寻找
					line[i]=x;
				}
			  	ZhiXianData zhixian = ifzhixian(line,10);
				if (zhixian.flag != 0 && zhixian.a > 0 && zhixian.variance < 1)
				{
				  	zhaohao_zhangai_state = 1;
					zhaohao_huandao_state = 0;
					zhangaitime = zhangaisettime;
					printf("ZA L");
					return;
				}
				else
				{
					p1=(EdgePoint){x,y};
				 	//环岛判断成功，画线，并设置标志位
					QuXianHuaXian(p1, p2);
					zhaohao_huandao_state = 1;
					printf("HD L\n");
					return;
				}
			}
			prex=lastx;
			lastx=x;
		}
		zhaohao_huandao_state = 0;
	}
	else if (zhaohao_huandao_state == 1)//左环岛识别
	{
	  	x = line[0]-10;
		if(x<1)x=1;
	 	for (;obj_img[59][x] > 0x00; x--);//边界寻找
		prex=x;
		x = line[1]-10;
		if(x<1)x=1;
		for (;obj_img[58][x] > 0x00; x--);//边界寻找
		lastx=x;
		for (y = 57; y > miny; y--)
		{
			x = line[59-y]-10;
			if(x<1)x=1;
			for (;obj_img[y][x] > 0x00; x--);//边界寻找
			if(x>5 && lastx-prex>5 && x-lastx<3 && obj_img[y+3][x-4] != 0x00)
			{
			 	p1=(EdgePoint){x,y};
			  	if(y>35 || x > 50)zhaohao_huandao_state = 2;
			  	break;
			}
			prex=lastx;
			lastx=x;
		}
		QuXianHuaXian(p1, p2);
	}
	else zhaohao_huandao_state = 0;
}

void YouHuanDaoIn()
{
  	if(zhaohao_huandao_state > 0) return;
	int x=29, y;//寻线用
	int lastx,prex;
	EdgePoint p1=(EdgePoint){70,9},p2=(EdgePoint){20,59};//画线用，存有初始值
	int line[60];//存线用
	int miny;
	for (y = 59; y > 4; y--)//寻边界
	{
	 	x=x+10;
		if (x>78)x = 78;
	  	if (obj_img[y][x] == 0x00) break;
		for (;obj_img[y][x] > 0x00; x--);//边界寻找
		line[59-y]=x;
		miny=y;
	}
	if (miny<10 && zhaohao_huandao_state == 0)//判断环岛第一步，右边线是不是直线
	{
		ZhiXianData zhixiandata = ifzhixian(line, 30);
		if (zhixiandata.flag != 0 && zhixiandata.a > 0 && zhixiandata.variance < 1)
			zhaohao_huandao_state = 1;//1标记 下一步环岛验证用
		else
			return;
	}
	//不管是不是环岛，在这里先把补线点p2找到
	p2.x=line[0];
	p2.y=59;
	//判断
	if (zhaohao_huandao_state == 1)//判断环岛第二步，左边是否有拐点
	{
	  	x = line[0]+10;
		if(x>78)x=78;
	 	for (;obj_img[59][x] > 0x00; x++);//边界寻找
		prex=x;
		x = line[1]+10;
		if(x>78)x=78;
		for (;obj_img[58][x] > 0x00; x++);//边界寻找
		lastx=x;
		for (y = 57; y > miny; y--)
		{
		  	x = line[59 - y]+10;
			if(x>78)x=78;
			for (;obj_img[y][x] > 0x00; x++);//边界寻找
			//if里面是环岛的判断条件
			if(y < 30 && x<71 && prex-lastx>5 && lastx-x<2 && obj_img[y+2][x+4] != 0x00 && obj_img[y-2][x-3] != 0x00)
			{
			  	ZhiXianData zhixiandata = ifzhixian(line+38-y, 20);
				if (zhixiandata.flag == 0 || zhixiandata.a < 0 || zhixiandata.variance > 1)
				{
					zhaohao_huandao_state = 0;//-1标记 下一步环岛验证用
					return;
				}
			  	y = y+12;
			  	for (int i = 0; i < 10; i++,y--)
				{
				  	x = line[59-y]+10;
					if(x>78)x=78;
					for (;obj_img[y][x] > 0x00; x++);//边界寻找
					line[i]=x;
				}
				ZhiXianData zhixian = ifzhixian(line,10);
				if(zhixian.flag != 0 && zhixian.a < 0 && zhixian.variance < 1)
				{
				  	zhaohao_zhangai_state = -1;
					zhaohao_huandao_state = 0;
					zhangaitime = zhangaisettime;
					printf("ZA R\n");
					return;
				}
				else
				{
					p1=(EdgePoint){x,y};
				 	//环岛判断成功，画线，并设置标志位
					QuXianHuaXian(p1, p2);
					zhaohao_huandao_state = -1;
					printf("HD R\n");
					return;
				}
			}
			prex=lastx;
			lastx=x;
		}
		zhaohao_huandao_state = 0;
	}
	else if (zhaohao_huandao_state == -1)//左环岛识别
	{
	  	x = line[0]+10;
		if(x>78)x=78;
	 	for (;obj_img[59][x] > 0x00; x++);//边界寻找
		prex=x;
		x = line[1]+10;
		if(x>78)x=78;
		for (;obj_img[58][x] > 0x00; x++);//边界寻找
		lastx=x;
		for (y = 57; y > miny; y--)
		{
		  	x = line[59 - y]+10;
			if(x>78)x=78;
			for (;obj_img[y][x] > 0x00; x++);//边界寻找
			//if里面是环岛的判断条件
			if(x<74 && prex-lastx>5 && lastx-x<2 && obj_img[y+2][x+4] != 0x00 && obj_img[y-2][x-3] != 0x00)
			{
			 	p1=(EdgePoint){x,y};
			  	if(y>35 || x < 30)zhaohao_huandao_state = -2;
			  	break;
			}
			prex=lastx;
			lastx=x;
		}
		QuXianHuaXian(p1, p2);
	}
	else zhaohao_huandao_state = 0;
}

/*环岛检测，pigbi
*/
void ZuoHuanDaoPingBi()
{
    if(huandaotime==0) 
	{
		zhaohao_huandao_state = 0;
		printf("HD OT\n");
		return;
	}
	int x=49, y;//寻线用
	int lastx,prex;
	EdgePoint p1=(EdgePoint){9,9},p2=(EdgePoint){15,59};//画线用，存有初始值
	int line[60];//存线用
	int miny;
	for (y = 59; y > 9; y--)//寻边界
	{
	 	x=x-10;
		if (x<1)x = 1;
	  	if (obj_img[y][x] == 0x00) break;
		for (;obj_img[y][x] > 0x00; x++);//边界寻找
		line[59-y]=x;
		miny=y;
	}
	x = line[0]-10;
	if(x<1)x=1;
	for (;obj_img[59][x] > 0x00; x--);//边界寻找
	prex=x;
	x = line[1]-10;
	if(x<1)x=1;
	for (;obj_img[58][x] > 0x00; x--);//边界寻找
	lastx=x;
	p1=(EdgePoint){prex,59};
	for (y = 57; y > miny; y--)
	{
		x = line[59-y]-10;
		if(x<1)x=1;
		for (;obj_img[y][x] > 0x00; x--);//边界寻找
		if(x>5 && lastx-prex>5 && x-lastx<3 && obj_img[y+3][x-4] != 0x00)
		{
		 	p1=(EdgePoint){x,y};
			HuaXian(p1, p2);
		  	return;
		}
		prex=lastx;
		lastx=x;
	}
}
void YouHuanDaoPingBi()
{
  	if(huandaotime==0) 
	{
		zhaohao_huandao_state = 0;
		printf("HD OT\n");
		return;
	}
	int x=29, y;//寻线用
	int lastx,prex;
	EdgePoint p1=(EdgePoint){70,9},p2=(EdgePoint){65,59};//画线用，存有初始值
	int line[60];//存线用
	int miny;
	for (y = 59; y > 9; y--)//寻边界
	{
	 	x=x+10;
		if (x>78)x = 78;
	  	if (obj_img[y][x] == 0x00) break;
		for (;obj_img[y][x] > 0x00; x--);//边界寻找
		line[59-y]=x;
		miny=y;
	}
	x = line[0]+10;
	if(x>78)x=78;
	for (;obj_img[59][x] > 0x00; x++);//边界寻找
	prex=x;
	x = line[1]+10;
	if(x>78)x=78;
	for (;obj_img[58][x] > 0x00; x++);//边界寻找
	lastx=x;
	for (y = 57; y > miny; y--)
	{
		x = line[59-y]+10;
		if(x>78)x=78;
		for (;obj_img[y][x] > 0x00; x++);//边界寻找
		if(x <74 && prex-lastx>5 && lastx-x<2 && obj_img[y+2][x+4] != 0x00)
		{
		 	p1=(EdgePoint){x,y};
			HuaXian(p1, p2);
		  	return;
		}
		prex=lastx;
		lastx=x;
	}
}

/*环岛出
 * 
 */
void ZuoHuanDaoOut()
{
	EdgePoint p1,p2;
	int x=49,y;
	int lastx,prex;
	int line[60];
	int miny;
	for (y = 59; y > 9; y--)//寻边界
	{
	 	x=x-10;
		if (x<1)x = 1;
	  	if (obj_img[y][x] == 0x00) break;
		for (;obj_img[y][x] > 0x00; x++);//边界寻找
		line[59-y]=x;
		miny=y;
	}
	x = line[0]-10;
	if(x<1)x=1;
	for (;obj_img[59][x] > 0x00; x--);//边界寻找
	prex=x;
	x = line[1]-10;
	if(x<1)x=1;
	for (;obj_img[58][x] > 0x00; x--);//边界寻找
	lastx=x;
	for (y = 57; y > miny; y--)
	{
		x = line[59-y]-10;
		if(x<1)x=1;
		for (;obj_img[y][x] > 0x00; x--);//边界寻找
		if(x < 40 && x >9 && lastx-prex>5 && x-lastx<2 && obj_img[y+2][x-4] != 0x00)
		{
		  	p1=(EdgePoint){x,y};
			p2=(EdgePoint){15,59};
		  	zhaohao_huandao_state=3;
			huandaotime = huandaosettime;
			printf("HD P\n");
			HuaXian(p1, p2);
		  	return;
		}
		prex=lastx;
		lastx=x;
	}
	//计算补线点1
	for (p1.y = 59,p1.x=14; obj_img[p1.y][p1.x] == 0x00 && p1.y > 5; p1.y--);
	p1.y--;
	for (; p1.y > 1 && obj_img[p1.y][p1.x] > 0x00; p1.y--);
	//计算补点线2
	for (p2.x = 39, p2.y = 59; obj_img[p2.y][p2.x] > 0x00; p2.x++);
	//画线
	QuXianHuaXian(p1, p2);
}

void YouHuanDaoOut()
{
	EdgePoint p1,p2;
	int x=29,y;
	int lastx,prex;
	int line[60];
	int miny;
	for (y = 59; y > 9; y--)//寻边界
	{
	 	x=x+10;
		if (x>78)x = 78;
	  	if (obj_img[y][x] == 0x00) break;
		for (;obj_img[y][x] > 0x00; x--);//边界寻找
		line[59-y]=x;
		miny=y;
	}
	x = line[0]+10;
	if(x>78)x=78;
	for (;obj_img[59][x] > 0x00; x++);//边界寻找
	prex=x;
	x = line[1]+10;
	if(x>78)x=78;
	for (;obj_img[58][x] > 0x00; x++);//边界寻找
	lastx=x;
	for (y = 57; y > miny; y--)
	{
		x = line[59-y]+10;
		if(x>78)x=78;
		for (;obj_img[y][x] > 0x00; x++);//边界寻找
		if(x > 40 && x <70 && prex-lastx>5 && lastx-x<2 && obj_img[y+2][x+4] != 0x00)
		{
		  	p1=(EdgePoint){x,y};
			p2=(EdgePoint){64,59};
		  	zhaohao_huandao_state=-3;
			huandaotime = huandaosettime;
			printf("HD P\n");
			HuaXian(p1, p2);
		  	return;
		}
		prex=lastx;
		lastx=x;
	}
	//计算补线点1
	for (p1.y = 59,p1.x=65; obj_img[p1.y][p1.x] == 0x00 && p1.y > 5; p1.y--);
	p1.y--;
	for (; p1.y > 1 && obj_img[p1.y][p1.x] > 0x00; p1.y--);
	//计算补点线2
	for (p2.x = 39, p2.y = 59; obj_img[p2.y][p2.x] > 0x00; p2.x--);
	//画线
	QuXianHuaXian(p1, p2);
}


/* 斑马线识别
 *
 *
 */
void BanMaXian()
{
	int i,j;
	int banmaxian = 0;
	int flag = 0;
	for(i = 59; i > 15; i--)
	{
		banmaxian=0;
		flag = 0;
		for(j = 0; j < 79; j++)
		{
			if(flag == 0)
			{
				if(obj_img[i][j] != 0x00)
				{
					banmaxian++;
					flag = 1;
				}
			}
			else if(flag == 1)
			{
				if(obj_img[i][j] == 0x00)
				{
					banmaxian++;
					flag = 0; 
				}
			}
		}
		if(banmaxian > banmaxian_change)
		{
		  	if(i<20)
				img_clear_line(i+5,i-10);
			else if(i<40)
			  	img_clear_line(i+7,i-12);
			else 
			  	img_clear_line(i+10,i-15);
			zhaohao_banma_state = 1; 
			return;
		}
	}
	zhaohao_banma_state = 0;
}

void ZuoZhangAi()
{	
  	if(zhangaitime == 0)
	{
	  	printf("ZA OT");
		zhaohao_zhangai_state = 0;
		return;
	}
  	int x=49, y;//寻线用
	int lastx;
	int line[60];//存线用
	int miny;
	for (y = 59; y > 4; y--)//寻边界
	{
	 	x=x-10;
		if (x<1)x = 1;
	  	if (obj_img[y][x] == 0x00) break;
		for (;obj_img[y][x] > 0x00; x++);//边界寻找
		line[59-y]=x;
		miny=y;
	}
	x = line[0]-10;
	if(x<1)x=1;
	for (;obj_img[59][x] > 0x00; x--);//边界寻找
	lastx=x;
	EdgePoint p1=(EdgePoint){0,0},p2=(EdgePoint){35,15},p3=(EdgePoint){0,0},p4=(EdgePoint){lastx+(line[0]-lastx)/4,59};
	for (y = 58; y > miny; y--)
	{
		x = line[59-y]-10;
		if(x<1)x=1;
		for (;obj_img[y][x] > 0x00; x--);//边界寻找
		if(lastx-x>5)
		{
		 	p2=(EdgePoint){lastx+(line[60-y]-lastx)*zhangaiweight/100,y+1};
		}
		else if(x-lastx>5)
		{
		 	p3=(EdgePoint){x+(line[59-y]-x)*zhangaiweight/100,45};
		}
		lastx=x;
		p1=(EdgePoint){x,y};
	}
	HuaXian(p1,p2);
	if(p3.x==0) HuaXian(p2,p4);
	else
	{
		HuaXian(p2,p3);
		HuaXian(p3,p4);
	}
	if(p2.y>45)zhaohao_zhangai_state = 0;
}

void YouZhangAi()
{	
  	if(zhangaitime == 0)
	{
	  	printf("ZA OT");
		zhaohao_zhangai_state = 0;
		return;
	}
  	int x=29, y;//寻线用
	int lastx;
	int line[60];//存线用
	int miny;
	for (y = 59; y > 4; y--)//寻边界
	{
	 	x=x+10;
		if (x>78)x = 78;
	  	if (obj_img[y][x] == 0x00) break;
		for (;obj_img[y][x] > 0x00; x--);//边界寻找
		line[59-y]=x;
		miny=y;
	}
	x = line[0]+10;
	if(x>78)x=78;
	for (;obj_img[59][x] > 0x00; x++);//边界寻找
	lastx=x;
	EdgePoint p1=(EdgePoint){79,0},p2=(EdgePoint){45,15},p3=(EdgePoint){0,0},p4=(EdgePoint){lastx+(line[0]-lastx)/4,59};
	for (y = 58; y > miny; y--)
	{
		x = line[59-y]+10;
		if(x>78)x=78;
		for (;obj_img[y][x] > 0x00; x++);//边界寻找
		if(x-lastx>5)
		{
		 	p2=(EdgePoint){lastx+(line[60-y]-lastx)*zhangaiweight/100,y+1};
		}
		else if(lastx-x>5)
		{
		 	p3=(EdgePoint){x+(line[59-y]-x)*zhangaiweight/100,45};
		}
		lastx=x;
		p1=(EdgePoint){x,y};
	}
	HuaXian(p1,p2);
	if(p3.x==0) HuaXian(p2,p4);
	else
	{
		HuaXian(p2,p3);
		HuaXian(p3,p4);
	}
	if(p2.y>45)zhaohao_zhangai_state = 0;
}


void SDYS()
{
  	//相关程序时间的倒计时
 	if(huandaotime>0)huandaotime--;
	if(zhangaitime>0)zhangaitime--;
	//斑马线检测
	BanMaXian();
	if(startandfinish==1)
	{
                //printf("startandfinish\n");
	  	img_clear_line(49,59);
		startandfinish = 0;
		return;
	}
	if(zhaohao_banma_state==1) return;
	//添加边沿保护线
	for (int i = 0; i < 60;i++)
	{
	  obj_img[i][0]= 0x00;
	  obj_img[i][79]=0x00;
	}
	//中线保护
	for(int y = 59; y > 49; y--)
	{
	  	if(obj_img[y][39] == 0x00) return;
	}
	//障碍优先
	if(zhaohao_zhangai_state == 1)ZuoZhangAi();
	else if(zhaohao_zhangai_state == -1)YouZhangAi();
	//环岛带障碍检测
 	else if(zhaohao_huandao_state < 2 && zhaohao_huandao_state > -2)
	{
		ZuoHuanDaoIn();
		YouHuanDaoIn();
	}
	else if(zhaohao_huandao_state ==  2)ZuoHuanDaoOut();
	else if(zhaohao_huandao_state == -2)YouHuanDaoOut();
	else if(zhaohao_huandao_state ==  3)ZuoHuanDaoPingBi();
	else if(zhaohao_huandao_state == -3)YouHuanDaoPingBi();
	else zhaohao_huandao_state = 0;
}

void ShiZiWan()
{
	int i;
	int x,y;
	int lx,rx;
	int flag = 0;
	int yline[80];
	int miny_x = 79;
	int miny_x_10;
	int sumy;
	int last_sumy = 1200;
	if(zhaohao_huandao_state != 0 || zhaohao_zhangai_state != 0) return;
	x = 39;
	for(y = 50; y > 25; y--)
	{
		for (lx = x; lx > 1 && obj_img[y][lx] > 0x00; lx--);
		for (rx = x; rx < CAMERA_W-2 && obj_img[y][rx] > 0x00; rx++);
		if(lx == 1 && rx == CAMERA_W-2) flag++;
	}
	if (flag < 5) return;
	yline[miny_x]=59;
	for(x = 78;x > 1; x--)
	{
		for(y = 59; obj_img[y][x] == 0x00 && y > 5;y--);
		y--;
		for(;y > 1 && obj_img[y][x] > 0x00; y--);
		yline[x] = y;
		if(x>10 && x<70 && y < yline[miny_x])
			miny_x = x;
	}
	if(miny_x < 20)miny_x = 20;
	else if(miny_x > 59)miny_x = 59;
	miny_x_10 = miny_x +10;
	for(x = miny_x-10; x < miny_x_10; x++)
	{
		sumy = 0;
		for(i = x -10; i < x+10; i++)
			sumy+= yline[i];
		if(sumy < last_sumy)
		{
			miny_x = x;
			last_sumy = sumy;
		}
	}
	if(miny_x < 14)miny_x = 14;
	else if(miny_x > 64)miny_x = 64;
	HuaXian((EdgePoint){4,59},(EdgePoint){miny_x-10,45});
	HuaXian((EdgePoint){74,59},(EdgePoint){miny_x+10,45});
	HuaXian((EdgePoint){miny_x-10,45},(EdgePoint){miny_x-10,yline[miny_x-10]});
	HuaXian((EdgePoint){miny_x+10,45},(EdgePoint){miny_x+10,yline[miny_x+10]});
}

void SetHuanDaoTime(int time)
{
  	huandaosettime = time;
}

void SetZhangAiTime(int time)
{ 
  	zhangaisettime = time;
}

void SetBanMaChange(int change)
{
  	banmaxian_change = change;
}

void StartAndFinish()
{
  	startandfinish = 1;
}

void ClearState()
{
  	zhaohao_huandao_state = 0;//环岛状态
	zhaohao_zhangai_state = 0;//障碍状态
	zhaohao_banma_state = 0;//斑马线状态
	huandaotime = 0;
	zhangaitime = 0;
	StartAndFinish();
}
