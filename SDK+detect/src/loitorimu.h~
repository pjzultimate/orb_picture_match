#ifndef LOITORIMU_H
#define LOITORIMU_H

#include <sys/time.h>

/*
* IMU 数据结构体
*/
typedef struct 
{
	float imu_time;		// 从camera启动开始之后经历过的时间，以秒为单位
	timeval system_time;	// 本次IMU数据采集时刻的系统时间
	unsigned char num;	// 数据编号，从1-200 ，每秒钟归零一次
	float rx,ry,rz;			// 旋转角速度，单位：角度/s
	float ax,ay,az;		// 加速度，单位：m/s^2
	float qw,qx,qy,qz;	// 经过融合滤波之后输出的旋转姿态四元数
}visensor_imudata;

/*
*  相机开启时刻的时间戳
*/
extern timeval visensor_startTime;

bool visensor_query_imu_update();
bool visensor_mark_imu_update();
bool visensor_erase_imu_update();
int visensor_set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
int visensor_open_port(const char* dev_str);
int visensor_get_imu_frame(int fd, unsigned char* imu_frame);
int visensor_get_imu_data(unsigned char* imu_frame,short int* acc_offset,visensor_imudata *imudata_struct,bool show_data);
int visensor_send_imu_frame(int fd, unsigned char* data, int len);

#endif
