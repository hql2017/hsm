/*
 * common_function.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef COMMON_FUNCTION_H_
#define COMMON_FUNCTION_H_

#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"
#include "main.h"
//PID结构体,控制块
typedef struct { 
    float set_value;//设定值 
    float actual_value;//实际值
    float error;//偏差 
    float error_next;//上一个偏差 
    float error_last;//上上一个偏差 
    float kp,ki,kd;//定义比例，积分，微分参数 
    }PID_ControlBlock;

// 定义卡尔曼滤波器结构
typedef struct {
    double last_estimate; // 上次估计值
    double estimate; // 当前估计值
    double variance; // 估计噪声方差
    double kalman_gain; // 卡尔曼增益
    double error_estimate; // 误差估计值
} KalmanFilter;
// 2字节大小端
#define SWAP2BYTES(num) do{\
	u16 temp = num;\
	num = (((temp) >> 8) | (((temp) & 0xFF) << 8));\
	}while(0);
	
// 4字节大小端
#define SWAP4BYTES(num) do{\
u32 temp = num;\
num = ((((temp) >> 24) & 0xFF) | (((temp) >> 8) & 0xFF00) | (((temp) << 8) & 0xFF0000) | (((temp) << 24) & 0xFF000000));\
}while(0);
	
	
extern  unsigned short int  crc16Num(unsigned char *pData,unsigned  int length);
extern void DWT_Init(void);
extern void delay_us(volatile unsigned int  nus);
unsigned int sumCheck(unsigned char *pData,unsigned  int length);
extern unsigned  int crc32_MPEG(unsigned char *pData,unsigned  int length);
void kalman_filter_init(KalmanFilter* kf, double initial_estimate, double variance) ;
double kalman_filter_update(KalmanFilter* kf, double measurement) ;
void sord_data(unsigned  short int *dataBuff,unsigned  short int length,unsigned char flag );//排序
 unsigned short CRC16_TABLE_Check(unsigned char *str, unsigned short len);
 unsigned  short int match_max(unsigned  short int *dataBuff,unsigned  short int length );

 extern float pid_realise_updata(PID_ControlBlock *pid,float speed);
 extern int Position_PID (PID_ControlBlock *pid,int feedbackValue,int target);
 extern void pid_init(PID_ControlBlock *pid );


HAL_StatusTypeDef compare_buff_no_change(unsigned char *buff1,unsigned char *buff2,unsigned int length);
#endif /* COMMON_FUNCTION */

