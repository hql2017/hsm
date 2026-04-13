/*
 *POWER_2000W_BSP_H_.h
 *
 *  Created on: Jan 26, 2026
 *      Author: Hql2017
 */

#ifndef POWER_2000W_BSP_H_
#define POWER_2000W_BSP_H_

#define P2000W_CODE_STA_QUERY    0x01
#define P2000W_CODE_VOLTAGE_SET  0x02
#define P2000W_CODE_PULSE_FREQ   0x03
#define P2000W_CODE_PULSE_WIDTH  0x04   //脉宽
#define P2000W_CODE_PRO_HOT_CTR  0x05 	//预燃
#define P2000W_CODE_RELEY_CTR    0x06	//继电器控制
#define P2000W_CODE_PULSE_OUT    0x07	//脉冲输出
#define P2000W_CODE_RECONNECT    0x08   //重连(读取软件版本号)7E 7E 0A 08 00 00 A0 15 0D 0A
 

#define P2000W_VOLTAGE_FREQ_SET_COMMON  0x90 //电压频率一起  
#define P2000W_CMD_ACK_MASK    0x7F//(code)（命令响应| 功能码）

#define P2000W_PRO_HOT_WAIT_TIMEOUT  12000 //预燃烧等待时间12s
#define P2000W_RELEY_WAIT_TIMEOUT  4000//继电器时间3.5s
#define P2000W_FRAME_TIMEOUT      300//200//阻塞时间ms
#define P2000W_FRAME_DELAY_TIME  100 //最小数据帧间隔100
   
#define P2000W_STATUS_MASK   0x1F//控制字(位值1有效，0失效)
#define P2000W_STA_B0_PULSE_OUT_OK  0x01//输出有效 1有效 0失效
#define P2000W_STA_B1_PFC_OK 0x02//PFC状态ok
#define P2000W_STA_B2_BURST_OK_  0x04//BURST  正常
#define P2000W_STA_B3_PRO_HOT_OK  0x08//预燃ok
#define P2000W_STA_B4_RELAY_OK  0x10//接触器输出ok
	
#define P2000W_ERROR_CODE_MASK   0x64//故障掩码 
#define P2000W_ERROR_LLC_OVERLOAD  0x04//LLC过流
#define P2000W_ERROR_PRO_HOT 0x20//预燃错误	
#define P2000W_ERROR_TEMPRATURE_OVERLOAD  0x40//过温	
 
 typedef struct {	 
	unsigned short int voltageRef;//电压参考值
	unsigned char temprature;// 整流桥温度 30 对应 -10° T=realT+40
	unsigned char c_temprature;// 电容温度T=realT+40 
	unsigned char ctr_status;// 控制状态
	unsigned char error_code;// 故障状态
	unsigned char init_status;//初始化状态字0,正在初始化 1：空闲等待 2：软启动 3：正常运行 4：故障	
}__attribute__ ((packed)) P_2000W_STATUS;//设备状态
extern P_2000W_STATUS p2000w_status;

typedef struct {	
	unsigned short int p2000wHeart;
	unsigned short int outVoltageSet;//输出电压值
	unsigned short int freqSet;//频率
	unsigned short int pulseWidthSet;//电脉宽
	unsigned short int proHotCtr;//预燃控制指令	
}P_2000W_SET_PARAM;//设置数据
extern P_2000W_SET_PARAM p2000w_ctr_param;	
void app_p2000W_receive_config( void );
void app_p2000w_init( void );
 unsigned short int app_p2000w_package_check(void) ;
 void app_p2000w_read_status_req(void);
 void app_p2000w_out_voltage_set(unsigned short int voltage);
 void app_p2000w_pulse_freq_set(unsigned short int  freq);
 void app_p2000w_pulse_width_set(unsigned short int  pulseWidthUs);
 void app_p2000w_ctr_tansmit(unsigned char code,unsigned char *ctrData);
 void app_p2000w_v_q_set(unsigned short int voltage,unsigned short int freq,unsigned short int  pulseWidthUs);

 void app_p2000w_pulse_start(unsigned char cmd);//0 stop
 
 #define LPUART_MAX_BUFF_LENGTH 64

#endif /* POWER_2000W_BSP_H_ */
 
