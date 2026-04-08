
/*
 *  Created on: Dec 24, 2025
 *      Author: Hql2017
 */
#ifndef CAN_MODBUSRTU__H_
#define  CAN_MODBUSRTU__H_
/**
 * R:read 
 * W:write 
 * 非标准modbusRTU功能码
 * ************/
#define L980_MAX_MOTOR_DISTANCE_UM    35000  //32mm,最远运动距离
#define L980_MAX_PROHOT_WAIT_TIME  20*SYS_1_SECOND_TICKS//20s
#define L980_POSITON_WAIT_TIMEOUT  20*SYS_1_SECOND_TICKS//10s
#define L980_CAN_FRAME_TIMEOUT  1000
#define L980_CAN_MINI_TIME_MS  50//安全包间隔
#define L980_CODE_MASK  0x80
#define L980_REG_WRITE_MASK  0x80// write:bit7=1 ;read:bit7=0。
#define L980_REG_MASK   0x7F
typedef enum {		
	RTU_CODE_SINGLE_PACKAGE=0,//单包
	RTU_CODE_LONG_BYTES_PACKAGE  
}rtu_function_code;

typedef enum {
	L980_NO_OPTION=0,
	L980_REG_HEART_STATUS,  		//r只读		
	L980_REG_CTR_PRO_HOT,			//w准备(4Bytes) (unsigned short 0) energe;(unsigned  short 1) DACValue(>0 prohot；=0 exit prohot);
	L980_REG_JT_CTR_STOP,			//w启停	(2Bytes) (unsigned short 0)=ctr cmd;(0 stop)
	L980_REG_COUNTDOWN_TIMERS,      //r/w倒计时(2Bytes)	( short int 0) timers(180S) (>0 使能 <=0 失能)
	L980_REG_ENERGE_CALI_PARAM, 	//r/w(4Bytes)	(unsigned short 0)energe,(unsigned short 1）DAC(DAC值校准) ;r:realValue ;w:setValue
	L980_REG_PULSE_COUNT_AND_TIME,	//r/w激光运行时间(4bytes）（unsigned int 0） r:realTime;w:setTime.
	L980_REG_AUXILIARY_BULB,    	//r/w指示光(2Bytes) byte0=duty;byte1=freq; r:realValue;w:setValue
	L980_REG_MOTOR_POSITION,		//r/W偏光片位置(4Bytes)(μm)(unsigned short 0)set position。(unsigned short 1) ctr cmd(0：stop ;其他位置:>L980_MAX_MOTOR_DISTANCE_UM找0)	
	L980_REG_LASER_TEMPRATURE,		//r/w激光器温度(2Bytes)(10倍241:24.1℃) (short int 0) r:realSetT;w:setT.
	L980_REG_MULTIPLE_READ,         //多通道读
	L980_REG_MULTIPLE_WRITE,        //多通道写
	L980_REG_SYNC_CONFIG,           //同步配置数据写返回（1成功；0失败）	
	L980_REG_ENGINEER_MODE,         //工程模式，无需心跳(控制板不开放)(2Bytes)	( unsigned short int 0) 1，调试模式；0正常模式。
	L980_REG_TEC_CTR,              //制冷片开关（水循环异常需要关闭）//w启停	(2Bytes) (unsigned short 0)=ctr cmd;
	L980_REG_SYS_POWER_OFF,         //系统开关
	REG_AUX_REG,                    //r/w其他数据
}L980_cmd;

#define L980_STA_HEART_BIT0     0x01//1，ok;0 失败;
#define L980_STATUS_BYTE_MASK   0x0E//状态掩码(bit 1~3)
#define L980_STA_PROHOT_BIT1    0x02//0x01<<1//1，ok;0 准备失败。
#define L980_STA_PULSEOUT_BIT2  0x04//0x01<<2//1，输出ok;
#define L980_STA_TIMERS_BIT3    0x08//0x01<<3 //1，正在运行;0停止

#define L980_ERROR_HALF_MASK   0xF0//错误掩码,高4位
#define L980_ERR_TEMPRATURE_BIT4     0x10//0x01<<4//1，高温;0 正常
#define L980_ERR_ENERGE_BIT5         0x20//0x01<<5//1，激光能量过高
#define L980_ERR_LADER_POEER_BIT6    0x40//0x01<<6//1，激光电源异常
#define L980_ERR_MOTOR_BIT7          0x80//0x01<<7//1，丝杆移动异常
/** 
 * CAN数据帧解析
 * 读指令数据结构：
 * reg+(datalen=1)+(Nbytes 需要读取的数据长度)+crc
 * 读返回：reg+(datalen=N)+(Nbytes数据)+crc
 * 写指令数据结构：
 * reg+(datalen=N)+(Nbytes数据)+crc
 * pPkt：point package struct
 * 小端模式数据
 * “校验码高字节在前”
 * ************/
typedef struct { 
    unsigned char laser980Reg;
	unsigned char packLen;//数据部分长度
    unsigned char *data;//bytes
	unsigned char crcH;//crcH
    unsigned char crcL;	//crcL	
}__attribute__((packed)) L980_can_app_package;

typedef struct {	 
	short int  realtemprature;   			//10倍温度值（-40.0~150.0）
	unsigned char   staByte;				//heart; bit1 准备就绪;bit2 脉冲输出;bit3 倒计时 1正在运行;
	unsigned char   reserveByte;			//保留，0正常：!0工程模式
	unsigned short int    useEnerge;         //运行能量值	
	unsigned short int    dacValue;          //dac电压值0~1500mV			
	unsigned short int    energeFeedback;    	//能量计采集值
	unsigned short int    realPosition;         //实际位置
	unsigned  int    laserUseTimeS;   			//激光运行时间
	unsigned short int    tec_switch;               //制冷片开关
}__attribute__((packed)) L980_STATUS;			//状态数据
typedef union 
{
	L980_STATUS  sta;
	unsigned char data[sizeof(L980_STATUS)];//(16bit)
}U_L980_STATUS;
extern U_L980_STATUS  u_s_l980;
typedef struct {	 	
	short int   targetTempratureSet;      //温度值10倍（-40.0~150.0）
	unsigned char   auxLedBulbDutySet;    //指示灯占空比
	unsigned char   auxLedBulbFreqSet;    //指示灯频率	
	unsigned short int    positionSet;    //位置 
	short int    timerSet;       //倒计时>0 使能 ；<=0  失能。		
}__attribute__((packed)) L980_SET_PARAM;
typedef union 
{
	L980_SET_PARAM  set_param;
	unsigned char data[sizeof(L980_SET_PARAM)];
}U_L980_CONFIG_PARAM;
extern U_L980_CONFIG_PARAM u_l980;

void CAN_modbusRTU_init(void);
void CAN_receivePackageHandle(unsigned char *data,unsigned char packageType);
void CAN_RTU_transmitPackage(unsigned char function,unsigned char code,unsigned char *data);
void CAN_receivePackageHandle(unsigned char *data,unsigned char packageType);
#if 1//主设备
void L980_appWriteReg(unsigned char reg,unsigned char len,unsigned char *data);
void L980_appReadReq(unsigned char reg,unsigned char len);
#endif 
extern unsigned  int can_count;//test
#endif /*  CAN_MODBUSRTU__H_ */


