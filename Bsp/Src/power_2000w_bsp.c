/*
 * power_2000w_bsp.c
 *  only write
 *  Created on: Jan 26, 2026
 *      Author: Hql2017
 */
#include "stdio.h"
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "power_2000w_bsp.h"

#include "lwrb.h"
#define MAX_LWRB_UART_QUEN  128
static uint8_t lwrb_rx_buff[MAX_LWRB_UART_QUEN+1];  // 
static uint8_t gx_package_data[MAX_LWRB_UART_QUEN+1];  // 
static uint8_t uart_gx_rx_byte;  // 
static lwrb_t u_gx_lwrb;//
static uint8_t lwrb_len=0;
//power 2000W

typedef struct {
	//需要上传
   unsigned short int head;//0x7E7E
   unsigned char packLen;//package Len
   unsigned char code;//code
   unsigned char *data;//data
   unsigned short int crc;// 校验
   unsigned short int end;// 0x0A0D 
}__attribute__ ((packed)) PWR_2000W_APP_PACKAGE;//光纤应用数据解析
//0~200mJ =5*40
U_P2000W_TX_MSG u_p2000w_tx_msg;
P_2000W_STATUS p2000w_status;
P_2000W_SET_PARAM p2000w_ctr_param;	
static unsigned char p2000w_soft_version[5]={0x14,0xEA,0x07,0x04,0x06};
static HAL_StatusTypeDef app_p2000w_transmit(unsigned char code,unsigned char  *data,unsigned short int dataLen);
/***************************************************************************//**
 * @brief MODBUS/CRC-16
 * @param *data缓存，长度len
 * @note 多项式0xA001 
 * @return 返回crc16结果
*******************************************************************************/
static uint16_t p2000w_crc16(  uint8_t *data, uint16_t len)
{  	
	#if 1
	uint16_t crc = 0xFFFF;  // 初始值
	uint16_t polynomial=0xA001;
	for (size_t pos = 0; pos < len; pos++) 
	{
		crc ^= (uint16_t)data[pos];  
		for (uint8_t i = 8; i != 0; i--) 
		{  
			if ((crc & 0x0001) != 0) 
			{  // 如果最低位是1
				crc >>= 1;
				crc ^= polynomial;
			} 
			else 
			{
				crc >>= 1;
			}
		}
	}
	#else 
	uint16_t crc=CRC16_TABLE_Check(data,len);
	#endif
	return  crc;
} 

 /***************************************************************************//**
 * @brief app_p2000W_pack_handle
 * @param 
 * @note 光纤应用数据解析
 * @return 
*******************************************************************************/

void app_p2000W_pack_handle( unsigned  char *data,unsigned short int len )
{
	unsigned char code;
	PWR_2000W_APP_PACKAGE pPkt;
	pPkt.head=(data[1]<<8)|data[0];
	pPkt.packLen=data[2];
	pPkt.code=data[3];	
	pPkt.end=(data[(pPkt.packLen)-1]<<8)|data[(pPkt.packLen)-2];
	code=(pPkt.code)&P2000W_CMD_ACK_MASK;	
	#if 0   
		if(code!=P2000W_CODE_STA_QUERY) 
		{  
			DEBUG_PRINTF("GX_r:\r\n");
			for(int i=0;i<len;i++)
			{
			DEBUG_PRINTF(" %02x",data[i]);
			}
			DEBUG_PRINTF(" Len=%d\r\n",len); 
		}	
	  #endif	
	switch(code)
	{
		case P2000W_CODE_STA_QUERY:
			{    
				p2000w_status.voltageRef=(data[5]<<8)|data[4];//(pPkt.data[1]<<8)|pPkt.data[0];
				p2000w_status.temprature=data[6];//pPkt.data[2];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
				p2000w_status.c_temprature=data[7];//pPkt.data[3];
				p2000w_status.ctr_status=data[8];//pPkt.data[4];
				p2000w_status.error_code=data[9];//pPkt.data[5]; 
				p2000w_status.init_status=data[10];//pPkt.data[6];				  
			}
		break;
		case P2000W_CODE_VOLTAGE_SET:
			if(data[4]==0)
			{
				DEBUG_PRINTF("p2000w outvoltage Set fail");
			}
			else 
			{
				DEBUG_PRINTF("p2000w outvoltage Set successs");
			}						      
		break;
		case P2000W_CODE_PULSE_FREQ:
			if(data[4]==0)
			{
				DEBUG_PRINTF("p2000w freq Set fail");
			}
			else 
			{
				DEBUG_PRINTF("p2000w freq Set successs");
			}				
		break;
		case P2000W_CODE_PULSE_WIDTH:
				if(data[4]==0)
				{
					DEBUG_PRINTF("p2000w pulse width set fail");
				}
				else 
				{
					DEBUG_PRINTF("p2000w pulse width Set successs");
				}		
		break;
		case P2000W_CODE_PRO_HOT_CTR:
				if(data[4]==0)
				{
					if(p2000w_ctr_param.proHotCtr==0)  p2000w_ctr_param.proHotCtr=1;
					else p2000w_ctr_param.proHotCtr=0;
					DEBUG_PRINTF("p2000w prohot cmd send fail");
				}
				else 
				{					
					DEBUG_PRINTF("p2000w prohot cmd send successs");
				}	
		break;
		case P2000W_CODE_RELEY_CTR:
			if(data[4]==0)
			{
				DEBUG_PRINTF("p2000w reley cmd send fail");
			}
			else 
			{					
				DEBUG_PRINTF("p2000w reley cmd send successs");
			}
		break;
		case P2000W_CODE_PULSE_OUT:
			if(data[4]==0)
			{			
				DEBUG_PRINTF("p2000w pulse out cmd send fail");
			}
			else 
			{					
				DEBUG_PRINTF("p2000w  pulse out cmd send successs");
			}
		break;
		case  P2000W_VOLTAGE_FREQ_SET_COMMON:
			if(data[4]==0)
			{
				DEBUG_PRINTF("p2000w voltage freq pulsWidth set fail");
			}
			else 
			{					
				DEBUG_PRINTF("p2000w voltage freq pulsWidth set successs");
			}
		break;
		case P2000W_CODE_RECONNECT:
			if(data[4]!=0)
			{	
				for(unsigned char i=0;i<5;i++)
				{
					p2000w_soft_version[i]=data[4+i];
				}	
				DEBUG_PRINTF("p2000w Version=0x%02x%02x%02x%02x%02x\r\n",p2000w_soft_version[0],p2000w_soft_version[1],\
					p2000w_soft_version[2],p2000w_soft_version[3],p2000w_soft_version[4]);
			}
			else 
			{					
				DEBUG_PRINTF("p2000w read ver fail");
			}
		break;
		default:
		break;
	}	
}
/***************************************************************************//**
 * @brief app_p2000W_receive_config
 * @param 
 * @note  开始接收光纤数据
 * @return 
*******************************************************************************/
void app_p2000W_receive_config( void )
{	
	lwrb_init(&u_gx_lwrb,lwrb_rx_buff,MAX_LWRB_UART_QUEN);
	HAL_UART_Receive_IT(&hlpuart1,&uart_gx_rx_byte,1);
}
/***************************************************************************//**
 * @brief app_p2000w_init
 * @param 
 * @note  电源控制初始化
 * @return 
*******************************************************************************/
void app_p2000w_init( void )
{
	app_high_voltage_solenoid(ENABLE);
	memset(&p2000w_status,0,sizeof(P_2000W_STATUS));//clear
	memset(&p2000w_ctr_param,0,sizeof(P_2000W_SET_PARAM));//clear
	app_p2000W_receive_config();
}
 /***************************************************************************//**
 * @brief app_p2000W_uart_gx_receive
 * @param 
 * @note  光纤接收数据缓存
 * @return 
*******************************************************************************/
void app_p2000W_uart_gx_receive( void )
{
	lwrb_len=lwrb_write(&u_gx_lwrb, &uart_gx_rx_byte, 1);//
	HAL_UART_Receive_IT(&hlpuart1,&uart_gx_rx_byte,1);	
}
/***************************************************************************//**
 * @brief app_p2000w_package_check
 * @param 
 * @note  20ms轮询
 * @return 
*******************************************************************************/
unsigned short int app_p2000w_package_check(void) 
{
	static unsigned int peekLen=8,skipLen=0;
	unsigned char packLen=0;
	unsigned int readLen = lwrb_peek(&u_gx_lwrb,skipLen,gx_package_data,peekLen);
	if(readLen>0)
	{	
		#if 0      
		DEBUG_PRINTF("GX_r:\r\n");
		for(int i=0;i<readLen;i++)
		{
		  DEBUG_PRINTF(" %02x",gx_package_data[skipLen+i]);
		}
		DEBUG_PRINTF(" Len=%d\r\n",skipLen); 	
	  #endif	
		while(skipLen<readLen)
		{			
			if(gx_package_data[skipLen]==0x7E&&gx_package_data[skipLen+1]==0x7E)
			{//find head
				packLen=gx_package_data[skipLen+2];
				if(packLen>readLen)	
				{
					peekLen = packLen;  
					break;                  
				}
				else 
				{
					if(gx_package_data[skipLen+packLen-1]==0x0A&&gx_package_data[skipLen+packLen-2]==0x0D)
					{
						unsigned short int  crcValue = (gx_package_data[skipLen+packLen-3]<<8)|gx_package_data[skipLen+packLen-4]; 
						if(crcValue == p2000w_crc16(&gx_package_data[skipLen],packLen-4))					
						{
							peekLen = packLen;
							app_p2000W_pack_handle(&gx_package_data[skipLen],packLen);												
						}
					}					
					skipLen+=packLen;
					break;
				}			
			}			
			skipLen++;
		}
		lwrb_skip(&u_gx_lwrb,skipLen);
		skipLen=0;					
	}
	return packLen;
} 
 /************************************************************************//**
  * @brief 读状态
  * @param  
  * @note  
  * @retval 
  *****************************************************************************/
 void app_p2000w_read_status_req(void)
 {
	HAL_StatusTypeDef err;
	unsigned char dataBuff[2];	
	dataBuff[0]=0;//len	
	dataBuff[1]=0;//len
	err = app_p2000w_transmit(P2000W_CODE_STA_QUERY,dataBuff,2);	
 }
 /************************************************************************//**
  * @brief 重连，软件复位
  * @param  
  * @note 软件复位
  * @retval 
  *****************************************************************************/
 void app_p2000w_re_connect_req(void)
 {
	HAL_StatusTypeDef err;
	unsigned char dataBuff[2];	
	dataBuff[0]=0;	
	dataBuff[1]=0;
	err = app_p2000w_transmit(P2000W_CODE_RECONNECT,dataBuff,2);	
 }
  /************************************************************************//**
  * @brief 设置电源输出电压值
  * @param voltage
  * @note   12000~23200(320V~600V,800V~31200(铒激光备用))
  *         7000~23200(200V~600V,800V~31200(铒激光备用))
  *       CAN_send_V=(Vout/1216)*4.7*(4096/3.3)*8;
  * @retval 
  *****************************************************************************/
 void app_p2000w_out_voltage_set(unsigned short int voltage)
 {
	HAL_StatusTypeDef err;
	unsigned char dataBuff[2];
	unsigned short int txValue;
	//txValue=(voltage/1216)*4.7*(4096/3.3)*8;=(unsigned short int)(voltage*38.3795);
	if(voltage<LASER_1064_MIN_ENERGE_V) txValue=7675;
	else if(voltage>LASER_1064_MAX_ENERGE_V) txValue=23000;
	//else txValue=voltage*38.3795853269537;//((voltage/1216)*4.7*(4096/3.3)*8);
	else txValue=(unsigned short  )(((float)voltage*19251.2)/501.6);			
	dataBuff[0]=txValue&0xFF;
	dataBuff[1]=(txValue>>8)&0xFF;
	err = app_p2000w_transmit(P2000W_CODE_VOLTAGE_SET,dataBuff,2);		
 }
/************************************************************************//**
* @brief 设置脉冲频率
* @param  freq
* @note  1~100Hz
* @retval 
*****************************************************************************/
 void app_p2000w_pulse_freq_set(unsigned short int  freq)
 {
	HAL_StatusTypeDef err;
	unsigned char dataBuff[2];
	if(freq>100) freq=100;
	if(freq<1)   freq=1;
	dataBuff[0]=freq&0xFF;//小端
	dataBuff[1]=(freq>>8)&0xFF;
	err = app_p2000w_transmit(P2000W_CODE_PULSE_FREQ,dataBuff,2);	
 }
  /************************************************************************//**
  * @brief 设置脉冲宽度
  * @param  pulseWidthUs
  * @note  1us分辨率
  * @retval 
  *****************************************************************************/
 void app_p2000w_pulse_width_set(unsigned short int  pulseWidthUs)
 {		
	HAL_StatusTypeDef err;
	unsigned char dataBuff[2];
	if(pulseWidthUs<80) pulseWidthUs=80;//100us
	if(pulseWidthUs>240) pulseWidthUs=240;//240us
	dataBuff[0]=(pulseWidthUs)&0xFF;
	dataBuff[1]=((pulseWidthUs)>>8)&0xFF;
	err= app_p2000w_transmit(P2000W_CODE_PULSE_WIDTH,dataBuff,2);
 }
 /***************************************************************************//**
 * @brief 脉宽频率一起设置
 * @param 
 * @note  
 * @return 
*******************************************************************************/
void app_p2000w_v_q_set(unsigned short int voltage,unsigned short int freq,unsigned short int  pulseWidthUs)
{
	HAL_StatusTypeDef err;
	unsigned char dataBuff[6];
	unsigned short int txValue;
	if(voltage<LASER_1064_MIN_ENERGE_V) txValue=12000;
	else if(voltage>LASER_1064_MAX_ENERGE_V) txValue=23200;
	else txValue=(voltage-LASER_1064_MIN_ENERGE_V)*40+12000;
	dataBuff[0]=txValue&0xFF;//小端
	dataBuff[1]=(txValue>>8)&0xFF;
	if(freq>100) freq=100;
	if(freq<1)   freq=1;
	dataBuff[2]=freq&0xFF;//小端
	dataBuff[3]=(freq>>8)&0xFF;
	if(pulseWidthUs<100) pulseWidthUs=100;//100us
	if(pulseWidthUs>240) pulseWidthUs=240;//240us
	dataBuff[4]=(pulseWidthUs/20)&0xFF;
	dataBuff[5]=((pulseWidthUs/20)>>8)&0xFF;
	err = app_p2000w_transmit(P2000W_VOLTAGE_FREQ_SET_COMMON,dataBuff,4);	
}
   /************************************************************************//**
  * @brief 输出控制
  * @param  ctrCmd 控制命令
  * @note  
  * @retval 
  *****************************************************************************/
 void app_p2000w_ctr_tansmit(unsigned char code,unsigned char *ctrData)
 {	
	HAL_StatusTypeDef err=HAL_OK;	
	
	unsigned short int cmdTemp=(ctrData[1]<<8)|ctrData[0];
	if(code==P2000W_CODE_PRO_HOT_CTR)
	{
		if(cmdTemp!=0)
		{
			if((p2000w_status.ctr_status&P2000W_STA_B3_PRO_HOT_OK)!=P2000W_STA_B3_PRO_HOT_OK)
			{
				err = app_p2000w_transmit(code,ctrData,2);
			}
		}
		else 
		{
			err = app_p2000w_transmit(code,ctrData,2);
		}
	}
	else if (code==P2000W_CODE_RELEY_CTR)
	{
		if(cmdTemp!=0)
		{
			if((p2000w_status.ctr_status&P2000W_STA_B3_PRO_HOT_OK)==P2000W_STA_B3_PRO_HOT_OK)
			{
				if((p2000w_status.ctr_status&P2000W_STA_B4_RELAY_OK)!=P2000W_STA_B4_RELAY_OK)
				{
					err = app_p2000w_transmit(code,ctrData,2);
				}
			}
			else
			{
				DEBUG_PRINTF("p2000w prohot not complete!\r\n");
			}
		}
		else 
		{			
			err = app_p2000w_transmit(code,ctrData,2);			
		}
	}
	else if (code==P2000W_CODE_PULSE_OUT)
	{
		if(cmdTemp!=0)
		{
			if((p2000w_status.ctr_status&P2000W_STA_B4_RELAY_OK)==P2000W_STA_B4_RELAY_OK)
			{
				if((p2000w_status.ctr_status&P2000W_STA_B0_PULSE_OUT_OK)!=P2000W_STA_B0_PULSE_OUT_OK)
				{
					err = app_p2000w_transmit(code,ctrData,2);
				}	
			}
			else
			{
				DEBUG_PRINTF("p2000w reley not open,must open reley!\r\n");
			}
		}
		else 
		{
			err = app_p2000w_transmit(code,ctrData,2);				
		}
		
	}	
	else if (code==P2000W_CODE_STA_QUERY)
	{
		if(p2000w_status.init_status!=4)
		{
			app_p2000w_read_status_req();			
		}
		else
		{
			DEBUG_PRINTF("p2000w init fail!\r\n");
		}
	}
	else if(code==P2000W_CODE_RECONNECT)
	{	
		if((p2000w_status.ctr_status&P2000W_STA_B1_PFC_OK)!=P2000W_STA_B1_PFC_OK)
		{
			app_p2000w_re_connect_req();
		}
	}
	else if(code==P2000W_CODE_VOLTAGE_SET)
	{
		app_p2000w_out_voltage_set(cmdTemp);
	}
	else if(code==P2000W_CODE_PULSE_FREQ)
	{
		app_p2000w_pulse_freq_set(cmdTemp);
	}
	else if(code==P2000W_CODE_PULSE_WIDTH)
	{
		app_p2000w_pulse_width_set(cmdTemp);
	}
	else 
	{
		DEBUG_PRINTF("control cmd is error\r\n");
	}	
 } 
 /***************************************************************************//**
 * @brief 向电源发送数据包
 * @param 
 * @note  
 * @return 返回操作状态
*******************************************************************************/
HAL_StatusTypeDef app_p2000w_transmit(unsigned char code,unsigned char  *data,unsigned short int dataLen)
{ 
	HAL_StatusTypeDef err;
	unsigned char  transmitBuff[32];
	if (dataLen > 24) {  // 确保总长度不超过缓冲区大小
        return HAL_ERROR;
    }
	transmitBuff[0]=0x7E;
	transmitBuff[1]=0x7E;
	transmitBuff[2]=dataLen+8;
	transmitBuff[3]=code;
	if (dataLen != 0 && data != NULL) 
	{
   	 memcpy(&transmitBuff[4],data,dataLen);
	}
	uint16_t crc = p2000w_crc16(transmitBuff, dataLen + 4);
	transmitBuff[dataLen+4] = crc&0xFF;
	transmitBuff[dataLen+5] = (crc>>8)&0xFF;
	transmitBuff[dataLen+6] = 0x0D;
	transmitBuff[dataLen+7] = 0x0A;
	 // 等待设备空闲
	 uint32_t timeout=0;
	 while (HAL_GPIO_ReadPin(IX9_BUSY_INT_in_GPIO_Port, IX9_BUSY_INT_in_Pin) == GPIO_PIN_SET) 
	 {
        HAL_Delay(1);
		timeout++;
		if(timeout>5)
		{
			break;
		}
    }
	// 发送数据（首次尝试失败后重试一次）
    err = HAL_UART_Transmit(&hlpuart1, transmitBuff, dataLen + 8, 10);
    if (err == HAL_BUSY || err == HAL_TIMEOUT) 
	{
        HAL_Delay(1);  // 短暂延迟后重试
        err = HAL_UART_Transmit(&hlpuart1, transmitBuff, dataLen + 8, 10);
    }
	return err;
}
/***************************************************************************//**
 * @brief 脉冲输出
 * @param 
 * @note  
 * @return 
*******************************************************************************/
void app_p2000w_pulse_start(unsigned char cmd)
{
	unsigned char dataBuff[2];
	if(cmd==0) //stop
	{
		dataBuff[0]=0;
		dataBuff[1]=0;
		app_p2000w_ctr_tansmit(P2000W_CODE_PULSE_OUT,0);
	}
	else 
	{
		dataBuff[0]=cmd;
		dataBuff[1]=0;
		app_p2000w_ctr_tansmit(P2000W_CODE_PULSE_OUT,dataBuff);
	}
}
 








