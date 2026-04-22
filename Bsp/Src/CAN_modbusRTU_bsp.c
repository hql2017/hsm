
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 

#include "CAN_modbusRTU_bsp.h"
#include "fdcan.h"
#include "tim.h"
#include "main.h"

U_L980_STATUS  u_s_l980;
U_L980_CONFIG_PARAM u_l980;
  /**
  * @brief CAN_modbusRTU_init
  * @param  void
  * @note   
  * @retval None
  */
void CAN_modbusRTU_init(void)
{
     
}
/************************************************************************//**
  * @brief  CAN_crc16Num
  * @param   
  * @note    CRC校验函数,LSB
  * @retval None
  ****************************************************************************/
static unsigned short int CAN_crc16Num(unsigned char *pData, int length)
{
	uint16_t crc = 0xFFFF;	
	for (int i = 0; i < length; i++)
	{
		crc ^= pData[i];			
		for (int j = 0; j < 8; j++)
		{
			if (crc & 1)
			{
				crc >>= 1;
				crc ^= 0xA001;			
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	return crc;
}
/************************************************************************//**
  * @brief  CAN_RTU_transmitPackage
  * @param   packNum:总包数
  * @note    
  * @retval None
  ****************************************************************************/
 void CAN_RTU_transmitPackage(unsigned char function,unsigned char packNum,unsigned char *data)
 {
  if(function==RTU_CODE_LONG_BYTES_PACKAGE)
  {//分包
    unsigned char transmitBuff[8];
    unsigned char tansNum=0; 
    transmitBuff[0]= packNum;//总包数  
    while(tansNum<packNum)
    {
      memcpy(&transmitBuff[1],data,6);
      transmitBuff[7]=tansNum;
      data+=6;
      tansNum++;      
      APP_CAN_SEND_DATA(transmitBuff,8,CAN_RTU_SLAVE_ID+RTU_CODE_LONG_BYTES_PACKAGE);
      HAL_Delay(1);
    }    
  }
  else 
  {       
    APP_CAN_SEND_DATA(data,8,CAN_RTU_SLAVE_ID);
  }  
 } 
 
 /************************************************************************//**
  * @brief  L980_appDataParaphrase
  * @param   
  * @note    can 数据处理
  * @retval None
  ****************************************************************************/
 void CAN_appLongPackageDataParaphrase(unsigned char *data,unsigned char Len)
 {

 }

  /************************************************************************//**
  * @brief  L980_appReadAck
  * @param    
  * @note   单通道读
  * @retval None
  ****************************************************************************/
 void L980_appReadAck(unsigned char reg,unsigned char *data)
 {
    switch(reg)
    {   
      case L980_REG_HEART_STATUS:
      DEBUG_PRINTF("sta \r\n"); 
        memcpy(u_s_l980.data,data,sizeof(L980_STATUS));         
        break;
      case L980_REG_PULSE_COUNT_AND_TIME:  
        // memcpy((unsigned char *)&u_s_l980.sta.laserUseTimeS,data,4);
        memcpy((unsigned char *)&u_sys_param.sys_config_param.laser_use_timeS,data,4);  
        break;
      case 	L980_REG_ENERGE_CALI_PARAM:      
        {  
          u_s_l980.sta.useEnerge=(data[1]<<8)|data[0];
          u_s_l980.sta.dacValue=(data[3]<<8)|data[2];
        }
      break;    
      case L980_REG_AUXILIARY_BULB:           
        u_l980.set_param.auxLedBulbDutySet=data[0];
        u_l980.set_param.auxLedBulbFreqSet=data[1];  
        break;   
      case L980_REG_MOTOR_POSITION: 
        u_l980.set_param.positionSet=(data[1]<<8)|data[0];
        u_s_l980.sta.realPosition=(data[3]<<8)|data[2];   
        break;
      case L980_REG_LASER_TEMPRATURE: 
        u_l980.set_param.targetTempratureSet=(data[1]<<8)|data[0];           
        break;
      case L980_REG_COUNTDOWN_TIMERS:            
        u_l980.set_param.timerSet= (data[1]<<8)|data[0];
        break;
      case L980_REG_SYNC_CONFIG:       
        memcpy(u_l980.data,data, sizeof(L980_SET_PARAM));
        break;        
      default:
        break;
    }
 }
 /************************************************************************//**
  * @brief  L980_appWriteAck
  * @param    
  * @note   单通道写
  * @retval None
  ****************************************************************************/
 void L980_appWriteAck(unsigned char reg,unsigned char *data)
 {
  switch(reg)
    {   
      case L980_REG_HEART_STATUS: 
          //memcpy(u_s_l980.data,data,sizeof(L980_STATUS));  
        break;
        case L980_REG_CTR_PRO_HOT: 
        {
          u_s_l980.sta.useEnerge=(data[1]<<8)|data[0];
          u_s_l980.sta.dacValue=(data[3]<<8)|data[2];
          if(u_s_l980.sta.dacValue>0)
          {
            DEBUG_PRINTF("980 prohot set energe ok=%d DAC=%d\r\n", u_s_l980.sta.useEnerge,u_s_l980.sta.dacValue); 
          }
          else
          {
            DEBUG_PRINTF("980 prohot set energe fail, exit prohot finish\r\n");
          }
        }
      break;    
      case L980_REG_JT_CTR_STOP:
          if(data[0]!=0)
          {       
            DEBUG_PRINTF("980 pulse out \r\n"); 
          }  
          else 
          {
            DEBUG_PRINTF("980 pusle stop\r\n");  
          }
          break;
      case L980_REG_PULSE_COUNT_AND_TIME:  
         //memcpy((unsigned char *)&u_s_l980.sta.laserUseTimeS,data,4); 
         memcpy((unsigned char *)&u_sys_param.sys_config_param.laser_use_timeS,data,4);
        break;
      case L980_REG_ENERGE_CALI_PARAM:
        {  
          u_s_l980.sta.useEnerge=(data[1]<<8)|data[0];   
          u_s_l980.sta.dacValue=(data[3]<<8)|data[2];          
        }
      break;    
      case L980_REG_AUXILIARY_BULB: 
        {
          u_l980.set_param.auxLedBulbDutySet=data[0];
          u_l980.set_param.auxLedBulbFreqSet=data[1];
        }  
        break;   
      case L980_REG_MOTOR_POSITION: 
          u_l980.set_param.positionSet=(data[1]<<8)|data[0];
          u_s_l980.sta.realPosition=(data[3]<<8)|data[2];   
        break;
      case L980_REG_LASER_TEMPRATURE:           
          u_l980.set_param.targetTempratureSet=(data[1]<<8)|data[0];           
        break;
      case L980_REG_COUNTDOWN_TIMERS: 
            u_l980.set_param.timerSet= (data[3]<<8)|data[2];          
           if( u_l980.set_param.timerSet> 0)
           {
            DEBUG_PRINTF("l980 timer enable=%ds\n",u_l980.set_param.timerSet);   
           }
           else
           {
            DEBUG_PRINTF("l980 timer disable\r\n");
           }   
          
        break;
       case  L980_REG_TEC_CTR:
          {
            u_s_l980.sta.tec_switch=(data[1]<<8)|data[0];
            if( u_s_l980.sta.tec_switch!=0)
            { 
              DEBUG_PRINTF("l980 tec on\r\n");   
            }
            else DEBUG_PRINTF("l980 tec off\r\n"); 
          }
       break;
      case L980_REG_SYNC_CONFIG:    
          if(data[0]!=0)  
          {
            DEBUG_PRINTF("l980 write config ok\r\n");            
          } 
          else  DEBUG_PRINTF("l980 write config fail\r\n");
          //memcpy(u_l980.data,data, sizeof(L980_SET_PARAM));
        break;        
      default:
        break;
    }
 }

 /************************************************************************//**
  * @brief  L980_appRegDataParaphrase
  * @param    
  * @note   L980应用数据解析
  * @retval None
  ****************************************************************************/
 void L980_appRegDataParaphrase(L980_can_app_package *pPkt,unsigned char functionCode)
 {   
    if((u_s_l980.sta.staByte&L980_STA_HEART_BIT0)!=L980_STA_HEART_BIT0)    
    {
      DEBUG_PRINTF("l980 _connnect success\r\n");
      u_s_l980.sta.staByte|=L980_STA_HEART_BIT0;  //heart
    }  
    if(functionCode == L980_REG_WRITE_MASK)
    {   
      L980_appWriteAck(pPkt->laser980Reg,pPkt->data);       
    }  
    else 
    { 
      DEBUG_PRINTF("r=%02x\r\n",pPkt->laser980Reg);
      L980_appReadAck(pPkt->laser980Reg,pPkt->data);
    }
 }
/************************************************************************//**
  * @brief  CAN_receivePackageHandle
  * @param  data:数据缓存；packageType 长包、短包
  * @note   标准帧，固定长度8bytes
  * typedef struct {   
          unsigned char packageNum;   //总包数
          unsigned char *data;        //6bytes	
          unsigned char currentNum;	//当前包序号	
      }__attribute__((packed)) can_long_package;
  * @retval None
  ****************************************************************************/
 void CAN_receivePackageHandle(unsigned char *data,unsigned char packageType)
 {  
    static unsigned char canAppDataBuff[128];
    static L980_can_app_package pL980;
    unsigned short int crcValue;  
    unsigned char len;
    unsigned char functionCode;
    
    if(packageType==RTU_CODE_SINGLE_PACKAGE) 
    { 
      len = data[1]+4;
      crcValue=(data[len-2]<<8)|data[len-1];  
      if(crcValue!=CAN_crc16Num(data,len-2)) return;     
      functionCode=data[0]&L980_REG_WRITE_MASK;      
      pL980.laser980Reg = data[0]&L980_REG_MASK;
      pL980.packLen = data[1];   
      pL980.data = &data[2];  
      memcpy(pL980.data,data,data[1]);   
      pL980.crcH = data[pL980.packLen+2];  
      pL980.crcL = data[pL980.packLen+3]; 
     L980_appRegDataParaphrase(&pL980,functionCode);
    } 
    else  
    {
      if(data[0]>16)   return;//too long
     
      if((data[0])>data[7])
      {   
        memcpy(&canAppDataBuff[data[7]*6],&data[1],6);         
        if(data[0]==(data[7]+1))
        {//重组结束 
          len = canAppDataBuff[1]+4;
          if(len>128) return ;          
          crcValue=(canAppDataBuff[len-2]<<8)|canAppDataBuff[len-1];
         
          if(crcValue!=CAN_crc16Num(canAppDataBuff,len-2)) return;  
                
          functionCode=canAppDataBuff[0]&L980_REG_WRITE_MASK;
          pL980.laser980Reg=canAppDataBuff[0]&L980_REG_MASK;
          
          if(pL980.laser980Reg==REG_AUX_REG)
          {         
            //CAN_LongPackageHandle(canAppDataBuff);
          }
          else
          {              
            pL980.packLen=canAppDataBuff[1];   
            pL980.data=&canAppDataBuff[2];     
            pL980.crcH= canAppDataBuff[pL980.packLen+2];  
            pL980.crcL= canAppDataBuff[pL980.packLen+3];  
            L980_appRegDataParaphrase(&pL980,functionCode);          
          } 
        }
      }
    }
 }
#if 1 
// 主设备
/************************************************************************//**
  * @brief  L980_appReadReq
  * @param  reg:起始地址 
  *         len:读取长度
  * @note   读请求
  * @retval None
  ****************************************************************************/
 void L980_appReadReq(unsigned char reg,unsigned char len)
 {  
    unsigned char sendBuff[8];  
    sendBuff[0]=reg; 
    sendBuff[1]=1; 
    sendBuff[2]=len;
    sendBuff[3]=(CAN_crc16Num(sendBuff,3)>>8)&0xFF;
    sendBuff[4]=CAN_crc16Num(sendBuff,3)&0xFF;  
    sendBuff[5]=0; 
    sendBuff[6]=0;
    sendBuff[7]=0;    
    CAN_RTU_transmitPackage(RTU_CODE_SINGLE_PACKAGE,1,sendBuff);
 }
 /************************************************************************//**
  * @brief  L980_appWriteReg
  * @param    
  * @note   写数据
  * @retval None
  ****************************************************************************/
 void L980_appWriteReg(unsigned char reg,unsigned char len,unsigned char *data)
 {  
    unsigned char sendBuff[64],packNum,packLen; 
    packLen=len+4;   
    if(len>64)  return ;   
    sendBuff[0]=reg|L980_REG_WRITE_MASK; 
    sendBuff[1]=len;
    if(len>0) memcpy(&sendBuff[2],data,len);
    sendBuff[len+2]=(CAN_crc16Num(sendBuff,len+2)>>8)&0xFF;
    sendBuff[len+3]=CAN_crc16Num(sendBuff,len+2)&0xFF; 
    if(packLen>8)
    {
      packNum=(packLen)/6;///8;
      if((packLen)%6!=0)
      {
        packNum+=1;         
        memset(&sendBuff[len+4],0,(packLen)%6);    
      }
      CAN_RTU_transmitPackage(RTU_CODE_LONG_BYTES_PACKAGE,packNum,sendBuff);
    }
    else
    { 
      packNum=1;
      if(packLen<8)   
      {
        memset(&sendBuff[packLen],0,8-packLen);    
      }   
      CAN_RTU_transmitPackage(RTU_CODE_SINGLE_PACKAGE,packNum,sendBuff);
    }
 }
  #endif
