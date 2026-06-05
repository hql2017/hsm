/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include <math.h>
#include "string.h"
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#include "fdcan.h"
#include "iwdg.h"

#include "air_gzp6816d_bsp.h"
#include "eeprom_bsp.h"
#include "max31865atp_bsp.h"
#include "IS31FL3193_bsp.h" 
#include "tmc2226_step_bsp.h"
#include "mer_mcp1081_bsp.h"
#include "drv_RF24L01.h"
#include "user_can1.h"
#include "CAN_modbusRTU_bsp.h"
#include "power_2000w_bsp.h"
#include "fan_bsp.h"
#include "buzz_bsp.h"
 #include "max31865atp_bsp.h"
#ifdef ONE_WIRE_BUS_JT_SLAVE
#include "one_wire_bus.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */
sys_genaration_status sGenSta;
sys_enviroment_assistant_param sEnvParam;
LASER_CONTROL_PARAM  laser_ctr_param;
U_SYS_CONFIG_PARAM   u_sys_param;
U_SYS_CONFIG_PARAM   u_sys_default_param;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//auxStatusEvent01
#define EVENTS_AUX_STATUS_IO1_BIT  0x01
#define EVENTS_AUX_STATUS_IO2_BIT  0x01<<1
#define EVENTS_AUX_STATUS_IO3_BIT  0x01<<2
#define EVENTS_AUX_STATUS_IO4_BIT  0x01<<3
#define EVENTS_AUX_STATUS_IO5_BIT  0x01<<4
#define EVENTS_AUX_STATUS_IO6_BIT  0x01<<5
#define EVENTS_AUX_STATUS_IO7_BIT  0x01<<6//治疗水容量ok
#define EVENTS_AUX_STATUS_IO8_BIT  0x01<<7                                            
#define EVENTS_AUX_STATUS_9_NTC_BIT                         0x0001<<8
#define EVENTS_AUX_STATUS_10_IBUS_BIT                       0x0001<<9
#define EVENTS_AUX_STATUS_11_VBUS_BIT                       0x0001<<10
#define EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT           		0x0001<<11
#define EVENTS_AUX_STATUS_13_K2_TEMPRATURE_BIT           		0x0001<<12
#define EVENTS_AUX_STATUS_14_EMERGENCY_KEY_BIT              0x0001<<13
#define EVENTS_AUX_STATUS_15_WATER_AIR_PREPARE_BIT          0x0001<<14//水雾准备
#define EVENTS_AUX_STATUS_16_COOL_WATER_BIT                 0x0001<<15//冷却液位正常

#define EVENTS_AUX_STATUS_ALL_BITS     (EVENTS_AUX_STATUS_IO1_BIT|EVENTS_AUX_STATUS_IO2_BIT|EVENTS_AUX_STATUS_IO3_BIT|EVENTS_AUX_STATUS_IO4_BIT|EVENTS_AUX_STATUS_IO5_BIT\
			|EVENTS_AUX_STATUS_IO6_BIT|EVENTS_AUX_STATUS_IO7_BIT|EVENTS_AUX_STATUS_IO8_BIT|EVENTS_AUX_STATUS_9_NTC_BIT|EVENTS_AUX_STATUS_10_IBUS_BIT|EVENTS_AUX_STATUS_11_VBUS_BIT\
			|EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT|EVENTS_AUX_STATUS_13_K2_TEMPRATURE_BIT|EVENTS_AUX_STATUS_14_EMERGENCY_KEY_BIT| EVENTS_AUX_STATUS_15_WATER_AIR_PREPARE_BIT |EVENTS_AUX_STATUS_16_CLEAN_BIT\
    )
//laserEvent02Handle  
#define EVENTS_LASER_JT_ENABLE_BIT        	      0x01      //脚踏开放
#define EVENTS_LASER_1064_PREPARE_OK_BIT        	0x01<<1   //1064laser
#define EVENTS_LASER_980_PREPARE_OK_BIT  					0x01<<2   //980laser
#define EVENTS_LASER_PREPARE_OK_ALL_BITS_MASK        	(EVENTS_LASER_JT_ENABLE_BIT|EVENTS_LASER_1064_PREPARE_OK_BIT|EVENTS_LASER_980_PREPARE_OK_BIT)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 296 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
uint32_t myTask03Buffer[ 128 ];
osStaticThreadDef_t myTask03ControlBlock;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .cb_mem = &myTask03ControlBlock,
  .cb_size = sizeof(myTask03ControlBlock),
  .stack_mem = &myTask03Buffer[0],
  .stack_size = sizeof(myTask03Buffer),
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for myTask08 */
osThreadId_t myTask08Handle;
const osThreadAttr_t myTask08_attributes = {
  .name = "myTask08",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for myTask09 */
osThreadId_t myTask09Handle;
const osThreadAttr_t myTask09_attributes = {
  .name = "myTask09",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for myTask10 */
osThreadId_t myTask10Handle;
const osThreadAttr_t myTask10_attributes = {
  .name = "myTask10",
  .stack_size = 176 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for myTask11 */
osThreadId_t myTask11Handle;
const osThreadAttr_t myTask11_attributes = {
  .name = "myTask11",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for myTask12 */
osThreadId_t myTask12Handle;
const osThreadAttr_t myTask12_attributes = {
  .name = "myTask12",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for rgbQueue02 */
osMessageQueueId_t rgbQueue02Handle;
const osMessageQueueAttr_t rgbQueue02_attributes = {
  .name = "rgbQueue02"
};
/* Definitions for musicQueue03 */
osMessageQueueId_t musicQueue03Handle;
const osMessageQueueAttr_t musicQueue03_attributes = {
  .name = "musicQueue03"
};
/* Definitions for keyJTMessageQueue01 */
osMessageQueueId_t keyJTMessageQueue01Handle;
const osMessageQueueAttr_t keyJTMessageQueue01_attributes = {
  .name = "keyJTMessageQueue01"
};
/* Definitions for p2000wTxMessageQueue04 */
osMessageQueueId_t p2000wTxMessageQueue04Handle;
uint8_t p2000wTxMessageQueue04Buffer[ 4 * sizeof( uint32_t ) ];
osStaticMessageQDef_t p2000wTxMessageQueue04ControlBlock;
const osMessageQueueAttr_t p2000wTxMessageQueue04_attributes = {
  .name = "p2000wTxMessageQueue04",
  .cb_mem = &p2000wTxMessageQueue04ControlBlock,
  .cb_size = sizeof(p2000wTxMessageQueue04ControlBlock),
  .mq_mem = &p2000wTxMessageQueue04Buffer,
  .mq_size = sizeof(p2000wTxMessageQueue04Buffer)
};
/* Definitions for canTxQueue05 */
osMessageQueueId_t canTxQueue05Handle;
uint8_t myQueue05Buffer[ 4 * 8 ];
osStaticMessageQDef_t myQueue05ControlBlock;
const osMessageQueueAttr_t canTxQueue05_attributes = {
  .name = "canTxQueue05",
  .cb_mem = &myQueue05ControlBlock,
  .cb_size = sizeof(myQueue05ControlBlock),
  .mq_mem = &myQueue05Buffer,
  .mq_size = sizeof(myQueue05Buffer)
};
/* Definitions for laserWorkTimer01 */
osTimerId_t laserWorkTimer01Handle;
osStaticTimerDef_t laserWorkTimer01ControlBlock;
const osTimerAttr_t laserWorkTimer01_attributes = {
  .name = "laserWorkTimer01",
  .cb_mem = &laserWorkTimer01ControlBlock,
  .cb_size = sizeof(laserWorkTimer01ControlBlock),
};
/* Definitions for cleanTimer02 */
osTimerId_t cleanTimer02Handle;
const osTimerAttr_t cleanTimer02_attributes = {
  .name = "cleanTimer02"
};
/* Definitions for tmcMaxRunTimer03 */
osTimerId_t tmcMaxRunTimer03Handle;
const osTimerAttr_t tmcMaxRunTimer03_attributes = {
  .name = "tmcMaxRunTimer03"
};
/* Definitions for p2000wHeartTimer04 */
osTimerId_t p2000wHeartTimer04Handle;
osStaticTimerDef_t p2000wHeartTimer04ControlBlock;
const osTimerAttr_t p2000wHeartTimer04_attributes = {
  .name = "p2000wHeartTimer04",
  .cb_mem = &p2000wHeartTimer04ControlBlock,
  .cb_size = sizeof(p2000wHeartTimer04ControlBlock),
};
/* Definitions for beepHearttTimer05 */
osTimerId_t beepHearttTimer05Handle;
const osTimerAttr_t beepHearttTimer05_attributes = {
  .name = "beepHearttTimer05"
};
/* Definitions for powerOffBinarySem02 */
osSemaphoreId_t powerOffBinarySem02Handle;
const osSemaphoreAttr_t powerOffBinarySem02_attributes = {
  .name = "powerOffBinarySem02"
};
/* Definitions for laserPrapareReqSem03 */
osSemaphoreId_t laserPrapareReqSem03Handle;
const osSemaphoreAttr_t laserPrapareReqSem03_attributes = {
  .name = "laserPrapareReqSem03"
};
/* Definitions for laserCloseSem05 */
osSemaphoreId_t laserCloseSem05Handle;
const osSemaphoreAttr_t laserCloseSem05_attributes = {
  .name = "laserCloseSem05"
};
/* Definitions for hmiCanBusIdleSem06 */
osSemaphoreId_t hmiCanBusIdleSem06Handle;
const osSemaphoreAttr_t hmiCanBusIdleSem06_attributes = {
  .name = "hmiCanBusIdleSem06"
};
/* Definitions for CANBusReceiveFrameSem04 */
osSemaphoreId_t CANBusReceiveFrameSem04Handle;
const osSemaphoreAttr_t CANBusReceiveFrameSem04_attributes = {
  .name = "CANBusReceiveFrameSem04"
};
/* Definitions for p2000wHeartBinarySem07 */
osSemaphoreId_t p2000wHeartBinarySem07Handle;
const osSemaphoreAttr_t p2000wHeartBinarySem07_attributes = {
  .name = "p2000wHeartBinarySem07"
};
/* Definitions for coolWaterDepthBinarySem01 */
osSemaphoreId_t coolWaterDepthBinarySem01Handle;
const osSemaphoreAttr_t coolWaterDepthBinarySem01_attributes = {
  .name = "coolWaterDepthBinarySem01"
};
/* Definitions for auxStatusEvent01 */
osEventFlagsId_t auxStatusEvent01Handle;
osStaticEventGroupDef_t auxStatusEvent01ControlBlock;
const osEventFlagsAttr_t auxStatusEvent01_attributes = {
  .name = "auxStatusEvent01",
  .cb_mem = &auxStatusEvent01ControlBlock,
  .cb_size = sizeof(auxStatusEvent01ControlBlock),
};
/* Definitions for laserEvent02 */
osEventFlagsId_t laserEvent02Handle;
osStaticEventGroupDef_t laserEvent02ControlBlock;
const osEventFlagsAttr_t laserEvent02_attributes = {
  .name = "laserEvent02",
  .cb_mem = &laserEvent02ControlBlock,
  .cb_size = sizeof(laserEvent02ControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
unsigned  int app_get_cali_devid(void);
void app_set_default_sys_config_param(void);
void app_sys_genaration_status_manage(void);
void app_sys_param_load(void);
unsigned char app_sys_param_save_data(void);
void app_air_pump_manage(unsigned char air_level);
void app_pwr_gx_semo(unsigned char code);
void app_fresh_laser_status_param(void);
unsigned short int  app_laser_1064_energe_to_voltage(unsigned short int energe);
unsigned short int  app_laser_980_energe_to_voltage(unsigned short int energe);
unsigned short int app_hmi_package_check(unsigned char* pBuff,unsigned short int buffLen) ;
void app_t_clean_run_timer(unsigned char *runflag);
void app_treatment_water_prepare(unsigned char *ctrflag,unsigned int runtimeMs);
void app_buzz_music(music_type  music_num,unsigned char volume);
void app_jdq_restart(void);
void app_sram_status_monitor( void );
void app_circle_water_PTC_manage(float circleWaterTmprature,unsigned  int sysTimeMs);

void app_p2000w_status_handle( P_2000W_STATUS *pSta );
void app_p2000w_pulse_auto_adjust_voltage( unsigned short int targetEnerge,unsigned short int realEnerge,unsigned short int voltageSet);

#ifdef ONE_WIRE_BUS_SLAVE
unsigned int  app_owb_key_scan(unsigned short int timeMs);
#endif

//1064能量值对应电压表0~200mJ，环境温度27度


static unsigned short int energe_140us_voltage[41]={ 200,220,230,245,260,280,
  290,300,310,320,330,340,345,350,355,360,365,375,385,395,405,415,425,
  432,440,448,456,464,472,480,488,496,500,505,510,515,520,525,
530,535,540};
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void auxTask02(void *argument);
void keyScanTask03(void *argument);
void laserWorkTask04(void *argument);
void fastAuxTask05(void *argument);
void hmiAppTask06(void *argument);
void canReceiveTask07(void *argument);
void powerOffTask08(void *argument);
void laserProhotTask09(void *argument);
void ge2117ManageTask10(void *argument);
void musicTask11(void *argument);
void p2000wReceiveTask12(void *argument);
void LaserWorkTimerCallback01(void *argument);
void cleanWaterCallback02(void *argument);
void tmcMaxRunTimesCallback03(void *argument);
void p2000wHeartCallback04(void *argument);
void beepHeartCallback05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
  // osThreadGetState(defaultTaskHandle);
   #ifdef IWDG_USED
   HAL_IWDG_Refresh(&hiwdg1); 
   #endif 
}
/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of powerOffBinarySem02 */
  powerOffBinarySem02Handle = osSemaphoreNew(1, 0, &powerOffBinarySem02_attributes);

  /* creation of laserPrapareReqSem03 */
  laserPrapareReqSem03Handle = osSemaphoreNew(1, 0, &laserPrapareReqSem03_attributes);

  /* creation of laserCloseSem05 */
  laserCloseSem05Handle = osSemaphoreNew(1, 0, &laserCloseSem05_attributes);

  /* creation of hmiCanBusIdleSem06 */
  hmiCanBusIdleSem06Handle = osSemaphoreNew(1, 0, &hmiCanBusIdleSem06_attributes);

  /* creation of CANBusReceiveFrameSem04 */
  CANBusReceiveFrameSem04Handle = osSemaphoreNew(1, 0, &CANBusReceiveFrameSem04_attributes);

  /* creation of p2000wHeartBinarySem07 */
  p2000wHeartBinarySem07Handle = osSemaphoreNew(1, 0, &p2000wHeartBinarySem07_attributes);

  /* creation of coolWaterDepthBinarySem01 */
  coolWaterDepthBinarySem01Handle = osSemaphoreNew(1, 1, &coolWaterDepthBinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of laserWorkTimer01 */
  laserWorkTimer01Handle = osTimerNew(LaserWorkTimerCallback01, osTimerOnce, NULL, &laserWorkTimer01_attributes);

  /* creation of cleanTimer02 */
  cleanTimer02Handle = osTimerNew(cleanWaterCallback02, osTimerOnce, NULL, &cleanTimer02_attributes);

  /* creation of tmcMaxRunTimer03 */
  tmcMaxRunTimer03Handle = osTimerNew(tmcMaxRunTimesCallback03, osTimerOnce, NULL, &tmcMaxRunTimer03_attributes);

  /* creation of p2000wHeartTimer04 */
  p2000wHeartTimer04Handle = osTimerNew(p2000wHeartCallback04, osTimerPeriodic, NULL, &p2000wHeartTimer04_attributes);

  /* creation of beepHearttTimer05 */
  beepHearttTimer05Handle = osTimerNew(beepHeartCallback05, osTimerPeriodic, NULL, &beepHearttTimer05_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of rgbQueue02 */
  rgbQueue02Handle = osMessageQueueNew (3, sizeof(uint16_t), &rgbQueue02_attributes);

  /* creation of musicQueue03 */
  musicQueue03Handle = osMessageQueueNew (3, sizeof(uint16_t), &musicQueue03_attributes);

  /* creation of keyJTMessageQueue01 */
  keyJTMessageQueue01Handle = osMessageQueueNew (3, sizeof(uint16_t), &keyJTMessageQueue01_attributes);

  /* creation of p2000wTxMessageQueue04 */
  p2000wTxMessageQueue04Handle = osMessageQueueNew (4, sizeof(uint32_t), &p2000wTxMessageQueue04_attributes);

  /* creation of canTxQueue05 */
  canTxQueue05Handle = osMessageQueueNew (4, 8, &canTxQueue05_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(auxTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(keyScanTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(laserWorkTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(fastAuxTask05, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(hmiAppTask06, NULL, &myTask06_attributes);

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(canReceiveTask07, NULL, &myTask07_attributes);

  /* creation of myTask08 */
  myTask08Handle = osThreadNew(powerOffTask08, NULL, &myTask08_attributes);

  /* creation of myTask09 */
  myTask09Handle = osThreadNew(laserProhotTask09, NULL, &myTask09_attributes);

  /* creation of myTask10 */
  myTask10Handle = osThreadNew(ge2117ManageTask10, NULL, &myTask10_attributes);

  /* creation of myTask11 */
  myTask11Handle = osThreadNew(musicTask11, NULL, &myTask11_attributes);

  /* creation of myTask12 */
  myTask12Handle = osThreadNew(p2000wReceiveTask12, NULL, &myTask12_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */ 
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of auxStatusEvent01 */
  auxStatusEvent01Handle = osEventFlagsNew(&auxStatusEvent01_attributes);

  /* creation of laserEvent02 */
  laserEvent02Handle = osEventFlagsNew(&laserEvent02_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  *     typedef struct { 
      unsigned char systemParamFlag;                        //系统参数，0未加载；1待机；2开启；3异常；
      unsigned char jtFlag;                                 //脚踏输入，0未加载；1待机；2开启；3异常；
      unsigned char rgbFlag;                                //rgb氛围灯，0未加载；1待机；2开启；3异常；
      unsigned char auxiliary_bulbFlag;                     //激光指示灯，0未加载；1待机；2开启；3异常；
      unsigned char hmiLcdLoadFlag;                         //显示屏，0未加载；1待机；2开启；3异常；
      unsigned char coolWaterSystemLoadFlag;                //冷却水系统，0未加载；1待机；2开启；3异常；
      unsigned char treatmentWaterSystemLoadFlag;           //治疗水系统，0未加载；1待机；2开启；3异常；
      unsigned char tempratureSystemLoadFlag;               //温度系统，0未加载；1待机；2开启；3异常；
      unsigned char eTempratureAirpressureSystemLoadFlag;   //环境温度气压系统，0未加载；1待机；2开启；3异常；
      unsigned char airPressureSystemLoad
      Flag;              //气压系统，0未加载；1待机；2开启；3异常；
      unsigned char laserPowerSystemLoadFlag;               //激光电源系统，0未加载；1待机；2开启；3异常；
    }__attribute__ ((packed)) SYS_LOAD_STATUS;//辅助系统加载状态
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  uint32_t timeout;
  uint8_t load_sta;
  float treatmentWaterC; 
  app_mcu_power_switch(ENABLE);    
  app_lcd_power_12V_switch(ENABLE);   
  for(;;)
  { 
    //load 
    HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);    
    app_beep_pwm(music_tab_c[14],50);
    HAL_Delay(400);     
    app_beep_pwm(0,0);  
    do
		{
			HAL_Delay(200);//beep
			DEBUG_PRINTF("please release power key%d\r\n",HAL_GPIO_ReadPin(KEY_PWR_SWITCH_GPIO_Port,KEY_PWR_SWITCH_Pin));				 
		}while(HAL_GPIO_ReadPin(KEY_PWR_SWITCH_GPIO_Port,KEY_PWR_SWITCH_Pin)==GPIO_PIN_RESET);
    DEBUG_PRINTF("load system config param\r\n");
    EEPROM_M24C32_init();	
    app_sys_param_load();       
    DEBUG_PRINTF("DEVICEID=%x\r\n",u_sys_param.sys_config_param.equipmentId);
    DEBUG_PRINTF("load hmi lcd power...\r\n");  
    app_lcd_power_12V_switch(ENABLE);   
    DEBUG_PRINTF("load rgb...  \r\n");
    IS3_init();   
    load_sta=0; 
    DEBUG_PRINTF("load circle water system...\r\n"); 
    app_circle_water_pump_switch( ENABLE );     
    timeout=0;
    do
    { 
      HAL_Delay(100);
      timeout+=100; 
      if(timeout>5000)
      {
        app_circle_water_pump_switch( DISABLE);
        DEBUG_PRINTF("load circle load fail  \r\n");
        load_sta=3;//err
        break;
      }
    }while(app_get_io_status(In8_water_circle_ok)!=SUCCESS);   
    if(load_sta==0)  
    {
      DEBUG_PRINTF("load circle load ok  \r\n");      
    }     
    load_sta = 0;
    DEBUG_PRINTF("load enciroment airpressure system...\r\n");
    GZP6816D_init();    
    GZP6816D_start_sampling(); 	
    timeout=0;
    do
    { 
      HAL_Delay(300);
      timeout+=300;   
      if(timeout>2000)
      { 
        DEBUG_PRINTF("eViromentAirPressure load fail \r\n"); 
        sEnvParam.air_gzp_enviroment_pressure_kpa=94.0;        
        load_sta=3;// err
        break;
      }          
    }while(GZP6816D_IsBusy()!=0);               
    if(load_sta==0)  
    {
      GZP6816D_get_cal(&sEnvParam.air_gzp_enviroment_pressure_kpa,&sEnvParam.enviroment_temprature);  
      DEBUG_PRINTF("eViromentAirPressure load ok %.2f kPa  eviroment temprature =%.1f ℃\r\n",sEnvParam.air_gzp_enviroment_pressure_kpa,sEnvParam.enviroment_temprature); 
    }  
    load_sta=0;  
    DEBUG_PRINTF("load cool tmprature  measure system...\r\n");   
		max_31865_pt1000();	
    HAL_Delay(300);//50HZ
    float temp_t_f=Get_pt_tempture();
		if(temp_t_f<60.0&&temp_t_f>-40.0)
		{
			sEnvParam.eth_k1_temprature= temp_t_f;
			sEnvParam.eth_k2_temprature = temp_t_f; 			
		}
		else 
		{
			sEnvParam.eth_k1_temprature= 25.5; //err
			sEnvParam.eth_k2_temprature = 25.0; 
			load_sta=1;			
		}
    if(load_sta==0) 
    {
      DEBUG_PRINTF("PT1000 load ok k1_T=%.1f k2_T=%.1f \r\n",sEnvParam.eth_k1_temprature,sEnvParam.eth_k2_temprature); 
    }
    else DEBUG_PRINTF("PT1000 load fail  \r\n");
    /***********NTC,laser_energe,iBus,vBus,air_pump_pressure气泵气压，参数****************** */    
    app_start_multi_channel_adc();
    DEBUG_PRINTF("load adc sampling NTC ,laserenergetic,iBus,vbus,air pressure...\r\n");
    HAL_Delay(100);//wait >64ms 
    app_get_adc_value(AD1_NTC_INDEX,&sEnvParam.NTC_temprature);
   // app_get_adc_value(AD2_LASER_1064_INDEX,&sEnvParam.laser_1064_energy);
    app_get_adc_value(AD1_OCP_Ibus_INDEX,&sEnvParam.iBus);
    app_get_adc_value(AD1_24V_VBUS_INDEX,&sEnvParam.vBus);    
    app_get_adc_value(AD1_AIR_PRESSER_INDEX,&sEnvParam.air_pump_pressure);
    app_get_adc_value(AD1_WATER_PRESSER_INDEX,&sEnvParam.treatment_water_pressure);
    DEBUG_PRINTF("adc load:NTC=%.2f℃ laser_energe=%.1f iBus=%.1fmA ,vBus=%.1fmV,air_pump_pressure=%.2fkPa,treat_water_pressure=%.2fkPa\r\n",sEnvParam.NTC_temprature,\
      sEnvParam.laser_1064_energy,sEnvParam.iBus,sEnvParam.vBus,sEnvParam.air_pump_pressure,sEnvParam.treatment_water_pressure); 
     
    if(sEnvParam.NTC_temprature>-40&&sEnvParam.NTC_temprature<150)
    {      
      DEBUG_PRINTF("adc load:NTC_ad_channel ok=%.2f℃\r\n",sEnvParam.NTC_temprature);         
    }
    else
    {
      DEBUG_PRINTF("adc load:NTC_ad_channel error\r\n"); 
    } 
    if(sEnvParam.laser_1064_energy>50)
    {
      DEBUG_PRINTF("adc load:laser_1064_energy_ad_channel error\r\n");    
    }     
    if(sEnvParam.iBus>MAX_IBUS_MA)
    {
      DEBUG_PRINTF("adc load:ibus_ad_channel error\r\n");      
    }   
    else 
    {
      DEBUG_PRINTF("adc load:ibus_ad_channel ok=%.2fmA\r\n",sEnvParam.iBus); 
    }       
    if(sEnvParam.vBus<MIN_VBUS_MV)
    {
      DEBUG_PRINTF("adc load:vbus_ad_channel error\r\n");
    }   
    else 
    {      
      DEBUG_PRINTF("adc load:vbus_ad_channel ok=%.2fmV\r\n",sEnvParam.vBus); 
    }  
    if(sEnvParam.air_pump_pressure+20<sEnvParam.air_gzp_enviroment_pressure_kpa)
    {
      DEBUG_PRINTF("adc load:air_pressure_ad_channel error\r\n");
    }   
    else 
    {                
      DEBUG_PRINTF("adc load:air_pressure_ad_channel ok=%.2fkPa\r\n",sEnvParam.air_pump_pressure); 
    }   
    
    load_sta=0;
    DEBUG_PRINTF("load treatment water system...\r\n");
    tmc2226_init();   
    if(app_get_io_status(In7_water_ready_ok)!=SUCCESS) 
    {
      tmc2226_start(TMC_WATER_OUT_DIR_VALUE,3,CONTINUOUS_STEPS_COUNT);
    }
    timeout=0;
    do
    { 
      HAL_Delay(50);
      timeout+=50; 
      app_get_adc_value(AD1_WATER_PRESSER_INDEX,&sEnvParam.treatment_water_pressure);
      if(timeout>5000)
      {
        DEBUG_PRINTF("treatment water load fail \r\n");          
        load_sta=3;
        break;
      }
    }while(app_get_io_status(In7_water_ready_ok)!=SUCCESS);     
    tmc2226_stop();    
    if(load_sta==0)
    {     
      load_sta=1;
      DEBUG_PRINTF(" treatment water load ok \r\n");   
    }    
    #ifdef IWDG_USED
    MX_IWDG1_Init(); 
    #endif      		

    osThreadTerminate(defaultTaskHandle);    
   //osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_auxTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_auxTask02 */
void auxTask02(void *argument)
{
  /* USER CODE BEGIN auxTask02 */
  /* Infinite loop */
  uint32_t led_tick;
	uint16_t rgbRun=1; 
  led_tick=0;
  //app_mcp61_calibration(0);
  //Fan
  fan_init(); 
  for(;;)
  {
		if(osKernelGetTickCount()>=led_tick+1000)//1000)
		{          
			led_tick=osKernelGetTickCount();	
      app_fan_manage(1000);	
			HAL_GPIO_TogglePin(MCU_SYS_health_LED_GPIO_Port,MCU_SYS_health_LED_Pin); 
      app_sram_status_monitor();  
      //DEBUG_PRINTF("air pre=%.1f water_pressure=%.1f \r\n",sEnvParam.air_pump_pressure,sEnvParam.treatment_water_pressure);//
     // DEBUG_PRINTF("temprature=%.1f  \r\n",sEnvParam.eth_k1_temprature);     
		}	    
		/**********************RGB****************************/		
		osStatus_t rgb_s=osMessageQueueGet(rgbQueue02Handle,&rgbRun,0,5);	 
    switch(rgbRun)
    {
      case 0:
        rgb_color_all(0);        
      break;
      case RGB_G_STANDBY:      
        {
          Green_Breath(u_sys_param.sys_config_param.rgb_light); 
          osDelay(15);//frq=(1000/(64*(15+1) ))*2
          if(rgb_s==osOK)	        
          {
            fan_spd_set(FAN25_NUM,1000);
            fan_spd_set(FAN38_COMPRESSOR_NUM,1000);          
          }
          else {
            if(sEnvParam.eth_k1_temprature>30.0)	
            {//high temprature
              fan_spd_set(FAN25_NUM,3000);
              fan_spd_set(FAN38_COMPRESSOR_NUM,3000);       
            }
            else  if(sEnvParam.eth_k1_temprature>24.0)	
            {
              fan_spd_set(FAN25_NUM,2000);
              fan_spd_set(FAN38_COMPRESSOR_NUM,2000);       
            }
            else  if(sEnvParam.eth_k1_temprature>10.0)	
            {
              fan_spd_set(FAN25_NUM,1000);
              fan_spd_set(FAN38_COMPRESSOR_NUM,1000);       
            }
          }
        }
      break;
      case RGB_LASER_PREPARE_OK:
        {
          rgb_color_all(2);
          if(rgb_s==osOK)	
          {
            if(sEnvParam.eth_k1_temprature>32.0)	
            {//high temprature
              fan_spd_set(FAN25_NUM,4000);
              fan_spd_set(FAN38_COMPRESSOR_NUM,4000);       
            }
            else if(sEnvParam.eth_k1_temprature>28.0)	
            {//high temprature
              fan_spd_set(FAN25_NUM,3000);
              fan_spd_set(FAN38_COMPRESSOR_NUM,3000);       
            }
            else  if(sEnvParam.eth_k1_temprature>10.0)	
            {
              fan_spd_set(FAN25_NUM,2000);
              fan_spd_set(FAN38_COMPRESSOR_NUM,2000);       
            }         
          }
        }
      break;
      case RGB_LASER_WORK_STATUS:
        app_rgb_breath_ctl(laser_ctr_param.laserFreq,9); 
      break;
      default:
        rgb_color_all(0);        
      break;
    }		
    osDelay(1);
  }
  /* USER CODE END auxTask02 */
}

/* USER CODE BEGIN Header_keyScanTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_keyScanTask03 */
void keyScanTask03(void *argument)
{
  /* USER CODE BEGIN keyScanTask03 */
  unsigned int recKeyValue,rf24KeyValue;
	app_key_message key_message; 
  static app_key_message history_key_message;   
	RF24_init();
  #ifdef ONE_WIRE_BUS_JT_SLAVE
  one_wire_bus_init();
  #endif  
	/*Infinite loop */    
	for(;;)
	{ 
		osDelay(50);
    recKeyValue = app_IO_key_scan(50); 
    rf24KeyValue  = app_RF24_key_scan(50);      
    if(u_sys_param.sys_config_param.jt_status!=0&&sEnvParam.JT_ID==u_sys_param.sys_config_param.jtId)
    {
      if((recKeyValue&0XFF)==IO_KEY_IDLE)
      {
        if(rf24KeyValue!=KEY_NO_CONNECT)
        {        
          recKeyValue|=rf24KeyValue;
        }      
      } 
      else if((recKeyValue&0XFF)==KEY_NO_CONNECT)
      {        
        recKeyValue&=0xFF00;
        recKeyValue|=rf24KeyValue;
      } 
    }
    else 
    {
      recKeyValue&=0xFF00;
    } 
		key_message=app_key_value_analysis(recKeyValue);		
		if(key_message==key_pwr_long_press)
		{
			osSemaphoreRelease(powerOffBinarySem02Handle);
			key_message = NO_KEY_MESSAGE;	
      history_key_message=key_message;
		}
		else  
		{	      
      if(app_remote_key_sta()==ERROR)
      {
        if(sGenSta.laser_param_B7_ykls_status!=0) DEBUG_PRINTF("JT remote locked\r\n");
        sGenSta.laser_param_B7_ykls_status = 0;
        if(history_key_message!=key_jt_release)
        {           
          if(osMessageQueuePut(keyJTMessageQueue01Handle,&history_key_message,0,0)==osOK)  
          { 
            history_key_message = key_jt_release;
            key_message = NO_KEY_MESSAGE;                
          } 
        }
      }
      else 
      {
        if(sGenSta.laser_param_B7_ykls_status==0) sGenSta.laser_param_B7_ykls_status=1;//release
        if(key_message!=NO_KEY_MESSAGE)
        {
          if(osEventFlagsGet(laserEvent02Handle)!=0)
          {  
            if(osMessageQueuePut(keyJTMessageQueue01Handle,&key_message,0,0)==osOK)  
            {            
              //DEBUG_PRINTF("JT key press %d\r\n",key_message);
              history_key_message = key_message;
              key_message =	NO_KEY_MESSAGE;                									
            }  
          }
          else 
          {
            if(history_key_message!=key_jt_release)
            {     
              if(osMessageQueuePut(keyJTMessageQueue01Handle,&history_key_message,0,0)==osOK)  
              { //DEBUG_PRINTF("JT key press %d\r\n",key_message);                
                history_key_message=key_jt_release;
                key_message =	NO_KEY_MESSAGE;									
              }              	
            }           
          } 
        }                        
      }
		}		
	}
  /* USER CODE END keyScanTask03 */
}

/* USER CODE BEGIN Header_laserWorkTask04 */
/**
* @brief Function implementing the laserTask04 thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_laserWorkTask04 */
void laserWorkTask04(void *argument)
{
  /* USER CODE BEGIN laserWorkTask04 */
  /* Infinite loop */
  uint8_t recKeyMessage;	
	uint16_t rgbMessage;	//0关闭//1待机绿色2：准备OK紫色常亮；3脉冲输出紫色呼吸
	uint32_t timeout=0;
  osStatus_t statusJT;
  float e_feedback,fisrt_pulse_cali;//首脉冲校准
	LASER_CONTROL_PARAM *pLaserConfig;
	pLaserConfig = &laser_ctr_param;
  uint32_t event;
  uint32_t jdq_Volate_heart=0; 
  uint8_t l980_cmdBuff[4];
  osStatus_t m_stat;
  osStatus_t p_mtxs;
  osStatus_t can_tx_sta; 
  unsigned int energe_over_count=0,energe_down_count=0;
  U_CAN_TX_MSG u_CAN_tx_t_msg;
  U_P2000W_TX_MSG u_p2000w_msg;
  unsigned short int test_energe;
  for(;;)
  {      
    event=osEventFlagsWait(laserEvent02Handle,EVENTS_LASER_PREPARE_OK_ALL_BITS_MASK,osFlagsNoClear,portMAX_DELAY);
    osStatus_t laser_close_sem = osSemaphoreAcquire(laserCloseSem05Handle,10);    
    statusJT= osMessageQueueGet(keyJTMessageQueue01Handle,&recKeyMessage,0,10);  
    if(laser_ctr_param.timerEnableFlag==0||laser_ctr_param.timerCtr==0) 
    {
      sGenSta.laser_run_B5_timer_status=0;     
      osTimerStop(laserWorkTimer01Handle);
    }
    if(event==(EVENTS_LASER_1064_PREPARE_OK_BIT|EVENTS_LASER_JT_ENABLE_BIT))
    {      
      if(laser_close_sem==osOK&&sGenSta.laser_run_B0_pro_hot_status)
      {     
        osTimerStop(laserWorkTimer01Handle);
        sGenSta.laser_run_B5_timer_status=0;               
        tmc2226_stop();  
        app_deflate_air_solenoid(DISABLE);       
        if(p2000w_ctr_param.p2000wHeart==0) {
         // app_high_voltage_solenoid(DISABLE);
        }  
        else {  
          if((p2000w_status.ctr_status&P2000W_STA_B0_PULSE_OUT_OK)==P2000W_STA_B0_PULSE_OUT_OK)
          {
            u_p2000w_msg.msg.code=P2000W_CODE_PULSE_OUT;
            u_p2000w_msg.msg.cmd=0;
              p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,P2000W_FRAME_DELAY_TIME);
              if(p_mtxs!=osOK) 
              {
                DEBUG_PRINTF("stop pulse fail ,resend once!\r\n");
                osDelay(P2000W_FRAME_DELAY_TIME);
                osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,0);
              }
              else  DEBUG_PRINTF("p2000w stop pulse cmd\r\n");   
          }
          timeout=0;  
          do
          {
            osDelay(P2000W_FRAME_DELAY_TIME);
            timeout+=P2000W_FRAME_DELAY_TIME;
            if(timeout>P2000W_FRAME_TIMEOUT)
            {
              DEBUG_PRINTF("p2000w pulse stop timeout!\r\n");
              break;
            }          
          }while((p2000w_status.ctr_status&P2000W_STA_B0_PULSE_OUT_OK)==P2000W_STA_B0_PULSE_OUT_OK); 
          if((p2000w_status.ctr_status&P2000W_STA_B4_RELAY_OK)==P2000W_STA_B4_RELAY_OK)
          {
            u_p2000w_msg.msg.code=P2000W_CODE_RELEY_CTR;
            u_p2000w_msg.msg.cmd = 0;
            p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,P2000W_FRAME_DELAY_TIME);
            if(p_mtxs!=osOK) 
            {
              DEBUG_PRINTF("reley off fail ,resend once!\r\n");
              osDelay(P2000W_FRAME_DELAY_TIME);
              osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,0);
            }
            else DEBUG_PRINTF("p2000w close reley\r\n");    
          }
          timeout=0;  
          do
          {
            osDelay(P2000W_FRAME_DELAY_TIME);
            timeout+=P2000W_FRAME_DELAY_TIME;
            if(timeout>P2000W_RELEY_WAIT_TIMEOUT)
            {
              DEBUG_PRINTF("p2000w relay off timeout!\r\n");
              break;
            }          
          }while((p2000w_status.ctr_status&P2000W_STA_B4_RELAY_OK)==P2000W_STA_B4_RELAY_OK); 
          if((p2000w_status.ctr_status&P2000W_STA_B3_PRO_HOT_OK)==P2000W_STA_B3_PRO_HOT_OK)
          {  
            u_p2000w_msg.msg.code=P2000W_CODE_PRO_HOT_CTR;
            u_p2000w_msg.msg.cmd=0;
            p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,P2000W_FRAME_DELAY_TIME);
            if(p_mtxs!=osOK) 
            {
              DEBUG_PRINTF("exit prohot fail ,resend once!\r\n");
              osDelay(P2000W_FRAME_DELAY_TIME);
              osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,0);
            }           
            else DEBUG_PRINTF("p2000w exit prohot\r\n");
          }  
          timeout=0;  
          do
          {
            osDelay(P2000W_FRAME_DELAY_TIME);
            timeout+=P2000W_FRAME_DELAY_TIME;
            if(timeout>P2000W_PRO_HOT_WAIT_TIMEOUT)
            {
              DEBUG_PRINTF("p2000w exit pohot timeout !\r\n");
              break;
            }          
          }while((p2000w_status.ctr_status&P2000W_STA_B3_PRO_HOT_OK)==P2000W_STA_B3_PRO_HOT_OK);  
          if((p2000w_status.ctr_status&P2000W_STA_B3_PRO_HOT_OK)!=P2000W_STA_B3_PRO_HOT_OK)  
          {
            sGenSta.laser_run_B0_pro_hot_status=0;              
            DEBUG_PRINTF("prohot close ok\r\n");        
          } 
          else
          {
            DEBUG_PRINTF("prohot exit fail! power off\r\n");
           // app_high_voltage_solenoid(DISABLE); 
          }
        }                                
        if(laser_ctr_param.laserType!=0)     
        {//980 disconnnect switch to 1064
          sGenSta.laser_run_B4_laser_980_out_status=sGenSta.laser_run_B1_laser_out_status;
        }    
        p2000w_ctr_param.proHotCtr=0;       
         osTimerStop(beepHearttTimer05Handle); 
        rgbMessage = RGB_G_STANDBY;
        osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);
        osEventFlagsClear(laserEvent02Handle,EVENTS_LASER_PREPARE_OK_ALL_BITS_MASK);
        timeout=0;     
        while(osMessageQueueGet(keyJTMessageQueue01Handle,&recKeyMessage,0,0)==osOK&&timeout<5)
        {//clear key
          timeout++;          
        }
        recKeyMessage=key_jt_release;
      }      
      if(recKeyMessage==key_jt_long_press&&sGenSta.laser_run_B5_timer_status==0&&sGenSta.laser_run_B0_pro_hot_status!=0)
      {           
        if(sGenSta.laser_run_B1_laser_out_status==0)
        { 
          #if 1  //test       
          test_energe=laser_ctr_param.ledLightLevel*5;
          test_energe%=201;           
          unsigned short int   cali_freq_energe=1.0;
          //target 118 =100*（1+0.18）
           //target 100 =85*（1+0.18）
          if(laser_ctr_param.laserFreq<25)
          {
            cali_freq_energe=( unsigned short int )test_energe*(laser_ctr_param.laserFreq*0.015);
          }
          else {
           // cali_freq_energe=test_energe*(0.8+(laser_ctr_param.laserFreq-20)*0.005);
           cali_freq_energe=( unsigned short int )test_energe*(0.24-(laser_ctr_param.laserFreq)*0.004);
          }
          float cali_temprature_energe=0.0;//温度补偿
          cali_temprature_energe=0.02*(sEnvParam.eth_k1_temprature-26.0); 
          if(cali_temprature_energe>0.15) 
          {
            cali_temprature_energe=0.15; 
          }
          if(cali_temprature_energe<-0.15) 
          {
            cali_temprature_energe=-0.15; 
          }
          if(cali_temprature_energe<0)
          {
            test_energe+=(unsigned short int )fabsf(cali_temprature_energe*test_energe);  
          }  
          else test_energe-=(unsigned short int )fabsf(cali_temprature_energe*test_energe);  
          test_energe-=(cali_freq_energe);               
          test_energe%=201;  
          unsigned short int test_V = energe_140us_voltage[test_energe/5]+laser_ctr_param.timerCtr*1;                          
         // unsigned short int test_V=laser_ctr_param.ledLightLevel*1+200+laser_ctr_param.airPressureLevel*100;//energe_140us_voltage[energeNum];//200~600V;          
          u_p2000w_msg.msg.code=P2000W_CODE_VOLTAGE_SET;
          u_p2000w_msg.msg.cmd=test_V;
          p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,P2000W_FRAME_DELAY_TIME);
          if(p_mtxs!=osOK) 
          {
            DEBUG_PRINTF("set test voltage fial ,resend once!\r\n");
            osDelay(P2000W_FRAME_DELAY_TIME);
            osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,0);
          }
          else DEBUG_PRINTF("set test voltage=%dV\r\n",test_V);  
          unsigned short int pulse_width_offeset=0,pulseProUs,pulseOffDelayUs;   
          if(test_V<280)//无关触发信号
          {
            pulseOffDelayUs=10;
            pulseProUs=(unsigned short int)(34+(280-test_V)*0.40);                   
          }   
          else if(test_V<310)
          {
            pulseOffDelayUs=10;
            pulseProUs=(unsigned short int)(25+(310-test_V)*0.30);                     
          }  
          else if(test_V<370)
          {
            pulseProUs=(unsigned short int)(19+(370-test_V)*0.10); 
            pulseOffDelayUs=10;                              
          }    
          else if(test_V<470)
          {
            pulseProUs=(unsigned short int)(14.5+(470-test_V)*0.05); 
            if(test_V<410)  {
              pulseOffDelayUs=11;
            }
            else pulseOffDelayUs=10;
          }                    
          else
          {    
            pulseOffDelayUs=11;
            pulseProUs=14;                 
          }  
          pulse_width_offeset =pulseProUs+pulseOffDelayUs;  
          u_p2000w_msg.msg.code=P2000W_CODE_PULSE_WIDTH;
          //u_p2000w_tx_msg.msg.cmd=p2000w_ctr_param.pulseWidthSet;  
          u_p2000w_msg.msg.cmd=p2000w_ctr_param.pulseWidthSet-pulse_width_offeset;
          p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,P2000W_FRAME_DELAY_TIME); 
          {
            if(p_mtxs!=osOK) 
            {
              DEBUG_PRINTF("set puls width ,resend once!\r\n");
              osDelay(P2000W_FRAME_DELAY_TIME);
              osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,0); 
            }
            else DEBUG_PRINTF("set pulse =%dus\r\n",u_p2000w_msg.msg.cmd);   
          } 
          osDelay(P2000W_FRAME_TIMEOUT);
          #endif
          if((p2000w_status.ctr_status&P2000W_STA_B4_RELAY_OK)!=P2000W_STA_B4_RELAY_OK)
          {
            u_p2000w_msg.msg.code=P2000W_CODE_RELEY_CTR;
            u_p2000w_msg.msg.cmd=  1;
              osStatus_t p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,P2000W_FRAME_DELAY_TIME);
              if(p_mtxs!=osOK)   
              {
                DEBUG_PRINTF("p2000w relay on fail ,resend once!\r\n");
                osDelay(P2000W_FRAME_DELAY_TIME);
                osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,0);
              }
              else  DEBUG_PRINTF("p2000w reley on cmd\r\n"); 
          }            
          timeout=0;  
          do
          {
            osDelay(P2000W_FRAME_DELAY_TIME);
            timeout+=P2000W_FRAME_DELAY_TIME;
            if(timeout>P2000W_RELEY_WAIT_TIMEOUT)
            {
              break;
            }          
          }while((p2000w_status.ctr_status&P2000W_STA_B4_RELAY_OK)!=P2000W_STA_B4_RELAY_OK); 
          if((p2000w_status.ctr_status&P2000W_STA_B4_RELAY_OK)==P2000W_STA_B4_RELAY_OK)
          {
            u_p2000w_msg.msg.code=P2000W_CODE_PULSE_OUT;
            u_p2000w_msg.msg.cmd=  1;
              osStatus_t p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,P2000W_FRAME_DELAY_TIME);
              if(p_mtxs!=osOK)   
              {
                DEBUG_PRINTF("p2000w pulse out fail ,resend once!\r\n");
                osDelay(P2000W_FRAME_DELAY_TIME);
                osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,0);
              }
              else  DEBUG_PRINTF("p2000w start pulse cmd\r\n");
          }           
          timeout=0;  
          do
          {
            osDelay(P2000W_FRAME_DELAY_TIME);
            timeout+=P2000W_FRAME_DELAY_TIME;
            if(timeout>P2000W_FRAME_TIMEOUT)
            {
              break;
            }          
          }while((p2000w_status.ctr_status&P2000W_STA_B0_PULSE_OUT_OK)!=P2000W_STA_B0_PULSE_OUT_OK); 
          if((p2000w_status.ctr_status&P2000W_STA_B0_PULSE_OUT_OK)==P2000W_STA_B0_PULSE_OUT_OK)
          {
            if(laser_ctr_param.beep!=0)
            {
              if(osTimerIsRunning(beepHearttTimer05Handle)==pdFALSE) osTimerStart(beepHearttTimer05Handle,500);  
            }
            if(laser_ctr_param.timerEnableFlag!=0&&laser_ctr_param.timerCtr!=0) 
            {
              if(osTimerIsRunning(laserWorkTimer01Handle)==pdFALSE)
              {
                unsigned int timeCount;
                if(pLaserConfig->proCali==0||laser_ctr_param.timerCtr>180)  
                { 
                  timeCount=180*SYS_1_SECOND_TICKS; 
                }  
                else   timeCount =  laser_ctr_param.timerCtr*SYS_1_SECOND_TICKS; 
                osTimerStart(laserWorkTimer01Handle,laser_ctr_param.timerCtr*SYS_1_SECOND_TICKS);
              }  
            } 
            if(pLaserConfig->proCali==0)
            {               
              if(pLaserConfig->treatmentWaterLevel!=0||pLaserConfig->airPressureLevel!=0)
              {   
                app_deflate_air_solenoid(ENABLE);                             
                if(pLaserConfig->treatmentWaterLevel!=0) tmc2226_start(TMC_WATER_OUT_DIR_VALUE,laser_ctr_param.treatmentWaterLevel,CONTINUOUS_STEPS_COUNT);                  
              }  
            }                 
            DEBUG_PRINTF("p2000w pulse out\r\n");
            sGenSta.laser_run_B1_laser_out_status=1; 
            if(laser_ctr_param.laserType!=0)     
            {//980 disconnnect switch to 1064
              sGenSta.laser_run_B4_laser_980_out_status=sGenSta.laser_run_B1_laser_out_status;
            }   
            rgbMessage = RGB_LASER_WORK_STATUS;
            osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);
            if(u_sys_param.sys_config_param.beep!=0)
            {
              if(osTimerIsRunning(beepHearttTimer05Handle)==pdFALSE) osTimerStart(beepHearttTimer05Handle,500);//1000 slow,500 mid ;  200 fast;
            }
          } 
          else
          {
            DEBUG_PRINTF("p2000w pulse open fail\r\n");
            sGenSta.laser_run_B1_laser_out_status=0;
            if(laser_ctr_param.laserType!=0)     
            {//980 disconnnect switch to 1064
              sGenSta.laser_run_B4_laser_980_out_status=sGenSta.laser_run_B1_laser_out_status;
            }  
          }   
        }  
        else 
        {
          #if 1// energe moniter        
          if(sGenSta.laser_run_B1_laser_out_status!=0&&statusJT==osOK)
          {  
            app_get_adc_value(AD2_LASER_1064_INDEX,&e_feedback);  
          //unsigned short int vol_e = app_laser_1064_energe_to_voltage(laser_ctr_param.laserEnerge); 
         // double p_avg=(vol_e*0.00172)*u_sys_param.sys_config_param.laser_pulse_width_us*laser_ctr_param.laserFreq;
            //Ppeak=vol_e*param;
            // R(光电管响应度 ,1064nm光R≈0.795mA/mW);
            //Vr(光电管)*增益=Vpeak//光电管电压
             //Ipeak=(Ppeak*R)//光电管电流
            //e_feedback*param=Ppeak//峰值功率
            //Pavg=Ppeak*u_sys_param.sys_config_param.laser_pulse_width_us*laserFreq；//占空比
            //E=Pavg/freq;->Pavg=freq*E //脉冲能量
            //Ppeak=E/u_sys_param.sys_config_param.laser_pulse_width_us;//峰值功率 
            float peak_P = (e_feedback*0.0007);//peak  power ,峰值功率
            double p_avg=(peak_P)*(u_sys_param.sys_config_param.laser_pulse_width_us)*laser_ctr_param.laserFreq;                                   
            sEnvParam.laser_1064_energy=p_avg/laser_ctr_param.laserFreq;//能量             
            DEBUG_PRINTF("E=%.1fmJ peak_P=%.2fmw feedBck=%.1fmV pulseCount=%d rb=%d 980=%d\r\n",sEnvParam.laser_1064_energy,peak_P,e_feedback,u_sys_param.sys_config_param.laser_pulse_count,u_sys_param.sys_config_param.RDB_use_timeS,u_sys_param.sys_config_param.laser_use_timeS);              
            if(sEnvParam.laser_1064_energy>0&&laser_ctr_param.laserEnerge>0&&e_feedback>0)
            {               
              if(sEnvParam.laser_1064_energy>laser_ctr_param.laserEnerge+5)   
              {  
                energe_over_count++;
                if(energe_over_count>3){
                  energe_down_count=0;
                  energe_over_count=0;
                 if(sEnvParam.laser_1064_energy>laser_ctr_param.laserEnerge*1.20) sGenSta.laser_param_B01_energe_status=2; //over load
                  app_p2000w_pulse_auto_adjust_voltage(laser_ctr_param.laserEnerge,sEnvParam.laser_1064_energy,p2000w_ctr_param.outVoltageSet);
                }                
              } 
              else if(sEnvParam.laser_1064_energy+5<laser_ctr_param.laserEnerge)   
              {  
                energe_down_count++;
                if(energe_down_count>3){
                  energe_down_count=0;
                  energe_over_count=0;
                  if( sEnvParam.laser_1064_energy>laser_ctr_param.laserEnerge*0.80) sGenSta.laser_param_B01_energe_status=3; //low load
                  app_p2000w_pulse_auto_adjust_voltage(laser_ctr_param.laserEnerge,sEnvParam.laser_1064_energy,p2000w_ctr_param.outVoltageSet);
                }               
              }   
              else{                
                sGenSta.laser_param_B01_energe_status=1;
              } 
            }            
            else{             
              sGenSta.laser_param_B01_energe_status=0;  
            }                  
          }
          #else 
          if(sGenSta.laser_run_B1_laser_out_status!=0) 
          {
            sEnvParam.laser_1064_energy=1;
            app_get_adc_value(AD2_LASER_1064_INDEX,&e_feedback);
            //DEBUG_PRINTF("energe=%.1f\r\n",sEnvParam.laser_1064_energy);        
          }
          #endif
        }                   
        sGenSta.laser_param_B456_jt_status = recKeyMessage;
      }
      else //if(recKeyMessage!=key_jt_long_press)
      {   
        sGenSta.laser_param_B01_energe_status=1;//   
        osTimerStop(beepHearttTimer05Handle);         
        osTimerStop(laserWorkTimer01Handle); 

        if(sGenSta.laser_run_B5_timer_status!=0&&recKeyMessage!=key_jt_long_press)     sGenSta.laser_run_B5_timer_status=0;        
        if(sGenSta.laser_run_B1_laser_out_status!=0)
        {           
          if(pLaserConfig->proCali==0&&pLaserConfig->treatmentWaterLevel!=0)
          {   
            tmc2226_stop();
          }  
          if((p2000w_status.ctr_status&P2000W_STA_B0_PULSE_OUT_OK)==P2000W_STA_B0_PULSE_OUT_OK)
          {
            u_p2000w_msg.msg.code=P2000W_CODE_PULSE_OUT;
            u_p2000w_msg.msg.cmd=  0;
            osStatus_t p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,P2000W_FRAME_DELAY_TIME);
            if(p_mtxs!=osOK)   
            {
              DEBUG_PRINTF("p2000w pulse out stop fail ,resend once!\r\n");
              osDelay(P2000W_FRAME_DELAY_TIME);
              osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_msg.data,0,0);
            }
            else  DEBUG_PRINTF("p2000w stop pulse cmd\r\n");
          }
          timeout = 0;            
          do
          {
            osDelay(P2000W_FRAME_DELAY_TIME);
            timeout += P2000W_FRAME_DELAY_TIME;
            if(timeout>P2000W_RELEY_WAIT_TIMEOUT)
            {             
              break;
            }          
          }while((p2000w_status.ctr_status&P2000W_STA_B0_PULSE_OUT_OK)==P2000W_STA_B0_PULSE_OUT_OK); 
          if((p2000w_status.ctr_status&P2000W_STA_B0_PULSE_OUT_OK)!=P2000W_STA_B0_PULSE_OUT_OK)
          {
            sGenSta.laser_run_B1_laser_out_status=0;
            if(laser_ctr_param.laserType!=0)     
            {//980 disconnnect switch to 1064              
              sGenSta.laser_run_B4_laser_980_out_status=sGenSta.laser_run_B1_laser_out_status;
            }     
            rgbMessage = RGB_LASER_PREPARE_OK;
            osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);
            DEBUG_PRINTF("stop 1064 \r\n");  
          } 
          app_deflate_air_solenoid(DISABLE);            
          sGenSta.laser_param_B456_jt_status = recKeyMessage;
        }    
      }  
    }     
    else if(event==(EVENTS_LASER_980_PREPARE_OK_BIT|EVENTS_LASER_JT_ENABLE_BIT))
    {
      if(laser_close_sem==osOK&&sGenSta.laser_run_B0_pro_hot_status!=0)
      { 
        sGenSta.laser_run_B5_timer_status=0;
        osTimerStop(laserWorkTimer01Handle);                	        
        tmc2226_stop();  
        app_deflate_air_solenoid(DISABLE);
        if((u_s_l980.sta.staByte&L980_STA_HEART_BIT0)!=L980_STA_HEART_BIT0)
        {                          
          DEBUG_PRINTF("l980 disconnect!,please power off");         
          memset(&u_s_l980.sta,0,sizeof(L980_STATUS));//clear
        }
        else
        {
          if((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)==L980_STA_PULSEOUT_BIT2) 
          {
            u_CAN_tx_t_msg.msg.typeCode=L980_REG_JT_CTR_STOP|L980_REG_WRITE_MASK;
            u_CAN_tx_t_msg.msg.dataLen=2;
            u_CAN_tx_t_msg.msg.buff[0]=0;
            u_CAN_tx_t_msg.msg.buff[1]=0;            
            can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_t_msg.data,0,0);
            if(can_tx_sta!=osOK)   
            {
              DEBUG_PRINTF("L980 stop pulse out fail ,resend once!\r\n");
              osDelay(L980_CAN_MINI_TIME_MS);
              osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_t_msg.data,0,0);
            }
          }          
          timeout=0; 
          do
          {
            osDelay(L980_CAN_MINI_TIME_MS);
            timeout+=L980_CAN_MINI_TIME_MS;
            if(timeout>L980_CAN_FRAME_TIMEOUT)
            { //l980 power off 
              break;
            }          
          }while((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)==L980_STA_PULSEOUT_BIT2); 
          if((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)!=L980_STA_PULSEOUT_BIT2) 
          {  
            rgbMessage = RGB_LASER_PREPARE_OK;
            osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);  
            sGenSta.laser_run_B4_laser_980_out_status=0;  
          }
          if((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)==L980_STA_PROHOT_BIT1)           
          {
            u_CAN_tx_t_msg.msg.typeCode=L980_REG_CTR_PRO_HOT|L980_REG_WRITE_MASK;
            u_CAN_tx_t_msg.msg.dataLen=4;
            u_CAN_tx_t_msg.msg.buff[0]=0;
            u_CAN_tx_t_msg.msg.buff[1]=0; 
            u_CAN_tx_t_msg.msg.buff[2]=0;
            u_CAN_tx_t_msg.msg.buff[3]=0;            
            can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_t_msg.data,0,0);
            if(can_tx_sta!=osOK)   
            {
              DEBUG_PRINTF("L980 exit prohot cmd send fail ,resend once!\r\n");
              osDelay(L980_CAN_MINI_TIME_MS);
              osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_t_msg.data,0,0);
            }         
          }
          timeout=0; 
          do
          {
            osDelay(L980_CAN_MINI_TIME_MS);
            timeout+=L980_CAN_MINI_TIME_MS;
            if(timeout>L980_MAX_PROHOT_WAIT_TIME)
            { //l980 power off               
              break;
            }          
          }while((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)==L980_STA_PROHOT_BIT1);  
          if((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)!=L980_STA_PROHOT_BIT1)   
          {   
            DEBUG_PRINTF("l980 exit prohot succcess \r\n");  
          }          
          else
          {
            DEBUG_PRINTF("l980 exit porhot timeout!power off\r\n"); 
            //power off
            #if 0 
            m_stat=osMutexAcquire(canBusMutex02Handle,L980_CAN_FRAME_TIMEOUT);
            if(m_stat==osOK)
            {  
              osSemaphoreAcquire(hmiCanBusIdleSem06Handle,HMI_CAN_FRAME_DELAY_TIME);
              l980_cmdBuff[0]=1;
              l980_cmdBuff[1]=0;            
              L980_appWriteReg(L980_REG_SYS_POWER_OFF,2,l980_cmdBuff);              
              osMutexRelease(canBusMutex02Handle);               
            } 
            #endif 
          }
        }  
        sGenSta.laser_run_B0_pro_hot_status=0; 
        sGenSta.laser_run_B4_laser_980_out_status=0;       
        rgbMessage = RGB_G_STANDBY;
        osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);     
        osTimerStop(beepHearttTimer05Handle);      
        osEventFlagsClear(laserEvent02Handle,EVENTS_LASER_PREPARE_OK_ALL_BITS_MASK);
        timeout=0;
        while(osMessageQueueGet(keyJTMessageQueue01Handle,&recKeyMessage,0,0)==osOK&&timeout<5)
        {//clear key
          timeout++;
        } 
        recKeyMessage=key_jt_release;        
      }                      
      if(recKeyMessage==key_jt_long_press&&sGenSta.laser_run_B5_timer_status==0&&sGenSta.laser_run_B0_pro_hot_status!=0)
      {          
        if(sGenSta.laser_run_B4_laser_980_out_status==0)        
        {  
          if((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)!=L980_STA_PULSEOUT_BIT2) 
          {
            u_CAN_tx_t_msg.msg.typeCode=L980_REG_JT_CTR_STOP|L980_REG_WRITE_MASK;
            u_CAN_tx_t_msg.msg.dataLen=2;
            u_CAN_tx_t_msg.msg.buff[0]=1;
            u_CAN_tx_t_msg.msg.buff[1]=0;            
            can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_t_msg.data,0,0);
            if(can_tx_sta!=osOK)   
            {
              DEBUG_PRINTF("L980 start pulse out fail ,resend once!\r\n");
              osDelay(L980_CAN_MINI_TIME_MS);
              osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_t_msg.data,0,0);
            }                         
          }    
          timeout=0;  
          do{
            osDelay(L980_CAN_MINI_TIME_MS);
            timeout+=L980_CAN_MINI_TIME_MS;
            if(timeout>L980_MAX_PROHOT_WAIT_TIME)
            {
              DEBUG_PRINTF("l980 pulse out  fail,exit prohot\r\n"); 
              break;
            }          
          } while((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)!=L980_STA_PULSEOUT_BIT2);
          if((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)==L980_STA_PULSEOUT_BIT2)
          {            
            if(laser_ctr_param.timerEnableFlag!=0&&laser_ctr_param.timerCtr!=0) 
            {
              if(osTimerIsRunning(laserWorkTimer01Handle)==pdFALSE)
              {
                unsigned int timeCount;
                if(pLaserConfig->proCali==0||laser_ctr_param.timerCtr>180)  
                { 
                  timeCount=180*SYS_1_SECOND_TICKS; 
                }  
                else   timeCount =  laser_ctr_param.timerCtr*SYS_1_SECOND_TICKS; 
                osTimerStart(laserWorkTimer01Handle,laser_ctr_param.timerCtr*SYS_1_SECOND_TICKS);
              }  
            } 
            if(pLaserConfig->proCali==0)
            {               
              if(pLaserConfig->treatmentWaterLevel!=0||pLaserConfig->airPressureLevel!=0)
              {   
                app_deflate_air_solenoid(ENABLE);             
                if(pLaserConfig->treatmentWaterLevel!=0) tmc2226_start(TMC_WATER_OUT_DIR_VALUE,laser_ctr_param.treatmentWaterLevel,CONTINUOUS_STEPS_COUNT);                  
              }  
            } 
            sGenSta.laser_run_B4_laser_980_out_status=1;
            rgbMessage = RGB_LASER_WORK_STATUS;
            osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);
            DEBUG_PRINTF("l980 pulse out ok\r\n");
            if(laser_ctr_param.beep!=0)
            {
              if(osTimerIsRunning(beepHearttTimer05Handle)==pdFALSE) osTimerStart(beepHearttTimer05Handle,500); //1000 slow,500 mid ;  200 fast; 
            }           
          }  
          else
          {
            DEBUG_PRINTF("l980 pulse out fail!\r\n");
            sGenSta.laser_run_B4_laser_980_out_status=0;
            // power off
          }                 
        }  
        else 
        {         
          #if 1// energe moniter        
          if(sGenSta.laser_run_B4_laser_980_out_status!=0&&statusJT==osOK) // cali         
          {     
            sEnvParam.laser_1064_energy=(u_s_l980.sta.energeFeedback)*0.004;
            DEBUG_PRINTF("e_p=%.1fw feedBck=%dmV  rdb=%d 980=%d\r\n",sEnvParam.laser_1064_energy,u_s_l980.sta.energeFeedback,u_sys_param.sys_config_param.RDB_use_timeS,u_sys_param.sys_config_param.laser_use_timeS);              
            //l980 can send         
            if(sEnvParam.laser_1064_energy>0&&laser_ctr_param.laserEnerge>0)
            {   
              if(sEnvParam.laser_1064_energy>laser_ctr_param.laserEnerge*1.30)   
              {
                sGenSta.laser_param_B01_energe_status=1;//2; //over load
              }      
              else sGenSta.laser_param_B01_energe_status=1;
            }
            else 
            {
              sGenSta.laser_param_B01_energe_status = 0;//err
            }              
          }
          #else 
          if(sGenSta.laser_run_B4_laser_980_out_status!=0) 
          {
            sEnvParam.laser_1064_energy=1;                      
          }
          #endif
        }                   
      }
      else //if(recKeyMessage!=key_jt_long_press)
      {
        sGenSta.laser_param_B01_energe_status = 1;
        osTimerStop(beepHearttTimer05Handle);         
        osTimerStop(laserWorkTimer01Handle); 
        if(sGenSta.laser_run_B5_timer_status!=0&&recKeyMessage!=key_jt_long_press)     sGenSta.laser_run_B5_timer_status=0;          
        if(sGenSta.laser_run_B4_laser_980_out_status!=0)
        {    
          if(pLaserConfig->proCali==0&&pLaserConfig->treatmentWaterLevel!=0)
          {   
            tmc2226_stop();
          }        
          if((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)==L980_STA_PULSEOUT_BIT2)
          {
            u_CAN_tx_t_msg.msg.typeCode=L980_REG_JT_CTR_STOP|L980_REG_WRITE_MASK;
            u_CAN_tx_t_msg.msg.dataLen=2;
            u_CAN_tx_t_msg.msg.buff[0]=0;
            u_CAN_tx_t_msg.msg.buff[1]=0;            
            can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_t_msg.data,0,0);
            if(can_tx_sta!=osOK)   
            {
              DEBUG_PRINTF("L980 stop pulse out fail ,resend once!\r\n");
              osDelay(L980_CAN_MINI_TIME_MS);
              osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_t_msg.data,0,0);
            }                       
          }
          timeout=0;  
          do{
            osDelay(L980_CAN_MINI_TIME_MS);
            timeout+=L980_CAN_MINI_TIME_MS;
            if(timeout>L980_CAN_FRAME_TIMEOUT)
            {
              break;
            }                      
          }while((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)==L980_STA_PULSEOUT_BIT2);
          if((u_s_l980.sta.staByte&L980_STA_PULSEOUT_BIT2)!=L980_STA_PULSEOUT_BIT2)
          {
            DEBUG_PRINTF("l980 stop pulse out ok\r\n"); 
            sGenSta.laser_run_B4_laser_980_out_status=0;
            rgbMessage = RGB_LASER_PREPARE_OK;
            osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);              
          }             
          app_deflate_air_solenoid(DISABLE);   
          sGenSta.laser_param_B456_jt_status = recKeyMessage;           
        }                      
      }  
    }
    else
    {
      if(recKeyMessage==key_jt_release) sGenSta.laser_param_B456_jt_status=key_jt_release;      
    }   
    osDelay(5);      
  }
  /* USER CODE END laserWorkTask04 */
}

/* USER CODE BEGIN Header_fastAuxTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fastAuxTask05 */
void fastAuxTask05(void *argument)
{
  /* USER CODE BEGIN fastAuxTask05 */
  /* Infinite loop */    
  app_mcp61_init(); 
  for(;;)
  {
		/***********环境气压、温度监测*******************/ 	 
    unsigned char  status=app_gzp6816d_listen(osKernelGetTickCount(),&sEnvParam.air_gzp_enviroment_pressure_kpa,&sEnvParam.enviroment_temprature);    
    if(status!=0) sEnvParam.air_gzp_enviroment_pressure_kpa =95.0;//error   
    /***********NTC,laser_energe,iBus,Vbus,vBus,air_pump_pressure气泵气压，参数*******************/     
    app_get_adc_value(AD1_NTC_INDEX,&sEnvParam.NTC_temprature);//蠕动泵，状态   
    app_get_adc_value(AD1_OCP_Ibus_INDEX,&sEnvParam.iBus);		
    app_get_adc_value(AD1_24V_VBUS_INDEX,&sEnvParam.vBus);     
    /***********气泵管理*******************/
		app_get_adc_value(AD1_AIR_PRESSER_INDEX,&sEnvParam.air_pump_pressure);
		app_air_pump_manage(laser_ctr_param.airPressureLevel);  
		/***********aux genaration状态检查*******************/       
		app_fresh_laser_status_param(); 
    /***********循环水液位检查*******************/ 
    app_get_adc_value(AD1_WATER_PRESSER_INDEX,&sEnvParam.treatment_water_pressure);	     
    app_mcp61_get_singgle_c_value_req();    
    osDelay(5);
    app_mcp61_package_check();
    sEnvParam.cool_water_depth= app_mcp61_c_value()*10;//real depth =0.1pf*10
    
    osStatus_t cool_sta=osSemaphoreAcquire(coolWaterDepthBinarySem01Handle,5);
    if(cool_sta==osOK)
    {
      /***********更新报警阈值*******************/
      if(u_sys_param.sys_config_param.cool_water_depth_low!=u_sys_default_param.sys_config_param.cool_water_depth_low)
      {
        u_sys_default_param.sys_config_param.cool_water_depth_low=u_sys_param.sys_config_param.cool_water_depth_low;
        unsigned short int   low_value= (unsigned short int)(u_sys_param.sys_config_param.cool_water_depth_low*100);
        app_mcp61_set_alart_value_low(low_value);//(<low_value,flag==0)
        osDelay(5);
        app_mcp61_package_check();        
        unsigned short int high_value=low_value+(low_value>>3);//(>low_value*112.5%,flag==1)
        app_mcp61_set_alart_value_high(high_value);
        osDelay(5);
        app_mcp61_package_check();
      }
      if(u_sys_param.sys_config_param.cool_water_depth_high!=u_sys_default_param.sys_config_param.cool_water_depth_high)
      {//overflow
        u_sys_default_param.sys_config_param.cool_water_depth_high=u_sys_param.sys_config_param.cool_water_depth_high;       
      }
    }  
  }
  /* USER CODE END fastAuxTask05 */
}

/* USER CODE BEGIN Header_hmiAppTask06 */

/**
* @brief Function implementing the hmiTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hmiAppTask06 */
void hmiAppTask06(void *argument)
{
  /* USER CODE BEGIN hmiAppTask06 */
  /* Infinite loop */ 
  //osDelay(13000);
  uint16_t send_music_num = MUSIC_SYS_ON;
  osMessageQueuePut(musicQueue03Handle,&send_music_num,0,100);
  u_sys_param.sys_config_param.synchronousFlag=0; 
  uint32_t syncTimeOutS=0;
  unsigned char can_tx_Message[8],hmi_can_idle_flag=0;
  osStatus_t m_status;
  U_CAN_TX_MSG u_c_t_msg;   
  for(;;)
  {   
    while(u_sys_param.sys_config_param.synchronousFlag!=3)   
    {
      if(HAL_GetTick()>20000)
      {
        syncTimeOutS+=HMI_CAN_FRAME_DELAY_TIME;
				if((syncTimeOutS%1000)==0)
				{
					DEBUG_PRINTF("synchronous req=%d\r\n",syncTimeOutS);
					app_hmi_sysnc_req();
				}        
      }  
      if(syncTimeOutS>5000)
      {
        DEBUG_PRINTF("sync fail use local paramete\r\n");
        u_sys_param.sys_config_param.synchronousFlag=3;
      }     
			osSemaphoreAcquire(hmiCanBusIdleSem06Handle,HMI_CAN_FRAME_DELAY_TIME); 			
    }    
    //:HMI_CAN_FRAME_DELAY_TIME=2*P2000W_FRAME_DELAY_TIME    
    osStatus_t s_status = osSemaphoreAcquire(hmiCanBusIdleSem06Handle,portMAX_DELAY);//2*HMI_CAN_FRAME_DELAY_TIME);   
    if(s_status==osOK)
    {
      osStatus_t p_m = osMessageQueueGet(canTxQueue05Handle,can_tx_Message,0,0);      
      if(p_m==osOK)
      { 
        U_CAN_TX_MSG* p_can_tx_msg=(U_CAN_TX_MSG*)can_tx_Message;
        #ifdef L980_USED 
        hmi_can_idle_flag=0;//idle
        if(p_can_tx_msg->msg.typeCode==0)
        {//HMI send package to L980,can_tx_Message[0] is reg addr,can_tx_Message[1] is data len
          osDelay(L980_CAN_MINI_TIME_MS);
          L980_appReadReq(L980_REG_HEART_STATUS,sizeof(L980_STATUS) );
        }        
        else
        {
          if((p_can_tx_msg->msg.typeCode&L980_REG_WRITE_MASK))  L980_appWriteReg(p_can_tx_msg->msg.typeCode&L980_REG_MASK,p_can_tx_msg->msg.dataLen,p_can_tx_msg->msg.buff);
          else L980_appReadReq(p_can_tx_msg->msg.typeCode&L980_REG_MASK,p_can_tx_msg->msg.dataLen);          
        } 
        #else 
        hmi_can_idle_flag=1;//busy
        #endif
      }
      else 
      {
				app_sys_genaration_status_manage();
        sGenSta.genaration_io_status = osEventFlagsGet(auxStatusEvent01Handle);
        if(hmi_can_idle_flag==0)
        {  
          app_hmi_report_status(&sGenSta);
          #ifdef L980_USED            
          hmi_can_idle_flag=1;  
          #endif       
        }
        else 
        {
          hmi_can_idle_flag=0;
          #ifdef L980_USED 
          L980_appReadReq(L980_REG_HEART_STATUS,sizeof(L980_STATUS));//4);
          #endif
        }
      }      
    } 
    #ifdef L980_USED  
    short int  l980_timer;
    if(laser_ctr_param.timerEnableFlag!=0)
    {
      l980_timer=(short int )(laser_ctr_param.timerCtr);
    }
    else 
    {
      l980_timer=(-(short int )(laser_ctr_param.timerCtr));
    }
    
    /*****************激光指示灯***********************/	
    if(l980_timer!=u_l980.set_param.timerSet)
    {     
      u_l980.set_param.timerSet=l980_timer;
      u_c_t_msg.msg.typeCode=L980_REG_COUNTDOWN_TIMERS|L980_REG_WRITE_MASK;
      u_c_t_msg.msg.dataLen=2;
      u_c_t_msg.msg.buff[0]=u_l980.set_param.timerSet&0xFF;
      u_c_t_msg.msg.buff[1]=(u_l980.set_param.timerSet>>8)&0xFF;
      osStatus_t can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_c_t_msg.data,0,0);
      if(can_tx_sta!=osOK)   
      {
        DEBUG_PRINTF("l980 timerSet update fail ,resend once!\r\n");
        osDelay(L980_CAN_MINI_TIME_MS);
        osMessageQueuePut(canTxQueue05Handle,u_c_t_msg.data,0,0);
      }
      else  DEBUG_PRINTF("l980 timerSet update \r\n");
      #endif
    }
		osDelay(1);
  }
  /* USER CODE END hmiAppTask06 */
}

/* USER CODE BEGIN Header_canReceiveTask07 */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canReceiveTask07 */
void canReceiveTask07(void *argument)
{
  /* USER CODE BEGIN canReceiveTask07 */
  /* Infinite loop */
  uint16_t peekLen=0;
  uint8_t buff[8];
  uint32_t Identifier;
  uint16_t len;
  unsigned char fd_canRxBuff[MAX_FDCAN_FRAME_DATALEN+1];//hmi
  unsigned short int  fd_canRxLen = 0;//hmi (initialized to avoid undefined behavior)
  unsigned  int l980_heart_timeout = 0,can_rec_timeout=0;
  osStatus_t m_status,can_tx_sta;
  U_CAN_TX_MSG u_CA_t_msg;
  for(;;)
  {
    /*******************CAN RX-DATA********************/
    can_rec_timeout=osKernelGetTickCount();	    
    while(FDCAN1_Receive_Msg(buff, &Identifier, &len))
    {  
      if(Identifier==CAN_RTU_MASTER_ID||(Identifier==CAN_RTU_MASTER_ID+1))//L980			
      {
				#if 0       
          DEBUG_PRINTF("CAN_receive_pack:\r\n");
          for(int i=0;i<8;i++)
          {
            DEBUG_PRINTF(" %02x",buff[i]);
          }
          DEBUG_PRINTF(" Len=%d\r\n",len); 	
				#endif		
        uint8_t rtuPackageType = (Identifier&0x01);  
        #ifdef L980_USED          
        CAN_receivePackageHandle(buff,rtuPackageType); 
        #endif
        l980_heart_timeout=0;         
      }    
			else //if(Identifier==CAN_BROADCAST_ID)//屏幕
      {
        memcpy(&fd_canRxBuff[fd_canRxLen],buff,8);
        fd_canRxLen+=len;
        fd_canRxLen%=(MAX_FDCAN_FRAME_DATALEN+1);     
        if(fd_canRxLen>=8) 
        {       
          #if 0       
          DEBUG_PRINTF("CAN_lcd_receive_pack:\r\n");
          for(int i=0;i<peekLen;i++)
          {
            DEBUG_PRINTF(" %02x",fd_canRxBuff[i+readLen]);
          }
          DEBUG_PRINTF(" Len=%d\r\n",peekLen);      
          #endif   
          peekLen = app_hmi_package_check(fd_canRxBuff,fd_canRxLen);
          if(peekLen!=0)
          {          
            if(peekLen<fd_canRxLen)
            {  
              uint16_t packLen =  fd_canRxLen-peekLen; 
              memcpy(fd_canRxBuff,&fd_canRxBuff[peekLen],packLen); 
              fd_canRxLen-=peekLen;           
            }
            else  {//buff full!
              fd_canRxLen=0;            
            }                       
          }  
        }
        /*****************激光指示灯***********************/	
        if(laser_ctr_param.ledLightLevel!=0) 
        {
          if(sGenSta.laser_run_B3_laser_pilot_lamp_status==0)
          {  
            sGenSta.laser_run_B3_laser_pilot_lamp_status=1;	
          }         
          if(u_sys_param.sys_config_param.laser_led_light!=laser_ctr_param.ledLightLevel)
          {
            u_sys_param.sys_config_param.laser_led_light =laser_ctr_param.ledLightLevel;
            u_l980.set_param.auxLedBulbDutySet=laser_ctr_param.ledLightLevel;  //fresh
            u_CA_t_msg.msg.typeCode=L980_REG_AUXILIARY_BULB|L980_REG_WRITE_MASK;
            u_CA_t_msg.msg.dataLen=2;
            u_CA_t_msg.msg.buff[0]=u_l980.set_param.auxLedBulbDutySet;
            u_CA_t_msg.msg.buff[1]=10; //固定10k           
            can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_CA_t_msg.data,0,0);
            if(can_tx_sta!=osOK)   
            {
              DEBUG_PRINTF("L980 set auxled fail ,resend once!\r\n");
              osDelay(L980_CAN_MINI_TIME_MS);
              osMessageQueuePut(canTxQueue05Handle,u_CA_t_msg.data,0,0);
            }
            else DEBUG_PRINTF("L980  set auxled duty=%d\r\n",u_l980.set_param.auxLedBulbDutySet);              
            app_auxiliary_bulb_pwm(laser_ctr_param.ledLightLevel,ENABLE); 
          }          
        }
        else
        {
          if(sGenSta.laser_run_B3_laser_pilot_lamp_status!=0)
          {
            u_sys_param.sys_config_param.laser_led_light=laser_ctr_param.ledLightLevel;
            u_l980.set_param.auxLedBulbDutySet=laser_ctr_param.ledLightLevel;  //fresh
            u_CA_t_msg.msg.typeCode=L980_REG_AUXILIARY_BULB|L980_REG_WRITE_MASK;
            u_CA_t_msg.msg.dataLen=2;
            u_CA_t_msg.msg.buff[0]=0;
            u_CA_t_msg.msg.buff[1]=0;            
            can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_CA_t_msg.data,0,0);
            if(can_tx_sta!=osOK)   
            {
              DEBUG_PRINTF("L980 off auxled fail ,resend once!\r\n");
              osDelay(L980_CAN_MINI_TIME_MS);
              osMessageQueuePut(canTxQueue05Handle,u_CA_t_msg.data,0,0);
            }
            else DEBUG_PRINTF("L980 off auxled \r\n"); 
            app_auxiliary_bulb_pwm(0,DISABLE);           
            sGenSta.laser_run_B3_laser_pilot_lamp_status=0;
          }	 
        }	        
      }
      if(osKernelGetTickCount()>can_rec_timeout+HMI_CAN_FRAME_DELAY_TIME) break; 
    }     
		l980_heart_timeout+=5;		
    if(l980_heart_timeout>L980_CAN_FRAME_TIMEOUT)
    {//l980 heart timeout   
      l980_heart_timeout= 0;      
      if((u_s_l980.sta.staByte&L980_STA_HEART_BIT0)==L980_STA_HEART_BIT0) 
      {
        DEBUG_PRINTF("l980 disconnect !\r\n");
        u_s_l980.sta.staByte&=(~L980_STA_HEART_BIT0);
        if(laser_ctr_param.laserType!=0)//980
        {//l980,exit prohot semo  
          if(sGenSta.laser_run_B0_pro_hot_status!=0&&laser_ctr_param.proHotCtr!=0&&laser_ctr_param.laserType!=0)
          {
            DEBUG_PRINTF("l980 disconnect exitpro !\r\n"); 
            laser_ctr_param.proHotCtr = 0;
            osSemaphoreRelease(laserCloseSem05Handle);
          }  
        }         
      }
    }     
   	osDelay(5);   
  }
  /* USER CODE END canReceiveTask07 */
}

/* USER CODE BEGIN Header_powerOffTask08 */
/**
* @brief Function implementing the myTask08 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_powerOffTask08 */
void powerOffTask08(void *argument)
{
  /* USER CODE BEGIN powerOffTask08 */
  /* Infinite loop */
  for(;;)
  {
    osStatus_t status = osSemaphoreAcquire(powerOffBinarySem02Handle,portMAX_DELAY);
    sGenSta.genaration_io_status = osEventFlagsGet(auxStatusEvent01Handle);	
    sGenSta.laser_run_B6_close_device_status=1;    
    uint8_t cmdL[4];
    cmdL[0]=1;      
    cmdL[1]=0;    
    L980_appWriteReg(L980_REG_SYS_POWER_OFF,1,cmdL);  
    app_hmi_report_status(&sGenSta);       
		if(app_sys_param_save_data()==0)
		{
			DEBUG_PRINTF("sys param save ok\r\n");
		}	      
    app_beep_pwm(music_tab_c[1],50);
    HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
    HAL_Delay(500);     
    app_beep_pwm(0,0);
    HAL_Delay(1000);
		app_mcu_power_switch(DISABLE);   
		DEBUG_PRINTF("DEVICE POWER OFF\r\n"); 
		while(1)
		{					
			DEBUG_PRINTF("please release power key\r\n");
			HAL_Delay(1000);
      #ifdef IWDG_USED
      HAL_IWDG_Refresh(&hiwdg1); 
      #endif      	
		}	
   //osDelay(1);
  }
  /* USER CODE END powerOffTask08 */
}

/* USER CODE BEGIN Header_laserProhotTask09 */
/**
* @brief Function implementing the myTask09 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_laserProhotTask09 */
void laserProhotTask09(void *argument)
{
  /* USER CODE BEGIN laserProhotTask09 */
  /* Infinite loop */
  unsigned int  timeout;
	unsigned  short int rgbMessage=1;
  unsigned  short int laser_freq,laser_Voltage;	  		
  float outVoltage; 
  unsigned char  local_proHotCtr=0,local_lasertype;	
  unsigned char  cmdBuff[4]={0};
  osStatus_t m_stat;
  osStatus_t p_mtxs;
  osStatus_t can_tx_sta;
  U_CAN_TX_MSG u_CAN_tx_msg;
  U_P2000W_TX_MSG u_p2000w_tx_msg;
  for(;;) 
  {
		osStatus_t sta = osSemaphoreAcquire(laserPrapareReqSem03Handle,portMAX_DELAY);
    if(HAL_GPIO_ReadPin(EMERGENCY_LASER_STOP_STATUS_in_GPIO_Port,EMERGENCY_LASER_STOP_STATUS_in_Pin)==GPIO_PIN_RESET)
    {
      local_proHotCtr =	laser_ctr_param.proHotCtr;
    }
    else
    {
      local_proHotCtr =	0;
      laser_ctr_param.proHotCtr=0;
      DEBUG_PRINTF("emergency key is press !!\r\n");
    } 
		laser_freq = laser_ctr_param.laserFreq;
    local_lasertype = laser_ctr_param.laserType;
    timeout=0;
    do
    {
      osDelay(50); 
      timeout+=50;      
    } while(u_sys_param.sys_config_param.synchronousFlag!=3&&timeout<12000); 
    //check 980 connection status
    if((u_s_l980.sta.staByte&L980_STA_HEART_BIT0)!=L980_STA_HEART_BIT0&&local_lasertype!=0) 
    {// 980  disconnnect  change to 1064
      laser_ctr_param.laserFreq=60;
      laser_freq=laser_ctr_param.laserFreq;     
      local_lasertype=0;
    }   
    if(local_proHotCtr!=0)	
    {	 
      if(local_lasertype==1)//980
      {         
        if((u_s_l980.sta.staByte&L980_STA_HEART_BIT0)==L980_STA_HEART_BIT0) 
        {  //positon  
          if((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)!=L980_STA_PROHOT_BIT1)   
          { 
            u_l980.set_param.positionSet=33000;     
            u_CAN_tx_msg.msg.typeCode=L980_REG_MOTOR_POSITION|L980_REG_WRITE_MASK;
            u_CAN_tx_msg.msg.dataLen=4;
            u_CAN_tx_msg.msg.buff[0]=u_l980.set_param.positionSet&0xFF;
            u_CAN_tx_msg.msg.buff[1]=(u_l980.set_param.positionSet>>8)&0xFF;
            u_CAN_tx_msg.msg.buff[2]=u_l980.set_param.positionSet&0xFF;
            u_CAN_tx_msg.msg.buff[3]=(u_l980.set_param.positionSet>>8)&0xFF;         
            can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_msg.data,0,0);
            if(can_tx_sta!=osOK)   
            {
              DEBUG_PRINTF("L980 set position fail ,resend once!\r\n");
              osDelay(L980_CAN_MINI_TIME_MS);
              osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_msg.data,0,0);
            }
            else  DEBUG_PRINTF("980 set position=%dμm",u_l980.set_param.positionSet);           
            unsigned short int dacValue = (app_laser_980_energe_to_voltage(laser_ctr_param.laserEnerge));              
            if(laser_ctr_param.ctrTestMode)  
            {              
              dacValue = 750;//laserEnerge=100mJ p=6W              
            }             
            else dacValue = laser_ctr_param.laserEnerge*6+300;
            u_CAN_tx_msg.msg.typeCode=L980_REG_CTR_PRO_HOT|L980_REG_WRITE_MASK;
            u_CAN_tx_msg.msg.dataLen=4;
            u_CAN_tx_msg.msg.buff[0]=laser_ctr_param.laserEnerge&0xFF;
            u_CAN_tx_msg.msg.buff[1]=(laser_ctr_param.laserEnerge>>8)&0xFF;
            u_CAN_tx_msg.msg.buff[2]=dacValue&0xFF;
            u_CAN_tx_msg.msg.buff[3]=(dacValue>>8)&0xFF;         
            can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_msg.data,0,0);
            if(can_tx_sta!=osOK)   
            {
              DEBUG_PRINTF("set 980 energe fail ,resend once!\r\n");
              osDelay(L980_CAN_MINI_TIME_MS);
              osMessageQueuePut(canTxQueue05Handle,u_CAN_tx_msg.data,0,0);
            }
            else  DEBUG_PRINTF("set 980 energe=%d dac=%d \r\n",laser_ctr_param.laserEnerge,dacValue);
          }     
          timeout = 0;
          do
          {
            osDelay(L980_CAN_MINI_TIME_MS);  
            timeout += L980_CAN_MINI_TIME_MS; 
            if(timeout>L980_MAX_PROHOT_WAIT_TIME)    
            {              
              DEBUG_PRINTF("l980 prohot timeout \r\n"); 
              break;
            }   
          }while((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)!=L980_STA_PROHOT_BIT1);
          if((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)==L980_STA_PROHOT_BIT1)
          {
            rgbMessage = RGB_LASER_PREPARE_OK;              
            osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);   
            osEventFlagsSet(laserEvent02Handle,EVENTS_LASER_980_PREPARE_OK_BIT|EVENTS_LASER_JT_ENABLE_BIT);   
            DEBUG_PRINTF("l980 prohot success !\r\n");
            sGenSta.laser_run_B0_pro_hot_status = 1;
          } 
          else
          {
            sGenSta.laser_run_B0_pro_hot_status=0;
            local_proHotCtr = 0;
            laser_ctr_param.proHotCtr = 0;
            DEBUG_PRINTF("l980 prohot fail !\r\n");
          }  
        } 
        else   
        { 
          DEBUG_PRINTF("laser 980 is  disconnect! prohot fail\r\n");  
          sGenSta.laser_run_B0_pro_hot_status = 0;   
          laser_ctr_param.proHotCtr = 0;
          local_proHotCtr = 0; 
        }       
      }
      else //1064
      {              
        p2000w_ctr_param.proHotCtr=local_proHotCtr;  
        if(p2000w_ctr_param.p2000wHeart==0)
        {
          DEBUG_PRINTF("p2000W disconnect ! \r\n");            
         // app_high_voltage_solenoid(DISABLE);
          DEBUG_PRINTF("restart p2000w ! \r\n"); 
          osDelay(P2000W_FRAME_TIMEOUT); 
          app_high_voltage_solenoid(ENABLE);
          osDelay(P2000W_FRAME_TIMEOUT);
          //re connect
          u_p2000w_tx_msg.msg.code=P2000W_CODE_RECONNECT;
          u_p2000w_tx_msg.msg.cmd=1; 
          p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,P2000W_FRAME_DELAY_TIME); 
          {
            if(p_mtxs!=osOK) 
            {
              DEBUG_PRINTF("set puls width ,resend once!\r\n");
              osDelay(P2000W_FRAME_DELAY_TIME);
              osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0); 
            }
          } 
        }       
        timeout = 0;
        do
        {
          osDelay(P2000W_FRAME_DELAY_TIME);  
          timeout += P2000W_FRAME_DELAY_TIME;
          if(timeout>P2000W_PRO_HOT_WAIT_TIMEOUT)    
          {           
            break;
          }   
        }while((p2000w_status.ctr_status&P2000W_STA_B1_PFC_OK)!=P2000W_STA_B1_PFC_OK );
        unsigned char  local_p2000w_err=p2000w_status.error_code; 
        if((local_p2000w_err&P2000W_ERROR_CODE_MASK)==0&&p2000w_ctr_param.p2000wHeart!=0&&(p2000w_status.ctr_status&P2000W_STA_B1_PFC_OK)==P2000W_STA_B1_PFC_OK )         
        {     
          p2000w_ctr_param.outVoltageSet =energe_140us_voltage[laser_ctr_param.laserEnerge/5]; //app_laser_1064_energe_to_voltage(laser_ctr_param.laserEnerge);  
          p2000w_ctr_param.freqSet = laser_ctr_param.laserFreq; 
          p2000w_ctr_param.pulseWidthSet = u_sys_param.sys_config_param.laser_pulse_width_us; 
          //保护脉宽
          u_p2000w_tx_msg.msg.code=P2000W_CODE_MAX_VOL_WIDTH;
          if(p2000w_ctr_param.outVoltageSet<280)   u_p2000w_tx_msg.msg.cmd=600; 
          if(p2000w_ctr_param.outVoltageSet<400)   u_p2000w_tx_msg.msg.cmd=500; 
          else u_p2000w_tx_msg.msg.cmd=400;
          p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,P2000W_FRAME_DELAY_TIME); 
          {
            if(p_mtxs!=osOK) 
            {
              DEBUG_PRINTF("set max pulse width ,resend once!\r\n");
              osDelay(P2000W_FRAME_DELAY_TIME);
              osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0); 
            }
          } 
          #if 0          
          //test                              
          unsigned short int test_V=laser_ctr_param.ledLightLevel*5+200;//energe_140us_voltage[energeNum];//200~600V;        
          p2000w_ctr_param.outVoltageSet =test_V;
          #endif  
          #if 1  
          unsigned short int pulse_width_offeset=0,pulseProUs,pulseOffDelayUs;  
          
          if(p2000w_ctr_param.outVoltageSet<280)//无关触发信号
          {
            pulseOffDelayUs=10;
            pulseProUs=(unsigned short int)(146-p2000w_ctr_param.outVoltageSet*0.40);                   
          }   
          else if(p2000w_ctr_param.outVoltageSet<310)
          {
            pulseOffDelayUs=10;
            pulseProUs=(unsigned short int)(118-p2000w_ctr_param.outVoltageSet*0.30);                     
          }   
          else if( p2000w_ctr_param.outVoltageSet<370)
          {
            pulseOffDelayUs=10;         
            pulseProUs=(unsigned short int)(56- p2000w_ctr_param.outVoltageSet*0.10); 
          }    
          else if( p2000w_ctr_param.outVoltageSet<470)
          {
            if(p2000w_ctr_param.outVoltageSet<410)  {
              pulseOffDelayUs=11;
            }
            else pulseOffDelayUs=10;
            pulseProUs=(unsigned short int)(38- p2000w_ctr_param.outVoltageSet*0.05);                            
          }                    
          else
          {    
            pulseOffDelayUs=11;
            pulseProUs=14;                 
          }  
          pulse_width_offeset =pulseProUs+pulseOffDelayUs;  
          u_p2000w_tx_msg.msg.code=P2000W_CODE_PULSE_WIDTH;
          //u_p2000w_tx_msg.msg.cmd=p2000w_ctr_param.pulseWidthSet;  
          //电源效率和灯管特性在不同频率下有差异，导致脉冲峰值功率变化能量波动，做脉宽补偿（0~12us.1~60Hz）       
          unsigned short int e_q_cali_pulse = 0;//laser_ctr_param.laserFreq/5; 
          u_p2000w_tx_msg.msg.cmd=p2000w_ctr_param.pulseWidthSet;//-pulse_width_offeset+e_q_cali_pulse;  
          p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,P2000W_FRAME_DELAY_TIME); 
          {
            if(p_mtxs!=osOK) 
            {
              DEBUG_PRINTF("set puls width ,resend once!\r\n");
              osDelay(P2000W_FRAME_DELAY_TIME);
              osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0); 
            }
          } 
          u_p2000w_tx_msg.msg.code=P2000W_CODE_PULSE_FREQ;         
          u_p2000w_tx_msg.msg.cmd=p2000w_ctr_param.freqSet;
          p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,P2000W_FRAME_DELAY_TIME); 
          {
            if(p_mtxs!=osOK) 
            {
              DEBUG_PRINTF("set freq ,resend once!\r\n");
              osDelay(P2000W_FRAME_DELAY_TIME);
              osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0); 
            }
          }  
          u_p2000w_tx_msg.msg.code=P2000W_CODE_STA_QUERY;
          u_p2000w_tx_msg.msg.cmd=0;
          p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,P2000W_FRAME_DELAY_TIME); 
          {
            if(p_mtxs!=osOK) 
            {
              DEBUG_PRINTF(" sta req ,resend once!\r\n");
              osDelay(P2000W_FRAME_DELAY_TIME);
              osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0); 
            }
          } 
          unsigned short int   cali_energe= p2000w_ctr_param.outVoltageSet;
          unsigned short int   cali_freq_energe=1.0;//频率补偿
          //target 118 =100*（1+0.18）
           //target 100 =85*（1+0.18）
          if(laser_ctr_param.laserFreq<25)
          {
            cali_freq_energe=( unsigned short int )cali_energe*(laser_ctr_param.laserFreq*0.015);
          }
          else {
           // cali_freq_energe=test_energe*(0.8+(laser_ctr_param.laserFreq-20)*0.005);
           cali_freq_energe=( unsigned short int )cali_energe*(0.24-(laser_ctr_param.laserFreq)*0.004);
          }
          float cali_temprature_energe=0.0;//温度补偿
          cali_temprature_energe=0.02*(sEnvParam.eth_k1_temprature-26.0); 
          if(cali_temprature_energe>0.15) 
          {
            cali_temprature_energe=0.15; 
          }
          if(cali_temprature_energe<-0.15) 
          {
            cali_temprature_energe=-0.15; 
          }
          if(cali_temprature_energe<0)
          {
            cali_energe+=(unsigned short int )fabsf(cali_temprature_energe*p2000w_ctr_param.outVoltageSet);  
          }  
          else cali_energe-=(unsigned short int )fabsf(cali_temprature_energe*p2000w_ctr_param.outVoltageSet);  
          cali_energe-=(cali_freq_energe);               
          cali_energe%=201;  
          u_p2000w_tx_msg.msg.code=P2000W_CODE_VOLTAGE_SET;
          #if 1   
          u_p2000w_tx_msg.msg.cmd= cali_energe; 
          #else 
          u_p2000w_tx_msg.msg.cmd=p2000w_ctr_param.outVoltageSet; 
          #endif      
          p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,P2000W_FRAME_DELAY_TIME);
          if(p_mtxs!=osOK) 
          {            
            DEBUG_PRINTF("set voltage fial ,resend once!\r\n");
            osDelay(P2000W_FRAME_DELAY_TIME);
            osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0);
          }
          DEBUG_PRINTF("lasr_vol=%dv freq=%dHz pulseW=%dus",p2000w_ctr_param.outVoltageSet,p2000w_ctr_param.freqSet,p2000w_ctr_param.pulseWidthSet);
          #else   
          osSemaphoreAcquire(p2000wHeartBinarySem07Handle,2*P2000W_FRAME_TIMEOUT);
          app_p2000w_v_q_set(p2000w_ctr_param.outVoltageSet,p2000w_ctr_param.freqSet,p2000w_ctr_param.pulseWidthSet);
          #endif             
          osDelay(P2000W_FRAME_TIMEOUT);
          if((p2000w_status.ctr_status&P2000W_STA_B3_PRO_HOT_OK)!=P2000W_STA_B3_PRO_HOT_OK)
          {
            u_p2000w_tx_msg.msg.code=P2000W_CODE_PRO_HOT_CTR;
            u_p2000w_tx_msg.msg.cmd=p2000w_ctr_param.proHotCtr;
            p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0);
            if(p_mtxs==osOK)
            {
              DEBUG_PRINTF("prohot req\r\n");
            }
            else
            {
              DEBUG_PRINTF("prohot req fail,resend once !\r\n");
              osDelay(P2000W_FRAME_DELAY_TIME);  
              osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0);
            }
          }                            
          timeout = 0;
          do
          {
            osDelay(P2000W_FRAME_DELAY_TIME);  
            timeout += P2000W_FRAME_DELAY_TIME;
            if(timeout>P2000W_PRO_HOT_WAIT_TIMEOUT)    
            {              
              DEBUG_PRINTF("p2000w prohot timeout ,ctr status=0x%02x\r\n",p2000w_status.ctr_status); 
              break;
            }   
          }while((p2000w_status.ctr_status&P2000W_STA_B3_PRO_HOT_OK)!=P2000W_STA_B3_PRO_HOT_OK);            
          if((p2000w_status.ctr_status&P2000W_STA_B3_PRO_HOT_OK)==P2000W_STA_B3_PRO_HOT_OK) 
          {
            if((p2000w_status.ctr_status&P2000W_STA_B4_RELAY_OK)!=P2000W_STA_B4_RELAY_OK)
            {               
              u_p2000w_tx_msg.msg.code=P2000W_CODE_RELEY_CTR;
              u_p2000w_tx_msg.msg.cmd=1;
              p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0);
              if(p_mtxs==osOK)
              {
                DEBUG_PRINTF("p2000w reley on \r\n"); 
              }
              else{
                DEBUG_PRINTF("p2000w reley on fail,resend once\r\n"); 
                osDelay(P2000W_FRAME_DELAY_TIME);  
                osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0);
              }
            }       
            timeout=0;
            do
            {
              osDelay(P2000W_FRAME_DELAY_TIME);  
              timeout += P2000W_FRAME_DELAY_TIME; 
              if(timeout>P2000W_RELEY_WAIT_TIMEOUT)    
              {                
                DEBUG_PRINTF("p2000w reley open timeout\r\n");                   
                break;
              }   
            }while((p2000w_status.ctr_status&P2000W_STA_B4_RELAY_OK)!=P2000W_STA_B4_RELAY_OK);
            if((p2000w_status.ctr_status&P2000W_STA_B4_RELAY_OK)==P2000W_STA_B4_RELAY_OK)
            {
              rgbMessage = RGB_LASER_PREPARE_OK;
              sGenSta.laser_run_B5_timer_status=0;               
              osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);   
              osEventFlagsSet(laserEvent02Handle,EVENTS_LASER_1064_PREPARE_OK_BIT|EVENTS_LASER_JT_ENABLE_BIT); 
              DEBUG_PRINTF("p2000w prohot success\r\n");
              sGenSta.laser_run_B0_pro_hot_status=1;
            }
            else
            {
              DEBUG_PRINTF("laser 1064 reley open fail");  
              u_p2000w_tx_msg.msg.code=P2000W_CODE_PRO_HOT_CTR;
              u_p2000w_tx_msg.msg.cmd=0;
              p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0);
              if(p_mtxs==osOK)
              {
                DEBUG_PRINTF("exit prohot \r\n"); 
              }
              else{
                DEBUG_PRINTF("exit prohot fail,resend once\r\n"); 
                osDelay(P2000W_FRAME_DELAY_TIME);  
                osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_tx_msg.data,0,0);
              }
              laser_ctr_param.proHotCtr=0;
              local_proHotCtr=0;
            }
          }            
          else {            
            DEBUG_PRINTF("laser 1064 prohot  fail!\r\n");
            p2000w_ctr_param.proHotCtr = 0;
            sGenSta.laser_run_B0_pro_hot_status = 0;   
            local_proHotCtr = 0;
          }          
        }
        else 
        {
          if(p2000w_ctr_param.p2000wHeart==0)
          {
            DEBUG_PRINTF("1064 prohot fail : p2000W disconnect ! power off \r\n");            
            //app_high_voltage_solenoid(DISABLE);              
          }
          else
          {
            //app_high_voltage_solenoid(DISABLE);
            if((p2000w_status.ctr_status&P2000W_STA_B1_PFC_OK)!=P2000W_STA_B1_PFC_OK)
            {
              DEBUG_PRINTF("1064 prohot fail : PFC ERR ! power off\r\n");
            }            
            if((p2000w_status.error_code&P2000W_ERROR_LLC_OVERLOAD)==P2000W_ERROR_LLC_OVERLOAD)
            {
              DEBUG_PRINTF("1064 prohot fail : temprature high please wait 10 secends !power off\r\n");
            }
            else  DEBUG_PRINTF("1064 prohot fail: p2000w,error code=0x%02x! power off\r\n",p2000w_status.error_code);  
          } 
          p2000w_ctr_param.proHotCtr = 0;
          sGenSta.laser_run_B0_pro_hot_status = 0;   
          local_proHotCtr = 0;
        }                      
      }
    }		
    else 
    {
      if(sGenSta.laser_run_B0_pro_hot_status!=0)
      {
        osSemaphoreRelease(laserCloseSem05Handle);
      }  
      p2000w_ctr_param.proHotCtr=0;
    }	    	
		osDelay(1);	
  }
  /* USER CODE END laserProhotTask09 */
}

/* USER CODE BEGIN Header_ge2117ManageTask10 */
/**
* @brief Function implementing the myTask10 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ge2117ManageTask10 */
void ge2117ManageTask10(void *argument)
{
  /* USER CODE BEGIN ge2117ManageTask10 */
  
  /* Infinite loop */
  uint32_t local_time100mS=0,local_timeMs=0;
  local_time100mS = 0;  
  static KalmanFilter kalmTemprature;
  //float k0_cool_temprature;
  max_31865_pt1000(); 
  kalman_filter_init(&kalmTemprature, 25.0, 0.1);
  osDelay(200);
  for(;;)
  {  
    osDelay(100);
    if(laser_ctr_param.cleanCtr==0) 
    { 
      app_treatment_water_prepare(&laser_ctr_param.air_water_prepare_ctr,100);
    } 
    local_time100mS++;
    local_timeMs+=100;       
    float temp_t_f = Get_pt_tempture();
    sEnvParam.eth_k1_temprature =(float) kalman_filter_update(&kalmTemprature, temp_t_f); 
    if((u_s_l980.sta.staByte&L980_STA_HEART_BIT0)==L980_STA_HEART_BIT0)
    {
      sEnvParam.eth_k2_temprature = u_s_l980.sta.realtemprature*0.1; 
    }
    else sEnvParam.eth_k2_temprature = temp_t_f;
   
    if(sEnvParam.eth_k2_temprature>ERR_LOW_TEMPRATURE_LASER&&sEnvParam.eth_k2_temprature<ERR_HIGH_TEMPRATURE_LASER)
    {
      app_circle_water_PTC_manage(sEnvParam.eth_k2_temprature,local_timeMs);//tec   
    }
    else 
    {
      app_circle_water_PTC_manage(u_sys_param.sys_config_param.cool_temprature_target*0.1,local_timeMs);//stop      
    }    
  }
  /* USER CODE END ge2117ManageTask10 */
}

/* USER CODE BEGIN Header_musicTask11 */
/**
* @brief Function implementing the myTask11 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_musicTask11 */
void musicTask11(void *argument)
{
  /* USER CODE BEGIN musicTask11 */
  /* Infinite loop */
  uint16_t music_num=0;  
  for(;;)
  {
    osMessageQueueGet(musicQueue03Handle,&music_num,0,portMAX_DELAY);	
    app_buzz_music(music_num,50);  
    osDelay(10);
  }
  /* USER CODE END musicTask11 */
}

/* USER CODE BEGIN Header_p2000wReceiveTask12 */
/**
* @brief Function implementing the myTask12 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_p2000wReceiveTask12 */
void 

p2000wReceiveTask12(void *argument)
{
  /* USER CODE BEGIN p2000wReceiveTask12 */
  /* Infinite loop */
  unsigned short int packLen;  
  unsigned  int heartTimeOut=0;
  unsigned char pTxMessageBuff[4]={0};
  U_P2000W_TX_MSG u_p2000w_ctr_msg;
  app_p2000w_init();
	osTimerStart(p2000wHeartTimer04Handle,P2000W_FRAME_DELAY_TIME);
  for(;;)
  {  
    osStatus_t status  = osSemaphoreAcquire(p2000wHeartBinarySem07Handle,10);
    if(status==osOK)
    {  
      osStatus_t p_m = osMessageQueueGet(p2000wTxMessageQueue04Handle,pTxMessageBuff,0,0);
      if(p_m==osOK)   {
        U_P2000W_TX_MSG *p_u_msg;
        p_u_msg = (U_P2000W_TX_MSG*)pTxMessageBuff;        
        app_p2000w_ctr_tansmit(p_u_msg->msg.code,&p_u_msg->data[2]);
      }
      else {//refresh status
        u_p2000w_ctr_msg.msg.code=P2000W_CODE_STA_QUERY;
      u_p2000w_ctr_msg.msg.cmd=0;
        app_p2000w_ctr_tansmit(P2000W_CODE_STA_QUERY,&u_p2000w_ctr_msg.data[2]);
      }
      osDelay(10);
    } 
    packLen = app_p2000w_package_check();
    if(packLen==0)
    {        
      heartTimeOut += 20;
      if(heartTimeOut>P2000W_FRAME_TIMEOUT)
      {//heart err 
        heartTimeOut = 0;  
        if(p2000w_ctr_param.p2000wHeart!=0)
        {            
          DEBUG_PRINTF("p2000W disconnect!\r\n");  
          p2000w_ctr_param.p2000wHeart=0;
          if((p2000w_status.ctr_status&P2000W_STA_B3_PRO_HOT_OK)==P2000W_STA_B3_PRO_HOT_OK)         
          {//p2000W,exit prohot 
            osSemaphoreRelease(laserCloseSem05Handle);
          }
          memset(&p2000w_status,0,sizeof(P_2000W_STATUS));//clear
        }
      }  
    }      
    else 
    {
      if(packLen>5)        
      {  
        heartTimeOut=0;       
        app_p2000w_status_handle(&p2000w_status);
      }       
    } 
    if(p2000w_status.init_status==4)
    {     
      osSemaphoreAcquire(p2000wHeartBinarySem07Handle,2*P2000W_FRAME_DELAY_TIME);
      DEBUG_PRINTF("p2000w init fail ,restart\r\n");       
      app_p2000w_init(); 
      osDelay(P2000W_FRAME_DELAY_TIME);    
      u_p2000w_ctr_msg.msg.code=P2000W_CODE_RECONNECT;
      u_p2000w_ctr_msg.msg.cmd=0;
      osStatus_t status2  = osSemaphoreAcquire(p2000wHeartBinarySem07Handle,P2000W_FRAME_DELAY_TIME*2);
      if(status2==osOK)
      {
        osStatus_t p_m = osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_ctr_msg.data,0,1);
        if(p_m!=osOK)
        { //re connect        
          DEBUG_PRINTF("p2000w reconnect cmd tx fail resend once ! \r\n");  
          osDelay(P2000W_FRAME_DELAY_TIME);   
          osMessageQueuePut(p2000wTxMessageQueue04Handle,u_p2000w_ctr_msg.data,0,0);         
        }       
      }
    }  
    osDelay(10);
  }
  
  /* USER CODE END p2000wReceiveTask12 */
}

/* LaserWorkTimerCallback01 function */
void LaserWorkTimerCallback01(void *argument)
{
  /* USER CODE BEGIN LaserWorkTimerCallback01 */
  sGenSta.laser_run_B5_timer_status=1;
  /* USER CODE END LaserWorkTimerCallback01 */
}

/* cleanWaterCallback02 function */
void cleanWaterCallback02(void *argument)
{
  /* USER CODE BEGIN cleanWaterCallback02 */
  osTimerStop(cleanTimer02Handle);  
  if(laser_ctr_param.cleanCtr!=0)
  {    
    laser_ctr_param.cleanCtr = 0; 
  } 
  tmc2226_stop(); 
  osTimerDelete(cleanTimer02Handle);       
  /* USER CODE END cleanWaterCallback02 */
}

/* tmcMaxRunTimesCallback03 function */
void tmcMaxRunTimesCallback03(void *argument)
{
  /* USER CODE BEGIN tmcMaxRunTimesCallback03 */
  osTimerStop(tmcMaxRunTimer03Handle);
  tmc2226_stop();
  if(laser_ctr_param.air_water_prepare_ctr != 0)  
  {
    laser_ctr_param.air_water_prepare_ctr=0;
  }  
  osTimerDelete(tmcMaxRunTimer03Handle);
  /* USER CODE END tmcMaxRunTimesCallback03 */
}

/* p2000wHeartCallback04 function */
void p2000wHeartCallback04(void *argument)
{
  /* USER CODE BEGIN p2000wHeartCallback04 */
  osSemaphoreRelease(p2000wHeartBinarySem07Handle);
  #if 1
  static unsigned int time=0;
  time+=P2000W_FRAME_DELAY_TIME;
  if(time>=HMI_CAN_FRAME_DELAY_TIME) //100ms
  {
    time = 0;
    osSemaphoreRelease(hmiCanBusIdleSem06Handle);
  } 
  #else //50ms  
  osSemaphoreRelease(hmiCanBusI
  dleSem06Handle);
  #endif
  /* USER CODE END p2000wHeartCallback04 */
}
/* beepHeartCallback05 function */
void beepHeartCallback05(void *argument)
{
  /* USER CODE BEGIN beepHeartCallback05 */
  uint16_t s_music_num = MUSIC_LASER_WORK;
 //osMessageQueuePut(musicQueue03Handle,&s_music_num,0,0);
  /* USER CODE END beepHeartCallback05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to correspon
  * 
  * ding EXTI line.
  * @retval None
  */
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {    
    if(GPIO_Pin==FAN1_COUNT_in_Pin)
    {
      app_fan_feed_count(1);
    }
    if(GPIO_Pin==FAN2_COUNT_in_Pin)
    {
      app_fan_feed_count(2);
    }  
    if(GPIO_Pin==LASER_PULSE_COUNT_in_Pin)
    {   
      if(laser_ctr_param.lowEnergeMode==0)
      { 
        //unsigned short int timus_offeset=(app_laser_1064_energe_to_voltage(laser_ctr_param.laserEnerge)-80)/20; 
        //unsigned char Len=4*(u_sys_param.sys_config_param.laser_pulse_width_us-timus_offeset)/5;    
        unsigned char Len=u_sys_param.sys_config_param.laser_pulse_width_us+40;//4*(u_sys_param.sys_config_param.laser_pulse_width_us)/5;
        pulse_adc_start(Len); 
        u_sys_param.sys_config_param.laser_pulse_count++;      
      }
    }   
    #ifdef ONE_WIRE_BUS_JT_SLAVE 
    if(GPIO_Pin==FOOT_SWITCH_IN_Pin)
    {  
      owb_dq_edge_callback();    
    }
    #endif
 }
 /*************************************************************************//**
  * @brief 系统一般状态信号管理
  * @param 无
  * @note   周期性更新事件正常标志
  * @retval 无
  *****************************************************************************/
void app_sys_genaration_status_manage(void)
{ //高压电磁阀
  if(app_get_io_status(In1_high_voltage_solenoid)==SUCCESS)
  {      
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO1_BIT);
  }
  else 
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO1_BIT); 
  } 
  #if 0
  //堵气阀
  if(app_get_io_status(In2_deflate_air_solenoid)==SUCCESS)
  {      
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO2_BIT);
  }
  else   
  {   
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO2_BIT);
  } 
  //泄气阀
  if(app_get_io_status(In3_chocke_air_solenoid)==SUCCESS)
  {  
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO3_BIT);
  }
  else 
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO3_BIT);
  }
  #else
  //气阀 ,不再使用
  osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO2_BIT);
  osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO3_BIT);
  #endif 
   //环境温度报警
  if(app_get_io_status(In4_enviroment_tmprature_alert)==SUCCESS)
  {  
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO4_BIT);    
  }
  else 
  {    
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO4_BIT);
  }	//气泵过热报警
	if(app_get_io_status(In5_h_air_error)==SUCCESS)
	{  
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO5_BIT); 
	}
	else 
  {   
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO5_BIT);
  }  
	//气泵气压过高信号报警,低报警
	if(app_get_io_status(In6_Hyperbaria_OFF_Signal)==SUCCESS)
	{ 
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO6_BIT);  
	}
	else
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO6_BIT);
  }  
  //治疗水OK就绪信号 
	if(app_get_io_status(In7_water_ready_ok)==SUCCESS&&sEnvParam.treatment_water_depth!=0)
	{  
     osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO7_BIT);
	}
	else 
  {      
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO7_BIT);
  }
  //水循环就绪信号
	if(app_get_io_status(In8_water_circle_ok)==SUCCESS&&sEnvParam.cool_water_depth>=u_sys_param.sys_config_param.cool_water_depth_low)
	{ 
    if(sEnvParam.cool_water_depth<=u_sys_param.sys_config_param.cool_water_depth_high)
    {     
      osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO8_BIT);
    } 
	  else osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO8_BIT);
  }
	else 
  {  
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO8_BIT);
  }
  if(sEnvParam.NTC_temprature>MAX_TMC2226_NTC_TEMPRATURE)
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_9_NTC_BIT);  
  }
  else
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_9_NTC_BIT);
  }
  if(sEnvParam.iBus<MAX_IBUS_MA)//<10A
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_10_IBUS_BIT);    
  }
  else
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_10_IBUS_BIT);
  }  
  if(sEnvParam.vBus>MIN_VBUS_MV)
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_11_VBUS_BIT);
  }
  else
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_11_VBUS_BIT);
  } 
  if(sEnvParam.eth_k1_temprature>ERR_LOW_TEMPRATURE_LASER&&sEnvParam.eth_k1_temprature<ERR_HIGH_TEMPRATURE_LASER)
  { 
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT);       
  }  
  else
  {   
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT);           
  }  
		if(sEnvParam.eth_k2_temprature>ERR_LOW_TEMPRATURE_LASER&&sEnvParam.eth_k2_temprature<ERR_HIGH_TEMPRATURE_LASER)
    {
      osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_13_K2_TEMPRATURE_BIT);
    }
    else 
    {     
      osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_13_K2_TEMPRATURE_BIT);
    } 
  //紧急开关
  if(app_get_io_status(In9_emergency_ok)==SUCCESS)
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_14_EMERGENCY_KEY_BIT);
  }
  else 
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_14_EMERGENCY_KEY_BIT);
    if(laser_ctr_param.proHotCtr!=0&&sGenSta.laser_run_B0_pro_hot_status!=0)
    {
      laser_ctr_param.proHotCtr=0;   
      DEBUG_PRINTF("emergency!\r\n");   
      osSemaphoreRelease(laserCloseSem05Handle);  
    } 
  }
  //治疗水瓶液位 低有效
  if(HAL_GPIO_ReadPin(TREATMENT_WATER_DEPTH_in_GPIO_Port,TREATMENT_WATER_DEPTH_in_Pin)==GPIO_PIN_RESET)
  {
    sEnvParam.treatment_water_depth=1;   
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_15_WATER_AIR_PREPARE_BIT ); 
  }  
  else
  {
    sEnvParam.treatment_water_depth=0;
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_15_WATER_AIR_PREPARE_BIT );
  } 
  if(sEnvParam.cool_water_depth<u_sys_param.sys_config_param.cool_water_depth_low||sEnvParam.cool_water_depth>u_sys_param.sys_config_param.cool_water_depth_high)
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_16_COOL_WATER_BIT);
  }
  else 
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_16_COOL_WATER_BIT);
  }
  //DEBUG_PRINTF("IO8~1=%d%d%d%d%d%d%d%d\r\n" ,sGenSta.water_circle_ok_status,sGenSta.water_ready_ok_status,\
    sGenSta.Hyperbaria_OFF_Signal_staus,sGenSta.h_air_error_status ,sGenSta.enviroment_tmprature_alert_status,\
    sGenSta.chocke_air_solenoid_status, sGenSta.deflate_air_solenoid_status,sGenSta.high_voltage_solenoid_status);
}
/************************************************************************//**
  * @brief 计算设备ID
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
unsigned  int app_get_cali_devid(void)
{
	unsigned  int id;
	unsigned char buf[12];//96bit	
	buf[0]=HAL_GetUIDw0()&0xFF;
	buf[1]=(HAL_GetUIDw0()>>8)&0xFF;
	buf[2]=(HAL_GetUIDw0()>>16)&0xFF;
	buf[3]=(HAL_GetUIDw0()>>24)&0xFF;

	buf[4]=HAL_GetUIDw1()&0xFF;
	buf[5]=(HAL_GetUIDw1()>>8)&0xFF;
	buf[6]=(HAL_GetUIDw1()>>16)&0xFF;
	buf[7]=(HAL_GetUIDw1()>>24)&0xFF;	

	buf[8]=HAL_GetUIDw2()&0xFF;
	buf[9]=(HAL_GetUIDw2()>>8)&0xFF;
	buf[10]=(HAL_GetUIDw2()>>16)&0xFF;
	buf[11]=(HAL_GetUIDw2()>>24)&0xFF;	
	id =  crc32_MPEG(buf,12);
	return id;
}	
/************************************************************************//**
  * @brief 使用默认系统参数
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
void app_set_default_sys_config_param(void)
{	
	u_sys_param. sys_config_param.synchronousFlag = 0;
  if(u_sys_param. sys_config_param.equipmentId==0||u_sys_param. sys_config_param.equipmentId==0xFFFFFFFF)
  {
	  u_sys_param. sys_config_param.equipmentId = app_get_cali_devid();//默认参数
  }
  u_sys_param. sys_config_param.jtId=0;
  u_sys_param. sys_config_param.jt_status=1; 
  u_sys_param. sys_config_param.cool_temprature_low=210;//21.0f
  u_sys_param. sys_config_param.cool_temprature_target=240;//24.0f
  u_sys_param. sys_config_param.cool_temprature_high=280;//28.0f
  u_sys_param. sys_config_param.photodiod_low=0;//50mj
  u_sys_param. sys_config_param.photodiod_mid=99;//100mJ
  u_sys_param. sys_config_param.photodiod_high=200;//205mJ
  u_sys_param. sys_config_param.laser_pulse_count=0;
  u_sys_param. sys_config_param.laser_use_timeS=0;
  u_sys_param. sys_config_param.RDB_use_timeS=0; 
  u_sys_param. sys_config_param.laser_pulse_width_us=140;
  u_sys_param. sys_config_param.treatment_water_depth_high=150;//15.1pf
  u_sys_param. sys_config_param.treatment_water_depth_low=1;//1.0pF
  u_sys_param. sys_config_param.cool_water_depth_high=69;//7.0pf
  u_sys_param. sys_config_param.cool_water_depth_low=62;//6.1pF
  u_sys_param. sys_config_param.air_low_pressure=140;
  u_sys_param. sys_config_param.air_mid_pressure=170;
  u_sys_param. sys_config_param.air_high_pressure=190;
  u_sys_param. sys_config_param.t_water_low=15;//10ml/min
  u_sys_param. sys_config_param.t_water_mid=25;
  u_sys_param. sys_config_param.t_water_high=35;
  u_sys_param. sys_config_param.dit_time_min=5;//5min
  u_sys_param. sys_config_param.clean_time_min=5;
  u_sys_param. sys_config_param.tec_switch=1;
  u_sys_param. sys_config_param.low_freq=60;//MAX60Hz 
  u_sys_param. sys_config_param.charge_width_us=180;//180us//
  u_sys_param.sys_config_param.laser_led_light=50;            //）激光指示灯亮度
  u_sys_param.sys_config_param.rgb_light=50;                  //）rgb状态指示灯亮度
  u_sys_param.sys_config_param.beep=50;                       //）音量
  for(uint8_t i=0;i<40;i++)
  {  
    u_sys_param. sys_config_param.e_cali[i].energe_cali=2500;
    u_sys_param. sys_config_param.e_cali[i].power_cali=2500;//
  }
  u_sys_param.sys_config_param.checkSum=sumCheck(u_sys_param.data,SYS_LASER_CONFIG_PARAM_LENGTH);//sizeof(SYS_CONFIG_PARAM)-4);	 
	DEBUG_PRINTF("sys param load failed! load defalut param\r\n");
  memcpy(u_sys_default_param.data,u_sys_param.data,sizeof(SYS_CONFIG_PARAM)); 
  laser_ctr_param.airPressureLevel=1;
  laser_ctr_param.treatmentWaterLevel=0; 
  laser_ctr_param.ledLightLevel=20;
}
/************************************************************************//**
  * @brief 加载本地系统参数
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
  void app_sys_param_load(void)
  {
    unsigned char flag;
    flag = EEPROM_M24C32_Read(EEROM_SYS_PARAM_SAVE_ADDR, u_sys_param.data, sizeof(SYS_CONFIG_PARAM));
    unsigned int sum = sumCheck(u_sys_param.data,SYS_LASER_CONFIG_PARAM_LENGTH);   
    if(u_sys_param.sys_config_param.synchronousFlag==EEROM_DATA_ERR_CHECK_FLAG||u_sys_param.sys_config_param.checkSum!=sum)//
    {
      app_set_default_sys_config_param();
      DEBUG_PRINTF("load default sys param\r\n");		 
    }
    else 
    { //check param
      u_sys_param.sys_config_param.synchronousFlag=0;//请求配置      
      if(u_sys_param.sys_config_param.cool_temprature_target>280||u_sys_param.sys_config_param.cool_temprature_target<210)
      {
        u_sys_param.sys_config_param.cool_temprature_target = 240;
      } 
      memcpy(u_sys_default_param.data,u_sys_param.data,sizeof(SYS_CONFIG_PARAM));  
      laser_ctr_param.airPressureLevel=1;
      laser_ctr_param.treatmentWaterLevel=0; 
      laser_ctr_param.ledLightLevel=20;
      #if 1
      DEBUG_PRINTF("***************sys param read ok*************************\r\n");
      DEBUG_PRINTF("synchronousFlag=%d\r\n", u_sys_param. sys_config_param.synchronousFlag);
      DEBUG_PRINTF("equipmentId=%x\r\n",u_sys_param. sys_config_param.equipmentId);
      DEBUG_PRINTF("jtId=%d\r\n",u_sys_param. sys_config_param.jtId);
      DEBUG_PRINTF("jt_status=%d\r\n",u_sys_param. sys_config_param.jt_status);
      DEBUG_PRINTF("cool_temprature_low=%.1f℃\r\n",u_sys_param. sys_config_param.cool_temprature_low*0.1);
      DEBUG_PRINTF("cool_temprature_target=%.1f℃\r\n",u_sys_param. sys_config_param.cool_temprature_target*0.1);
      DEBUG_PRINTF("cool_temprature_high=%.1f℃\r\n",u_sys_param. sys_config_param.cool_temprature_high*0.1);

      DEBUG_PRINTF("photodiod_low=%d\r\n",u_sys_param. sys_config_param.photodiod_low);
      DEBUG_PRINTF("photodiod_mid=%d\r\n",u_sys_param. sys_config_param.photodiod_mid);
      DEBUG_PRINTF("photodiod_high=%d\r\n",u_sys_param. sys_config_param.photodiod_high); 
      DEBUG_PRINTF("laser_pulse_count=%d\r\n",u_sys_param. sys_config_param.laser_pulse_count);   
      DEBUG_PRINTF("laser_use_timeS=%d\r\n",u_sys_param. sys_config_param.laser_use_timeS);   
      DEBUG_PRINTF("RDB_use_timeS=%d\r\n",u_sys_param. sys_config_param.RDB_use_timeS);  
      DEBUG_PRINTF("laser_pulse_width_us=%d\r\n",u_sys_param. sys_config_param.laser_pulse_width_us);
      DEBUG_PRINTF("treatment_water_depth_high=%.1fr\n",u_sys_param. sys_config_param.treatment_water_depth_high*0.1);
      DEBUG_PRINTF("treatment_water_depth_low=%.1f\r\n",u_sys_param. sys_config_param.treatment_water_depth_low*0.1);
      DEBUG_PRINTF("cool_water_depth_high=%.1f \r\n",u_sys_param. sys_config_param.cool_water_depth_high*0.1);
      DEBUG_PRINTF("cool_water_depth_low=%.1f \r\n",u_sys_param. sys_config_param.cool_water_depth_low*0.1);
      DEBUG_PRINTF("air_low_pressure=%d kPa\r\n",u_sys_param. sys_config_param.air_low_pressure); 
      DEBUG_PRINTF("air_mid_pressure=%d kPa\r\n",u_sys_param. sys_config_param.air_mid_pressure);
      DEBUG_PRINTF("air_high_pressure=%d kPa\r\n",u_sys_param. sys_config_param.air_high_pressure);
      DEBUG_PRINTF("t_water_low=%d ml/min\r\n",u_sys_param. sys_config_param.t_water_low);
      DEBUG_PRINTF("t_water_mid=%d ml/min\r\n",u_sys_param. sys_config_param.t_water_mid);
      DEBUG_PRINTF("t_water_high=%d  ml/min\r\n",u_sys_param. sys_config_param.t_water_high);
      DEBUG_PRINTF("dit_time_min=%d  min\r\n",u_sys_param. sys_config_param.dit_time_min);
      DEBUG_PRINTF("clean_time_min=%d  min\r\n",u_sys_param. sys_config_param.clean_time_min);
      DEBUG_PRINTF("tec_switch=%d\r\n",u_sys_param. sys_config_param.tec_switch);
      DEBUG_PRINTF("low_freq=%d\r\n",u_sys_param. sys_config_param.low_freq);
      DEBUG_PRINTF("charge_width_us=%d\r\n",u_sys_param. sys_config_param.charge_width_us);  
      DEBUG_PRINTF("laser_led_light=%d\r\n",u_sys_param. sys_config_param.laser_led_light);
      DEBUG_PRINTF("rgb_light=%d\r\n",u_sys_param. sys_config_param.rgb_light);
      DEBUG_PRINTF("beep=%d\r\n",u_sys_param. sys_config_param.beep);
      DEBUG_PRINTF("eCali=%d\r\n",u_sys_param. sys_config_param.e_cali->energe_cali);
      DEBUG_PRINTF("pCali=%d\r\n",u_sys_param. sys_config_param.e_cali->power_cali);
      DEBUG_PRINTF("*******************sys param end*************************\r\n");
      #endif
    }	  
  }
/************************************************************************//**
  * @brief 系统参数保存到本地
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 unsigned char app_sys_param_save_data(void)
 {
    unsigned char flag=0;	 
    if(compare_buff_no_change(u_sys_param.data,u_sys_default_param.data,sizeof(SYS_CONFIG_PARAM))!=HAL_OK)
    {
      u_sys_param.sys_config_param.checkSum=sumCheck(u_sys_param.data,SYS_LASER_CONFIG_PARAM_LENGTH);
      flag = EEPROM_M24C32_Write(EEROM_SYS_PARAM_SAVE_ADDR, u_sys_param.data, sizeof(SYS_CONFIG_PARAM));	
    }		  
    return flag;
 }

 /************************************************************************//**
  * @brief 气泵管理
  * @param air_level 气泵气压等级
  * @note   
  * @retval 
  *****************************************************************************/
 void app_air_pump_manage(unsigned char air_level)
 {		
    unsigned char duty_cali=0,duty;
		float air_pressure=MID_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;
		if(air_level==1)
    {      
      duty=20;
      air_pressure = u_sys_param.sys_config_param.air_low_pressure+sEnvParam.air_gzp_enviroment_pressure_kpa;
      //air_pressure = MIN_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;
    }
    else if(air_level==3)
    {     
      duty=30;
      air_pressure   =  u_sys_param.sys_config_param.air_mid_pressure+sEnvParam.air_gzp_enviroment_pressure_kpa;
      //air_pressure = MAX_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;			
    }
    else if(air_level==2)
    {      
      duty=25;
      air_pressure = u_sys_param.sys_config_param.air_high_pressure+sEnvParam.air_gzp_enviroment_pressure_kpa;
      //air_pressure=MID_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;
    }    
    
    if(air_level!=sGenSta.air_level_status)//((eventFlag&EVENTS_AUX_STATUS_IO6_BIT)== EVENTS_AUX_STATUS_IO6_BIT)&&sGenSta.laser_run_B0_pro_hot_status!=0)//
    {
      if(sEnvParam.air_gzp_enviroment_pressure_kpa>100.0)
      {
        duty_cali=(sEnvParam.air_pump_pressure-95.0)*100/air_pressure;
        duty_cali%=5;//校准值不超过5%  
        app_air_pum_pwm_set(duty-duty_cali);     
      }
      else  if(sEnvParam.air_gzp_enviroment_pressure_kpa<90.0)
      {
        duty_cali=(95.0-sEnvParam.air_pump_pressure)*100/air_pressure;
        duty_cali%=5;//校准值不超过5%
        app_air_pum_pwm_set(duty+duty_cali);
      }
      else 
      {
        duty_cali=0;
        app_air_pum_pwm_set(duty);
      }           
    }
    #if 1
    //本机取消气瓶
    sGenSta.laser_param_B23_air_pump_pressure_status=1;
    #else 
		if(sEnvParam.air_pump_pressure+10<(MIN_AIR_PUMP_PRESSURE + sEnvParam.air_gzp_enviroment_pressure_kpa) )	
		{
			sGenSta.laser_param_B23_air_pump_pressure_status = 0;
		}	
		else if(sEnvParam.air_pump_pressure > ( MAX_AIR_PUMP_PRESSURE + sEnvParam.air_gzp_enviroment_pressure_kpa+10) )	
		{
			sGenSta.laser_param_B23_air_pump_pressure_status = 2;
		}	
		else 
		{
			sGenSta.laser_param_B23_air_pump_pressure_status = 1;
		}	
    #endif		
 }
 /************************************************************************//**
  * @brief 更新状态参数
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
  void app_fresh_laser_status_param(void)
  {
    if(sGenSta.laser_run_B0_pro_hot_status!=0&&laser_ctr_param.laserType!=0)
    {//980
      sGenSta.circle_water_box_temprature = (char)((int)sEnvParam.eth_k2_temprature);
    }
    else
    {
      sGenSta.circle_water_box_temprature = (char)((int)sEnvParam.eth_k1_temprature);
    }  
    if(sGenSta.laser_run_B0_pro_hot_status==0) sGenSta.laser_param_B01_energe_status = 1; 
    sGenSta.treatment_water_level_status = laser_ctr_param.treatmentWaterLevel;	
    sGenSta.laser_run_B2_gx_test_status = laser_ctr_param.ctrTestMode;  
  }
  /************************************************************************//**
  * @brief laser
  * @param energe ,能量
  * @note   能量单位mJ;脉宽100~200us；暂时固定脉宽140us用来调试
  * @retval  换算后电压1V
  *****************************************************************************/
  unsigned short int  app_laser_1064_energe_to_voltage(unsigned short int energe)
  {
    unsigned short int ret_vol;   
   #if 1
   //float p_avg=(e_feedback*0.00253)*u_sys_param.sys_config_param.laser_pulse_width_us*laser_ctr_param.laserFreq;
  // sEnvParam.laser_1064_energy=p_avg/laser_ctr_param.laserFreq; 
  // p_avg=sEnvParam.laser_1064_energy*laser_ctr_param.laserFreq;  
  // p_peak=p_avg/(u_sys_param.sys_config_param.laser_pulse_width_us*laser_ctr_param.laserFreq)==(ret_rol)*param; 
  // sEnvParam.laser_1064_energy/(u_sys_param.sys_config_param.laser_pulse_width_us)==(ret_rol)*param; 
    //(ret_rol)= sEnvParam.laser_1064_energy/(u_sys_param.sys_config_param.laser_pulse_width_us*param2); 
   ret_vol=energe*295/u_sys_param.sys_config_param.laser_pulse_width_us+200;
   #else
      if(energe<50) ret_vol=energe*0.4+300;   
      else ret_vol=(energe-20)*1.10+300; 
    #endif  
    if(ret_vol<LASER_1064_MIN_ENERGE_V) ret_vol=LASER_1064_MIN_ENERGE_V;
    if(ret_vol>LASER_1064_MAX_ENERGE_V) ret_vol=LASER_1064_MAX_ENERGE_V;
    return ret_vol;
  }
  /************************************************************************//**
  * @brief app_laser_980_energe_to_voltage
  * @param energe ,能量
  * @note   能量单位mJ; 返回DAC值
  * @retval  换算后电压100mV
  *****************************************************************************/
  unsigned short int  app_laser_980_energe_to_voltage(unsigned short int energe)
  {
    unsigned short int ret_dac; 
    ret_dac=energe*6+100;  
    if(ret_dac>LASER_980_MAX_ENERGE_DAC_MV) ret_dac=LASER_980_MAX_ENERGE_DAC_MV;
    return ret_dac;
  }
 /***************************extern api**********************************************************/
 /************************************************************************//**
  * @brief 给出canBus数据接收信号量
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 void app_canBbus_receive_semo(void) 
 {  
	  osSemaphoreRelease(CANBusReceiveFrameSem04Handle);   
 }
 /************************************************************************//**
  * @brief 给出激光准备信号量
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
void app_laser_preapare_semo(void)
{
  osSemaphoreRelease( laserPrapareReqSem03Handle); 
}   
 /************************************************************************//**
  * @brief 冷却水位更新
  * @param 无 
  * @note   
  * @retval 
  *****************************************************************************/
 void app_cool_water_depth_semo(void)
 {
    osSemaphoreRelease( coolWaterDepthBinarySem01Handle); 
 }  
/************************************************************************//**
  * @brief 水雾准备
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 void app_treatment_water_prepare(unsigned char *ctrflag,unsigned int runtimeMs)
 {
  static unsigned char local_tmc_flag;    
  if(*ctrflag!=0)
  {
    if(local_tmc_flag==0)
    {
      tmc2226_start(TMC_WATER_OUT_DIR_VALUE,3,CONTINUOUS_STEPS_COUNT);     
      // create timer using callback function (was incorrectly passing handle)
      tmcMaxRunTimer03Handle = osTimerNew(tmcMaxRunTimesCallback03, osTimerOnce, NULL, &tmcMaxRunTimer03_attributes);
      if(osTimerIsRunning(tmcMaxRunTimer03Handle)==pdFALSE) osTimerStart(tmcMaxRunTimer03Handle,20*SYS_1_MINUTES_TICK);//MAX20minutes
      local_tmc_flag = 1;
    }    
  }
  else 
  {
    if(local_tmc_flag!=0)
    {
      osTimerStop(tmcMaxRunTimer03Handle);     
      tmc2226_start(!TMC_WATER_OUT_DIR_VALUE,3,CONTINUOUS_STEPS_COUNT); 
      osDelay(300);
      tmc2226_stop(); 
      osTimerDelete(tmcMaxRunTimer03Handle);      
      local_tmc_flag = 0;      
    }    
  }
}
  /************************************************************************//**
  * @brief 消毒、清洗指令
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 void app_t_clean_run_timer(unsigned char *runflag)
 {
    if(*runflag!=0)
    {
      unsigned int runtimeS;
      if(*runflag ==1) runtimeS = u_sys_param.sys_config_param.dit_time_min*SYS_1_MINUTES_TICK;
      else if(*runflag ==2) runtimeS = u_sys_param.sys_config_param.clean_time_min*SYS_1_MINUTES_TICK;
      cleanTimer02Handle = osTimerNew(cleanWaterCallback02, osTimerOnce, NULL, &cleanTimer02_attributes);
      if(osTimerIsRunning(cleanTimer02Handle)==pdFALSE) osTimerStart(cleanTimer02Handle,runtimeS);      
      tmc2226_start(TMC_WATER_OUT_DIR_VALUE,3,CONTINUOUS_STEPS_COUNT);        
    }   
    else
    { 
      if(osTimerIsRunning(cleanTimer02Handle)==pdTRUE) 
      {
        osTimerStop(cleanTimer02Handle);
        tmc2226_stop(); 
        osTimerDelete(cleanTimer02Handle);
      }
    }   
 }
/***************************************************************************//**
 * @brief 数据包检查
 * @param 
 * @note  最小包长8字节
 * @return 
*******************************************************************************/
unsigned short int app_hmi_package_check(unsigned char* pBuff,unsigned short int buffLen) 
{
	unsigned short int retLen = 0;
    unsigned short i = 0;

    while (i < buffLen)
    {
        // need at least 3 bytes to read header+length
        if (buffLen < i+3)
        {
            retLen = i;
            break;
        }
        if (pBuff[i] == 0x7E && pBuff[i + 1] == 0x7E)
        {
            unsigned short pLen = pBuff[i + 2];
            // basic sanity: minimum packet length (protocol dependent). Use 8 as previous comment suggested.
            if (pLen >= 8)
            {
                // if full packet not yet received, wait for more data
                if ((unsigned int)i + (unsigned int)pLen <= (unsigned int)buffLen)
                {
                    // safe to read CRC bytes now
                    unsigned short crc_read = (unsigned short)pBuff[i + pLen - 4] | ((unsigned short)pBuff[i + pLen - 3] << 8);
                    if (crc_read == crc16Num(pBuff + i, pLen - 4))
                    {
                        HMI_Parse_Data(&pBuff[i], pLen);
                        U_CAN_TX_MSG u_msg;
                        u_msg.msg.typeCode=0;//busy
                        osMessageQueuePut(canTxQueue05Handle,u_msg.data,0,0);
                        retLen = pLen + i;
                        i += pLen;
                    }
                    else
                    {
                        i++;
                    }
                }
                else
                {
                  retLen = i;
                  break;
                }
            }
            else
            {
              i++;
            }
        }
        else
        {
            i++;
        }
    }

    if (i == buffLen)
    {
        retLen = i;
    }
    return retLen;
}
/************************************************************************//**
* @brief 蜂鸣器PWM
* @param delayPai节拍
* @note  
* @retval  
*****************************************************************************/
 void app_beep_pai_tim(unsigned char delayPai)
 {
    //1s一个音符，0.25秒一个节拍
    unsigned  int dp;
    dp = (delayPai-30)*75;//125 ; 
    osDelay(dp);
 }
/************************************************************************//**
* @brief 蜂鸣器music
* @param volnum 音量0~100；
* @note  
* @retval  
*****************************************************************************/
 void app_buzz_music(music_type  music_num,unsigned char volume)
 {
   unsigned int j, vol;  
   vol=1;//1低，2，中，3高
   if(music_num==MUSIC_SHORT_PROMT)
   {
      app_beep_pwm(music_tab_c[1],50);
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
      osDelay(300);     
      app_beep_pwm(0,0);
      osDelay(300);    
   }
   else if(music_num==MUSIC_LONG_PROMT)
   {
      app_beep_pwm(music_tab_c[16],50);
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1); 
      osDelay(1500);     
      app_beep_pwm(0,0);   
      osDelay(500); 
   }    
   else if(music_num==MUSIC_TWO_TIGER)
   {
      vol=1;
      app_beep_pwm(50,50);
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1); 
      for(j=0;j<32;j++)     
      {     
        app_beep_pwm(music_tab_c[music_two_tiger[j*2]*vol],50);
        app_beep_pai_tim(music_two_tiger[2*j+1]);	
      } 
      app_beep_pwm(0,50);   
   }
   else if(music_num==MUSIC_STAR)
   {   
      vol=1;
      app_beep_pwm(50,50);
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
      for(j=0;j<42;j++)     
      { 
        app_beep_pwm(music_tab_c[music_star[j*2]*vol],50);
        app_beep_pai_tim(music_star[2*j+1]);	
      }  
      app_beep_pwm(0,0);      
   }
   else if(music_num==MUSIC_HAPPY)
   {
      vol=2;      
      app_beep_pwm(50,50);
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
      for(j=0;j<48;j++)     
      { 
        app_beep_pwm(music_tab_c[music_happy[2*j]*vol],50);
        app_beep_pai_tim(music_happy[2*j+1]);	
      } 
      app_beep_pwm(0,0);       
   }
   else if(music_num==MUSIC_SYS_ON)
   {  
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);   
      app_beep_pwm(music_tab_c[6],50);
      osDelay(300);
      app_beep_pwm(music_tab_c[16],50);
      osDelay(300);
      app_beep_pwm(music_tab_c[21],50);
      osDelay(400);     
      app_beep_pwm(0,0);
   }
   else if(music_num==MUSIC_LASER_WORK)
   {
      if(laser_ctr_param.beep!=0)
      {
        HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);   
        app_beep_pwm(music_tab_c[16],50);
        osDelay(200);     
        app_beep_pwm(0,0);        
      }      
   }
   else app_beep_pwm(0,0);
 }
 /************************************************************************//**
  * @brief 循环水水加热管理  water 
  * @param circleWaterTmprature 冷却液温度
  * @note    冷却水需维持22~26摄氏度，最佳25℃, 1秒调用一次
  * @retval None
  *****************************************************************************/
void app_circle_water_PTC_manage(float circleWaterTmprature,unsigned  int sysTimeMs)
{
  static unsigned int local_TimeMs;
  float compareTemp=0; 
  osStatus_t m_status,can_tx_sta;
  U_CAN_TX_MSG u_pt_msg;
  uint8_t cmd[2]; 
  if(sysTimeMs<local_TimeMs)  local_TimeMs=sysTimeMs;
  if(sysTimeMs< local_TimeMs+500)  return;
  if((u_s_l980.sta.staByte&L980_STA_HEART_BIT0)!=L980_STA_HEART_BIT0) return;
  local_TimeMs=sysTimeMs;
  compareTemp=circleWaterTmprature-(u_sys_param.sys_config_param.cool_temprature_target*0.1);  
  //target temprature   
  if((u_s_l980.sta.staByte&L980_STA_PROHOT_BIT1)==L980_STA_PROHOT_BIT1&&u_s_l980.sta.tec_switch==0)
  { 
    if(u_sys_param.sys_config_param.cool_temprature_target!=u_l980.set_param.targetTempratureSet)
    {
      u_pt_msg.msg.typeCode=L980_REG_LASER_TEMPRATURE|L980_REG_WRITE_MASK;
      u_pt_msg.msg.dataLen=2;
      u_pt_msg.msg.buff[0]=u_sys_param.sys_config_param.cool_temprature_target&0xFF;
      u_pt_msg.msg.buff[1]=(u_sys_param.sys_config_param.cool_temprature_target>>8)&0xFF;            
      can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_pt_msg.data,0,0);
      if(can_tx_sta!=osOK)   
      {
        DEBUG_PRINTF("L980 set target temprature fail ,resend once!\r\n");
        osDelay(L980_CAN_MINI_TIME_MS);
        osMessageQueuePut(canTxQueue05Handle,u_pt_msg.data,0,0);
      }
      else DEBUG_PRINTF("L980 set tec target temprature=%.1f!tec on\r\n",u_sys_param.sys_config_param.cool_temprature_target*0.1);
    }
    //tec en       
    u_pt_msg.msg.typeCode=L980_REG_TEC_CTR|L980_REG_WRITE_MASK;
    u_pt_msg.msg.dataLen=2;
    u_pt_msg.msg.buff[0]=1;
    u_pt_msg.msg.buff[1]=0;            
    can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_pt_msg.data,0,0);
    if(can_tx_sta!=osOK)   
    {
      DEBUG_PRINTF("L980 tec on fail ,resend once!\r\n");
      osDelay(L980_CAN_MINI_TIME_MS);
      osMessageQueuePut(canTxQueue05Handle,u_pt_msg.data,0,0);
    }
    else DEBUG_PRINTF("L980 tec on\r\n");  
  }    
  else  
  {
    if(u_sys_param.sys_config_param.tec_switch==0&&u_s_l980.sta.tec_switch!=0) 
    {
      u_pt_msg.msg.typeCode=L980_REG_TEC_CTR|L980_REG_WRITE_MASK;
      u_pt_msg.msg.dataLen=2;
      u_pt_msg.msg.buff[0]=0;
      u_pt_msg.msg.buff[1]=0;            
      can_tx_sta = osMessageQueuePut(canTxQueue05Handle,u_pt_msg.data,0,0);
      if(can_tx_sta!=osOK)   
      {
        DEBUG_PRINTF("L980 tec off fail ,resend once!\r\n");
        osDelay(L980_CAN_MINI_TIME_MS);
        osMessageQueuePut(canTxQueue05Handle,u_pt_msg.data,0,0);
      }
      else DEBUG_PRINTF("L980 tec off\r\n"); 
    }
        
  }          
}
/************************************************************************//**
* @brief 激光能量反馈计算
* @param 
* @note  0.75mA/mW
* @retval  能量值
*****************************************************************************/
unsigned short int app_energe_cali( unsigned int adVoltage)
{
  static unsigned short int e_value;
  sEnvParam.laser_1064_energy = e_value;    
  return e_value;
}
/************************************************************************//**
  * @brief app_p2000w_status_handle
  * @param P_2000W_STATUS 状态数据
  * @note  
  * @retval    
  *****************************************************************************/
 void app_p2000w_status_handle( P_2000W_STATUS *pSta)
 {
    unsigned char errCode=(pSta->error_code&P2000W_ERROR_CODE_MASK);    
    if((P2000W_ERROR_LLC_OVERLOAD&errCode)==P2000W_ERROR_LLC_OVERLOAD)
    {
      DEBUG_PRINTF("p2000w LLC overload,stop laser 1064 out \r\n");      
      if(sGenSta.laser_run_B0_pro_hot_status!= 0) osSemaphoreRelease(laserCloseSem05Handle);
    }
    if((P2000W_ERROR_PRO_HOT&errCode)==P2000W_ERROR_PRO_HOT)
    {     
      pSta->ctr_status&=(~P2000W_STA_B3_PRO_HOT_OK);
      DEBUG_PRINTF("p2000w prohot ERROR\r\n");      
    }   
    if((P2000W_ERROR_TEMPRATURE_OVERLOAD&errCode)==P2000W_ERROR_TEMPRATURE_OVERLOAD)
    {
      DEBUG_PRINTF("p2000w temprature High,stop laser 1064 out\r\n");       
      if(sGenSta.laser_run_B0_pro_hot_status != 0) osSemaphoreRelease(laserCloseSem05Handle);
    }  

 }
 /************************************************************************//**
  * @brief 任务状态异常
  * @param 
  * @note 
  * @retval  
  *****************************************************************************/
  void app_task_status_error_handle( osThreadId_t *thread_id )
  {    
    if(*thread_id==myTask02Handle)
    {
      IS3_init();
      //myTask02Handle = osThreadNew(auxTask02, NULL, &myTask02_attributes);
    }
    else 
    { 
      //reestart   
      osThreadTerminate(*thread_id);
      if(*thread_id==myTask03Handle)
      {
        myTask03Handle = osThreadNew(keyScanTask03, NULL, &myTask03_attributes);
      }
      else if(*thread_id==myTask04Handle)
      {
        myTask04Handle = osThreadNew(laserWorkTask04, NULL, &myTask04_attributes);
      }
      else if(*thread_id==myTask05Handle)
      {
        myTask05Handle = osThreadNew(fastAuxTask05, NULL, &myTask05_attributes);
      }
      else if(*thread_id==myTask06Handle)
      {
        myTask06Handle = osThreadNew(hmiAppTask06, NULL, &myTask06_attributes);
      }
      else if(*thread_id==myTask07Handle)
      {
        myTask07Handle = osThreadNew(canReceiveTask07, NULL, &myTask07_attributes);
      }
      else if(*thread_id==myTask08Handle)
      {
        myTask08Handle = osThreadNew(powerOffTask08, NULL, &myTask08_attributes);
      }
      else if(*thread_id==myTask09Handle)
      {
        myTask09Handle = osThreadNew(laserProhotTask09, NULL, &myTask09_attributes);
      }
      else if(*thread_id==myTask10Handle)
      {
        myTask10Handle = osThreadNew(ge2117ManageTask10, NULL, &myTask10_attributes);
      }
      else if(*thread_id==myTask11Handle)
      {
        myTask11Handle = osThreadNew(musicTask11, NULL, &myTask11_attributes);
      }
      else if(*thread_id==myTask12Handle)
      {
        myTask12Handle = osThreadNew(p2000wReceiveTask12, NULL, &myTask12_attributes);
      }
    }
  }
  /************************************************************************//**
  * @brief app_p2000w_pulse_auto_adjust_voltage
  * @param targetEnerge 目标能量，voltageRange电压调整范围,realEnerge测量能量值
  * @note  
  * @retval    
  *****************************************************************************/
 void app_p2000w_pulse_auto_adjust_voltage( unsigned short int targetEnerge,unsigned short int realEnerge,unsigned short int voltageSet)
 {
  U_P2000W_TX_MSG p_send_msg;
  osStatus_t p_mtxs;
  static  short int cali_volatage=0;
  p_send_msg.msg.code=P2000W_CODE_VOLTAGE_SET;
  if(targetEnerge>200) targetEnerge=200;  
  if(realEnerge+5<targetEnerge||realEnerge>5+targetEnerge){      
    if(realEnerge+5<targetEnerge)    
    {
      if(cali_volatage<100){
        cali_volatage+=2;
        if(realEnerge+20<targetEnerge) cali_volatage+=15; 
      }    
      if(cali_volatage<0) p_send_msg.msg.cmd=voltageSet-abs(cali_volatage);
      else p_send_msg.msg.cmd=voltageSet+abs(cali_volatage);     
    }
    else if(realEnerge>5+targetEnerge){      
     if(cali_volatage>-100){
      cali_volatage-=2;
      if(realEnerge>targetEnerge+20) cali_volatage-=15;//能量过低时加快校准速度 
     } 
     if(cali_volatage<0) p_send_msg.msg.cmd = voltageSet-abs(cali_volatage);     
     else p_send_msg.msg.cmd=voltageSet+abs(cali_volatage);     
    }
    p_mtxs= osMessageQueuePut(p2000wTxMessageQueue04Handle,p_send_msg.data,0,P2000W_FRAME_DELAY_TIME);
    if(p_mtxs!=osOK) 
    {
      DEBUG_PRINTF("set test voltage fial ,resend once!\r\n");
      osDelay(P2000W_FRAME_DELAY_TIME);
      osMessageQueuePut(p2000wTxMessageQueue04Handle,p_send_msg.data,0,0);
    }
    else DEBUG_PRINTF("set  cali voltage=%dV\r\n", p_send_msg.msg.cmd);   
  }
 }

  /************************************************************************//**
  * @brief 内存使用状态
  * @param 
  * @note 
  * @retval  
  *****************************************************************************/
 void app_sram_status_monitor( void )
 {
  osThreadState_t taskState; 
  uint32_t taskSize,taskSpace,sumtaskSize=0,sumtaskSpace=0;  
  taskState=osThreadGetState(defaultTaskHandle);
  taskSize= defaultTask_attributes.stack_size ;
  taskSpace=osThreadGetStackSpace(defaultTaskHandle); 
  //DEBUG_PRINTF("defaultTask s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  //单次任务用后删除
    //if( taskSpace<50) DEBUG_PRINTF("out of memory task1 %d!\r\n",taskSpace);
    //app_task_status_error_handle(&defaultTaskHandle );
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState=osThreadGetState(myTask02Handle);
  taskSize= myTask02_attributes.stack_size ;
  taskSpace=osThreadGetStackSpace(myTask02Handle); 
 //DEBUG_PRINTF("myTask02 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {      
    if( taskSpace<50) DEBUG_PRINTF("out of memory task2 %d!\r\n",taskSpace);
    app_task_status_error_handle(&myTask02Handle);
  }  
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState=osThreadGetState(myTask03Handle);
  taskSize= myTask03_attributes.stack_size ;
  taskSpace=osThreadGetStackSpace(myTask03Handle); 
 // DEBUG_PRINTF("myTask03 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  
    if( taskSpace<50) DEBUG_PRINTF("out of memory task3!\r\n");
    app_task_status_error_handle( &myTask02Handle);
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState=osThreadGetState(myTask04Handle);
  taskSize= myTask04_attributes.stack_size ;
  taskSpace=osThreadGetStackSpace(myTask04Handle); 
 // DEBUG_PRINTF("myTask04 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  
    if( taskSpace<50) DEBUG_PRINTF("out of memory task4!\r\n");
    app_task_status_error_handle( &myTask03Handle);
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState=osThreadGetState(myTask05Handle);
  taskSize= myTask05_attributes.stack_size;
  taskSpace=osThreadGetStackSpace(myTask04Handle); 
  //DEBUG_PRINTF("myTask05 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  
    if( taskSpace<50) DEBUG_PRINTF("out of memory task5!\r\n");
    app_task_status_error_handle( &myTask05Handle);
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState=osThreadGetState(myTask06Handle);
  taskSize= myTask06_attributes.stack_size ;
  taskSpace=osThreadGetStackSpace(myTask06Handle); 
  //DEBUG_PRINTF("myTask06 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  
    if( taskSpace<50) DEBUG_PRINTF("out of memory task6!\r\n");
    app_task_status_error_handle( &myTask06Handle);
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState=osThreadGetState(myTask07Handle);
  taskSize= myTask07_attributes.stack_size ;
  taskSpace=osThreadGetStackSpace(myTask07Handle); 
  //DEBUG_PRINTF("myTask07 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  
    if( taskSpace<50) DEBUG_PRINTF(" out of memory task7!\r\n");
    app_task_status_error_handle( &myTask07Handle);
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState=osThreadGetState(myTask08Handle);
  taskSize= myTask08_attributes.stack_size;
  taskSpace=osThreadGetStackSpace(myTask08Handle); 
 // DEBUG_PRINTF("myTask08 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  
    if(taskSpace<50) DEBUG_PRINTF(" out of memory task8!\r\n");
    app_task_status_error_handle(&myTask08Handle);
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState=osThreadGetState(myTask09Handle);  
  taskSize= myTask09_attributes.stack_size;
  taskSpace=osThreadGetStackSpace(myTask09Handle); 
 //DEBUG_PRINTF("myTask09 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  
    if(taskSpace<50) DEBUG_PRINTF(" out of memory task9!\r\n");
    app_task_status_error_handle( &myTask09Handle);
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState=osThreadGetState(myTask10Handle);
  taskSize= myTask10_attributes.stack_size ;
  taskSpace=osThreadGetStackSpace(myTask10Handle); 
  //DEBUG_PRINTF("myTask10 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  
    if(taskSpace<50) DEBUG_PRINTF(" out of memory task10!\r\n");
    app_task_status_error_handle( &myTask10Handle);
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState=osThreadGetState(myTask11Handle);
  taskSize= myTask11_attributes.stack_size;
  taskSpace=osThreadGetStackSpace(myTask11Handle); 
  //DEBUG_PRINTF("myTask11 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  
    if(taskSpace<50) DEBUG_PRINTF(" out of memory task11!\r\n");
    app_task_status_error_handle(&myTask11Handle);
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  taskState = osThreadGetState(myTask12Handle);
  taskSize = myTask12_attributes.stack_size;
  taskSpace = osThreadGetStackSpace(myTask12Handle); 
  //DEBUG_PRINTF("myTask12 s=%d Size=%d Space=%d \r\n",taskState,taskSize,taskSize-taskSpace);
  if(taskState==osThreadError||taskSpace<50)
  {  
    if(taskSpace<50) DEBUG_PRINTF("out of memory task12!\r\n");
    app_task_status_error_handle(&myTask11Handle);
  }
  sumtaskSize+=taskSize;
  sumtaskSpace+=taskSpace;
  //DEBUG_PRINTF("all cpuAll=%d cpuUse=%d =%d\r\n",sumtaskSize,sumtaskSize-sumtaskSpace,(sumtaskSize-sumtaskSpace)*100/sumtaskSize);
 }
/* USER CODE END Application */

