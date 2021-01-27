#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "string.h"


#define SizeofReferee 100
#define SizeofRemote 18
#define SizeofMinipc  9
extern uint8_t USART3_RX_DATA[(SizeofRemote)];//遥控
extern uint16_t USART3_RX_NUM;
extern uint8_t USART6_RX_DATA[(SizeofReferee)];//裁判系统
extern uint16_t USART6_RX_NUM;


/* 本模块向外部提供的数据类型定义 --------------------------------------------*/

///////////////遥控/////////////////////
typedef struct //遥控器及键鼠通道
{ 
	int16_t x; //!< Byte 6-7 
	int16_t y; //!< Byte 8-9 
	int16_t z; //!< Byte 10-11 
	uint8_t press_l; //!< Byte 12 
	uint8_t press_r; //!< Byte 13 
}Mouse; 
typedef 	struct 
{ 
  uint16_t ch0; 
  uint16_t ch1; 
  uint16_t ch2; 
  uint16_t ch3; 
  uint8_t s1; 
  uint8_t s2; 
}Rc; 
typedef struct 
{ 
	uint16_t v; //!< Byte 14-15 
}Key; 
	typedef struct 
{ 
  Rc rc; 
  Mouse mouse; 
  Key key; 
}RC_Ctl_t; 
////////////////遥控/////////////////////


/*******************云台状态标志位******************************/
typedef struct
{
	uint8_t gimbal_mode;
	uint8_t gimbal_flag;
	uint8_t remote_mode;
	uint8_t minipc_mode;
	uint8_t minipc_color;   //视觉需要的颜色信息
	uint8_t bopan_mode;
	uint8_t bopan_lastmode;
	uint8_t bopan_flag;     //拨盘反转标志位
}Gimbal_Status_t;


/*******************mpu6500*********************************/
typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
	
  int16_t last_gx;
  int16_t last_gy;
  int16_t last_gz; 
	
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;


/* 本模块向外部提供的宏定义 --------------------------------------------------*/

/* 本模块向外部提供的接口常量声明 --------------------------------------------*/
/**************外接陀螺仪*******************/

/*****************mpu6500*****************/
extern uint8_t MPU_id;
extern IMUDataTypedef imu_data;
extern IMUDataTypedef imu_data_offest;
/****************遥控********************/
extern RC_Ctl_t RC_Ctl; //遥控数据

/* 本模块向外部提供的接口函数原型声明 ----------------------------------------*/
//外接陀螺仪

//遥控
void Remote_Ctrl(void);
void Remote_Disable(void);
//mpu6500

//////裁判系统
////void DataVerify(void);
////// 使用方法 发送用户数据 Send_FrameData(SelfDefinedDara, userMessage,tSelfDefineInfo(userMessage)); 
////void Send_FrameData(tCmdID cmdid, uint8_t * pchMessage,uint8_t dwLength); 
////void sendata(void);

#endif
