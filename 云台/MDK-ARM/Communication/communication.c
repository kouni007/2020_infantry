/*******************************************************************************
                      版权所有 (C), 2017-, Mushiny
 *******************************************************************************
  文 件 名   : communication.c
  版 本 号   : 初稿
  作    者   : liyifeng
  生成日期   : 2018年7月
  最近修改   :
  功能描述   : 串口通信库
  函数列表   :
*******************************************************************************/
/* 包含头文件 ----------------------------------------------------------------*/
#include "communication.h "
#include "Power_restriction.h"


/* 内部自定义数据类型 --------------------------------------------------------*/
 uint8_t USART3_RX_DATA[(SizeofRemote)];//遥控
 uint16_t USART3_RX_NUM;
uint8_t USART6_RX_DATA[(SizeofReferee)];//裁判系统
 uint16_t USART6_RX_NUM;


/* 内部宏定义 ----------------------------------------------------------------*/

/* 任务相关信息定义-----------------------------------------------------------*/

/* 内部常量定义---------------------------------------------------------------*/

/* 外部变量声明 --------------------------------------------------------------*/

/* 外部函数原型声明 ----------------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/

//mpu6500
uint8_t MPU_id = 0;
int16_t gy_data_filter[5];
int16_t gz_data_filter[5];
RC_Ctl_t RC_Ctl; //遥控数据
IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

/* 内部函数原型声明 ----------------------------------------------------------*/

/* 任务主体部分 --------------------------------------------------------------*/



/***************************************************************************************
**
	*	@brief	Remote_Ctrl(void)
	*	@param
	*	@supplement	使用串口接收遥控器数据
	*	@retval	
****************************************************************************************/
void Remote_Ctrl(void)//遥控数据接收
{ 
	 uint8_t  * buff = USART3_RX_DATA;
	 RC_Ctl.rc.ch0 = (buff[0]| (buff[1] << 8)) & 0x07ff; //!< Channel 0 
	 RC_Ctl.rc.ch1 = ((buff[1] >> 3) | (buff[2] << 5)) & 0x07ff; //!< Channel 1 
	 RC_Ctl.rc.ch2 = ((buff[2] >> 6) | (buff[3] << 2) | //!< Channel 2 
	 (buff[4] << 10)) & 0x07ff; 
	 RC_Ctl.rc.ch3 = ((buff[4] >> 1) | (buff[5] << 7)) & 0x07ff; //!< Channel 3 
	 RC_Ctl.rc.s1 = ((buff[5] >> 4)& 0x000C) >> 2; //!< Switch left 
	 RC_Ctl.rc.s2 = ((buff[5] >> 4)& 0x0003); //!< Switch right 
	 RC_Ctl.mouse.x = buff[6] | (buff[7] << 8); //!< Mouse X axis 
	 RC_Ctl.mouse.y = buff[8] | (buff[9] << 8); //!< Mouse Y axis 
	 RC_Ctl.mouse.z = buff[10] | (buff[11] << 8); //!< Mouse Z axis 
	 RC_Ctl.mouse.press_l = buff[12]; //!< Mouse Left Is Press ? 
	 RC_Ctl.mouse.press_r = buff[13]; //!< Mouse Right Is Press ? 
	 RC_Ctl.key.v = buff[14] | (buff[15] << 8); //!< KeyBoard value 
}

/***************************************************************************************
**
	*	@brief	Remote_Ctrl(void)
	*	@param
	*	@supplement	使用串口接收遥控器数据
	*	@retval	
****************************************************************************************/
void Remote_Disable(void)//遥控数据接收
{ 

	 RC_Ctl.rc.ch0 = 0; //!< Channel 0 
	 RC_Ctl.rc.ch1 = 0; //!< Channel 1 
	 RC_Ctl.rc.ch2 = 0; 
	 RC_Ctl.rc.ch3 = 0; //!< Channel 3 
	 RC_Ctl.rc.s1 = 0; //!< Switch left 
	 RC_Ctl.rc.s2 = 0; //!< Switch right 
	 RC_Ctl.mouse.x = 0; //!< Mouse X axis 
	 RC_Ctl.mouse.y = 0; //!< Mouse Y axis 
	 RC_Ctl.mouse.z = 0; //!< Mouse Z axis 
	 RC_Ctl.mouse.press_l = 0; //!< Mouse Left Is Press ? 
	 RC_Ctl.mouse.press_r = 0; //!< Mouse Right Is Press ? 
	 RC_Ctl.key.v = 0; //!< KeyBoard value 
}







