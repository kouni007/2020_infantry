#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H
#ifdef __cplusplus
 extern "C" {
#endif 
/* 包含头文件----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h" 
#include "pid.h"
#include "gimbal_control.h"	 
#include "mecanum_calc.h"
#include "Motor_USE_CAN.h"
#include "data_processing.h"
#include "user_lib.h"
#include "arm_math.h"
/* 本模块向外部提供的数据类型定义--------------------------------------------*/	
typedef struct
{
	float dstVmmps_Y;
	float dstVmmps_X;
	float dstVmmps_W;
	char  flag;
}moto3508_type;

typedef struct
{
	uint8_t chassis_mode;
	uint8_t chassis_flag;
}Chassis_Status_t;
/* 本模块向外部提供的宏定义--------------------------------------------------*/
#define CHASSIS_PERIOD 5

#define Middle_Angle 3600           //根据实际车的中间位置的yaw轴反馈角度来定

#define Degree45_Angle  4621

/* 本模块向外部提供的接口常量声明--------------------------------------------*/
extern moto3508_type  moto_3508_set; 
extern uint32_t total_current;
extern Chassis_Status_t chassis_status;
/* 本模块向外部提供的接口函数原型声明----------------------------------------*/
void Chassis_Task(void const * argument);
float Absolute_angle_Conversion(int32_t angle);
/* 全局配置区----------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif 
