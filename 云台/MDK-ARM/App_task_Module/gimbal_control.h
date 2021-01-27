#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H
/* 包含头文件----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "pid.h"
#include "communication.h "
#include "Motor_USE_CAN.h"
#include "chassis_control.h"
#include "atom_imu.h"
#include "decode.h"

/* 本模块向外部提供的宏定义--------------------------------------------------*/	 
#define GIMBAL_PERIOD 1
#define Yaw_Middle_Angle 3600
#define Pitch_Middle_Angle 3141
/* 本模块向外部提供的数据类型定义--------------------------------------------*/
typedef struct{
		int32_t expect;           //最终pid运算值
		uint8_t	step;
		uint8_t mode;
		int32_t expect_pc;        //视觉设定值
    int16_t err;
} Pos_Set;


/* 本模块向外部提供的宏定义--------------------------------------------------*/

/* 本模块向外部提供的接口常量声明--------------------------------------------*/

extern Pos_Set  yaw_tly_set;
extern Pos_Set  pit_set;
extern Pos_Set  yaw_set;
extern int8_t gimbal_disable_flg;
extern Gimbal_Status_t gimbal_status;
extern int32_t pitch_zhi ;
extern int32_t yaw_zhi ;
extern pid_t pid_pit_start ;
extern pid_t pid_pit_start_spd ;
extern pid_t pid_yaw_start ;
extern pid_t pid_yaw_start_spd ;
/* 本模块向外部提供的接口函数原型声明----------------------------------------*/
void Gimbal_Task(void const * argument);
void Gimbal_angle_Conversion(moto_measure_t *ptr);

/* 全局配置区----------------------------------------------------------------*/

#endif
