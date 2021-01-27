#ifndef __GUN_CONTROL_H
#define __GUN_CONTROL_H
/* 包含头文件----------------------------------------------------------------*/
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "Motor_USE_CAN.h"
#include "Motor_USE_TIM.h"
#include "pid.h"
#include "minipc.h"

/* 本模块向外部提供的数据类型定义--------------------------------------------*/

typedef struct 
{
	uint8_t remote_mode;       //操作手指定的发射模式
	uint8_t limit_flag;              //弹丸位置变化标志位
  uint8_t limit_level;       //限位开关电平，压限位开关为高电平，为1，反之，为0
	uint8_t last_limit_level;  //上次的限位开关电平
	uint16_t cnt;              //弹丸计数值，发射一颗弹丸加二
	uint16_t last_cnt;         //上次的弹丸计数值
	uint16_t danfa_cnt;        //单发时的弹丸计数值
	float val;
  uint16_t heat_forecast;
 }Shoot_status_t;

 
typedef struct Heat_Gun_t
{
	int16_t  shted_bullet;
	int16_t limt_bullet;
	int16_t last_limt_bullet;
	uint16_t limt_heat;
	uint16_t rel_heat;
	uint16_t last_rel_heat;
	float    remain_power;
	uint8_t  limt_spd;
	uint8_t  roboLevel;
	uint8_t  sht_flg;
	uint8_t  stop_flg;
	uint8_t  heat_down_flg;
}Heat_Gun_t;

volatile typedef struct 
{
	volatile uint16_t rel_heat;
	volatile float remain_power;
}Power_Heat;

/* 本模块向外部提供的宏定义--------------------------------------------------*/
#define GUN_PERIOD  10
#define Mocha_PERIOD  10
#define BLOCK_TIME 4000
#define REVERSE_TIME 2000
#define DELAY_TIME  100

#define Limit_switch_Control 0      //0为无限位开关控制，1为有限位开关控制

#define Mocha_Stop         0
#define Mocha_danfa_mode   1
#define Mocha_Duofa_mode   2
#define Mocha_Lianfa_mode  3

#define bopan_Stop          0
#define bopan_danfa_mode    1
#define bopan_Duofa_mode    2
#define bopan_Lianfa_mode   3
#define bopan_shangdan_mode 4
#define bopan_guodu_mode    5
#define bopan_fanzhuan_mode 10
/* 本模块向外部提供的接口常量声明--------------------------------------------*/
extern volatile Power_Heat * ptr_power_heat;
extern Heat_Gun_t  ptr_heat_gun_t;
extern volatile float remain_power;
extern Shoot_status_t Shoot_status;
extern int32_t set_speed;
extern uint8_t Mocha_flag;
extern uint8_t Magazine_flag;
/* 本模块向外部提供的接口函数原型声明----------------------------------------*/
void Gun_Task(void const * argument);
void Mocha_Task(void const * argument);
void Shoot_Heat_Task(void const *argument);
void bopan_check( uint8_t  Set_cnt );
void Moto_dial_Angle_Reset(void);
/* 全局配置区----------------------------------------------------------------*/

#endif
