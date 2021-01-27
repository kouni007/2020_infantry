/* 包含头文件----------------------------------------------------------------*/
#include "gimbal_control.h"
#include "Power_restriction.h"
#include "SystemState.h"
#include "user_lib.h"

/* 内部变量------------------------------------------------------------------*/

Pos_Set  yaw_set={0};
Pos_Set  yaw_tly_set={0};
Pos_Set  pit_set={0};
Mode_Set Gimbal={0};
Mode_Set Minipc={0};
Mode_Set Shoot={0};

pid_t pid_yaw       = {0};  //yaw轴位置环
pid_t pid_yaw_jy901 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_pit       = {0};	//pit轴位置环
pid_t pid_pit_dashen = {0};       //大神符参数
pid_t pid_pit_dashen_spd = {0};
pid_t pid_yaw_spd   = {0};	//yaw轴速度环
pid_t pid_pit_spd   = {0};	//pit轴速度环
pid_t pid_yaw_jy901_spd = {0};
pid_t pid_pit_jy901 = {0};
pid_t pid_pit_jy901_spd = {0};

pid_t pid_yaw_saber = {0};  //外接陀螺仪 /*目前只用于位置环*/
pid_t pid_yaw_saber_spd = {0};
pid_t pid_pit_saber = {0};
pid_t pid_pit_saber_spd = {0};

//zimiao
pid_t pid_yaw_zimiao = {0};        //
pid_t pid_yaw_zimiao_spd = {0};
pid_t pid_pit_zimiao = {0};        //
pid_t pid_pit_zimiao_spd = {0};
/* 内部函数原型声明----------------------------------------------------------*/
/**                                                           //待续
	**************************************************************
	** Descriptions: 云台pid初始化
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/


/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	云台电机控制
	*	@retval	
****************************************************************************************/
void Gimbal_Task(void const * argument)
{
	
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)		
  {
			
	    RefreshTaskOutLineTime(GimbalContrlTask_ON);
      
			
		  osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
  }
 
}
