/* 包含头文件----------------------------------------------------------------*/
#include "offline_check.h"
#include "SystemState.h"
#include "gpio.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "usart.h"
#include "gimbal_control.h"

/* 内部变量------------------------------------------------------------------*/

/* 内部函数原型声明----------------------------------------------------------*/

/* 任务主体部分 -------------------------------------------------------------*/



void Led_Task(void const * argument)
{
	
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		
		RefreshTaskOutLineTime(LedTask_ON);
		

				if((SystemState.OutLine_Flag & 0x01)) //遥控器掉线判断
				{
						
//					  gimbal_status.remote_mode = 1;
						osDelayUntil(&xLastWakeTime,200);	
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);					
				}else  
				{
					
					 HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
//						gimbal_status.remote_mode=0;
						
				}
				
				if((SystemState.OutLine_Flag & 0x02))  //Yaw轴电机掉线判断
				{
						
						osDelayUntil(&xLastWakeTime,200);   
				} else  
				{

				}

				if((SystemState.OutLine_Flag & 0x04))  //Pitch轴电机掉线判断
				{
					
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{

						
				}
				if((SystemState.OutLine_Flag & 0x08))  //拨盘电机掉线判断
				{
						

						osDelayUntil(&xLastWakeTime,200);   
				} else 
        {	

					
				}
				if((SystemState.OutLine_Flag & 0x10))   //MiniPC掉线检测
				{
				
						osDelayUntil(&xLastWakeTime,200);
					  
				} else  
				{

					
				}
	  	osDelayUntil(&xLastWakeTime,LED_PERIOD); 
		
	 }
 
}



void Check_Task(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
	      if((SystemState.task_OutLine_Flag & 0x01))   //测试任务掉线判断
				{
					  printf("testTask GG \n\t");
					  osDelayUntil(&xLastWakeTime,100);
				}
				
				if((SystemState.task_OutLine_Flag & 0x02))   //遥控数据处理任务掉线判断
				{
						printf("RemoteDataTask GG \n\t");
			      Remote_Disable();
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag & 0x04))  //云台控制任务掉线判断
				{
						printf("GimbalContrlTask GG \n\t");
					  Cloud_Platform_Motor_Disable(&hcan1);
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag & 0x08))  //枪口控制任务掉线判断
				{
						printf("GunTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag & 0x10))  //LED检测任务掉线判断
				{
						printf("LedTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 


				if((SystemState.task_OutLine_Flag & 0x20))  //掉线检测任务掉线判断
				{
						printf("vOutLineCheckTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 

				osDelayUntil(&xLastWakeTime,Check_PERIOD);
			}
}


//断线检测
void vOutLineCheck_Task(void const *argument)
{

	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		RefreshTaskOutLineTime(vOutLineCheckTask_ON);
		
	  limit_check();
		TASK_Check();       //任务断线检测
		OutLine_Check();    //模块断线检测
		osDelayUntil(&xLastWakeTime,20);
		
	}
}

