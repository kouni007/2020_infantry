/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "offline_check.h"
#include "SystemState.h"
#include "gpio.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "usart.h"
#include "gimbal_control.h"

/* �ڲ�����------------------------------------------------------------------*/

/* �ڲ�����ԭ������----------------------------------------------------------*/

/* �������岿�� -------------------------------------------------------------*/



void Led_Task(void const * argument)
{
	
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		
		RefreshTaskOutLineTime(LedTask_ON);
		

				if((SystemState.OutLine_Flag & 0x01)) //ң���������ж�
				{
						
//					  gimbal_status.remote_mode = 1;
						osDelayUntil(&xLastWakeTime,200);	
            HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);					
				}else  
				{
					
					 HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
//						gimbal_status.remote_mode=0;
						
				}
				
				if((SystemState.OutLine_Flag & 0x02))  //Yaw���������ж�
				{
						
						osDelayUntil(&xLastWakeTime,200);   
				} else  
				{

				}

				if((SystemState.OutLine_Flag & 0x04))  //Pitch���������ж�
				{
					
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{

						
				}
				if((SystemState.OutLine_Flag & 0x08))  //���̵�������ж�
				{
						

						osDelayUntil(&xLastWakeTime,200);   
				} else 
        {	

					
				}
				if((SystemState.OutLine_Flag & 0x10))   //MiniPC���߼��
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
	      if((SystemState.task_OutLine_Flag & 0x01))   //������������ж�
				{
					  printf("testTask GG \n\t");
					  osDelayUntil(&xLastWakeTime,100);
				}
				
				if((SystemState.task_OutLine_Flag & 0x02))   //ң�����ݴ�����������ж�
				{
						printf("RemoteDataTask GG \n\t");
			      Remote_Disable();
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag & 0x04))  //��̨������������ж�
				{
						printf("GimbalContrlTask GG \n\t");
					  Cloud_Platform_Motor_Disable(&hcan1);
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag & 0x08))  //ǹ�ڿ�����������ж�
				{
						printf("GunTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState.task_OutLine_Flag & 0x10))  //LED�����������ж�
				{
						printf("LedTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 


				if((SystemState.task_OutLine_Flag & 0x20))  //���߼����������ж�
				{
						printf("vOutLineCheckTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 

				osDelayUntil(&xLastWakeTime,Check_PERIOD);
			}
}


//���߼��
void vOutLineCheck_Task(void const *argument)
{

	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		RefreshTaskOutLineTime(vOutLineCheckTask_ON);
		
	  limit_check();
		TASK_Check();       //������߼��
		OutLine_Check();    //ģ����߼��
		osDelayUntil(&xLastWakeTime,20);
		
	}
}

