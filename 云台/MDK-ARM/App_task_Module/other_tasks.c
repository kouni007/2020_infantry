#include "other_tasks.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "protocol.h"
#include "gimbal_control.h"
/* �ڲ�����------------------------------------------------------------------*/
uint32_t test_i = 0;
/* �ڲ�����ԭ������----------------------------------------------------------*/

/* �������岿�� -------------------------------------------------------------*/
void testTask(void const * argument)
{
	int16_t angle[5];	
	osDelay(200);//��ʱ200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
		float pSinVal;
	float pCosVal;
	for(;;)
	{ 				
		RefreshTaskOutLineTime(testTask_ON);
		arm_sin_cos_f32(35, &pSinVal, &pCosVal);
    test_i++;
		int16_t  *ptr = angle; //��ʼ��ָ��
		angle[0] = yaw_set.expect;  
		angle[1] =  yaw_get.total_angle;
		angle[2] =	pit_get.total_angle;
		angle[3] = 	pit_set.expect;
		angle[4] =  pid_pit.pos_out;
		/*������ʾ��������������*/
		vcan_sendware((uint8_t *)ptr,5 * sizeof(angle[0]));
//		 HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);	
//		 HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);	
//		 HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);	
		
		  osDelayUntil(&xLastWakeTime,20);
	}
}
