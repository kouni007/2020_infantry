/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gimbal_control.h"
#include "Power_restriction.h"
#include "SystemState.h"
#include "user_lib.h"

/* �ڲ�����------------------------------------------------------------------*/

Pos_Set  yaw_set={0};
Pos_Set  yaw_tly_set={0};
Pos_Set  pit_set={0};
Mode_Set Gimbal={0};
Mode_Set Minipc={0};
Mode_Set Shoot={0};

pid_t pid_yaw       = {0};  //yaw��λ�û�
pid_t pid_yaw_jy901 = {0};  //��������� /*Ŀǰֻ����λ�û�*/ 
pid_t pid_pit       = {0};	//pit��λ�û�
pid_t pid_pit_dashen = {0};       //���������
pid_t pid_pit_dashen_spd = {0};
pid_t pid_yaw_spd   = {0};	//yaw���ٶȻ�
pid_t pid_pit_spd   = {0};	//pit���ٶȻ�
pid_t pid_yaw_jy901_spd = {0};
pid_t pid_pit_jy901 = {0};
pid_t pid_pit_jy901_spd = {0};

pid_t pid_yaw_saber = {0};  //��������� /*Ŀǰֻ����λ�û�*/
pid_t pid_yaw_saber_spd = {0};
pid_t pid_pit_saber = {0};
pid_t pid_pit_saber_spd = {0};

//zimiao
pid_t pid_yaw_zimiao = {0};        //
pid_t pid_yaw_zimiao_spd = {0};
pid_t pid_pit_zimiao = {0};        //
pid_t pid_pit_zimiao_spd = {0};
/* �ڲ�����ԭ������----------------------------------------------------------*/
/**                                                           //����
	**************************************************************
	** Descriptions: ��̨pid��ʼ��
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/


/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	��̨�������
	*	@retval	
****************************************************************************************/
void Gimbal_Task(void const * argument)
{
	
	osDelay(200);//��ʱ200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)		
  {
			
	    RefreshTaskOutLineTime(GimbalContrlTask_ON);
      
			
		  osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
  }
 
}
