/*******************************************************************************
*                     ��Ȩ���� (C), 2017-,NCUROBOT
************************************************************************************************************************************************************
* �� �� ��   : Motor_USE_TIM.c
* �� �� ��   : ����
* ��    ��   : NCURM
* ��������   : 2018��7��
* ����޸�   :
* ��������   : �����ģ����ʹ��TIM���п��Ƶĵ��
* �����б�   :
*
*							Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo)

*							Friction_Wheel_Motor_Stop(void)
*					
	******************************************************************************
*/
#include "Motor_USE_TIM.h"
#include "pid.h"

uint16_t mc_count[2]={0,0};

void TIM1_PWM_Init(uint32_t speed1,uint32_t speed2)
{
	  TIM1->CCR1=speed1;
	  TIM1->CCR2=speed1;
//	  TIM1->CCR3=speed1;
//	  TIM1->CCR4=speed1;
}

/**
	**************************************************************
	** Descriptions: Ħ���ֵ����ʼ������
	** Input:  NULL	
	** Output: NULL
	**************************************************************
**/
void GUN_Init(void)
{
	__HAL_TIM_ENABLE(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);   //PE9,�Ҳ�Ħ����
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);   //PE11,���Ħ����
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);   //PH12,��౸��
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);   //PH12,��౸��
	
	HAL_Delay(100);
  TIM1->CCR1 = 1050;
	HAL_Delay(3000);
	TIM1->CCR1 = 1100;
	HAL_Delay(2000);
	TIM1->CCR1= 1050;
  HAL_Delay(1000);
	
	TIM1->CCR2 = 1050;
	HAL_Delay(3000);
	TIM1->CCR2 = 1100;
	HAL_Delay(2000);
	TIM1->CCR2= 1050;
//	HAL_Delay(3000);
//	TIM1->CCR3 = 1500;
//	HAL_Delay(3000);
//	TIM1->CCR3 = 1000;
//	HAL_Delay(2000);
//	TIM1->CCR3= 1050;
//	HAL_Delay(2000);


//	TIM1->CCR4 = 1050;
//	HAL_Delay(3000);
//	TIM1->CCR4 = 1100;
//	HAL_Delay(2000);
//	TIM1->CCR4= 1300;
//	
  HAL_Delay(100);
}

/**
	**************************************************************
	** Descriptions: Ħ���ֵ����������
	** Input: 	
	**	����ת��:
	**					wheelone
	**					wheeltwo
	** Output: NULL
	**************************************************************
**/

	
 void Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo)
{
  TIM1_PWM_Init(wheelone,wheeltwo);
}

void Friction_Wheel_Motor_Stop(void)
{
	
	htim1.Instance->CR1 &= ~(0x01);  //�رն�ʱ��
		
}

void Maichong_Count(uint8_t i)
{
	
	if(i == 0)
	mc_count[0]++;
	
	if(i == 1)
	mc_count[1]++;
	
	
}

