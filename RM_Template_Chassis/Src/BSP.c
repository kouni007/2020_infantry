#include "BSP.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "Power_restriction.h"

volatile unsigned long long FreeRTOSRunTimeTicks;
uint32_t  uhADCxConvertedValue[10];
/**
	**************************************************************
	** Descriptions:	JY61����/������
	** Input:	huart  ����ָ��Ĵ��ڣ�������Ҫ��Ϊ115200
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void JY61_SLEEPorUNSLEEP(UART_HandleTypeDef *huart)
{
	uint8_t buff[3] = {0xff,0xaa,0x60};
	//����,������
	HAL_UART_Transmit(huart,buff,3,10);
}

/**
	**************************************************************
	** Descriptions: JY61֡���뺯��
	** Input: 	
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void JY61_Frame(void)
{

	
}



void JY901_Init(void)
{
	
}




void BSP_Init(void)
{
	MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM13_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
	
	/*CAN������*/
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	
	/*����CAN��ADC*/
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
//	HAL_ADC_Start(&hadc1);           //�˺��������󣬻������� HAL_ADC_Start_DMA �������ͻ������DMAֻ����һ�Ρ�����ԭ����δ�ҵ�
	
	/*���߼��*/
	SystemState_Inite();
	Bsp_UART_Receive_IT(&huart2,USART6_RX_DATA,100);	
	Bsp_UART_Receive_IT(&huart1,Recive_103,12);	

	/*ʹ��can�ж�*/
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uhADCxConvertedValue, 10); 
}
