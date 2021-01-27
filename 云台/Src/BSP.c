#include "BSP.h"

void BSP_Init(void)
{
	MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
	delay_init();
	
	/*CAN过滤器*/
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
	
		/*摩擦轮*/
	GUN_Init();
	
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	/*使能串口的DMA接收，开启串口空闲中断*/
	Bsp_UART_Receive_IT(&huart3,USART3_RX_DATA,SizeofRemote); //这一步的目的是创建一段接受内存
	Bsp_UART_Receive_IT(&huart6,USART6_RX_DATA,SizeofReferee);
	/*使能can中断*/
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}

