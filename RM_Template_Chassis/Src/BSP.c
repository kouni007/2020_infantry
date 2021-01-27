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
	** Descriptions:	JY61休眠/解休眠
	** Input:	huart  发送指令的串口，波特率要求为115200
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void JY61_SLEEPorUNSLEEP(UART_HandleTypeDef *huart)
{
	uint8_t buff[3] = {0xff,0xaa,0x60};
	//休眠,解休眠
	HAL_UART_Transmit(huart,buff,3,10);
}

/**
	**************************************************************
	** Descriptions: JY61帧对齐函数
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
	
	/*CAN过滤器*/
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	
	/*开启CAN和ADC*/
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
//	HAL_ADC_Start(&hadc1);           //此函数开启后，会与后面的 HAL_ADC_Start_DMA 函数相冲突，导致DMA只开启一次。具体原因暂未找到
	
	/*断线检测*/
	SystemState_Inite();
	Bsp_UART_Receive_IT(&huart2,USART6_RX_DATA,100);	
	Bsp_UART_Receive_IT(&huart1,Recive_103,12);	

	/*使能can中断*/
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uhADCxConvertedValue, 10); 
}
