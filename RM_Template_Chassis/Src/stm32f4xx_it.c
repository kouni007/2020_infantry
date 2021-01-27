/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "SystemState.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Power_restriction.h"

/* USER CODE END Includes */
uint32_t DMA_FLAGS;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
CAN_RxHeaderTypeDef  CAN1_Rx_Header;
CAN_RxHeaderTypeDef  CAN2_Rx_Header;
uint8_t CAN1_RX_date[8];
uint8_t CAN2_RX_date[8];
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern  osThreadId RemoteDataTaskHandle;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim13;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */

void USART1_IRQHandler(void)
{
 uint8_t tmp1,tmp2;
	tmp1 = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
 tmp2 = __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE);
	
   if((tmp1 != RESET) && (tmp2 != RESET))
{
   	__HAL_DMA_DISABLE(&hdma_usart1_rx);
		
		DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart1_rx);	//dma传输完成产生的标志位
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAGS);

  	__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,12);
		__HAL_DMA_ENABLE(&hdma_usart1_rx);

	for(int i=0;i<3;i++)
	{
	 	uint8_t get[4];
		 for(int j=0;j<4;j++)
		 get[j]=Recive_103[4*i+j];

	 if(i==0)   Get_103.CAP_Voltage  = exchange_data(get);
   if(i==1) 	Get_103.ALL_Current  = exchange_data(get);
   if(i==2) 	Get_103.CAP_Current  = exchange_data(get);
	} 

  __HAL_UART_CLEAR_IDLEFLAG(&huart1);

}
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
 	uint8_t tmp1,tmp2;
 	tmp1 = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);   //空闲中断中将已收字节数取出后，停止DMA
  tmp2 = __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE);
	   
  if((tmp1 != RESET))
//			&& (tmp2 != RESET))
  {  
	__HAL_DMA_DISABLE(&hdma_usart2_rx);
		
	DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart2_rx);	
	__HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx,DMA_FLAGS);

 USART6_RX_NUM=(100)-(hdma_usart2_rx.Instance->NDTR);
    
  Referee_Data_Handler();  
   

	__HAL_DMA_SET_COUNTER(&hdma_usart2_rx,100);
	__HAL_DMA_ENABLE(&hdma_usart2_rx);

				/*清除IDLE标志位*/
   __HAL_UART_CLEAR_IDLEFLAG(&huart2);		
	}
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) 
	{
    HAL_IncTick();
  }else if(htim->Instance == TIM3)  //1ms的定时周期
	{
		RefreshSysTime();
	}else if(htim->Instance == TIM4)  //1ms的定时周期
	{
		
	}else if(htim->Instance == TIM13) //1ms的定时周期
	{
		
	}

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static  BaseType_t  pxHigherPriorityTaskWoken;
	if(hcan == &hcan1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_Rx_Header, CAN1_RX_date);
		switch(CAN1_Rx_Header.StdId)
		{
			case CAN_3508Moto1_ID:     
			{
					if(moto_chassis_get[0].msg_cnt++ <= 50)	
					{
						 get_moto_offset(&moto_chassis_get[0], CAN1_RX_date);
					}else
					{		
						 moto_chassis_get[0].msg_cnt = 51;	
						 get_moto_measure_3508(&moto_chassis_get[0], CAN1_RX_date);
						 RefreshDeviceOutLineTime(Motor1_NO);
					}
			}break;
			
			case CAN_3508Moto2_ID:     
			{
					if(moto_chassis_get[1].msg_cnt++ <= 50)	
					{
						 get_moto_offset(&moto_chassis_get[1], CAN1_RX_date);
					}else
					{		
						 moto_chassis_get[1].msg_cnt = 51;	
						 get_moto_measure_3508(&moto_chassis_get[1], CAN1_RX_date);
						 RefreshDeviceOutLineTime(Motor2_NO);
				  }
			}break;
			
			case CAN_3508Moto3_ID:    
			{		
					if(moto_chassis_get[2].msg_cnt++ <= 50)	
					{
						 get_moto_offset(&moto_chassis_get[2], CAN1_RX_date);
					}else
					{		
						 moto_chassis_get[2].msg_cnt = 51;	
						 get_moto_measure_3508(&moto_chassis_get[2], CAN1_RX_date);
						 RefreshDeviceOutLineTime(Motor3_NO);
					}
			}break;
			
			case CAN_3508Moto4_ID:    
			{			
					if(moto_chassis_get[3].msg_cnt++ <= 50)	
					{
						 get_moto_offset(&moto_chassis_get[3], CAN1_RX_date);
					}else
					{		
						 moto_chassis_get[3].msg_cnt = 51;	
						 get_moto_measure_3508(&moto_chassis_get[3], CAN1_RX_date);
						 RefreshDeviceOutLineTime(Motor4_NO);
					}
			}break;
						case 0x110:     
			{
					CAN_Get_Remote(CAN1_RX_date);
				  vTaskNotifyGiveFromISR(RemoteDataTaskHandle,&pxHigherPriorityTaskWoken);
		      portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
			}break;
			default:break;
			
		}
	}else if(hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2_Rx_Header, CAN2_RX_date);
		
		switch(CAN2_Rx_Header.StdId)
		{
			case 0x110:     
			{
					CAN_Get_Remote(CAN2_RX_date);
				  vTaskNotifyGiveFromISR(RemoteDataTaskHandle,&pxHigherPriorityTaskWoken);
		      portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
			}break;
			
			case 0x119:     
			{
					
			}break;
			
			case 0x120:     
			{
					CAN_Get_Gimbal(CAN2_RX_date);
			}break;
			
			case 0x130:    
			{		
					
			}break;
	    default:break;
		}
		
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
