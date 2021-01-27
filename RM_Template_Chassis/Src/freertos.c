/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "data_processing.h"
#include "offline_check.h"
#include "other_tasks.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId defaultTaskHandle;
osThreadId testTaskHandle; 				
osThreadId RemoteDataTaskHandle;  
osThreadId RefereeDataTaskHandle;				
osThreadId MiniPCDataTaskHandle;
osThreadId LedTaskHandle;
osThreadId vOutLineCheckTaskHandle;
osThreadId CheckTaskHandle;
/* USER CODE END Variables */


int test_count = 0;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void const * argument); 
extern void testTask(void const * argument);
extern void Remote_Data_Task(void const * argument);
extern void Referee_Data_Task(void const * argument);
extern void MiniPC_Data_task(void const * argument);
extern void Led_Task(void const * argument);
extern void vOutLineCheck_Task(void const *argument);
extern void Check_Task(void const *argument);
extern void Chassis_Contrl_Task(void const * argument);
/* USER CODE END FunctionPrototypes */



void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(Task1, testTask, osPriorityNormal, 0, 256);                         //�������������Զ�
	testTaskHandle = osThreadCreate(osThread(Task1), NULL);


  osThreadDef(chassisTask, Chassis_Contrl_Task, osPriorityHigh, 0, 256);          //ң�������ݴ���
	RemoteDataTaskHandle = osThreadCreate(osThread(chassisTask), NULL);
	
	osThreadDef(RemoteDataTask, Remote_Data_Task, osPriorityHigh, 0, 256);          //ң�������ݴ���
	RemoteDataTaskHandle = osThreadCreate(osThread(RemoteDataTask), NULL);

	osThreadDef(MiniPCDataTask, MiniPC_Data_task, osPriorityAboveNormal, 0, 128);   //�Ӿ����ݴ���
	MiniPCDataTaskHandle = osThreadCreate(osThread(MiniPCDataTask), NULL);


  /***********���߼��**************/
//	osThreadDef(LedTask, Led_Task, osPriorityAboveNormal, 0, 256);                  //����ģ����߱�־�������ƴ���
//	LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);
//	
//	osThreadDef(CheckTask, Check_Task, osPriorityNormal, 0, 128);                   //����������߱�־���д���
//	CheckTaskHandle = osThreadCreate(osThread(CheckTask), NULL);
//	
//	osThreadDef(vOutLineCheckTask, vOutLineCheck_Task, osPriorityNormal, 0, 128);   //����������߼�⡢ģ�飨�����ң�����ȣ����߼�⼰���ö��߱�־
//	vOutLineCheckTaskHandle = osThreadCreate(osThread(vOutLineCheckTask), NULL);
  /* USER CODE END RTOS_THREADS */

}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1000);
  }
  /* USER CODE END 5 */ 
}     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
