#include "other_tasks.h"
#include "BSP.h"
#include "Power_restriction.h"

/* 内部变量------------------------------------------------------------------*/
uint32_t test_i = 0;
char test2[]="7";
int current_adc_get;
float I_value[8];
float I_measure;
float I_SUM;
float V_value;
float Total_power;

extern float total_set;
/* 内部函数原型声明----------------------------------------------------------*/

/* 任务主体部分 -------------------------------------------------------------*/
void testTask(void const * argument)
{
	int16_t angle[4];
  int getbuff = 0;	
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{ 				
		RefreshTaskOutLineTime(testTask_ON);
    test_i++;

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_RESET);
				uint32_t *buff = uhADCxConvertedValue;

		
		getbuff = 0;
		for(uint8_t i = 0;i < 10;i++)
		{
			getbuff += buff[i];
		}
		V_value =  getbuff * 0.1f * 3.3 / 4096;
		
		I_value[7] = I_value[6];
		I_value[6] = I_value[5];
		I_value[5] = I_value[4];
		I_value[4] = I_value[3];
		I_value[3] = I_value[2];
		I_value[2] = I_value[1];
		I_value[1] = I_value[0];
		I_value[0] = (getbuff * 0.1f * (0.00080566f) - 1.65f) / 0.066f;    //0.066为线性度
		
		I_SUM = 0;
	  for(uint8_t i = 0;i < 8;i++)
		{
			I_SUM += I_value[i];
		}
		I_measure = I_SUM / 8;
		Total_power = I_measure * 24.0 * 1.25;
//HAL_UART_Transmit(&huart6, (uint8_t*)test2, 1, 10);
				int16_t angle1[5];
    int16_t  *ptr = angle1; //初始化指针
		angle1[0]	= (int16_t)(100*Robot.heat.shoot_17_speed);
		angle1[1]	= (int16_t)(100*Robot.Chassis_Power.Chassis_Power_buffer);
		angle1[2]	= ((int16_t)100*Robot.Chassis_Power.chassis_Power);
		angle1[3]	= (int16_t)(test_id);
		//angle1[4] = (int16_t)(Get_103.CAP_Current);
		//用虚拟示波器，发送数据
		//vcan_sendware((uint8_t*)angle1, sizeof(angle1));
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[0]));
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[1]));
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[2]));
		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[3]));	
//		vcan_sendware((uint8_t *)ptr,4*sizeof(angle1[4]));	
		
		
		
//float var[5];
//var[0] =	(float)(Get_103.CAP_Voltage);
//var[1] = (float)(Get_103.ALL_Current);
//var[2] =	(float)(Get_103.CAP_Current);
//var[3] = (float)(100*Robot.Chassis_Power.chassis_Power);
//var[4] = (float)(100*Robot.Chassis_Power.Chassis_Power_buffer);

//vcan_sendware((uint8_t*)var, sizeof(var));
		
		
		  osDelayUntil(&xLastWakeTime,20);
	}
}
