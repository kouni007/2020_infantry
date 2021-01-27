#ifndef __OFFLINE_CHECK_H
#define __OFFLINE_CHECK_H

/* ����ͷ�ļ�----------------------------------------------------------------*/

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

/* ��ģ�����ⲿ�ṩ�ĺ궨��--------------------------------------------------*/	
#define LED_PERIOD  600
#define Check_PERIOD  100


/* ��ģ�����ⲿ�ṩ�Ľӿں���ԭ������----------------------------------------*/		
void Led_Task(void const * argument);
void Check_Task(void const * argument);
























#endif
