#ifndef  __BSP_H
#define  __BSP_H

#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "communication.h "
#include "Motor_USE_TIM.h"
#include "Motor_USE_CAN.h"
#include "bsp_delay.h"






void BSP_Init(void);


#endif

