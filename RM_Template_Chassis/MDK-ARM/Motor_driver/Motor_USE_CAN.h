#ifndef __MOTOR_USE_CAN_H
#define __MOTOR_USE_CAN_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "can.h"
#include "communication.h "
#include "chassis_control.h"
#include "pid.h"

#define FILTER_BUF_LEN		5

typedef enum
{
	CAN_3508Moto_ALL_ID = 0x200,
	CAN_3508Moto1_ID = 0x201,
	CAN_3508Moto2_ID = 0x202,
	CAN_3508Moto3_ID = 0x203,
	CAN_3508Moto4_ID = 0x204,
		
}CAN_Message_ID;

typedef struct{
	int16_t	 			speed_rpm;
	int16_t  			real_current;
	int16_t  			given_current;
	uint8_t  			hall;
	uint16_t 			angle;				//abs angle range:[0,8191]
	uint16_t 			last_angle;	//abs angle range:[0,8191]
	uint16_t			offset_angle;
	int32_t				round_cnt;
	int32_t				total_angle;
	uint8_t				buf_idx;
	uint16_t			angle_buf[FILTER_BUF_LEN];
	uint16_t			fited_angle;	
	uint32_t			msg_cnt;
	int32_t      run_time;
	int32_t      cmd_time;
	int32_t      reverse_time;
	int32_t      REVE_time;
}moto_measure_t;


extern moto_measure_t   moto_chassis_get[];
extern moto_measure_t   moto_gimbal_get;

void Chassis_Motor( CAN_HandleTypeDef * hcan,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void get_moto_measure_3508(moto_measure_t *ptr,uint8_t CAN_RX_date[]);
void get_moto_measure_6623(moto_measure_t *ptr,uint8_t CAN_RX_date[]);
void get_moto_offset(moto_measure_t *ptr,uint8_t CAN_RX_date[]);
void CAN_Send_Chassis_status( CAN_HandleTypeDef * hcan);
void CAN_Send_Minipc( CAN_HandleTypeDef * hcan);

void CAN_Get_Remote(uint8_t CAN_RX_date[]);
void CAN_Get_Gimbal(uint8_t CAN_RX_date[]);

#endif
