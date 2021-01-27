/*******************************************************************************
*                     ��Ȩ���� (C), 2017-,NCUROBOT
********************************************************************************
* �� �� ��   : Motor_USE_CAN.c
* �� �� ��   : ����
* ��    ��   : NCURM
* ��������   : 2018��7��
* ����޸�   :
* ��������   : �����ģ����ʹ��CAN���п��Ƶĵ��
* �����б�   :

*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Motor_USE_CAN.h"
#include "protocol.h"
#include "SystemState.h"
/* �ڲ��Զ����������� --------------------------------------------------------*/

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/
/*********************���̵���Ĳ�������***************************/
moto_measure_t   moto_chassis_get[4] = {0};//4 �� 3508
moto_measure_t   moto_gimbal_get;
RC_Ctl_t  RC_Ctl; //ң������
/* �ⲿ����ԭ������ ----------------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/

static CAN_TxHeaderTypeDef	 Chassis_Motor_Data;
static CAN_TxHeaderTypeDef  CANSend_status_Data;
static CAN_TxHeaderTypeDef  CANSend_Minipc_Data;


extern uint8_t CAN_RX_date[8];
/* ����ԭ������ ----------------------------------------------------------*/

/**
	**************************************************************
	** Descriptions: ���̵����������
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN2
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	    uint8_t CAN_TX_DATA[8];
	
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;

			CAN_TX_DATA[0] = iq1 >> 8;
			CAN_TX_DATA[1] = iq1;
			CAN_TX_DATA[2] = iq2 >> 8;
			CAN_TX_DATA[3] = iq2;
			CAN_TX_DATA[4] = iq3 >> 8;
			CAN_TX_DATA[5] = iq3;
			CAN_TX_DATA[6] = iq4 >> 8;
			CAN_TX_DATA[7] = iq4;
	
			HAL_CAN_AddTxMessage(hcan, &Chassis_Motor_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}	

/**
	**************************************************************
	** Descriptions: ���̵��ʧ�ܺ���
	** Input: 	
	**			   hcan:Ҫʹ�õ�CAN2
	**					iqn:��n�����̵���ĵ���ֵ
	** Output: NULL
	**************************************************************
**/
void Chassis_Motor_Disable( CAN_HandleTypeDef * hcan)
{
	    uint8_t CAN_TX_DATA[8];
	
			Chassis_Motor_Data.DLC = 0x08;
			Chassis_Motor_Data.IDE = CAN_ID_STD;
			Chassis_Motor_Data.RTR = CAN_RTR_DATA;
			Chassis_Motor_Data.StdId = 0x200;
  
			CAN_TX_DATA[0] = 0x00;
			CAN_TX_DATA[1] = 0x00;
			CAN_TX_DATA[2] = 0x00;
			CAN_TX_DATA[3] = 0x00;
			CAN_TX_DATA[4] = 0x00;
			CAN_TX_DATA[5] = 0x00;
			CAN_TX_DATA[6] = 0x00;
			CAN_TX_DATA[7] = 0x00;
	
			HAL_CAN_AddTxMessage(hcan, &Chassis_Motor_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}	

/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ��6623����ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_6623(moto_measure_t *ptr,uint8_t CAN_RX_date[])
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	ptr->real_current  = (int16_t)(CAN_RX_date[2]<<8 | CAN_RX_date[3]);
	ptr->given_current = (int16_t)(CAN_RX_date[4]<<8 | CAN_RX_date[5]);
	ptr->speed_rpm = ptr->real_current;
//	ptr->hall = hcan->pRxMsg->Data[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
/**                                                           //����
	**************************************************************
	** Descriptions: ��ȡCANͨѶ��3508����ķ���ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
void get_moto_measure_3508(moto_measure_t *ptr,uint8_t CAN_RX_date[])
{
	/*BUG!!! dont use this para code*/

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	ptr->speed_rpm  = (int16_t)(CAN_RX_date[2]<<8 | CAN_RX_date[3]);
	ptr->real_current = (int16_t)(CAN_RX_date[4]<<8 | CAN_RX_date[5]);
	ptr->hall = CAN_RX_date[6];
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
/**
	**************************************************************
	** Descriptions:��ȡ�������ֵ��ƫ��ֵ
	** Input: 	
	**			  ptr:Ŀ�����ݵ��ڴ��ַ
	**				hcan->pRxMsg->Data:���������CAN�����ݵ�����
	** Output: NULL
	**************************************************************
**/
/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr,uint8_t CAN_RX_date[])
{
	ptr->angle = (uint16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	ptr->offset_angle = ptr->angle;
}



/**
	**************************************************************
	** Descriptions: ����ͨ�ţ����͸�ң�صĵ��̣�
	** Input: 	ң��������
	**			  
	**				
	** Output: NULL
	**************************************************************
**/
void CAN_Send_Chassis_status( CAN_HandleTypeDef * hcan)
{
	    uint8_t CAN_TX_DATA[8];
	
			CANSend_status_Data.DLC = 0x08;
			CANSend_status_Data.IDE = CAN_ID_STD;
			CANSend_status_Data.RTR = CAN_RTR_DATA;
			CANSend_status_Data.StdId = 0x170;

			CAN_TX_DATA[0] = 0;
			CAN_TX_DATA[1] = 0;
			CAN_TX_DATA[2] = 0;
			CAN_TX_DATA[3] = 0;
			CAN_TX_DATA[4] = 0;
			CAN_TX_DATA[5] = 0;
			CAN_TX_DATA[6] = 0;
			CAN_TX_DATA[7] = 0;
	
			HAL_CAN_AddTxMessage(hcan, &CANSend_status_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}	


void CAN_Send_Minipc( CAN_HandleTypeDef * hcan)
{
	    uint8_t CAN_TX_DATA[8];
	
			CANSend_Minipc_Data.DLC = 0x08;
			CANSend_Minipc_Data.IDE = CAN_ID_STD;
			CANSend_Minipc_Data.RTR = CAN_RTR_DATA;
			CANSend_Minipc_Data.StdId = 0x180;

			CAN_TX_DATA[0] = 0;
			CAN_TX_DATA[1] = 0;
			CAN_TX_DATA[2] = 0;
			CAN_TX_DATA[3] = 0;
			CAN_TX_DATA[4] = 0;
			CAN_TX_DATA[5] = 0;
			CAN_TX_DATA[6] = 0;
			CAN_TX_DATA[7] = 0;
	
			HAL_CAN_AddTxMessage(hcan, &CANSend_Minipc_Data, CAN_TX_DATA, (uint32_t *)CAN_TX_MAILBOX0 );
}	


void CAN_Get_Remote(uint8_t CAN_RX_date[])   //����ң������Ϣ
{
	    RC_Ctl.key.v = (int16_t)(CAN_RX_date[0]<<8 |CAN_RX_date[1]) ;
			RC_Ctl.rc.ch0 = (int16_t) (CAN_RX_date[2]<<8 |CAN_RX_date[3]) ;
			RC_Ctl.rc.ch1 = (int16_t) (CAN_RX_date[4]<<8 |CAN_RX_date[5]) ;
			RC_Ctl.rc.s1 = CAN_RX_date[6];
			RC_Ctl.rc.s2 = CAN_RX_date[7];
}


void CAN_Get_Gimbal(uint8_t CAN_RX_date[])   //������̨��Ϣ
{
	    moto_gimbal_get.angle = (int16_t)(CAN_RX_date[0]<<8 | CAN_RX_date[1]) ;
	    moto_gimbal_get.total_angle  =(int16_t) (CAN_RX_date[2]<<8 | CAN_RX_date[3]) ;
      gimbal_status.minipc_mode =  (uint8_t)CAN_RX_date[4];         //flag----minipc.mode
      gimbal_status.gimbal_mode = (uint8_t)CAN_RX_date[5];         //flag1----gimbal.mode
      gimbal_status.remote_mode = (uint8_t)CAN_RX_date[6];       //remote.mode

}
