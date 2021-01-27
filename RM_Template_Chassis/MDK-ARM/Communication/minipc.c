/*******************************************************************************
                      ��Ȩ���� (C), 2017-,NCUROBOT
 *******************************************************************************
  �� �� ��   : minipc.c
  �� �� ��   : ����
  ��    ��   : NCUERM
  ��������   : 2018��7��
  ����޸�   :
  ��������   : ���ֽ���
  �����б�   :void Get_MiniPC_Data(void)
							void Send_MiniPC_Data(unsigned char cmd1,
																		unsigned char cmd2,
																		unsigned char state)
*******************************************************************************/

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "minipc.h"
/* �ڲ��Զ����������� --------------------------------------------------------*/

/* �ڲ��궨�� ----------------------------------------------------------------*/

/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/

/* ���õ��ⲿ����ԭ������ ----------------------------------------------------------
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, 
																		uint8_t *pData, uint16_t Size, uint32_t Timeout);
*/
/* �ڲ����� ------------------------------------------------------------------*/
Minipc_Rx minipc_rx;
Minipc_Tx minipc_tx;
uint8_t USART2_RX_DATA[(SizeofMinipc)];		//MiniPC
uint8_t UART7_RX_DATA[(SizeofMinipc)];		//MiniPC
uint16_t USART2_RX_NUM;
uint16_t UART7_RX_NUM;
/* ����ԭ������ ----------------------------------------------------------*/

/**
	**************************************************************
	** Descriptions: ��ȡ����minipc������
	** Input: NULL
	**			  
	** Output: NULL
	**************************************************************
**/
void Get_MiniPC_Data(void)
{
		uint8_t *buff = USART6_RX_DATA;
			
		minipc_rx.frame_header = buff[0];
		minipc_rx.frame_tail 	 = buff[8];
		if((minipc_rx.frame_header == 0xFF) && (minipc_rx.frame_tail == 0xFE))
		{
			minipc_rx.angle_yaw  = (int16_t)(buff[1] << 8 | buff[2]);
			minipc_rx.angle_pit  = (int16_t)(buff[3] << 8 | buff[4]);
			minipc_rx.state_flag = buff[5];
			minipc_rx.distance   = buff[6]<<8|buff[7];
		}
}

/**
	**************************************************************
	** Descriptions: ���͸�pc������
	** Input: cmd1���Ƿ���pc����
            cmd2��ģʽ�л�
	**			  
	** Output: NULL
	**************************************************************
**/

void Send_MiniPC_Data(uint8_t flag1,uint8_t flag2,uint16_t yaw_angle , uint16_t pit_angle ,uint8_t color)
{
	minipc_tx.frame_header = 0xFF;
	minipc_tx.cmd1 			   = flag1;
	minipc_tx.cmd2 				 = flag2;
  minipc_tx.cmd3         = yaw_angle>>8;
  minipc_tx.cmd4         = yaw_angle;
  minipc_tx.cmd5         = pit_angle>>8;
  minipc_tx.cmd6         = pit_angle; 
  minipc_tx.cmd6         = color; 
	minipc_tx.frame_tail   = 0xFE;
	
//HAL_UART_Transmit(&huart6,(uint8_t *)&minipc_tx,sizeof(minipc_tx),10);

}

