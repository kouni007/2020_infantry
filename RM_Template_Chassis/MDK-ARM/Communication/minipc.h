#ifndef __minipc_H
#define __minipc_H
#include "stm32f4xx.h"
#include "main.h"
#include "communication.h "

typedef struct{

unsigned char 		frame_header; 		  //֡ͷ0xFF
 int16_t 					angle_yaw;     			//yaw angle
 int16_t 					angle_pit;     			//pitch angle 
unsigned char 		state_flag;     		//��ǰ״̬����0 δ��׼Ŀ�� ���� 1 ����������Ŀ�꡿��2 Զ��������Ŀ�꡿��3 ���ģʽ��
	int16_t 					distance;     			//Ŀ�����
	int16_t           angle;
unsigned char 		frame_tail; 	  	  //֡β0xFE
}Minipc_Rx;

typedef struct{

unsigned char 		frame_header; 		  //֡ͷ0xFF
unsigned char 		cmd1;     					//cmd1
unsigned char 		cmd2;     					//cmd2 
unsigned char 		cmd3;     					//cmd1
unsigned char  		cmd4;     					//cmd2 	
unsigned char     cmd5;
unsigned char     cmd6;  
unsigned char     cmd7;  
unsigned char 		frame_tail; 	  	  //֡β0xFE
}Minipc_Tx;


extern Minipc_Rx minipc_rx;
extern Minipc_Tx minipc_tx;

extern uint8_t USART2_RX_DATA[(SizeofMinipc)];		//MiniPC
extern uint8_t UART7_RX_DATA[(SizeofMinipc)];		//MiniPC
extern uint16_t USART2_RX_NUM;

void Get_MiniPC_Data(void);
void Send_MiniPC_Data(uint8_t flag1,uint8_t flag2,uint16_t yaw_angle , uint16_t pit_angle ,uint8_t color);


#endif
