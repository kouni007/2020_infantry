#include "stm32f4xx_hal.h"
#include "Power_restriction.h"
#include "protocol.h"
#include "adc.h"
#include "gpio.h"
#include "pid.h"
#include "data_processing.h"
#include "can.h"
float all_sent;
uint8_t  judge_state; //����״̬

#define   min_zero(x) ((x<0) ? (0) :x)

Cap_Sent  Sent_103;
Cap_Get   Get_103;
uint8_t   Recive_103[12];
uint8_t   Discharge_Flag=0;
uint8_t   Send_103[8];


/*���ʿ���*/

#define safety_buff    50  //��ֹ��������
#define judge_buff     20  //����������
#define emergency_buff 10  //������������

float    last_current=0;//�ϴη����ܵ���
float    all_sent=0;//��ǰĿ�����
float    Max=0;//������ת�ص���
float    Max_Current;//���ݳ�繦��
uint8_t  over=0;

uint8_t  judge_state; //����״̬
uint8_t  STATE;//����״̬
uint32_t judge_time=0;//���߼��
uint8_t  Switch_mode=0;//ģʽ����
uint8_t  Stall_Flag=0;

uint16_t   test_id=0;



/*
 ** ��������:������ʵ�ĵ���ֵ=>spi���͵�
 ** ����:��ʵ����
 ** ���:ʵ�ʵ���
 */	


void package_frame(const void *data)
{
	int i;
	for(i=1; i<5; i++)
	{
		Send_103[i] = ((unsigned char *)data)[i-1];
	}
}
float exchange_data(unsigned char *data)
{
	float float_data;
	float_data = *((float*)data);
	return float_data;
}
/*
** ��������:�Գ������ݵĳ��ʹ�ܣ��ͶԵ���Ĺ��緽ʽ����
** ����:��翪�أ� ����ѡ��
** ���:
*/
void CAP_Mode (uint8_t Power_Supply , uint8_t charge)
{
   Sent_103.Power_Supply = Power_Supply;
   Sent_103.Charge = charge;	
}

/*
** ��������:�Գ������ݵ�ģʽѡ�񣬳����������
*/
void super_cap( float Power, uint16_t Buffer, float Voltage, float Current_Charge, float Moto_Power )   /* ���԰�+�ڶ��� */
{                                                                                                                                                     
	float	CAP_Current;                                                                        /* ����Ŀ������� */
	//	float	Power_Max = (float) (Buffer + 25);                                                /* 60 --> 85�� 50 --> 75 */
	float	Power_Max = (float)(Buffer - 5);                                                      /* 60 --> 55�� 50 --> 45 */
	if(Power_Max<=0)Power_Max = 0;
	if(level==1){
			if ( (Buffer - (Power - 50.0f) * 0.7f < 20.0f) )   /* ��Ҫ��������  20 */
		
		//50w���ݣ�(Buffer - (Power -50)*0.7<20.0f
		//80w ���ݣ�(Buffer - (Power -80)*0.2<30.0f  
		
	  {
		Max	= (1800+1.85 * Buffer * Buffer) * 0.85f*1.2f;//������ת�ص��� /1800  1.85   0.85
    
		//50w ����   Max	= (1800+1.85 * Buffer * Buffer) * 0.85f*1.2f;
		//80w ����   Max	= (1800+1.85 * Buffer * Buffer) * 0.85f*1.6f;
		
		Max_Current	= (500 + Buffer * Buffer * 8) / 1000;//���ݳ�����

		if ( Max_Current > Power_Max/24.0f )
			Max_Current = Power_Max/24.0f;
		if ( Max > 2200*1.2f ) //2200
			
		//50w ����  Max > 2200*1.2f  
		//80w ���� Max>3520
		
			Max = 2200*1.2f;
		
		//50w ����   Max=2200*1.2f
		//80w ����   Max=3520
		
		over = 1;
	   }
	else over = 0;		
	}
	else if(level==2){
			if ( (Buffer - (Power -80)*0.2<30.0f ) )   /* ��Ҫ��������  20 */
		
		//50w���ݣ�(Buffer - (Power -50)*0.7<20.0f
		//80w ���ݣ�(Buffer - (Power -80)*0.2<30.0f  
		
	   {
		Max	= (1800+1.85 * Buffer * Buffer) * 0.85f*1.6f;//������ת�ص��� /1800  1.85   0.85
    
		//50w ����   Max	= (1800+1.85 * Buffer * Buffer) * 0.85f*1.2f;
		//80w ����   Max	= (1800+1.85 * Buffer * Buffer) * 0.85f*1.6f;
		
		Max_Current	= (500 + Buffer * Buffer * 8) / 1000;//���ݳ�����

		if ( Max_Current > Power_Max/24.0f )
			Max_Current = Power_Max/24.0f;
		if ( Max>3520 ) //2200
			
		//50w ����  Max > 2200*1.2f  
		//80w ���� Max>3520
		
			Max=3520;
		
		//50w ����   Max=2200*1.2f
		//80w ����   Max=3520
		
		over = 1;
		
	  }
	else over = 0;		 
	}


	if ( over == 1 && Voltage <= 14 ) /* ���� */
	{
		if ( all_sent < 1 )
			all_sent = 1;

		last_current = Max;
		CAP_Mode( BATTERY, Charge_OFF );
		CAP_Current = (0) / Voltage;

		for ( int i = 0; i < 4; i++ )
			pid_3508_spd[i].pos_out = ( pid_3508_spd[i].pos_out * last_current ) / (all_sent);
		test_id = 1000;
	}
	 else if ( over == 1 && Voltage > 14 )//С�ĵ����ݷŵ�ʱ��������һֱ�ܸߣ������޷��Լ��˳�������
	{
		if(Moto_Power >= 40)
		{
		 CAP_Mode( SUPERCAP, Charge_ON );
		 CAP_Current = (Max_Current * 24) / Voltage; /* ע������ǲ��ǳ�Voltage */
		}
		else if(Moto_Power < 40)
		{
		 CAP_Mode( BATTERY, Charge_ON );
		 CAP_Current = (40-Moto_Power) / Voltage; /* ע������ǲ��ǳ�Voltage */
		}
		test_id = 2000;
	}else if ( Moto_Power >= Power_Max && over == 0 )
	{
		CAP_Mode( BATTERY, Charge_ON );
		CAP_Current = (0) / Voltage;
		test_id = 3000;
	}else if ( Moto_Power < Power_Max && over == 0 )
	{
		CAP_Mode( BATTERY, Charge_ON );
		CAP_Current = (Power_Max - Moto_Power) / Voltage;
		test_id = 4000;
	}

	CAP_Current=0.25;
//	Sent_103.Charge=Charge_ON;
//	Sent_103.Power_Supply=SUPERCAP;
	Send_103[0]	= 0xFF;
	package_frame( &CAP_Current );
	Send_103[5]	= Sent_103.Charge;
	Send_103[6]	= Sent_103.Power_Supply;
	Send_103[7]	= 0xFF;
}
