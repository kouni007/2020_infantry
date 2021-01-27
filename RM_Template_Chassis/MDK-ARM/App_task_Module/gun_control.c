/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gun_control.h"
#include "math.h"
#include "SystemState.h"
#include "user_lib.h"
#include "gimbal_control.h"

/* �ڲ�����------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //���̵��λ�û�
pid_t pid_dial_spd  = {0};	//���̵���ٶȻ�

Heat_Gun_t  ptr_heat_gun_t;
Mode_Set bopan = {0};	
Mode_Set Shoot_heat = {0};
ramp_function_source_t shoot;
extern uint8_t shot_frequency;
uint8_t finish_flag = 0;
static uint8_t Delay_flg;
uint8_t Guodu_or_Stop_mode_flag = 0;   //����ģʽ��ֹͣģʽ�ı�־λ(�����ж��Ƿ��һ�ν���������ģʽ)
uint8_t Reverse_mode_flag = 0;         //��תģʽ�ı�־λ(�����ж��Ƿ��һ�ν�������ģʽ)
/* �ڲ�����ԭ������----------------------------------------------------------*/
void Gun_Pid_Init()
{
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000, 0.2f, 0.0000f,	2.0f);  
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000, 1.5f,	0.1f,	0.0f	);  
}

/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	void Gun_Task(void const * argument)
	*	@param
	*	@supplement	��������
	*	@retval	
****************************************************************************************/
void Gun_Task(void const * argument)
{ 
	osDelay(2000);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	Gun_Pid_Init();
	
  static int32_t set_angle = 0;
	int32_t set_speed = 0;
	static uint8_t set_cnt = 0;
  /*�趨����*/
  bopan.flag=1;
	
	static float Guodu_Start_time;
	static float Reverse_Start_time;
	/*************************      ģʽ˵��      ******************************
	δѹ��λ����    :  ���軹�ڵ����У���δ������λ����
	ѹ��λ����      :  ���赽����λ���أ���λ���ؽ�ͨ
	����λ����      :  ����ͨ������λ���أ���������Ħ���ַ����ȥ
	
	Limit_switch_Control : �Ƿ������λ���ؿ���
	
	Shoot_mouse.mode��0  
										1  Shoot.mode = Mocha_danfa_mode  , bopan.mode = bopan_danfa_mode(Ħ���ֵ���ָ��ת�ٺ��趨ģʽ)
										2  Shoot.mode = Mocha_Duofa_mode  , bopan.mode = bopan_Duofa_mode(Ħ���ֵ���ָ��ת�ٺ��趨ģʽ)
										3  Shoot.mode = Mocha_Lianfa_mode , bopan.mode = bopan_Lianfa_mode(Ħ���ֵ���ָ��ת�ٺ��趨ģʽ)
										4  Shoot.mode = Mocha_Duofa_mode  , bopan.mode = bopan_Lianfa_mode(Ħ���ֵ���ָ��ת�ٺ��趨ģʽ)
										
	bopan.mode      :  bopan_Stop           0       
										 bopan_danfa_mode     1           Shoot.mode == 1             ***����ģʽ
										 bopan_Duofa_mode     2           Shoot.mode == 2             ***����ģʽ
										 bopan_Lianfa_mode    3           Shoot.mode == 3             ***����ģʽ
										 bopan_shangdan_mode  4           �ϵ���ָ������������λ���ش�( ѹ��λ����  ��ʾ����ָ��λ��)
										 bopan_guodu_mode     5           ����ģʽ��ͣתһС��
										 bopan_fanzhuan_mode  10
										 
  bopan.flag      : 0   ��ת��־λ
	                  1
	
	Shoot.flag      : 0   ��λ���ص�ƽδ�仯
(��λ���ص�ƽ�仯)  1   ��λ���ص�ƽ�����仯
										2   �ϵ���־
										
  Shoot.shoot_flg : 0   δѹ��λ����  ��  ����λ����
(��λ���ص�ƽ)      1   ѹ��λ���� 
	
	Delay_flg       :
	****************************************************************************/
	for(;;)
	{
		
			RefreshTaskOutLineTime(GunTask_ON);

		  moto_dial_get.cmd_time = GetSystemTimer();    //��ȡ����ʵʱʱ��
			if(Limit_switch_Control)                      //�Ƿ������λ���ؿ���
			{    
					 if(Shoot.flag == 2 )  
					 {
							bopan.mode = bopan_shangdan_mode;
							Delay_flg = 0;
					 }else if(Shoot.shoot_flg == 1 && bopan.mode == bopan_shangdan_mode)      // ѹ��λ���� && �ϵ�ģʽ
							bopan.mode = bopan_Stop;
					 else if(Shoot.flag == 1 && bopan.mode == bopan_danfa_mode && Shoot.cnt == Shoot.danfa_cnt + 2) //Ԥ��ʱ�Ѿ�����һ
							Delay_flg = 1;        //��λ���ص�ƽ�仯 && ����ģʽ && �����ͨ����λ���� ���������ģʽ
					 else if(Shoot.flag == 1 && bopan.mode == bopan_Duofa_mode && Shoot.cnt == Shoot.last_cnt + 6 ) 
							Delay_flg = 1;        //��λ���ص�ƽ�仯 && �෢ģʽ && 3�ŵ����ͨ����λ���� ������ͣһ��ʱ�� 
					 else
					 {        
							Delay_flg = 0;
					 }                       
			}
			
		 if(Delay_flg || ( moto_dial_get.cmd_time - Guodu_Start_time < DELAY_TIME ))  //�������ģʽ������ͣһ��ʱ��    
		 {
				 bopan.mode = bopan_guodu_mode;
			   moto_dial_get.run_time = GetSystemTimer();                 //��ȡ���̿�ʼת����ʱ��
		 }
	 
		 bopan_check(set_cnt);//�������	

    /*�жϷ���ģʽ*/
   switch(bopan.mode)
   {
			case bopan_Stop://ֹͣ
			{
				  if(Guodu_or_Stop_mode_flag == 0)  //�����״ν���ֹͣģʽ
					{
						  Moto_dial_Angle_Reset();
						  set_cnt = 0;
						  set_angle = moto_dial_get.total_angle;
						
						  Guodu_or_Stop_mode_flag = 1;
					}else
					{
					    set_cnt = 0;
					}
					
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
					set_speed = pid_dial_pos.pos_out;
			}break;
			
      case bopan_danfa_mode://��������ģʽ
      {
					set_cnt = 1;    
					set_angle = -36860 * set_cnt - 500;  //һ������۽Ƕ�  8191*36/8    2006������ٱ�Ϊ36    500��Ϊ�����ת�ض��ӵ�����
				
					/*pidλ�û�*/
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
					set_speed = pid_dial_pos.pos_out;
				
          Guodu_or_Stop_mode_flag = 0;
				  Reverse_mode_flag = 0;
      }break;
			
      case bopan_Duofa_mode://�̶��Ƕ�ģʽ   ������
      {
					set_cnt = 3;
					set_angle = -36860 * set_cnt - 500;
				
					/*pidλ�û�*/
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
					set_speed = pid_dial_pos.pos_out;
					
					Guodu_or_Stop_mode_flag = 0;
					Reverse_mode_flag = 0;
      }break;
			
      case bopan_Lianfa_mode://����ģʽ
      { 
					set_speed = -3000;
					set_cnt = 1;
					
				  Guodu_or_Stop_mode_flag = 0;
					Reverse_mode_flag = 0;
      }break;
			
			case bopan_shangdan_mode://�ϵ�
      { 
					moto_dial_get.cmd_time = GetSystemTimer();
					set_speed = -500;
					set_cnt = 0;
				
				  Guodu_or_Stop_mode_flag = 0;
					Reverse_mode_flag = 0;
      }break;
      
      case bopan_guodu_mode://����
      { 
				  if(Guodu_or_Stop_mode_flag == 0)  //�����״ν������ģʽ
					{
						  Moto_dial_Angle_Reset();
						  set_cnt = 0;
						  set_angle = moto_dial_get.total_angle;
						
						  Guodu_or_Stop_mode_flag = 1;
						  Guodu_Start_time = GetSystemTimer();  //��ȡ����ģʽ��ʼʱ��ʱ��
					}else
					{
							Delay_flg = 0;
							set_cnt = 0;
					}
				
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
			    set_speed = pid_dial_pos.pos_out;
					
      }break;
      
			case bopan_fanzhuan_mode://��ת
			{
				if(Reverse_mode_flag == 0)        //�����״ν��뷴תģʽ
				{
					  set_cnt = 0;
						Reverse_Start_time = GetSystemTimer();	 //��ȡ��תģʽ��ʼ��ʱ��
						Reverse_mode_flag = 1;
				}else
				{
						if( moto_dial_get.cmd_time - Reverse_Start_time > REVERSE_TIME )//��ת�趨    
						{
							Moto_dial_Angle_Reset();
							bopan.flag = 1;
							set_cnt = 0;
						}else 
						{			
							set_speed = 1000;
						}
				}
			}break;
			default :break;
    }
     /*�ٶȻ�*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
     /*�����������*/
		 Allocate_Motor(&hcan1,pid_dial_spd.pos_out);
 
		if(bopan.last_mode != Mocha_Duofa_mode && bopan.mode == Mocha_Duofa_mode)  
			 Shoot.last_cnt = Shoot.cnt;		 //ģʽ�л�Ϊ������෢ʱ�������ʱ�������ֵ(ÿ�ŵ������ֵΪ2)��
		else if(bopan.last_mode != Mocha_danfa_mode && bopan.mode == Mocha_danfa_mode)  
			 Shoot.danfa_cnt = Shoot.cnt;
		
		bopan.last_mode = bopan.mode; 

    osDelayUntil(&xLastWakeTime,GUN_PERIOD);

	 }
}


/***************************************************************************************
**
	*	@brief	void Mocha_Task(void const *argument)
	*	@param
	*	@supplement	   
	*	@retval	
****************************************************************************************/
void Mocha_Task(void const *argument)  //���Բ����йر��˴�����ʹ��Ħ�����ٶȲ���
{
	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	ramp_init(&shoot,0.01,150,100);//Ħ����б��           0.01Ϊб��  150��100Ϊ�����Сֵ(���ֵҲΪĿ��ֵ)
	
	for(;;)  //Ŀǰ��������Ħ���ֵ�����ת��
	{
		shoot.max_value = 118;																	
		ramp_calc(&shoot,100);  //100Ϊ����ϵ��
	
	 osDelayUntil(&xLastWakeTime,Mocha_PERIOD);
		
	}
}



/***************************************************************************************
**
	*	@brief	void Shoot_Heat_Task(void const *argument)
	*	@param
	*	@supplement	   
	*	@retval	
****************************************************************************************/
void Shoot_Heat_Task(void const *argument)
{
  
  osDelay(500);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  
  
  for(;;)
  {
    
    Shoot_heat.val = Robot.heat.shoot_17_cooling_limit - Robot.heat.shoot_17_heat;//ʣ������
    
    if(Shoot_heat.val<Shoot_heat.heat_forecast)
    {
      Shoot_heat.flag = 0;
    }else Shoot_heat.flag = 1;
    
    osDelayUntil(&xLastWakeTime,10);
  }
   
}



/***************************************************************************************
**
	*	@brief	void bopan_check( uint8_t  Set_cnt )
	*	@param
	*	@supplement	  ���̶�ת���
	*	@retval	
****************************************************************************************/
void bopan_check( uint8_t  Set_cnt )
{
	
	   	if(bopan.flag  == 1)
			{
		     /*�жϲ����Ƿ�ת��λ��*/			
					if(my_abs(moto_dial_get.total_angle) >= 36859 * Set_cnt)      //ת��һ�����裬ת����ת36/8 = 4.5 Ȧ
					{
							if((bopan.mode == bopan_danfa_mode) && ( Set_cnt != 0) && (!Limit_switch_Control))
							{ 
								 Delay_flg = 1;
							}
							
							if((bopan.mode != bopan_guodu_mode) && (bopan.mode != bopan_Stop))   //������ģʽ����ΪSet_cntΪ0�����Բ��������Ƶ������
							{
								 Moto_dial_Angle_Reset();
						  }
							
							moto_dial_get.run_time = GetSystemTimer();                 //��ȡ���̿�ʼת����ʱ��
							bopan.flag = 1;
					}
					else   //�ս��뵥�����෢�����������ϵ�ģʽ  �Ż������ж�
					{
							if( my_abs(moto_dial_get.cmd_time - moto_dial_get.run_time) > BLOCK_TIME )//��ת�ж�    
							{
									bopan.flag = 0;	
							}
					}
			}
								
			if(bopan.flag == 0)   
				bopan.mode = bopan_fanzhuan_mode;//��ת
	
}


/***************************************************************************************
**
	*	@brief	void Moto_dial_Angle_Reset(void)
	*	@param
	*	@supplement	  ���̽Ƕ�����
	*	@retval	
****************************************************************************************/
void Moto_dial_Angle_Reset(void)
{
		moto_dial_get.round_cnt = 0;            
		moto_dial_get.offset_angle = moto_dial_get.angle;
		moto_dial_get.total_angle = 0;
}
