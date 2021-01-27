/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "gun_control.h"
#include "math.h"
#include "SystemState.h"
#include "user_lib.h"
#include "gimbal_control.h"
#include "protocol.h"
#include "data_processing.h"

/* �ڲ�����------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //���̵��λ�û�
pid_t pid_dial_spd  = {0};	//���̵���ٶȻ�
pid_t pid_moca_spd[2]  = {0};	//Ħ���ֵ���ٶȻ�
Shoot_status_t Shoot_status ={0};

Heat_Gun_t  ptr_heat_gun_t;
ramp_function_source_t shoot;
extern uint8_t shot_frequency;
int32_t set_speed = 0;
uint8_t finish_flag = 0;
static uint8_t Delay_flg;

uint8_t Mocha_flag=0;  //Ħ���ֿ��ر�־λ
uint8_t Magazine_flag=0;  //���ָǿ��ر�־λ


uint8_t Guodu_or_Stop_mode_flag = 0;   //����ģʽ��ֹͣģʽ�ı�־λ(�����ж��Ƿ��һ�ν���������ģʽ)
uint8_t Reverse_mode_flag = 0;         //��תģʽ�ı�־λ(�����ж��Ƿ��һ�ν�������ģʽ)
uint32_t pwmval;
/* �ڲ�����ԭ������----------------------------------------------------------*/
void Gun_Pid_Init()
{
		PID_struct_init(&pid_dial_pos, POSITION_PID, 8000, 5000, 0.17f, 0.0f,	0.0f);  
		PID_struct_init(&pid_dial_spd, POSITION_PID, 8000, 5000, 2.9f,	0.0f,	0.0f	);  
		PID_struct_init(&pid_moca_spd[0], POSITION_PID, 7000, 4000, 1.3f,	0.005f,	0.01f	); 
		PID_struct_init(&pid_moca_spd[1], POSITION_PID, 8000, 6000, 1.4f,	0.005f,	0.005f	); 
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
	
	static uint8_t set_cnt = 0;
  /*�趨����*/
  gimbal_status.bopan_flag = 1;
	
	static float Guodu_Start_time;
	static float Reverse_Start_time;
	/*************************      ģʽ˵��      ******************************
	δѹ��λ����    :  ���軹�ڵ����У���δ������λ����
	ѹ��λ����      :  ���赽����λ���أ���λ���ؽ�ͨ
	����λ����      :  ����ͨ������λ���أ���������Ħ���ַ����ȥ
	
	Limit_switch_Control : �Ƿ������λ���ؿ���
	
	Shoot_status.remote_mode��0  
										1  Shoot.mode = Mocha_danfa_mode  , bopan.mode = bopan_danfa_mode(Ħ���ֵ���ָ��ת�ٺ��趨ģʽ)
										2  Shoot.mode = Mocha_Duofa_mode  , bopan.mode = bopan_Duofa_mode(Ħ���ֵ���ָ��ת�ٺ��趨ģʽ)
										3  Shoot.mode = Mocha_Lianfa_mode , bopan.mode = bopan_Lianfa_mode(Ħ���ֵ���ָ��ת�ٺ��趨ģʽ)
										4  Shoot.mode = Mocha_Duofa_mode  , bopan.mode = bopan_Lianfa_mode(Ħ���ֵ���ָ��ת�ٺ��趨ģʽ)
										
	gimbal_status.bopan_mode      :  bopan_Stop           0       
										 bopan_danfa_mode     1           Shoot.mode == 1             ***����ģʽ
										 bopan_Duofa_mode     2           Shoot.mode == 2             ***����ģʽ
										 bopan_Lianfa_mode    3           Shoot.mode == 3             ***����ģʽ
										 bopan_shangdan_mode  4           �ϵ���ָ������������λ���ش�( ѹ��λ����  ��ʾ����ָ��λ��)
										 bopan_guodu_mode     5           ����ģʽ��ͣתһС��
										 bopan_fanzhuan_mode  10
										 
  gimbal_status.bopan_flag      : 0   ��ת��־λ
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
		  
		  if(Magazine_flag==1){  //��������
				pwmval=1500;
				TIM1->CCR3=pwmval;

				
			}
			else{
				pwmval=500;
				TIM1->CCR3=pwmval;

			}
				
			if(Limit_switch_Control)                      //�Ƿ������λ���ؿ���
			{    
					 if(Shoot_status.limit_flag == 2 )  
					 {
							gimbal_status.bopan_mode = bopan_shangdan_mode;
							Delay_flg = 0;
					 }else if(Shoot_status.limit_level == 1 && gimbal_status.bopan_mode == bopan_shangdan_mode)      // ѹ��λ���� && �ϵ�ģʽ
							gimbal_status.bopan_mode = bopan_Stop;
					 else if(Shoot_status.limit_flag == 1 && gimbal_status.bopan_mode == bopan_danfa_mode && Shoot_status.cnt == Shoot_status.danfa_cnt + 2) //Ԥ��ʱ�Ѿ�����һ
							Delay_flg = 1;        //��λ���ص�ƽ�仯 && ����ģʽ && �����ͨ����λ���� ���������ģʽ
					 else if(Shoot_status.limit_flag == 1 && gimbal_status.bopan_mode == bopan_Duofa_mode && Shoot_status.cnt == Shoot_status.last_cnt + 6 ) 
							Delay_flg = 1;        //��λ���ص�ƽ�仯 && �෢ģʽ && 3�ŵ����ͨ����λ���� ������ͣһ��ʱ�� 
					 else
					 {        
							Delay_flg = 0;
					 }                       
			}
			
		 if(Delay_flg || ( moto_dial_get.cmd_time - Guodu_Start_time < DELAY_TIME ))  //�������ģʽ������ͣһ��ʱ��    
		 {
				 gimbal_status.bopan_mode = bopan_guodu_mode;
			   moto_dial_get.run_time = GetSystemTimer();                 //��ȡ���̿�ʼת����ʱ��
		 }
	 
		 bopan_check(set_cnt);//�������	

    /*�жϷ���ģʽ*/
   switch(gimbal_status.bopan_mode)
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
					set_angle = 36860* set_cnt ;  //һ������۽Ƕ�  8191*36/8    2006������ٱ�Ϊ36    500��Ϊ�����ת�ض��ӵ�����
				
					/*pidλ�û�*/
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
					set_speed = pid_dial_pos.pos_out;
          Guodu_or_Stop_mode_flag = 0;
				  Reverse_mode_flag = 0;
//					set_speed = 432;
//					set_cnt = 1;
					
				  Guodu_or_Stop_mode_flag = 0;
					Reverse_mode_flag = 0;
      }break;
			
      case bopan_Duofa_mode://�̶��Ƕ�ģʽ   ������
      {
					set_cnt = 3;
					set_angle = -29488 * set_cnt ;
				
					/*pidλ�û�*/
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
					set_speed = pid_dial_pos.pos_out;
					
					Guodu_or_Stop_mode_flag = 0;
					Reverse_mode_flag = 0;
      }break;
			
      case bopan_Lianfa_mode://����ģʽ
      { 
//				set_speed=heat_limit();
				  if(RC_Ctl.rc.s2==1){
						set_speed = 270;
					}
					else if(RC_Ctl.rc.s2==2){
					set_speed = 270*20; //1HZ
					}
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
							gimbal_status.bopan_flag = 1;
							set_cnt = 0;
						}else 
						{			
							set_speed = -1000;
						}
				}
			}break;
			default :break;
    }
     /*�ٶȻ�*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
     /*�����������*/
		 Allocate_Motor(&hcan1,pid_dial_spd.pos_out);
 
		if(gimbal_status.bopan_lastmode != Mocha_Duofa_mode && gimbal_status.bopan_mode == Mocha_Duofa_mode)  
			 Shoot_status.last_cnt = Shoot_status.cnt;		 //ģʽ�л�Ϊ������෢ʱ�������ʱ�������ֵ(ÿ�ŵ������ֵΪ2)��
		else if(gimbal_status.bopan_lastmode != Mocha_danfa_mode && gimbal_status.bopan_mode == Mocha_danfa_mode)  
			 Shoot_status.danfa_cnt = Shoot_status.cnt;
		
		gimbal_status.bopan_lastmode = gimbal_status.bopan_mode; 

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
	fp32 max_value=1250;
	xLastWakeTime = xTaskGetTickCount();
	ramp_init(&shoot,0.01,1500,1050);//Ħ����б��           0.01Ϊб��  150��100Ϊ�����Сֵ(���ֵҲΪĿ��ֵ)
	
	for(;;)  //Ŀǰ��������Ħ���ֵ�����ת�� ����  ���12 ����18 1350  ����28 1800 ���12.5 ����20 1450 ����22 1800 ���13 ����18 1800
	         //ֱ�� ���13 ����18 1350   1450 ���13.5 ����20 1350 ����28 1450 ���14 ����20 1350 ���� 28 1550 ���12.5 ����18 1350 ���12 ����18 1350 ����27 1450
	{

		if(Mocha_flag==1){
		  if(flag_add==1){
		      max_value++;
		      flag_add=0;
	    }
	   if(flag_sub==1){
		     max_value--;
		   flag_sub=0;
	   }
			shoot.max_value = max_value;
		}
    else if(Mocha_flag==0){
		  shoot.max_value = 1050;
		}				
			ramp_calc(&shoot,1000);  //100Ϊ����ϵ��
			Friction_Wheel_Motor(shoot.out,shoot.out);
//	    pid_calc(&pid_moca_spd[0],moto_moca_get[0].speed_rpm ,-5400);//3508����Ħ����
//			pid_calc(&pid_moca_spd[1],moto_moca_get[1].speed_rpm ,5400);//3508����Ħ����
//		  Moca_Motor(&hcan2,pid_moca_spd[0].pos_out,pid_moca_spd[1].pos_out);//3508����Ħ����
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
    
    Shoot_status.val = Robot.heat.shoot_17_cooling_limit - Robot.heat.shoot_17_heat;//ʣ������
    

    
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
	
	   	if(gimbal_status.bopan_flag  == 1)
			{
		     /*�жϲ����Ƿ�ת��λ��*/			
					if(my_abs(moto_dial_get.total_angle) >= 29488 * Set_cnt )      //ת��һ�����裬ת����ת36/10 = 3.6 Ȧ
					{
							if((gimbal_status.bopan_mode == bopan_danfa_mode) && ( Set_cnt != 0) && (!Limit_switch_Control))
							{ 
								 Delay_flg = 1;
							}
							
							if((gimbal_status.bopan_mode != bopan_guodu_mode) && (gimbal_status.bopan_mode != bopan_Stop))   //������ģʽ����ΪSet_cntΪ0�����Բ��������Ƶ������
							{
								 Moto_dial_Angle_Reset();
						  }
							
							moto_dial_get.run_time = GetSystemTimer();                 //��ȡ���̿�ʼת����ʱ��
							gimbal_status.bopan_flag = 1;
					}
					else   //�ս��뵥�����෢�����������ϵ�ģʽ  �Ż������ж�
					{
							if( my_abs(moto_dial_get.cmd_time - moto_dial_get.run_time) > BLOCK_TIME )//��ת�ж�    
							{
									gimbal_status.bopan_flag = 0;	
							}
					}
			}
								
			if(gimbal_status.bopan_flag == 0)   
				gimbal_status.bopan_mode = bopan_fanzhuan_mode;//��ת
	
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
