/* ����ͷ�ļ�----------------------------------------------------------------*/
#include "chassis_control.h"
#include "Power_restriction.h"
#include "data_processing.h"
#include "SystemState.h"
#include "arm_math.h"

/* �ڲ�����------------------------------------------------------------------*/
pid_t pid_3508_pos;     		 //���̵��λ�û�
pid_t pid_3508_spd[4];			 //���̵���ٶȻ�
pid_t pid_3508_current[4];	 //���̵����������	
pid_t pid_chassis_follow = {0};//���̸���λ�û�

moto3508_type  moto_3508_set = {.flag = 0}; 
Chassis_Status_t chassis_status;

static float Current_set[4] = {0};  //���ݸ��������ƵĻ���
extern float Total_power;
float total_set;
float send;
//���Ա���
int16_t angle[2];
uint32_t total_current;
float real_current[4];

/* �ڲ�����ԭ������----------------------------------------------------------*/
void Chassis_pid_init(void)
{
	
	 PID_struct_init(&pid_3508_pos, POSITION_PID, 10000, 1000, 1.5f,	0.01f,	2.0f);  // motos angular rate closeloop.pid:1.5,0.0,20.0
//	 pid_3508_pos.deadband = 150;
	
	 PID_struct_init(&pid_chassis_follow, POSITION_PID,10000,1000, 50.0f, 0.31f , 0.01f);
//	 pid_chassis_follow.deadband = 500;

	
	 for(int i=0; i<4; i++)
	 { 
		 PID_struct_init(&pid_3508_spd[i], POSITION_PID, 8000,5000,1.0f,0.0f,0.0f);  //4 motos angular rate closeloop.
   }
	
		PID_struct_init(&pid_3508_current[0], POSITION_PID, 6000, 500, 0.6f, 0.01f, 0.01f);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[1], POSITION_PID, 6000, 500, 0.6f, 0.01f,	0.01f);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[2], POSITION_PID, 6000, 500, 0.6f, 0.01f, 0.01f);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[3], POSITION_PID, 6000, 500, 0.6f, 0.01f,	0.01f);  //4 motos angular rate closeloop.
  PID_struct_init(&pid_3508_Current, POSITION_PID, 3000, 1500, 0.8f, 0.001f,	2.01f);
			
}
/* �������岿�� -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Chassis_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	���̿�������
	*	@retval	
****************************************************************************************/
void Chassis_Contrl_Task(void const * argument)
{
	static float  wheel[4] = {0,0,0,0};
	static int32_t Angle_Mid_err;
	static int32_t Angle_45degree_err;
	static float absolute_angle_degree;
	static float absolute_angle_degree_45;
	static int16_t Speed_W;
  static int16_t Speed_X;
  static int16_t Speed_Y;

	float pSinVal;
	float pCosVal;

	osDelay(200);//��ʱ200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	Chassis_pid_init();
	
	for(;;)
	{
		 chassis_status.chassis_mode =gimbal_status.gimbal_mode ;//gimbal_status.gimbal_mode
		

		
		switch(chassis_status.chassis_mode)
		{
			case 1:		//���̸���
			{
					Angle_Mid_err = Middle_Angle - moto_gimbal_get.angle;
		      absolute_angle_degree = Absolute_angle_Conversion(Angle_Mid_err);   //ת���Ƕ�
			   if(ABS(absolute_angle_degree) < 3.3f)  //�ṩһ��5��ĽǶ�����
		     {
			      Speed_W = 0;
		     }else
		     {
					  //void power_limit(float  Current_get[4]);
			      Speed_W = pid_calc(&pid_chassis_follow, absolute_angle_degree, 0);
//					 Speed_W = 0;
		     }
				 motor_move_setvmmps(wheel, moto_3508_set.dstVmmps_X, moto_3508_set.dstVmmps_Y, Speed_W);
	    }break;
			
			case 2:    //����
			{
				 //void power_limit(float  Current_get[4]);
				 Speed_W = moto_3508_set.dstVmmps_W;
				 motor_move_setvmmps(wheel, moto_3508_set.dstVmmps_X, moto_3508_set.dstVmmps_Y, Speed_W);
			}break;
			
			case 3:   //С����
			{
				// void power_limit(float  Current_get[4]);
				if(level==1){
					 Speed_W = 3000;   //��3000��Ϊ0�����ڵ���  2020-07-04
				}
				else if(level==2){
					 Speed_W =4500;   //��3000��Ϊ0�����ڵ���  2020-07-04
				}
				Angle_Mid_err = Middle_Angle - moto_gimbal_get.angle;
		    absolute_angle_degree = Absolute_angle_Conversion(Angle_Mid_err);   //ת���Ƕ�
				
				 arm_sin_cos_f32(absolute_angle_degree, &pSinVal, &pCosVal);
				 Speed_X = moto_3508_set.dstVmmps_X * pCosVal - moto_3508_set.dstVmmps_Y * pSinVal;
         Speed_Y = moto_3508_set.dstVmmps_X * pSinVal + moto_3508_set.dstVmmps_Y * pCosVal;
				
				 motor_move_setvmmps(wheel, Speed_X, Speed_Y, Speed_W);
			}break;
			
			case 4:    //����
			{
				// void power_limit(float  Current_get[4]);
				 Speed_W = 0;
				 motor_move_setvmmps(wheel, moto_3508_set.dstVmmps_X, moto_3508_set.dstVmmps_Y, Speed_W);
			}break;
			
			case 5:  //��������
			{
				Speed_W = 0;
				 motor_move_setvmmps(wheel, moto_3508_set.dstVmmps_X, moto_3508_set.dstVmmps_Y, Speed_W);
			}break;
			
			case 6:  //45�ȵж�ģʽ
			{
				  Angle_45degree_err = Degree45_Angle - moto_gimbal_get.angle;
		      absolute_angle_degree_45 = Absolute_angle_Conversion(Angle_45degree_err);   //ת���Ƕ�
				
				Angle_Mid_err = Middle_Angle - moto_gimbal_get.angle;
				absolute_angle_degree=Absolute_angle_Conversion(Angle_Mid_err);
				
				if(ABS(absolute_angle_degree_45) < 3.3f)  //�ṩһ��5��ĽǶ�����
		     {
			      Speed_W = 0;
		     }else
		     {
					  //void power_limit(float  Current_get[4]);
			      Speed_W = pid_calc(&pid_chassis_follow, absolute_angle_degree_45, 0);
//					 Speed_W = 0;
		     }
				 arm_sin_cos_f32(absolute_angle_degree, &pSinVal, &pCosVal);
				 Speed_X = moto_3508_set.dstVmmps_X * pCosVal - moto_3508_set.dstVmmps_Y * pSinVal;
         Speed_Y = moto_3508_set.dstVmmps_X * pSinVal + moto_3508_set.dstVmmps_Y * pCosVal;
				 
				 motor_move_setvmmps(wheel, Speed_X, Speed_Y, Speed_W);
				
			}break;
			
			default:break;
			
	  }
		for(int i=0; i<4; i++)
			{
				pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, wheel[i]);//��Ŀ��ֵ�ͷ��ص����ݽ���
			}
			   
			
			
				all_sent = (ABS(pid_3508_spd[0].pos_out) 
			        + ABS(pid_3508_spd[1].pos_out)
       			   + ABS(pid_3508_spd[2].pos_out)
          			+ ABS(pid_3508_spd[3].pos_out));//��Ҫ�����ĵ���ֵ

	   	super_cap (Robot.Chassis_Power.chassis_Power,
			            Robot.Chassis_Power.Chassis_Power_buffer,
			            Get_103.CAP_Voltage, Get_103.CAP_Current ,
			            Get_103.ALL_Current * Robot.Chassis_Power.Chassis_Volt);
//			for(int32_t m=0;m<4000000;)
//			m++;
			osDelay(50);
			HAL_UART_Transmit(&huart1, (uint8_t *)&Send_103, 8, 15); 

		Chassis_Motor(&hcan1,
											pid_3508_spd[0].pos_out,
											pid_3508_spd[1].pos_out, 
											pid_3508_spd[2].pos_out, 
											pid_3508_spd[3].pos_out);

	 		osDelayUntil(&xLastWakeTime, 10);
		}
}
	 
		
//		  motor_move_setvmmps(wheel, moto_3508_set.dstVmmps_X, moto_3508_set.dstVmmps_Y, moto_3508_set.dstVmmps_W);
			
//	   for(int i = 0; i < 4; i++)
//		 {		
//			  pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, wheel[i]);
//		 }
		 
//		 total_current = ABS(moto_chassis_get[0].real_current) + ABS(moto_chassis_get[1].real_current) +
//		                   ABS(moto_chassis_get[2].real_current) + ABS(moto_chassis_get[3].real_current);
//		 
//		 total_set = ABS(pid_3508_spd[0].pos_out) + ABS(pid_3508_spd[1].pos_out) + 
//		 ABS(pid_3508_spd[2].pos_out) + ABS(pid_3508_spd[3].pos_out);
//		 
//		 if(Total_power > 70)
//		 {
//			 pid_3508_spd[0].pos_out = (pid_3508_spd[0].pos_out / (total_set + 1.0f)) * 8000; 
//			 pid_3508_spd[1].pos_out = (pid_3508_spd[1].pos_out / (total_set + 1.0f)) * 8000; 
//			 pid_3508_spd[2].pos_out = (pid_3508_spd[2].pos_out / (total_set + 1.0f)) * 8000; 
//			 pid_3508_spd[3].pos_out = (pid_3508_spd[3].pos_out / (total_set + 1.0f)) * 8000; 
//		 }
		 
//		 
//		 if(gimbal_status.remote_mode == 1)
//		 {
//			 Chassis_Motor(&hcan1, 0, 0, 0, 0);
//		 }else
//		 {			 
//			 Chassis_Motor(&hcan1,
//													pid_3508_spd[0].pos_out,
//													pid_3508_spd[1].pos_out, 
//													pid_3508_spd[2].pos_out, 
//													pid_3508_spd[3].pos_out);	
//		 }			 
//		
//		 osDelayUntil(&xLastWakeTime, CHASSIS_PERIOD);
//	}
//}


float Absolute_angle_Conversion(int32_t angle)      //ת��Ϊ���ԽǶ�,��ת����λ������е�Ƕ�ת����-180�㡪180�㣩
{
	  float absolute_angle;
	  while(1)
		{
			 if(angle < -4096)
			 {
				  angle += 8192;
			 }else if(angle > 4096)
			 {
				  angle -= 8192;
			 }else
			 {
				  break;
			 }
		}
		absolute_angle = (float)angle * 360 / 8192;
		return absolute_angle;
}
