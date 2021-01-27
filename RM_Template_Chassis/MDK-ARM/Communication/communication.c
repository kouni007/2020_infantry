/*******************************************************************************
                      ��Ȩ���� (C), 2017-, Mushiny
 *******************************************************************************
  �� �� ��   : communication.c
  �� �� ��   : ����
  ��    ��   : liyifeng
  ��������   : 2018��7��
  ����޸�   :
  ��������   : ����ͨ�ſ�
  �����б�   :
*******************************************************************************/
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "communication.h"
#include "Power_restriction.h"


/* �ڲ��Զ����������� --------------------------------------------------------*/
 uint8_t USART1_RX_DATA[(SizeofRemote)];//ң��
 uint16_t USART1_RX_NUM;
 uint8_t USART6_RX_DATA[(100)];//����ϵͳ
 uint16_t USART6_RX_NUM;
 uint8_t UART8_RX_DATA[(SizeofJY901)];//���������
 uint16_t UART8_RX_NUM;
 struct STime			stcTime;
 struct SAcc 			stcAcc;
 struct SGyro 		stcGyro;
 struct SAngle 		stcAngle;
 struct SMag 			stcMag;
 struct SDStatus  stcDStatus;
 struct SPress 		stcPress;
 struct SLonLat 	stcLonLat;
 struct SGPSV 		stcGPSV;
 struct SQ        stcQ;
/* �ڲ��궨�� ----------------------------------------------------------------*/

/* ���������Ϣ����-----------------------------------------------------------*/

/* �ڲ���������---------------------------------------------------------------*/

/* �ⲿ�������� --------------------------------------------------------------*/

/* �ⲿ����ԭ������ ----------------------------------------------------------*/

/* �ڲ����� ------------------------------------------------------------------*/
//���������
JY901_t   ptr_jy901_t_yaw =  {0};
JY901_t   ptr_jy901_t_pit =  {0};
JY901_t  	ptr_jy901_t_angular_velocity = {0};

XTLY tly;

//��̨��Ϣ
Gimbal_Status_t gimbal_status;

//mpu6500
uint8_t MPU_id = 0;
int16_t gy_data_filter[5];
int16_t gz_data_filter[5];
IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

/* �ڲ�����ԭ������ ----------------------------------------------------------*/

/* �������岿�� --------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	JY901_Data_Pro()
	*	@param
	*	@supplement	���ж��б����ã����ڴ��ڽ��������ǵ����ݣ��������ݽ��д���
	*	@retval	
****************************************************************************************/


/**********************jy601***********************/

void select(uint8_t * buff,uint8_t i)
{
	switch(buff[i+1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
	{
//		case 0x50:	memcpy(&stcTime,&buff[i+2],8);		break;
		case 0x51:	memcpy(&stcAcc,&buff[i+2],8);			break;
		case 0x52:	memcpy(&stcGyro,&buff[i+2],8);		break;
		case 0x53:	memcpy(&stcAngle,&buff[i+2],8);		break;
//		case 0x54:	memcpy(&stcMag,&buff[i+2],8);			break;
//		case 0x55:	memcpy(&stcDStatus,&buff[i+2],8);	break;
//		case 0x56:	memcpy(&stcPress,&buff[i+2],8);		break;
//		case 0x57:	memcpy(&stcLonLat,&buff[i+2],8);	break;
//		case 0x58:	memcpy(&stcGPSV,&buff[i+2],8);		break;
//		case 0x59:	memcpy(&stcQ,&buff[i+2],8);				break;
	}
}

void JY901_Data_Pro()
{	
	uint8_t  * buff = UART8_RX_DATA;//����8
	
	//jy601
	switch(buff[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
	{

		case 0x52:	memcpy(&stcGyro,&buff[2],8);		  break;    //memcpy����������
		case 0x53:	memcpy(&stcAngle,&buff[2],8);		  break;

	}
	
	
		switch(buff[12])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
	{

		case 0x52:	memcpy(&stcGyro,&buff[13],8);		  break;
		case 0x53:	memcpy(&stcAngle,&buff[13],8);		  break;

	}
	
		
		switch(buff[23])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
	{

		case 0x52:	memcpy(&stcGyro,&buff[24],8);		  break;
		case 0x53:	memcpy(&stcAngle,&buff[24],8);		  break;

	}
	
	//jy901
	/*Ѱ�ҵ�һ��֡ͷλ��*/
//	for(uint8_t i = 0;i < SizeofJY901/3;i++)
//	{
//		if(buff[i] == 0x55)
//		{
//			select(buff,i);
//			ucRxCnt = i;
//			break;
//		}
//	}
//	/*������ɶλ���*/
//			memcpy(buff_transition,&buff_last[ucRxCnt + 11],11 - ucRxCnt - 1);
//			memcpy(&buff_transition[11 - ucRxCnt - 1],buff_last,ucRxCnt + 1);
//			//�����ϴλ���ֵ
//			memcpy(buff_last,buff,11);
//			
//			select(buff_transition,0);

//	printf("���ٶ�:%3f,�Ƕ�:%3f\n\r",(float)stcGyro.w[0]/32768*2000,(float)stcAngle.Angle[0]/32768*180);	
	

	
			ptr_jy901_t_pit.JY901_angle = (float)stcAngle.Angle[1] * 0.005493f;
			ptr_jy901_t_yaw.JY901_angle = (float)stcAngle.Angle[2] * 0.005493f;	
	   
//		if(pritnf_JY901){    //������
//			printf("gz=%f\n",ptr_jy901_t_yaw.JY901_angle);
//    	float *ptr = NULL; //��ʼ��ָ��
//			ptr = &(ptr_jy901_t_yaw.final_angle);	
//			/*������ʾ��������������*/
//			vcan_sendware((uint8_t *)ptr,sizeof(ptr_jy901_t_yaw.final_angle));
//		 }
		if(ptr_jy901_t_yaw.times > 5)
			{
				ptr_jy901_t_yaw.times = 6;
				ptr_jy901_t_yaw.err = ptr_jy901_t_yaw.JY901_angle-ptr_jy901_t_yaw.JY901_angle_last;
			if(ptr_jy901_t_yaw.err < -180)  
				ptr_jy901_t_yaw.angle_round++;
			else if(ptr_jy901_t_yaw.err > 180)  
				ptr_jy901_t_yaw.angle_round--;
			ptr_jy901_t_yaw.total_angle = (ptr_jy901_t_yaw.angle_round*360+ptr_jy901_t_yaw.JY901_angle-ptr_jy901_t_yaw.first_angle) * 22.75f;
			
			ptr_jy901_t_pit.err = ptr_jy901_t_pit.JY901_angle-ptr_jy901_t_pit.JY901_angle_last;
			if(ptr_jy901_t_pit.err < -180) 
				ptr_jy901_t_pit.angle_round++;
			else if(ptr_jy901_t_pit.err > 180)
				ptr_jy901_t_pit.angle_round--;
			//�������ս��
		  ptr_jy901_t_pit.total_angle = (ptr_jy901_t_pit.angle_round*360+ptr_jy901_t_pit.JY901_angle-ptr_jy901_t_pit.first_angle);
//		  ptr_jy901_t_pit.total_angle=(ptr_jy901_t_pit.JY901_angle-ptr_jy901_t_pit.first_angle);

			}
			else 
			{
				ptr_jy901_t_yaw.first_angle = ptr_jy901_t_yaw.JY901_angle;
				ptr_jy901_t_pit.first_angle = ptr_jy901_t_pit.JY901_angle;
			}
			

			ptr_jy901_t_yaw.JY901_angle_last = ptr_jy901_t_yaw.JY901_angle;
			ptr_jy901_t_pit.JY901_angle_last = ptr_jy901_t_pit.JY901_angle;
			ptr_jy901_t_yaw.times++;

		
			

			ptr_jy901_t_angular_velocity.vx = stcGyro.w[0] * 0.06103516f;
			ptr_jy901_t_angular_velocity.vy = stcGyro.w[1] * 0.06103516f;
			ptr_jy901_t_angular_velocity.vz = stcGyro.w[2] * 0.06103516f;			
			
//			ptr_jy901_t_angular_velocity.vz = Limit_filter(ptr_jy901_t_angular_velocity.vz_last,ptr_jy901_t_angular_velocity.vz,700);
//			ptr_jy901_t_angular_velocity.vy = Limit_filter(ptr_jy901_t_angular_velocity.vy_last,ptr_jy901_t_angular_velocity.vy,700);
			
			ptr_jy901_t_angular_velocity.vy_last = ptr_jy901_t_angular_velocity.vy;
			ptr_jy901_t_angular_velocity.vz_last = ptr_jy901_t_angular_velocity.vz;
			



}

void tly_Pro()
{
	 static uint16_t tly_cnt=0;
		uint8_t  * buff = UART8_RX_DATA;//����8
	  if(buff[0] == 0x4d)
		{
			tly_cnt++;
			tly.yaw_angle=buff[1]<<8|buff[2];
			if(tly.yaw_angle>0XF8F7)
			{
				tly.yaw_angle=(tly.yaw_angle-65535);
			}else tly.yaw_angle=tly.yaw_angle;       //��ʵ�Ƕ� * 10
		}
		
		if(tly_cnt>20)
		{
			tly_cnt=21;
			
			tly.angle_err=tly.yaw_angle-tly.yaw_last_angle;
			if(tly.angle_err<-1800) tly.angle_cnt++;
			else if(tly.angle_err>1800) tly.angle_cnt--;
			
			tly.final_angle = tly.angle_cnt*3600+tly.yaw_angle-tly.first_angle;
			
			
		}else 	tly.first_angle=tly.yaw_angle;
	  tly.yaw_last_angle=tly.yaw_angle;
	 
}



/**********************jy601***********************/



//void select(uint8_t * buff,uint8_t i)
//{
//	switch(buff[i+1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
//	{
//		case 0x50:	memcpy(&stcTime,&buff[i+2],8);		break;
//		case 0x51:	memcpy(&stcAcc,&buff[i+2],8);			break;
//		case 0x52:	memcpy(&stcGyro,&buff[i+2],8);		break;
//		case 0x53:	memcpy(&stcAngle,&buff[i+2],8);		break;
//		case 0x54:	memcpy(&stcMag,&buff[i+2],8);			break;
//		case 0x55:	memcpy(&stcDStatus,&buff[i+2],8);	break;
//		case 0x56:	memcpy(&stcPress,&buff[i+2],8);		break;
//		case 0x57:	memcpy(&stcLonLat,&buff[i+2],8);	break;
//		case 0x58:	memcpy(&stcGPSV,&buff[i+2],8);		break;
//		case 0x59:	memcpy(&stcQ,&buff[i+2],8);				break;
//	}
//}

//void JY901_Data_Pro()
//{	
//  int16_t data_sum=0;
//	
//	uint8_t  * buff = UART8_RX_DATA;//����8
//	/*Ѱ�ҵ�һ��֡ͷλ��*/
//	for(uint8_t i = 0;i < SizeofJY901;i++)
//	{
//		if(buff[i] == 0x55)
//		{
//			select(buff,i);
//			ucRxCnt = i;
//			break;
//		}
//	}
//	/*������ɶλ���*/
//			memcpy(buff_transition,&buff_last[ucRxCnt + 11],11 - ucRxCnt - 1);
//			memcpy(&buff_transition[11 - ucRxCnt - 1],buff_last,ucRxCnt + 1);
//			//�����ϴλ���ֵ
//			memcpy(buff_last,buff,11);
//			
//			select(buff_transition,0);
//			ptr_jy901_t_angular_velocity.vx = stcGyro.w[0] * 0.06103516f;
//			ptr_jy901_t_angular_velocity.vy = stcGyro.w[1] * 0.06103516f;
//			ptr_jy901_t_angular_velocity.vz = stcGyro.w[2] * 0.06103516f;			
//			
//			ptr_jy901_t_angular_velocity.vz = Limit_filter(ptr_jy901_t_angular_velocity.vz_last,ptr_jy901_t_angular_velocity.vz,700);
//			ptr_jy901_t_angular_velocity.vy = Limit_filter(ptr_jy901_t_angular_velocity.vy_last,ptr_jy901_t_angular_velocity.vy,700);
//			
//			ptr_jy901_t_angular_velocity.vy_last = ptr_jy901_t_angular_velocity.vy;
//			ptr_jy901_t_angular_velocity.vz_last = ptr_jy901_t_angular_velocity.vz;
//			



//}

/***************************************************************************************
**
	*	@brief	Remote_Ctrl(void)
	*	@param
	*	@supplement	ʹ�ô��ڽ���ң��������
	*	@retval	
****************************************************************************************/
void Remote_Ctrl(void)//ң�����ݽ���
{ 
	 uint8_t  * buff = USART1_RX_DATA;
	 RC_Ctl.rc.ch0 = (buff[0]| (buff[1] << 8)) & 0x07ff; //!< Channel 0 
	 RC_Ctl.rc.ch1 = ((buff[1] >> 3) | (buff[2] << 5)) & 0x07ff; //!< Channel 1 
	 RC_Ctl.rc.ch2 = ((buff[2] >> 6) | (buff[3] << 2) | //!< Channel 2 
	 (buff[4] << 10)) & 0x07ff; 
	 RC_Ctl.rc.ch3 = ((buff[4] >> 1) | (buff[5] << 7)) & 0x07ff; //!< Channel 3 
	 RC_Ctl.rc.s1 = ((buff[5] >> 4)& 0x000C) >> 2; //!< Switch left 
	 RC_Ctl.rc.s2 = ((buff[5] >> 4)& 0x0003); //!< Switch right 
	 RC_Ctl.mouse.x = buff[6] | (buff[7] << 8); //!< Mouse X axis 
	 RC_Ctl.mouse.y = buff[8] | (buff[9] << 8); //!< Mouse Y axis 
	 RC_Ctl.mouse.z = buff[10] | (buff[11] << 8); //!< Mouse Z axis 
	 RC_Ctl.mouse.press_l = buff[12]; //!< Mouse Left Is Press ? 
	 RC_Ctl.mouse.press_r = buff[13]; //!< Mouse Right Is Press ? 
	 RC_Ctl.key.v = buff[14] | (buff[15] << 8); //!< KeyBoard value 
}

/***************************************************************************************
**
	*	@brief	Remote_Ctrl(void)
	*	@param
	*	@supplement	ʹ�ô��ڽ���ң��������
	*	@retval	
****************************************************************************************/
void Remote_Disable(void)//ң�����ݽ���
{ 

	 RC_Ctl.rc.ch0 = 0; //!< Channel 0 
	 RC_Ctl.rc.ch1 = 0; //!< Channel 1 
	 RC_Ctl.rc.ch2 = 0; 
	 RC_Ctl.rc.ch3 = 0; //!< Channel 3 
	 RC_Ctl.rc.s1 = 0; //!< Switch left 
	 RC_Ctl.rc.s2 = 0; //!< Switch right 
	 RC_Ctl.mouse.x = 0; //!< Mouse X axis 
	 RC_Ctl.mouse.y = 0; //!< Mouse Y axis 
	 RC_Ctl.mouse.z = 0; //!< Mouse Z axis 
	 RC_Ctl.mouse.press_l = 0; //!< Mouse Left Is Press ? 
	 RC_Ctl.mouse.press_r = 0; //!< Mouse Right Is Press ? 
	 RC_Ctl.key.v = 0; //!< KeyBoard value 
}



