/* 包含头文件----------------------------------------------------------------*/
#include "data_processing.h"
#include "SystemState.h"
#include "user_lib.h"
#include "main.h"
#include "AX-12A.h"
#include "gun_control.h"
/* 内部变量------------------------------------------------------------------*/
pid_t pid_minipc_yaw = {0};
pid_t pid_minipc_pit = {0};

uint8_t Reset_Mocha = 0;
uint8_t flag_add;
uint8_t flag_sub;

uint8_t test;
/* 内部函数原型声明-----------------------------------------------------------*/


void Minipc_Pid_Init()
{
		PID_struct_init(&pid_minipc_yaw, POSITION_PID, 50, 1, 1.0f,	0.00f, 0.0f);  
		PID_struct_init(&pid_minipc_pit, POSITION_PID, 6000, 5000, 0.5f,	0.00f, 0.0f);   
}


/* 任务主体部分 -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	遥控数据接收及处理任务
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void const * argument)
{
	  uint32_t NotifyValue;

		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
		NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //未有任务通知则进入堵塞状态去等待任务通知
		
    if(NotifyValue == 1)
		{
			RefreshTaskOutLineTime(RemoteDataTask_ON);
			
			Remote_Ctrl();   //接收机原始数据处理
			
//			Send_MiniPC_Data(gimbal_status.minipc_color,gimbal_status.minipc_mode,yaw_get.offset_angle,pit_get.offset_angle,0);
			CAN_Send_Remote(&hcan2,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
			
			switch(RC_Ctl.rc.s1)  //左拨钮
			{
					case 1: //上
					{
						gimbal_status.remote_mode = 0;
						MouseKeyControlProcess();
					}break; 
					case 2: //下
					{
						gimbal_status.remote_mode = 0;
				    RemoteControlProcess();
					}break; 
					case 3: //中
					{
						gimbal_status.remote_mode = 1;
					}break;  
					default :break;
			}
		
		}
		
		osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}

/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	与遥控器进行对接，对遥控器的数据进行处理，实现对底盘、云台、发射机构的控制
	*	@retval	
****************************************************************************************/
void RemoteControlProcess()  
{
		switch(RC_Ctl.rc.s2)
		{
		
//				case 1: //上
//				{
//						gimbal_status.gimbal_mode = 2;  
//					yaw_set.expect =yaw_set.expect + (RC_Ctl.rc.ch2 - 0x400) / 10;
//  		  	pit_set.expect = pit_set.expect + (0x400 - RC_Ctl.rc.ch3) / 20;	
//				}break; 
//				case 2: //下
//				{
//						gimbal_status.gimbal_mode = 2;  
//					yaw_set.expect=0;
//				}break; 
//				case 3: //中
//				{ 
//						gimbal_status.gimbal_mode = 2;  
//					yaw_set.expect=1000;	 
//				}break; 
//				default :break;
//			}
				
			
			
		
			case 3: //中
			{
				 gimbal_status.gimbal_mode = 2;            //编码器模式 
//				 Set_AX6(2,819,0x3ff);       //逆时针角度   240° 关
				gimbal_status.bopan_mode = bopan_Stop;
				Mocha_flag=0;
			}break; 
			case 2: //下
			{
						gimbal_status.gimbal_mode = 3;            //小陀螺模式  3
	//			 Set_AX6(2,819,0x3ff);       //逆时针角度   240° 关
				gimbal_status.bopan_mode = bopan_Stop;
			   Mocha_flag=0;
			}break; 
			case 1: //上
			{
						gimbal_status.gimbal_mode = 1;            //跟随模式  

//				 Set_AX6(2,512,0x3ff);       //逆时针角度   150°
         gimbal_status.bopan_mode =bopan_Stop ;
				Mocha_flag=0;
//				gimbal_status.gimbal_mode = 4;    //视觉模式
//				gimbal_status.minipc_mode = 1;    //自瞄模式
			}break; 
			
			
			
			
			
			
			default :break;
		}					
        
    if((gimbal_status.gimbal_mode == 1) || (gimbal_status.gimbal_mode == 3))            //陀螺仪模式
		{
			  yaw_tly_set.expect = yaw_tly_set.expect + (RC_Ctl.rc.ch2 - 0x400) / 10;	
				pit_set.expect = pit_set.expect + (0x400 - RC_Ctl.rc.ch3) / 20;	              //pit_set.expect_remote不受模式影响		
			
		}else if(gimbal_status.gimbal_mode == 2 )       // 编码器模式  
		{
				yaw_set.expect = yaw_set.expect + (RC_Ctl.rc.ch2 - 0x400) / 10;
		  	pit_set.expect = pit_set.expect + (0x400 - RC_Ctl.rc.ch3) / 20;	 
		}
//    Mocha_flag=1;
		
		if(pit_set.expect<-583) pit_set.expect=-583;
		else if(pit_set.expect>443)  pit_set.expect=443;


//				  if(RC_Ctl.rc.s2 == 1)            //上
//		{    
//			 gimbal_status.bopan_mode= bopan_danfa_mode;
//		}
//		else if(RC_Ctl.rc.s2== 2)       //下
//		{
//        gimbal_status.bopan_mode = bopan_Lianfa_mode;
//		}else                             //中
//		{
//        gimbal_status.bopan_mode = bopan_Stop;    
//		}		
}

/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	对键鼠的数据进行处理
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
	  static uint16_t delay=0;
	
 	
		if(F_Press){   //F键跟随
			test=1;
			gimbal_status.gimbal_mode = 1;  
		}
		if(CTRL_Press && F_Press){    //取消跟随模式后切换成分离
			test=2;
			gimbal_status.gimbal_mode = 2;  
		}
		
		
		if(R_Press){    //R键开弹仓
			Magazine_flag = 1;
		}
		if(CTRL_Press&&R_Press){
			Magazine_flag = 0;
		}

		
		if(Left_Press){   //左键发射一发
			if(delay>press_times && Left_Press){   //按键消抖
				gimbal_status.bopan_mode = bopan_danfa_mode;
				delay=0;
			}
			delay++;
		}
		
		if(Right_Press){    //右键连发
			gimbal_status.bopan_mode=bopan_Lianfa_mode;
		}
		
		if(C_Press){    //小陀螺模式
			gimbal_status.gimbal_mode=3;                                                                        
		}
		if(C_Press&&CTRL_Press){   //取消小陀螺后切换成跟随模式
			gimbal_status.gimbal_mode=1;
		}
		
		if(X_Press){   //X键 45度模式
			
			gimbal_status.gimbal_mode = 6;  
		}
		if(CTRL_Press && X_Press){    //取消45度模式后切换成跟随
			
			gimbal_status.gimbal_mode = 1; 
		}
//		
		if(Z_Press){
			Mocha_flag=1;
		}
		if(CTRL_Press && Z_Press){    
			
			Mocha_flag = 0; 
		}
		if(Z_Press){
				flag_add=1;
			}
			if(G_Press){
				flag_sub=1;
			}
		
    
	
	if((gimbal_status.gimbal_mode == 1) || (gimbal_status.gimbal_mode == 3)|| (gimbal_status.gimbal_mode == 6))            //陀螺仪模式
		{
			  yaw_tly_set.expect = yaw_tly_set.expect + RC_Ctl.mouse.x*1.2f;	
				pit_set.expect = pit_set.expect - RC_Ctl.mouse.y *1.2f;	              //pit_set.expect_remote不受模式影响		
			
		}else if(gimbal_status.gimbal_mode == 2 )       // 编码器模式  
		{
				yaw_set.expect = yaw_set.expect + RC_Ctl.mouse.x*1.0f;
		  	pit_set.expect = pit_set.expect - RC_Ctl.mouse.y *1.0f;	  
		}
		if(pit_set.expect<-583) pit_set.expect=-583;
		else if(pit_set.expect>443)  pit_set.expect=443;


             
		
}

/***************************************************************************************
**
	*	@brief	hard_brak()
	*	@param
	*	@supplement	紧急停止函数
	*	@retval	
****************************************************************************************/
void hard_brak()
{

	
}





/***************************************************************************************
**
	*	@brief	MiniPC_Data_task(void const * argument)
	*	@param
	*	@supplement	视觉数据处理任务
	*	@retval	
****************************************************************************************/
void MiniPC_Data_task(void const * argument)
{
	static uint8_t frist_flag = 0;
  
  static int16_t  frist_angle;//pit的角度偏差
	minipc_rx.state_flag = 0;
	minipc_rx.angle_pit  = 0;
	minipc_rx.angle_yaw  = 0;
  uint32_t NotifyValue;
	Minipc_Pid_Init();
	osDelay(4000);//延时4000ms，等待云台任务进入正常运转后才开始视觉数据接收
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
		frist_angle  =  pit_get.total_angle-pit_set.expect_pc;
	for(;;)
	{
	   NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{
			Get_MiniPC_Data();  //获取视觉传来的信息
			
			switch(gimbal_status.minipc_mode)
			{
					case 1 ://自瞄
					{
						if(my_abs(minipc_rx.angle_yaw) > 640 || my_abs(minipc_rx.angle_pit) > 360)  //pc数据异常
						{
							
						}else
						{
								if(minipc_rx.state_flag == 1)    //视觉检测到瞄准至目标附近后，不再调整位置
								{
									yaw_set.expect_pc -= minipc_rx.angle_yaw;    //yaw右边为负
									pit_set.expect_pc -= minipc_rx.angle_pit;    //               
									
								}else if(minipc_rx.state_flag == 2)
								{
									yaw_set.expect_pc -= minipc_rx.angle_yaw;    //yaw右边为负
									pit_set.expect_pc -= minipc_rx.angle_pit;    // 
									gimbal_status.bopan_mode = bopan_danfa_mode;
								}      
						}
						minipc_rx.angle_yaw = 0;
						minipc_rx.angle_pit = 0; 
					}break ;
					
					case 2 ://打符						
					{
						
//						if(my_abs(minipc_rx.angle_yaw)>500 || my_abs(minipc_rx.angle_yaw)>500)  //pc数据异常
//						{
//						}else
//						{
//								Shoot_status.remote_mode = 1; //先启动抹茶轮
//								if( frist_flag == 0)
//								{
//										if(shoot.out == 128 )  frist_flag=1;
//								
//										yaw_set.expect_pc = minipc_rx.angle_yaw;
//										pit_set.expect_pc = minipc_rx.angle_pit;
//								}
//							
//								if(minipc_rx.state_flag)//检测到不同装甲块时立即调准位置
//								{
//										yaw_set.expect_pc = minipc_rx.angle_yaw;
//										pit_set.expect_pc = minipc_rx.angle_pit;
//								}
//								else if( frist_flag )//误差不能设置的太大
//								{
//										yaw_set.expect_pc = minipc_rx.angle_yaw ;               
//										pit_set.expect_pc = minipc_rx.angle_pit ;
//										gimbal_status.bopan_mode = 1;
//								}else     
//										gimbal_status.bopan_mode = 0;
//								
//						 }break;
					 }
					 default : break;
				}
			 
		}

	  osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
  }
}
