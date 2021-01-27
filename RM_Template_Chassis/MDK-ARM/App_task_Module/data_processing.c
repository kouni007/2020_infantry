/* 包含头文件----------------------------------------------------------------*/
#include "data_processing.h"
#include "SystemState.h"
#include "chassis_control.h"
#include "user_lib.h"
#include "main.h"
#include "AX-12A.h"

/* 内部变量------------------------------------------------------------------*/
pid_t pid_minipc_yaw = {0};
pid_t pid_minipc_pit = {0};
Mode_Set Shoot_mouse = {0};
int16_t Y_speed_max = 2500;
int16_t Y_speed_min = -2500; 
int16_t X_speed_max=2500;
int16_t X_speed_min=-2500;
int16_t W_speed_max = 2000;
int16_t W_speed_min = -2000; 
/* 内部函数原型声明-----------------------------------------------------------*/
uint8_t level;  //自定义等级

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
			 
			switch(RC_Ctl.rc.s1)  //左拨钮
			{
					case 1: //上
					{
						MouseKeyControlProcess();
					}break; 
					case 2: //下
					{
				    RemoteControlProcess();
					}break; 
					case 3: //中
					{

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
	 moto_3508_set.dstVmmps_X = ((0x400 - RC_Ctl.rc.ch0 ) * 5);//ch0  右摇杆—左右4   9
	 moto_3508_set.dstVmmps_Y = ((0x400 - RC_Ctl.rc.ch1 ) * 7);//ch1  右摇杆—上下		20

    //	50w 数据	 moto_3508_set.dstVmmps_X = ((0x400 - RC_Ctl.rc.ch0 ) * 5);//ch0  右摇杆—左右4   9
	  //             moto_3508_set.dstVmmps_Y = ((0x400 - RC_Ctl.rc.ch1 ) * 7);//ch1  右摇杆—上下		20
    
	  //80w  数据    moto_3508_set.dstVmmps_X = ((0x400 - RC_Ctl.rc.ch0 ) * 6);//ch0  右摇杆—左右4
	  //             moto_3508_set.dstVmmps_Y = ((0x400 - RC_Ctl.rc.ch1 ) * 13);//ch1  右摇杆—上下		



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
	
		if(V_Press){
			level=1;
		}
		if(B_Press){
			level=2;
		}
		if(SHIFT_Press)
			{
					Y_speed_max = 5000;  //(NORMAL_SPEED_MAX)*3.5;    //*4.188596/10000 为m/s
					Y_speed_min = -5000; //(NORMAL_SPEED_MIN)*3.5;
				  X_speed_max=5000;
				  X_speed_min=5000;
				
			}else 
			{
				if(level==1){
					Y_speed_max = 4620;  //(NORMAL_SPEED_MAX)*3.5;
					Y_speed_min = -4620; //(NORMAL_SPEED_MIN)*3.5;
					
					X_speed_max= 3300;
					X_speed_min= -3300;
				}
				else if(level==2){
					Y_speed_max = 8580;  //(NORMAL_SPEED_MAX)*3.5;
					Y_speed_min = -8580; //(NORMAL_SPEED_MIN)*3.5;
					
					X_speed_max= 3960;
					X_speed_min= -3960;
				}

			}
			
		  if(CTRL_Press)
		  {
					Y_speed_max = 2000;  //(NORMAL_SPEED_MAX)*3.5;
					Y_speed_min = -2000; //(NORMAL_SPEED_MIN)*3.5;
				
					X_speed_max=2000;
					X_speed_min=-2000;
		  }
 
      
			if(W_Press)   				
				 moto_3508_set.dstVmmps_Y -= ACC_SPEED;//按下W键
			else if(S_Press)                 
				 moto_3508_set.dstVmmps_Y += ACC_SPEED;//按下S键
			else {
				if(moto_3508_set.dstVmmps_Y>-DEC_SPEED&&moto_3508_set.dstVmmps_Y<DEC_SPEED) 	 moto_3508_set.dstVmmps_Y = 0;
        if(moto_3508_set.dstVmmps_Y>0) 	                   moto_3508_set.dstVmmps_Y -= DEC_SPEED;
        if(moto_3508_set.dstVmmps_Y<0) 		                 moto_3508_set.dstVmmps_Y += DEC_SPEED;
			
			}
			
			if(chassis_status.chassis_mode==2)
			{
				if(Q_Press) moto_3508_set.dstVmmps_W -= ACC_W_SPEED;
				else if(E_Press) moto_3508_set.dstVmmps_W += ACC_W_SPEED;
				else{
					 if(moto_3508_set.dstVmmps_W>-DEC_W_SPEED&&moto_3508_set.dstVmmps_W<DEC_W_SPEED) 	 moto_3508_set.dstVmmps_W = 0;
           if(moto_3508_set.dstVmmps_W>0) 	                   moto_3508_set.dstVmmps_W -= DEC_W_SPEED;
           if(moto_3508_set.dstVmmps_W<0) 		                 moto_3508_set.dstVmmps_W += DEC_W_SPEED;
				}
			}
			
			
			if(moto_3508_set.dstVmmps_Y < Y_speed_min)  
			{					
				moto_3508_set.dstVmmps_Y = Y_speed_min;
			}else if(moto_3508_set.dstVmmps_Y > Y_speed_max)  
			{				
				 moto_3508_set.dstVmmps_Y = Y_speed_max;
			}


			if(D_Press)                       
				 moto_3508_set.dstVmmps_X -= ACC_SPEED; //按下D键
			else if(A_Press)    		         
				 moto_3508_set.dstVmmps_X += ACC_SPEED;//按下A键
			else {
            if(moto_3508_set.dstVmmps_X>-DEC_SPEED&&moto_3508_set.dstVmmps_X<DEC_SPEED) 		moto_3508_set.dstVmmps_X = 0;		
            if(moto_3508_set.dstVmmps_X>0) 	                   moto_3508_set.dstVmmps_X -= DEC_SPEED;
            if(moto_3508_set.dstVmmps_X<0) 		                  moto_3508_set.dstVmmps_X += DEC_SPEED;
            
			}
				 
			
			
			if(moto_3508_set.dstVmmps_X < X_speed_min)  
			{					
				 moto_3508_set.dstVmmps_X = X_speed_min;
			}else if(moto_3508_set.dstVmmps_X > X_speed_max)  
			{				
				 moto_3508_set.dstVmmps_X = X_speed_max;
			}
	
	
	/*
		if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT )
			{
					XY_speed_max = 6000;  //(NORMAL_SPEED_MAX)*3.5;    //*4.188596/10000 为m/s
					XY_speed_min = -6000; //(NORMAL_SPEED_MIN)*3.5;
			}else 
			{
					XY_speed_max = 8594;  //(NORMAL_SPEED_MAX)*3.5;
					XY_speed_min = -8594; //(NORMAL_SPEED_MIN)*3.5;
			}
			
		  if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL )
		  {
					XY_speed_max = 2000;  //(NORMAL_SPEED_MAX)*3.5;
					XY_speed_min = -2000; //(NORMAL_SPEED_MIN)*3.5;
		  }
 
 
			if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)                      
				 moto_3508_set.dstVmmps_Y -= ACC_SPEED;//按下W键
			else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)                 
				 moto_3508_set.dstVmmps_Y += ACC_SPEED;//按下S键
			else 
				 moto_3508_set.dstVmmps_Y = 0;
			
			if(moto_3508_set.dstVmmps_Y < XY_speed_min)  
			{					
				 moto_3508_set.dstVmmps_Y = XY_speed_min;
			}else if(moto_3508_set.dstVmmps_Y > XY_speed_max)  
			{				
				 moto_3508_set.dstVmmps_Y = XY_speed_max;
			}


			if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)                       
				 moto_3508_set.dstVmmps_X -= ACC_SPEED; //按下D键
			else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)    		         
				 moto_3508_set.dstVmmps_X += ACC_SPEED;//按下A键
			else 
				 moto_3508_set.dstVmmps_X = 0;
			
			
			if(moto_3508_set.dstVmmps_X < XY_speed_min)  
			{					
				 moto_3508_set.dstVmmps_X = XY_speed_min;
			}else if(moto_3508_set.dstVmmps_X > XY_speed_max)  
			{				
				 moto_3508_set.dstVmmps_X = XY_speed_max;
			}

					//车体旋转
					if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_E)   //
					{
              moto_3508_set.dstVmmps_W += ACC_SPEED;
              
					}else if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q)
          {
              moto_3508_set.dstVmmps_W -= ACC_SPEED;
              
          }else
          {
            	moto_3508_set.dstVmmps_W = 0;
          }
					
					if(moto_3508_set.dstVmmps_W < W_speed_min)  
					{					
						moto_3508_set.dstVmmps_W = W_speed_min;
					}else if(moto_3508_set.dstVmmps_W > W_speed_max)  
					{				
						moto_3508_set.dstVmmps_W = W_speed_max;
					}*/
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

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
		
	for(;;)
	{
	  sendata();
	

	  osDelayUntil(&xLastWakeTime, 500);
  }
}
