/*******************************************************************************
                      版权所有 (C), 2017-,NCUROBOT
********************************************************************************
* 文 件 名   : protocol.c
* 版 本 号   : 初稿
* 作    者   : NCUERM
* 生成日期   : 2018年7月
* 最近修改   :
* 功能描述   : crc校验函数
* 函数列表   :		
*						  uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength)
*						  uint8_t	append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength)
*					    uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength)
*						  uint16_t append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength)
*
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "protocol.h"
#include "communication.h"
#include "data_processing.h"
/* 内部自定义数据类型 --------------------------------------------------------*/

/* 内部宏定义 ----------------------------------------------------------------*/

/* 任务相关信息定义-----------------------------------------------------------*/

/* 内部常量定义---------------------------------------------------------------*/
ROBOT Robot;
float lianfa_speed;
float heat_limit_max;
float heat_limit_min;
const uint8_t CRC8_INIT     = 0xff;
const uint8_t CRC8_TAB[256] = {
  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
  0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
  0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
  0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
  0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
  0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
  0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
  0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
  0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
  0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
  0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
  0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
  0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
  0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
  0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
  0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

uint16_t       CRC_INIT        = 0xffff;
const uint16_t wCRC_Table[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/* 外部变量声明 --------------------------------------------------------------*/

/* 外部函数原型声明 ----------------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/

/* 函数原型声明 ----------------------------------------------------------*/

//crc8 generator polynomial:G(x)=x8+x5+x4+1

/*
** Descriptions: CRC8 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint8_t get_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8)
{
  uint8_t ucIndex;
  while (dwLength--)
  {
    ucIndex = ucCRC8 ^ (*pchMessage++);
    ucCRC8  = CRC8_TAB[ucIndex];
  }
  return (ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength)
{
  uint8_t ucExpected = 0;
  if ((pchMessage == 0) || (dwLength <= 2))
    return 0;
  ucExpected = get_crc8_check_sum(pchMessage, dwLength - 1, CRC8_INIT);
//	printf("crc8:%d\n",ucExpected);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint8_t append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength)
{
  uint8_t ucCRC = 0;
  if ((pchMessage == 0) || (dwLength <= 2))
    return 0 ;
  ucCRC = get_crc8_check_sum((uint8_t*)pchMessage, dwLength - 1, CRC8_INIT);
  pchMessage[dwLength - 1] = ucCRC;
	  return ucCRC;
}

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t get_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t chData;
  if (pchMessage == NULL)
  {
    return 0xFFFF;
  }
  while (dwLength--)
  {
    chData = *pchMessage++;
    (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
  }
  return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength)
{
  uint16_t wExpected = 0;
  if ((pchMessage == NULL) || (dwLength <= 2))
  {
    return 0;
  }
  wExpected = get_crc16_check_sum(pchMessage, dwLength - 2, CRC_INIT);
//	printf("crc16:%d\n",wExpected);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint16_t append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength)
{
  uint16_t wCRC = 0;
  if ((pchMessage == NULL) || (dwLength <= 2))
  {
//		printf("wrong!\n");
    return 0;
  }
  wCRC = get_crc16_check_sum((uint8_t*)pchMessage, dwLength - 2, CRC_INIT);
  pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
	return wCRC;
}



/*
** Descriptions: 发送数据帧函数
** Input: tCmdID：发送数据的命令id
**			pchMessage：发送数据的首地址，需要是tSelfDefine格式的
**			dwLength：数据长度
** Output: 无
**   发送函数是通过串口6发送出去的
*/
void Send_FrameData(tCmdID cmdid, uint8_t * pchMessage,uint8_t dwLength)
{	
	uint8_t i;
	uint8_t *addr;
  custom_dataFrame SelfDefineFrame = {.FrameHeader.SOF = 0xA5};// 帧
	
	if ((pchMessage == 0) || (dwLength != sizeof(custom_data)))
		return ;
	
	addr = (uint8_t *)&SelfDefineFrame.custom_data.data1;
	
	SelfDefineFrame.CmdID = cmdid; //命令
	SelfDefineFrame.FrameHeader.DataLength = dwLength;//数据长度
//	SelfDefineFrame.FrameHeader.Seq++;  //帧数
	
	SelfDefineFrame.FrameHeader.Seq = 0;
	
  for(i = 0;i < dwLength;i++)
	{
			*addr++ = *pchMessage++;//数据拷贝(1)
	}

	  append_crc8_check_sum((uint8_t *)&SelfDefineFrame.FrameHeader,sizeof(SelfDefineFrame.FrameHeader));//加入帧头校验 
  	i = sizeof(SelfDefineFrame.FrameHeader) + sizeof(SelfDefineFrame.CmdID)  + sizeof(SelfDefineFrame.CRC16) + dwLength;//计算实际帧的长度
	  SelfDefineFrame.CRC16 = append_crc16_check_sum((uint8_t *)&SelfDefineFrame,i);
	
	HAL_UART_Transmit(&huart6,(uint8_t *)&SelfDefineFrame,22,100);
	
}
/*
** Descriptions: 发送数据测试函数
**
**
**   发送函数是通过串口6发送出去的
*/

void sendata(void)
{
	//(1)
	client_custom_data_t       custom_data_t; 
	custom_data_t.data1 = 1.0f;
	custom_data_t.data2 = 2.0f;
	custom_data_t.data3 = 3.0f;
	Send_FrameData(custom_data,(uint8_t *)&custom_data_t,sizeof(custom_data_t));	

}


/***************************************************************************************
**
	*	@brief	JSYS_Task(void const * argument)
	*	@param
	*	@supplement	
	*	@retval	
****************************************************************************************/
//void Referee_Data_Task(void const * argument)
//{
//	    tFrame   *Frame;
//	
//	    uint32_t NotifyValue;
//	for(;;)
//	{
//    
//		NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
//    
//    if(NotifyValue==1)
//		{
//			  NotifyValue=0;
//			
//				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1); //GRE_H
//        uint8_t *buff=USART6_RX_DATA;
//			for(int8_t i=0;i<USART6_RX_NUM;i++)
//			{
//					if(buff[i]==0xA5)
//					{
//					   Frame = (tFrame *)&buff[i];
//						
//					    if( verify_crc16_check_sum((uint8_t *)Frame, Frame->FrameHeader.DataLength + sizeof(tFrameHeader) + sizeof(tCmdID) + sizeof(Frame->CRC16))
//		             && verify_crc8_check_sum((uint8_t *)Frame,sizeof(tFrameHeader)))
//								 {
//									 if(Frame->CmdID==game_robot_state)
//									 {
//											Robot.id = Frame->Data.game_robot_state.robot_id;//id号
//											Robot.level = Frame->Data.game_robot_state.robot_level;//等级
//                      Robot.remainHp = Frame->Data.game_robot_state.remain_HP;//剩余血量
//                      Robot.maxHp  = Frame->Data.game_robot_state.max_HP;//最大血量
//                      Robot.heat.shoot_17_cooling_rate = Frame->Data.game_robot_state.shooter_heat0_cooling_rate;//17每秒冷却值
//                      Robot.heat.shoot_17_cooling_limit = Frame->Data.game_robot_state.shooter_heat0_cooling_limit;//17热量上限
//                      Robot.heat.shoot_42_cooling_rate = Frame->Data.game_robot_state.shooter_heat1_cooling_rate;//
//                      Robot.heat.shoot_42_cooling_limit = Frame->Data.game_robot_state.shooter_heat1_cooling_limit;//
//											
//									 }
//									 if(Frame->CmdID == power_heat_data)
//									 {
//											Robot.heat.shoot_17_heat = Frame->Data.power_heat_data.shooter_heat0;//17枪口热量
//                      Robot.heat.shoot_42_heat = Frame->Data.power_heat_data.shooter_heat1;
//                     
//                      Robot.Chassis_Power.Chassis_Current = Frame->Data.power_heat_data.chassis_current;//电流
//                      Robot.Chassis_Power.chassis_Power = Frame->Data.power_heat_data.chassis_power;//功率
//                      Robot.Chassis_Power.Chassis_Power_buffer = Frame->Data.power_heat_data.chassis_power_buffer;//缓冲
//                      Robot.Chassis_Power.Chassis_Volt = Frame->Data.power_heat_data.chassis_volt;//电压
//									 }
//									 if(Frame->CmdID==game_robot_pos)
//									 {
//                     
//										 Robot.postion.x = Frame->Data.game_robot_pos.x;
//                     Robot.postion.y = Frame->Data.game_robot_pos.y;
//                     Robot.postion.z = Frame->Data.game_robot_pos.z;
//                     Robot.postion.yaw = Frame->Data.game_robot_pos.yaw;
//                    
//									 }
//                   
//                   if(Frame->CmdID == buff_musk)
//                   {
//                     Robot.buff = Frame->Data.buff_musk.power_rune_buff;
//                     
//                   }
//                   if( Frame->CmdID == shoot_data)
//                   {
//                     if(Frame->Data.shoot_data.bullet_type == 1) //17
//                     {
//                     Robot.heat.shoot_17_freq = Frame->Data.shoot_data.bullet_freq;
//                     Robot.heat.shoot_17_speed = Frame->Data.shoot_data.bullet_speed;
//                     }
//                     if(Frame->Data.shoot_data.bullet_type == 2)//42
//                     {
//                     Robot.heat.shoot_42_freq = Frame->Data.shoot_data.bullet_freq;
//                     Robot.heat.shoot_42_speed = Frame->Data.shoot_data.bullet_speed;
//                     }
//                   }
//											 i=i+sizeof(Frame);
//								}
//					}
//				
//			}
//		}
//	 }

// }
//	
				  void Referee_Data_Handler(void)
{
		static tFrame *Frame;
		static uint8_t crc8_flag = 0,crc16_flag = 0;
		uint8_t *buff = USART6_RX_DATA;

		for(uint8_t num = 0; num < USART6_RX_NUM; num++)
		{
				if(buff[num]==0xA5)
				{		
					
						Frame = (tFrame *)&buff[num];

						if (!verify_crc8_check_sum((uint8_t *)Frame,5))
						{
								continue;
						}
						else if(!verify_crc16_check_sum((uint8_t *)Frame, 5 + 2 +  Frame->FrameHeader.DataLength + 2))
						{
								continue;
						}

						if(Frame->CmdID==game_robot_state)
						{
							Robot.id = Frame->Data.game_robot_state.robot_id;//id号
							Robot.level = Frame->Data.game_robot_state.robot_level;//等级
							Robot.remainHp = Frame->Data.game_robot_state.remain_HP;//剩余血量
							Robot.maxHp  = Frame->Data.game_robot_state.max_HP;//最大血量
							Robot.heat.shoot_17_cooling_rate = Frame->Data.game_robot_state.shooter_heat0_cooling_rate;//17每秒冷却值
							Robot.heat.shoot_17_cooling_limit = Frame->Data.game_robot_state.shooter_heat0_cooling_limit;//17；冷却上限
							Robot.heat.shoot_42_cooling_rate = Frame->Data.game_robot_state.shooter_heat1_cooling_rate;//
							Robot.heat.shoot_42_cooling_limit = Frame->Data.game_robot_state.shooter_heat1_cooling_limit;//
							
						}
						if(Frame->CmdID == power_heat_data)
						{
							Robot.heat.shoot_17_heat = Frame->Data.power_heat_data.shooter_heat0;//17枪口热量
							Robot.heat.shoot_42_heat = Frame->Data.power_heat_data.shooter_heat1;
						 
							Robot.Chassis_Power.Chassis_Current = Frame->Data.power_heat_data.chassis_current;//电流
							Robot.Chassis_Power.chassis_Power = Frame->Data.power_heat_data.chassis_power;//功率
							Robot.Chassis_Power.Chassis_Power_buffer = Frame->Data.power_heat_data.chassis_power_buffer;//缓冲
							Robot.Chassis_Power.Chassis_Volt = Frame->Data.power_heat_data.chassis_volt;//电压
						}
						if(Frame->CmdID==game_robot_pos)
						{
						 
						 Robot.postion.x = Frame->Data.game_robot_pos.x;
						 Robot.postion.y = Frame->Data.game_robot_pos.y;
						 Robot.postion.z = Frame->Data.game_robot_pos.z;
						 Robot.postion.yaw = Frame->Data.game_robot_pos.yaw;

						}

						if(Frame->CmdID == buff_musk)
						{
							Robot.buff = Frame->Data.buff_musk.power_rune_buff;
						}
						if( Frame->CmdID == shoot_data)
						{
							 if(Frame->Data.shoot_data.bullet_type == 1) //17
							 {
								 Robot.heat.shoot_17_speed_last=Robot.heat.shoot_17_speed;
								 Robot.heat.shoot_17_freq = Frame->Data.shoot_data.bullet_freq;
								 Robot.heat.shoot_17_speed = Frame->Data.shoot_data.bullet_speed;
							 }
							 if(Frame->Data.shoot_data.bullet_type == 2)//42
							 {
								 Robot.heat.shoot_42_freq = Frame->Data.shoot_data.bullet_freq;
								 Robot.heat.shoot_42_speed = Frame->Data.shoot_data.bullet_speed;
							 }
						}

						num = num + 5 + 2 +  Frame->FrameHeader.DataLength + 2;

				}
		}
		
		//清空缓存
		memset(USART6_RX_DATA,0,SizeofReferee);
}

/***************************************************************************************
**
	*	@brief		枪口热量限制函数
	*	@param  
	*	@supplement	
	*	@retval	
****************************************************************************************/
	
float heat_limit(void)
	{
		static uint16_t lengque_flag=0;
		switch(Robot.level)
		{
			case 1://一级步兵 上限180 冷却30
			{
					switch(Robot.heat.shoot_17_cooling_rate)
						{
							case 30:
								{
									lianfa_speed=2160;
									heat_limit_max=160;
									heat_limit_min=90;
									break;//使射频达到10HZ	上下限根据实验测得
								}
							case 90:
								{
									lianfa_speed=3240;
									heat_limit_max=150;
									heat_limit_min=90;
									break;//使射频达到15HZ	上下限根据实验测得
								}
							case 150:
								{
									lianfa_speed=4320;
									heat_limit_max=140;
									heat_limit_min=90;
									break;//使射频达到20HZ	上下限根据实验测得
								}
					}break;
				}
			case 2://二级步兵 上限240 冷却40
			{
					switch(Robot.heat.shoot_17_cooling_rate)
						{
							case 40:
								{
									lianfa_speed=2592;
									heat_limit_max=210;
									heat_limit_min=90;
									break;//使射频达到12HZ	上下限根据实验测得
								}
							case 120:
								{
									lianfa_speed=4320;
									heat_limit_max=210;
									heat_limit_min=90;
									break;//使射频达到20HZ	上下限根据实验测得
								}
							case 200:
								{
									lianfa_speed=4320;
									heat_limit_max=210;
									heat_limit_min=90;
									break;//使射频达到20HZ	上下限根据实验测得
								}
				}break;
			}
			case 3://三级步兵 上限300 冷却50
			{
					switch(Robot.heat.shoot_17_cooling_rate)
						{
							case 50:
								{
									lianfa_speed=3240;
									heat_limit_max=270;
									heat_limit_min=120;
									break;//使射频达到15HZ	上下限根据实验测得
								}
							case 150:
								{
									lianfa_speed=4320;
									heat_limit_max=260;
									heat_limit_min=120;
									break;//使射频达到20HZ	上下限根据实验测得
								}
							case 250:
								{
									lianfa_speed=4320;
									heat_limit_max=260;
									heat_limit_min=120;
									break;//使射频达到20HZ	上下限根据实验测得
								}
							}break;
			}
		}
		if(Robot.heat.shoot_17_heat>heat_limit_max)
			{
				lengque_flag=1;
			}
		if(lengque_flag==1&&Robot.heat.shoot_17_heat<heat_limit_min)
			{
				lengque_flag=0;
			}
		if(lengque_flag==0)
			{
				return lianfa_speed;
			}else return 0;
		
		
	}
 


