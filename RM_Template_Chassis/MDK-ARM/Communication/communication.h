#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "string.h"


#define SizeofReferee 100
#define SizeofRemote 18
#define SizeofJY901	 11
#define SizeofMinipc  9
extern uint8_t USART1_RX_DATA[(SizeofRemote)];//遥控
extern uint16_t USART1_RX_NUM;
extern uint8_t USART6_RX_DATA[(100)];//裁判系统
extern uint16_t USART6_RX_NUM;
extern uint8_t UART8_RX_DATA[(SizeofJY901)];//外接陀螺仪
extern uint16_t UART8_RX_NUM;

/* 本模块向外部提供的数据类型定义 --------------------------------------------*/
///////////外接陀螺仪////////////////
typedef struct 
{
 int32_t yaw_angle;
 int32_t yaw_last_angle;
 int32_t yaw_chaoxian_angle;
 int32_t first_angle;
 int32_t final_angle;
 int32_t angle_err;
 int32_t angle_cnt;
}XTLY;

struct STime
{
	unsigned char ucYear;
	unsigned char ucMonth;
	unsigned char ucDay;
	unsigned char ucHour;
	unsigned char ucMinute;
	unsigned char ucSecond;
	unsigned short usMiliSecond;
};
struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};
struct SMag
{
	short h[3];
	short T;
};

struct SDStatus
{
	short sDStatus[4];
};

struct SPress
{
	long lPressure;
	long lAltitude;
};

struct SLonLat
{
	long lLon;
	long lLat;
};

struct SGPSV
{
	short sGPSHeight;
	short sGPSYaw;
	long lGPSVelocity;
};
struct SQ
{ short q[4];
};

typedef struct    //外接陀螺仪    可以改成套用电机参数的结构体moto_measure_t  _待续
{
	float err;
	float JY901_angle;
	float JY901_angle_last;
	float first_angle;
	float angle_round;
  float total_angle;
  float last_final_angle;
	float vx;
	float vy;	
	float vz;
	float vx_last;
	float vy_last;
	float vz_last;
	int8_t frame;
	uint8_t times;
}JY901_t;
///////////////遥控/////////////////////
typedef struct //遥控器及键鼠通道
		{ 
			int16_t x; //!< Byte 6-7 
			int16_t y; //!< Byte 8-9 
			int16_t z; //!< Byte 10-11 
			uint8_t press_l; //!< Byte 12 
			uint8_t press_r; //!< Byte 13 
    }Mouse; 
	typedef 	struct 
		{ 
	 uint16_t ch0; 
	 uint16_t ch1; 
	 uint16_t ch2; 
	 uint16_t ch3; 
	 uint8_t s1; 
	 uint8_t s2; 
		}Rc; 
	typedef struct 
		{ 
		  uint16_t v; //!< Byte 14-15 
		}Key; 
		typedef struct 
{ 
  Rc rc; 
  Mouse mouse; 
  Key key; 
}RC_Ctl_t; 
////////////////遥控/////////////////////



/*******************云台状态标志位******************************/
typedef struct
{
	uint8_t minipc_mode;
	uint8_t gimbal_mode;
	uint8_t gimbal_flag;
	uint8_t remote_mode;
}Gimbal_Status_t;


/*******************mpu6500*********************************/
typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
	
  int16_t last_gx;
  int16_t last_gy;
  int16_t last_gz; 
	
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;


/* 本模块向外部提供的宏定义 --------------------------------------------------*/

/* 本模块向外部提供的接口常量声明 --------------------------------------------*/
/**************外接陀螺仪*******************/
extern JY901_t  	ptr_jy901_t_yaw;//外接陀螺仪数据
extern JY901_t  	ptr_jy901_t_pit;
extern JY901_t    ptr_jy901_t_angular_velocity;
extern XTLY tly;

/****************遥控********************/
extern RC_Ctl_t RC_Ctl; //遥控数据

//云台数据
extern Gimbal_Status_t gimbal_status;

/* 本模块向外部提供的接口函数原型声明 ----------------------------------------*/
//外接陀螺仪
void JY901_Data_Pro(void);
void tly_Pro(void);
//遥控
void Remote_Ctrl(void);
void Remote_Disable(void);

//////裁判系统
////void DataVerify(void);
////// 使用方法 发送用户数据 Send_FrameData(SelfDefinedDara, userMessage,tSelfDefineInfo(userMessage)); 
////void Send_FrameData(tCmdID cmdid, uint8_t * pchMessage,uint8_t dwLength); 
////void sendata(void);



//JY901 全局配置
#define SAVE 			0x00
#define CALSW 		0x01
#define RSW 			0x02
#define RRATE			0x03
#define BAUD 			0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define HXOFFSET	0x0b
#define HYOFFSET	0x0c
#define HZOFFSET	0x0d
#define D0MODE		0x0e
#define D1MODE		0x0f
#define D2MODE		0x10
#define D3MODE		0x11
#define D0PWMH		0x12
#define D1PWMH		0x13
#define D2PWMH		0x14
#define D3PWMH		0x15
#define D0PWMT		0x16
#define D1PWMT		0x17
#define D2PWMT		0x18
#define D3PWMT		0x19
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define GPSBAUD		0x1c

#define YYMM				0x30
#define DDHH				0x31
#define MMSS				0x32
#define MS					0x33
#define AX					0x34
#define AY					0x35
#define AZ					0x36
#define GX					0x37
#define GY					0x38
#define GZ					0x39
#define HX					0x3a
#define HY					0x3b
#define HZ					0x3c			
#define Roll				0x3d
#define Pitch				0x3e
#define Yaw					0x3f
#define TEMP				0x40
#define D0Status		0x41
#define D1Status		0x42
#define D2Status		0x43
#define D3Status		0x44
#define PressureL		0x45
#define PressureH		0x46
#define HeightL			0x47
#define HeightH			0x48
#define LonL				0x49
#define LonH				0x4a
#define LatL				0x4b
#define LatH				0x4c
#define GPSHeight   0x4d
#define GPSYAW      0x4e
#define GPSVL				0x4f
#define GPSVH				0x50
#define q0          0x51
#define q1          0x52
#define q2          0x53
#define q3          0x54
      
#define DIO_MODE_AIN 0
#define DIO_MODE_DIN 1
#define DIO_MODE_DOH 2
#define DIO_MODE_DOL 3
#define DIO_MODE_DOPWM 4
#define DIO_MODE_GPS 5		
#endif
