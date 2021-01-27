#include "stm32f4xx_hal.h"
#define SUPERCAP 1
#define BATTERY 0

#define Charge_ON 1
#define Charge_OFF 0
	
void CAP_Mode (uint8_t Power_Supply, uint8_t charge);
void super_cap (float Power, uint16_t Buffer,float Voltage,float Current_Charge,float Moto_Power);//测试版+第二版
float exchange_data(unsigned char *data);
extern uint8_t  judge_state; //掉线状态
extern float all_sent;


typedef struct
{
float  CAP_Voltage;
float  CAP_Current;
float  ALL_Current;
} Cap_Get;

typedef struct
{
uint8_t Spi_Data;
uint8_t Power_Supply;
uint8_t Charge;
}Cap_Sent;

extern Cap_Sent Sent_103;
extern Cap_Get  Get_103;
extern uint8_t   Recive_103[12];
extern uint8_t Send_103[8];
extern float all_sent;
extern uint8_t over;
extern float Max;
extern float last_current;
extern uint8_t judge_state;  //后期加
extern uint16_t   test_id;
