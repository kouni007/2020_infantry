/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"

#include "MahonyAHRS.h"
#include "math.h"


#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定


/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */


void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

extern SPI_HandleTypeDef hspi1;


uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;


static uint8_t first_temperate;
pid_t imu_temp_pid = {0};
INS ins_yaw = {0};

fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad



/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void INS_task(void const *pvParameters)
{
    //wait a time
    osDelay(INS_TASK_INIT_TIME);
    while(BMI088_init())    //等待陀螺仪初始化完成
    {
        osDelay(100);
    }
    while(ist8310_init())   //等待磁力计初始化完成
    {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

    PID_struct_init(&imu_temp_pid, POSITION_PID, 4500, 4400, 1600.0f, 0.2f, 0.0f);  //参数设定值参照例程

    AHRS_init(INS_quat, bmi088_real_data.accel, ist8310_real_data.mag);


    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }


    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;

    while (1)
    {
        //wait spi DMA tansmit done
        //等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }


        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))  //判断标志位
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);   //对应标志位清零
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
//            imu_temp_control(bmi088_real_data.temp);
        }


        AHRS_update(INS_quat, 0.001f, bmi088_real_data.gyro, bmi088_real_data.accel, ist8310_real_data.mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
	/*0628  处理陀螺仪数据，解决云台疯转问题*/   
				/*
				Log : 黄杰辉
				陀螺仪数据INS_angle受到影响，增加很慢,原因在于->>>>>>osDelay(1000);  0701
				ins_angle_offest 和 ins_angle 一直都是同步的，导致angle_round一直为0
				*/
				
				ins_yaw.ins_angle = (float)INS_angle[0];
				
				
				
			if(ins_yaw.times++ > 5)  
			{	
				ins_yaw.times = 6;
				ins_yaw.err = ins_yaw.ins_angle - ins_yaw.ins_angle_last;
				if(ins_yaw.err < -180)
				{
					ins_yaw.angle_round++;
				}
				else if(ins_yaw.err > 180)
				{
					ins_yaw.angle_round--;
				}
				
				ins_yaw.final_angle = (ins_yaw.angle_round*360 + ins_yaw.ins_angle - ins_yaw.first_angle)*(-60.75f) ;
				
			}
			else
			{
				ins_yaw.first_angle = ins_yaw.ins_angle;
			}
			
			ins_yaw.ins_angle_last = ins_yaw.ins_angle;
			HAL_Delay(1);
			ins_yaw.times++;


    }
}

void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
//   MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
	  MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0.0, 0.0, 0.0); // 强行六轴解算
}
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f) * 57.296f;
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2])) * 57.296f;
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f) * 57.296f;
}

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
			  pid_calc(&imu_temp_pid, temp, 45.0f);
        if (imu_temp_pid.pos_out < 0.0f)
        {
            imu_temp_pid.pos_out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.pos_out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.IntegralLimit = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}



/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
void imu_cmd_spi_dma(void)
{

        //开启陀螺仪的DMA传输
        if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);     //陀螺仪数据标志位第[0]位 置0
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);     //陀螺仪数据标志位第[1]位 置1，表明现在处于等待dma接收状态

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);   //选中陀螺仪
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT); //开启dma，等待接收
            return;
        }
        //开启加速度计的DMA传输
        if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);   //加速度数据标志位第[0]位 置0
            accel_update_flag |= (1 << IMU_SPI_SHFITS);   //加速度数据标志位第[1]位 置1，表明现在处于等待dma接收状态

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET); //选中加速度计
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }
        



        if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);   //温度数据标志位第[0]位 置0
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);   //温度数据标志位第[1]位 置1，表明现在处于等待dma接收状态

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET); //选中加速度计（温度是由加速度计反馈的）
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}



