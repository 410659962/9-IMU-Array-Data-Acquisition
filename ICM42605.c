/*
 * ICM42605.c
 *
 *  Created on: Jun 24, 2024
 *      Author: zty
 */
#include "ICM42605.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart1; // 声明外部UART句柄

volatile uint8_t uart_tx_ready = 1;        // UART传输就绪标志
// DMA传输完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        uart_tx_ready = 1; // 标记传输完成
    }
}


// 使用查表法替代switch-case
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} CS_Pin;

const CS_Pin csPins[9] = {
    {GPIOB, GPIO_PIN_0},  // 1
    {GPIOA, GPIO_PIN_4},  // 2
    {GPIOA, GPIO_PIN_3},  // 3
    {GPIOA, GPIO_PIN_2},  // 4
    {GPIOA, GPIO_PIN_1},  // 5
    {GPIOA, GPIO_PIN_0},  // 6
    {GPIOC, GPIO_PIN_15}, // 7
    {GPIOC, GPIO_PIN_14}, // 8
    {GPIOC, GPIO_PIN_13}  // 9
};

void CS(uint8_t devNum, uint8_t state)
{
    if(devNum <1 || devNum >9) return;
    HAL_GPIO_WritePin(csPins[devNum-1].port, csPins[devNum-1].pin,
                     state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//读取阵列中9个imu的id
void readid()
{
	//ICM42605_WHO_AM_I | 0x80高位置1读，置0写
    uint8_t who_am_i;
    char error_msg[30];
    for (int i = 0; i < 9; i++)
    {
    	CS(i+1, 0);
    	// 读取WHO_AM_I寄存器（地址0x75）验证通信
    	uint8_t read_cmd[2] = {ICM42605_WHO_AM_I | 0x80, 0x00};
    	uint8_t rx_buf[2];
    	HAL_SPI_TransmitReceive(&hspi1, read_cmd, rx_buf, 2, 5);
    	who_am_i = rx_buf[1]; // 第二个字节为实际数据
    	if (who_am_i != 0x42) { // ICM42605的WHO_AM_I值应为0x42
    		snprintf(error_msg, sizeof(error_msg), "init_error_sensor_%d,%2x\r\n", i+1, rx_buf[1]);
    		while(1)
    		{
    			HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY); // 自定义错误处理
    			HAL_Delay(1000);
    		}
    	}
        else{
        	ICM42605_Init();	//ID校验成功，初始化ICM42605
        }
        CS(i+1, 1);
    }
}

//初始化阵列中9个imu的加速度计和陀螺仪
//GYRO_MODE；ACCEL_MODE：LN低噪声；地址：0x4E 写值0000 1111 0x0F
//加速度计：地址：0x50；量程±4g；频率：1kHz 写值0100 0110 0x46
//陀螺仪：地址：0x4F；量程：±250dps；频率：1kHz；写值0110 0110 0x66
//void ICM42605_Init()
//{
//    // 定义配置命令数组，包含所有需要写入的寄存器地址和值
//    static const uint8_t configCommands[][2] = {
//        {ICM42605_PWR_MGMT0,   0x0F},  // 电源管理配置
//        {ICM42605_GYRO_CONFIG0,0x66},  // 陀螺仪配置
//        {ICM42605_ACCEL_CONFIG0,0x46}  // 加速度计配置
//    };
//    for(int i=0; i<9; i++)
//    {
//    	CS(i+1, 0);
//    	// 批量发送所有配置命令
//        for (int cmdIdx = 0; cmdIdx < 3; cmdIdx++) {
//            HAL_SPI_Transmit(&hspi1, (uint8_t*)configCommands[cmdIdx], 2, 5);
//        }
//    	CS(i+1, 1);
//    }
//    // 等待设备稳定
//    HAL_Delay(10);
//}
//原来的初始化程序
void ICM42605_Init()
{
	uint8_t icm42605PWR_MGMT0[2] = {ICM42605_PWR_MGMT0 , 0x0F};
	uint8_t icm42605GYRO_CONFIG0[2] = {ICM42605_GYRO_CONFIG0 , 0x66};
	uint8_t icm42605ACCEL_CONFIG0[2] = {ICM42605_ACCEL_CONFIG0 , 0x46};
	for(int i=0;i<9;i++)
	{
		CS(i+1,0);
		HAL_SPI_Transmit(&hspi1, icm42605PWR_MGMT0, 2, 5);
		HAL_SPI_Transmit(&hspi1, icm42605GYRO_CONFIG0, 2, 5);
		HAL_SPI_Transmit(&hspi1, icm42605ACCEL_CONFIG0, 2, 5);
		CS(i+1,1);
	}
	HAL_Delay(1);
}

/*加速度计和陀螺仪各个轴的地址
 * Upper byte of Accel X-axis data:0x1F ; ICM42605_ACCEL_DATA_X1
 * Lower byte of Accel X-axis data:0x20 ; ICM42605_ACCEL_DATA_X0
 * Upper byte of Accel Y-axis data:0x21 ; ICM42605_ACCEL_DATA_Y1
 * Lower byte of Accel Y-axis data:0x22 ; ICM42605_ACCEL_DATA_Y0
 * Upper byte of Accel Z-axis data:0x23 ; ICM42605_ACCEL_DATA_Z1
 * Lower byte of Accel Z-axis data:0x24 ; ICM42605_ACCEL_DATA_Z0
 * Upper byte of Gyro X-axis data:0x25 ; ICM42605_GYRO_DATA_X1
 * Lower byte of Gyro X-axis data:0x26 ; ICM42605_GYRO_DATA_X0
 * Upper byte of Gyro Y-axis data:0x27 ; ICM42605_GYRO_DATA_Y1
 * Lower byte of Gyro Y-axis data:0x28 ; ICM42605_GYRO_DATA_Y0
 * Upper byte of Gyro Z-axis data:0x29 ; ICM42605_GYRO_DATA_Z1
 * Lower byte of Gyro Z-axis data:0x2A ; ICM42605_GYRO_DATA_Z0
*/

// 批量读取单个传感器的全部6轴数据
void read_imu_data(int devNum, float accel[3], float gyro[3])
{
    uint8_t txBuf[13] = {0};
    uint8_t rxBuf[13] = {0};
    txBuf[0] = ICM42605_ACCEL_DATA_X1 | 0x80; // 起始地址+读标志

    CS(devNum, 0);
    HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 13, 10);
    CS(devNum, 1);

    // 解析加速度计
    accel[0] = ((int16_t)(rxBuf[1]<<8 | rxBuf[2])) / 8192.0f; // X轴
    accel[1] = ((int16_t)(rxBuf[3]<<8 | rxBuf[4])) / 8192.0f; // Y轴
    accel[2] = ((int16_t)(rxBuf[5]<<8 | rxBuf[6])) / 8192.0f; // Z轴

    // 解析陀螺仪
    gyro[0] = ((int16_t)(rxBuf[7]<<8 | rxBuf[8])) / 131.072f; // 陀螺仪X轴
    gyro[1] = ((int16_t)(rxBuf[9]<<8 | rxBuf[10])) / 131.072f; // 陀螺仪Y轴
    gyro[2] = ((int16_t)(rxBuf[11]<<8 | rxBuf[12])) / 131.072f; // 陀螺仪Z轴
}
void readallgyracc()
{
	float imubuffer[54];
	char tail[4] = {0x00, 0x00, 0x80, 0x7f};
	char tou[4] = {0xAA, 0x55};
//	char temp[10],GYRACCmessage[1024]={0};
	int bcount=0;
	for(int i=0;i<9;i++)
	{
		float accel[3], gyro[3];
		read_imu_data(i+1, accel, gyro);
		// 填充加速度计数据
		for(int j=0; j<3; j++)
		{
			imubuffer[bcount++] = accel[j];// g
		}
		// 填充陀螺仪数据
		for(int j=0; j<3; j++)
		{
			imubuffer[bcount++] = gyro[j];// deg/s
		}
	}
//	for(bcount=0;bcount<53;bcount++)
//	{
//		sprintf(temp,"%f,",imubuffer[bcount]);
//		strcat(GYRACCmessage,temp);
//	}
//	sprintf(temp,"%f\r\n",imubuffer[53]);
//	strcat(GYRACCmessage,temp);
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)GYRACCmessage, strlen(GYRACCmessage));

	HAL_UART_Transmit(&huart1, (uint8_t*)tou, 2*sizeof(char),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t*)imubuffer, 54*sizeof(float),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t*)tail, 4*sizeof(char),HAL_MAX_DELAY);
}



/*
// 全局双缓冲区定义
float bufferA[54] = {0}, bufferB[54] = {0};
volatile float *current_buffer = bufferA;  // 采集缓冲区
volatile float *transmit_buffer = bufferB; // 传输缓冲区


void readallgyracc()
{

    // 切换缓冲区指针（乒乓操作）
    volatile float *temp = current_buffer;
    current_buffer = transmit_buffer;
    transmit_buffer = temp;

    // 启动新数据采集到current_buffer
	int bcount=0;
	for(int i=0; i<9; i++)
	{
		float accel[3], gyro[3];
		read_imu_data(i+1, accel, gyro);
		// 填充加速度计数据
		for(int j=0; j<3; j++)
		{
			// 正确写法：使用current_buffer指针
			((float*)current_buffer)[bcount++] = accel[j];
		}
		// 填充陀螺仪数据
		for(int j=0; j<3; j++)
		{
			((float*)current_buffer)[bcount++] = gyro[j];
		}
	}
    // 启动DMA传输（仅当UART空闲时）
    if (uart_tx_ready) {
        uart_tx_ready = 0;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)transmit_buffer, 54*sizeof(float));
    }
}
*/



//===========之前自己写的函数================//
float readaccgyr(uint8_t x,uint8_t y,int i,int j)
{
	float temp;
	int16_t xyzdata = 0;
	uint8_t revalue1[3] = {0},gyraccad1[3] = {0};
	gyraccad1[0] = x | 0x80;
//	uint8_t revalue1[2] = {0},gyraccad1[2] = {x | 0x80 , 0x00};
//	uint8_t revalue2[2] = {0},gyraccad2[2] = {y | 0x80 , 0x00};
	CS(i+1,0);
//	HAL_SPI_TransmitReceive(&hspi1, gyraccad1, revalue1, 2, 5);
//	HAL_SPI_TransmitReceive(&hspi1, gyraccad2, revalue2, 2, 5);
	HAL_SPI_TransmitReceive(&hspi1, gyraccad1, revalue1, 3, 5);
	CS(i+1,1);
//	xyzdata = (revalue1[1] << 8) | revalue2[1];
	xyzdata = (revalue1[1] << 8) | revalue1[2];
	if(j==1)	temp = xyzdata/8192.0;
	else	temp = xyzdata/131.072;
	return temp;
}
//void readgyracc(float b[54])
//{
//	char temp[10],GYRACCmessage[1024]={0};
//	int bcount=0;
//	for(int i=0;i<9;i++)
//	{
//		b[bcount++] = readaccgyr(ICM42605_ACCEL_DATA_X1,ICM42605_ACCEL_DATA_X0,i,1);
//		b[bcount++] = readaccgyr(ICM42605_ACCEL_DATA_Y1,ICM42605_ACCEL_DATA_Y0,i,1);
//		b[bcount++] = readaccgyr(ICM42605_ACCEL_DATA_Z1,ICM42605_ACCEL_DATA_Z0,i,1);
//		b[bcount++] = readaccgyr(ICM42605_GYRO_DATA_X1,ICM42605_GYRO_DATA_X0,i,0);
//		b[bcount++] = readaccgyr(ICM42605_GYRO_DATA_Y1,ICM42605_GYRO_DATA_Y0,i,0);
//		b[bcount++] = readaccgyr(ICM42605_GYRO_DATA_Z1,ICM42605_GYRO_DATA_Z0,i,0);
//	}
//	for(bcount=0;bcount<53;bcount++)
//	{
//		sprintf(temp,"%f,",b[bcount]);
//		strcat(GYRACCmessage,temp);
//	}
//	sprintf(temp,"%f\r\n",b[53]);
//	strcat(GYRACCmessage,temp);
//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)GYRACCmessage, strlen(GYRACCmessage));//发送频率为50Hz
//}

/*
void readgyracc(float b[54])
{
	char temp[10],GYRACCmessage[1024]={0};
	int bcount=0;
	for(int i=0;i<9;i++)
	{
		float accel[3], gyro[3];
		read_imu_data(i+1, accel, gyro);
		// 填充加速度计数据
		for(int j=0; j<3; j++)
		{
			// 正确写法：使用current_buffer指针
			b[bcount++] = accel[j];
		}
		// 填充陀螺仪数据
		for(int j=0; j<3; j++)
		{
			b[bcount++] = gyro[j];
		}
	}
	for(bcount=0;bcount<53;bcount++)
	{
		sprintf(temp,"%f,",b[bcount]);
		strcat(GYRACCmessage,temp);
	}
	sprintf(temp,"%f\r\n",b[53]);
	strcat(GYRACCmessage,temp);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)GYRACCmessage, strlen(GYRACCmessage));//发送频率为50Hz
}
*/
