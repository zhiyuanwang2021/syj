#ifndef  __IO_SPI_H__
#define  __IO_SPI_H__

#include "delay.h"
#include "main.h"
/*****************************端口定义*********************************/
//1.端口定义

#define		NSS_RESET	HAL_GPIO_WritePin(P_ADCCS1_GPIO_Port, P_ADCCS1_Pin, GPIO_PIN_RESET)
#define		NSS_SET		HAL_GPIO_WritePin(P_ADCCS1_GPIO_Port, P_ADCCS1_Pin, GPIO_PIN_SET)

#define		CS1_RESET	HAL_GPIO_WritePin(P_ADCCS1_GPIO_Port, P_ADCCS1_Pin, GPIO_PIN_RESET)
#define		CS1_SET		HAL_GPIO_WritePin(P_ADCCS1_GPIO_Port, P_ADCCS1_Pin, GPIO_PIN_SET)

#define		CS2_RESET	HAL_GPIO_WritePin(P_ADCCS2_GPIO_Port, P_ADCCS2_Pin, GPIO_PIN_RESET)
#define		CS2_SET		HAL_GPIO_WritePin(P_ADCCS2_GPIO_Port, P_ADCCS2_Pin, GPIO_PIN_SET)

#define		CS3_RESET	HAL_GPIO_WritePin(P_ADCCS3_GPIO_Port, P_ADCCS3_Pin, GPIO_PIN_RESET)
#define		CS3_SET		HAL_GPIO_WritePin(P_ADCCS3_GPIO_Port, P_ADCCS3_Pin, GPIO_PIN_SET)

#define		SCK_RESET	HAL_GPIO_WritePin(P_ADCSCLK_GPIO_Port, P_ADCSCLK_Pin, GPIO_PIN_RESET)
#define		SCK_SET		HAL_GPIO_WritePin(P_ADCSCLK_GPIO_Port, P_ADCSCLK_Pin, GPIO_PIN_SET)

#define		SDO_READ	HAL_GPIO_ReadPin(P_ADCMISO_GPIO_Port,P_ADCMISO_Pin)

#define		SDI_RESET	HAL_GPIO_WritePin(P_ADCMOSI_GPIO_Port, P_ADCMOSI_Pin, GPIO_PIN_RESET)
#define		SDI_SET		HAL_GPIO_WritePin(P_ADCMOSI_GPIO_Port, P_ADCMOSI_Pin, GPIO_PIN_SET)
/*****************************接口函数*********************************/

void spi_io_Initl(void);						//初始化SPI端口电平	
void spi_Write_Byte(unsigned char _1Byte);		//单字节写函数
unsigned char spi_Read_Byte(void);				//单字节读函数

#endif


