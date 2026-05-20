/**
******************************************************************************
* @file         ADS1274.h
* @version      V1.0
* @date         2023-08-30                  
* @brief        ADS1274.c锟斤拷头锟侥硷拷
******************************************************************************
*/
#ifndef _ADS1274_H_
#define _ADS1274_H_
#include "stm32h7xx_hal.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "gParameter.h"
/***************----- W5500 GPIO锟斤拷锟斤拷 -----***************/
#define ADC_SYNC		GPIO_PIN_1	//锟斤拷锟斤拷ADS1274锟斤拷SYNC锟斤拷锟斤拷	 
#define ADC_SYNC_PORT	GPIOD
	
#define ADC_DRDY		GPIO_PIN_2	//锟斤拷锟斤拷ADS1274锟斤拷DRDY锟斤拷锟斤拷
#define ADC_DRDY_PORT	GPIOD

#define CLKDIV  1
#define f_CLK_DATD 512
#define ADS_CLK 2560000
//DRDY = ADS_CLK/f_CLK_DATD=5kHz

#define ADS_CHANNEL_NUM 3

// Sensitive mV/V  
// ADC_AMPLIFIER 锟脚达拷248锟斤拷
/*
锟斤拷锟斤拷锟斤拷锟斤拷压为+-5V锟斤拷Sensitive = 2锟斤拷锟斤拷锟剿凤拷锟斤拷锟斤拷说锟窖刮?+-10mV
锟脚达拷248锟斤拷锟斤拷锟脚达拷锟轿?+-2480mV
锟斤拷锟斤拷 锟斤拷锟斤拷100kN
+-100kN 锟斤拷应 +-5*Sensitive
锟斤拷前锟斤拷 = ADCCODE/ADC_RESOLUTION * ADC_VREF /ADC_AMPLIFIER 
		/(ADC_EXCITATION_VOLTAGE*Sensitive)*ADC_NomialValue
*/

#define ADC_NomialValue 49000.0 //N 5t = 5000*9.8N
// #define ADC_VREF 2499.0  //mV
#define ADC_EXCITATION_VOLTAGE 10.0 //V (+5V-(-5V)=10V)
#define ADC_AMPLIFIER 248.0  //1+49.4/0.2
#define ADC_RESOLUTION 8388608.0  //2^23
#define ADC_Sensitive 2.0 //mV/V
#define ADC_FILTER_LEN 1
#define ADC_FACTOR 1.201226008469083e-7
#define ADC_FACTOR_ROUND 1.2e-7f //ADC FACTOR round off
#define ADC_FACTOR_CS5530_ROUND 9.3132e-7f //ADC FACTOR
//9.3132e-7 = ADC_VREF(2500mV) /ADC_RESOLUTION(8388606) /ADC_AMPLIFIER(64) / ADC_EXCITATION_VOLTAGE(5V)
#define ADC_FACTOR_LOAD 1.201226008469083e-4
//1.201226008469083e-7 = ADC_VREF /ADC_RESOLUTION /ADC_AMPLIFIER / ADC_EXCITATION_VOLTAGE

typedef struct{
	uint8_t flag;
	uint8_t channelNum;
	uint8_t tempNum;
	//uint8_t buf[ADS_CHANNEL_NUM*3];
	uint8_t buf[ADS_CHANNEL_NUM][3];
	int32_t value[ADS_CHANNEL_NUM][ADC_FILTER_LEN];//code value
	int32_t valueLast[ADS_CHANNEL_NUM][ADC_FILTER_LEN];//code value
	int32_t valueFilter[ADS_CHANNEL_NUM];
	int32_t valueTemp[ADS_CHANNEL_NUM];
	float  value_calcu[ADS_CHANNEL_NUM];//real value
}ads1274_t;

extern ads1274_t ads1274_par;

void ADS1274_Init(void);
void ADS1274_Read_Data(ads1274_t *par,SPI_HandleTypeDef *_hspix);
void ADS1274_Data_Process(ads1274_t *par);
#endif
