#ifndef  __CS553X_H__
#define  __CS553X_H__

// #include "IO_SPI.h"                          // Legacy CS5530 bit-banged SPI disabled during CS5552 migration.
#include "main.h"
//#include "F10x_UART1.h"						//UART1
/****************************端口定义*********************************/
//1.转换完成信号检测

//#define RDY		HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)					//转换完成信号，检测SDO第一个下降沿，也可直接判断SDO！！！

/****************************参数定义*********************************/
//1.命令寄存器及常用命令定义

#define CMD_NULL			0x00			//清除端口标志位使ADC工作在连续转换模式

#define CMD_SYNC0			0xFE	
#define CMD_SYNC1			0xFF

#define RW_WRITE			0x00			//写操作
#define RW_READ				0x01			//读操作

#define REG_OFFSET			0x01			//偏移寄存器
#define REG_GAIN			0x02			//增益寄存器
#define REG_CFG				0x03			//配置寄存器
#define REG_SYS_OFFSET_CAL	0x05			//系统偏移校准
#define REG_SYS_GAIN_CAL	0x06			//系统增益校准

#define MC_ONE				0x00			//单次转换模式
#define MC_CONV				0x01			//连续转换模式

//2.配置寄存器及其参数定义
	
#define FRS_0				0				//默认字输出速率
#define FRS_1				1				//按比例系数乘以5/6

#define FRS_Mode			FRS_1		

#if 1
	#define WR_120Hz		0x00			//@FRS = 0
	#define WR_60Hz			0x01			//@FRS = 0
	#define WR_30Hz			0x02			//@FRS = 0
	#define WR_15Hz			0x03			//@FRS = 0
	#define WR_7_5Hz		0x04			//@FRS = 0
	#define WR_3840Hz		0x08			//@FRS = 0			
	#define WR_1920Hz		0x09			//@FRS = 0				
	#define WR_960Hz		0x0A 			//@FRS = 0
	#define WR_480Hz		0x0B			//@FRS = 0
	#define WR_240Hz		0x0C 			//@FRS = 0
#endif

#if 0
	#define WR_100Hz		0x00			//@FRS = 1
	#define WR_50Hz			0x01			//@FRS = 1
	#define WR_25Hz			0x02			//@FRS = 1
	#define WR_12_5Hz		0x03			//@FRS = 1
	#define WR_6_2_5Hz		0x04			//@FRS = 1
	#define WR_3200Hz		0x08			//@FRS = 1			
	#define WR_1600Hz		0x09			//@FRS = 1				
	#define WR_800Hz		0x0A 			//@FRS = 1
	#define WR_400Hz		0x0B			//@FRS = 1
	#define WR_200Hz		0x0C 			//@FRS = 1
#endif

#define VREF_0				0x00			//2.5 V < VREF < [(VA+) - (VA-)]
#define VREF_1				0x01			//1.0 V < VREF < 2.5V

#define ADC_VREF			VREF_1			//默认使用5V

#define Unipolar			0x01			//单极性
#define Bipolar				0x00			//双极性


typedef enum{
	cs5530Channel1 = 0,
	cs5530Channel2,
	cs5530Channel3,
	cs5530ChannelNumMax,
}cs5530Channel_e;

typedef enum{
	cs5530NoStart = 0,
	cs5530RunNormal,
	cs5530RunAbnormal,
}cs5530RunState_e;

typedef struct{
	uint8_t runState;
	uint8_t csChannel;
	int32_t Code[cs5530ChannelNumMax];
	float Voltage[cs5530ChannelNumMax];
	float Value[cs5530ChannelNumMax];
}cs5530_t;

extern cs5530_t cs5530;

/*****************************接口函数*********************************/

void adc_sw_Reset(void);					//ADC软复位
void adc_Write_CFG_Register(unsigned char nVREF,unsigned char nFRS,unsigned char nWRX,unsigned char nUB);	//写配置寄存器
void adc_Read_CFG_Register(void);			//读配置寄存器
void adc_Write_Gain_Register(long int g_Value);		//写增益寄存器
void adc_Read_Gain_Register(void);			//读增益寄存器
void adc_Write_Offset_Register(long int r_Value);	//写偏移寄存器
void adc_Initl(void);						//初始化
signed long int adc_Read_Data_Register(void);		//读数据寄存器函数
void cs5530MultiCollect(uint8_t channel);
void cs5530Init(void);
void cs5530DataGet(void);
void cs5530ResetMonitor(void);
#endif


