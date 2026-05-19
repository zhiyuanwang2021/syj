/**********************************************************************	 
* 公司名称 : 麦恩迪电路设计有限公司
* 模块名称 ：CS550驱动
* 系统主频 : 72 MHz
* 创建人   : eysmcu
* 修改人   : eysmcu 
* 创建日期 : 2019年10月12日
* 淘宝网址 : http://mindesigner.taobao.com
* 淘宝ID号 : eysmcu

						  版权所有 @ 麦恩迪
***********************************************************************/

#include "CS553X.h"	
#include "usart.h"
#include "delay.h"
#include "elog.h"
/****************************参数定义**********************************/
//1.命令寄存器
union REG_CMD  									//命令寄存器定义						
{											
	unsigned char BYTE;							//Byte  Access														
	struct 										//Bit   Access
	{											
		unsigned char REG:3;					//寄存器选择
		unsigned char RW :1;					//读写控制
		unsigned char    :2;					//NU
		unsigned char MC :1;					//转换模式
		unsigned char B7 :1;					//最高位
	}	BIT;
}CMD;

//2.配置寄存器(D31-D16) 	
union REG_CONFIG_H  							//配置寄存器定义					
{										
	unsigned short WORD;						//WORD Access 			
	struct 										
	{											//Byte Access	
		unsigned char L;					
		unsigned char H;					
	}	BYTE;																	
	struct 										//Bit  Access
	{											
		unsigned char      :3;					//NU
		unsigned char FRS  :1;					//滤波器速度选择
		unsigned char      :3;					//NU
		unsigned char A0   :1;					//输出锁存位0
		unsigned char A1   :1;					//输出锁存位1
		unsigned char VRS  :1;					//参考电压选择
		unsigned char      :1;					//NU
		unsigned char IS   :1;					//输入短路
		unsigned char RV   :1;					//有效复位
		unsigned char RS   :1;					//系统复位
		unsigned char PDW  :1;					//节电模式
		unsigned char PSS  :1;					//节电模式选择		
	}	BIT;
}CONFIG_H;

//3.配置寄存器(D15-D00)
union REG_CONFIG_L  							//配置寄存器定义 						
{										
	unsigned short WORD;						//WORD Access 			
	struct 										
	{											//Byte Access	
		unsigned char L;					
		unsigned char H;					
	}	BYTE;																	
	struct 										//Bit  Access
	{											
		unsigned char      :8;					//NU									
		unsigned char      :1;					//NU
		unsigned char OCD  :1;					//开路检测位
		unsigned char UBP  :1;					//信号极性选择
		unsigned char WR3_0:4;					//字速率
		unsigned char      :1;					//NU	
	}	BIT;
}CONFIG_L;

//4.数据寄存器(D31-D16)	
union REG_DATA_H 								//数据寄存器16位(D31-D16)			
{										
	unsigned short WORD;						//WORD Access 			
	struct 										
	{											
		unsigned char L;					
		unsigned char H;						//Byte Access						
	}	BYTE;																	
}DATA_H;

//5.数据寄存器(D15-D0)	
union REG_DATA_L 								//数据寄存器16位(D15-D0)			
{										
	unsigned short WORD;						//WORD Access 			
	struct 										
	{											
		unsigned char L;					
		unsigned char H;						//Byte Access						
	}	BYTE;
	struct 										//Bit  Access
	{	
		unsigned char      :2;					//NU
		unsigned char OF   :1;					//超范围标志位
		unsigned char      :5;					//NU
		unsigned char D15_8:8;					//转换结果LSB
	}	BIT;	
}DATA_L;

cs5530_t cs5530;

/**********************************************************************
* 名称 : adc_sw_Reset(void)
* 功能 : ADC软复位函数
* 输入 : 无
* 输出 ：无
* 说明 : 无
***********************************************************************/
void adc_sw_Reset(void)
{
	unsigned char x= 0x00;
	//NSS_RESET;									//CS = 0
	CS1_RESET;
	CS2_RESET;
	CS3_RESET;
	delay_us(10);
	for(x = 0;x < 20;x ++)	   					//至少发送15个SYNC1
	{
		spi_Write_Byte(CMD_SYNC1);	  			//0xFF
		delay_us(10);
	}
	spi_Write_Byte(CMD_SYNC0);			 		//0xFE	
	//NSS_SET;									//CS = 1
	CS1_SET;
	CS2_SET;
	CS3_SET;
}
/**********************************************************************
* 名称 : adc_Write_CFG_Register(unsigned char nVREF,
							    unsigned char nFRS,
								unsigned char nWRX,	
								unsigned char nUB)
* 功能 : ADC写配置寄存器
* 输入 : .........
* 输出 ：无
* 说明 :  关于nUB极性变量设置的重要说明

#define Unipolar			0x00				//单极性
#define Bipolar				0x01				//双极性

1.通常情况下ADC的单极性是指单端信号，既以GND为参考的信号，而双极性信号
指差分信号，CS553X信号的模拟部分实际是支持正负激励的，既VA+接正压，而
VA-接负压，两者的差值绝对值不超过6V即可，因此，CS553X芯片在设置nUB极性
的时候需要注意，采用正压激励的时候必须设置为单极性，采用正负压激励的时
候设置为双极性，否则，读取数据寄存器的值会发生异常；

2.实际使用中，特别是采集电桥信号的时候，用户可以采用6V作为激励；
***********************************************************************/
void adc_Write_CFG_Register(unsigned char nVREF,//基准电压
							unsigned char nFRS,	//滤波器速度
							unsigned char nWRX,	//速率字
							unsigned char nUB)	//极性
{
	CONFIG_H.WORD = 0x00;
	CONFIG_L.WORD = 0x00;
	CMD.BYTE      = 0x00;
	
	//1.设置命令寄存器
	CMD.BIT.REG = REG_CFG;						//选择配置寄存器
	CMD.BIT.RW  = RW_WRITE;						//写操作
	CMD.BIT.MC  = 0;							//模式写0
	CMD.BIT.B7  = 0;							//最高位写0
	
	//2.设置配置寄存器
	CONFIG_H.BIT.VRS = nVREF;					//设置参考电压
	CONFIG_H.BIT.FRS = nFRS;					//设置滤波器速度
	CONFIG_L.BIT.WR3_0 = nWRX;					//设置转换速度
	CONFIG_L.BIT.UBP = nUB;						//设置极性
	
	//3.写入寄存器
	//NSS_RESET;									//CS = 0
	CS1_RESET;
	CS2_RESET;
	CS3_RESET;
	spi_Write_Byte(CMD.BYTE);					delay_us(10);
	
	spi_Write_Byte(CONFIG_H.BYTE.H);			delay_us(10);
	spi_Write_Byte(CONFIG_H.BYTE.L);			delay_us(10);
	spi_Write_Byte(CONFIG_L.BYTE.H);			delay_us(10);
	spi_Write_Byte(CONFIG_L.BYTE.L);			delay_us(10);
	//NSS_SET;									//CS = 1	
	CS1_SET;
	CS2_SET;
	CS3_SET;
}
/**********************************************************************
* 名称 : adc_Read_CFG_Register(void)
* 功能 : ADC读配置寄存器
* 输入 : 无
* 输出 ：无
* 说明 : 无
***********************************************************************/
void adc_Read_CFG_Register(void)
{
	CONFIG_H.WORD = 0x00;					
	CONFIG_L.WORD = 0x00;
	CMD.BYTE      = 0x00;
	
	//1.设置命令寄存器
	CMD.BIT.REG = REG_CFG;						//选择配置寄存器
	CMD.BIT.RW  = RW_READ;						//读操作
	CMD.BIT.MC  = 0;							//模式写0
	CMD.BIT.B7  = 0;							//最高位写0	

	//2.读取寄存器数据
	NSS_RESET;									//CS = 0
	spi_Write_Byte(CMD.BYTE);					delay_us(10);
	
	CONFIG_H.BYTE.H = spi_Read_Byte();			delay_us(10);
	CONFIG_H.BYTE.L = spi_Read_Byte();			delay_us(10);
	CONFIG_L.BYTE.H = spi_Read_Byte();			delay_us(10);
	CONFIG_L.BYTE.L = spi_Read_Byte();			delay_us(10);
	NSS_SET;									//CS = 1		
}
/**********************************************************************
* 名称 : adc_Write_Gain_Register(long int g_Value)
* 功能 : ADC写增益寄存器
* 输入 : g_Value -- 增益值
* 输出 ：无
* 说明 : 实际设置时增益寄存器的值不要超过40！！！！
***********************************************************************/
void adc_Write_Gain_Register(long int g_Value)
{
	unsigned char x = 0x0;
	
	CMD.BYTE = 0x00;
	
	//1.设置命令寄存器
	CMD.BIT.REG = REG_GAIN;						//选择增益寄存器
	CMD.BIT.RW  = RW_WRITE;						//写操作
	CMD.BIT.MC  = 0;							//模式写0
	CMD.BIT.B7  = 0;							//最高位写0			
	
	//2.写入寄存器数据
	//NSS_RESET;									//CS = 0
	CS1_RESET;
	CS2_RESET;
	CS3_RESET;
	spi_Write_Byte(CMD.BYTE);
	delay_us(10);
	for(x = 4;x > 0;x --)
	{
		spi_Write_Byte(g_Value >> (x - 1) * 8);
		delay_us(10);
	}
	//NSS_SET;									//CS = 1		
	CS1_SET;
	CS2_SET;
	CS3_SET;
}
/**********************************************************************
* 名称 : adc_Read_Gain_Register(void)
* 功能 : ADC读增益寄存器
* 输入 : 无
* 输出 ：无
* 说明 : 无
***********************************************************************/
void adc_Read_Gain_Register(void)
{
	long int REG_Value = 0x00;
	unsigned char x = 0x00;
	
	CMD.BYTE = 0x00;
	
	//1.设置命令寄存器
	CMD.BIT.REG = REG_GAIN;						//选择增益寄存器
	CMD.BIT.RW  = RW_READ;						//读操作
	CMD.BIT.MC  = 0;							//模式写0
	CMD.BIT.B7  = 0;							//最高位写0	

	//2.读取寄存器数据
	NSS_RESET;									//CS = 0
	spi_Write_Byte(CMD.BYTE);				
	delay_us(10);
	for(x = 0;x < 4;x ++)
	{
		REG_Value = (REG_Value << 8) + spi_Read_Byte();
		delay_us(10);
	}
	NSS_SET;									//CS = 1		
}
/**********************************************************************
* 名称 : adc_Write_Offset_Register(long int r_Value)
* 功能 : ADC写偏移寄存器
* 输入 : r_Value -- 寄存器数据
* 输出 ：无
* 说明 : 无
***********************************************************************/
void adc_Write_Offset_Register(long int r_Value)
{
	unsigned char x = 0x0;
	
	CMD.BYTE = 0x00;
	
	//1.设置命令寄存器
	CMD.BIT.REG = REG_OFFSET;					//选择偏移寄存器
	CMD.BIT.RW  = RW_WRITE;						//写操作
	CMD.BIT.MC  = 0;							//模式写0
	CMD.BIT.B7  = 0;							//最高位写0			
	
	//2.写入寄存器数据
	//NSS_RESET;									//CS = 0
	CS1_RESET;
	CS2_RESET;
	CS3_RESET;
	spi_Write_Byte(CMD.BYTE);
	delay_us(10);
	for(x = 4;x > 0;x --)
	{
		spi_Write_Byte(r_Value >> (x - 1) * 8);
		delay_us(10);
	}
	//NSS_SET;									//CS = 1	
	CS1_SET;
	CS2_SET;
	CS3_SET;	
}
/**********************************************************************
* 名称 : adc_Start(unsigned char nMODE)
* 功能 : ADC启动转换
* 输入 : nMODE -- 转换模式
* 输出 ：无
* 说明 : 无
***********************************************************************/
void adc_Start(unsigned char nMODE)
{
	CMD.BYTE = 0x00;
	
	//1.设置命令寄存器
	CMD.BIT.REG = 0x00;						
	CMD.BIT.RW  = 0x00;							
	CMD.BIT.MC  = nMODE;						//设置转换模式
	CMD.BIT.B7  = 1;							//最高位写1
	
	//2.写入模式并启动转换
	//NSS_RESET;									//CS = 0
	CS1_RESET;
	CS2_RESET;
	CS3_RESET;
	spi_Write_Byte(CMD.BYTE);				
//	NSS = 1;									//CS = 1	
	CS1_SET;
	CS2_SET;
	CS3_SET;	
}
/**********************************************************************
* 名称 : adc_Initl(void)
* 功能 : ADC初始化函数
* 输入 : 无
* 输出 ：无
* 说明 : 无
***********************************************************************/
void adc_Initl(void)
{
	adc_sw_Reset();								//复位ADC
	//printf("adc_sw_Reset");
	//delay_ms(1);
	adc_Write_CFG_Register(ADC_VREF,FRS_Mode,WR_60Hz,Bipolar);		//设置配置寄存器
	//printf("adc_Write_CFG_Register");
	//delay_ms(1);
	adc_Write_Offset_Register(0x00000000);		//写偏移寄存器	
	//printf("adc_Write_Offset_Register");
	//delay_ms(1);
	adc_Write_Gain_Register(0x01000000);		//写增益寄存器
	//printf("adc_Write_Gain_Register");
	//delay_ms(1);
	adc_Start(MC_CONV);							//启动转换
	cs5530.csChannel = cs5530Channel1;
	CS1_RESET;
	//printf("adc_Start");
}
/**********************************************************************
* 名称 : adc_Read_Data_Register(void)
* 功能 : ADC读数据寄存器函数
* 输入 : 无
* 输出 ：无
* 说明 : 无
***********************************************************************/
signed long int adc_Read_Data_Register(void)
{
	signed long int REG_Value = 0x00;
	
	DATA_H.WORD = DATA_L.WORD = 0x00;			//清除寄存器数据
	
	//1.等待ADC转换完成
//	while(RDY != 0);							//RDY = 0???
	
	//2.读取ADC数据寄存器数据
//	NSS = 0;									delay_us(10);
	spi_Write_Byte(CMD_NULL);					delay_us(1);    	
	
	DATA_H.BYTE.H =	spi_Read_Byte();			delay_us(1); 
	DATA_H.BYTE.L =	spi_Read_Byte();			delay_us(1); 
	DATA_L.BYTE.H =	spi_Read_Byte();			delay_us(1); 
	DATA_L.BYTE.L =	spi_Read_Byte();			delay_us(1); 
	
//	printf("DATA_L.BYTE.L:0x%2x\r\n",DATA_L.BYTE.L);
//	printf("0x%2x,0x%2x,0x%2x,0x%2x\r\n",DATA_H.BYTE.H,DATA_H.BYTE.L,DATA_L.BYTE.H,DATA_L.BYTE.L);
//	
//	printf("0x%4x,0x%4x\r\n",DATA_H.WORD,DATA_L.WORD);
	
//	printf("DATA_H.BYTE.H:0x%x\r\n",DATA_H.BYTE.H);
//	printf("DATA_H.BYTE.L:0x%x\r\n",DATA_H.BYTE.L);
//	printf("DATA_L.BYTE.H:0x%x\r\n",DATA_L.BYTE.H);
//	printf("DATA_L.BYTE.L:0x%x\r\n",DATA_L.BYTE.L);

//	NSS = 1;									delay_us(10);
	
    //unipolar
	// REG_Value = DATA_H.WORD;
	// REG_Value =(((REG_Value << 12))&0xFFFF0000) + DATA_L.BYTE.H;

	//bipolar
	if( (DATA_H.BYTE.H&0x80) == 0x00)
	REG_Value = ((((DATA_H.WORD << 8))&0xFFFF00) + DATA_L.BYTE.H) & 0x7FFFFF;
	else
	{
		REG_Value = -((~((((DATA_H.WORD << 8))&0xFFFF00) + DATA_L.BYTE.H) + 1) & 0x7FFFFF);
	}


//	printf("REG_Value:0x%4x\r\n",REG_Value);
//	printf("REG_Value>>8:0x%4x\r\n",((REG_Value >> 8)&0x00FFFFFF));
	//3.判断是否超量程
	if(DATA_L.BIT.OF == 1)						//超过量程
	{
		//用户自行添加代码！！！！
	}
	
	return(REG_Value);		   				//输出数据(舍弃低8位)	
}

#define CS5530_V_SCALE 39.0625 //mV 2500 / 64   2500对应2^23 - 1;5000对应2^24 - 1;	

void cs5530MultiCollect(uint8_t channel)
{
  int32_t _code = 0x00;
  float _voltage;
	_code = adc_Read_Data_Register();
  //unipolar
  _voltage = (float)((double)_code / 8388606 * CS5530_V_SCALE );
  //bipolar
  switch (channel)
  {
  case cs5530Channel1:
  cs5530.Code[cs5530Channel1] = _code;
  cs5530.Voltage[cs5530Channel1] = _voltage;
    break;
  case cs5530Channel2:
  cs5530.Code[cs5530Channel2] = _code;
  cs5530.Voltage[cs5530Channel2] = _voltage;
    break;
  case cs5530Channel3:
  cs5530.Code[cs5530Channel3] = _code;
  cs5530.Voltage[cs5530Channel3] = _voltage;
    break;
  default:
    break;
  }
}

void cs5530Init(void){
	cs5530.runState = cs5530NoStart;
	delay_init(HAL_RCC_GetSysClockFreq());//软件延时初始化
	//HAL_NVIC_DisableIRQ(P_ADCMISO_EXTI_IRQn);
	spi_io_Initl();						//软件SPI初始化
	//delay_ms(10);
	adc_Initl();						//初始化CS5530
	cs5530.runState = cs5530RunNormal;
}
	
void cs5530DataProcess(uint8_t channel){
	switch (channel){
		case cs5530Channel1:

		break;
		case cs5530Channel2:

		break;
		case cs5530Channel3:

		break;
		default:
		break;
	}
}

void cs5530DataGet(void){
	extern uint32_t okCounter;
	uint8_t readOk = 0;
	if(cs5530.runState == cs5530RunNormal){
		if(SDO_READ == GPIO_PIN_RESET){
			cs5530MultiCollect(cs5530.csChannel);
			readOk = 1;	
			okCounter++;
		}
		if(readOk == 1){
			if(++cs5530.csChannel >= cs5530ChannelNumMax)	
			cs5530.csChannel = cs5530Channel1;
			switch (cs5530.csChannel)
			{
			case cs5530Channel1:
			CS3_SET;
			CS2_SET;
			CS1_RESET;
			break;
			case cs5530Channel2:
			CS1_SET;
			CS3_SET;
			CS2_RESET;
			break;
			case cs5530Channel3:
			CS2_SET;
			CS1_SET;
			CS3_RESET;
			break;
			default:
			break;
			}
		}
	}
}

void cs5530ResetMonitor(void){
	static uint8_t _abnormalCounter = 0;
	static int32_t _CodeRecord[3] = 0;
	uint8_t i = 0;
	if(cs5530.runState == cs5530RunNormal){
		if(cs5530.Code[cs5530Channel1] - _CodeRecord[cs5530Channel1] == 0 
		&& cs5530.Code[cs5530Channel2] - _CodeRecord[cs5530Channel2] == 0 
		&& cs5530.Code[cs5530Channel3] - _CodeRecord[cs5530Channel3] == 0){
			_abnormalCounter++;
			if(_abnormalCounter >= 100){
				cs5530.runState = cs5530RunAbnormal;
				log_i("cs5530 Run Abnormal");
				cs5530Init();
				_abnormalCounter = 0;
			}
		}
	}
	for(;i<3;i++){
		_CodeRecord[i] = cs5530.Code[i];
	}
}
