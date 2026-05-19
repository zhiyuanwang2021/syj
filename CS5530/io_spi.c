/**********************************************************************	 
* 公司名称 : 麦恩迪电路设计有限公司
* 模块名称 ：IO模拟SPI
* 系统主频 : 72 MHz
* 创建人   : eysmcu
* 修改人   : eysmcu 
* 创建日期 : 2019年10月12日
* 淘宝网址 : http://mindesigner.taobao.com
* 淘宝ID号 : eysmcu
* 软件版本 : Keil UV5 14.0.0

						  版权所有 @ 麦恩迪
***********************************************************************/

#include "io_spi.h"

/**********************************************************************
* 名称 : spi_io_Initl(void)
* 功能 : SPI端口初始化
* 输入 : 无
* 输出 ：无
* 说明 : 无
***********************************************************************/
void spi_io_Initl(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;   				//GPIO端口设置

//	//1.设置IO时钟
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA	 		//使能GPIO端口时钟
//	                      |RCC_APB2Periph_GPIOB
//						  |RCC_APB2Periph_GPIOC
//						  |RCC_APB2Periph_GPIOD
//						  |RCC_APB2Periph_GPIOE
//						  |RCC_APB2Periph_GPIOF
//						  |RCC_APB2Periph_GPIOG, ENABLE);
//						  
//	//2.输出端口初始化	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12			//PB12 --> NSS(SPI/NSS )
//								| GPIO_Pin_13			//PB13 --> SCK(SPI/SCLK)
//								| GPIO_Pin_15; 			//PB15 --> SDI(SPI/MOSI)
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//设置为推挽式输出	
//    GPIO_Init(GPIOB, &GPIO_InitStructure); 
//	
//	//3.输入端口定义
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;			//PB14 --> SDO(SPI/MISO)
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//设置为上拉输入	
//    GPIO_Init(GPIOB, &GPIO_InitStructure); 	
	
	//3.设置端口初始电平
	NSS_SET;
	CS1_SET;
	CS2_SET;
	CS3_SET;	
	SCK_RESET;											//串行同步时钟的空闲状态为低电平	
	SDI_SET;
}
/**********************************************************************
* 名称 : spi_Write_Byte(unsigned char _1Byte)
* 功能 : 单字节写函数
* 输入 : _1Byte -- 单字节数据
* 输出 ：无
* 说明 : 无
***********************************************************************/
void spi_Write_Byte(unsigned char _1Byte)
{
	unsigned char x = 0x00;
	SCK_RESET;											//SCK = 0
	for(x = 0;x < 8;x ++)
	{
		if((_1Byte & 0x80) == 0x80)					
			 SDI_SET;									//SDI = 1
		else SDI_RESET;									//SDI = 0
		
		SCK_SET;			delay_us(1);				//SCK = 1
		SCK_RESET;			delay_us(1);				//SCK = 0
		_1Byte = _1Byte << 1;	
	}	
}
/**********************************************************************
* 名称 : spi_Read_Byte(void)
* 功能 : 单字节读函数
* 输入 : 无
* 输出 ：_1Byte -- 数据
* 说明 : 无
***********************************************************************/
unsigned char spi_Read_Byte(void)
{
	unsigned char _1Byte = 0x00,x = 0x00;
	SCK_RESET;											//SCK = 0
	for(x = 0;x < 8;x ++)
	{
		_1Byte = _1Byte << 1;
		SCK_SET;			delay_us(1);				//SCK = 1
		if(SDO_READ == GPIO_PIN_SET)									//SDO = 1
			 _1Byte = _1Byte + 0x01;
		else _1Byte = _1Byte & 0xFE;					//SDO = 0
		SCK_RESET;			delay_us(1);				//SCK = 0		
	}
	return(_1Byte);										//返回值
}
	





