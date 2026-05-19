#ifndef  __DELAY_H__
#define  __DELAY_H__

#include "main.h"

/*****************************接口函数*********************************/

extern void delay_init(uint32_t rcc_clk);
extern void delay_ms(uint16_t nms);					//ms延时函数
extern void delay_us(uint32_t nus);				  	//us延时函数

#endif




