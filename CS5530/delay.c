/**********************************************************************	 
* 公司名称 : 麦恩迪电路设计有限公司
* 模块名称 ：延时函数
* 系统主频 : 72 MHz
* 创建人   : eysmcu
* 修改人   : eysmcu 
* 创建日期 : 2019年10月12日
* 淘宝网址 : http://mindesigner.taobao.com
* 淘宝ID号 : eysmcu
* 注意事项 ：该程序借鉴了ALIENTEK STM32开发板例程！！！
***********************************************************************/

#include "delay.h"									 			//延时
#include "tim.h"
#include "gParameter.h"
#include "cmsis_os.h"
													   	
/**
*	@brief	delay clk init
*	@para[in] rcc clk	usually，the para should be the retval of function-HAL_RCC_GetSysClockFreq(void)
*	@revtal	num
*	@date	2024-3-15 14:50:31
*	@auther	Pan
*/

// static uint32_t fac_us;

void delay_init(uint32_t rcc_clk)
{
	// fac_us = (uint32_t)(rcc_clk/1000000.0f/8.0f);
    // printf("fac_us:%d\r\n",fac_us);
    HAL_TIM_Base_Start(&htim14);
}   
														
/**********************************************************************
* 名称 : delay_us(u32 nus)	 
* 功能 : 延时nus函数
* 输入 : nus为要延时的us数.
* 输出 : 无
* 说明 : 无
***********************************************************************/		    								   
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
    uint32_t told,tnow,tcnt=0;
    uint32_t reload=200-1;
    ticks=nus*20; 
    __HAL_TIM_SET_COUNTER(&htim14,0);
    told = 0; 
    while(1)
    {
        tnow=__HAL_TIM_GET_COUNTER(&htim14);
        if(tnow!=told)
        {
            if(tnow>told)tcnt+=tnow-told;
            else tcnt+=reload+tnow-told;
            told=tnow;
            if(tcnt>=ticks)break; 
        }
    };	
    __HAL_TIM_SET_COUNTER(&htim14,0);		    
}
/**********************************************************************
* 名称 : delay_ms(u16 nms)	 
* 功能 : 延时nms函数
* 输入 : nms:要延时的ms数.
* 输出 : 无
* 说明 : 无
***********************************************************************/
void delay_ms(uint16_t nus)
{	
    if(stateFlag.os_start==0)
        HAL_Delay(nus);//此行不可少
    else
        osDelay(nus);
}

