#include "ADS1274.h"
#ifndef hspi3
#define hspix NULL
#else
#define hspix hspi3 //
#endif // !hspi1

void ADS1274_Par_Init(void);
void ADS1274_SYNC(void);
 
ads1274_t ads1274_par;
void ADS1274_Init()
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//模拟ADS_CLK晶振信号20MHz
	ADS1274_Par_Init();
	ADS1274_SYNC();

}

void ADS1274_Par_Init(void)
{
	uint8_t i=0;
	ads1274_par.channelNum=ADS_CHANNEL_NUM;
	for(i=0;i<ads1274_par.channelNum;i++)
	ads1274_par.valueFilter[i]=0;
	
}

void ADS1274_SYNC(void)
{
	HAL_GPIO_WritePin(ADC_DRDY_PORT, ADC_SYNC, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(ADC_DRDY_PORT, ADC_SYNC, GPIO_PIN_SET);
	HAL_Delay(1);
}


//从SPIx读ADC值
void ADS1274_Read_Data(ads1274_t *par,SPI_HandleTypeDef *_hspix)
{
	uint8_t *p = &par->buf[0][0];	
		HAL_SPI_Receive(_hspix,p,9,1000);
}

void ADS1274_Data_Process(ads1274_t *par)
{
	uint8_t (*p)[3] = par->buf;	
	uint8_t i=0;
	for(i=0;i<par->channelNum;i++)//数据重组
	{
		// if( (p[i][0]&0x80) == 0x00)
		// par->valueFilter[i] = *(int32_t*)p[i];
		// else
		// {
		// 	par->valueFilter[i] = *(int32_t*)p[i]-0x1000000;	
		// }
		if( (p[i][0]&0x80) == 0x00)
		par->valueFilter[i] = (p[i][0]<<16)+(p[i][1]<<8)+p[i][2];
		else
		{
			par->valueFilter[i] = ((p[i][0]<<16)+(p[i][1]<<8)+p[i][2])-0x1000000;	
		}
	}
}










