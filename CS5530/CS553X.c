/*
 * CS5530 driver disabled for CS5552 migration.
 *
 * Notes:
 * 1. Legacy CS5530 implementation is preserved below in a disabled block.
 * 2. Public API names and channel order are intentionally kept unchanged.
 * 3. SPI pins, chip-select order, and application call sites stay in place
 *    so the CS5552 driver can be inserted with minimal upper-layer changes.
 */
#if 0
/**********************************************************************	 
* 鍏徃鍚嶇О : 楹︽仼杩數璺璁℃湁闄愬叕鍙?
* 妯″潡鍚嶇О 锛欳S550椹卞姩
* 绯荤粺涓婚 : 72 MHz
* 鍒涘缓浜?  : eysmcu
* 淇敼浜?  : eysmcu 
* 鍒涘缓鏃ユ湡 : 2019骞?0鏈?2鏃?
* 娣樺疂缃戝潃 : http://mindesigner.taobao.com
* 娣樺疂ID鍙?: eysmcu

						  鐗堟潈鎵€鏈?@ 楹︽仼杩?
***********************************************************************/

#include "CS553X.h"	
#include "usart.h"
#include "delay.h"
#include "elog.h"
/****************************鍙傛暟瀹氫箟**********************************/
//1.鍛戒护瀵勫瓨鍣?
union REG_CMD  									//鍛戒护瀵勫瓨鍣ㄥ畾涔?					
{											
	unsigned char BYTE;							//Byte  Access														
	struct 										//Bit   Access
	{											
		unsigned char REG:3;					//瀵勫瓨鍣ㄩ€夋嫨
		unsigned char RW :1;					//璇诲啓鎺у埗
		unsigned char    :2;					//NU
		unsigned char MC :1;					//杞崲妯″紡
		unsigned char B7 :1;					//鏈€楂樹綅
	}	BIT;
}CMD;

//2.閰嶇疆瀵勫瓨鍣?D31-D16) 	
union REG_CONFIG_H  							//閰嶇疆瀵勫瓨鍣ㄥ畾涔?				
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
		unsigned char FRS  :1;					//婊ゆ尝鍣ㄩ€熷害閫夋嫨
		unsigned char      :3;					//NU
		unsigned char A0   :1;					//杈撳嚭閿佸瓨浣?
		unsigned char A1   :1;					//杈撳嚭閿佸瓨浣?
		unsigned char VRS  :1;					//鍙傝€冪數鍘嬮€夋嫨
		unsigned char      :1;					//NU
		unsigned char IS   :1;					//杈撳叆鐭矾
		unsigned char RV   :1;					//鏈夋晥澶嶄綅
		unsigned char RS   :1;					//绯荤粺澶嶄綅
		unsigned char PDW  :1;					//鑺傜數妯″紡
		unsigned char PSS  :1;					//鑺傜數妯″紡閫夋嫨		
	}	BIT;
}CONFIG_H;

//3.閰嶇疆瀵勫瓨鍣?D15-D00)
union REG_CONFIG_L  							//閰嶇疆瀵勫瓨鍣ㄥ畾涔?						
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
		unsigned char OCD  :1;					//寮€璺娴嬩綅
		unsigned char UBP  :1;					//淇″彿鏋佹€ч€夋嫨
		unsigned char WR3_0:4;					//瀛楅€熺巼
		unsigned char      :1;					//NU	
	}	BIT;
}CONFIG_L;

//4.鏁版嵁瀵勫瓨鍣?D31-D16)	
union REG_DATA_H 								//鏁版嵁瀵勫瓨鍣?6浣?D31-D16)			
{										
	unsigned short WORD;						//WORD Access 			
	struct 										
	{											
		unsigned char L;					
		unsigned char H;						//Byte Access						
	}	BYTE;																	
}DATA_H;

//5.鏁版嵁瀵勫瓨鍣?D15-D0)	
union REG_DATA_L 								//鏁版嵁瀵勫瓨鍣?6浣?D15-D0)			
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
		unsigned char OF   :1;					//瓒呰寖鍥存爣蹇椾綅
		unsigned char      :5;					//NU
		unsigned char D15_8:8;					//杞崲缁撴灉LSB
	}	BIT;	
}DATA_L;

cs5530_t cs5530;

/**********************************************************************
* 鍚嶇О : adc_sw_Reset(void)
* 鍔熻兘 : ADC杞浣嶅嚱鏁?
* 杈撳叆 : 鏃?
* 杈撳嚭 锛氭棤
* 璇存槑 : 鏃?
***********************************************************************/
void adc_sw_Reset(void)
{
	unsigned char x= 0x00;
	//NSS_RESET;									//CS = 0
	CS1_RESET;
	CS2_RESET;
	CS3_RESET;
	delay_us(10);
	for(x = 0;x < 20;x ++)	   					//鑷冲皯鍙戦€?5涓猄YNC1
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
* 鍚嶇О : adc_Write_CFG_Register(unsigned char nVREF,
							    unsigned char nFRS,
								unsigned char nWRX,	
								unsigned char nUB)
* 鍔熻兘 : ADC鍐欓厤缃瘎瀛樺櫒
* 杈撳叆 : .........
* 杈撳嚭 锛氭棤
* 璇存槑 :  鍏充簬nUB鏋佹€у彉閲忚缃殑閲嶈璇存槑

#define Unipolar			0x00				//鍗曟瀬鎬?
#define Bipolar				0x01				//鍙屾瀬鎬?

1.閫氬父鎯呭喌涓婣DC鐨勫崟鏋佹€ф槸鎸囧崟绔俊鍙凤紝鏃互GND涓哄弬鑰冪殑淇″彿锛岃€屽弻鏋佹€т俊鍙?
鎸囧樊鍒嗕俊鍙凤紝CS553X淇″彿鐨勬ā鎷熼儴鍒嗗疄闄呮槸鏀寔姝ｈ礋婵€鍔辩殑锛屾棦VA+鎺ユ鍘嬶紝鑰?
VA-鎺ヨ礋鍘嬶紝涓よ€呯殑宸€肩粷瀵瑰€间笉瓒呰繃6V鍗冲彲锛屽洜姝わ紝CS553X鑺墖鍦ㄨ缃畁UB鏋佹€?
鐨勬椂鍊欓渶瑕佹敞鎰忥紝閲囩敤姝ｅ帇婵€鍔辩殑鏃跺€欏繀椤昏缃负鍗曟瀬鎬э紝閲囩敤姝ｈ礋鍘嬫縺鍔辩殑鏃?
鍊欒缃负鍙屾瀬鎬э紝鍚﹀垯锛岃鍙栨暟鎹瘎瀛樺櫒鐨勫€间細鍙戠敓寮傚父锛?

2.瀹為檯浣跨敤涓紝鐗瑰埆鏄噰闆嗙數妗ヤ俊鍙风殑鏃跺€欙紝鐢ㄦ埛鍙互閲囩敤6V浣滀负婵€鍔憋紱
***********************************************************************/
void adc_Write_CFG_Register(unsigned char nVREF,//鍩哄噯鐢靛帇
							unsigned char nFRS,	//婊ゆ尝鍣ㄩ€熷害
							unsigned char nWRX,	//閫熺巼瀛?
							unsigned char nUB)	//鏋佹€?
{
	CONFIG_H.WORD = 0x00;
	CONFIG_L.WORD = 0x00;
	CMD.BYTE      = 0x00;
	
	//1.璁剧疆鍛戒护瀵勫瓨鍣?
	CMD.BIT.REG = REG_CFG;						//閫夋嫨閰嶇疆瀵勫瓨鍣?
	CMD.BIT.RW  = RW_WRITE;						//鍐欐搷浣?
	CMD.BIT.MC  = 0;							//妯″紡鍐?
	CMD.BIT.B7  = 0;							//鏈€楂樹綅鍐?
	
	//2.璁剧疆閰嶇疆瀵勫瓨鍣?
	CONFIG_H.BIT.VRS = nVREF;					//璁剧疆鍙傝€冪數鍘?
	CONFIG_H.BIT.FRS = nFRS;					//璁剧疆婊ゆ尝鍣ㄩ€熷害
	CONFIG_L.BIT.WR3_0 = nWRX;					//璁剧疆杞崲閫熷害
	CONFIG_L.BIT.UBP = nUB;						//璁剧疆鏋佹€?
	
	//3.鍐欏叆瀵勫瓨鍣?
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
* 鍚嶇О : adc_Read_CFG_Register(void)
* 鍔熻兘 : ADC璇婚厤缃瘎瀛樺櫒
* 杈撳叆 : 鏃?
* 杈撳嚭 锛氭棤
* 璇存槑 : 鏃?
***********************************************************************/
void adc_Read_CFG_Register(void)
{
	CONFIG_H.WORD = 0x00;					
	CONFIG_L.WORD = 0x00;
	CMD.BYTE      = 0x00;
	
	//1.璁剧疆鍛戒护瀵勫瓨鍣?
	CMD.BIT.REG = REG_CFG;						//閫夋嫨閰嶇疆瀵勫瓨鍣?
	CMD.BIT.RW  = RW_READ;						//璇绘搷浣?
	CMD.BIT.MC  = 0;							//妯″紡鍐?
	CMD.BIT.B7  = 0;							//鏈€楂樹綅鍐?	

	//2.璇诲彇瀵勫瓨鍣ㄦ暟鎹?
	NSS_RESET;									//CS = 0
	spi_Write_Byte(CMD.BYTE);					delay_us(10);
	
	CONFIG_H.BYTE.H = spi_Read_Byte();			delay_us(10);
	CONFIG_H.BYTE.L = spi_Read_Byte();			delay_us(10);
	CONFIG_L.BYTE.H = spi_Read_Byte();			delay_us(10);
	CONFIG_L.BYTE.L = spi_Read_Byte();			delay_us(10);
	NSS_SET;									//CS = 1		
}
/**********************************************************************
* 鍚嶇О : adc_Write_Gain_Register(long int g_Value)
* 鍔熻兘 : ADC鍐欏鐩婂瘎瀛樺櫒
* 杈撳叆 : g_Value -- 澧炵泭鍊?
* 杈撳嚭 锛氭棤
* 璇存槑 : 瀹為檯璁剧疆鏃跺鐩婂瘎瀛樺櫒鐨勫€间笉瑕佽秴杩?0锛侊紒锛侊紒
***********************************************************************/
void adc_Write_Gain_Register(long int g_Value)
{
	unsigned char x = 0x0;
	
	CMD.BYTE = 0x00;
	
	//1.璁剧疆鍛戒护瀵勫瓨鍣?
	CMD.BIT.REG = REG_GAIN;						//閫夋嫨澧炵泭瀵勫瓨鍣?
	CMD.BIT.RW  = RW_WRITE;						//鍐欐搷浣?
	CMD.BIT.MC  = 0;							//妯″紡鍐?
	CMD.BIT.B7  = 0;							//鏈€楂樹綅鍐?			
	
	//2.鍐欏叆瀵勫瓨鍣ㄦ暟鎹?
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
* 鍚嶇О : adc_Read_Gain_Register(void)
* 鍔熻兘 : ADC璇诲鐩婂瘎瀛樺櫒
* 杈撳叆 : 鏃?
* 杈撳嚭 锛氭棤
* 璇存槑 : 鏃?
***********************************************************************/
void adc_Read_Gain_Register(void)
{
	long int REG_Value = 0x00;
	unsigned char x = 0x00;
	
	CMD.BYTE = 0x00;
	
	//1.璁剧疆鍛戒护瀵勫瓨鍣?
	CMD.BIT.REG = REG_GAIN;						//閫夋嫨澧炵泭瀵勫瓨鍣?
	CMD.BIT.RW  = RW_READ;						//璇绘搷浣?
	CMD.BIT.MC  = 0;							//妯″紡鍐?
	CMD.BIT.B7  = 0;							//鏈€楂樹綅鍐?	

	//2.璇诲彇瀵勫瓨鍣ㄦ暟鎹?
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
* 鍚嶇О : adc_Write_Offset_Register(long int r_Value)
* 鍔熻兘 : ADC鍐欏亸绉诲瘎瀛樺櫒
* 杈撳叆 : r_Value -- 瀵勫瓨鍣ㄦ暟鎹?
* 杈撳嚭 锛氭棤
* 璇存槑 : 鏃?
***********************************************************************/
void adc_Write_Offset_Register(long int r_Value)
{
	unsigned char x = 0x0;
	
	CMD.BYTE = 0x00;
	
	//1.璁剧疆鍛戒护瀵勫瓨鍣?
	CMD.BIT.REG = REG_OFFSET;					//閫夋嫨鍋忕Щ瀵勫瓨鍣?
	CMD.BIT.RW  = RW_WRITE;						//鍐欐搷浣?
	CMD.BIT.MC  = 0;							//妯″紡鍐?
	CMD.BIT.B7  = 0;							//鏈€楂樹綅鍐?			
	
	//2.鍐欏叆瀵勫瓨鍣ㄦ暟鎹?
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
* 鍚嶇О : adc_Start(unsigned char nMODE)
* 鍔熻兘 : ADC鍚姩杞崲
* 杈撳叆 : nMODE -- 杞崲妯″紡
* 杈撳嚭 锛氭棤
* 璇存槑 : 鏃?
***********************************************************************/
void adc_Start(unsigned char nMODE)
{
	CMD.BYTE = 0x00;
	
	//1.璁剧疆鍛戒护瀵勫瓨鍣?
	CMD.BIT.REG = 0x00;						
	CMD.BIT.RW  = 0x00;							
	CMD.BIT.MC  = nMODE;						//璁剧疆杞崲妯″紡
	CMD.BIT.B7  = 1;							//鏈€楂樹綅鍐?
	
	//2.鍐欏叆妯″紡骞跺惎鍔ㄨ浆鎹?
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
* 鍚嶇О : adc_Initl(void)
* 鍔熻兘 : ADC鍒濆鍖栧嚱鏁?
* 杈撳叆 : 鏃?
* 杈撳嚭 锛氭棤
* 璇存槑 : 鏃?
***********************************************************************/
void adc_Initl(void)
{
	adc_sw_Reset();								//澶嶄綅ADC
	//printf("adc_sw_Reset");
	//delay_ms(1);
	adc_Write_CFG_Register(ADC_VREF,FRS_Mode,WR_60Hz,Bipolar);		//璁剧疆閰嶇疆瀵勫瓨鍣?
	//printf("adc_Write_CFG_Register");
	//delay_ms(1);
	adc_Write_Offset_Register(0x00000000);		//鍐欏亸绉诲瘎瀛樺櫒	
	//printf("adc_Write_Offset_Register");
	//delay_ms(1);
	adc_Write_Gain_Register(0x01000000);		//鍐欏鐩婂瘎瀛樺櫒
	//printf("adc_Write_Gain_Register");
	//delay_ms(1);
	adc_Start(MC_CONV);							//鍚姩杞崲
	cs5530.csChannel = cs5530Channel1;
	CS1_RESET;
	//printf("adc_Start");
}
/**********************************************************************
* 鍚嶇О : adc_Read_Data_Register(void)
* 鍔熻兘 : ADC璇绘暟鎹瘎瀛樺櫒鍑芥暟
* 杈撳叆 : 鏃?
* 杈撳嚭 锛氭棤
* 璇存槑 : 鏃?
***********************************************************************/
signed long int adc_Read_Data_Register(void)
{
	signed long int REG_Value = 0x00;
	
	DATA_H.WORD = DATA_L.WORD = 0x00;			//娓呴櫎瀵勫瓨鍣ㄦ暟鎹?
	
	//1.绛夊緟ADC杞崲瀹屾垚
//	while(RDY != 0);							//RDY = 0???
	
	//2.璇诲彇ADC鏁版嵁瀵勫瓨鍣ㄦ暟鎹?
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
	//3.鍒ゆ柇鏄惁瓒呴噺绋?
	if(DATA_L.BIT.OF == 1)						//瓒呰繃閲忕▼
	{
		//鐢ㄦ埛鑷娣诲姞浠ｇ爜锛侊紒锛侊紒
	}
	
	return(REG_Value);		   				//杈撳嚭鏁版嵁(鑸嶅純浣?浣?	
}

#define CS5530_V_SCALE 39.0625 //mV 2500 / 64   2500瀵瑰簲2^23 - 1;5000瀵瑰簲2^24 - 1;	

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
	delay_init(HAL_RCC_GetSysClockFreq());//杞欢寤舵椂鍒濆鍖?
	//HAL_NVIC_DisableIRQ(P_ADCMISO_EXTI_IRQn);
	spi_io_Initl();						//杞欢SPI鍒濆鍖?
	//delay_ms(10);
	adc_Initl();						//鍒濆鍖朇S5530
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
#endif

#include "CS553X.h"

cs5530_t cs5530;

static void cs5530CompatClearData(void)
{
    uint8_t i;

    for (i = 0; i < cs5530ChannelNumMax; i++) {
        cs5530.Code[i] = 0;
        cs5530.Voltage[i] = 0.0f;
        cs5530.Value[i] = 0.0f;
    }
}

void adc_sw_Reset(void)
{
    /* CS5530 retired: wait for CS5552 implementation. */
}

void adc_Write_CFG_Register(unsigned char nVREF,
                            unsigned char nFRS,
                            unsigned char nWRX,
                            unsigned char nUB)
{
    (void)nVREF;
    (void)nFRS;
    (void)nWRX;
    (void)nUB;
}

void adc_Read_CFG_Register(void)
{
}

void adc_Write_Gain_Register(long int g_Value)
{
    (void)g_Value;
}

void adc_Read_Gain_Register(void)
{
}

void adc_Write_Offset_Register(long int r_Value)
{
    (void)r_Value;
}

void adc_Initl(void)
{
    cs5530.csChannel = cs5530Channel1;
    cs5530CompatClearData();
}

signed long int adc_Read_Data_Register(void)
{
    return 0;
}

void cs5530MultiCollect(uint8_t channel)
{
    (void)channel;
}

void cs5530Init(void)
{
    cs5530.runState = cs5530NoStart;
    cs5530.csChannel = cs5530Channel1;
    cs5530CompatClearData();
}

void cs5530DataProcess(uint8_t channel)
{
    (void)channel;
}

void cs5530DataGet(void)
{
}

void cs5530ResetMonitor(void)
{
}
