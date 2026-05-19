#include "RS485.h"
#include "usart.h"
#include "CRC16.h"
#include <elog.h>
#include "in_out.h"

#ifndef sign(x)
	#define sign(x) ((x)>0?1:((x)<0?-1:1))
#endif

uint8_t rs485_rxbuf[rs485_rxbuf_size]={0};//串口单字节接收上位机Modbus指令缓冲区
uint8_t rs485_txbuf[rs485_txbuf_size]={0};//串口单字节发送上位机Modbus指令缓冲区
uint8_t buf_485[3]={0};

manualBox485_t       manualBox485={
	.mainBoardAddr = 0x01,
	.manualBoxAddr = 0x02,
	.reflag = 0,
	.recount = 0,
};
manualBoxHoldreg_t   manualBoxHoldreg;
manualBoxCoil_t      manualBoxCoil={
	.word = 0xFFFF,
};



void RS485_init(UART_HandleTypeDef *huartx)
{
	HAL_UARTEx_ReceiveToIdle_IT(huartx, rs485_rxbuf, rs485_rxbuf_size);//485接收初始化
	RS485_EN_RESET;//失能EN 485进入接收模式
}

void RS485_test()
{
		//printf("rxbuf:%x\r\n",buf[0]);
		RS485_EN_SET;//
		manualBox485Func10Request(&manualBox485,&manualBoxHoldreg,pose.orig,speedPose.filter,force.filterTrans,strain1.filterTrans);
		RS485_EN_RESET;//
}

uint8_t manualBoxCoilUpdate(const uint8_t _coilNum,const bool _state)
{
	uint8_t err = 0;
	switch((_coilNum+1))
	{
		case 1:
			if(_state == true){
				mySemaphore.manualBox485.up = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFFD;//DI2
				//printf("manualBox485 Up");
			}else{
				mySemaphore.manualBox485.halt = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
				//printf("manualBox485 Up halt");
			}
			break;
		case 2:
			if(_state == true){
				mySemaphore.manualBox485.rapidUp = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFFB;//DI3
				//printf("manualBox485 rapidUp");
			}else{
				mySemaphore.manualBox485.halt = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
				//printf("manualBox485 rapidUp halt");
			}
			break;
		case 3:
			if(_state == true){
				mySemaphore.manualBox485.down = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFF7;//DI4
				//printf("manualBox485 Down");
			}else{
				mySemaphore.manualBox485.halt = 1;
				if(manualBox.type == manualBox485_Type)	
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
				//printf("manualBox485 Down halt");
			}
			break;
		case 4:
			if(_state == true){
				mySemaphore.manualBox485.rapidDown = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFEF;//DI5
				//printf("manualBox485 rapidDown");
			}else{
				mySemaphore.manualBox485.halt = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
				//printf("manualBox485 rapidDown halt");
			}
			break;
		case 5:
			if(_state == true){
				mySemaphore.manualBox485.testStart = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFFE;//DI1
				//printf("manualBox485 testStart");
			}else{
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
				
			}
			break;
		case 6:
			if(_state == true){
				mySemaphore.manualBox485.reset = 1;
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word &= 0xFFDF;//DI6
				//printf("manualBox485 reset");
			}else{
				if(manualBox.type == manualBox485_Type)
					manualBoxCoil.word |= 0x003F;//DI1-DI6 set 1
			}
			break;
		case 7:
			if(_state == true){
				mySemaphore.manualBox485.clampUpperFixture = 1;
				//printf("manualBox485 clampUpperFixture");
			}
			break;
		case 8:
			if(_state == true){
				mySemaphore.manualBox485.releaseUpperFixture = 1;
				//printf("manualBox485 releaseUpperFixture");
			}
			break;
		case 9:
			if(_state == true){
				mySemaphore.manualBox485.clampLowerFixture = 1;
				//printf("manualBox485 clampLowerFixture");
			}
			break;
		case 10:
			if(_state == true){
				mySemaphore.manualBox485.releaseLowerFixture = 1;
				//printf("manualBox485 releaseLowerFixture");
			}
			break;
		default:
			printf("unnamed coil num-address:0x%2X",_coilNum);
			err = 1;
			break;
	}
	return err;
}

/**
 * @brief send the callBack of the response of writing multi holdregs
 * @param[in out] _mb485
 * @param[out] _mbHoldreg
 * @param[in] _pose
 * @param[in] _poseSpeed
 * @param[in] _load
 * @param[in] _strain
 */
void manualBox485Func03ResponseCallBack(manualBox485_t *_mb485,manualBoxHoldreg_t *_mbHoldreg,
		const float _pose,const float _poseSpeed,const float _load,const float _strain){
	uint16_t regAdd,regLen,crc;//寄存器起始地址,寄存器单元长度，数据，crc校验码
	uint32_t poseInt32,poseSpeedInt32,loadInt32,strainInt32;
	uint8_t sizeByte,i=0,j=0;//字节数
	uint16_t* p=(uint16_t*)_mbHoldreg;//指向保持寄存器结构体首地址
	regAdd = MB485_regAdd;//读取寄存器起始地址
	regLen = MB485_regLen;//寄存器单元长度
	sizeByte = regLen*2;//字节数

	poseInt32 		= (uint32_t)((_pose * 1e6)*sign(_pose));
	if(sign(_pose) == -1){
		poseInt32 = poseInt32 | 0x80000000;
	}
	poseSpeedInt32 	= (uint32_t)((_poseSpeed * 1e6)*sign(_poseSpeed));
	if(sign(_poseSpeed) == -1){
		poseSpeedInt32 = poseSpeedInt32 | 0x80000000;
	}
	loadInt32 		= (uint32_t)((_load * 1e3)*sign(_load));
	if(sign(_load) == -1){
		loadInt32 = loadInt32 | 0x80000000;
	}
	strainInt32 	= (uint32_t)((_strain * 1e6)*sign(_strain));
	if(sign(_strain) == -1){
		strainInt32 = strainInt32 | 0x80000000;
	}

	_mbHoldreg->poseH16 		= (uint16_t)((poseInt32 >> 16)&0xFFFF);
	_mbHoldreg->poseL16 		= (uint16_t)((poseInt32)&0xFFFF);
	_mbHoldreg->poseSpeedH16 	= (uint16_t)((poseSpeedInt32 >> 16)&0xFFFF);
	_mbHoldreg->poseSpeedL16 	= (uint16_t)((poseSpeedInt32)&0xFFFF);
	_mbHoldreg->loadH16 		= (uint16_t)((loadInt32 >> 16)&0xFFFF);
	_mbHoldreg->loadL16 		= (uint16_t)((loadInt32)&0xFFFF);
	_mbHoldreg->strainH16 		= (uint16_t)((strainInt32 >> 16)&0xFFFF);
	_mbHoldreg->strainL16 		= (uint16_t)((strainInt32)&0xFFFF);

	//发送请求数据包
	_mb485->sendbuf[i++]=_mb485->manualBoxAddr;
	_mb485->sendbuf[i++]=0x10;//function code
	_mb485->sendbuf[i++]=(uint8_t)(regAdd>>8);
	_mb485->sendbuf[i++]=(uint8_t)regAdd;//寄存器起始地址低
	_mb485->sendbuf[i++]=(uint8_t)(regLen>>8);//寄存器单元长度高
	_mb485->sendbuf[i++]=(uint8_t)regLen;//寄存器单元长度低
	_mb485->sendbuf[i++]=sizeByte;//字节数
	for(j=0;j<regLen;j++){
		_mb485->sendbuf[i++]=(uint8_t)(*(p+j+1)>>8)&0xFF;
		_mb485->sendbuf[i++]=(uint8_t)(*(p+j+1))&0xFF;
	}

	crc = CRC16(_mb485->sendbuf,i);    //计算要返回数据的CRC
	_mb485->sendbuf[i++]=(uint8_t)(crc>>8);//CRC16高
	_mb485->sendbuf[i++]=(uint8_t)crc;//CRC16低
	//发送数据包 
	//HAL_UART_Transmit_DMA(&huart2,_mb485->sendbuf,i);
	HAL_UART_Transmit(&huart2, _mb485->sendbuf, i,2);//
}

/**
 * @brief send the request of writing multi holdregs
 * @param[in out] _mb485
 * @param[out] _mbHoldreg
 * @param[in] _pose
 * @param[in] _poseSpeed
 * @param[in] _load
 * @param[in] _strain
 */
void manualBox485Func10Request(manualBox485_t *_mb485,manualBoxHoldreg_t *_mbHoldreg,
		const float _pose,const float _poseSpeed,const float _load,const float _strain){
	uint16_t regAdd,regLen,crc;//寄存器起始地址,寄存器单元长度，数据，crc校验码
	uint32_t poseInt32,poseSpeedInt32,loadInt32,strainInt32;
	uint8_t sizeByte,i=0,j=0;//字节数
	uint16_t* p=(uint16_t*)_mbHoldreg;//指向保持寄存器结构体首地址
	regAdd = MB485_regAdd;//读取寄存器起始地址
	regLen = MB485_regLen;//寄存器单元长度
	sizeByte = regLen*2;//字节数

	poseInt32 		= (uint32_t)((_pose * 1e6)*sign(_pose));
	if(sign(_pose) == -1){
		poseInt32 = poseInt32 | 0x80000000;
	}
	poseSpeedInt32 	= (uint32_t)((_poseSpeed * 1e6)*sign(_poseSpeed));
	if(sign(_poseSpeed) == -1){
		poseSpeedInt32 = poseSpeedInt32 | 0x80000000;
	}
	loadInt32 		= (uint32_t)((_load * 1e3)*sign(_load));
	if(sign(_load) == -1){
		loadInt32 = loadInt32 | 0x80000000;
	}
	strainInt32 	= (uint32_t)((_strain * 1e6)*sign(_strain));
	if(sign(_strain) == -1){
		strainInt32 = strainInt32 | 0x80000000;
	}

	_mbHoldreg->poseH16 		= (uint16_t)((poseInt32 >> 16)&0xFFFF);
	_mbHoldreg->poseL16 		= (uint16_t)((poseInt32)&0xFFFF);
	_mbHoldreg->poseSpeedH16 	= (uint16_t)((poseSpeedInt32 >> 16)&0xFFFF);
	_mbHoldreg->poseSpeedL16 	= (uint16_t)((poseSpeedInt32)&0xFFFF);
	_mbHoldreg->loadH16 		= (uint16_t)((loadInt32 >> 16)&0xFFFF);
	_mbHoldreg->loadL16 		= (uint16_t)((loadInt32)&0xFFFF);
	_mbHoldreg->strainH16 		= (uint16_t)((strainInt32 >> 16)&0xFFFF);
	_mbHoldreg->strainL16 		= (uint16_t)((strainInt32)&0xFFFF);

	//发送请求数据包
	_mb485->sendbuf[i++]=_mb485->manualBoxAddr;
	_mb485->sendbuf[i++]=0x10;//function code
	_mb485->sendbuf[i++]=(uint8_t)(regAdd>>8);
	_mb485->sendbuf[i++]=(uint8_t)regAdd;//寄存器起始地址低
	_mb485->sendbuf[i++]=(uint8_t)(regLen>>8);//寄存器单元长度高
	_mb485->sendbuf[i++]=(uint8_t)regLen;//寄存器单元长度低
	_mb485->sendbuf[i++]=sizeByte;//字节数
	for(j=0;j<regLen;j++){
		_mb485->sendbuf[i++]=(uint8_t)(*(p+j+1)>>8)&0xFF;
		_mb485->sendbuf[i++]=(uint8_t)(*(p+j+1))&0xFF;
	}

	crc = CRC16(_mb485->sendbuf,i);    //计算要返回数据的CRC
	_mb485->sendbuf[i++]=(uint8_t)(crc>>8);//CRC16高
	_mb485->sendbuf[i++]=(uint8_t)crc;//CRC16低
	//发送数据包 
	//HAL_UART_Transmit_DMA(&huart2,_mb485->sendbuf,i);
	HAL_UART_Transmit(&huart2, _mb485->sendbuf, i,2);//
}

/**
 * @brief receive the response of writing multi holdregs
 * @param[in out] _mb485
 */
manualBox485_e manualBox485Func10ResponseCallBack(manualBox485_t *_mb485){
	uint16_t regAdd,regLen,sizeByte,crc;//寄存器起始地址,寄存器单元长度，字节数，数据，crc校验码
	uint8_t i=0,j=0;
	regAdd = (_mb485->rcbuf[2]<<8)+_mb485->rcbuf[3];//读取寄存器起始地址
	regLen = (_mb485->rcbuf[4]<<8)+_mb485->rcbuf[5];//寄存器单元长度
	if(regAdd != MB485_regAdd || regLen != MB485_regLen){
		printf("manual box func10 request err!");
		return MB485_errFunc10;
	} 
	return MB485_noErr;
}


/**
 * @brief receive the request of writing single coil
 * @param[in out] _mb485
 * @param[out] _mbCoil
 */
manualBox485_e manualBox485Func05Response(manualBox485_t *_mb485,manualBoxCoil_t *_mbCoil){
	uint16_t regAddr,coilState,crc;//寄存器起始地址，线圈状态,crc校验码
	uint8_t i=0;
	bool state;
	regAddr = (_mb485->rcbuf[2]<<8)+_mb485->rcbuf[3];//读取寄存器起始地址,即线圈地址
	coilState = (_mb485->rcbuf[4]<<8)+_mb485->rcbuf[5];//读取线圈状态
	state = (coilState == 0xFF00 ? true:false);//设置对应引脚状态
	//printf("regAddr:%d,state:%d\r\n",regAddr,state);
	manualBoxCoilUpdate((uint8_t)regAddr,state);
	//发送回应数据包
	_mb485->sendbuf[i++]=_mb485->mainBoardAddr;//ID号：发送本机设备地址
	_mb485->sendbuf[i++]=0x05;//功能码
	_mb485->sendbuf[i++]=(uint8_t)(regAddr>>8);//线圈地址高
	_mb485->sendbuf[i++]=(uint8_t)regAddr;//线圈地址低
	_mb485->sendbuf[i++]=(uint8_t)(coilState>>8);//线圈状态高
	_mb485->sendbuf[i++]=(uint8_t)coilState;//线圈状态低
	crc = CRC16(_mb485->sendbuf,i);    //计算要返回数据的CRC
	_mb485->sendbuf[i++]=(uint8_t)(crc>>8);//CRC16高
	_mb485->sendbuf[i++]=(uint8_t)crc;//CRC16低

	//发送数据包 
	HAL_UART_Transmit(&huart2,_mb485->sendbuf,i,1);
}


 
