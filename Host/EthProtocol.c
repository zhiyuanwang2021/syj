/*
数据帧分为6个部分：起始标志位/帧头，功能码，有效数据长度，有效载荷数据，CRC校验码。
数据帧结构如下表所示：
											Name	帧头		功能码		数据长度		有效载荷数据Payload		校验码
											Size	2Byte		1Byte			2Byte					N Byte					2 Bytes
								备注：超过1字节长度的数据都以高位在前、低位在后。
											(1)起始标志位/帧头（Header）
											数据帧以“0xAA 0xAA”作为起始标志位；
											(2)功能码（FunCode）
											不同功能的数据，有效载荷数据的定义不同，通过功能码来区分。功能码见附件1。
											(3)数据长度（Length）
											定义这个数据包中有效载荷数据的字节数。
											(4)有效载荷数据（Payload）
											不同功能的数据，有效载荷数据的定义不同。详见附件2。
											(5)帧尾（Tail）
											帧尾固定位“0x55 0x55”作为结束标志位。
											
*/

#include "EthProtocol.h"
#include "usart.h"
#include "myFifo.h"
#include "elog.h"

//#define fifo_debug_rev

#define TAIL_CHECK_NUM1 0x55
#define TAIL_CHECK_NUM2 0x55

/*
		上位机给下位机发送数据处理
		即下位机收
		数据帧以“0xAA 0xAA”作为起始标志位
*/
#define REV_CHECK_NUM1 0xAA 
#define REV_CHECK_NUM2 0xAA
uint8_t buf_eth[DATA_BUF_SIZE];

uint8_t UTC_revbuf_process(UTC_ETH* EthStruct)
{
	uint16_t i=0,j=0,k=0,l=0;
	uint8_t buf_last=0,buf_now=0,i_last=0;
	uint8_t* p;
	uint16_t DataLen=0;//帧有效载荷数据长度
	uint8_t FuncNum=0;
	p = EthStruct->Revbuf;
	memset(buf_eth,0,sizeof(buf_eth));//清空buf
	//拆包
	//log_i("size %d",EthStruct->revsize);
	for(i=0;i<EthStruct->revsize;i++)
	{
			buf_now = p[i];
			i_last = i;
			if( buf_last == REV_CHECK_NUM1 && buf_now == REV_CHECK_NUM2 ) //找到帧头
			{
					j=0;
					buf_eth[j++]= REV_CHECK_NUM1;
					buf_eth[j++]= REV_CHECK_NUM2;
					buf_eth[j++] = p[++i];//功能码
					FuncNum = buf_eth[j-1];
					buf_eth[j++] = p[++i];//数据长度H
					buf_eth[j++] = p[++i];//数据长度L
				  DataLen = buf_eth[j-2]*256+buf_eth[j-1];
				  for(k=0;k<DataLen;k++)
					{
						buf_eth[j++] = p[++i];//有效载荷数据 Payload
					}
					// 读取数据帧尾
					buf_eth[j++] = p[++i];//帧尾H
					buf_eth[j++] = p[++i];//帧尾L						
					if(buf_eth[j-2] == TAIL_CHECK_NUM1 && buf_eth[j-1] == TAIL_CHECK_NUM2)//帧尾正确
					{
#ifdef fifo_debug_rev
						printf("DataLen:%d\r\n",DataLen);
						printf("FuncNum:0x%2x\r\n",FuncNum);
						printf("j:%d\r\n",j);
						printf("buf_eth:");
						for(k=0;k<j-2;k++)
						printf("%2x ",buf_eth[k]);
						printf("\r\n");
#endif
						buf_now	= buf_eth[j-1];	
						//处理此帧buf_eth  压入循环队列
						rev_ring_fifo_push(j-2,buf_eth);
#ifdef fifo_debug_rev1
							for(k=0;k<REV_FIFO_SIZE;k++)
									{
										printf("Fifo%d  ",k);
											for(l=0;l<50;l++)
											printf("%2x",rev_fifo_data[k][l]);
										printf("\r\n");
									}
									printf("\r\n \r\n");
						
#endif
					}
					else//帧尾不正确，继续寻找下一帧，从上一次找到0xAA 0xAA 再重新开始寻找
					{
						i = i_last;
						buf_now	= p[i];	
					}
			}
			buf_last = buf_now;
	}
		return 0;
}

/*
		下位机给上位机发送数据处理
		即下位机发
		数据帧以“0xAA 0xAA”作为起始标志位
*/
#define SEND_CHECK_NUM1 0xAA 
#define SEND_CHECK_NUM2 0xAA
uint8_t UTC_sendbuf_process(uint8_t* Data,UTC_ETH* EthStruct)
{
	uint16_t i=0,j=0,DataLen=0;
	uint8_t ret=0;
	uint8_t* p;
	p = EthStruct->Sendbuf;
	memset(buf_eth,0,sizeof(buf_eth));//清空buf
	//帧头
	buf_eth[j++] = SEND_CHECK_NUM1;
	buf_eth[j++] = SEND_CHECK_NUM2;
	//功能码
	buf_eth[j++]  = Data[0];
	//数据长度
	buf_eth[j++]  = Data[1];
	buf_eth[j++]  = Data[2];
	DataLen = (Data[1]<<8)+ Data[2];
	//有效载荷数据Payload
	for(i=0;i<DataLen;i++)
	buf_eth[j++]  = Data[i+3];
	
	if((EthStruct->sendsize+5+DataLen) <= DATA_BUF_SIZE)//超过缓冲区，则此帧数据丢失
	{
		if(p[EthStruct->sendsize] != 0x00)//防止数据覆盖
			EthStruct->sendsize ++;
		
		for(i=0;i<j;i++)
		p[EthStruct->sendsize++]=buf_eth[i];
		
		p[EthStruct->sendsize++] = TAIL_CHECK_NUM1;
		p[EthStruct->sendsize++] = TAIL_CHECK_NUM2;
		
		//记得清空Data相关空间 以防下次继续发送同样的值
		ret = 0;
	}
	else
	{
			ret = 1;//缓冲区满，则此帧数据丢失  根据ret判断是否需要下一周期重发
	}
	return ret;
}

utcSetOnPosMsg_t utcSetOnPosMsg={
	.Control = POS_MODE,
	.Position = 0,
	.DControl = POS_MODE,
	.Destination = 0,
	.Reached = 0,
	.Time = 0,
	.DoPError = 0,
	.usTAN = 0,
	.timeoutAllowed = 10.0f,
};
utcSetOnTPosMsg_t utcSetOnTPosMsg={
	.Control = POS_MODE,
	.Position = 0,
	.DControl = POS_MODE,
	.Destination = 0,
	.Reached = 0,
	.Time = 0,
	.DoPError = 0,
	.usTAN = 0,
};
utcSetOnLPosMsg_t utcSetOnLPosMsg={
	.Control = POS_MODE,
	.Position = 0,
	.DControl = POS_MODE,
	.Destination = 0,
	.Reached = 0,
	.Time = 0,
	.DoPError = 0,
	.usTAN = 0,
};
utcSetOnSftMsg_t utcSetOnSftMsg;
utcSetOnRuntimeError_t utcSetOnRuntimeError;
utcSetOnSystemMsg_t utcSetOnSystemMsg;
utcSetOnDebugMsg_t utcSetOnDebugMsg;

/**
 * @brief Update the system message content and set the corresponding semaphore.
 * @param _msgContent A pointer to the message content to be copied.
 * @param _utcSetOnSystemMsg A pointer to the system message structure.
 */
void utcSetOnSystemMsgUpdate(char* _msgContent,utcSetOnSystemMsg_t* _utcSetOnSystemMsg){
	if (_msgContent == NULL || _utcSetOnSystemMsg == NULL) {
        return; 
    }
	strncpy(_utcSetOnSystemMsg->Text, _msgContent, SYSTEM_MSG_TEXT_LEN);
	mySemaphore.utcSetOn.SetOnSystem = 1;
}

/**
 * @brief Update the debug message content and set the corresponding semaphore.
 * @param _msgContent A pointer to the message content to be copied.
 * @param _utcSetOnDebugMsg A pointer to the debug message structure.
 */
void utcSetOnDebugMsgUpdate(char* _msgContent,utcSetOnDebugMsg_t* _utcSetOnDebugMsg){
	if (_msgContent == NULL || _utcSetOnDebugMsg == NULL) {
        return; 
    }
	strncpy(_utcSetOnDebugMsg->Text, _msgContent, DEBUG_MSG_TEXT_LEN);
	mySemaphore.utcSetOn.SetOnDebug = 1;
}








