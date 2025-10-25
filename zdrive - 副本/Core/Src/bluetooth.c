#include "bluetooth.h"
#include "string.h"


/*----------------------------------variable-----------------------------------*/

PACK_MSG PackMsg = {0};
RX_MSGPACK RxMsgPack = {0};
TX_MSGPACK TxMsgPack = {0};

/*----------------------------------function----------------------------------*/

/**
 * @brief 处理接收的数据包，将数据包中的数据存入RxMsgPack中
 * 
 * @param PackMsg 
 * @param RxMsgPack 
 * @return true 
 * @return false 
 */
 bool Debug_ReceiveMsg(PACK_MSG *PackMsg, RX_MSGPACK *RxMsgPack)
{
    uint8_t rxindex = 0;
    #if RX_BOOL_NUM
    uint8_t boolbit = 0;
    for (int i = 0; i < RX_BOOL_NUM; i++)
        RxMsgPack->bools[i] = (PackMsg->RxData[i/8] & (0x01 << (boolbit ++ %8))) ? true : false;//判断bool第i位是否为1
    rxindex = (RX_BOOL_NUM + 7) >> 3;//bools所占字节数
    #endif    
    #if RX_BYTE_NUM
    for (u8 i = 0; i < RX_BYTE_NUM; i++) 
    {
        RxMsgPack->bytes[i] = PackMsg->RxData[rxindex++];
    }
    #endif 
    #if RX_SHORT_NUM

    for (uint8_t i = 0; i < RX_SHORT_NUM; i++) 
    {
        memcpy(&(RxMsgPack->shorts[i]),&(PackMsg->RxData[rxindex]),2);
        rxindex += 2;
    }
    #endif 
    #if RX_INT_NUM
    for (int i = 0; i < RX_INT_NUM; i++)
    {
        memcpy(&(RxMsgPack->ints[i]),&(PackMsg->RxData[rxindex]),4);
        rxindex += 4;
    }
    
    #endif 
    #if RX_FLOAT_NUM
    for (int i = 0; i < RX_FLOAT_NUM; i++)
    {
        memcpy(&(RxMsgPack->floats[i]),&(PackMsg->RxData[rxindex]),4);
        rxindex += 4;
    }

    #endif 

    if (rxindex != RX_PACK_SIZE)
    {  
        return false;
    }
    else return true;

}


 bool Deal_RXPack(PACK_MSG *PackMsg,uint8_t data)
 {
    if (PackMsg->Debug_status == Debug_rx)
    {
        PackMsg->Suffix = data;//将数据存入Suffix，后续判断是否为包尾

        if (PackMsg->Suffix == DEBUG_SUFFIX)//如果Suffix为包尾，则将Debug_status置为Debug_wait
        {
            uint8_t sum = 0;
            for(int i = 0; i < PackMsg->RX_Size - 1; i++)
            {
                sum += PackMsg->RxData[i];//计算校验和
            }
            PackMsg->Debug_status = Debug_wait;
            
            if(PackMsg->RxData[PackMsg->RX_Size - 1] == sum)//如果校验和与校验位数据相等（校验通过），则调用Debug_ReceiveMsg函数接收数据
            {
                if (Debug_ReceiveMsg(PackMsg,&RxMsgPack) != true)
                {
                    return false;
                }
                PackMsg->RX_Size = 0;
                
            }

        }
		else { PackMsg->RxData[PackMsg->RX_Size ++] = data; }//如果Suffix不为包尾，则将数据存入RxData，并将RX_Size加1
    }
    else
    {
        PackMsg->Prefix = data;
        if (PackMsg->Prefix == DEBUG_PREFIX)//判断是否接收到了包头
        {
            PackMsg->Debug_status = Debug_rx;
        }
    }
    if(PackMsg->RX_Size >= DATA_LENGTH) 
    {
        PackMsg->Debug_status = Debug_wait;
        PackMsg->RX_Size = 0;
        return false;
    }
    return true;

}

/**
 * @brief       向调试器发送信息前将要发送的信息打包处理好
 * 
 * @param       PackMsg:包括要发送的数据包
 * @param       TxMsgPack:要发送的信息
 */
 bool Deal_TXPack(PACK_MSG *PackMsg, TX_MSGPACK *TxMsgPack)
 {
    uint8_t txindex = 0;
    uint8_t sum = 0;

    PackMsg->TxData[txindex ++] = DEBUG_PREFIX;
#if TX_BYTE_NUM

    for (int i = 0; i < TX_BYTE_NUM; i++)
    {
        memcpy(&(PackMsg->TxData[txindex++]),&(TxMsgPack->bytes[i]),1);
        
    }

#endif

#if TX_SHORT_NUM
  for (int i = 0; i < TX_SHORT_NUM; i++) 
  {
    memcpy(&(PackMsg->TxData[txindex]),&(TxMsgPack->shorts[i]),2);
    txindex += 2;
  }
#endif
#if TX_INT_NUM
  for (int i = 0; i < TX_INT_NUM; i++) 
  {
    memcpy(&(PackMsg->TxData[txindex]),&(TxMsgPack->ints[i]),4);
    txindex += 4;
  }
#endif
#if TX_FLOAT_NUM
  for (uint8_t i = 0; i < TX_FLOAT_NUM; i++) 
  {
    memcpy(&(PackMsg->TxData[txindex]),&(TxMsgPack->floats[i]),4);
    txindex += 4;
  }
#endif

  for (int i = 1; i < txindex; i++)
  {
    sum += PackMsg->TxData[i];
  }

  PackMsg->TxData[txindex ++] = sum;
  PackMsg->TxData[txindex ++] = DEBUG_SUFFIX;

	if(ABS( txindex - TX_PACK_SIZE) != 3)
	{
		return false;
	}
	
  if (HAL_UART_Transmit_DMA(&huart2, PackMsg->TxData, txindex) != HAL_OK) {
    Error_Handler();
  }
	
  return true;

 }

