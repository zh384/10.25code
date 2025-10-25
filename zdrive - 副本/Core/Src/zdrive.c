#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "mathfun.h"
#include "main.h"
#include "can.h"
#include "string.h"
#include "zdrive.h"

CAN_QUEUE CAN1_Queue,CAN2_Queue;
CAN_QUEUE VESC_Queue;
CAN_QUEUE ZDrive_Queue;
QueueEndian endian;
CAN_QUEUE can_queue;
uint8_t can_x;
ZDRIVE_MOTOR ZDrive[10];

CAN_RxHeaderTypeDef Z_RxHeader;
CAN_RxHeaderTypeDef M_RxHeader;
uint8_t rxmsg[8]={0};
uint8_t RxData[8]={0};

void Change_ArrayByte(uint8_t* data, uint8_t dlc) {
    if (dlc == 0 || dlc == 1) {
        return; 
    }
    uint8_t temp;
    uint8_t start = 0;
    uint8_t end = dlc - 1;
    while (start < end) {
        temp = data[start];
        data[start] = data[end];
        data[end] = temp;
        start++;
        end--;
    }
}

void CAN_Queue_Init(CAN_QUEUE* can_queue, QueueEndian endian, uint8_t can_x) {
    can_queue->size = CAN_QUEUE_MAX_SIZE;
    can_queue->capacity = 0;
    can_queue->head = can_queue->tail = 0;
    can_queue->can_x = can_x;
    can_queue->storage_endian = endian;
}

bool CAN_Queue_Wheather_Empty(CAN_QUEUE* can_queue) {
    return (can_queue->capacity == 0);
}

bool CAN_Queue_Wheather_Full(CAN_QUEUE* can_queue) {
    return (can_queue->size == can_queue->capacity);
}

bool CAN_Queue_Push(CAN_QUEUE* queue, CAN_FRAME frame) {
    if (queue->capacity <= queue->size)
    {
       if (queue->storage_endian == BIG_ENDIAN)
       {
           Change_ArrayByte(frame.Data, frame.DLC);
       }
        queue->buffer[queue->tail] = frame;
        queue->tail = (queue->tail + 1) % queue->size;
        queue->capacity++;
        return true;
    }
    else return false;
}

bool CAN_Queue_Pop(CAN_QUEUE* queue)
{
    if (queue->capacity > 0)
    {
        CAN_FRAME temp_frame = queue->buffer[queue->head];
        queue->head = (queue->head + 1) % queue->size;
        queue->capacity--;
        CAN_TxHeaderTypeDef TxHeader;
        uint32_t* pTxMailbox = 0;
        TxHeader.DLC = temp_frame.DLC;
        if (temp_frame.IDE == CAN_ID_STD) {
            TxHeader.StdId = temp_frame.ID;
        }
        if (temp_frame.IDE == CAN_ID_EXT) {
            TxHeader.ExtId = temp_frame.ID;
        }
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = temp_frame.IDE;
        if (queue->can_x == 1) {
            HAL_CAN_AddTxMessage(&hcan1, &TxHeader, temp_frame.Data, pTxMailbox);
        }
        if (queue->can_x == 2) {
            HAL_CAN_AddTxMessage(&hcan2, &TxHeader, temp_frame.Data, pTxMailbox);
        }
        return true;
    }
    else return false;
}

void CanCommand(CAN_FRAME* frame, uint32_t IDE, uint32_t Id, uint8_t* Data, uint8_t DLC) {
    frame->IDE = IDE;
    frame->ID = Id;
    frame->DLC = DLC;
    memcpy(frame->Data, Data, DLC);
}

void ZDrive_ReceiveHandler(CAN_RxHeaderTypeDef* Z_RxHeader, uint8_t RxData[]) {
    uint8_t id = Z_RxHeader->StdId;
    uint8_t ctrl_id = RxData[0];
    if (id > 10) return;
    ZDrive[id - 1].Status.error = Zdrive_Well;
    switch (ctrl_id) {
			case 0x5E:
        {
            memcpy(&(ZDrive[id - 1].ValueRealNow.angle), &RxData[1], sizeof(float));
            ZDrive[id - 1].ValueRealNow.angle *= (360.f / ZDrive[id - 1].Param.Ratio);
            break;
        }
			case 0x52:
        {
            memcpy(&(ZDrive[id - 1].ValueRealNow.current), &RxData[1], sizeof(float));
            break;
        }
			case 0x5C:
        {
            memcpy(&(ZDrive[id - 1].ValueRealNow.speed), &RxData[1], sizeof(float));
            ZDrive[id - 1].ValueRealNow.speed *= 60;
            break;
        }
			case 0x3C:
        {
            float temp_mode;
            memcpy(&temp_mode, &RxData[1], sizeof(float));
            ZDrive[id - 1].ModeRead = (ZDRIVE_MODE)temp_mode;
            break;
        }
			case 0x46:
        {
            float temp_pin;
            memcpy(&temp_pin, &RxData[1], sizeof(float));
            ZDrive[id - 1].ValueRealNow.pin = (float)temp_pin * 360 / 5.6f;
            break;
        }
			case 0x40:
        {
            float temp_error;
            memcpy(&temp_error, &RxData[1], sizeof(float));
            ZDrive[id - 1].ValueRealNow.pin = (ZDRIVE_ERROR)temp_error;
            break;
        }
				default:
            break;
    }
}

void ZDrive_Ask(uint8_t type) {
    CAN_FRAME ZDrive_TxMsg;
    uint8_t order[1] = { type - 1 };
    CanCommand(&ZDrive_TxMsg, CAN_ID_STD, 0, order, 1);
    CAN_Queue_Push(&ZDrive_Queue, ZDrive_TxMsg);
}

void ZDrive_Set(uint8_t type,uint8_t id, float arg1, float arg2){
    CAN_FRAME ZDrive_TxMsg;
    if (type == Zdrive_PVT_Mode) {
        float speed = arg1 / 360.f;
        float angle = arg2 / 60.f;

        uint8_t order[8];
        memcpy(order, &speed, 4);
        memcpy(order + 4, &angle, 4);
      
        CanCommand(&ZDrive_TxMsg, CAN_ID_STD, id, order, 8);
        CAN_Queue_Push(&ZDrive_Queue, ZDrive_TxMsg);
    }
    else {
        float temp = arg1;

        switch (type) {
					case
                SpeedSet:temp = arg1 / 60.f * ZDrive[id - 1].Param.Ratio;
                break;

					case
                PositionSet:temp = arg1 / 360.f * ZDrive[id - 1].Param.Ratio;
                break;
          case
                PositionReal:temp = arg1 / 360.f * ZDrive[id - 1].Param.Ratio;
                break;
          default:
							  break;
        }

        uint8_t order[5];
        order[0] = type;

        memcpy(&order[1], &temp, 4);

        CanCommand(&ZDrive_TxMsg, CAN_ID_STD, id, order, 5);
        CAN_Queue_Push(&ZDrive_Queue, ZDrive_TxMsg);
    }
}//ok

//void clearCommand(Queue*queue){
//	queue->front=0;
//	queue->rear=0;
//	queue->count=0;
//}




void ZDrive_Init() {
    for (int i = 0; i < USE_ZDRIVE_NUM; i++) {
        ZDrive[i].Param.reductionRatio = 1.f;
        ZDrive[i].Param.gearRatio = 1.f;

//#if Ratio_Config==1
//        ZDrive[i].Param.reductionRatio = ZDRIVE_RATIO_VALUES[i][0];
//        ZDrive[i].Param.gearRatio = ZDRIVE_RATIO_VALUES[i][1];
//#endif 
        ZDrive[i].Param.Ratio = ZDrive[i].Param.reductionRatio * ZDrive[i].Param.gearRatio;

        ZDrive[i].enable = false;
        ZDrive[i].begin = false;

        ZDrive[i].Mode = Zdrive_Disable;

        ZDrive[i].ValueSetNow.speed = 0.f;
        ZDrive[i].ValueSetNow.angle = 0.f;
        ZDrive[i].ValueSetNow.current = 0.f;

        ZDrive[i].Argum.lockAngle = 0;

        ZDrive[i].PVT_Mode.firstFlag = false;
        ZDrive[i].PVT_Mode.PVTModeFlag = false;

        ZDrive[i].Status.error = Zdrive_Well;

        HAL_Delay(100);
        ZDrive_Set(PositionReal, i + 1, 0.f, 0.f);
        ZDrive_Ask(Mode);
    }
}

void ZDrive_Func() {
    for (int i = 0; i < USE_ZDRIVE_NUM; i++) {
        if (ZDrive[i].enable) {
            if (ZDrive[i].begin) {
                ZDrive[i].Argum.lockAngle = ZDrive[i].ValueRealNow.angle;
                switch (ZDrive[i].Mode) {
									case Zdrive_Disable:
											{
                        if (ZDrive[i].ModeRead != Zdrive_Disable) {
                            ZDrive_Set(Mode, i + 1, (float)Zdrive_Disable, 0);
													  ZDrive_Ask(Mode);
                        }
                        break;
                    }
                    case Zdrive_Current: 
											{
                        if (ZDrive[i].ModeRead != Zdrive_Current) {
                            ZDrive_Set(Mode, i + 1, (float)Zdrive_Current, 0);
													  ZDrive_Ask(Mode);
                        }
                        else {
                            ZDrive_Set(CurrentSet, i + 1, ZDrive[i].ValueSetNow.current, 0);
                        }
                        break;
                    }
                    case Zdrive_Speed: {
                        if (ZDrive[i].ModeRead != Zdrive_Speed) {
                            ZDrive_Set(Mode, i + 1, (float)Zdrive_Speed, 0);
														ZDrive_Ask(Mode);
                        }
                        else {
                            ZDrive_Set(SpeedSet, i + 1, ZDrive[i].ValueSetNow.speed, 0);
                        }
                        break;
                    }
                    case Zdrive_Position: {
                        if (ZDrive[i].ModeRead != Zdrive_Position) {
                            ZDrive_Set(Mode, i + 1, (float)Zdrive_Position, 0);
                            ZDrive_Ask(Mode);
                        }
                        else {
                            if (!ZDrive[i].PVT_Mode.PVTModeFlag) {
                                if (ABS(ZDrive[i].ValueSetNow.angle - ZDrive[i].ValueSetLast.angle) > 0.5f) {
                                    ZDrive[i].ValueSetLast.angle = ZDrive[i].ValueSetNow.angle;
                                    ZDrive_Set(PositionSet, i + 1, ZDrive[i].ValueSetNow.angle, 0);
                                }
                            }
                        }
                        break;
                    }
                    default: 
                        break;
                }
            }
//          else {
//            if (ZDrive[i].ModeRead != Zdrive_Position) {
//                ZDrive_Set(Mode, i + 1, (float)Zdrive_Position, 0);
//            }
//            else {
//                if (ABS(ZDrive[i].ValueSetNow.angle - ZDrive[i].ValueSetLast.angle) > 0.5f) {
//                    ZDrive[i].ValueSetLast.angle = ZDrive[i].ValueSetNow.angle;
//                    ZDrive_Set(PositionSet, i + 1, ZDrive[i].ValueSetNow.angle, 0);
//                }
//            }

//        }
				}
        else {
            if (ZDrive[i].ModeRead != Zdrive_Disable) {
                ZDrive_Set(Mode, i + 1, Zdrive_Disable, 0);
                ZDrive_Ask(Mode);
            }
        }
    }
    ZDrive_Ask(PositionReal);
    ZDrive_Ask(SpeedReal);
    ZDrive_Ask(CurrentReal);
		ZDrive_Ask(Mode);
}



