#pragma once

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "can.h"



#define USE_ZDRIVE_NUM 10
#define Ratio_Config 0
#define CAN_QUEUE_MAX_SIZE 64


typedef enum {
    Zdrive_Disable,
    Zdrive_Current,
    Zdrive_Speed,
    Zdrive_Position,
    Zdrive_Test,
    Zdrive_RVCalibration,
    Zdrive_EncoderLineCalibration,
    Zdrive_EncoudeOffsetCalibration,
    Zdrive_VKCalibration,
    Zdrive_SaveSetting,
    Zdrive_EraseSetting,
    Zdrive_ClearErr,
    Zdrive_Brake,
    Zdrive_PVT_Mode,
}ZDRIVE_MODE;

typedef enum {
    Acceleration = 0x29,
    Deceleration = 0x2B,
    Mode = 0x3D,
    Error = 0x41,
    CurrentSet = 0x43,
    SpeedSet = 0x45,
    PositionSet = 0x47,
    CurrentReal = 0x53,
    SpeedReal = 0x5D,
    PositionReal = 0x5F,
}ZdriveOrder;

typedef enum {
    Zdrive_Well,
    Zdrive_InsufficientVoltage,
    Zdrive_OverVoltage,
    Zdrive_InstabilityCurrent,
    Zdrive_OverCurrent,
    Zdrive_OverSpeed,
    Zdrive_ExcessiveR,
    Zdrive_ExcessiveInductance,
    Zdrive_LoseEncoderl,
    Zdrive_PolesErr,
    Zdrive_VKCalibrationErr,
    Zdrive_ModeErr,
    Zdrive_ParameterErr,
    Zdrive_Hot
}ZDRIVE_ERROR;

typedef enum {
    PVT_Angle,
    PVT_Speed,
}PVT_MODE;

typedef struct {
    float current;
    float speed;
    float angle;
    float pin;
}ZDRIVEVALUE;

typedef struct {
    float gearRatio;
    float reductionRatio;
    float Ratio;
}ZDRIVE_PARAM;

typedef struct {
    uint32_t lastRxTime;
    uint32_t timeoutTicks;
    uint32_t stuckCount;
    float lockAngle;
}ZDRIVE_ARGUM;

typedef struct {
    bool timeoutFlag;
    bool stuckFlag;

    ZDRIVE_ERROR error;
}ZDRIVE_STATUS;

typedef struct {
    bool PVTModeFlag;
    bool firstFlag;

    float controlTime;
    float targetTime;
    float targetSpeed;
    float targetangle;
    float angle0;

    PVT_MODE pvt_Mode;
}ZDRIVE_PVT; 

typedef struct {
    bool enable;
    bool begin;
    bool brake;

    ZDRIVE_MODE Mode, ModeRead;
    ZDRIVE_PARAM Param;
    ZDRIVE_ARGUM Argum;
    ZDRIVEVALUE ValueSetLast, ValueSetNow, ValueRealNow;
    ZDRIVE_STATUS Status;
    ZDRIVE_PVT PVT_Mode;
}ZDRIVE_MOTOR;

typedef struct CAN_FRAME {
    uint32_t ID;
    uint32_t DLC;
    uint32_t IDE;
    uint8_t Data[8];
}CAN_FRAME;

typedef enum {
    LITTLE_ENDIAN, 
    BIG_ENDIAN     
} QueueEndian;

typedef struct CAN_QUEUE {
    uint8_t can_x;
    uint32_t head;
    uint32_t tail;
    uint32_t size;
    uint32_t capacity;
    QueueEndian storage_endian;
    CAN_FRAME buffer[CAN_QUEUE_MAX_SIZE];
}CAN_QUEUE;


extern ZDRIVE_MOTOR ZDrive[10];
extern CAN_QUEUE CAN2_Queue;
extern CAN_QUEUE VESC_Queue;
extern CAN_QUEUE ZDrive_Queue;
extern QueueEndian endian;

extern CAN_RxHeaderTypeDef Z_RxHeader;
extern CAN_RxHeaderTypeDef M_RxHeader;
extern uint8_t rxmsg[8];
extern uint8_t RxData[8];

void ZDrive_Func(void);
void ZDrive_Init(void);
void ZDrive_Set(uint8_t type,uint8_t id, float arg1, float arg2);
void ZDrive_Ask(uint8_t type);
void ZDrive_ReceiveHandler(CAN_RxHeaderTypeDef* Z_RxHeader, uint8_t RxData[]);
void CanCommand(CAN_FRAME* frame, uint32_t IDE, uint32_t Id, uint8_t* Data, uint8_t DLC);
bool CAN_Queue_Pop(CAN_QUEUE* queue);
bool CAN_Queue_Push(CAN_QUEUE* queue, CAN_FRAME frame);
bool CAN_Queue_Wheather_Full(CAN_QUEUE* can_queue);
void CAN_Queue_Init(CAN_QUEUE* can_queue, QueueEndian endian, uint8_t can_x);
void Change_ArrayByte(uint8_t* data, uint8_t dlc);

extern CAN_QUEUE can_queue;
extern uint8_t can_x;
