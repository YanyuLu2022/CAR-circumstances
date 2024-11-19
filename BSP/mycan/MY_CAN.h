#ifndef MY_CAN__H
#define MY_CAN__H

#include "stm32f1xx_hal.h"
#include "task.h"
#include <stm32f1xx_hal_can.h>
#include "can.h"
#define CanTxQueueLenght 10
#define CanRxQueueLenght 10
/*
MOD:切换模式
0: 静止模式
1：蓝牙模式
2：CAN模式
3：自动驾驶模式
directive:控制移动方向
0：不变
1：停
2：上
3：下
4：左
5：右
6：左转
7：右转
speed:加速或减速
0：不变速
1：加速
2：减速
*/
typedef struct
{
    uint8_t directive;
    uint8_t MOD;  
    uint8_t speed; 
} CANRx_Queue_Data;
/*
MOD:切换模式
0: 静止模式
1：蓝牙模式
2：CAN模式
3：自动驾驶模式
directive:控制移动方向
0：不变
1：停
2：上
3：下
4：左
5：右
6：左转
7：右转
8：加速
9：减速
*/
typedef struct
{
    uint8_t directive;//移动方向
    uint8_t MOD;//模式
    uint8_t speed; //速度
    uint32_t StdId; 
    uint32_t DLC;
    uint32_t ExtId;  
} CANTx_Queue_Data;


void Init_My_CAN(void);
void Can_Write_Thread_Start(void *PThread, StackType_t *PThread_Stack, int LMotor_Stack,CAN_TxHeaderTypeDef * can_TxConfig);
void CAN_DATA_Init(CAN_FilterTypeDef *can_Filter_default,CAN_TxHeaderTypeDef *can_Tx_default);
void Can_Read_Thread_Start(void *PThread, StackType_t *PThread_Stack, int LMotor_Stack,CAN_FilterTypeDef * can_Filterconfig);
QueueHandle_t ReCanRxQueueStruct(void);
QueueHandle_t ReCanTxQueueStruct(void);
#endif // !MY_CAN__H
