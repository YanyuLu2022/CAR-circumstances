#include "FreeRTOS.h"	  // ARM.FreeRTOS::RTOS:Core
#include "task.h"		  // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h" // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"		  // ARM.FreeRTOS::RTOS:Core
#include "cmsis_os.h"
#include "queue.h"
#include "can.h"
#include <stm32f1xx_hal_can.h>
#include "MY_CAN.h"
#include <string.h>
typedef struct
{
	uint32_t Rx_ID;
	uint8_t Rx_Lenght;
	uint8_t Rx_DATA[8];
} CAN_RX_DATA;
#define RXBUFFERSIZE 256 //
// 初始化
CAN_FilterTypeDef can_Filter = {0}; // 过滤器初始化结构体
CAN_HandleTypeDef *hcan1 = &hcan;
// 发送
CAN_TxHeaderTypeDef can_Tx; // 发送结构体
// 接收
static CAN_RX_DATA can_rx_data; //
uint8_t recvBuf[8];				// 接收数组
CAN_RxHeaderTypeDef can_Rx;		// 接收结构体
// 信号量句柄
static SemaphoreHandle_t can_rx_Semaphore;
// 队列句柄
static QueueHandle_t Can_RxQueue;
static QueueHandle_t Can_TxQueue;

void CAN_DATA_Init(CAN_FilterTypeDef *can_Filter_default,CAN_TxHeaderTypeDef *can_Tx_default)
{
	if(NULL != can_Filter_default)
	{
	can_Filter_default->FilterIdHigh = 0x0;
	can_Filter_default->FilterIdLow = 0;
	can_Filter_default->FilterMaskIdHigh = 0;
	can_Filter_default->FilterMaskIdLow = 0;
	can_Filter_default->FilterBank = 0;							// 选择过滤器
	can_Filter_default->FilterFIFOAssignment = CAN_FILTER_FIFO0; // FIFO
	can_Filter_default->FilterMode = CAN_FILTERMODE_IDLIST;		// 过滤器模式
	can_Filter_default->FilterScale = CAN_FILTERSCALE_32BIT;		// 16或32位模式
	can_Filter_default->FilterActivation = CAN_FILTER_ENABLE;	// 启动该过滤器
	}
	if(NULL != can_Tx_default)
	{
	can_Tx_default->StdId = 0x0;					 // STID
	can_Tx_default->ExtId = 0;					 // EXID
	can_Tx_default->IDE = 0;						 // 标准帿
	can_Tx_default->RTR = 0;						 // 数据帿
	can_Tx_default->DLC = 1;						 // 数据大小最大为8
	can_Tx_default->TransmitGlobalTime = DISABLE; // 不使用时间触发模式用
	}
}
/*
初始化，并且赋默认值
启动 can模块
创建队列
*/
void Init_My_CAN(void)
{
	HAL_CAN_Start(hcan1); // 启动 can模块
	Can_RxQueue = xQueueCreate(CanTxQueueLenght, sizeof(CANRx_Queue_Data));
	Can_TxQueue = xQueueCreate(CanRxQueueLenght, sizeof(CANTx_Queue_Data));
}
/*
	返回队列句柄
*/
QueueHandle_t ReCanRxQueueStruct(void)
{
	return Can_RxQueue;
}
QueueHandle_t ReCanTxQueueStruct(void)
{
	return Can_TxQueue;
}
/*
	Can发送任务
*/
void Can_Write_Thread(void *argument)
{
	static uint8_t sendBuf[8];			   // 发送字符
	CAN_TxHeaderTypeDef *CANTx = argument; // 读取初始值
	static CANTx_Queue_Data TxData;		   // 发送队列
	uint32_t box;
	can_Tx.StdId = CANTx->StdId;						   // STID
	can_Tx.ExtId = CANTx->ExtId;						   // EXID
	can_Tx.IDE = CANTx->IDE;							   // 标准模式
	can_Tx.RTR = CANTx->RTR;							   // 数据帿
	can_Tx.DLC = CANTx->DLC;							   // 长度
	can_Tx.TransmitGlobalTime = CANTx->TransmitGlobalTime; // 不使用时间触发模式用

	while (1)
	{
		xQueueReceive(Can_TxQueue, &TxData, portMAX_DELAY);
		// box邮箱号
		sendBuf[0] = TxData.directive;
		sendBuf[1] = TxData.MOD;
		sendBuf[2] = TxData.speed;
		if(NULL != TxData.StdId)can_Tx.StdId  = TxData.StdId;
		if(NULL != TxData.ExtId)can_Tx.ExtId  = TxData.ExtId;
		if(NULL != TxData.DLC)can_Tx.DLC  = TxData.DLC;		
		for (int i = 0; i < 10000; i++)
		{

			if (HAL_OK == HAL_CAN_AddTxMessage(hcan1, &can_Tx, sendBuf, &box))
				break;
		}
		memset(sendBuf, 0, sizeof(sendBuf));
	}
}
/*
	Can接收任务
*/
void Can_Read_Thread(void *argument)
{

	CAN_FilterTypeDef *CANFilter = argument;
	can_Filter.FilterIdHigh = CANFilter->FilterIdHigh;
	can_Filter.FilterIdLow = CANFilter->FilterIdLow;
	can_Filter.FilterMaskIdHigh = CANFilter->FilterMaskIdHigh;
	can_Filter.FilterMaskIdLow = CANFilter->FilterMaskIdLow;
	can_Filter.FilterFIFOAssignment = CANFilter->FilterFIFOAssignment;
	can_Filter.FilterBank = CANFilter->FilterBank;
	can_Filter.FilterMode = CANFilter->FilterMode;
	can_Filter.FilterScale = CANFilter->FilterScale;
	can_Filter.FilterActivation = CANFilter->FilterActivation;
	// 创建二值信号量
	can_rx_Semaphore = xSemaphoreCreateBinary();
	xSemaphoreTake(can_rx_Semaphore, 0);
	// 初始化过滤器
	HAL_CAN_ConfigFilter(hcan1, &can_Filter);
	// 开启接收中断
	HAL_CAN_ActivateNotification(hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	static CANRx_Queue_Data queue_Date;
	while (1)
	{

		xSemaphoreTake(can_rx_Semaphore, portMAX_DELAY);
		queue_Date.directive = can_rx_data.Rx_DATA[0];
		queue_Date.MOD = can_rx_data.Rx_DATA[1];	
		queue_Date.speed = can_rx_data.Rx_DATA[2];	
		xQueueSend(Can_RxQueue, &queue_Date, 0); /* code */	
		memset(&queue_Date, 0, sizeof(queue_Date));

	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &can_Rx, recvBuf);
	if (can_Rx.IDE == CAN_ID_STD)
	{
		// UsartPrintf(&huart1,"标准ID:%#X", can_Rx.StdId);
		can_rx_data.Rx_ID = can_Rx.StdId;
	}
	else if (can_Rx.IDE == CAN_ID_EXT)
	{
		// UsartPrintf(&huart1,"扩展ID:%#X", can_Rx.ExtId);
		can_rx_data.Rx_ID = can_Rx.ExtId;
	}

	if (can_Rx.RTR == CAN_RTR_DATA)
	{
		can_rx_data.Rx_Lenght = can_Rx.DLC;
		for (int i = 0; i < can_rx_data.Rx_Lenght; i++)
		{
			can_rx_data.Rx_DATA[i] = recvBuf[i];
		}
		// UsartPrintf(&huart1,"数据为%s \r\n", recvBuf);
		xSemaphoreGiveFromISR(can_rx_Semaphore, NULL);
	}
	else if (can_Rx.RTR == CAN_RTR_REMOTE)
	{
		// UsartPrintf(&huart1,"遥控帧\r\n");
	}
}
/*
创建发送任务
*/
void Can_Write_Thread_Start(void *PThread, StackType_t *PThread_Stack, int LMotor_Stack,CAN_TxHeaderTypeDef * can_TxConfig)
{
	xTaskCreateStatic(Can_Write_Thread,
					  "Whrit_CAN",
					  LMotor_Stack,
					  can_TxConfig,
					  (osPriority_t)osPriorityNormal,
					  PThread_Stack,
					  PThread);
}
/*
创建接收任务
*/
void Can_Read_Thread_Start(void *PThread, StackType_t *PThread_Stack, int LMotor_Stack,CAN_FilterTypeDef * can_Filterconfig)
{
	xTaskCreateStatic(Can_Read_Thread,
					  "Read_CAN",
					  LMotor_Stack,
					  can_Filterconfig,
					  (osPriority_t)osPriorityNormal,
					  PThread_Stack,
					  PThread);
}
