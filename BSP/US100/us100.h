#ifndef __US100_H_
#define __US100_H_
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"  
#include "queue.h"
#include "main.h"



#define Us100QueueLenght 10
typedef struct
{

	float distant;


}U100_Struct;

void US100_Init( void );
uint8_t US100_Thread_Start(TaskHandle_t PThread,StackType_t * P_Stack,int L_Stack);
void US100_Thread_Delete(TaskHandle_t xTaskToDelete);
QueueHandle_t RetQueueUS100(void);
#endif // __US100_H_

