#include "cmsis_os.h"
#include "FreeRTOS.h"	  // ARM.FreeRTOS::RTOS:Core
#include "task.h"		  // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h" // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"		  // ARM.FreeRTOS::RTOS:Core
#include "us100.h"
#include "tim.h"


#define TRIG_H HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET)
#define TRIG_L HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET)
#define Us100_TIME_nvic &htim2

float distant;				   // 测量距离
uint32_t measure_Buf[3] = {0}; // 存放定时器计数值的数组
uint8_t measure_Cnt = 0;	   // 状态标志位
uint32_t high_time;			   // 超声波模块返回的高电平时间
static U100_Struct U100_DATA;

// 队列句柄
static QueueHandle_t US100_Queue;


#define  DEM_CR      *(volatile uint32_t *)0xE000EDFC
#define  DWT_CR      *(volatile uint32_t *)0xE0001000
#define  DWT_CYCCNT  *(volatile uint32_t *)0xE0001004
#define  DEM_CR_TRCENA                   (1 << 24)
#define  DWT_CR_CYCCNTENA                (1 <<  0)

void DWT_Init()
{
    DEM_CR  |=  DEM_CR_TRCENA; /*对DEMCR寄存器的位24控制，写1使能DWT外设。*/
    DWT_CYCCNT = 0;/*对于DWT的CYCCNT计数寄存器清0。*/
    DWT_CR  |=  DWT_CR_CYCCNTENA;/*对DWT控制寄存器的位0控制，写1使能CYCCNT寄存器。*/
}

void DWT_DelayUS(uint32_t _ulDelayTime)
{
    uint32_t tCnt, tDelayCnt;
    uint32_t tStart;
           
    tStart = DWT_CYCCNT; /* 刚进入时的计数器值 */
    tCnt = 0;
    tDelayCnt = _ulDelayTime * (SystemCoreClock / 1000000);
    /* 需要的节拍数 */    /*SystemCoreClock :系统时钟频率*/                 

    while(tCnt < tDelayCnt)
      {
        tCnt = DWT_CYCCNT - tStart; 
        /* 求减过程中，如果发生第一次32位计数器重新计数，依然可以正确计算 */       
      }
}

void DWT_DelayMS(uint32_t _ulDelayTime)
{
        DWT_DelayUS(1000*_ulDelayTime);
}

void delay_us(uint32_t us)//主频72M
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
	{
		;
	}
}



QueueHandle_t RetQueueUS100(void)
{
	return US100_Queue;
}

void US100_Init(void)
{
	// 创建队列
	US100_Queue = xQueueCreate(Us100QueueLenght, sizeof(U100_Struct));
	DWT_Init();
}

void US100_Thread(void *argument)
{
	HAL_TIM_Base_Start_IT(Us100_TIME_nvic);
	while (1)
	{

		switch (measure_Cnt)
		{
		case 0:

			TRIG_H;
			DWT_DelayUS(20);			
			TRIG_L;

			measure_Cnt++;
			__HAL_TIM_SET_CAPTUREPOLARITY(Us100_TIME_nvic, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Start_IT(Us100_TIME_nvic, TIM_CHANNEL_4); // 启动输入捕获       或者: __HAL_TIM_ENABLE(&htim5);
			break;
		case 3:
			high_time = measure_Buf[1] - measure_Buf[0]; // 高电平时间
			distant = (high_time * 0.034) / 2;			 // 单位cm
			U100_DATA.distant = distant;
			xQueueSend(US100_Queue, &U100_DATA, 0);
			measure_Cnt = 0; // 清空标志位
			TIM2->CNT = 0;	 // 清空计时器计数
			vTaskDelay(200);
			break;
		}
	}
}

/*
StaticTask_t * PThread,任务控制块，句柄就是指向任务控制块的指针
StackType_t * P_Stack,//
int L_Stack

*/
uint8_t US100_Thread_Start(TaskHandle_t PThread, StackType_t *P_Stack, int L_Stack)
{

	if (NULL != xTaskCreateStatic(US100_Thread,
								  "us100_th",
								  L_Stack,
								  NULL,
								  (osPriority_t)osPriorityNormal,
								  P_Stack,
								  PThread))
	{

		return 1;
	}
	return 0;
}

void US100_Thread_Delete(TaskHandle_t xTaskToDelete)
{
	HAL_TIM_IC_Stop_IT(Us100_TIME_nvic, TIM_CHANNEL_4);
	vTaskDelete(xTaskToDelete);
	measure_Buf[0] = 0;
	measure_Buf[1] = 0;
	measure_Cnt = 0;
	distant = 0; // 测量距离
	high_time = 0;
	TIM2->CNT = 0;
}



void US100_Timer_ISR(void) //
{

	switch (measure_Cnt)
	{
	case 1:
		measure_Buf[0] = HAL_TIM_ReadCapturedValue(Us100_TIME_nvic, TIM_CHANNEL_4);			   // 获取当前的捕获值.
		__HAL_TIM_SET_CAPTUREPOLARITY(Us100_TIME_nvic, TIM_CHANNEL_4, TIM_ICPOLARITY_FALLING); // 设置为下降沿捕获
		measure_Cnt++;
		break;
	case 2:
		measure_Buf[1] = HAL_TIM_ReadCapturedValue(Us100_TIME_nvic, TIM_CHANNEL_4); // 获取当前的捕获值.
		HAL_TIM_IC_Stop_IT(Us100_TIME_nvic, TIM_CHANNEL_4);							// 停止捕获   或者: __HAL_TIM_DISABLE(&htim5);
		measure_Cnt++;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//
{
	
	if(TIM2 == htim->Instance)// 判断触发的中断的定时器为TIM2
	{
			US100_Timer_ISR();
	
	}
	
}

