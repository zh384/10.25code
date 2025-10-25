/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "zdrive.h"
#include "valve.h"
#include "bluetooth.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t flag = 0;
uint8_t flag_rise = 0;
uint8_t flag_descend = 0;
uint8_t flag_advance = 0;
uint8_t flag_advance_second = 0;
int16_t speed_up = 0;
int16_t speed_down = 0;
uint8_t cmd=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void is_get_enrygy(void *argument);
void launch(void *argument);
void dj(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(is_get_enrygy, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(launch, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(dj, NULL, &myTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_is_get_enrygy */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_is_get_enrygy */
__weak void is_get_enrygy(void *argument)
{
  /* USER CODE BEGIN is_get_enrygy */
  /* Infinite loop */
  for(;;)
  { uint32_t id = M_RxHeader.ExtId;
		if(id==0x01020100){
		   CAN_FRAME tx_frame;
       uint8_t data[8] = {'C', 'R'};
       CanCommand(&tx_frame, CAN_ID_EXT, 0x01020100, data, 2);
       CAN_Queue_Push(&CAN2_Queue, tx_frame);
		}
		if(id==0x010205FF){
		  __disable_irq();
      NVIC_SystemReset();
		}
		if(id==0x01020501){
			if(rxmsg[1]==0){
			  ZDrive[0].enable = false;
				ZDrive[0].begin = false;
				ZDrive[0].Mode = Zdrive_Disable;
				ZDrive[1].enable = false;
				ZDrive[1].begin = false;
				ZDrive[1].Mode = Zdrive_Disable;
				CAN_FRAME tx_frame;
        uint8_t data[8] = {'M', 0};
        CanCommand(&tx_frame, CAN_ID_EXT, 0x05020101, data, 2);
        CAN_Queue_Push(&CAN2_Queue, tx_frame);
    
			}
			if(rxmsg[1]==1){
			  ZDrive[0].enable = true;
				ZDrive[0].begin = true;
				ZDrive[0].Mode = Zdrive_Speed;
				ZDrive[1].enable = true;
				ZDrive[1].begin = true;
				ZDrive[1].Mode = Zdrive_Speed;
        CAN_FRAME tx_frame;
        uint8_t data[8] = {'M', 1};
        CanCommand(&tx_frame, CAN_ID_EXT, 0x05020101, data, 2);
        CAN_Queue_Push(&CAN2_Queue, tx_frame); 
			}  
		}
    osDelay(1);
  }
  /* USER CODE END is_get_enrygy */
}

/* USER CODE BEGIN Header_launch */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_launch */
__weak void launch(void *argument)
{
  /* USER CODE BEGIN launch */
  /* Infinite loop */
  for(;;)
  {
		if(RxMsgPack.bools[5]==1){
		  radiotube(cmd);
		}
    osDelay(1);
  }
  /* USER CODE END launch */
}

/* USER CODE BEGIN Header_dj */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dj */
__weak void dj(void *argument)
{
  /* USER CODE BEGIN dj */
  /* Infinite loop */
  for(;;)
  { if(RxMsgPack.bools[0]==0){
	     for(int i=0;i<10;i++){
			     ZDrive[i].enable=0;
           ZDrive[i].begin=0;
           ZDrive[i].Mode=Zdrive_Disable;				 
			 }
		if(RxMsgPack.bools[0]==1){
	     for(int i=0;i<10;i++){
			     ZDrive[i].enable=1;
           ZDrive[i].begin=1;
           ZDrive[i].Mode=Zdrive_Position;				 
			 }
		if(RxMsgPack.bools[1]==1){
		   ZDrive[8].ValueSetNow.angle=100;
			 ZDrive[9].ValueSetNow.angle=100;
		}
		if(RxMsgPack.bools[2]==1){
		   for(int k=0;k<8;k++){
			      ZDrive[k].ValueSetNow.angle=100;
			 }
		}
		if(RxMsgPack.bools[3]==1){
		  ZDrive[8].ValueSetNow.angle=-100;
			ZDrive[9].ValueSetNow.angle=-100;
		}
		if(RxMsgPack.bools[4]==1){
		  for(int k=0;k<8;k++){
			      ZDrive[k].ValueSetNow.angle=100;
			 }
		}
	}
    osDelay(1);
  }
}
  /* USER CODE END dj */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


