/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "app_tasks.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
/* Definitions for uart7TransTask */
osThreadId_t uart7TransTaskHandle;
const osThreadAttr_t uart7TransTask_attributes = {
  .name = "uart7TransTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uart8RecTask */
osThreadId_t uart8RecTaskHandle;
const osThreadAttr_t uart8RecTask_attributes = {
  .name = "uart8RecTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for can1RecTask */
osThreadId_t can1RecTaskHandle;
const osThreadAttr_t can1RecTask_attributes = {
  .name = "can1RecTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for can2RecTask */
osThreadId_t can2RecTaskHandle;
const osThreadAttr_t can2RecTask_attributes = {
  .name = "can2RecTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for beepTask */
osThreadId_t beepTaskHandle;
const osThreadAttr_t beepTask_attributes = {
  .name = "beepTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uart7RecTask */
osThreadId_t uart7RecTaskHandle;
const osThreadAttr_t uart7RecTask_attributes = {
  .name = "uart7RecTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for can1TransTask */
osThreadId_t can1TransTaskHandle;
const osThreadAttr_t can1TransTask_attributes = {
  .name = "can1TransTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for can2TransTask */
osThreadId_t can2TransTaskHandle;
const osThreadAttr_t can2TransTask_attributes = {
  .name = "can2TransTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* Definitions for MicrorosTransTask */
osThreadId_t microRosTaskHandle;
const osThreadAttr_t microRosTask_attributes = {
  .name = "microRosTask",
  .stack_size = 3072 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE END FunctionPrototypes */

void StartUart7TransTask(void *argument);
void StartUart8RecTask(void *argument);
void StartCan1RecTask(void *argument);
void StartCan2RecTask(void *argument);
void StartImuTask(void *argument);
void StartBeepTask(void *argument);
void StartLedTask(void *argument);
void Startuart7RecTask(void *argument);
void StartCan1TransTask(void *argument);
void StartCan2TransTask(void *argument);
void StartMicroRosTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  AppTasks_InitRuntime();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of uart7TransTask */
  // uart7TransTaskHandle = osThreadNew(StartUart7TransTask, NULL, &uart7TransTask_attributes);

  /* creation of uart8RecTask */
  uart8RecTaskHandle = osThreadNew(StartUart8RecTask, NULL, &uart8RecTask_attributes);

  /* creation of can1RecTask */
  can1RecTaskHandle = osThreadNew(StartCan1RecTask, NULL, &can1RecTask_attributes);

  /* creation of can2RecTask */
  can2RecTaskHandle = osThreadNew(StartCan2RecTask, NULL, &can2RecTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);

  /* creation of beepTask */
  beepTaskHandle = osThreadNew(StartBeepTask, NULL, &beepTask_attributes);

  /* creation of ledTask */
  // ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  /* creation of uart7RecTask */
  // uart7RecTaskHandle = osThreadNew(Startuart7RecTask, NULL, &uart7RecTask_attributes);

  /* creation of can1TransTask */
  can1TransTaskHandle = osThreadNew(StartCan1TransTask, NULL, &can1TransTask_attributes);

  /* creation of can2TransTask */
  can2TransTaskHandle = osThreadNew(StartCan2TransTask, NULL, &can2TransTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* creation of MicrorosTransTask */
  microRosTaskHandle = osThreadNew(StartMicroRosTask, NULL, &microRosTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartUart7TransTask */
/**
  * @brief  Function implementing the uart7TransTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUart7TransTask */
__weak void StartUart7TransTask(void *argument)
{
  /* USER CODE BEGIN StartUart7TransTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUart7TransTask */
}

/* USER CODE BEGIN Header_StartUart8RecTask */
/**
* @brief Function implementing the uart8RecTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUart8RecTask */
__weak void StartUart8RecTask(void *argument)
{
  /* USER CODE BEGIN StartUart8RecTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUart8RecTask */
}

/* USER CODE BEGIN Header_StartCan1RecTask */
/**
* @brief Function implementing the can1RecTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCan1RecTask */
__weak void StartCan1RecTask(void *argument)
{
  /* USER CODE BEGIN StartCan1RecTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCan1RecTask */
}

/* USER CODE BEGIN Header_StartCan2RecTask */
/**
* @brief Function implementing the can2RecTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCan2RecTask */
__weak void StartCan2RecTask(void *argument)
{
  /* USER CODE BEGIN StartCan2RecTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCan2RecTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
__weak void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartBeepTask */
/**
* @brief Function implementing the beepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBeepTask */
__weak void StartBeepTask(void *argument)
{
  /* USER CODE BEGIN StartBeepTask */
  uint8_t remoter_presse_flag = 0;
  /* Infinite loop */
  for(;;)
  {
    // if (count_press)
    // {
    //   count_press--;
    //   if (count_press <= 0)
    //   {
    //     count_press = 0U;
    //     HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    //   }
    // }
    // HAL_UART_Transmit(&huart7, remoter_tx_buf, remoter_rx_dma_buf_len, 100);
    HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
    osDelay(500);
  }
  /* USER CODE END StartBeepTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
__weak void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {
    // HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    osDelay(500);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_Startuart7RecTask */
/**
* @brief Function implementing the uart7RecTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startuart7RecTask */
__weak void Startuart7RecTask(void *argument)
{
  /* USER CODE BEGIN Startuart7RecTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Startuart7RecTask */
}

/* USER CODE BEGIN Header_StartCan1TransTask */
/**
* @brief Function implementing the can1TransTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCan1TransTask */
__weak void StartCan1TransTask(void *argument)
{
  /* USER CODE BEGIN StartCan1TransTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCan1TransTask */
}

/* USER CODE BEGIN Header_StartCan2TransTask */
/**
* @brief Function implementing the can2TransTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCan2TransTask */
__weak void StartCan2TransTask(void *argument)
{
  /* USER CODE BEGIN StartCan2TransTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCan2TransTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;

  __disable_irq();
  for (;;)
  {
  }
}

/* USER CODE END Application */

