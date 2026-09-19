#include "app_tasks.h"

#include <string.h>

#include "fdcan.h"
#include "usart.h"

osMessageQueueId_t g_uart7RxQueueHandle = NULL;
osMessageQueueId_t g_uart8RxQueueHandle = NULL;
osMessageQueueId_t g_uart7FrameQueueHandle = NULL;
osMessageQueueId_t g_beepFrameQueueHandle = NULL;
osMessageQueueId_t g_can1TxQueueHandle = NULL;
osMessageQueueId_t g_can2TxQueueHandle = NULL;

static uint8_t g_uart7DmaBuffer[APP_UART7_DMA_BUFFER_SIZE];
static uint8_t g_uart8DmaBuffer[APP_UART8_DMA_BUFFER_SIZE];

static void AppTasks_StartCan(FDCAN_HandleTypeDef *hfdcan)
{
  if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_TX_BUFFER0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void AppTasks_StartUartDmaReceive(UART_HandleTypeDef *huart,
                                         uint8_t *buffer,
                                         uint16_t buffer_size)
{
  HAL_StatusTypeDef status;

  if ((huart == NULL) || (buffer == NULL) || (buffer_size == 0U))
  {
    return;
  }

  if (huart->RxState != HAL_UART_STATE_READY)
  {
    return;
  }

  status = HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, buffer_size);
  if (status != HAL_OK)
  {
    Error_Handler();
  }

  if (huart->hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
}

static void AppTasks_HandleUart7RxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
  AppUart7RxChunk chunk;

  if ((huart == NULL) || (huart->Instance != UART7) || (g_uart7RxQueueHandle == NULL))
  {
    return;
  }

  if (size > (uint16_t)sizeof(chunk.data))
  {
    size = (uint16_t)sizeof(chunk.data);
  }

  if (size > 0U)
  {
    chunk.size = size;
    memcpy(chunk.data, g_uart7DmaBuffer, size);
    (void)osMessageQueuePut(g_uart7RxQueueHandle, &chunk, 0U, 0U);
  }
}

static void AppTasks_HandleUart8RxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
  AppUart8RxChunk chunk;

  if ((huart == NULL) || (huart->Instance != UART8) || (g_uart8RxQueueHandle == NULL))
  {
    return;
  }

  if (size > (uint16_t)sizeof(chunk.data))
  {
    size = (uint16_t)sizeof(chunk.data);
  }

  if (size > 0U)
  {
    chunk.size = size;
    memcpy(chunk.data, g_uart8DmaBuffer, size);
    (void)osMessageQueuePut(g_uart8RxQueueHandle, &chunk, 0U, 0U);
  }
}

void AppTasks_InitRuntime(void)
{
  // g_uart7RxQueueHandle = osMessageQueueNew(APP_UART7_RX_QUEUE_DEPTH,
  //                                          sizeof(AppUart7RxChunk),
  //                                          NULL);
  g_uart8RxQueueHandle = osMessageQueueNew(APP_UART8_RX_QUEUE_DEPTH,
                                           sizeof(AppUart8RxChunk),
                                           NULL);
  // g_uart7FrameQueueHandle = osMessageQueueNew(APP_FRAME_QUEUE_DEPTH,
  //                                             sizeof(AppFrameMessage),
  //                                             NULL);
  g_beepFrameQueueHandle = osMessageQueueNew(APP_FRAME_QUEUE_DEPTH,
                                             sizeof(AppFrameMessage),
                                             NULL);
  g_can1TxQueueHandle = osMessageQueueNew(APP_CAN_TX_QUEUE_DEPTH,
                                          sizeof(AppCanTxMessage),
                                          NULL);
  g_can2TxQueueHandle = osMessageQueueNew(APP_CAN_TX_QUEUE_DEPTH,
                                          sizeof(AppCanTxMessage),
                                          NULL);

  AppTasks_StartCan(&hfdcan1);
  AppTasks_StartCan(&hfdcan2);
}

void AppTasks_StartUart7DmaReceive(void)
{
  // AppTasks_StartUartDmaReceive(&huart7,
  //                              g_uart7DmaBuffer,
  //                              (uint16_t)sizeof(g_uart7DmaBuffer));
  return;
}

void AppTasks_StartUart8DmaReceive(void)
{
  AppTasks_StartUartDmaReceive(&huart8,
                               g_uart8DmaBuffer,
                               (uint16_t)sizeof(g_uart8DmaBuffer));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  // if ((huart != NULL) && (huart->Instance == UART7))
  // {
  //   AppTasks_HandleUart7RxEvent(huart, Size);
  //   AppTasks_StartUart7DmaReceive();
  //   return;
  // }

  if ((huart != NULL) && (huart->Instance == UART8))
  {
    AppTasks_HandleUart8RxEvent(huart, Size);
    AppTasks_StartUart8DmaReceive();
  }
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
  if(hfdcan->Instance == FDCAN1)
  {
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  }

  else if(hfdcan->Instance == FDCAN2)
  {
    return;
  }
}