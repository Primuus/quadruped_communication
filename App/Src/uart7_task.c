#include "app_tasks.h"

#include <string.h>

#include "usart.h"

static AppFrameMessage g_uart7Message;
static AppUart7RxChunk g_uart7RxChunk;
static QlpParser g_uart7Parser;

static uint32_t Uart7Task_ReadU32LE(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static osMessageQueueId_t Uart7Task_GetCanQueue(uint8_t addr)
{
  if ((addr >= APP_CAN1_ADDR_MIN) && (addr <= APP_CAN1_ADDR_MAX))
  {
    return g_can1TxQueueHandle;
  }

  if ((addr >= APP_CAN2_ADDR_MIN) && (addr <= APP_CAN2_ADDR_MAX))
  {
    return g_can2TxQueueHandle;
  }

  return NULL;
}

static void Uart7Task_OnFrame(QlpParser *frame)
{
  AppCanTxMessage can_message;
  osMessageQueueId_t queue;

  if ((frame == NULL) || (frame->len != APP_UART_CAN_FRAME_LEN))
  {
    return;
  }

  queue = Uart7Task_GetCanQueue(frame->addr);
  if (queue == NULL)
  {
    return;
  }

  can_message.ext_id = Uart7Task_ReadU32LE(frame->data) & 0x1FFFFFFFU;
  memcpy(can_message.data, &frame->data[4], sizeof(can_message.data));
  (void)osMessageQueuePut(queue, &can_message, 0U, 0U);
}

void Startuart7RecTask(void *argument)
{
  (void)argument;

  Qlp_ParserInit(&g_uart7Parser);
  AppTasks_StartUart7DmaReceive();

  for (;;)
  {
    if (osMessageQueueGet(g_uart7RxQueueHandle, &g_uart7RxChunk, NULL, osWaitForever) == osOK)
    {
      Qlp_ParseBytes(&g_uart7Parser,
                     g_uart7RxChunk.data,
                     g_uart7RxChunk.size,
                     Uart7Task_OnFrame);
    }
  }
}

void StartUart7TransTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    if (osMessageQueueGet(g_uart7FrameQueueHandle, &g_uart7Message, NULL, osWaitForever) == osOK)
    {
      (void)HAL_UART_Transmit(&huart7,
                              g_uart7Message.frame,
                              g_uart7Message.frame_len,
                              HAL_MAX_DELAY);
    }
  }
}
