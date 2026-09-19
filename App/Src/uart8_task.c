#include "app_tasks.h"

#include <string.h>

static AppFrameMessage g_frameMessage;
static AppUart8RxChunk g_uart8RxChunk;
static QlpParser g_uart8Parser;

static void Uart8Task_OnFrame(QlpParser *frame)
{
  if ((frame == NULL) || (frame->len > QLP_MAX_DATA_LEN))
  {
    return;
  }

  g_frameMessage.frame_len = (uint16_t)(QLP_OVERHEAD_SIZE + frame->len);
  if (g_frameMessage.frame_len > (uint16_t)sizeof(g_frameMessage.frame))
  {
    return;
  }

  g_frameMessage.last_data_byte = (frame->len > 0U) ? frame->data[frame->len - 1U] : 0U;
  memcpy(g_frameMessage.frame, frame->buffer, g_frameMessage.frame_len);
  if (g_frameMessage.frame[1] == 0x11)
  {
    if (g_uart7FrameQueueHandle != NULL)
    {
      (void)osMessageQueuePut(g_uart7FrameQueueHandle, &g_frameMessage, 0U, 0U);
    }
  }
  if (g_frameMessage.frame[1] & 0x20)
  {

  }

  if (g_beepFrameQueueHandle != NULL)
  {
    (void)osMessageQueuePut(g_beepFrameQueueHandle, &g_frameMessage, 0U, 0U);
  }
}

void StartUart8RecTask(void *argument)
{
  (void)argument;

  Qlp_ParserInit(&g_uart8Parser);
  AppTasks_StartUart8DmaReceive();

  for (;;)
  {
    if (osMessageQueueGet(g_uart8RxQueueHandle, &g_uart8RxChunk, NULL, osWaitForever) == osOK)
    {
      Qlp_ParseBytes(&g_uart8Parser,
                     g_uart8RxChunk.data,
                     g_uart8RxChunk.size,
                     Uart8Task_OnFrame);
    }
  }
}
