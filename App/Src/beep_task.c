#include "app_tasks.h"

extern osMessageQueueId_t g_beepFrameQueueHandle;
static AppFrameMessage g_beepMessage;

#define APP_BEEP_SHORT_HOLD_MS 200U
#define APP_BEEP_LONG_HOLD_MS 500U

static uint32_t BeepTask_GetHoldMs(uint8_t last_data_byte)
{
  if ((last_data_byte >= 0x01U) && (last_data_byte <= 0x0FU))
  {
    return APP_BEEP_SHORT_HOLD_MS;
  }

  if ((last_data_byte >= 0x11U) && (last_data_byte <= 0x1FU))
  {
    return APP_BEEP_LONG_HOLD_MS;
  }

  return 0U;
}

static uint32_t BeepTask_GetNextDeadline(uint32_t current_deadline, uint32_t now, uint32_t hold_ms)
{
  if ((current_deadline == 0U) || ((int32_t)(now - current_deadline) >= 0))
  {
    return now + hold_ms;
  }

  return current_deadline + hold_ms;
}

static uint32_t BeepTask_GetWaitTimeout(uint32_t beep_deadline, uint32_t now)
{
  if (beep_deadline == 0U)
  {
    return osWaitForever;
  }

  if ((int32_t)(now - beep_deadline) >= 0)
  {
    return 0U;
  }

  return (uint32_t)(beep_deadline - now);
}

void StartBeepTask(void *argument)
{
  uint32_t beep_deadline = 0U;
  uint32_t hold_ms;
  uint32_t timeout;
  osStatus_t status;
  uint32_t now;

  (void)argument;

  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

  for (;;)
  {
    now = osKernelGetTickCount();
    timeout = BeepTask_GetWaitTimeout(beep_deadline, now);
    status = osMessageQueueGet(g_beepFrameQueueHandle, &g_beepMessage, NULL, timeout);
    now = osKernelGetTickCount();

    if (status == osOK)
    {
      hold_ms = BeepTask_GetHoldMs(g_beepMessage.last_data_byte);
      if (hold_ms > 0U)
      {
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        beep_deadline = BeepTask_GetNextDeadline(beep_deadline, now, hold_ms);
      }
    }

    if ((beep_deadline != 0U) && ((int32_t)(now - beep_deadline) >= 0))
    {
      beep_deadline = 0U;
      HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    }
  }
}
