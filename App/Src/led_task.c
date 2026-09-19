#include "app_tasks.h"

// void StartLedTask(void *argument)
// {
//   (void)argument;
//   HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
//   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//   HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
//   HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
//   for (;;)
//   {
//     HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//     osDelay(100U);
//     HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//     HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//     osDelay(100U);
//     HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//     HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//     osDelay(100U);
//     HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//     HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//     osDelay(100U);
//     HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//   }
// }

void StartLedTask(void *argument)
{
  (void)argument;

  uint32_t last_count = 0;

  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);

  for (;;)
  {
    if (g_tau_cmd.valid && g_tau_cmd.update_count != last_count)
    {
      last_count = g_tau_cmd.update_count;

      /* 收到新的 tau 数据时：四个灯一起快闪 */
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
      osDelay(100U);
    }
    else
    {
      /* 没有收到新 tau 数据时：保持原来的流水灯 */
      // HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
      // osDelay(100U);
      // HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
      //
      // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      // osDelay(100U);
      // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      //
      // HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      // osDelay(100U);
      // HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      //
      // HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
      // osDelay(100U);
      // HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
      osDelay(100U);
    }
  }
}