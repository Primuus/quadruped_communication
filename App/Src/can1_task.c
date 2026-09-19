#include "app_tasks.h"
#include "fdcan.h"
#include "robstrite.h"
#include "string.h"



static AppCanTxMessage g_can1TxMessage;
extern volatile AppTauCommand g_tau_cmd;


static const uint8_t g_can1MotorIdMap[6] =
{
  0U,
  1U,
  2U,
  3U,
  4U,
  5U
};

static void Can1Task_Send(const AppCanTxMessage *message)
{
  FDCAN_TxHeaderTypeDef tx_header;

  if (message == NULL)
  {
    return;
  }

  tx_header.Identifier = message->ext_id;
  tx_header.IdType = FDCAN_EXTENDED_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_8;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  uint32_t start_tick = osKernelGetTickCount();

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
  {
    if ((osKernelGetTickCount() - start_tick) >= 100)
    {
      return;
    }
    osDelay(1U);
  }
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&tx_header,(uint8_t *)message->data);
}

void StartCan1RecTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    osDelay(1000U);
  }
}

void StartCan1TransTask(void *argument)
{
  AppTauCommand local_tau_cmd;
  uint32_t last_update_count = 0U;

  (void)argument;
  memset(&local_tau_cmd, 0, sizeof(local_tau_cmd));

  MotorCan_SendRobStriteEnableRange(g_can1MotorIdMap,
                                    6U,
                                    CAN_ROBSTRITE_MASTER_ID,
                                    Can1Task_Send);

  //
  // AppTauCommand test = {0};
  // test.valid = 1;
  // test.update_count = last_update_count;
  // memset(test.tau, 0, sizeof(text.tau));
  // for (int i = 0; i < 12; i++){test.tau[i]=0.5;}
  // MotorCan_SendRobStriteTauRange(&test,
  //                                g_can1MotorIdMap,
  //                                0U,
  //                                6U,
  //                                Can1Task_Send);
  for (;;)
  {
    if ((g_tau_cmd.valid != 0U) &&
        (g_tau_cmd.update_count != last_update_count))
    {
      __disable_irq();
      memcpy(&local_tau_cmd, (const void *)&g_tau_cmd, sizeof(AppTauCommand));
      __enable_irq();

      last_update_count = local_tau_cmd.update_count;

      MotorCan_SendRobStriteTauRange(&local_tau_cmd,
                                     g_can1MotorIdMap,
                                     0U,
                                     6U,
                                     Can1Task_Send);
      HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    }


    while (osMessageQueueGet(g_can1TxQueueHandle,
                             &g_can1TxMessage,
                             NULL,
                             0U) == osOK)
    {
      Can1Task_Send(&g_can1TxMessage);
    }

    osDelay(1U);
  }
}
