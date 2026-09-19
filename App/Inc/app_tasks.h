#ifndef APP_TASKS_H
#define APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include "main.h"
#include "qlp_protocol.h"

#define APP_UART7_DMA_BUFFER_SIZE 64U
#define APP_UART8_DMA_BUFFER_SIZE 300U
#define APP_UART7_RX_QUEUE_DEPTH 4U
#define APP_UART8_RX_QUEUE_DEPTH 4U
#define APP_FRAME_QUEUE_DEPTH 4U
#define APP_CAN_TX_QUEUE_DEPTH 8U
#define APP_CAN_FRAME_DATA_SIZE 8U

#define APP_CAN1_ADDR_MIN 0x21U
#define APP_CAN1_ADDR_MAX 0x26U
#define APP_CAN2_ADDR_MIN 0x27U
#define APP_CAN2_ADDR_MAX 0x2CU

#define APP_UART_CAN_FRAME_LEN 12U

typedef struct
{
  uint16_t size;
  uint8_t data[APP_UART7_DMA_BUFFER_SIZE];
} AppUart7RxChunk;

typedef struct
{
  uint16_t size;
  uint8_t data[APP_UART8_DMA_BUFFER_SIZE];
} AppUart8RxChunk;

typedef struct
{
  uint16_t frame_len;
  uint8_t last_data_byte;
  uint8_t frame[QLP_MAX_FRAME_SIZE];
} AppFrameMessage;

typedef struct
{
  uint32_t ext_id;
  uint8_t data[APP_CAN_FRAME_DATA_SIZE];
} AppCanTxMessage;


typedef struct
{
    double tau[12];
    uint8_t valid;
    uint32_t update_count;
} AppTauCommand;

extern osMessageQueueId_t g_uart7RxQueueHandle;
extern osMessageQueueId_t g_uart8RxQueueHandle;
extern osMessageQueueId_t g_uart7FrameQueueHandle;
extern osMessageQueueId_t g_beepFrameQueueHandle;
extern osMessageQueueId_t g_can1TxQueueHandle;
extern osMessageQueueId_t g_can2TxQueueHandle;


void AppTasks_InitRuntime(void);
void AppTasks_StartUart7DmaReceive(void);
void AppTasks_StartUart8DmaReceive(void);

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

extern volatile AppTauCommand g_tau_cmd;
#ifdef __cplusplus
}
#endif

#endif
