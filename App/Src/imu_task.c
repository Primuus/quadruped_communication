#include "app_tasks.h"

void StartImuTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    osDelay(1000);
  }
}
