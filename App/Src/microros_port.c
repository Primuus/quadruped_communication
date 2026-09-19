#include "cmsis_os2.h"
#include "main.h"
#include "stm32h7xx.h"

#include <stdint.h>
#include <sys/time.h>

int usleep(uint32_t usec)
{
    uint32_t ms = usec / 1000U;
    if (ms == 0U) {
        ms = 1U;
    }
    osDelay(ms);
    return 0;
}

int _gettimeofday(struct timeval *tv, void *tzvp)
{
    (void)tzvp;

    if (tv == NULL) {
        return -1;
    }

    uint32_t ms = HAL_GetTick();
    tv->tv_sec = ms / 1000U;
    tv->tv_usec = (ms % 1000U) * 1000U;
    return 0;
}

/* 替代 GCC/atomic 运行时里的内存屏障符号 */
void __sync_synchronize(void)
{
    __DMB();
    __DSB();
    __ISB();
}

/* 替代 GCC/atomic 运行时里的 32bit 原子交换 */
unsigned int __atomic_exchange_4(volatile void *ptr, unsigned int val, int memorder)
{
    (void)memorder;

    volatile unsigned int *p = (volatile unsigned int *)ptr;
    unsigned int old;

    __disable_irq();
    old = *p;
    *p = val;
    __DMB();
    __enable_irq();

    return old;
}