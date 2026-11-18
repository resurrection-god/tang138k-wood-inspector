/*
 * board.c - small board services needed by several drivers.
 */

#include <stdio.h>
#include "board.h"
#include "platform.h"
#include "FreeRTOS.h"
#include "task.h"
#include "watchdog.h"

// Microsecond delay function based on MTIME (100 MHz APB clock = 100 ticks/us)
void board_delay_us(uint32_t us)
{
    uint32_t start = DEV_PLMT->MTIME[0];
    uint32_t ticks = us * 100;
    while ((DEV_PLMT->MTIME[0] - start) < ticks);
}

// FreeRTOS task run-time counter (portGET_RUN_TIME_COUNTER_VALUE):
// MTIME / 128, a 1.28 us tick, 32 bits, wraps every ~91 min. A shift rather
// than a division by 100: it is called on every task switch, and a 64-bit
// division on RV32 is a library call. For time shares the unit does not matter.
unsigned long board_runtime_us(void)
{
    uint32_t hi, lo;
    do {
        hi = DEV_PLMT->MTIME[1];
        lo = DEV_PLMT->MTIME[0];
    } while (hi != DEV_PLMT->MTIME[1]);
    return (unsigned long)((hi << 25) | (lo >> 7));
}


// ---- FreeRTOS hooks (main.c until 24.09.2026) ----------------------------------
//
// configASSERT() is also called by lower-layer drivers (i2c_bus, camera), so
// the handler lives here, in the lowest layer, not in main.c.

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("\r\n[FATAL] Stack overflow in task: %s!\r\n", pcTaskName);
    watchdog_fatal(WD_CAUSE_STACK, 0, pcTaskName);   // the WDT resets the board
    while(1);
}

// The FreeRTOS heap (configTOTAL_HEAP_SIZE) said no. Callers (lwIP mailboxes, task
// creation) handle NULL, so only say it: a silent failure was invisible (OPEN_ISSUES 1.2).
void vApplicationMallocFailedHook(void)
{
    static uint32_t s_n;
    printf("[HEAP] pvPortMalloc failed (#%u), free %u B\r\n", (unsigned)++s_n,
           (unsigned)xPortGetFreeHeapSize());
}

void vAssertCalled(const char * pcFile, unsigned long ulLine)
{
    volatile unsigned long ul = 0;

    (void) pcFile;
    (void) ulLine;

    printf("ASSERT! Line %d, file %s\r\n", ( int )ulLine, pcFile);
    watchdog_fatal(WD_CAUSE_ASSERT, (uint32_t)ulLine, pcFile);   // the WDT resets the board

    taskENTER_CRITICAL();
    {
        while (ul == 0)
        {
            portNOP();
        }
    }
    taskEXIT_CRITICAL();
}
