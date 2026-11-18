/*
 * watchdog.c - supervisor that resets the SoC when a watched task hangs (watchdog.h).
 *
 * The AE350 WDT is absent from our SoC config (every register at 0xF0500000 reads its ID
 * 0x03002004), and with AE350_APB_WDT_EN it times out after 0.49 ms at most (16-bit
 * counter on a 100 MHz clock, ERRORS 5.13), so the reset is the SMU software reset (SMUCR 0x3C, "reset the whole
 * system", MUG1029 table 16-6), issued by the supervisor task itself. It covers hung
 * and deadlocked tasks, a stalled lwIP thread, configASSERT and stack overflows. It
 * does not cover code spinning with interrupts off or a dead tick: nothing runs then.
 */
#include <stdio.h>
#include <string.h>
#include "platform.h"
#include "uart.h"
#include "cache.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/tcpip.h"
#include "memory_map.h"
#include "watchdog.h"

#define SMU_CMD_SW_RESET    0x3CU
#define SMU_WRSR_WDT        (1U << 3)
#define SMU_WRSR_SW         (1U << 4)

#define WD_PERIOD_MS        500U     // supervisor checks this often
#define WD_STALE_MS         3000U    // a task silent for longer counts as hung
#define WD_REC_MAGIC        0x57444F47U   /* "WDOG" */

// The record in DDR3 (WATCHDOG_REC_ADDR): survives the SoC reset, not a power-off.
typedef struct {
    uint32_t magic;
    uint32_t resets;        // supervisor resets since power-up (counted at boot)
    uint32_t cause;         // WD_CAUSE_*, 0 = none
    uint32_t detail;        // stale mask, assert line
    uint32_t uptime_ms;     // when it was recorded
    char     what[44];      // task name or file
} wd_rec_t;

static volatile wd_rec_t *const s_rec = (volatile wd_rec_t *)WATCHDOG_REC_ADDR;
static volatile uint32_t s_beat_ms[WD_TASK_COUNT];
static volatile uint32_t s_test_ms;       // console '#': time of the first press

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void rec_write(uint32_t cause, uint32_t detail, const char *what)
{
    uint32_t i;

    if (s_rec->magic != WD_REC_MAGIC || s_rec->cause != 0U) return;   // keep the first cause
    s_rec->cause = cause;
    s_rec->detail = detail;
    s_rec->uptime_ms = now_ms();
    for (i = 0; i < sizeof(s_rec->what) - 1U && what != NULL && what[i] != '\0'; ++i)
        s_rec->what[i] = what[i];
    s_rec->what[i] = '\0';
    ae350_dcache_flush_range(WATCHDOG_REC_ADDR, sizeof(wd_rec_t));
}

void watchdog_beat(wd_task_t t)
{
    if ((unsigned)t < WD_TASK_COUNT) s_beat_ms[t] = now_ms();
}

void watchdog_fatal(uint32_t cause, uint32_t detail, const char *what)
{
    volatile uint32_t spin;

    rec_write(cause, detail, what);
    uart_log_panic();                                 // the log ring, then direct output
    for (spin = 0; spin < 2000000U; ++spin) { }   // ~a few ms: let the UART drain the last line
    DEV_SMU->HART0_RESET_VECTOR = 0x80000000UL;
    DEV_SMU->SMUCR = SMU_CMD_SW_RESET;
    for (;;) { }
}

static void tcpip_probe(void *arg)
{
    (void)arg;
    watchdog_beat(WD_TASK_TCPIP);
}

static void task_watchdog(void *arg)
{
    static const char *const k_names[WD_TASK_COUNT] = { "ETHIN", "tcpip" };
    uint32_t t, i;

    (void)arg;
    t = now_ms();
    for (i = 0; i < WD_TASK_COUNT; ++i) s_beat_ms[i] = t;
    printf("[WDT] supervisor on: SoC reset when ETHIN or tcpip is silent for %u ms\r\n",
           (unsigned)WD_STALE_MS);
    for (;;) {
        (void)tcpip_try_callback(tcpip_probe, NULL);
        vTaskDelay(pdMS_TO_TICKS(WD_PERIOD_MS));
        t = now_ms();
        for (i = 0; i < WD_TASK_COUNT; ++i) {
            int32_t age = (int32_t)(t - s_beat_ms[i]);   // a beat after t reads as negative
            if (age > (int32_t)WD_STALE_MS) {
                printf("[WDT] task %s silent for %d ms: SoC reset\r\n", k_names[i], (int)age);
                watchdog_fatal(WD_CAUSE_STALE, 1U << i, k_names[i]);
            }
        }
    }
}

void watchdog_start(void)
{
#ifndef WATCHDOG_ENABLE
#define WATCHDOG_ENABLE 1
#endif
#if WATCHDOG_ENABLE
    // Highest priority: a task spinning at any lower priority cannot starve it.
    BaseType_t rc = xTaskCreate(task_watchdog, "Task_Watchdog", configMINIMAL_STACK_SIZE * 4, NULL,
                                configMAX_PRIORITIES - 1, NULL);
    if (rc != pdPASS) printf("[WDT] supervisor task NOT created: no watchdog\r\n");
#endif
}

void watchdog_boot_report(void)
{
    uint32_t wrsr = DEV_SMU->WRSR;

    // CPU-written memory, but nothing has written it since this reset: invalidate is safe.
    ae350_dcache_invalidate_range(WATCHDOG_REC_ADDR, sizeof(wd_rec_t));
    if (s_rec->magic != WD_REC_MAGIC) {
        memset((void *)s_rec, 0, sizeof(wd_rec_t));
        s_rec->magic = WD_REC_MAGIC;
    }
    // A cause counts only with the SMU software-reset bit (0x10, measured 21.10.2026): a
    // record left by a session that was then reset by the programmer or power is stale.
    if (s_rec->cause != 0U && (wrsr & SMU_WRSR_SW)) {
        s_rec->resets++;
        printf("[BOOT] SUPERVISOR RESET #%u (WRSR 0x%X): cause %u detail 0x%X at %u ms, %s\r\n",
               (unsigned)s_rec->resets, (unsigned)wrsr, (unsigned)s_rec->cause,
               (unsigned)s_rec->detail, (unsigned)s_rec->uptime_ms, (const char *)s_rec->what);
    } else {
        printf("[BOOT] reset status WRSR 0x%X (%s), supervisor resets since power-up %u\r\n",
               (unsigned)wrsr, (wrsr & SMU_WRSR_SW) ? "software" : (wrsr & SMU_WRSR_WDT) ? "WDT" :
               (wrsr & 0x3U) ? "power-on" : "hardware", (unsigned)s_rec->resets);
    }
    DEV_SMU->WRSR = wrsr;                 // W1C
    s_rec->cause = 0;
    ae350_dcache_flush_range(WATCHDOG_REC_ADDR, sizeof(wd_rec_t));
}

void watchdog_console(void)
{
    uint32_t t = now_ms(), i;

    printf("[WDT] supervisor resets since power-up %u, beats ago:", (unsigned)s_rec->resets);
    for (i = 0; i < WD_TASK_COUNT; ++i) printf(" %d ms", (int)(int32_t)(t - s_beat_ms[i]));
    printf("\r\n");
    if (s_test_ms != 0U && t - s_test_ms < 5000U) {
        printf("[WDT] test: SoC reset now\r\n");
        watchdog_fatal(WD_CAUSE_TEST, 0, "console test");
    } else {
        s_test_ms = t;
        printf("[WDT] '#' again within 5 s resets the SoC (test)\r\n");
    }
}
