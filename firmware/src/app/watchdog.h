/*
 * watchdog.h - supervisor that resets the SoC when a watched task hangs (21.10.2026,
 * OPEN_ISSUES 1.2, ERRORS 5.13).
 *
 * The AE350 hardware WDT cannot time out later than 0.49 ms here (ERRORS 5.13), so the supervisor task (the
 * highest priority) resets the SoC itself through the SMU software reset when a watched
 * task stops beating, and configASSERT / stack overflow do the same after recording the
 * cause. Not covered: code spinning with interrupts off and a dead tick interrupt -
 * nothing can run then (an FPGA-side watchdog would be needed). The
 * FPGA is not reset; a frozen MAC receive (ERRORS 3.7, symptom 2) keeps the core healthy
 * and is not this supervisor's case.
 *
 * Why the board restarted survives the reset in DDR3 (WATCHDOG_REC_ADDR, outside the
 * image and the rings); watchdog_boot_report() prints it with the SMU reset status.
 */
#ifndef APP_WATCHDOG_H_
#define APP_WATCHDOG_H_

#include <stdint.h>

typedef enum {
    WD_TASK_ETHIN = 0,      // MAC receive (ethernetif.c), beats every loop (<= 2 ms)
    WD_TASK_TCPIP,          // lwIP thread, probed by a tcpip callback from the supervisor
    WD_TASK_COUNT
} wd_task_t;

#define WD_CAUSE_STALE      1U   // a watched task stopped beating (mask in the record)
#define WD_CAUSE_ASSERT     2U   // configASSERT (line in the record)
#define WD_CAUSE_STACK      3U   // stack overflow hook
#define WD_CAUSE_TEST       4U   // console '#' twice
#define WD_CAUSE_REQUEST    5U   // DeviceReset requested over GigE Vision (gvcp.c)

void watchdog_start(void);                    // creates the supervisor, before the scheduler
void watchdog_beat(wd_task_t t);
void watchdog_fatal(uint32_t cause, uint32_t detail, const char *what);   // record, reset the SoC
void watchdog_boot_report(void);              // early in main(): last reset and its cause
void watchdog_console(void);                  // console '#': show; twice within 5 s resets (test)

#endif /* APP_WATCHDOG_H_ */
