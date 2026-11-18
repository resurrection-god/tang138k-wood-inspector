/*
 * console.h - UART console commands (console.c).
 */

#ifndef APP_CONSOLE_H_
#define APP_CONSOLE_H_

// Execute one console command. task_console (main.c) calls it for every byte
// received from the UART; an unknown character is ignored.
void console_command(int cmd);

// PHY, EEE and lwIP counters in one go: task_console prints it once, 25 s
// after the network is up (RX-freeze experiment, ERRORS 3.15).
void console_phy_boot_report(void);
void console_plic_dump(void);   /* console '!' */

#endif /* APP_CONSOLE_H_ */
