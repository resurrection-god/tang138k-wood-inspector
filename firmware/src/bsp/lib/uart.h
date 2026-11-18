/*
 * ******************************************************************************************
 * File		: uart.h
 * Author	: GowinSemicoductor
 * Chip		: AE350_SOC
 * Function	: UART predefine
 * ******************************************************************************************
 */

#ifndef __UART_H__
#define __UART_H__


// Definitions -------------------------------------------------------------------------------

#define UART1_USED_IN_PRINTF  0		// UART1 used in printf(); otherwise UART2


// Declarations ------------------------------------------------------------------------------

extern void uart_init(unsigned int baudrate);		// Initializes UART
extern int uart_getc(void);							// Input a character by UART
extern void uart_putc(int c);						// Print a character by UART
extern int uart_puts(const char *s);				// Print a string by UART
extern int outbyte(int c);							// Overwrite function for printf()
extern void uart_log_start(void);					// printf into the ring from now on
extern int uart_log_pump(void);						// feed the FIFO from the ring; 1 = more left
extern unsigned int uart_log_take_dropped(void);	// characters dropped on a full ring since the last call
extern void uart_log_panic(void);					// drain the ring by spinning, output direct


#endif	/* __UART_H__ */
