/*
 * ******************************************************************************************
 * File		: uart.c
 * Author	: GowinSemicoductor
 * Chip		: AE350_SOC
 * Function	: UART predefine
 * ******************************************************************************************
 */

// Includes ---------------------------------------------------------------------------------
#include "uart.h"
#include "platform.h"
#include "FreeRTOS.h"
#include "task.h"


// Definitions ------------------------------------------------------------------------------

// Define UART1 or UART2 used in printf()
#if UART1_USED_IN_PRINTF
#define	DEV_UART	DEV_UART1	// UART1
#else
#define	DEV_UART	DEV_UART2	// UART2
#endif

// Baud rate computed
#define BAUD_RATE(n)            ((UCLKFREQ + 8 * (n)) / (16 * (n)))

// Initializes UART
void uart_init(unsigned int baudrate)
{
	/* Set DLAB to 1 */
	DEV_UART->LCR |= 0x80;

	/* Set DLL for baud rate */
	DEV_UART->DLL = (BAUD_RATE(baudrate) >> 0) & 0xff;
	DEV_UART->DLM = (BAUD_RATE(baudrate) >> 8) & 0xff;

	/* LCR: length 8, no parity, 1 stop bit. */
	DEV_UART->LCR = 0x03;

	/* FCR: enable FIFO, reset TX and RX. */
	DEV_UART->FCR = 0x07;
}

// Input a character by UART
int uart_getc(void)
{
	#define SERIAL_LSR_RDR 0x01

	while ((DEV_UART->LSR & SERIAL_LSR_RDR) == 0);

	return DEV_UART->RBR;
}

// Print a character by UART
void uart_putc(int c)
{
	#define SERIAL_LSR_THRE 0x20

	while ((DEV_UART->LSR & SERIAL_LSR_THRE) == 0);

	DEV_UART->THR = c;
}

// Print a string by UART
int uart_puts(const char *s)
{
	int len = 0;

	while (*s)
	{
		uart_putc(*s);

		if (*s == '\n')
		{
			uart_putc('\r');
		}

		s++;
		len++;
	}

	return len;
}

// printf log ring (OPEN_ISSUES 1.5, ERRORS 3.18). uart_putc waits ~87 us per character at
// 115200, so a 150-character line held the printing task for 13 ms. Once uart_log_start()
// is called, outbyte only copies into the ring and a low-priority task (uart_log_pump)
// feeds the UART FIFO; a full ring drops characters and counts them instead of blocking.
// Fatal paths call uart_log_panic(): the ring is drained by spinning and output is direct.
#define LOG_RING_SIZE		8192U		// a power of two
#define UART_FIFO_DEPTH		16U

static char s_log[LOG_RING_SIZE];
static volatile unsigned int s_log_head, s_log_tail;	// head: writers, tail: the pump
static volatile unsigned int s_log_dropped;
static volatile int s_log_on;

static void log_push(int c)
{
	for (;;) {
		// Interrupts off, not a FreeRTOS critical section: printf may run in an ISR.
		unsigned long m = clear_csr(NDS_MSTATUS, MSTATUS_MIE);
		int full = s_log_head - s_log_tail >= LOG_RING_SIZE;

		if (!full)
			s_log[s_log_head++ & (LOG_RING_SIZE - 1U)] = (char)c;
		if (m & MSTATUS_MIE) set_csr(NDS_MSTATUS, MSTATUS_MIE);
		if (!full) return;
		// Full: a task waits for the pump (a long console dump keeps every line), an ISR or
		// code with interrupts off drops the character and counts it.
		if (!(m & MSTATUS_MIE) || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
			s_log_dropped++;
			return;
		}
		vTaskDelay(1);
	}
}

void uart_log_start(void)
{
	s_log_on = 1;
}

int uart_log_pump(void)
{
	unsigned int n;

	if ((DEV_UART->LSR & SERIAL_LSR_THRE) == 0) return 1;	// FIFO not empty yet
	for (n = 0; n < UART_FIFO_DEPTH && s_log_tail != s_log_head; ++n)
		DEV_UART->THR = s_log[s_log_tail++ & (LOG_RING_SIZE - 1U)];
	return s_log_tail != s_log_head;
}

unsigned int uart_log_take_dropped(void)
{
	unsigned long m = clear_csr(NDS_MSTATUS, MSTATUS_MIE);
	unsigned int d = s_log_dropped;

	s_log_dropped = 0;
	if (m & MSTATUS_MIE) set_csr(NDS_MSTATUS, MSTATUS_MIE);
	return d;
}

void uart_log_panic(void)
{
	s_log_on = 0;
	while (s_log_tail != s_log_head) {
		uart_putc(s_log[s_log_tail & (LOG_RING_SIZE - 1U)]);
		s_log_tail++;
	}
}

// Overwrite function for printf()
int outbyte(int c)
{
	if (s_log_on)
	{
		log_push(c);
		if (c == '\n') log_push('\r');
		return c;
	}
	uart_putc(c);

	if (c =='\n')
	{
		uart_putc('\r');
	}

	return c;
}
