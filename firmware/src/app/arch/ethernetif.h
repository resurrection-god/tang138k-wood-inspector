/*
 *********************************************************************************************
 * @file      ethernetif.c
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     Ethernet interface.
 *********************************************************************************************
 */

#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

#include "lwip/err.h"
#include "lwip/netif.h"

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */
#define NETIF_MTU								      ( 1500 )

#define NETIF_IN_TASK_STACK_SIZE			( 4096 )
#define NETIF_IN_TASK_PRIORITY			  ( 4 )

#define NETIF_OUT_TASK_STACK_SIZE			( 4096 )
#define NETIF_OUT_TASK_PRIORITY			  ( 3 )

/* USER CODE END 0 */


/* Exported functions ------------------------------------------------------- */
err_t ethernetif_init(struct netif *netif);

void ethernetif_input( void *argument );
void ethernetif_output( void *argument );
void ethernetif_notify_conn_changed(struct netif *netif);
/* RX watchdog counters (status word 34/35): stage-1 completes in the last 20 s and in all,
   drains, speed corrections. */
void eth_rx_health(uint32_t *recent, uint32_t *total, uint32_t *drain, uint32_t *speed);
/* Fast RX poll: the machine timer runs at 100 us, armed by the first lost RX interrupt (ERRORS 3.25). */
uint32_t eth_fast_armed_ms(void);

u32_t sys_jiffies(void);

/* RX watchdog stage-1 snapshots (21.10.2026, sick boots, ERRORS 3.7), console '!'. */
#define ETH_WD_EVENTS 8
typedef struct {
	uint32_t tick, ie, pending, mip, masked, irqs, late;   /* late: ms to the interrupt, FFFFFFFE drained */
} eth_wd_event_t;
extern eth_wd_event_t    g_eth_wd_ev[ETH_WD_EVENTS];
extern volatile uint32_t g_eth_wd_ev_n;

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
#endif
