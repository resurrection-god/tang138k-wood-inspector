/*
 *********************************************************************************************
 * @file      ethernetif.c
 * @author    GowinSemiconductor / User
 * @brief     Ethernet interface for Andes AE350 + Gowin Triple Speed Ethernet MAC IP.
 *********************************************************************************************
 */

/* Includes ------------------------------------------------------------ */
#include <string.h>
#include <stdio.h>
#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "ethernetif.h"
#include "ethernet.h"
#include "watchdog.h"

volatile uint32_t g_eth_rx_pbuf_cnt = 0;
volatile uint32_t g_eth_tx_cnt = 0;
volatile uint32_t g_eth_tx_oversize = 0;  // frames over the 1536 B window, refused (never seen)
// 30.09.2026: how many times the wait for the previous packet hit its limit (ETH_TX_EN
// did not clear; that must not happen; if it grows, polling ETH_TX_EN does not work in this
// MAC version). Printed by the console task when it grows.
volatile uint32_t g_eth_tx_busy_to   = 0;

/* Network interface name */
#define IFNAME0 'G'
#define IFNAME1 'W'

struct ethernetif {
	struct eth_addr *ethaddr;
/* Add whatever per-interface state that is needed here. */
};

xSemaphoreHandle s_xSemaphore = NULL;
QueueHandle_t xRxQueue = NULL;

sys_sem_t tx_sem = NULL;
sys_mbox_t eth_tx_mb = NULL;

static void low_level_init(struct netif *netif)
{ 
  //Init PHY
  eth_init(NULL,NULL,NULL);
  eth_set_mode(ETH_FULL_DUPLEX_1000M);
  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address (Unicast locally administered) */
  netif->hwaddr[0] = 0x00;
  netif->hwaddr[1] = 0x12;
  netif->hwaddr[2] = 0x34;
  netif->hwaddr[3] = 0x56;
  netif->hwaddr[4] = 0x78;
  netif->hwaddr[5] = 0x9A;

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

  /* The MAC exposes one shared RX register window, so only presence matters. */
  s_xSemaphore = xSemaphoreCreateBinary();
  xRxQueue = xQueueCreate(16, sizeof(struct pbuf *));
  sys_sem_new(&tx_sem , 0);
  sys_mbox_new(&eth_tx_mb , 50);

  /* create the task that handles the ETH_MAC */
	/* create the task that handles the ETH_MAC input packets */
	sys_thread_new("ETHIN",
	               ethernetif_input,
	               netif,
	               NETIF_IN_TASK_STACK_SIZE,
	               NETIF_IN_TASK_PRIORITY);
}

// Transmit (30.09.2026, modelled on the vendor driver ae350_lwip_raw_ping,
// ethernetif.c: `while (ETH_TX_EN);` before writing the next packet).
//
// The MAC clears ETH_TX_EN itself when the packet has gone onto the wire; it is
// the "transmitter busy" sign. So we wait for it AT THE START of the function,
// before writing into the transmit window, and after ETH_TX_EN = 1 return at
// once: lwIP prepares the next packet while this one goes onto the wire. Before
// that we waited AT THE END, blindly, with an estimate (len + 24) * 8 ns + 1.5 us
// (c0780b9; before it 15 us, 7307a6c, where polling ETH_TX_EN was replaced by a
// pause with no recorded reason), and all that time receive was disabled: TCP
// acknowledgements waited for the end of each transmission (ERRORS 5.9).
//
// Receive is disabled only while writing into the window 0x000..0x5FF (writes
// go to the transmit buffer, reads come from the receive buffer, the addresses
// are shared), as in the vendor driver. The wait limit is a safety net: a
// 1536 B packet leaves in ~12.5 us.
#define ETH_TX_BUSY_LIMIT_TICKS  3000U   /* 30 us in MTIME ticks (10 ns) */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
	(void)netif;
	uint32_t t_in = DEV_PLMT->MTIME[0];

	// The window holds 1536 B. A longer frame used to be cut and sent with a wrong length;
	// refuse it instead (unreachable at MSS 1460, OPEN_ISSUES 1.9).
	if (p->tot_len > 1536U)
	{
		g_eth_tx_oversize++;
		return ERR_IF;
	}

	// The previous packet is still on the wire: wait (there is one transmit window).
	if (DEV_ETHERNET->ETH_TX_EN & BIT0)
	{
		while ((DEV_ETHERNET->ETH_TX_EN & BIT0) &&
		       ((DEV_PLMT->MTIME[0] - t_in) < ETH_TX_BUSY_LIMIT_TICKS))
			;
		if (DEV_ETHERNET->ETH_TX_EN & BIT0) g_eth_tx_busy_to++;
	}

	DEV_ETHERNET->ETH_TX_IE = 0;
	DEV_ETHERNET->ETH_RX_IE = 0;

	static uint8_t s_tx_buffer[1536];
	uint32_t send_len = p->tot_len;           // <= 1536, checked above

	uint16_t offset = 0;
	for (struct pbuf *q = p; q != NULL && offset < send_len; q = q->next)
	{
		uint16_t copy_len = q->len;
		if (offset + copy_len > send_len) copy_len = send_len - offset;
		memcpy(s_tx_buffer + offset, q->payload, copy_len);
		offset += copy_len;
	}

	if (send_len < 60)
	{
		memset(s_tx_buffer + send_len, 0, 60 - send_len);
		send_len = 60;
	}

	DEV_ETHERNET->ETH_TX_LENGTH = send_len;

	uint32_t *tx_words = (uint32_t *)s_tx_buffer;
	uint32_t num_words = (send_len + 3) / 4;
	uint32_t i = 0;
	while (i + 4 <= num_words)
	{
		DEV_ETHERNET->ETH_TX_DATA[i]     = tx_words[i];
		DEV_ETHERNET->ETH_TX_DATA[i + 1] = tx_words[i + 1];
		DEV_ETHERNET->ETH_TX_DATA[i + 2] = tx_words[i + 2];
		DEV_ETHERNET->ETH_TX_DATA[i + 3] = tx_words[i + 3];
		i += 4;
	}
	while (i < num_words)
	{
		DEV_ETHERNET->ETH_TX_DATA[i] = tx_words[i];
		i++;
	}

	DEV_ETHERNET->ETH_TX_IC = BIT0;
	DEV_ETHERNET->ETH_TX_EN = BIT0;
	// Receive is enabled again at once: the packet goes onto the wire without us (see above).
	// Not while the ISR holds it masked for a frame ETHIN has not copied yet:
	// that frame would raise the interrupt again (ERRORS 3.15).
	taskENTER_CRITICAL();
	if (!g_eth_rx_masked) DEV_ETHERNET->ETH_RX_IE = 1;
	taskEXIT_CRITICAL();
	g_eth_tx_cnt++;
	return ERR_OK;
}

static struct pbuf * low_level_input(struct netif *netif)
{
	(void)netif;
	struct pbuf *p = NULL, *q = NULL;
	u16_t len = DEV_ETHERNET->ETH_RX_LENGTH & 0x07FF;

	if (len > 0 && len <= 1536)
	{
		/* One contiguous heap pbuf avoids RX pool freelist exhaustion under reconnect floods. */
		p = pbuf_alloc(PBUF_RAW, len, PBUF_RAM);
		if (p != NULL)
		{
			uint8_t *src = (uint8_t *)DEV_ETHERNET->ETH_RX_DATA;
			// Words rather than bytes (30.09.2026): every read of the MAC window is an AHB
			// transaction of ~125 ns, and byte by byte a 60 B TCP acknowledgement cost
			// ~7.5 us on the path that delays frame delivery (lwIP waits for these
			// acknowledgements, see send_frame_chunks). PBUF_RAM without ETH_PAD_SIZE is
			// one contiguous piece, its payload aligned to MEM_ALIGNMENT = 4; otherwise
			// the old byte-by-byte path.
			if (p->next == NULL && (((uint32_t)p->payload) & 3U) == 0U)
			{
				const volatile unsigned int *src32 = DEV_ETHERNET->ETH_RX_DATA;
				uint32_t *dst32 = (uint32_t *)p->payload;
				uint32_t nw = (uint32_t)len / 4U;
				uint32_t i;
				for (i = 0; i < nw; ++i) dst32[i] = src32[i];
				if (len & 3U)
				{
					uint32_t last = src32[nw];
					memcpy((uint8_t *)p->payload + nw * 4U, &last, len & 3U);
				}
			}
			else
			{
				uint16_t offset = 0;
				for (q = p; q != NULL && offset < len; q = q->next)
				{
					uint16_t copy_len = q->len;
					if (copy_len > (uint16_t)(len - offset)) {
						copy_len = (uint16_t)(len - offset);
					}
					for (uint16_t i = 0; i < copy_len; ++i) {
						((uint8_t *)q->payload)[i] = src[offset++];
					}
				}
			}

			g_eth_rx_pbuf_cnt++;
		}
	}

	// Acknowledge packet received and re-enable RX interrupt. With
	// ETH_RX_IC_AFTER_COPY this is the only RX_IC of a frame: it releases the
	// window to the next one only now, after the copy (ERRORS 3.15).
	DEV_ETHERNET->ETH_RX_IC = BIT0;
	taskENTER_CRITICAL();
	g_eth_rx_masked = 0;
	DEV_ETHERNET->ETH_RX_IE = 1;
	taskEXIT_CRITICAL();

	return p;
}

/* ---------------------------------------------------------------------------
 * RX watchdog
 *
 * Ethernet RX has been seen to stop for good while the core, lwIP and the link
 * stay healthy (ERRORS former 31.29; now ERRORS 3.7, 3.10). Snapshots over JTAG at the moment of failure
 * always show the MAC holding a received frame (RX_IS=1, RX_IE=1) while the
 * PLIC has source 4 claimed and never completed (pending=0, mip=0). Two manual
 * writes brought it back:
 *   - complete(IRQ_GP0_SOURCE) in the PLIC     (enough on place_option 3)
 *   - ETH_RX_IC = 1 in the MAC                  (also needed on place_option 2,
 *                                                where RX_IS had already dropped)
 * This watchdog does the same from ETHIN, which now waits for the semaphore with
 * a timeout instead of forever. It does not remove the cause - it keeps the
 * network alive while the cause is found, and its counters say how often and in
 * which form the lock-up happens.
 *
 *   stage 1  RX_IS=1, no RX interrupt since the last check (1 ms, ERRORS 3.25): PLIC pending(4)=0 ("lost")  -> complete(4);
 *            pending(4)=1 ("late") -> nothing; the check runs with interrupts off; counters WD cmp / late / maxc
 *   stage 2  still stuck on the next check                -> drain the frame here
 *   stage 3  RX_IS=0, no RX interrupt for ETH_RX_WD_IDLE_MS
 *            while we keep transmitting (lwIP retransmits
 *            into silence)                                 -> RX_IC + RX_IE + complete(4)
 *
 * Period 200 -> 2 ms (06.10.2026, ERRORS 3.10). On a "sick" boot (ERRORS 3.7:
 * about half of all boots of one bitstream, WD cmp growing from the start) the
 * RX interrupt is lost for nearly every frame, so every frame waited for this
 * watchdog: complete() after 400 ms, drain after 600 ms - the 1.5 s TCP stalls.
 * At 2 ms the same stages fire after 4 and 6 ms. A healthy boot is unaffected:
 * its interrupt arrives within microseconds and resets s_is_hits; the cost is
 * ETHIN waking 500 times a second. This does not cure the lost interrupts, it
 * makes them cheap. Polling RX without RX_IS is NOT an option: low_level_input()
 * takes the length from ETH_RX_LENGTH, which keeps the previous frame's length,
 * so it would hand lwIP the last frame again.
 * ------------------------------------------------------------------------- */
#define ETH_RX_WD_PERIOD_MS     1   /* 1 FreeRTOS tick = 1 ms (configTICK_RATE_HZ 1000); was 2 before 09.11.2026 (ERRORS 3.25) */
#ifndef ETH_WD_NOCOMPLETE_DIAG
#define ETH_WD_NOCOMPLETE_DIAG  0   /* 1: stage 1 records only, drain after 500 ms (ERRORS 3.16) */
#endif
#define ETH_RX_WD_IDLE_MS       3000
#define ETH_RX_WD_TX_MIN        3

extern volatile uint32_t g_eth_rx_cnt;
volatile uint32_t g_eth_wd_complete = 0;   /* "lost": RX_IS=1, no ISR, PLIC pending(4)=0 -> complete(4) written */
volatile uint32_t g_eth_wd_late     = 0;   /* "late": RX_IS=1, no ISR yet, pending(4)=1 -> nothing written */
volatile uint32_t g_eth_wd_max_cons = 0;   /* most lost interrupts in a row without an ISR in between */
volatile uint32_t g_eth_wd_drain    = 0;
volatile uint32_t g_eth_wd_rxic     = 0;
volatile uint32_t g_eth_wd_speed    = 0;   /* stage 4: ETH_MODE corrected to the link speed */
#define ETH_RX_WD_SPEED_MS      10000
extern volatile uint32_t g_gp0_irq_cnt;
eth_wd_event_t    g_eth_wd_ev[ETH_WD_EVENTS];
volatile uint32_t g_eth_wd_ev_n = 0;

/* ---------------------------------------------------------------------------
 * Fast RX poll (09.11.2026, ERRORS 3.25)
 *
 * On a sick boot the RX interrupt is lost on every frame, and the 1 ms watchdog makes each frame wait up to
 * 1 ms (the MAC holds one frame: ping 8-13 of 20 under flood). The first "lost" of the watchdog arms a 100 us
 * poll. The PIT of this SoC does not take writes (CTRL/RELOAD read back 0, 09.11.2026), so the poll rides on
 * the machine timer of the FreeRTOS tick: while armed the timer compare is stepped by 100 us and portasm.S
 * calls eth_tick_hook() instead of xTaskIncrementTick(); the hook calls xTaskIncrementTick() every 10th time.
 * Each hook while armed does the watchdog's check: RX_IS = 1 and the ISR counter has not moved since the last
 * hook -> PLIC pending(4) = 0 means lost -> complete(4) (the line is still high, so the interrupt follows at
 * once); pending(4) = 1 is late, nothing is written. The timer interrupt and the PLIC interrupt of source 4 are
 * different traps on one core, so the ISR of source 4 is not running while this code reads pending(4) and no
 * claim of source 4 is active. After 1 s without a lost the step goes back to 1 ms. No frame is copied here.
 * ------------------------------------------------------------------------- */
#define ETH_FAST_STEP_CYC       100000U       /* normal tick: 1 ms at 100 MHz */
#define ETH_FAST_STEP_ARMED     10000U        /* 100 us */
#define ETH_FAST_HOLD_MTICKS    100000000U    /* disarm after 1 s without a lost (MTIME, 100 MHz) */
extern size_t uxTimerIncrementsForOneTick;    /* port.c */
extern BaseType_t xTaskIncrementTick(void);
volatile uint32_t g_eth_fast_armed = 0;
volatile uint32_t g_eth_fast_lost  = 0;
volatile uint32_t g_eth_fast_late  = 0;
volatile uint32_t g_eth_fast_arms  = 0;
volatile uint32_t g_eth_fast_ticks = 0;
volatile uint32_t g_eth_fast_ms    = 0;       /* armed time, completed arm periods */
volatile uint32_t g_eth_fast_cyc   = 0;       /* cycles inside eth_tick_hook while armed (without the trap entry and exit) */
static uint32_t   s_fast_arm_t, s_fast_lost_t, s_fast_last_isr, s_fast_sub;

uint32_t eth_fast_armed_ms(void)
{
	uint32_t ms = g_eth_fast_ms;
	if (g_eth_fast_armed) ms += (DEV_PLMT->MTIME[0] - s_fast_arm_t) / 100000U;
	return ms;
}

static void eth_fast_poll_arm(void)
{
	taskENTER_CRITICAL();
	if (g_eth_fast_armed)
	{
		s_fast_lost_t = DEV_PLMT->MTIME[0];
	}
	else
	{
		s_fast_last_isr = g_gp0_irq_cnt;
		s_fast_sub = 0;
		s_fast_arm_t = s_fast_lost_t = DEV_PLMT->MTIME[0];
		g_eth_fast_armed = 1;
		g_eth_fast_arms++;
		uxTimerIncrementsForOneTick = ETH_FAST_STEP_ARMED;   /* from the next compare on */
	}
	taskEXIT_CRITICAL();
}

BaseType_t eth_tick_hook(void)
{
	if (!g_eth_fast_armed)
		return xTaskIncrementTick();

	uint32_t c0   = (uint32_t)read_csr(NDS_MCYCLE);
	uint32_t now  = DEV_PLMT->MTIME[0];
	uint32_t isr  = g_gp0_irq_cnt;
	BaseType_t sw = 0;
	g_eth_fast_ticks++;
	if (DEV_ETHERNET->ETH_RX_IS && isr == s_fast_last_isr)
	{
		if (!((((volatile uint32_t *)NDS_PLIC_BASE)[0x1000 / 4] >> IRQ_GP0_SOURCE) & 1U))
		{
			__nds__plic_complete_interrupt(IRQ_GP0_SOURCE);
			g_eth_fast_lost++;
			s_fast_lost_t = now;
		}
		else
		{
			g_eth_fast_late++;
		}
	}
	s_fast_last_isr = isr;
	if (++s_fast_sub >= ETH_FAST_STEP_CYC / ETH_FAST_STEP_ARMED)
	{
		s_fast_sub = 0;
		sw = xTaskIncrementTick();
		if ((uint32_t)(now - s_fast_lost_t) > ETH_FAST_HOLD_MTICKS)
		{
			g_eth_fast_ms += (now - s_fast_arm_t) / 100000U;
			g_eth_fast_armed = 0;
			uxTimerIncrementsForOneTick = ETH_FAST_STEP_CYC;
		}
	}
	g_eth_fast_cyc += (uint32_t)read_csr(NDS_MCYCLE) - c0;
	return sw;
}
/* Stage-1 completes per second for the last ETH_SICK_WINDOW_S seconds, so the PC can see a
   sick boot (OPEN_ISSUES 1.1, status word 34). A healthy board after a day reads ~1 per 46 s
   (ERRORS 3.15), a sick one 1–7 per 20 s even when idle (3.16). */
#define ETH_SICK_WINDOW_S       20U
static uint32_t   s_sick_hist[ETH_SICK_WINDOW_S];
static uint32_t   s_sick_pos;

void eth_rx_health(uint32_t *recent, uint32_t *total, uint32_t *drain, uint32_t *speed)
{
	*recent = g_eth_wd_complete - s_sick_hist[(s_sick_pos + 1U) % ETH_SICK_WINDOW_S];   /* the oldest second */
	*total  = g_eth_wd_complete;
	*drain  = g_eth_wd_drain;
	*speed  = g_eth_wd_speed;
}

static void eth_rx_watchdog(struct netif *netif)
{
	static uint32_t   s_last_rx    = 0;
	static uint32_t   s_tx_at_rx   = 0;
	static TickType_t s_rx_tick    = 0;
	static uint32_t   s_is_hits    = 0;
	static uint32_t   s_cons       = 0;   /* lost interrupts in a row, no ISR in between */
	static uint32_t   s_pend0      = 0;
	static TickType_t s_speed_tick = 0;
	static TickType_t s_sick_tick  = 0;
	TickType_t now = xTaskGetTickCount();

	if (now - s_sick_tick >= pdMS_TO_TICKS(1000))
	{
		s_sick_tick = now;
		s_sick_pos = (s_sick_pos + 1U) % ETH_SICK_WINDOW_S;
		s_sick_hist[s_sick_pos] = g_eth_wd_complete;
	}

	if (g_eth_rx_cnt != s_last_rx)
	{
		/* The interrupt came after a stage-1 snapshot: how late (ms), by itself or
		   after the complete below (ETH_WD_NOCOMPLETE_DIAG). */
		if (g_eth_wd_ev_n != 0U && g_eth_wd_ev[g_eth_wd_ev_n - 1U].late == 0xFFFFFFFFU)
			g_eth_wd_ev[g_eth_wd_ev_n - 1U].late = (uint32_t)(now - g_eth_wd_ev[g_eth_wd_ev_n - 1U].tick);
		s_last_rx  = g_eth_rx_cnt;
		s_tx_at_rx = g_eth_tx_cnt;
		s_rx_tick  = now;
		s_speed_tick = now;
		s_is_hits  = 0;
		s_cons     = 0;
		return;
	}

	if (DEV_ETHERNET->ETH_RX_IS)
	{
		s_is_hits++;
		if (s_is_hits <= 2)
		{
			/* 09.11.2026 (ERRORS 3.25): a frame waits (RX_IS=1) and no RX interrupt has come since
			   the previous check, 1 ms ago. Look at the PLIC with interrupts off, so the decision
			   cannot race the ISR on this core: pending(4)=0 is the stuck gateway of ERRORS 3.25
			   ("lost": complete(4) re-arms it, the line is still high, so the interrupt follows at
			   once); pending(4)=1 is an interrupt on its way ("late": nothing is written). */
			uint32_t pend;
			taskENTER_CRITICAL();
			pend = (((volatile uint32_t *)NDS_PLIC_BASE)[0x1000 / 4] >> IRQ_GP0_SOURCE) & 1U;
#if !ETH_WD_NOCOMPLETE_DIAG
			if (!pend) __nds__plic_complete_interrupt(IRQ_GP0_SOURCE);
#endif
			taskEXIT_CRITICAL();
			s_pend0 = pend;
			if (!pend)
			{
				g_eth_wd_complete++;                       /* lost */
				eth_fast_poll_arm();
				if (++s_cons > g_eth_wd_max_cons) g_eth_wd_max_cons = s_cons;
			}
			else if (s_is_hits == 1)
			{
				g_eth_wd_late++;                           /* late */
			}
		}
		if (s_is_hits == 1)
		{
			/* What the hardware holds at this moment (21.10.2026, ERRORS 3.7): is the
			   line seen by the PLIC (pending), delivered to the core (mip.MEIP), masked? */
			if (g_eth_wd_ev_n < ETH_WD_EVENTS)
			{
				uint32_t i = g_eth_wd_ev_n++;
				g_eth_wd_ev[i].tick    = (uint32_t)now;
				g_eth_wd_ev[i].ie      = DEV_ETHERNET->ETH_RX_IE;
				g_eth_wd_ev[i].pending = s_pend0;       /* before the complete above */
				g_eth_wd_ev[i].mip     = (uint32_t)read_csr(NDS_MIP);
				g_eth_wd_ev[i].masked  = g_eth_rx_masked;
				g_eth_wd_ev[i].irqs    = g_gp0_irq_cnt;
				g_eth_wd_ev[i].late    = 0xFFFFFFFFU;
			}
#if ETH_WD_NOCOMPLETE_DIAG
			/* Diagnostic build: no complete - does the interrupt come by itself? */
			return;
#endif
		}
		else if (s_is_hits >= (ETH_WD_NOCOMPLETE_DIAG ? 250U : 3U))   /* diag: 500 ms */
		{
			if (g_eth_wd_ev_n != 0U && g_eth_wd_ev[g_eth_wd_ev_n - 1U].late == 0xFFFFFFFFU)
				g_eth_wd_ev[g_eth_wd_ev_n - 1U].late = 0xFFFFFFFEU;      /* drained, never came */
			/* complete() did not release it: take the frame from task context,
			   exactly as the ISR-woken path would. */
			struct pbuf *p = low_level_input(netif);
			if (p != NULL && netif->input(p, netif) != ERR_OK)
			{
				pbuf_free(p);
			}
			g_eth_wd_drain++;
			s_is_hits = 0;
		}
		return;
	}
	s_is_hits = 0;

	if ((now - s_rx_tick) >= pdMS_TO_TICKS(ETH_RX_WD_IDLE_MS) &&
	    (g_eth_tx_cnt - s_tx_at_rx) >= ETH_RX_WD_TX_MIN)
	{
		/* Nothing received for seconds while we keep sending: the MAC may be
		   holding an unacknowledged frame with RX_IS already cleared. */
		DEV_ETHERNET->ETH_RX_IC = BIT0;
		g_eth_rx_masked = 0;
		DEV_ETHERNET->ETH_RX_IE = 1;
		__nds__plic_complete_interrupt(IRQ_GP0_SOURCE);
		g_eth_wd_rxic++;
		s_rx_tick  = now;
		s_tx_at_rx = g_eth_tx_cnt;
	}

	/* Stage 4 (15.10.2026, ERRORS 3.15): nothing received for 10 s - check that
	   the MAC runs at the link speed. A link that came back at 100M while
	   ETH_MODE stays 1000M (or the reverse) silences receive completely, with
	   RX_IS 0 and no interrupt - the "RX freeze" signature of ERRORS 3.7 - and
	   following PHYSR cures it without an FPGA reboot. One MIIM read per 10 s of
	   silence only: polling the PHY all the time makes boots sick (ERRORS 3.10). */
	if ((now - s_speed_tick) >= pdMS_TO_TICKS(ETH_RX_WD_SPEED_MS))
	{
		uint32_t mode = DEV_ETHERNET->ETH_MODE;
		uint16_t sp = eth_mac_follow_phy(0);
		if (DEV_ETHERNET->ETH_MODE != mode)
		{
			g_eth_wd_speed++;
			printf("[ETH WD] receive silent 10 s: MAC %u -> link %u Mbit/s (ETH_MODE %u -> %u)\r\n",
			       (unsigned)(mode == ETH_FULL_DUPLEX_1000M ? 1000 : (mode == ETH_FULL_DUPLEX_100M ? 100 : 10)),
			       (unsigned)sp, (unsigned)mode, (unsigned)DEV_ETHERNET->ETH_MODE);
		}
		s_speed_tick = now;
	}
}

void ethernetif_input(void *pParams)
{
	struct netif *netif = (struct netif *)pParams;
	struct pbuf *p = NULL;

	while (1)
	{
		watchdog_beat(WD_TASK_ETHIN);
		if (xSemaphoreTake(s_xSemaphore, pdMS_TO_TICKS(ETH_RX_WD_PERIOD_MS)) != pdTRUE)
		{
			eth_rx_watchdog(netif);
		}
		else
		{
TRY_GET_NEXT_PACKET:
			p = low_level_input(netif);
			if (p != NULL)
			{
				err_t err = netif->input(p, netif);
				if (err != ERR_OK)
				{
					pbuf_free(p);
					p = NULL;
				}
				else
				{
					// If another packet arrived while processing this one, drain it immediately!
					if (xSemaphoreTake(s_xSemaphore, 0) == pdTRUE)
					{
						goto TRY_GET_NEXT_PACKET;
					}
				}
			}
		}
	}
}

err_t ethernetif_init(struct netif *netif)
{
	struct ethernetif *ethernetif = mem_malloc(sizeof(struct ethernetif));
	if (ethernetif == NULL)
	{
		printf("ethernetif_init: out of memory\r\n");
		return ERR_MEM;
	}
  
	LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
	netif->hostname = "lwip";
#endif
	netif->state = ethernetif;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;

#if LWIP_IPV4
#if LWIP_ARP
	netif->output = etharp_output;
#endif
#endif

#if LWIP_IPV6
	netif->output_ip6 = ethip6_output;
#endif

	netif->linkoutput = low_level_output;

	/* initialize the hardware */
	low_level_init(netif);
	ethernetif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);
	etharp_init();
	return ERR_OK;
}

#if LWIP_NETIF_LINK_CALLBACK
#endif
