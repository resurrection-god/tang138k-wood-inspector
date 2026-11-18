/*
 * ******************************************************************************************
 * File		: ethernet.c
 * Author	: GowinSemicoductor
 * Chip		: AE350_SOC
 * Function	: Triple mode Ethernet driver definitions
 * ******************************************************************************************
 * */

#include "ethernet.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>          // eth_phy_init() always prints, not only with DEBUG_PRINT


// Definitions ------------------------------------------------------------------------------

// Ethernet information
// Serialises transmission through the MAC (declared in ethernet.h; lived in
// net_stack.c until 24.09.2026).
SemaphoreHandle_t xEthTxMutex = NULL;

static ETHERNET_INFO ethernet_info = {NULL, NULL, NULL};


// Ethernet receive interrupt event
void rx_int_event(void)
{
	uint8_t *rx_message = ethernet_info.rx_ptr;
	uint32_t *rx_data_length = ethernet_info.rx_data_length;

	if(rx_message == NULL)
	{
		return;
	}

	uint32_t rx_length = DEV_ETHERNET->ETH_RX_LENGTH;
	*rx_data_length = rx_length;
	uint8_t *rx_buffer = (uint8_t *)(DEV_ETHERNET->ETH_RX_DATA);

	for(int i = 0; i < rx_length; i++)
	{
		rx_message[i] = rx_buffer[i];
	}

	DEV_ETHERNET->ETH_RX_IC = BIT0;
}

// Initializes Ethernet
int32_t eth_init(uint8_t* rx_ptr, uint32_t *rx_data_length, uint32_t *miim_rd_ptr)
{
	ethernet_info.rx_ptr = rx_ptr;
	ethernet_info.rx_data_length = rx_data_length;
	ethernet_info.miim_rd_ptr = miim_rd_ptr;

	DEV_ETHERNET->ETH_TX_IE = 0;
	DEV_ETHERNET->ETH_RX_IE = 1; // Enable RX interrupt for incoming packets
	DEV_ETHERNET->MIIM_IE = 0;

	return AE350_DRIVER_OK;
}

// Transmit message
int32_t eth_tx(uint8_t *tx_message, uint32_t tx_length)
{
	uint8_t *tx_buffer = (uint8_t *)(DEV_ETHERNET->ETH_TX_DATA);

	if(tx_message == NULL)
	{
		return AE350_DRIVER_ERROR_PARAMETER;
	}

	for(int i = 0; i < tx_length; i++)
	{
		tx_buffer[i] = tx_message[i];
	}

	DEV_ETHERNET->ETH_TX_LENGTH = tx_length;
	DEV_ETHERNET->ETH_TX_IC = BIT0;
	DEV_ETHERNET->ETH_TX_EN = BIT0;

	while(DEV_ETHERNET->ETH_TX_EN);

	return AE350_DRIVER_OK;
}

// MIIM write
void miim_write(uint8_t miim_phy_address, uint8_t miim_reg_addr, uint16_t miim_wr_data)
{
	DEV_ETHERNET->MIIM_OP_MODE = 1;
	DEV_ETHERNET->MIIM_PHY_ADDR = miim_phy_address;
	DEV_ETHERNET->MIIM_REG_ADDR = miim_reg_addr;
	DEV_ETHERNET->MIIM_WR_DATA = miim_wr_data;
	DEV_ETHERNET->MIIM_OP_EN = BIT0;

	while(DEV_ETHERNET->MIIM_OP_EN);
}

// MIIM receive
void miim_receive(uint8_t miim_phy_address, uint8_t miim_reg_addr)
{
	DEV_ETHERNET->MIIM_OP_MODE = 0;
	DEV_ETHERNET->MIIM_PHY_ADDR = miim_phy_address;
	DEV_ETHERNET->MIIM_REG_ADDR = miim_reg_addr;
	DEV_ETHERNET->MIIM_OP_EN = BIT0;

	while(DEV_ETHERNET->MIIM_OP_EN);
}

// Set working mode
void eth_set_mode(uint32_t eth_mode)
{
	DEV_ETHERNET->ETH_MODE = eth_mode;
}

// Ethernet Driver structure
AE350_DRIVER_ETHERNET Driver_ETHERNET =
{
	eth_init,
	eth_tx,
	miim_write,
	miim_receive,
	eth_set_mode
};

volatile uint32_t g_eth_rx_cnt = 0;
// Every entry into this handler, whether or not RX_IS was set. Differs from
// g_eth_rx_cnt by TX-only and spurious entries (see g_plic_* in interrupt.c).
volatile uint32_t g_gp0_irq_cnt = 0;
volatile uint32_t g_eth_rx_masked = 0;
extern xSemaphoreHandle s_xSemaphore;

// Ethernet interrupt handler : gp0_irq_handler
void gp0_irq_handler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	g_gp0_irq_cnt++;

#if ETH_RX_IC_AFTER_COPY
	if (DEV_ETHERNET->ETH_RX_IS && DEV_ETHERNET->ETH_RX_IE)
	{
		g_eth_rx_cnt++;
		// Mask, do not acknowledge: RX_IC releases the window to the next frame,
		// so it is written by low_level_input() after the copy (ERRORS 3.15).
		DEV_ETHERNET->ETH_RX_IE = 0;
		g_eth_rx_masked = 1;
		// Read it back before mext_interrupt() writes complete(4): the store to the MAC
		// is posted, and a complete reaching the PLIC while the line is still high lets
		// the level gateway forward a request that the line then withdraws - the
		// gateway then waits for a completion nobody sends, and the next frames are
		// never signalled (sick boots, ERRORS 3.7; watchdog snapshot: RX_IS 1, RX_IE 1,
		// pending 0, claims = completes, cured by the watchdog's complete). 21.10.2026.
		// 🪦 Did not cure it on its own: 3 sick of 5 in a sick hour (ERRORS 3.16), nor did a
		// 2 us wait for the line itself (the removed ETH_IRQ_SETTLE_TICKS experiment).
		(void)DEV_ETHERNET->ETH_RX_IE;
		__asm volatile("fence iorw, iorw" ::: "memory");   // loads are non-blocking (MMISC_CTL bit 8)
#else
	if (DEV_ETHERNET->ETH_RX_IS)
	{
		g_eth_rx_cnt++;
		DEV_ETHERNET->ETH_RX_IC = BIT0;
#endif
		if (s_xSemaphore != NULL && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
		{
			xSemaphoreGiveFromISR(s_xSemaphore, &xHigherPriorityTaskWoken);
		}
	}

	if (DEV_ETHERNET->ETH_TX_IS)
	{
		DEV_ETHERNET->ETH_TX_IC = BIT0;
	}

	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


// ---- Realtek RTL8211F PHY over MIIM (main.c, phy_init_config until 24.09.2026;
// until 06.10.2026 the code took it for a Motorcomm YT8531C, see eth_phy_init) ----

static void eth_phy_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t val)
{
	DEV_ETHERNET->MIIM_IC = 0x03;
	DEV_ETHERNET->MIIM_IE = 0;
	DEV_ETHERNET->MIIM_OP_MODE = 1; // Write
	DEV_ETHERNET->MIIM_PHY_ADDR = phy_addr;
	DEV_ETHERNET->MIIM_REG_ADDR = reg_addr;
	DEV_ETHERNET->MIIM_WR_DATA = val;
	DEV_ETHERNET->MIIM_OP_EN = 1;
	
	for (volatile int i = 0; i < 50000; i++)
	{
		if (DEV_ETHERNET->MIIM_IS & 0x03)
			break;
	}
	DEV_ETHERNET->MIIM_IC = 0x03;
}

// Read a PHY register over MIIM (05.10.2026, RGMII delay experiment,
// ERRORS 3.10; docs/OPEN_ISSUES.md 1.1). 0xFFFF: the PHY did not answer.
static uint16_t eth_phy_read(uint8_t phy_addr, uint8_t reg_addr)
{
	uint16_t v = 0xFFFF;

	DEV_ETHERNET->MIIM_IC = 0x03;
	DEV_ETHERNET->MIIM_IE = 0;
	DEV_ETHERNET->MIIM_OP_MODE = 0; // Read
	DEV_ETHERNET->MIIM_PHY_ADDR = phy_addr;
	DEV_ETHERNET->MIIM_REG_ADDR = reg_addr;
	DEV_ETHERNET->MIIM_OP_EN = 1;

	for (volatile int i = 0; i < 50000; i++)
	{
		if (DEV_ETHERNET->MIIM_IS & MIIM_RD_DATA_VALID_IS)
		{
			v = (uint16_t)(DEV_ETHERNET->MIIM_RD_DATA & 0xFFFFU);
			break;
		}
	}
	DEV_ETHERNET->MIIM_IC = 0x03;
	return v;
}

uint16_t eth_phy_reg_read(uint8_t phy_addr, uint8_t reg_addr)
{
	return eth_phy_read(phy_addr, reg_addr);
}

void eth_phy_reg_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t val)
{
	eth_phy_write(phy_addr, reg_addr, val);
}

// MMD access per IEEE 802.3 clause 22 annex 22D: reg 13 = function | devad,
// reg 14 = address, then data (RTL8211F datasheet: MACR 13, MAADR 14).
uint16_t eth_phy_mmd_read(uint8_t phy_addr, uint8_t devad, uint16_t reg)
{
	eth_phy_write(phy_addr, 13, devad & 0x1FU);
	eth_phy_write(phy_addr, 14, reg);
	eth_phy_write(phy_addr, 13, 0x4000U | (devad & 0x1FU));
	return eth_phy_read(phy_addr, 14);
}

void eth_phy_mmd_write(uint8_t phy_addr, uint8_t devad, uint16_t reg, uint16_t val)
{
	eth_phy_write(phy_addr, 13, devad & 0x1FU);
	eth_phy_write(phy_addr, 14, reg);
	eth_phy_write(phy_addr, 13, 0x4000U | (devad & 0x1FU));
	eth_phy_write(phy_addr, 14, val);
}

uint16_t eth_mac_follow_phy(uint8_t phy_addr)
{
	uint16_t physr = eth_phy_read(phy_addr, ETH_PHY_PHYSR);
	uint32_t mode;
	uint16_t sp;
	bool fd;

	if (physr == 0xFFFF || !(physr & (1U << 2))) return 0;
	fd = (physr & (1U << 3)) != 0;
	switch ((physr >> 4) & 3U)
	{
	case 2:  mode = ETH_FULL_DUPLEX_1000M; sp = 1000; break;
	case 1:  mode = fd ? ETH_FULL_DUPLEX_100M : ETH_HALF_DUPLEX_100M; sp = 100; break;
	case 0:  mode = fd ? ETH_FULL_DUPLEX_10M : ETH_HALF_DUPLEX_10M; sp = 10; break;
	default: return 0;
	}
	if (DEV_ETHERNET->ETH_MODE != mode) eth_set_mode(mode);   // written only on a change
	return sp;
}

// Register on page `page` (RTL8211F: the page is in 0x1F, PAGSR); after the
// access the page goes back to 0, as all other accesses expect.
uint16_t eth_phy_paged_read(uint8_t phy_addr, uint16_t page, uint8_t reg_addr)
{
	uint16_t v;

	eth_phy_write(phy_addr, ETH_PHY_PAGSR, page);
	v = eth_phy_read(phy_addr, reg_addr);
	eth_phy_write(phy_addr, ETH_PHY_PAGSR, 0);
	return v;
}

void eth_phy_paged_write(uint8_t phy_addr, uint16_t page, uint8_t reg_addr, uint16_t val)
{
	eth_phy_write(phy_addr, ETH_PHY_PAGSR, page);
	eth_phy_write(phy_addr, reg_addr, val);
	eth_phy_write(phy_addr, ETH_PHY_PAGSR, 0);
}

bool eth_phy_link_status(uint8_t phy_addr, eth_phy_link_t *st)
{
	uint16_t bmsr = eth_phy_read(phy_addr, 1);      /* bit 2 latches at 0 on a link drop */
	uint16_t physr;

	if (bmsr == 0xFFFF) return false;
	physr = eth_phy_read(phy_addr, ETH_PHY_PHYSR);
	if (physr == 0xFFFF) return false;
	st->dropped = (bmsr & (1U << 2)) == 0;
	st->link = (physr & (1U << 2)) != 0;
	st->full_duplex = (physr & (1U << 3)) != 0;
	switch ((physr >> 4) & 3U)
	{
	case 0:  st->speed_mbps = 10;   break;
	case 1:  st->speed_mbps = 100;  break;
	case 2:  st->speed_mbps = 1000; break;
	default: st->speed_mbps = 0;    break;
	}
	return true;
}

// The PHY on the board is a Realtek RTL8211FI-CG (U10 on the Dock schematic,
// sheet 17; ID 0x001C:C916 is read from registers 2/3; datasheet in
// docs/hardware/RTL8211F-CG-Datasheet.pdf). Until 06.10.2026 this held a setup
// for a Motorcomm YT8531C: writing 0xA000 to 0x1E and 0x002B to 0x1F "enabled
// the 2 ns RX+TX delay". On the RTL8211F 0x1F is the page select (PAGSR), and
// the write left the PHY on page 0x2B: it did not touch the delays at all (they
// are set by the RXD0/RXDLY and RXD1/TXDLY straps), and reads of registers
// 16..30 came from the wrong page (ERRORS 3.10).
// The delays are still not changed here, only printed: they may be changed
// only after a measurement (console 'H'/'J').
void eth_phy_init(uint8_t phy_addr)
{
	uint16_t id1, id2, txd, rxd;

#if ETH_PHY_EXP == 2
	(void)id1; (void)id2; (void)txd; (void)rxd;
	printf("[PHY] ETH_PHY_EXP 2: no PHY init, PHY on its straps, MAC 1000M FD\r\n");
	return;
#endif
	printf("[PHY] ETH_PHY_EXP %u\r\n", (unsigned)ETH_PHY_EXP);
	printf("[PHY] Initializing Realtek RTL8211F: 1Gbps RGMII at addr 0x%02X...\r\n", phy_addr);

	// 1. Soft Reset PHY (BMCR bit 15)
	eth_phy_write(phy_addr, 0, 0x9140);
	vTaskDelay(pdMS_TO_TICKS(100));

	// 2. Page 0 (both after a reset and after earlier firmware), ID and RGMII delays.
	eth_phy_write(phy_addr, ETH_PHY_PAGSR, 0);
	id1 = eth_phy_read(phy_addr, 2);
	id2 = eth_phy_read(phy_addr, 3);
	txd = eth_phy_paged_read(phy_addr, ETH_PHY_PAGE_DELAY, ETH_PHY_REG_TXDLY);
	rxd = eth_phy_paged_read(phy_addr, ETH_PHY_PAGE_DELAY, ETH_PHY_REG_RXDLY);
	printf("[PHY] ID 0x%04X:%04X%s | RGMII delay: TX %s (0x%04X), RX %s (0x%04X) — straps, not changed\r\n",
	       id1, id2, (id1 == 0x001C && id2 == 0xC916) ? " (RTL8211F)" : " (NOT RTL8211F!)",
	       (txd & ETH_PHY_TXDLY_BIT) ? "+2ns" : "off", txd,
	       (rxd & ETH_PHY_RXDLY_BIT) ? "+2ns" : "off", rxd);

	// 3. Advertise 10/100 capabilities (ANAR reg 4)
	eth_phy_write(phy_addr, 4, 0x01E1); // 10/100 Full+Half Duplex + 802.3

	// 4. Advertise 1Gbps Full Duplex with automatic Master/Slave resolution (IEEE standard 0x0200)
#if ETH_PHY_EXP == 3
	eth_phy_write(phy_addr, 9, 0x1A00);     // manual master/slave, master
#elif ETH_PHY_EXP == 4
	eth_phy_write(phy_addr, 9, 0x1200);     // manual master/slave, slave
#else
	eth_phy_write(phy_addr, 9, 0x0200);
#endif
	// EEE (802.3az) advertisement off (owner, 15.10.2026, R-36; ERRORS 3.15): the
	// RTL8211F advertises it by default, the PC NIC accepts, and the link enters
	// LPI within seconds; power saving is worth nothing here, the Gowin MAC does
	// not claim EEE support, and the PHY counted EEE wake errors.
	// ETH_PHY_EXP 1 keeps the PHY default (EEE on) for A/B runs.
#if ETH_PHY_EXP != 1
	eth_phy_mmd_write(phy_addr, 7, 60, 0);
#endif

	// 5. Restart Auto-Negotiation (BMCR: AN enable + restart + 1Gbps speed bits)
	eth_phy_write(phy_addr, 0, 0x1340);

	// 6. Set MAC to 1Gbps Full Duplex (RGMII 125MHz mode)
	eth_set_mode(ETH_FULL_DUPLEX_1000M);

	printf("[PHY] 1Gbps Auto-Negotiation (Auto Master/Slave) restarted on PHY 0x%02X.\r\n", phy_addr);
}
