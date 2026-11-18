/*
 * ******************************************************************************************
 * File		: ethernet.h
 * Author	: GowinSemicoductor
 * Chip		: AE350_SOC
 * Function	: Triple mode Ethernet driver definitions
 * ******************************************************************************************
 * */

#ifndef __ETHERNET_H__
#define __ETHERNET_H__


// Includes ---------------------------------------------------------------------------------
#include <stdbool.h>
#include "platform.h"
#include "Driver_Common.h"


// Definitions  -----------------------------------------------------------------------------

// Register definition
typedef struct
{
	union
	{
		__O  unsigned int ETH_TX_DATA[384];     /* 0x000~0x5ff transmit data registers          */
		__I  unsigned int ETH_RX_DATA[384];     /* 0x000~0x5ff receive data registers           */
	};
	__IO unsigned int ETH_TX_LENGTH;            /* 0x600 transmit data length, bit[10:0]        */
	__IO unsigned int ETH_TX_EN;                /* 0x604 transmit enable, bit[0]                */
	__IO unsigned int ETH_TX_FAIL;              /* 0x608 transmit fail, bit[2,1,0]              */
	__I  unsigned int ETH_TX_IS;                /* 0x60c transmit interrupt status, bit[0]      */
	__O  unsigned int ETH_TX_IC;                /* 0x610 transmit interrupt clear, bit[0]       */
	__IO unsigned int ETH_TX_IE;                /* 0x614 transmit interrupt enable, bit[0]      */
		 unsigned int RESERVED_1[26];           /* 0x618~0x67f reserved                         */
	__I  unsigned int ETH_RX_LENGTH;            /* 0x680 receive data length                    */
	__I  unsigned int ETH_RX_IS;                /* 0x684 receive interrupt status, bit[0]       */
	__IO unsigned int ETH_RX_IE;                /* 0x688 receive interrupt enable, bit[0]       */
	__O  unsigned int ETH_RX_IC;                /* 0x68c receive interrupt clear, bit[0]        */
		 unsigned int RESERVED_2[28];           /* 0x690~0x6ff reserved                         */
	__IO unsigned int MIIM_OP_MODE;             /* 0x700 MIIM operation mode, bit[0]            */
	__IO unsigned int MIIM_PHY_ADDR;            /* 0x704 MIIM physical address, bit[4:0]        */
	__IO unsigned int MIIM_REG_ADDR;            /* 0x708 MIIM register address, bit[4:0]        */
	__IO unsigned int MIIM_WR_DATA;             /* 0x70c MIIM write data, bit[15:0]             */
	__I  unsigned int MIIM_RD_DATA;             /* 0x710 MIIM read data, bit[15:0]              */
	__I  unsigned int MIIM_IS;                  /* 0x714 MIIM interrupt status, bit[1,0]        */
	__IO unsigned int MIIM_IE;                  /* 0x718 MIIM interrupt enable, bit[1,0]        */
	__O  unsigned int MIIM_IC;                  /* 0x71c MIIM interrupt clear, bit[1,0]         */
	__IO unsigned int MIIM_OP_EN;               /* 0x720 MIIM operation enable, bit[0]          */
	__IO unsigned int ETH_MODE;                 /* 0x724 Ethernet operation mode, bit[2,1,0]    */
} ETHERNET_RegDef;

// Memory map definition
#define ETHERNET_BASE		AHB_EXT_BASE		/* Ethernet base address                        */

// Device definitions
#define AE350_ETHERNET		((ETHERNET_RegDef *) ETHERNET_BASE)
#define DEV_ETHERNET		AE350_ETHERNET

// Interrupt definition
// Use IRQ_GP0_SOURCE
// Use gp0_irq_handler

// Bit definition
#define BIT0                  (1UL << 0)

#define ETH_TX_LATE_COL       (1UL << 2)
#define ETH_TX_EXCESSIVE_COL  (1UL << 1)
#define ETH_TX_FAIL_FAIL      (1UL << 0)

#define MIIM_OP_END_IS        (1UL << 1)
#define MIIM_RD_DATA_VALID_IS (1UL << 0)

#define MIIM_OP_END_IE        (1UL << 1)
#define MIIM_RD_DATA_VALID_IE (1UL << 0)

#define MIIM_OP_END_IC        (1UL << 1)
#define MIIM_RD_DATA_VALID_IC (1UL << 0)

#define ETH_DUPLEX_MODE       (1UL << 2)

#define ETH_FULL_DUPLEX_100M    0UL
#define ETH_FULL_DUPLEX_1000M   1UL
#define ETH_FULL_DUPLEX_10M     2UL
#define ETH_HALF_DUPLEX_100M    4UL
#define ETH_HALF_DUPLEX_10M     6UL

#define ETH_RX 0
#define ETH_TX 1

// Debug
//#define DEBUG_PRINT


// Access structure of the Ethernet driver
typedef struct _AE350_DRIVER_ETHERNET
{
	int32_t (*init)             (uint8_t *rx_ptr, uint32_t *rx_data_length, uint32_t *miim_rd_ptr);			// Initializes
	int32_t (*eth_tx)           (uint8_t *tx_message, uint32_t tx_length);									// Transmit
	void    (*miim_write)       (uint8_t miim_phy_address, uint8_t miim_reg_addr, uint16_t miim_wr_data);	// MIIM write
	void    (*miim_receive)     (uint8_t miim_phy_address, uint8_t miim_reg_addr);							// MIIM receive
	void    (*eth_set_mode)     (uint32_t eth_mode);														// Set working mode
} const AE350_DRIVER_ETHERNET;

// Ethernet information
typedef struct _ETHERNET_INFO
{
	uint8_t  *rx_ptr;
	uint32_t *rx_data_length;
	uint32_t *miim_rd_ptr;
} ETHERNET_INFO;


extern void rx_int_event(void);
extern int32_t eth_init(uint8_t* rx_ptr, uint32_t *rx_data_length, uint32_t *miim_rd_ptr);
extern int32_t eth_tx (uint8_t *tx_message, uint32_t tx_length);
extern void miim_write(uint8_t miim_phy_address, uint8_t miim_reg_addr, uint16_t miim_wr_data);
extern void miim_receive(uint8_t miim_phy_address, uint8_t miim_reg_addr);
extern void eth_set_mode(uint32_t eth_mode);


// Mutex to serialize hardware Ethernet MAC transmissions
#include "FreeRTOS.h"
#include "semphr.h"
extern SemaphoreHandle_t xEthTxMutex;


// PHY: reset, 2 ns RGMII delays, 10/100/1000 advertisement, autonegotiation
// restart, MAC in 1000M FD. Called by task_console at start-up.
void eth_phy_init(uint8_t phy_addr);

// PHY Realtek RTL8211FI-CG (06.10.2026; not a Motorcomm YT8531C as was thought,
// ERRORS 3.10). PAGSR and PHYSR per the datasheet
// (docs/hardware/RTL8211F-CG-Datasheet.pdf, tables 39, 41). Page 0xD08 with the
// software RGMII delays is not described in the public datasheet; it is taken
// from the Linux driver (drivers/net/phy/realtek.c:
// RTL8211F_TX_DELAY = BIT(8) in 0xD08/0x11, RTL8211F_RX_DELAY = BIT(3) in 0xD08/0x15).
#define ETH_PHY_PAGSR           0x1F    /* page select, 0 = default */
#define ETH_PHY_PHYSR           0x1A    /* status: [5:4] speed, [3] FD, [2] link */
#define ETH_PHY_PAGE_DELAY      0x0D08
#define ETH_PHY_REG_TXDLY       0x11
#define ETH_PHY_TXDLY_BIT       (1U << 8)
#define ETH_PHY_REG_RXDLY       0x15
#define ETH_PHY_RXDLY_BIT       (1U << 3)

// PHY registers over MIIM: regular ones (0..31) and on page `page` (then page 0).
uint16_t eth_phy_reg_read(uint8_t phy_addr, uint8_t reg_addr);
uint16_t eth_phy_paged_read(uint8_t phy_addr, uint16_t page, uint8_t reg_addr);
void     eth_phy_paged_write(uint8_t phy_addr, uint16_t page, uint8_t reg_addr, uint16_t val);

// Link state: PHYSR (speed 10/100/1000, duplex, link now) and the BMSR latch:
// whether the link dropped since the last call. false: the PHY did not answer.
typedef struct {
	bool     link;        /* link now (PHYSR[2]) */
	bool     dropped;     /* the link dropped since the last call (BMSR[2] latched at 0) */
	uint16_t speed_mbps;  /* 10 / 100 / 1000, 0 = unknown */
	bool     full_duplex;
} eth_phy_link_t;
bool eth_phy_link_status(uint8_t phy_addr, eth_phy_link_t *st);

// RX-freeze experiment (15.10.2026, REVIEW_2026-10-15 R1-R3, ERRORS 3.15).
// ETH_PHY_EXP picks what eth_phy_init() does at boot:
//   0 - normal (reset, 10/100 + 1000FD advertised, auto master/slave, EEE
//       advertisement off: MMD 7.60 = 0, R-36);
//   1 - as 0 but EEE left on (the PHY default; until 15.10.2026 every build);
//   2 - eth_phy_init() does nothing: no MIIM at all, the PHY runs on its straps
//       (EEE on);
//   3 - as 0, master forced (reg 9 = 0x1A00);
//   4 - as 0, slave forced (reg 9 = 0x1200).
#ifndef ETH_PHY_EXP
#define ETH_PHY_EXP 0
#endif
// ETH_RX_IC_AFTER_COPY 1: the ISR masks RX_IE and leaves RX_IS set; ETHIN
// writes RX_IC only after it has copied the frame (as the Gowin reference
// rx_int_event does). 0: the ISR writes RX_IC at once, before the copy (until
// 15.10.2026) - under bursts the next frame overwrites the window during the
// copy (ERRORS 3.15).
#ifndef ETH_RX_IC_AFTER_COPY
#define ETH_RX_IC_AFTER_COPY 1
#endif
// 1 while the ISR has masked RX_IE for a frame ETHIN has not taken yet.
extern volatile uint32_t g_eth_rx_masked;
// Clause 45 registers through the clause 22 window (regs 13/14, MACR/MAADR).
uint16_t eth_phy_mmd_read(uint8_t phy_addr, uint8_t devad, uint16_t reg);
void     eth_phy_mmd_write(uint8_t phy_addr, uint8_t devad, uint16_t reg, uint16_t val);
void     eth_phy_reg_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t val);
// ETH_MODE from PHYSR: the MAC follows the negotiated speed. Returns the
// speed in Mbit/s, 0 when the link is down or the PHY did not answer (ETH_MODE
// then stays as it was).
uint16_t eth_mac_follow_phy(uint8_t phy_addr);

#endif		/* __ETHERNET_H__ */
