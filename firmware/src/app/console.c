/*
 * console.c - UART console commands (COM7): one letter, one action.
 *
 * Until 24.09.2026 the menu was the body of the task_console loop in main.c; the
 * task (LEDs, [ETH STATS], byte input) stayed there and calls console_command()
 * for every received character. The command list: tools/README.md, section "uart".
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "Driver_GPIO.h"
#include "uart.h"
#include "cache.h"   // ae350_dcache_invalidate_range(), command 'V'

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "camera.h"
#include "i2c_bus.h"
#include "memory_map.h"
#include "cam_trig.h"
#include "video.h"
#include "laser.h"
#include "settings.h"
#include "console.h"
#include "arch/sys_arch.h"
#include "arch/ethernetif.h"
#include "ethernet.h"     // PHY access: commands 'Y', 'A', 'Q', '2'..'4', '8'
#include "lwip/stats.h"   // lwip_stats: command '5'
#include "net_config.h"   // commands '6', '7'
#include "watchdog.h"     // commands '#', '$'
#include "lcd_hud.h"      // commands 'M', 'W', 'X'
#include "hik_client.h"   // commands 'G', 'T', 'D', '[', ']'

// Active camera connector. The Tang Mega TWI multiplexer address is
// {TWI_MUX_S2, TWI_MUX_S1, TWI_MUX_S0} = {1, 1, GPIO[6]}, and GPIO[6] is this
// index. Connector 0 corresponds to the multiplexer position the reference
// bring-up design reaches the camera at (its DEFAULT_MUX = 3, whose bit order
// is reversed relative to this project's naming).
#define ACTIVE_CAM				0

/*
 * =========================================================================================
 * FreeRTOS CPU Performance Profiler
 * =========================================================================================
 * - Prints runtime CPU execution stats table for all tasks.
 * - Shows exact CPU % load for Task_Video, Task_Ethernet, Task_Cam0, Task_Cam1, and IDLE.
 */
static void print_cpu_profiler_stats(void)
{
	printf("\r\n======================================================\r\n");
	printf(" [STATUS] Free Heap: %u bytes\r\n", (unsigned int)xPortGetFreeHeapSize());
	printf("======================================================\r\n\r\n");
}

// PHY Realtek RTL8211F (06.10.2026, ERRORS 3.10; docs/OPEN_ISSUES.md 1.1).
// RGMII delays of 2 ns each, on/off: TX is page 0xD08 reg. 0x11 bit 8, RX is
// 0xD08/0x15 bit 3 (Linux driver realtek.c; not described in the public
// datasheet). At start-up they are set by the RXD1/TXDLY and RXD0/RXDLY straps,
// the firmware does not touch them.
//   Y - registers, link state (PHYSR) and delays, receive watchdog counters;
//   H - toggle the RX delay (2 ns to RXC);
//   J - toggle the TX delay (2 ns to TXC).
// H/J shift the RGMII phase by half a bit period: the link may drop until the
// value is restored with the same command or the board is rebooted. UART only.
#define CONSOLE_PHY_ADDR        0
extern volatile uint32_t g_eth_wd_complete, g_eth_wd_drain, g_eth_wd_rxic, g_eth_wd_speed;

static void console_phy_dump(void)
{
	uint16_t bmsr0 = eth_phy_reg_read(CONSOLE_PHY_ADDR, 1);    /* link-drop latch */
	uint16_t bmsr1 = eth_phy_reg_read(CONSOLE_PHY_ADDR, 1);    /* current */
	uint16_t physr = eth_phy_reg_read(CONSOLE_PHY_ADDR, ETH_PHY_PHYSR);
	uint16_t txd = eth_phy_paged_read(CONSOLE_PHY_ADDR, ETH_PHY_PAGE_DELAY, ETH_PHY_REG_TXDLY);
	uint16_t rxd = eth_phy_paged_read(CONSOLE_PHY_ADDR, ETH_PHY_PAGE_DELAY, ETH_PHY_REG_RXDLY);
	static const char *const k_speed[4] = { "10M", "100M", "1000M", "?" };

	printf("[PHY] ID 0x%04X:%04X BMCR 0x%04X BMSR 0x%04X->0x%04X GBSR 0x%04X PAGSR 0x%04X\r\n",
	       eth_phy_reg_read(CONSOLE_PHY_ADDR, 2), eth_phy_reg_read(CONSOLE_PHY_ADDR, 3),
	       eth_phy_reg_read(CONSOLE_PHY_ADDR, 0), bmsr0, bmsr1,
	       eth_phy_reg_read(CONSOLE_PHY_ADDR, 0x0A), eth_phy_reg_read(CONSOLE_PHY_ADDR, ETH_PHY_PAGSR));
	printf("[PHY] PHYSR 0x%04X: link %u %s %s, master %u, MDI plug %u | delay TX %s (0x%04X) RX %s (0x%04X)"
	       " | WD cmp %u drain %u rxic %u\r\n",
	       physr, (unsigned)((physr >> 2) & 1U), k_speed[(physr >> 4) & 3U],
	       (physr & (1U << 3)) ? "FD" : "HD", (unsigned)((physr >> 11) & 1U), (unsigned)((physr >> 13) & 1U),
	       (txd & ETH_PHY_TXDLY_BIT) ? "+2ns" : "off", txd,
	       (rxd & ETH_PHY_RXDLY_BIT) ? "+2ns" : "off", rxd,
	       (unsigned)g_eth_wd_complete, (unsigned)g_eth_wd_drain, (unsigned)g_eth_wd_rxic);
}

static void console_phy_toggle(uint8_t reg, uint16_t bit, const char *name)
{
	uint16_t v = eth_phy_paged_read(CONSOLE_PHY_ADDR, ETH_PHY_PAGE_DELAY, reg);
	uint16_t nv = (uint16_t)(v ^ bit);

	eth_phy_paged_write(CONSOLE_PHY_ADDR, ETH_PHY_PAGE_DELAY, reg, nv);
	printf("[PHY] %s delay: 0xD08/0x%02X 0x%04X -> 0x%04X (%s)\r\n",
	       name, (unsigned)reg, v, nv, (nv & bit) ? "+2ns" : "off");
	console_phy_dump();
}

// RX-freeze experiment (15.10.2026, ERRORS 3.15; REVIEW_2026-10-15 R1-R4).
//   A - ETH_MODE from PHYSR (MAC follows the link speed);
//   Q - PHY soft reset (BMCR 0x9140), 3 s, then as A;
//   2 - advertise 10/100 only, restart AN (the link comes back at 100M, MAC untouched);
//   3 - advertise 1000FD again (reg 9 0x0200), restart AN (MAC untouched);
//   4 - link drop: PHY power-down for 1 s, then AN restart (MAC untouched);
//   8 - EEE/LPI registers; 9 - EEE advertisement off/on + AN restart;
//   5 - lwIP checksum and drop counters, ETH_MODE.
static void console_eee_dump(void)
{
	uint16_t pcs1 = eth_phy_mmd_read(CONSOLE_PHY_ADDR, 3, 1);   /* LPI latches clear on read */
	printf("[EEE] 3.0 0x%04X | 3.1 0x%04X (rxLPI now %u, latched rx %u tx %u) | 3.20 0x%04X | adv 7.60 0x%04X LP 7.61 0x%04X"
	       " | wake err 3.22 %u\r\n",
	       eth_phy_mmd_read(CONSOLE_PHY_ADDR, 3, 0), pcs1,
	       (unsigned)((pcs1 >> 8) & 1U), (unsigned)((pcs1 >> 10) & 1U), (unsigned)((pcs1 >> 11) & 1U),
	       eth_phy_mmd_read(CONSOLE_PHY_ADDR, 3, 20),
	       eth_phy_mmd_read(CONSOLE_PHY_ADDR, 7, 60), eth_phy_mmd_read(CONSOLE_PHY_ADDR, 7, 61),
	       (unsigned)eth_phy_mmd_read(CONSOLE_PHY_ADDR, 3, 22));
}

static void console_lwip_stats(void)
{
#if LWIP_STATS
	printf("[LWIP] link recv %u drop %u chk %u len %u mem %u | ip recv %u chk %u drop %u | tcp recv %u chk %u drop %u"
	       " | udp chk %u drop %u | icmp chk %u | ETH_MODE %u WD speed %u\r\n",
	       (unsigned)lwip_stats.link.recv, (unsigned)lwip_stats.link.drop, (unsigned)lwip_stats.link.chkerr,
	       (unsigned)lwip_stats.link.lenerr, (unsigned)lwip_stats.link.memerr,
	       (unsigned)lwip_stats.ip.recv, (unsigned)lwip_stats.ip.chkerr, (unsigned)lwip_stats.ip.drop,
	       (unsigned)lwip_stats.tcp.recv, (unsigned)lwip_stats.tcp.chkerr, (unsigned)lwip_stats.tcp.drop,
	       (unsigned)lwip_stats.udp.chkerr, (unsigned)lwip_stats.udp.drop, (unsigned)lwip_stats.icmp.chkerr,
	       (unsigned)DEV_ETHERNET->ETH_MODE, (unsigned)g_eth_wd_speed);
#endif
}

// PLIC state for the Ethernet source 4 (21.10.2026, sick boots, ERRORS 3.7): on a sick
// boot the MAC's interrupt line stays high while nobody claims it (GAO, 19.10.2026).
// Feature (preempt/vectored), trigger type, priority, pending, enable, threshold.
extern volatile uint32_t g_plic_claim_src4, g_plic_complete_src4, g_plic_claim_zero;   /* interrupt.c */

void console_plic_dump(void)
{
	volatile uint32_t *plic = (volatile uint32_t *)NDS_PLIC_BASE;

	printf("[PLIC] feature 0x%X trigger 0x%08X | src4 prio %u pending %u enable %u | threshold %u"
	       " | mmisc_ctl 0x%X mie 0x%X mip 0x%X | claim4 %u complete4 %u claim0 %u | MAC IS rx %u IE rx %u\r\n",
	       (unsigned)plic[0], (unsigned)plic[0x1080 / 4],
	       (unsigned)plic[IRQ_GP0_SOURCE], (unsigned)((plic[0x1000 / 4] >> IRQ_GP0_SOURCE) & 1U),
	       (unsigned)((plic[0x2000 / 4] >> IRQ_GP0_SOURCE) & 1U), (unsigned)plic[0x200000 / 4],
	       (unsigned)read_csr(NDS_MMISC_CTL), (unsigned)read_csr(NDS_MIE), (unsigned)read_csr(NDS_MIP),
	       (unsigned)g_plic_claim_src4, (unsigned)g_plic_complete_src4, (unsigned)g_plic_claim_zero,
	       (unsigned)DEV_ETHERNET->ETH_RX_IS, (unsigned)DEV_ETHERNET->ETH_RX_IE);
	for (uint32_t i = 0; i < g_eth_wd_ev_n && i < ETH_WD_EVENTS; ++i)
		printf("[PLIC] wd#%u tick %u: RX_IE %u masked %u pending4 %u mip 0x%X gp0 irqs %u late %d\r\n",
		       (unsigned)i, (unsigned)g_eth_wd_ev[i].tick, (unsigned)g_eth_wd_ev[i].ie,
		       (unsigned)g_eth_wd_ev[i].masked, (unsigned)g_eth_wd_ev[i].pending,
		       (unsigned)g_eth_wd_ev[i].mip, (unsigned)g_eth_wd_ev[i].irqs, (int)g_eth_wd_ev[i].late);
}

void console_phy_boot_report(void)
{
	console_plic_dump();
	console_phy_dump();
	console_eee_dump();
	console_lwip_stats();
}

void console_command(int cmd)
{
	if (cmd == 'Y' || cmd == 'y')
	{
		console_phy_dump();
	}
	else if (cmd == '!')
	{
		console_plic_dump();
	}
	else if (cmd == '#')
	{
		watchdog_console();      // twice within 5 s: the supervisor stops feeding (test)
	}
	else if (cmd == '$')
	{
		// Test of the supervisor against a hung task: ETHIN is suspended, the supervisor
		// must see it silent and reset the SoC within ~3.5 s.
		static TaskStatus_t st[24];
		UBaseType_t n = uxTaskGetSystemState(st, 24, NULL), i;
		for (i = 0; i < n; ++i)
			if (strcmp(st[i].pcTaskName, "ETHIN") == 0) {
				printf("[WDT] test: ETHIN suspended, reset expected in ~3.5 s\r\n");
				vTaskSuspend(st[i].xHandle);
			}
	}
	else if (cmd == 'A' || cmd == 'a')
	{
		uint16_t sp = eth_mac_follow_phy(CONSOLE_PHY_ADDR);
		printf("[PHY] MAC follows PHYSR: %u Mbit/s, ETH_MODE %u\r\n", (unsigned)sp, (unsigned)DEV_ETHERNET->ETH_MODE);
	}
	else if (cmd == 'Q' || cmd == 'q')
	{
		uint16_t sp;
		printf("[PHY] soft reset (BMCR 0x9140), 3 s...\r\n");
		eth_phy_reg_write(CONSOLE_PHY_ADDR, 0, 0x9140);
		vTaskDelay(pdMS_TO_TICKS(3000));
		sp = eth_mac_follow_phy(CONSOLE_PHY_ADDR);
		printf("[PHY] MAC follows PHYSR: %u Mbit/s, ETH_MODE %u\r\n", (unsigned)sp, (unsigned)DEV_ETHERNET->ETH_MODE);
		console_phy_dump();
	}
	else if (cmd == '2' || cmd == '3')
	{
		eth_phy_reg_write(CONSOLE_PHY_ADDR, 4, 0x01E1);
		eth_phy_reg_write(CONSOLE_PHY_ADDR, 9, (cmd == '2') ? 0x0000 : 0x0200);
		eth_phy_reg_write(CONSOLE_PHY_ADDR, 0, 0x1340);
		printf("[PHY] AN restarted, 1000FD %s, MAC untouched (ETH_MODE %u)\r\n",
		       (cmd == '2') ? "NOT advertised" : "advertised", (unsigned)DEV_ETHERNET->ETH_MODE);
	}
	else if (cmd == '4')
	{
		eth_phy_reg_write(CONSOLE_PHY_ADDR, 0, 0x1940);     /* AN enable + power down */
		vTaskDelay(pdMS_TO_TICKS(1000));
		eth_phy_reg_write(CONSOLE_PHY_ADDR, 0, 0x1340);     /* power up + AN restart */
		printf("[PHY] link dropped for 1 s, AN restarted, MAC untouched\r\n");
	}
	else if (cmd == '8')
	{
		console_eee_dump();
	}
	else if (cmd == '5')
	{
		console_lwip_stats();
	}
	// Board address (net_config.c, 15.10.2026): '6' shows the running, stored and
	// default address; '7' erases the stored one and returns to the default at
	// once - the way back when the address is lost.
	else if (cmd == '6')
	{
		net_config_print();
	}
	else if (cmd == '7')
	{
		int rc = net_config_forget();
		printf("[NET] stored address erased: %s\r\n", rc == 0 ? "OK, default now" : "store write FAILED");
		net_config_print();
	}
	else if (cmd == 'H' || cmd == 'h')
	{
		console_phy_toggle(ETH_PHY_REG_RXDLY, ETH_PHY_RXDLY_BIT, "RX");
	}
	else if (cmd == 'J' || cmd == 'j')
	{
		console_phy_toggle(ETH_PHY_REG_TXDLY, ETH_PHY_TXDLY_BIT, "TX");
	}
	else if (cmd == 'B' || cmd == 'b')
	{
		print_cpu_profiler_stats();
	}
	else if (cmd == 'M' || cmd == 'm')
	{
		lcd_hud_anim_next_mode();
	}
	else if (cmd == 'W' || cmd == 'w')
	{
		lcd_hud_cycle_rotation();
	}
	else if (cmd == 'X' || cmd == 'x')
	{
		lcd_hud_toggle_resolution();
	}
	else if (cmd == 'G' || cmd == 'g')
	{
		hik_client_toggle_enabled();
	}
	else if (cmd == 'T' || cmd == 't')
	{
		hik_client_toggle_decim();
	}
	else if (cmd == 'D' || cmd == 'd')
	{
		hik_client_toggle_auto_contrast();
		hik_client_print_status();
	}
	else if (cmd == '[')
	{
		hik_client_adjust_exposure(-50);
	}
	else if (cmd == ']')
	{
		hik_client_adjust_exposure(50);
	}
	else if (cmd == '1')
	{
		printf("[UART] Enabling Test Pattern #1 (Horizontal Ramp) on active cam (1080x1280)...\r\n");
		if (i2c_bus_lock(1000))
		{
			camera_configure(ACTIVE_CAM, 1, 0, 0, 20000, 100, 1000, 1, VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT, 0, 0);
			camera_set_trigger_mode(ACTIVE_CAM, 1, 1);
			video_invalidate_trigger_arming();
			i2c_bus_unlock();
		}
		printf("[UART] Test Pattern #1 activated!\r\n");
	}
	else if (cmd == '0')
	{
		printf("[UART] Disabling Test Pattern (Live Video) on active cam (%ux%u)...\r\n",
		       (unsigned int)VIDEO_FRAME_WIDTH, (unsigned int)VIDEO_FRAME_HEIGHT);
		if (i2c_bus_lock(1000))
		{
			camera_configure(ACTIVE_CAM, 1, 0, 0, 20000, 100, 1000, 0, VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT, 0, 0);
			camera_set_trigger_mode(ACTIVE_CAM, 1, 1);
			video_invalidate_trigger_arming();
			i2c_bus_unlock();
		}
		printf("[UART] Test Pattern disabled!\r\n");
	}
	else if (cmd == 'I' || cmd == 'i')
	{
		if (i2c_bus_lock(2000))
		{
			camera_dump_link_info(ACTIVE_CAM);
			i2c_bus_unlock();
		}
		else
		{
			printf("[UART] I2C bus busy - register dump aborted.\r\n");
		}
	}
	else if (cmd == 'C' || cmd == 'c')
	{
		printf("\r\n[CAM DIAG] Scanning all 8 TWI mux positions for 0x0000 (VEYE ID)...\r\n");
		if (i2c_bus_lock(5000))
		{
			for (int m = 0; m < 8; m++)
			{
				uint32_t val = 0;
				i2c_mux_select(m);
				int rc = camera_read_reg_status(0x0000, &val);
				printf("  Mux %d ({N19=%d,P19=%d,P26=%d}): rc=%d, id=0x%08X %s\r\n",
				       m, (m >> 2) & 1, (m >> 1) & 1, (m >> 0) & 1, rc, (unsigned int)val,
				       (rc == 0 && val == 0x56455945) ? "[FOUND VEYE!]" : "");
			}
			// Also print existing camera info
			for (int c = 0; c < 2; c++)
			{
				bool conn = false;
				uint32_t mid = 0, sw = 0, sh = 0, exp = 0, gain = 0, fps = 0, max_fps = 0, err = 0;
				uint8_t tm = 0, ts = 0, ae = 0, tp = 0;
				uint32_t rw = 0, rh = 0, ox = 0, oy = 0;
				camera_read_config(c, &conn, &mid, &sw, &sh, &tm, &ts, &ae,
				                   &exp, &gain, &fps, &max_fps, &err, &tp,
				                   &rw, &rh, &ox, &oy);
				printf("[CAM DIAG] Cam %d (Mux %d): %s | Model=0x%04X | Exp=%uus | Gain=%u.%udB | FPS=%u.%u\r\n",
				       c, camera_mux_position((uint8_t)c), conn ? "CONNECTED" : "NOT FOUND",
				       (unsigned int)mid, (unsigned int)exp, (unsigned int)(gain / 10), (unsigned int)(gain % 10),
				       (unsigned int)(fps / 100), (unsigned int)(fps % 100));
			}
			i2c_bus_unlock();
		}
		else
		{
			printf("[CAM DIAG] I2C bus busy!\r\n");
		}
	}
	// Removed 22.10.2026, their questions answered or the hardware unused: 'E'/'W' (EEPROM U24
	// dump and write test; settings in SPI flash, R-40), 'M' (memory read and printf timing),
	// 'X' (MAC window write, CPU vs DMA), 'G'/'T' (camera software trigger; 0x0112 mode 2 does
	// it), 'D' (raw slot dump, 'V' shows the headers), 'L' (packet log), '9' (EEE advertisement).
	// Settings storage (settings.c over nvstore.c, formerly net_stack.c section
	// "Settings in store"; network: 0x0160..0x016A). S, L and D are already taken
	// in the menu:
	//   'K' - partition state and both mode sets (read-only);
	//   'U' - save the settings partition (and the calibration, if present);
	//   'N' - re-read the settings from the store, dropping unsaved changes;
	//   'Z' - reset the settings in RAM to defaults (the store is not touched).
	// Without Ethernet this is the only way to return the board to defaults.
	else if (cmd == 'K' || cmd == 'k')
	{
		settings_print();
	}
	else if (cmd == 'U' || cmd == 'u')
	{
		settings_info_t in;
		int st = settings_save(SETTINGS_SEC_CFG, &in);
		printf("[SETTINGS] save settings -> status 0x%02X, seq %u\r\n",
		       (unsigned int)st, (unsigned int)in.seq);
		if (laser_calib_valid())
		{
			st = settings_save(SETTINGS_SEC_CALIB, &in);
			printf("[SETTINGS] save calibration -> status 0x%02X, seq %u\r\n",
			       (unsigned int)st, (unsigned int)in.seq);
		}
	}
	else if (cmd == 'N' || cmd == 'n')
	{
		settings_info_t in;
		int st = settings_load(SETTINGS_SEC_CFG, &in);
		printf("[SETTINGS] reload settings -> status 0x%02X, seq %u\r\n",
		       (unsigned int)st, (unsigned int)in.seq);
	}
	else if (cmd == 'Z' || cmd == 'z')
	{
		int st = settings_defaults(SETTINGS_RESET_GUARD);
		printf("[SETTINGS] defaults -> status 0x%02X (press 'U' to store them)\r\n",
		       (unsigned int)st);
	}
	else if (cmd == 'F' || cmd == 'f')
	{
		// Freeze the ring by stopping the sensor: with no MIPI data the FIFO
		// drains, the Lane 4 FSM parks in ST_IDLE and DDR3 stops changing, so
		// a slow JTAG dump cannot be torn mid-read.
		if (i2c_bus_lock(1000))
		{
			camera_set_acquisition(ACTIVE_CAM, 0);
			i2c_bus_unlock();
			printf("[UART] Sensor acquisition STOPPED - DDR3 ring is stable for a JTAG dump.\r\n");
		}
		else
		{
			printf("[UART] I2C bus busy - freeze aborted.\r\n");
		}
	}
	else if (cmd == 'R' || cmd == 'r')
	{
		if (i2c_bus_lock(1000))
		{
			camera_set_acquisition(ACTIVE_CAM, 1);
			video_invalidate_trigger_arming();   // the sensor writes the ring again
			i2c_bus_unlock();
			printf("[UART] Sensor acquisition RESUMED.\r\n");
		}
	}
	else if (cmd == 'P' || cmd == 'p')
	{
		// Issue a series of hardware pulses with the current settings.
		// It duplicates the TRIG command over TCP on purpose: TCP disappears exactly
		// when a build falls short on timing (ERRORS 3.1, former 31.20), and the
		// trigger is needed precisely at such moments.
		uint32_t freq, count, width;
		int rc;
		cam_trig_get(&freq, &count, &width);
		rc = cam_trig_apply(freq, count, width, 0);
		if (rc == 0) video_invalidate_trigger_arming();   /* the camera writes the ring */
		printf("[TRIG] %u.%02u Hz, %u pulse(s), %u us%s  rc=%d\r\n",
		       (unsigned int)(freq / 100), (unsigned int)(freq % 100),
		       (unsigned int)count, (unsigned int)width,
		       (count == 0) ? "  [CONTINUOUS]" : "", rc);
	}
	else if (cmd == 'O' || cmd == 'o')
	{
		// Cycle through the frequencies, including both ends of the range:
		// with one key, no parameter parsing on the UART.
		static const uint32_t freqs[] = { 50, 100, 1000, 10000, 60000, 120000 };
		static int fi = 0;
		uint32_t freq, count, width;
		cam_trig_get(&freq, &count, &width);
		fi = (fi + 1) % (int)(sizeof(freqs) / sizeof(freqs[0]));
		freq = freqs[fi];
		printf("[TRIG] frequency now %u.%02u Hz (press P to fire)\r\n",
		       (unsigned int)(freq / 100), (unsigned int)(freq % 100));
		if (cam_trig_apply(freq, count, width, CAM_TRIG_FLAG_STOP) == 0)
			video_invalidate_trigger_arming();
	}
	else if (cmd == 'S' || cmd == 's')
	{
		printf("[UART] Querying active camera status...\r\n");
		camera_params_t st;
		if (camera_state_get(ACTIVE_CAM, &st))
		{
			camera_params_t *cfg = &st;
			printf("[CAMERA 1] Connected=%d, Model=0x%08X, %ux%u, Exp=%u us, Gain=%u.%udB, FPS=%u.%uHz, TestPat=%u\r\n",
			       cfg->connected, (unsigned int)cfg->model_id, (unsigned int)cfg->sensor_w, (unsigned int)cfg->sensor_h,
			       (unsigned int)cfg->exp, (unsigned int)(cfg->gain / 10), (unsigned int)(cfg->gain % 10),
			       (unsigned int)(cfg->fps / 100), (unsigned int)(cfg->fps % 100),
			       (unsigned int)cfg->testpat);
		}
	}
	else if (cmd == 'V' || cmd == 'v')
	{
		// Scan the ring by the current parameters.
		printf("\r\n[VERIFY] Scanning DDR3 ring: %u slots at 0x%08X...\r\n",
		       (unsigned int)VIDEO_RING_SLOTS, (unsigned int)VIDEO_RING_BASE_ADDR);
		uint32_t latest_slot = 0;
		uint32_t max_frame_id = 0;
		uint32_t active_slot_addr = VIDEO_RING_BASE_ADDR;

		for (int s = 0; s < VIDEO_RING_SLOTS; s++)
		{
			uint32_t s_addr = VIDEO_RING_BASE_ADDR + ((uint32_t)s * VIDEO_SLOT_BYTES);
			ae350_dcache_invalidate_range(s_addr, 64);
			volatile uint32_t *shdr = (volatile uint32_t *)s_addr;
			if (shdr[0] == 0x56494430)
			{
				printf("  [SLOT %d @ 0x%08X] Magic=VID0, FrameID=#%u\r\n", s, (unsigned int)s_addr, (unsigned int)shdr[1]);
				if (shdr[1] > max_frame_id)
				{
					max_frame_id = shdr[1];
					latest_slot = s;
					active_slot_addr = s_addr;
				}
			}
			else
			{
				printf("  [SLOT %d @ 0x%08X] Empty (Magic=0x%08X)\r\n", s, (unsigned int)s_addr, (unsigned int)shdr[0]);
			}
		}

		printf("[ACTIVE LATEST FRAME] Slot %u at 0x%08X (FrameID=#%u)\r\n",
		       (unsigned int)latest_slot, (unsigned int)active_slot_addr, (unsigned int)max_frame_id);

		ae350_dcache_flush_all();
		volatile uint32_t *p_hdr = (volatile uint32_t *)active_slot_addr;
		uint32_t magic = p_hdr[0];
		uint32_t frame_id = p_hdr[1];
		uint32_t slot_idx = (p_hdr[3] >> 16) & 0xFF;
		uint32_t w = p_hdr[3] & 0xFFFF;
		uint32_t h = (p_hdr[4] >> 16) & 0xFFFF;
		uint32_t src = p_hdr[4] & 0xFFFF;
		uint32_t integrity = p_hdr[5];

		printf("[VID0 HDR] Magic=0x%08X, FrameID=#%u, Slot=%u, Geom=%ux%u, Src=%u, Integ=0x%08X\r\n",
		       (unsigned int)magic, (unsigned int)frame_id, (unsigned int)slot_idx,
		       (unsigned int)w, (unsigned int)h, (unsigned int)src, (unsigned int)integrity);

		// Row 0: pixels follow right after the 256-byte header (+0x100, ERRORS 1.8).
		const uint8_t *p_pix = (const uint8_t *)VIDEO_ROW_ADDR(active_slot_addr, 0);
		uint8_t min_v = 255, max_v = 0;
		uint32_t sum_v = 0;
		uint32_t test_rows = (h > 0 && h < 100) ? h : 100;
		uint32_t test_cols = (w > 0) ? w : VIDEO_FRAME_WIDTH;
		uint32_t total_tested = test_rows * test_cols;
		uint32_t ramp_matches = 0;

		printf("[RAW PIXELS 0..31]: ");
		for (int i = 0; i < 32; i++)
		{
			printf("%u ", p_pix[i]);
		}
		printf("\r\n");

		for (uint32_t r = 0; r < test_rows; r++)
		{
			const uint8_t *row_ptr = (const uint8_t *)VIDEO_ROW_ADDR(active_slot_addr, r);
			for (uint32_t c = 0; c < test_cols; c++)
			{
				uint8_t v = row_ptr[c];
				if (v < min_v) min_v = v;
				if (v > max_v) max_v = v;
				sum_v += v;
				if (v == (uint8_t)(c % 256))
				{
					ramp_matches++;
				}
			}
		}

		printf("[PIXEL STATS] Range: Min=%u, Max=%u, Mean=%u.%u\r\n",
		       min_v, max_v, (unsigned int)(sum_v / total_tested),
		       (unsigned int)((sum_v % total_tested) * 10 / total_tested));
		printf("[RAMP MATCH] %u / %u pixels (%u%% match to ideal 0..255 ramp)\r\n",
		       (unsigned int)ramp_matches, (unsigned int)total_tested,
		       (unsigned int)(ramp_matches * 100 / total_tested));

		if (ramp_matches * 100 / total_tested > 95)
		{
			printf(">>> [PASS] CAMERA TEST PATTERN #1 CONFIRMED VALID & STREAMING IN DDR3! <<<\r\n\r\n");
		}
		else
		{
			printf(">>> [INFO] Live Video or Different Pattern Active <<<\r\n\r\n");
		}
	}
}
