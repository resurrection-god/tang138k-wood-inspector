/*
 * board.h - board network settings and the Lane 4 clock
 *
 * Part of net_stack.h until 24.09.2026.
 */

#ifndef APP_BOARD_H_
#define APP_BOARD_H_

#include <stdint.h>

// Network Configuration. Since 15.10.2026 these are only the DEFAULT address:
// the running one comes from settings store (net_config.c, set by 0x0003 or
// tools/net/board_ip.py), and these values are used when nothing is stored or
// after console '7'. Mirror: IP_ADDR*/NETMASK_ADDR*/GW_ADDR* in arch/sys_arch.h
// (the netif starts with those before net_config_boot()), change them together.
// Since 03.11.2026 (owner) 10.0.0.33, gateway 10.0.0.1 (was 192.168.31.125 from 13.10.2026,
// docs/DECISIONS.md R-26, and 10.0.0.125 before that). On the PC: vmsp.HOST (tools/common/vmsp.py).
#define NET_IP_ADDR0            10
#define NET_IP_ADDR1            0
#define NET_IP_ADDR2            0
#define NET_IP_ADDR3            222

#define NET_NETMASK0            255
#define NET_NETMASK1            255
#define NET_NETMASK2            255
#define NET_NETMASK3            0

#define NET_GATEWAY0            10
#define NET_GATEWAY1            0
#define NET_GATEWAY2            0
#define NET_GATEWAY3            1

#define NET_MAC_ADDR0           0x00
#define NET_MAC_ADDR1           0x12
#define NET_MAC_ADDR2           0x34
#define NET_MAC_ADDR3           0x56
#define NET_MAC_ADDR4           0x78
#define NET_MAC_ADDR5           0x9B


// The lane4_clk clock. It is exactly DDR3_MEMORY_CLK/4: since 29.09.2026 the
// memory runs at 400 MHz (within the datasheet, ERRORS 1.11, experiment E3b),
// so 100 MHz and a 10 ns clock. From 17 to 29.09 it was 400/3 MHz and 7.5 ns
// with DDR3-1066, before 17.09 also 100 MHz (CLAUDE.md invariant 2, lane4_clk
// in the .sdc, LANE_CLK_HZ of cam_channel in scanner_top.v).
// The burst generator, the sync block and the FPGA hw_timestamp count in these
// clocks. All clock <-> time conversions in the firmware go only through these
// macros.
#define LANE_TICK_PS            10000U
#define LANE_NS_TO_TICKS(ns)    ((uint32_t)((uint64_t)(ns) / 10ULL))
#define LANE_US_TO_TICKS(us)    ((uint64_t)(us) * 100ULL)
#define LANE_TICKS_TO_US(t)     ((uint64_t)(t) / 100ULL)


// Delay on MTIME (APB bus 100 MHz = 100 ticks per us), without the scheduler:
// usable both before it starts and under the bus mutex.
void board_delay_us(uint32_t us);

#endif /* APP_BOARD_H_ */
