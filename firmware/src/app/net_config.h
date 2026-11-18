/*
 * net_config.h - the board's IPv4 address, mask and gateway (and MAC): kept in the
 * settings store (nvstore.h), set over the network (0x0003, spec § 4.1) or reset from the UART.
 *
 * Since 15.10.2026 (owner's request: moving the board between the direct cable
 * and the office network must not need a firmware change). The compile-time
 * address in board.h (NET_*) is the default: it is used when the store holds
 * no valid record, and console '7' returns to it.
 *
 * Record at 0x3D0 of the store (16 B; settings.h has the map):
 *   [0] 0xA5 magic, [1] version 1, [2..5] IP, [6..9] mask, [10..13] gateway
 *   (network order), [14..15] CRC-16/CCITT-FALSE of bytes 0..13, big-endian.
 * One copy: a write torn by a power cut fails the CRC and the board comes up
 * on the default address.
 */

#ifndef APP_NET_CONFIG_H_
#define APP_NET_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#define NET_CONFIG_NV_ADDR  0x3D0U
#define NET_CONFIG_BYTES        16U
// MAC record (since 22.10.2026), 16 B: [0] 'M' 0x4D, [1] version 1, [2..7] MAC, [8..13] 0xFF,
// [14..15] CRC-16 as above. Read at boot (net_config_boot) before the netif is up; without a valid
// record the board.h default NET_MAC_ADDR*. Set by 0x0005 (spec § 4.1), applies at the next boot.
#define NET_MAC_NV_ADDR     0x3F0U
// GigE Vision (since 23.10.2026), format-2 records of the store (nvstore.h, R-41):
// NV_TAG_USER_NAME - the user-defined name (bootstrap 0x00E8), ASCII, up to NET_NAME_MAX bytes;
// NV_TAG_GEV_IPCFG - {u8 version 1, u8 flags NET_IPCFG_*}; without it: persistent IP and DHCP on.
#define NET_NAME_MAX        16U
#define NET_IPCFG_PERSISTENT 0x1U   /* GEV 27.5 bit P: boot on the stored (or default) address */
#define NET_IPCFG_DHCP       0x2U   /* bit D: without P - DHCP, then link-local (LLA) */

typedef struct {
    uint8_t ip[4];
    uint8_t mask[4];
    uint8_t gw[4];      // 0.0.0.0 = no gateway
} net_config_t;

// Where the running address came from.
#define NET_CONFIG_SRC_DEFAULT  0   // no record in the store
#define NET_CONFIG_SRC_STORED   1   // a valid record
#define NET_CONFIG_SRC_DAMAGED  2   // a record with a bad CRC or bad values: default used
#define NET_CONFIG_SRC_NO_STORE 3  // the store did not answer: default used
#define NET_CONFIG_SRC_SET      4   // set over the network or the UART since boot
#define NET_CONFIG_SRC_DYNAMIC  5   // DHCP / link-local (Persistent IP flag off)

// The compile-time default (board.h).
void net_config_default(net_config_t *c);
// Read the store and apply the address to the netif. task_console calls it
// once, before the interface is brought up. Returns NET_CONFIG_SRC_*.
uint8_t net_config_boot(void);
// The IP configuration cycle again (GigE Vision FORCEIP with address 0): the stored address
// when the Persistent IP flag is on, otherwise DHCP / link-local.
void net_config_restart(void);
// GigE Vision IP configuration flags NET_IPCFG_* (store record; default persistent + DHCP).
// They apply at the next boot or net_config_restart(). Set: 0 OK, 2 store write failed.
uint32_t net_ipcfg_get(void);
int      net_ipcfg_set(uint32_t flags);
// The running address (from the netif).
void net_config_current(net_config_t *c);
// Values a board may take: mask contiguous /8../30, the IP a host address of
// its subnet (not 0, 127/8, multicast, network or broadcast), the gateway 0 or
// a host of the same subnet other than the IP.
bool net_config_valid(const net_config_t *c);
// Check, optionally write to the store (with read-back), then apply at once.
// 0 OK, 1 values rejected, 2 store write failed (nothing changed).
int  net_config_set(const net_config_t *c, bool save);
// Only check and write the store record, the address is not changed (the 0x0003
// handler answers first, then net_config_set(c, false)). Codes as above.
int  net_config_store(const net_config_t *c);
// The stored address; false (and the board.h default in *c) when no valid record.
bool net_config_stored(net_config_t *c);
// Put an address on the netif as is (no checks, the source is not changed); stops DHCP / LLA.
void net_config_apply(const net_config_t *c);
// Erase the store record and go back to the default address at once.
int  net_config_forget(void);
// True when `ip` (a.b.c.d as (a << 24) | ... | d) lies in the board's subnet.
bool net_config_on_link(uint32_t ip);
// Running, stored and default address to the UART (console '6').
void net_config_print(void);
// MAC: board.h default; valid = unicast and not all zero; store with read-back (NULL erases
// the record): 0 OK, 1 rejected, 2 store write failed.
void net_mac_default(uint8_t m[6]);
bool net_mac_valid(const uint8_t m[6]);
int  net_mac_store(const uint8_t m[6]);
// User-defined name: empty when none. Store: 0 OK, 1 longer than NET_NAME_MAX, 2 store write failed.
void net_name_get(char name[NET_NAME_MAX + 1]);
int  net_name_store(const char *name);

#endif /* APP_NET_CONFIG_H_ */
