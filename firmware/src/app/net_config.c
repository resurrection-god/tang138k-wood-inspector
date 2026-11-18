/*
 * net_config.c - the board's IPv4 address in settings store (net_config.h).
 */

#include <stdio.h>
#include <string.h>

#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/ip4_addr.h"
#include "lwip/dhcp.h"
#include "lwip/autoip.h"

#include "board.h"
#include "nvstore.h"
#include "net_config.h"

#define NC_MAGIC    0xA5U
#define NC_VERSION  1U

extern struct netif gnetif;          // arch/sys_arch.c

static uint8_t s_source = NET_CONFIG_SRC_DEFAULT;
static void net_mac_boot(void);

static uint32_t u32_of(const uint8_t a[4])
{
    return ((uint32_t)a[0] << 24) | ((uint32_t)a[1] << 16) | ((uint32_t)a[2] << 8) | a[3];
}

static uint16_t crc16_ccitt(const uint8_t *p, unsigned n)
{
    uint16_t crc = 0xFFFFU;
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

void net_config_default(net_config_t *c)
{
    const net_config_t d = {
        { NET_IP_ADDR0, NET_IP_ADDR1, NET_IP_ADDR2, NET_IP_ADDR3 },
        { NET_NETMASK0, NET_NETMASK1, NET_NETMASK2, NET_NETMASK3 },
        { NET_GATEWAY0, NET_GATEWAY1, NET_GATEWAY2, NET_GATEWAY3 },
    };
    *c = d;
}

bool net_config_valid(const net_config_t *c)
{
    uint32_t ip = u32_of(c->ip), mask = u32_of(c->mask), gw = u32_of(c->gw);
    uint32_t host = ~mask;

    if ((host & (host + 1U)) != 0U) return false;                 // mask not contiguous
    if (mask < 0xFF000000U || mask > 0xFFFFFFFCU) return false;   // /8 .. /30
    if (c->ip[0] == 0 || c->ip[0] == 127 || c->ip[0] >= 224) return false;
    if ((ip & host) == 0U || (ip & host) == host) return false;   // network / broadcast
    if (gw == 0U) return true;
    if ((gw & mask) != (ip & mask) || gw == ip) return false;
    if ((gw & host) == 0U || (gw & host) == host) return false;
    return true;
}

void net_config_apply(const net_config_t *c)
{
    ip4_addr_t ip, mask, gw;

    IP4_ADDR(&ip, c->ip[0], c->ip[1], c->ip[2], c->ip[3]);
    IP4_ADDR(&mask, c->mask[0], c->mask[1], c->mask[2], c->mask[3]);
    IP4_ADDR(&gw, c->gw[0], c->gw[1], c->gw[2], c->gw[3]);
    LOCK_TCPIP_CORE();
    dhcp_release_and_stop(&gnetif);             // a static address ends DHCP / LLA (no-ops if never started)
    autoip_stop(&gnetif);
    netif_set_addr(&gnetif, &ip, &mask, &gw);   // also a gratuitous ARP when the netif is up
    UNLOCK_TCPIP_CORE();
}

// DHCP (and LLA when it gets no lease, LWIP_DHCP_AUTOIP_COOP), or LLA alone. lwIP 2.1 refuses
// both on a netif that is not up ("old style port?"), so at boot it goes up here, ahead of
// task_console; without the link the clients wait and start on link-up.
static void dynamic_start(uint32_t flags)
{
    LOCK_TCPIP_CORE();
    dhcp_release_and_stop(&gnetif);
    autoip_stop(&gnetif);
    netif_set_addr(&gnetif, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4);
    if (!netif_is_up(&gnetif)) netif_set_up(&gnetif);
    if (flags & NET_IPCFG_DHCP) (void)dhcp_start(&gnetif);
    else (void)autoip_start(&gnetif);
    UNLOCK_TCPIP_CORE();
    s_source = NET_CONFIG_SRC_DYNAMIC;
    printf("[NET] Persistent IP off: %s\r\n", (flags & NET_IPCFG_DHCP) ? "DHCP, then link-local" : "link-local");
}

#define IPCFG_VERSION   1U

uint32_t net_ipcfg_get(void)
{
    uint8_t r[2];
    uint16_t n = 0;

    if (nv_get(NV_TAG_GEV_IPCFG, r, sizeof(r), &n) == 0 && n >= 2 && r[0] == IPCFG_VERSION)
        return r[1] & (NET_IPCFG_PERSISTENT | NET_IPCFG_DHCP);
    return NET_IPCFG_PERSISTENT | NET_IPCFG_DHCP;
}

int net_ipcfg_set(uint32_t flags)
{
    uint8_t r[2] = { IPCFG_VERSION, (uint8_t)(flags & (NET_IPCFG_PERSISTENT | NET_IPCFG_DHCP)) };
    return (nv_set(NV_TAG_GEV_IPCFG, r, sizeof(r)) == 0) ? 0 : 2;
}

void net_config_current(net_config_t *c)
{
    const ip4_addr_t *ip = netif_ip4_addr(&gnetif);
    const ip4_addr_t *mask = netif_ip4_netmask(&gnetif);
    const ip4_addr_t *gw = netif_ip4_gw(&gnetif);

    c->ip[0] = ip4_addr1(ip); c->ip[1] = ip4_addr2(ip); c->ip[2] = ip4_addr3(ip); c->ip[3] = ip4_addr4(ip);
    c->mask[0] = ip4_addr1(mask); c->mask[1] = ip4_addr2(mask);
    c->mask[2] = ip4_addr3(mask); c->mask[3] = ip4_addr4(mask);
    c->gw[0] = ip4_addr1(gw); c->gw[1] = ip4_addr2(gw); c->gw[2] = ip4_addr3(gw); c->gw[3] = ip4_addr4(gw);
}

bool net_config_on_link(uint32_t ip)
{
    net_config_t c;
    uint32_t mask;

    net_config_current(&c);
    mask = u32_of(c.mask);
    return (ip & mask) == (u32_of(c.ip) & mask);
}

// 0 valid record, 1 no record (erased chip or never written), 2 damaged, 3 no store.
static int read_record(net_config_t *c)
{
    uint8_t r[NET_CONFIG_BYTES];
    unsigned i;

    if (nv_read(NET_CONFIG_NV_ADDR, r, sizeof(r)) != 0) return 3;
    for (i = 0; i < sizeof(r) && r[i] == 0xFF; ++i) { }
    if (i == sizeof(r)) return 1;
    if (r[0] != NC_MAGIC || r[1] != NC_VERSION ||
        crc16_ccitt(r, 14) != (uint16_t)((r[14] << 8) | r[15]))
        return 2;
    memcpy(c->ip, r + 2, 4);
    memcpy(c->mask, r + 6, 4);
    memcpy(c->gw, r + 10, 4);
    return net_config_valid(c) ? 0 : 2;
}

uint8_t net_config_boot(void)
{
    net_config_t c;
    int rr = read_record(&c);
    uint32_t flags = net_ipcfg_get();

    net_mac_boot();
    if (!(flags & NET_IPCFG_PERSISTENT)) {
        dynamic_start(flags);
        return s_source;
    }
    if (rr == 0) {
        s_source = NET_CONFIG_SRC_STORED;
    } else {
        net_config_default(&c);
        s_source = (rr == 1) ? NET_CONFIG_SRC_DEFAULT :
                   (rr == 2) ? NET_CONFIG_SRC_DAMAGED : NET_CONFIG_SRC_NO_STORE;
    }
    net_config_apply(&c);
    printf("[NET] address %u.%u.%u.%u mask %u.%u.%u.%u gw %u.%u.%u.%u (%s)\r\n",
           c.ip[0], c.ip[1], c.ip[2], c.ip[3], c.mask[0], c.mask[1], c.mask[2], c.mask[3],
           c.gw[0], c.gw[1], c.gw[2], c.gw[3],
           s_source == NET_CONFIG_SRC_STORED ? "stored" :
           s_source == NET_CONFIG_SRC_DEFAULT ? "default, nothing stored" :
           s_source == NET_CONFIG_SRC_DAMAGED ? "default: stored record DAMAGED" : "default: store did not answer");
    return s_source;
}

bool net_config_stored(net_config_t *c)
{
    if (read_record(c) == 0) return true;
    net_config_default(c);
    return false;
}

void net_config_restart(void)
{
    net_config_t c;
    uint32_t flags = net_ipcfg_get();

    if (!(flags & NET_IPCFG_PERSISTENT)) {
        dynamic_start(flags);
        return;
    }
    s_source = net_config_stored(&c) ? NET_CONFIG_SRC_STORED : NET_CONFIG_SRC_DEFAULT;
    net_config_apply(&c);
    printf("[NET] IP configuration restarted: %u.%u.%u.%u (%s)\r\n", c.ip[0], c.ip[1], c.ip[2], c.ip[3],
           s_source == NET_CONFIG_SRC_STORED ? "stored" : "default");
}

int net_config_store(const net_config_t *c)
{
    uint8_t r[NET_CONFIG_BYTES], back[NET_CONFIG_BYTES];
    uint16_t crc;

    if (!net_config_valid(c)) return 1;
    r[0] = NC_MAGIC;
    r[1] = NC_VERSION;
    memcpy(r + 2, c->ip, 4);
    memcpy(r + 6, c->mask, 4);
    memcpy(r + 10, c->gw, 4);
    crc = crc16_ccitt(r, 14);
    r[14] = (uint8_t)(crc >> 8);
    r[15] = (uint8_t)crc;
    if (nv_write(NET_CONFIG_NV_ADDR, r, sizeof(r)) != 0 ||
        nv_read(NET_CONFIG_NV_ADDR, back, sizeof(back)) != 0 ||
        memcmp(r, back, sizeof(r)) != 0)
        return 2;
    return 0;
}

int net_config_set(const net_config_t *c, bool save)
{
    int rc;

    if (!net_config_valid(c)) return 1;
    if (save && (rc = net_config_store(c)) != 0) return rc;
    net_config_apply(c);
    s_source = NET_CONFIG_SRC_SET;
    return 0;
}

int net_config_forget(void)
{
    uint8_t r[NET_CONFIG_BYTES];
    net_config_t d;

    memset(r, 0xFF, sizeof(r));
    if (nv_write(NET_CONFIG_NV_ADDR, r, sizeof(r)) != 0) return 2;
    net_config_default(&d);
    net_config_apply(&d);
    s_source = NET_CONFIG_SRC_DEFAULT;
    return 0;
}

// ---- MAC address (record at NET_MAC_NV_ADDR, since 22.10.2026) ----------------------

#define NM_MAGIC    0x4DU   /* 'M' */

void net_mac_default(uint8_t m[6])
{
    const uint8_t d[6] = { NET_MAC_ADDR0, NET_MAC_ADDR1, NET_MAC_ADDR2,
                           NET_MAC_ADDR3, NET_MAC_ADDR4, NET_MAC_ADDR5 };
    memcpy(m, d, 6);
}

bool net_mac_valid(const uint8_t m[6])
{
    static const uint8_t zero[6] = { 0 };
    return (m[0] & 0x01U) == 0U && memcmp(m, zero, 6) != 0;   // unicast, not all zero
}

// 0 valid, 1 no record, 2 damaged, 3 no store (as read_record).
static int read_mac_record(uint8_t m[6])
{
    uint8_t r[NET_CONFIG_BYTES];
    unsigned i;

    if (nv_read(NET_MAC_NV_ADDR, r, sizeof(r)) != 0) return 3;
    for (i = 0; i < sizeof(r) && r[i] == 0xFF; ++i) { }
    if (i == sizeof(r)) return 1;
    if (r[0] != NM_MAGIC || r[1] != NC_VERSION ||
        crc16_ccitt(r, 14) != (uint16_t)((r[14] << 8) | r[15]))
        return 2;
    memcpy(m, r + 2, 6);
    return net_mac_valid(m) ? 0 : 2;
}

int net_mac_store(const uint8_t m[6])
{
    uint8_t r[NET_CONFIG_BYTES], back[NET_CONFIG_BYTES];
    uint16_t crc;

    memset(r, 0xFF, sizeof(r));
    if (m != NULL) {                          // NULL: erase the record (the default comes back)
        if (!net_mac_valid(m)) return 1;
        r[0] = NM_MAGIC;
        r[1] = NC_VERSION;
        memcpy(r + 2, m, 6);
        crc = crc16_ccitt(r, 14);
        r[14] = (uint8_t)(crc >> 8);
        r[15] = (uint8_t)crc;
    }
    if (nv_write(NET_MAC_NV_ADDR, r, sizeof(r)) != 0 ||
        nv_read(NET_MAC_NV_ADDR, back, sizeof(back)) != 0 ||
        memcmp(r, back, sizeof(r)) != 0)
        return 2;
    return 0;
}

// Before the netif is up (net_config_boot): the MAC lwIP and udp_tx send from.
static void net_mac_boot(void)
{
    uint8_t m[6];
    int rr = read_mac_record(m);

    if (rr != 0) net_mac_default(m);
    memcpy(gnetif.hwaddr, m, 6);
    printf("[NET] MAC %02X:%02X:%02X:%02X:%02X:%02X (%s)\r\n", m[0], m[1], m[2], m[3], m[4], m[5],
           rr == 0 ? "stored" : rr == 1 ? "default, nothing stored" :
           rr == 2 ? "default: stored record DAMAGED" : "default: store did not answer");
}

// ---- user-defined name (format-2 record NV_TAG_USER_NAME, since 23.10.2026) ---------

void net_name_get(char name[NET_NAME_MAX + 1])
{
    uint16_t n = 0;

    memset(name, 0, NET_NAME_MAX + 1);
    if (nv_get(NV_TAG_USER_NAME, (uint8_t *)name, NET_NAME_MAX, &n) != 0 || n > NET_NAME_MAX)
        memset(name, 0, NET_NAME_MAX + 1);   // none, or a record this firmware cannot hold
}

int net_name_store(const char *name)
{
    size_t n = strnlen(name, NET_NAME_MAX + 1);

    if (n > NET_NAME_MAX) return 1;
    return (nv_set(NV_TAG_USER_NAME, (const uint8_t *)name, (uint16_t)n) == 0) ? 0 : 2;
}
void net_config_print(void)
{
    static const char *const k_src[] = { "default", "store", "default (store record damaged)",
                                         "default (store did not answer)", "set since boot", "DHCP / link-local" };
    net_config_t c;
    int rr;

    net_config_current(&c);
    printf("[NET] now    %u.%u.%u.%u mask %u.%u.%u.%u gw %u.%u.%u.%u - %s\r\n",
           c.ip[0], c.ip[1], c.ip[2], c.ip[3], c.mask[0], c.mask[1], c.mask[2], c.mask[3],
           c.gw[0], c.gw[1], c.gw[2], c.gw[3], k_src[s_source < 6 ? s_source : 0]);
    printf("[NET] GigE Vision IP configuration: persistent %s, DHCP %s, link-local always\r\n",
           (net_ipcfg_get() & NET_IPCFG_PERSISTENT) ? "on" : "off", (net_ipcfg_get() & NET_IPCFG_DHCP) ? "on" : "off");
    rr = read_record(&c);
    if (rr == 0)
        printf("[NET] stored %u.%u.%u.%u mask %u.%u.%u.%u gw %u.%u.%u.%u\r\n",
               c.ip[0], c.ip[1], c.ip[2], c.ip[3], c.mask[0], c.mask[1], c.mask[2], c.mask[3],
               c.gw[0], c.gw[1], c.gw[2], c.gw[3]);
    else
        printf("[NET] stored - %s\r\n", rr == 1 ? "nothing" : rr == 2 ? "DAMAGED record" : "store did not answer");
    {
        uint8_t m[6];
        int mr = read_mac_record(m);
        printf("[NET] MAC now %02X:%02X:%02X:%02X:%02X:%02X, stored %s\r\n", gnetif.hwaddr[0], gnetif.hwaddr[1],
               gnetif.hwaddr[2], gnetif.hwaddr[3], gnetif.hwaddr[4], gnetif.hwaddr[5],
               mr == 0 ? "(see the next line)" : mr == 1 ? "nothing" : mr == 2 ? "DAMAGED record" : "store did not answer");
        if (mr == 0) printf("[NET] MAC stored %02X:%02X:%02X:%02X:%02X:%02X (applies at boot)\r\n",
                            m[0], m[1], m[2], m[3], m[4], m[5]);
    }
    nv_print();                              // where these records live (nvstore.h)
    net_config_default(&c);
    printf("[NET] default %u.%u.%u.%u mask %u.%u.%u.%u gw %u.%u.%u.%u ('7' erases the record and returns to it)\r\n",
           c.ip[0], c.ip[1], c.ip[2], c.ip[3], c.mask[0], c.mask[1], c.mask[2], c.mask[3],
           c.gw[0], c.gw[1], c.gw[2], c.gw[3]);
}
