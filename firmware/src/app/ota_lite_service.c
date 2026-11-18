/*
 * ota_lite_service.c - Lightweight Bootloader Verification VMSP Service
 *
 * Implements core networking, discovery, telemetry, memory/flash dump, and Safe
 * A/B OTA bootloader VMSP opcodes (no MIPI camera, laser, or GVSP streamer):
 *   - UDP 39150: OP_PROBE (0x0001) -> OP_INFO (0x0002)
 *   - TCP 39151: Control & OTA (0x0001, 0x0170..0x017D, 0x0300, 0x0306)
 *   - TCP 39152: Data / MEM_DUMP (0x0306 -> 0x0307) for flash isolation verification
 *
 * Built only in the OTA_LITE_PROFILE target (make_ota_lite.ps1 -> ae350_ota_lite.bin).
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "platform.h"
#include "cache.h"
#include "board.h"
#include "memory_map.h"
#include "spi_flash.h"
#include "vmsp_service.h"
#include "vmsp_protocol.h"
#include "net_config.h"
#include "ota.h"
#include "lcd_hud.h"
#include "console.h"
#include "arch/sys_arch.h"
#include "arch/ethernetif.h"

#include "lwip/inet.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"

#define VCP_NODE_ID             1U
#define VCP_MAX_PAYLOAD         4096U
#define VCP_DUMP_MAX            (16U * 1024U * 1024U)
#define VCP_DUMP_CHUNK          8192U

#define VCP_OP_PROBE            0x0001U
#define VCP_OP_DISCOVERY_INFO   0x0002U
#define VCP_OP_OTA_START        0x0170U
#define VCP_OP_OTA_START_ACK    0x0171U
#define VCP_OP_OTA_DATA         0x0172U
#define VCP_OP_OTA_DATA_ACK     0x0173U
#define VCP_OP_OTA_COMMIT       0x0174U
#define VCP_OP_OTA_COMMIT_ACK   0x0175U
#define VCP_OP_OTA_REBOOT       0x0176U
#define VCP_OP_OTA_REBOOT_ACK   0x0177U
#define VCP_OP_OTA_STATUS       0x0178U
#define VCP_OP_OTA_STATUS_ACK   0x0179U
#define VCP_OP_OTA_SET_SLOT     0x017AU
#define VCP_OP_OTA_SET_SLOT_ACK 0x017BU
#define VCP_OP_OTA_VERSION      0x017CU
#define VCP_OP_OTA_VERSION_ACK  0x017DU
#define VCP_OP_STATUS           0x0300U
#define VCP_OP_MEM_DUMP         0x0306U

#define VCP_ST_BAD_VERSION      0x04U
#define VCP_ST_BAD_OPCODE       0x05U
#define VCP_ST_BAD_LENGTH       0x06U
#define VCP_ST_BAD_CRC          0x07U
#define VCP_ST_DENIED           0x09U
#define VCP_ST_UNSUPPORTED      0x0CU
#define VCP_STATUS_WORDS        39U

extern struct netif gnetif;

TaskHandle_t TASK_VMSP_CONTROL_Handler = NULL;
TaskHandle_t TASK_VMSP_VIDEO_Handler = NULL;

static uint32_t rd_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static void wr_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static void wr_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static int send_all(int fd, const uint8_t *p, size_t len)
{
    while (len > 0U) {
        int n = lwip_send(fd, p, (int)len, 0);
        if (n <= 0) return -1;
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

static int recv_exact(int fd, uint8_t *p, size_t len)
{
    while (len > 0U) {
        int n = lwip_recv(fd, p, (int)len, 0);
        if (n <= 0) return -1;
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

static void build_header(uint8_t *h, uint16_t opcode, uint32_t seq, uint32_t target,
                         uint32_t plen, uint32_t crc)
{
    memcpy(h, "VMSP", 4U);
    wr_be16(h + 4, VMSP_PROTOCOL_VERSION);
    wr_be16(h + 6, opcode);
    wr_be32(h + 8, seq);
    wr_be32(h + 12, VCP_NODE_ID);
    wr_be32(h + 16, target);
    wr_be32(h + 20, plen);
    wr_be32(h + 24, crc);
}

static int reply_pkt(int fd, const vmsp_header_t *req, uint16_t opcode,
                     const uint8_t *body, uint32_t len)
{
    uint8_t tx[VMSP_HEADER_SIZE + 256U];
    if (len > 256U) return -1;
    size_t n = vmsp_encode_packet(opcode, req->seq_num, VCP_NODE_ID, req->sender_id,
                                  body, len, tx, sizeof(tx));
    if (n == 0U) return -1;
    return send_all(fd, tx, n);
}

static int reply_status(int fd, const vmsp_header_t *req, uint8_t st)
{
    return reply_pkt(fd, req, (uint16_t)(req->opcode | 1U), &st, 1U);
}

static void fill_info_payload(uint8_t out[28])
{
    const uint8_t *mac = gnetif.hwaddr;
    net_config_t nc;
    net_config_current(&nc);
    memset(out, 0, 28U);
    out[0] = 0x54U; out[1] = 0x4DU; out[2] = 0x4CU; out[3] = 0x54U; /* "TMLT" */
    memcpy(out + 10, mac, 6U);
    memcpy(out + 16, mac, 6U);
    memcpy(out + 22, nc.ip, 4U);
    out[26] = (uint8_t)VCP_NODE_ID;
    out[27] = 0x21U; /* v2.1 lite */
}

static int handle_status(int fd, const vmsp_header_t *req, uint32_t plen)
{
    uint8_t out[VCP_STATUS_WORDS * 4U];
    uint32_t v[VCP_STATUS_WORDS];
    uint32_t recent = 0U, total = 0U, drain = 0U, speed = 0U;
    uint32_t i;

    if (plen != 0U) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    memset(v, 0, sizeof(v));
    v[2] = DEV_PLMT->MTIME[0];
    v[28] = 1U;
    eth_rx_health(&recent, &total, &drain, &speed);
    v[34] = (recent > 0xFFFFU ? 0xFFFFU : recent) | ((total > 0xFFFFU ? 0xFFFFU : total) << 16);
    v[35] = (drain > 0xFFFFU ? 0xFFFFU : drain) | ((speed > 0xFFFFU ? 0xFFFFU : speed) << 16);

    for (i = 0U; i < VCP_STATUS_WORDS; ++i) wr_be32(out + 4U * i, v[i]);
    return reply_pkt(fd, req, (uint16_t)(VCP_OP_STATUS | 1U), out, sizeof(out));
}

static int handle_mem_dump(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t hdr[VMSP_HEADER_SIZE + 12U];
    uint8_t *pre = hdr + VMSP_HEADER_SIZE;
    uint32_t addr, size, off, crc;

    memset(pre, 0, 12U);
    if (plen != 8U) {
        pre[0] = VCP_ST_BAD_LENGTH;
        return reply_pkt(fd, req, (uint16_t)(VCP_OP_MEM_DUMP | 1U), pre, 12U);
    }
    addr = rd_be32(p);
    size = rd_be32(p + 4);
    wr_be32(pre + 4, addr);
    if (size == 0U || size > VCP_DUMP_MAX) {
        pre[0] = VCP_ST_DENIED;
        return reply_pkt(fd, req, (uint16_t)(VCP_OP_MEM_DUMP | 1U), pre, 12U);
    }
    wr_be32(pre + 8, size);

    crc = vmsp_crc32_update(VMSP_CRC32_INIT, pre, 12U);
    for (off = 0U; off < size; off += VCP_DUMP_CHUNK) {
        uint32_t chunk = (size - off > VCP_DUMP_CHUNK) ? VCP_DUMP_CHUNK : (size - off);
        ae350_dcache_flush_range(addr + off, chunk);
        crc = vmsp_crc32_update(crc, (const uint8_t *)(uintptr_t)(addr + off), chunk);
    }
    build_header(hdr, (uint16_t)(VCP_OP_MEM_DUMP | 1U), req->seq_num, req->sender_id,
                 12U + size, vmsp_crc32_final(crc));
    if (send_all(fd, hdr, sizeof(hdr)) < 0) return -1;

    for (off = 0U; off < size; off += VCP_DUMP_CHUNK) {
        uint32_t chunk = (size - off > VCP_DUMP_CHUNK) ? VCP_DUMP_CHUNK : (size - off);
        ae350_dcache_flush_range(addr + off, chunk);
        if (send_all(fd, (const uint8_t *)(uintptr_t)(addr + off), chunk) < 0) return -1;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return 0;
}

static int handle_ota_start(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[16];
    uint16_t out_len = 0U;
    if (plen < 8U) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    uint8_t target = p[0];
    uint32_t total_size = rd_be32(p + 4);
    uint32_t expected_crc = (plen >= 12U) ? rd_be32(p + 8) : 0U;
    ota_start(target, total_size, expected_crc, out, &out_len);
    return reply_pkt(fd, req, VCP_OP_OTA_START_ACK, out, out_len);
}

static int handle_ota_data(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[16];
    uint16_t out_len = 0U;
    if (plen < 4U) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    ota_data(rd_be32(p), p + 4, (uint16_t)(plen - 4U), out, &out_len);
    return reply_pkt(fd, req, VCP_OP_OTA_DATA_ACK, out, out_len);
}

static int handle_ota_commit(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[16];
    uint16_t out_len = 0U;
    if (plen < 4U) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    ota_commit(rd_be32(p), out, &out_len);
    return reply_pkt(fd, req, VCP_OP_OTA_COMMIT_ACK, out, out_len);
}

static int handle_ota_reboot(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[4] = { 0 };
    uint32_t delay_ms = (plen >= 4U) ? rd_be32(p) : 100U;
    reply_pkt(fd, req, VCP_OP_OTA_REBOOT_ACK, out, 1U);
    return ota_reboot(delay_ms);
}

static int handle_ota_status(int fd, const vmsp_header_t *req)
{
    uint8_t out[64];
    uint16_t out_len = 0U;
    ota_get_status(out, &out_len);
    return reply_pkt(fd, req, VCP_OP_OTA_STATUS_ACK, out, out_len);
}

static int handle_ota_set_slot(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[8];
    uint16_t out_len = 0U;
    if (plen < 1U) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    ota_set_active_slot(p[0], out, &out_len);
    return reply_pkt(fd, req, VCP_OP_OTA_SET_SLOT_ACK, out, out_len);
}

static int handle_ota_version(int fd, const vmsp_header_t *req)
{
    uint8_t out[64];
    uint8_t slot = ota_current_slot();
    const char *ver = ota_get_version_string();
    size_t vlen = strlen(ver);
    if (vlen > 48U) vlen = 48U;
    memset(out, 0, sizeof(out));
    out[0] = 0U;
    out[1] = slot;
    out[2] = (slot == 1U) ? 2U : 1U;
    out[3] = (slot == 1U) ? 1U : 0U;
    out[4] = 0U;
    out[5] = ota_get_led_pattern();
    wr_be32(out + 6, spi_flash_build_id());
    out[10] = (uint8_t)vlen;
    memcpy(out + 11, ver, vlen);
    return reply_pkt(fd, req, VCP_OP_OTA_VERSION_ACK, out, (uint32_t)(11U + vlen));
}

static void log_to_hud(uint16_t opcode, uint32_t plen)
{
    static uint32_t s_data_cnt = 0U;
    char lmsg[44];
    if (!lcd_hud_is_active()) return;
    lcd_hud_pulse_rx();
    if (opcode == VCP_OP_OTA_DATA) {
        if ((++s_data_cnt % 32U) != 1U) return;
        snprintf(lmsg, sizeof(lmsg), "OTA_DATA chunk #%u (%uB)", (unsigned)s_data_cnt, (unsigned)plen);
        lcd_hud_log_exchange(LCD_COL_WHITEBLUE, "LITE", lmsg);
        return;
    }
    const char *name = "CMD";
    uint8_t col = LCD_COL_WHITEBLUE;
    switch (opcode) {
        case VCP_OP_PROBE:        name = "PROBE"; col = LCD_COL_CYAN; break;
        case VCP_OP_STATUS:       name = "STATUS"; col = LCD_COL_WHITEBLUE; break;
        case VCP_OP_MEM_DUMP:     name = "MEM_DUMP"; col = LCD_COL_CYAN; break;
        case VCP_OP_OTA_START:    s_data_cnt = 0U; name = "OTA_START"; col = LCD_COL_GOLD; break;
        case VCP_OP_OTA_COMMIT:   name = "OTA_COMMIT"; col = LCD_COL_GREEN; break;
        case VCP_OP_OTA_REBOOT:   name = "OTA_REBOOT"; col = LCD_COL_RED; break;
        case VCP_OP_OTA_STATUS:   name = "OTA_STATUS"; col = LCD_COL_CYAN; break;
        case VCP_OP_OTA_SET_SLOT: name = "OTA_SLOT"; col = LCD_COL_GOLD; break;
        case VCP_OP_OTA_VERSION:  name = "OTA_VER"; col = LCD_COL_CYAN; break;
        default: break;
    }
    snprintf(lmsg, sizeof(lmsg), "%s (0x%04X, %uB)", name, (unsigned)opcode, (unsigned)plen);
    lcd_hud_log_exchange(col, "LITE", lmsg);
}

static int dispatch(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    log_to_hud(req->opcode, plen);
    switch (req->opcode) {
        case VCP_OP_PROBE: {
            uint8_t info[28];
            fill_info_payload(info);
            return reply_pkt(fd, req, VCP_OP_DISCOVERY_INFO, info, sizeof(info));
        }
        case VCP_OP_STATUS:       return handle_status(fd, req, plen);
        case VCP_OP_MEM_DUMP:     return handle_mem_dump(fd, req, p, plen);
        case VCP_OP_OTA_START:    return handle_ota_start(fd, req, p, plen);
        case VCP_OP_OTA_DATA:     return handle_ota_data(fd, req, p, plen);
        case VCP_OP_OTA_COMMIT:   return handle_ota_commit(fd, req, p, plen);
        case VCP_OP_OTA_REBOOT:   return handle_ota_reboot(fd, req, p, plen);
        case VCP_OP_OTA_STATUS:   return handle_ota_status(fd, req);
        case VCP_OP_OTA_SET_SLOT: return handle_ota_set_slot(fd, req, p, plen);
        case VCP_OP_OTA_VERSION:  return handle_ota_version(fd, req);
        default:                  return reply_status(fd, req, VCP_ST_UNSUPPORTED);
    }
}

static void serve_client(int fd, bool data_port)
{
    static uint8_t rx_ctrl[VCP_MAX_PAYLOAD];
    static uint8_t rx_data[VCP_MAX_PAYLOAD];
    uint8_t *rx_buf = data_port ? rx_data : rx_ctrl;

    for (;;) {
        uint8_t hdr_raw[VMSP_HEADER_SIZE];
        vmsp_header_t hdr;
        uint32_t crc;
        if (recv_exact(fd, hdr_raw, VMSP_HEADER_SIZE) != 0) break;
        if (memcmp(hdr_raw, "VMSP", 4U) != 0) break;
        hdr.protocol_ver  = (uint16_t)((hdr_raw[4] << 8) | hdr_raw[5]);
        hdr.opcode        = (uint16_t)((hdr_raw[6] << 8) | hdr_raw[7]);
        hdr.seq_num       = rd_be32(hdr_raw + 8);
        hdr.sender_id     = rd_be32(hdr_raw + 12);
        hdr.target_id     = rd_be32(hdr_raw + 16);
        hdr.payload_len   = rd_be32(hdr_raw + 20);
        hdr.payload_crc32 = rd_be32(hdr_raw + 24);
        if (hdr.payload_len > VCP_MAX_PAYLOAD) break;
        if (hdr.payload_len > 0U && recv_exact(fd, rx_buf, hdr.payload_len) != 0) break;
        if (hdr.protocol_ver != VMSP_PROTOCOL_VERSION) {
            if (reply_status(fd, &hdr, VCP_ST_BAD_VERSION) != 0) break;
            continue;
        }
        crc = vmsp_crc32(rx_buf, hdr.payload_len);
        if (crc != hdr.payload_crc32) {
            if (reply_status(fd, &hdr, VCP_ST_BAD_CRC) != 0) break;
            continue;
        }
        if (dispatch(fd, &hdr, rx_buf, hdr.payload_len) != 0) break;
    }
    lwip_close(fd);
    if (lcd_hud_is_active()) {
        lcd_hud_set_pc_state("LISTENING (39151 / 39152)", LCD_COL_WHITEBLUE);
    }
}

void task_vmsp_discovery_entry(void *pvParameters)
{
    struct sockaddr_in bind_addr;
    uint8_t rx[256];
    uint8_t tx[VMSP_HEADER_SIZE + 28U];
    int fd;
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(1500));
    fd = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        vTaskDelete(NULL);
        return;
    }
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(VMSP_PORT_DISCOVERY);
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    if (lwip_bind(fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        lwip_close(fd);
        vTaskDelete(NULL);
        return;
    }
    for (;;) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        vmsp_packet_t pkt;
        int n = lwip_recvfrom(fd, rx, sizeof(rx), 0, (struct sockaddr *)&peer, &peer_len);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (vmsp_decode_packet(rx, (size_t)n, VCP_NODE_ID, &pkt) != VMSP_DEC_OK) continue;
        if (pkt.header.opcode == VCP_OP_PROBE) {
            uint8_t info[28];
            size_t len;
            fill_info_payload(info);
            len = vmsp_encode_packet(VCP_OP_DISCOVERY_INFO, pkt.header.seq_num,
                                     VCP_NODE_ID, pkt.header.sender_id,
                                     info, sizeof(info), tx, sizeof(tx));
            if (len > 0U) {
                lwip_sendto(fd, tx, (int)len, 0, (struct sockaddr *)&peer, peer_len);
                log_to_hud(VCP_OP_PROBE, pkt.payload_len);
            }
        }
    }
}

static void accept_port_loop(uint16_t port, bool data_port, uint32_t delay_ms)
{
    struct sockaddr_in sa;
    int listen_fd = -1;
    int on = 1;

    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    while (listen_fd < 0) {
        listen_fd = lwip_socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd >= 0) {
            lwip_setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
            memset(&sa, 0, sizeof(sa));
            sa.sin_family = AF_INET;
            sa.sin_port = htons(port);
            sa.sin_addr.s_addr = INADDR_ANY;
            if (lwip_bind(listen_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0 ||
                lwip_listen(listen_fd, 2) < 0) {
                lwip_close(listen_fd);
                listen_fd = -1;
            }
        }
        if (listen_fd < 0) vTaskDelay(pdMS_TO_TICKS(500));
    }
    printf("[LITE] VMSP %s listener ready on TCP %u\r\n", data_port ? "data" : "control", port);

    for (;;) {
        struct sockaddr_in client;
        socklen_t client_len = sizeof(client);
        struct timeval snd = { 10, 0 };
        int idle = 10, intvl = 2, cnt = 5;
        int fd = lwip_accept(listen_fd, (struct sockaddr *)&client, &client_len);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        lwip_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd));
        lwip_setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
        lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
        if (data_port) {
            lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
        }
        if (lcd_hud_is_active()) {
            char cmsg[40];
            snprintf(cmsg, sizeof(cmsg), "PC %s (%s)", inet_ntoa(client.sin_addr), data_port ? "DATA" : "LITE");
            lcd_hud_set_pc_state(cmsg, LCD_COL_GREEN);
            lcd_hud_log_exchange(LCD_COL_GREEN, "CONN", cmsg);
        }
        serve_client(fd, data_port);
    }
}

void task_vmsp_control_entry(void *pvParameters)
{
    (void)pvParameters;
    accept_port_loop(NET_VMSP_CONTROL_PORT, false, 2000U);
}

void task_vmsp_video_entry(void *pvParameters)
{
    (void)pvParameters;
    accept_port_loop(NET_VMSP_VIDEO_PORT, true, 2200U);
}

/* Lightweight stubs for task_console() in main.c */
void console_command(int cmd) { (void)cmd; }
void console_phy_boot_report(void) { }
void console_plic_dump(void) { }
