/*
 * hik_client.c - Autonomous GigE Vision (GVCP/GVSP) client for Hikrobot cameras.
 *
 * Designed for zero dynamic allocations in the GVSP streaming hot path:
 * incoming Payload packets are downscaled on the fly to 200x120 RGB332 using
 * precomputed coordinate maps and a 2x2 Bayer dithering LUT, then submitted
 * to lcd_hud for 60 Hz RGB666 display.
 */

#include "hik_client.h"
#include "lcd_hud.h"
#include "net_config.h"
#include "ota.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include <stdio.h>
#include <string.h>

#define HIK_GVCP_PORT           3956U
#define HIK_GVCP_KEY            0x42U

/* GVCP commands & acks */
#define HIK_CMD_DISCOVERY       0x0002U
#define HIK_ACK_DISCOVERY       0x0003U
#define HIK_CMD_READREG         0x0080U
#define HIK_ACK_READREG         0x0081U
#define HIK_CMD_WRITEREG        0x0082U
#define HIK_ACK_WRITEREG        0x0083U

#define HIK_ST_SUCCESS          0x0000U
#define HIK_ST_ACCESS_DENIED    0x8006U

/* Hikrobot MV-CS004-10GM GenICam register map (Hikrobot_GigE_Vision_1_V4_0_35.xml) */
#define HIK_REG_HB_TIMEOUT      0x00000938UL
#define HIK_REG_CCP             0x00000A00UL
#define HIK_REG_SCP0            0x00000D00UL
#define HIK_REG_SCPS0           0x00000D04UL
#define HIK_REG_SCPD0           0x00000D08UL
#define HIK_REG_SCDA0           0x00000D18UL

#define HIK_REG_WIDTH           0x00030360UL
#define HIK_REG_HEIGHT          0x000303A0UL
#define HIK_REG_DECIMATION      0x00030600UL
#define HIK_REG_REVERSE_X       0x00030608UL
#define HIK_REG_REVERSE_Y       0x0003060CUL
#define HIK_REG_PIXEL_FORMAT    0x00030610UL
#define HIK_REG_ACQ_START       0x00030804UL
#define HIK_REG_ACQ_STOP        0x00030808UL
#define HIK_REG_FPS_UNIT        0x00010238UL
#define HIK_REG_FPS_CTRL        0x0003081CUL
#define HIK_REG_FPS_RESULT      0x00030824UL
#define HIK_REG_TRIGGER_MODE    0x00030840UL
#define HIK_REG_EXPOSURE_US     0x00030B04UL

#define HIK_PF_MONO8            0x01080001UL

#define HIK_NOTIF_WAKE          (1UL << 0)
#define HIK_NOTIF_RECONFIG      (1UL << 1)
#define HIK_NOTIF_EXPOSURE      (1UL << 2)

/* Task handles */
static TaskHandle_t         s_gvcp_task_h = NULL;
static TaskHandle_t         s_gvsp_task_h = NULL;

/* Runtime state */
static volatile hik_state_t s_state = HIK_STATE_DISABLED;
static volatile bool        s_enabled = false;
static volatile bool        s_decim_2x2 = false;
static volatile bool        s_auto_contrast = true;
static volatile uint32_t    s_cam_ip = 0U;          /* host order: (a<<24)|...|d */
static volatile uint32_t    s_cam_w = 0U;
static volatile uint32_t    s_cam_h = 0U;
static volatile uint32_t    s_exposure_us = 200U;
static volatile uint32_t    s_frames_ok = 0U;
static volatile uint32_t    s_frames_inc = 0U;
static volatile uint32_t    s_pkts_rx = 0U;
static volatile uint32_t    s_fps = 0U;
static volatile TickType_t  s_last_rx_tick = 0U;

/* Frame assembly & LUTs (static, zero heap allocation) */
static uint8_t              s_cam_fb[LCD_FB_SIZE];
static uint8_t              s_bayer_lut[4][256];
static uint8_t              s_stretch_lut[256];
static uint16_t             s_x_map[LCD_FB_W];
static uint16_t             s_y_map[LCD_FB_H];
static uint32_t             s_map_w = 0U;
static uint32_t             s_map_h = 0U;

/* ---------------------------------------------------------------------------
 * Endian helpers
 * ------------------------------------------------------------------------- */
static inline uint16_t rd_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static inline uint32_t rd_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)  | p[3];
}

static inline void wr_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static inline void wr_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

/* ---------------------------------------------------------------------------
 * Bayer 2x2 & Auto-Contrast LUTs + Resampler map
 * ------------------------------------------------------------------------- */
static void init_luts(void)
{
    static const int weights[4] = { 0, 2, 3, 1 };
    for (int p = 0; p < 4; ++p) {
        int d_rg = (weights[p] - 1) * 9;
        int d_bl = (weights[p] - 1) * 21;
        for (int v = 0; v < 256; ++v) {
            int rg = v + d_rg;
            int bl = v + d_bl;
            if (rg < 0) rg = 0; else if (rg > 255) rg = 255;
            if (bl < 0) bl = 0; else if (bl > 255) bl = 255;
            s_bayer_lut[p][v] = (uint8_t)((((uint32_t)rg >> 5) << 5)
                                        | (((uint32_t)rg >> 5) << 2)
                                        | ((uint32_t)bl >> 6));
        }
    }
    for (uint32_t v = 0U; v < 256U; ++v) {
        s_stretch_lut[v] = (uint8_t)v;
    }
}

static void update_stretch_lut(uint8_t lo, uint8_t hi)
{
    if (!s_auto_contrast || hi <= (uint8_t)(lo + 12U)) {
        for (uint32_t v = 0U; v < 256U; ++v) {
            s_stretch_lut[v] = (uint8_t)v;
        }
        return;
    }
    uint32_t span = (uint32_t)(hi - lo);
    for (uint32_t v = 0U; v < 256U; ++v) {
        if (v <= lo) {
            s_stretch_lut[v] = 0U;
        } else if (v >= hi) {
            s_stretch_lut[v] = 255U;
        } else {
            s_stretch_lut[v] = (uint8_t)(((v - lo) * 255U) / span);
        }
    }
}

static void update_resampler_maps(uint32_t w, uint32_t h)
{
    if (w == s_map_w && h == s_map_h) {
        return;
    }
    for (uint32_t x = 0U; x < LCD_FB_W; ++x) {
        s_x_map[x] = (uint16_t)((x * w) / LCD_FB_W);
    }
    for (uint32_t y = 0U; y < LCD_FB_H; ++y) {
        s_y_map[y] = (uint16_t)((y * h) / LCD_FB_H);
    }
    s_map_w = w;
    s_map_h = h;
}

/* ---------------------------------------------------------------------------
 * Compact 3x5 Micro-HUD Overlay (zero extra framebuffer)
 * ------------------------------------------------------------------------- */
typedef struct {
    char     ch;
    uint16_t bits; /* 15-bit 3x5 glyph, MSB = top-left */
} glyph3x5_t;

static const glyph3x5_t k_font3x5[] = {
    { '0', 0x7B6F }, { '1', 0x2492 }, { '2', 0x73E7 }, { '3', 0x73CF },
    { '4', 0x5BC9 }, { '5', 0x79CF }, { '6', 0x79EF }, { '7', 0x7249 },
    { '8', 0x7BEF }, { '9', 0x7BCF }, { 'A', 0x7B6D }, { 'B', 0x6BE6 },
    { 'C', 0x7247 }, { 'D', 0x6B6E }, { 'E', 0x79C7 }, { 'F', 0x79C4 },
    { 'G', 0x72EF }, { 'H', 0x5BED }, { 'I', 0x7497 }, { 'K', 0x5AED },
    { 'L', 0x4927 }, { 'M', 0x5FED }, { 'N', 0x7B6D }, { 'O', 0x7B6F },
    { 'P', 0x7BC4 }, { 'R', 0x7BED }, { 'S', 0x79CF }, { 'T', 0x7492 },
    { 'U', 0x5B6F }, { 'V', 0x5B6A }, { 'W', 0x5BED }, { 'X', 0x5AAD },
    { 'Y', 0x5ACA }, { ':', 0x0410 }, { '.', 0x0002 }, { '-', 0x01C0 },
    { ' ', 0x0000 }
};

static uint16_t lookup_glyph(char ch)
{
    if (ch >= 'a' && ch <= 'z') {
        ch = (char)(ch - ('a' - 'A'));
    }
    for (size_t i = 0; i < sizeof(k_font3x5) / sizeof(k_font3x5[0]); ++i) {
        if (k_font3x5[i].ch == ch) {
            return k_font3x5[i].bits;
        }
    }
    return 0x0000U;
}

static void hud_draw_badge(uint8_t *fb, const char *text, uint8_t dot_col)
{
    size_t len = strlen(text);
    uint32_t box_w = 10U + (uint32_t)len * 4U;
    if (box_w > LCD_FB_W - 4U) {
        box_w = LCD_FB_W - 4U;
    }

    for (uint32_t y = 2U; y < 9U; ++y) {
        for (uint32_t x = 2U; x < 2U + box_w; ++x) {
            fb[y * LCD_FB_W + x] = 0x00U;
        }
    }
    /* Status dot at (4..6, 4..6) */
    for (uint32_t y = 4U; y < 7U; ++y) {
        for (uint32_t x = 4U; x < 7U; ++x) {
            fb[y * LCD_FB_W + x] = dot_col;
        }
    }

    uint32_t cx = 9U;
    for (size_t i = 0; i < len && (cx + 3U) < LCD_FB_W; ++i) {
        uint16_t g = lookup_glyph(text[i]);
        for (uint32_t gy = 0U; gy < 5U; ++gy) {
            for (uint32_t gx = 0U; gx < 3U; ++gx) {
                uint32_t bit = 14U - (gy * 3U + gx);
                if ((g >> bit) & 1U) {
                    fb[(3U + gy) * LCD_FB_W + (cx + gx)] = 0xFFU;
                }
            }
        }
        cx += 4U;
    }
}

/* ---------------------------------------------------------------------------
 * GVCP Helpers
 * ------------------------------------------------------------------------- */
static uint16_t s_gvcp_req_id = 1U;

static uint16_t next_req_id(void)
{
    uint16_t id = s_gvcp_req_id++;
    if (s_gvcp_req_id == 0U) {
        s_gvcp_req_id = 1U;
    }
    return id;
}

static int udp_recv_timeout(int sock, void *buf, size_t cap,
                            struct sockaddr_in *src, uint32_t timeout_ms)
{
    fd_set rs;
    struct timeval tv;

    FD_ZERO(&rs);
    FD_SET(sock, &rs);
    tv.tv_sec = (long)(timeout_ms / 1000U);
    tv.tv_usec = (long)((timeout_ms % 1000U) * 1000U);
    if (lwip_select(sock + 1, &rs, NULL, NULL, &tv) <= 0 || !FD_ISSET(sock, &rs)) {
        return -1;
    }
    socklen_t slen = sizeof(*src);
    return lwip_recvfrom(sock, buf, cap, 0, (struct sockaddr *)src, &slen);
}

static int gvcp_transact(int sock, uint32_t cam_ip_host, uint8_t flags, uint16_t cmd,
                         const uint8_t *payload, uint16_t plen,
                         uint8_t *rx_buf, size_t rx_cap,
                         uint16_t *out_status, uint16_t *out_ack, uint16_t *out_len)
{
    uint8_t tx_buf[64];
    if (8U + plen > sizeof(tx_buf)) {
        return -1;
    }

    uint16_t rid = next_req_id();
    tx_buf[0] = HIK_GVCP_KEY;
    tx_buf[1] = flags;
    wr_be16(&tx_buf[2], cmd);
    wr_be16(&tx_buf[4], plen);
    wr_be16(&tx_buf[6], rid);
    if (plen > 0U && payload != NULL) {
        memcpy(&tx_buf[8], payload, plen);
    }

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(HIK_GVCP_PORT);
    dst.sin_addr.s_addr = htonl(cam_ip_host);

    if (lwip_sendto(sock, tx_buf, (size_t)(8U + plen), 0,
                    (struct sockaddr *)&dst, sizeof(dst)) < 0) {
        return -1;
    }

    for (int tries = 0; tries < 4; ++tries) {
        struct sockaddr_in src;
        int n = udp_recv_timeout(sock, rx_buf, rx_cap, &src, 400U);
        if (n < 8) {
            return -1;
        }
        uint16_t aid = rd_be16(&rx_buf[6]);
        if (aid == rid) {
            *out_status = rd_be16(&rx_buf[0]);
            *out_ack = rd_be16(&rx_buf[2]);
            *out_len = rd_be16(&rx_buf[4]);
            return n;
        }
    }
    return -1;
}

static int gvcp_readreg(int sock, uint32_t cam_ip_host, uint32_t addr,
                        uint32_t *out_val, uint16_t *out_status)
{
    uint8_t req[4];
    uint8_t rep[32];
    uint16_t st = 0U, ack = 0U, rlen = 0U;

    wr_be32(req, addr);
    int n = gvcp_transact(sock, cam_ip_host, 0x01U, HIK_CMD_READREG, req, 4U,
                          rep, sizeof(rep), &st, &ack, &rlen);
    if (out_status != NULL) {
        *out_status = st;
    }
    if (n < 12 || ack != HIK_ACK_READREG || st != HIK_ST_SUCCESS || rlen < 4U) {
        return -1;
    }
    *out_val = rd_be32(&rep[8]);
    return 0;
}

static int gvcp_writereg(int sock, uint32_t cam_ip_host, uint32_t addr,
                         uint32_t val, uint16_t *out_status)
{
    uint8_t req[8];
    uint8_t rep[32];
    uint16_t st = 0U, ack = 0U, rlen = 0U;

    wr_be32(&req[0], addr);
    wr_be32(&req[4], val);
    int n = gvcp_transact(sock, cam_ip_host, 0x01U, HIK_CMD_WRITEREG, req, 8U,
                          rep, sizeof(rep), &st, &ack, &rlen);
    if (out_status != NULL) {
        *out_status = st;
    }
    if (n < 8 || ack != HIK_ACK_WRITEREG || st != HIK_ST_SUCCESS) {
        return -1;
    }
    return 0;
}

static bool gvcp_discover_hikrobot(int sock, uint32_t *out_cam_ip)
{
    static uint8_t rep[320];
    uint8_t pkt[8];
    uint16_t rid = next_req_id();

    pkt[0] = HIK_GVCP_KEY;
    pkt[1] = 0x01U; /* ACK only (unicast reply, avoids broadcast-ACK filter) */
    wr_be16(&pkt[2], HIK_CMD_DISCOVERY);
    wr_be16(&pkt[4], 0U);
    wr_be16(&pkt[6], rid);

    struct sockaddr_in bcast;
    memset(&bcast, 0, sizeof(bcast));
    bcast.sin_family = AF_INET;
    bcast.sin_port = htons(HIK_GVCP_PORT);
    bcast.sin_addr.s_addr = htonl(0xFFFFFFFFUL);

    lwip_sendto(sock, pkt, sizeof(pkt), 0, (struct sockaddr *)&bcast, sizeof(bcast));

    for (int i = 0; i < 12; ++i) {
        struct sockaddr_in src;
        int n = udp_recv_timeout(sock, rep, sizeof(rep), &src, 250U);
        if (n < 0) {
            break;
        }
        if (n < 8 + 248) {
            continue;
        }
        if (rd_be16(&rep[2]) != HIK_ACK_DISCOVERY || rd_be16(&rep[6]) != rid) {
            continue;
        }
        const uint8_t *p = &rep[8];
        uint32_t cam_ip = rd_be32(&p[0x24]);
        const char *manuf = (const char *)&p[0x48];
        const char *model = (const char *)&p[0x68];
        bool is_hik = (strncmp(manuf, "Hikrobot", 8) == 0)
                   || (strncmp(manuf, "Hikvision", 9) == 0)
                   || (strncmp(model, "MV-", 3) == 0);
        if (is_hik && net_config_on_link(cam_ip)) {
            *out_cam_ip = cam_ip;
            return true;
        }
    }
    return false;
}

static void gvcp_release_camera(int sock, uint32_t cam_ip)
{
    if (cam_ip == 0U) {
        return;
    }
    (void)gvcp_writereg(sock, cam_ip, HIK_REG_ACQ_STOP, 0U, NULL);
    (void)gvcp_writereg(sock, cam_ip, HIK_REG_SCP0, 0U, NULL);
    (void)gvcp_writereg(sock, cam_ip, HIK_REG_CCP, 0U, NULL);
}

/* ---------------------------------------------------------------------------
 * GVSP Frame Receiver Task (UDP :5556)
 * ------------------------------------------------------------------------- */
static void hik_gvsp_rx_task(void *arg)
{
    static uint8_t rx_buf[1536];
    int sock;
    struct sockaddr_in addr;

    (void)arg;

    for (;;) {
        sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
        if (sock >= 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500U));
    }

    int rcvbuf = 65536;
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(HIK_GVSP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (lwip_bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        lwip_close(sock);
        vTaskDelete(NULL);
        return;
    }

    printf("[HIK_GVSP] Listening on UDP port %u\r\n", (unsigned int)HIK_GVSP_PORT);

    uint32_t cur_bid = 0U;
    bool     in_frame = false;
    uint32_t frame_w = 0U;
    uint32_t frame_h = 0U;
    uint32_t stride = 0U;
    uint32_t known_stride = 0U;
    uint32_t pkts_in_frame = 0U;
    uint8_t  cur_min = 255U;
    uint8_t  cur_max = 0U;
    uint32_t fps_frames = 0U;
    TickType_t fps_tick = xTaskGetTickCount();

    for (;;) {
        struct sockaddr_in cli;
        int n = udp_recv_timeout(sock, rx_buf, sizeof(rx_buf), &cli, 250U);
        if (n < 8) {
            continue;
        }

        uint8_t flag_b = rx_buf[4];
        bool ext_id = (flag_b & 0x80U) != 0U;
        uint8_t fmt = flag_b & 0x0FU;
        uint32_t bid = 0U;
        uint32_t pid = 0U;
        const uint8_t *body = NULL;
        uint32_t body_len = 0U;

        if (ext_id) {
            if (n < 20) continue;
            bid = rd_be32(&rx_buf[12]);
            pid = rd_be32(&rx_buf[16]);
            body = &rx_buf[20];
            body_len = (uint32_t)(n - 20);
        } else {
            bid = rd_be16(&rx_buf[2]);
            pid = ((uint32_t)rx_buf[5] << 16) | ((uint32_t)rx_buf[6] << 8) | rx_buf[7];
            body = &rx_buf[8];
            body_len = (uint32_t)(n - 8);
        }

        ++s_pkts_rx;
        s_last_rx_tick = xTaskGetTickCount();

        if (fmt == 1U) {
            /* GVSP Leader */
            if (body_len >= 24U) {
                uint32_t pf = rd_be32(&body[12]);
                uint32_t lw = rd_be32(&body[16]);
                uint32_t lh = rd_be32(&body[20]);
                if (pf == HIK_PF_MONO8 && lw > 0U && lh > 0U) {
                    if (in_frame) {
                        ++s_frames_inc;
                    }
                    cur_bid = bid;
                    in_frame = true;
                    frame_w = lw;
                    frame_h = lh;
                    stride = 0U;
                    pkts_in_frame = 0U;
                    cur_min = 255U;
                    cur_max = 0U;
                    update_resampler_maps(lw, lh);
                    s_cam_w = lw;
                    s_cam_h = lh;
                }
            }
        } else if (fmt == 3U && in_frame && bid == cur_bid && pid >= 1U) {
            /* GVSP Payload: downsample directly into s_cam_fb */
            if (pid == 1U) {
                stride = body_len;
                known_stride = body_len;
            } else if (stride == 0U) {
                stride = known_stride;
            }
            if (stride == 0U || body_len == 0U) {
                continue;
            }

            uint32_t off_start = (pid - 1U) * stride;
            uint32_t off_end = off_start + body_len;
            uint32_t frame_bytes = frame_w * frame_h;
            if (off_start >= frame_bytes) {
                continue;
            }
            if (off_end > frame_bytes) {
                off_end = frame_bytes;
            }

            uint32_t row_first = off_start / frame_w;
            uint32_t row_last = (off_end - 1U) / frame_w;

            for (uint32_t ly = 0U; ly < LCD_FB_H; ++ly) {
                uint32_t sy = s_y_map[ly];
                if (sy < row_first) {
                    continue;
                }
                if (sy > row_last) {
                    break;
                }

                uint32_t row_base = sy * frame_w;
                uint32_t col_lo = (off_start > row_base) ? (off_start - row_base) : 0U;
                uint32_t col_hi = ((off_end - row_base) < frame_w) ? (off_end - row_base) : frame_w;
                uint32_t lx = (col_lo * LCD_FB_W) / frame_w;
                while (lx < LCD_FB_W && s_x_map[lx] < col_lo) {
                    ++lx;
                }

                uint8_t *dst_row = &s_cam_fb[ly * LCD_FB_W];
                uint32_t y_par = (ly & 1U) << 1;
                for (; lx < LCD_FB_W; ++lx) {
                    uint32_t sx = s_x_map[lx];
                    if (sx >= col_hi) {
                        break;
                    }
                    uint8_t raw = body[(row_base + sx) - off_start];
                    if (raw < cur_min) cur_min = raw;
                    if (raw > cur_max) cur_max = raw;
                    uint8_t stretched = s_stretch_lut[raw];
                    dst_row[lx] = s_bayer_lut[y_par | (lx & 1U)][stretched];
                }
            }
            ++pkts_in_frame;
        } else if (fmt == 2U && in_frame && bid == cur_bid) {
            /* GVSP Trailer: finalize auto-contrast LUT, draw micro-HUD, push to LCD */
            in_frame = false;
            update_stretch_lut(cur_min, cur_max);
            ++s_frames_ok;
            ++fps_frames;

            TickType_t now = xTaskGetTickCount();
            if ((now - fps_tick) >= pdMS_TO_TICKS(1000U)) {
                s_fps = fps_frames;
                fps_frames = 0U;
                fps_tick = now;
            }

            char badge[24];
            snprintf(badge, sizeof(badge), "HIK %uFPS %s%s",
                     (unsigned int)s_fps,
                     s_decim_2x2 ? "2X2" : "1X1",
                     s_auto_contrast ? " AC" : "");
            hud_draw_badge(s_cam_fb, badge, 0x1CU);

            lcd_hud_submit_camera_frame(s_cam_fb, bid, true);
        }
    }
}

/* ---------------------------------------------------------------------------
 * GVCP Control & Heartbeat Task
 * ------------------------------------------------------------------------- */
static void hik_gvcp_task(void *arg)
{
    int sock;
    uint32_t active_cam_ip = 0U;
    bool ccp_held = false;
    bool hw_mode = false;
    uint16_t prev_hw_frames = 0U;
    uint16_t prev_hw_pkts = 0U;
    uint32_t hb_fail = 0U;

    (void)arg;

    for (;;) {
        sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
        if (sock >= 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500U));
    }

    int bcast = 1;
    lwip_setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(0U);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    (void)lwip_bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));

    /* Let Ethernet link & net_config settle before first discovery */
    vTaskDelay(pdMS_TO_TICKS(2000U));

    for (;;) {
        if (!s_enabled || !lcd_hud_is_active()) {
            if (hw_mode) {
                lcd_hud_set_hw_gvsp_mode(false, false);
                hw_mode = false;
            }
            if (ccp_held && active_cam_ip != 0U) {
                gvcp_release_camera(sock, active_cam_ip);
                ccp_held = false;
                active_cam_ip = 0U;
                s_cam_ip = 0U;
            }
            s_state = HIK_STATE_DISABLED;
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500U));
            continue;
        }

        if (active_cam_ip == 0U) {
            s_state = HIK_STATE_DISCOVER;
            uint32_t found_ip = 0U;
            if (gvcp_discover_hikrobot(sock, &found_ip)) {
                active_cam_ip = found_ip;
                s_cam_ip = found_ip;
                printf("[HIK_GVCP] Discovered Hikrobot camera at %u.%u.%u.%u\r\n",
                       (unsigned int)((found_ip >> 24) & 0xFFU),
                       (unsigned int)((found_ip >> 16) & 0xFFU),
                       (unsigned int)((found_ip >> 8) & 0xFFU),
                       (unsigned int)(found_ip & 0xFFU));
            } else {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000U));
                continue;
            }
        }

        if (!ccp_held) {
            uint16_t st = 0U;
            if (gvcp_writereg(sock, active_cam_ip, HIK_REG_CCP, 2U, &st) != 0) {
                if (st == HIK_ST_ACCESS_DENIED) {
                    if (s_state != HIK_STATE_CCP_BUSY) {
                        printf("[HIK_GVCP] Camera busy (CCP held by another host, e.g. MVS); waiting...\r\n");
                    }
                    s_state = HIK_STATE_CCP_BUSY;
                } else {
                    active_cam_ip = 0U;
                    s_cam_ip = 0U;
                }
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000U));
                continue;
            }

            ccp_held = true;
            s_state = HIK_STATE_CONFIG;
            hw_mode = lcd_hud_has_hw_gvsp();
            if (hw_mode) {
                lcd_hud_set_hw_gvsp_mode(true, true);
                uint32_t st0 = lcd_hud_get_hw_gvsp_stat();
                prev_hw_frames = (uint16_t)(st0 >> 16);
                prev_hw_pkts = (uint16_t)(st0 & 0xFFFFU);
            }

            net_config_t nc;
            net_config_current(&nc);
            uint32_t board_ip = ((uint32_t)nc.ip[0] << 24)
                              | ((uint32_t)nc.ip[1] << 16)
                              | ((uint32_t)nc.ip[2] << 8)
                              | (uint32_t)nc.ip[3];

            uint32_t old_ps = 0U;
            uint32_t exp_val = 0U;
            uint32_t fps_unit = 10000U;
            uint32_t fps_res = 0U;
            uint32_t scpd = hw_mode ? HIK_GVSP_HW_SCPD_TICKS : HIK_GVSP_SCPD_TICKS;
            uint32_t dst_port = hw_mode ? HIK_GVSP_HW_PORT : HIK_GVSP_PORT;
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_HB_TIMEOUT, 12000U, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_ACQ_STOP, 0U, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_TRIGGER_MODE, 0U, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_REVERSE_X, 0U, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_REVERSE_Y, 0U, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_PIXEL_FORMAT, HIK_PF_MONO8, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_DECIMATION,
                                s_decim_2x2 ? 0x00020002UL : 0x00010001UL, NULL);

            /* Lock camera acquisition to 60.00 Hz using XML-verified 0x0003081C */
            if (gvcp_readreg(sock, active_cam_ip, HIK_REG_FPS_UNIT, &fps_unit, NULL) != 0 || fps_unit == 0U) {
                fps_unit = 10000U;
            }
            uint32_t fps60_val = 60U * fps_unit;
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_FPS_CTRL, (1UL << 30) | (fps60_val & 0x3FFFFFFFUL), NULL);
            vTaskDelay(pdMS_TO_TICKS(10U));
            if (gvcp_readreg(sock, active_cam_ip, HIK_REG_FPS_RESULT, &fps_res, NULL) == 0 && fps_res > (65U * fps_unit)) {
                (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_FPS_CTRL, (1UL << 31) | (fps60_val & 0x3FFFFFFFUL), NULL);
                (void)gvcp_readreg(sock, active_cam_ip, HIK_REG_FPS_RESULT, &fps_res, NULL);
            }

            if (gvcp_readreg(sock, active_cam_ip, HIK_REG_EXPOSURE_US, &exp_val, NULL) == 0) {
                s_exposure_us = exp_val;
            }
            if (gvcp_readreg(sock, active_cam_ip, HIK_REG_SCPS0, &old_ps, NULL) != 0) {
                old_ps = 0U;
            }
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_SCPS0,
                                (old_ps & 0xFFFF0000UL) | HIK_GVSP_SCPS_BYTES, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_SCPD0, scpd, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_SCDA0, board_ip, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_SCP0, dst_port, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_ACQ_START, 0x80000000UL, NULL);

            lcd_hud_set_link_status(true, active_cam_ip);
            s_last_rx_tick = xTaskGetTickCount();
            hb_fail = 0U;
            s_state = HIK_STATE_STREAMING;
            printf("[HIK_GVCP] Stream started [%s] -> %u.%u.%u.%u:%u (SCPS=%u, SCPD=%u, FPS=%u.%02u, Decim=%s)\r\n",
                   hw_mode ? "HW-RTL" : "SW-AE350",
                   nc.ip[0], nc.ip[1], nc.ip[2], nc.ip[3], (unsigned int)dst_port,
                   (unsigned int)HIK_GVSP_SCPS_BYTES, (unsigned int)scpd,
                   (unsigned int)(fps_res / fps_unit),
                   (unsigned int)((fps_res % fps_unit) / (fps_unit >= 100U ? fps_unit / 100U : 1U)),
                   s_decim_2x2 ? "2x2" : "1x1");
        }

        uint32_t notif = 0U;
        xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notif, pdMS_TO_TICKS(1000U));

        if (!s_enabled) {
            continue;
        }
        if (notif & HIK_NOTIF_RECONFIG) {
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_ACQ_STOP, 0U, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_DECIMATION,
                                s_decim_2x2 ? 0x00020002UL : 0x00010001UL, NULL);
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_ACQ_START, 0x80000000UL, NULL);
            s_last_rx_tick = xTaskGetTickCount();
            printf("[HIK_GVCP] Reconfigured Decimation=%s\r\n", s_decim_2x2 ? "2x2" : "1x1");
        }
        if (notif & HIK_NOTIF_EXPOSURE) {
            (void)gvcp_writereg(sock, active_cam_ip, HIK_REG_EXPOSURE_US, s_exposure_us, NULL);
            printf("[HIK_GVCP] Exposure=%u us\r\n", (unsigned int)s_exposure_us);
        }

        uint32_t ccp_val = 0U;
        int ccp_ok = 0;
        for (int tries = 0; tries < 3; ++tries) {
            if (gvcp_readreg(sock, active_cam_ip, HIK_REG_CCP, &ccp_val, NULL) == 0 && (ccp_val & 3U) != 0U) {
                ccp_ok = 1;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(40U));
        }
        if (!ccp_ok) {
            ++hb_fail;
        } else {
            hb_fail = 0U;
        }

        if (hw_mode) {
            uint32_t hst = lcd_hud_get_hw_gvsp_stat();
            uint16_t hf = (uint16_t)(hst >> 16);
            uint16_t hp = (uint16_t)(hst & 0xFFFFU);
            uint16_t df = (uint16_t)(hf - prev_hw_frames);
            uint16_t dp = (uint16_t)(hp - prev_hw_pkts);
            prev_hw_frames = hf;
            prev_hw_pkts = hp;
            s_fps = df;
            s_frames_ok += df;
            s_pkts_rx += dp;
            if (dp > 0U) {
                s_last_rx_tick = xTaskGetTickCount();
            }
        }

        TickType_t rx_age = xTaskGetTickCount() - s_last_rx_tick;
        if (hb_fail >= 6U || rx_age > pdMS_TO_TICKS(6000U)) {
            printf("[HIK_GVCP] Stream stalled or heartbeat lost (hb_fail=%u, rx_age=%u ms); reconnecting...\r\n",
                   (unsigned int)hb_fail, (unsigned int)(rx_age * portTICK_PERIOD_MS));
            lcd_hud_set_link_status(false, active_cam_ip);
            gvcp_release_camera(sock, active_cam_ip);
            ccp_held = false;
            active_cam_ip = 0U;
            s_cam_ip = 0U;
            s_fps = 0U;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */
void hik_client_init(void)
{
    if (!lcd_hud_is_active()) {
        s_enabled = false;
        s_state = HIK_STATE_DISABLED;
        return;
    }

    init_luts();
    memset(s_cam_fb, 0, sizeof(s_cam_fb));
    s_enabled = (ota_current_slot() == 1U);

    xTaskCreate(hik_gvsp_rx_task, "hik_gvsp", 1024U, NULL, tskIDLE_PRIORITY + 3U, &s_gvsp_task_h);
    xTaskCreate(hik_gvcp_task,    "hik_gvcp", 1024U, NULL, tskIDLE_PRIORITY + 1U, &s_gvcp_task_h);
    printf("[HIK] Client initialized (auto=%s, GVSP UDP :%u)\r\n",
           s_enabled ? "ON" : "OFF", (unsigned int)HIK_GVSP_PORT);
}

void hik_client_set_enabled(bool en)
{
    s_enabled = en;
    if (s_gvcp_task_h != NULL) {
        xTaskNotify(s_gvcp_task_h, HIK_NOTIF_WAKE, eSetBits);
    }
}

bool hik_client_is_enabled(void)
{
    return s_enabled;
}

void hik_client_toggle_enabled(void)
{
    hik_client_set_enabled(!s_enabled);
    printf("[HIK] Client %s\r\n", s_enabled ? "ENABLED" : "DISABLED");
}

void hik_client_toggle_decim(void)
{
    s_decim_2x2 = !s_decim_2x2;
    if (s_gvcp_task_h != NULL && s_state == HIK_STATE_STREAMING) {
        xTaskNotify(s_gvcp_task_h, HIK_NOTIF_RECONFIG, eSetBits);
    }
    printf("[HIK] Decimation -> %s\r\n", s_decim_2x2 ? "2x2" : "1x1");
}

void hik_client_toggle_auto_contrast(void)
{
    s_auto_contrast = !s_auto_contrast;
    printf("[HIK] Auto-contrast -> %s\r\n", s_auto_contrast ? "ON" : "OFF");
}

void hik_client_adjust_exposure(int32_t delta_us)
{
    int32_t next = (int32_t)s_exposure_us + delta_us;
    if (next < 20) next = 20;
    if (next > 50000) next = 50000;
    s_exposure_us = (uint32_t)next;
    if (s_gvcp_task_h != NULL && s_state == HIK_STATE_STREAMING) {
        xTaskNotify(s_gvcp_task_h, HIK_NOTIF_EXPOSURE, eSetBits);
    }
}

void hik_client_get_stats(hik_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    out->state = s_state;
    out->enabled = s_enabled;
    out->decim_2x2 = s_decim_2x2;
    out->auto_contrast = s_auto_contrast;
    out->cam_ip = s_cam_ip;
    out->cam_w = s_cam_w;
    out->cam_h = s_cam_h;
    out->exposure_us = s_exposure_us;
    out->frames_ok = s_frames_ok;
    out->frames_incomplete = s_frames_inc;
    out->pkts_rx = s_pkts_rx;
    out->fps = s_fps;
}

void hik_client_print_status(void)
{
    hik_stats_t st;
    hik_client_get_stats(&st);
    printf("[HIK] State=%d Enabled=%u Cam=%u.%u.%u.%u (%ux%u) Decim=%s AC=%u Exp=%uus FPS=%u OK=%u Inc=%u Pkts=%u\r\n",
           (int)st.state,
           st.enabled ? 1U : 0U,
           (unsigned int)((st.cam_ip >> 24) & 0xFFU),
           (unsigned int)((st.cam_ip >> 16) & 0xFFU),
           (unsigned int)((st.cam_ip >> 8) & 0xFFU),
           (unsigned int)(st.cam_ip & 0xFFU),
           (unsigned int)st.cam_w, (unsigned int)st.cam_h,
           st.decim_2x2 ? "2x2" : "1x1",
           st.auto_contrast ? 1U : 0U,
           (unsigned int)st.exposure_us,
           (unsigned int)st.fps,
           (unsigned int)st.frames_ok,
           (unsigned int)st.frames_incomplete,
           (unsigned int)st.pkts_rx);
}
