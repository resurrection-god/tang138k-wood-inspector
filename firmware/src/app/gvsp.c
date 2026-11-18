/*
 * gvsp.c - GigE Vision streaming scheduler (gvsp.h).
 *
 * A frame block on the wire (GEV 2.2 § 25.2, extended ID, payload type 0x4001): leader
 * (template body + the strobe as timestamp), payload packets of (SCPS - 48) bytes over the
 * stream "1080 x H image bytes, then 48 bytes of chunks", trailer (template body). The FPGA
 * builds the packets (udp_rd.v, GVSP job); here: which frame, the packet range, and where
 * the range starts in the stream - the division by the row length the RTL does not do.
 * block_id64 = the FPGA frame number + 1, so a resend finds its slot in the history below.
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "memory_map.h"
#include "video.h"
#include "camera.h"
#include "laser.h"
#include "cache.h"
#include "frame_udp.h"
#include "spi_flash.h"
#include "gvsp.h"
#include "gvsp_gate.h"

#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include "lwip/inet.h"

extern struct netif gnetif;                     // arch/sys_arch.c

#define ROW_WORDS       (VIDEO_FRAME_WIDTH / 4U)    /* 270 */
#define CHUNK_WORDS     (GVSP_CHUNK_BYTES / 4U)
#define HIST            32U                         /* blocks kept for resend: the ring's depth */
#define JOB_TIMEOUT_MS  2000U

typedef struct {
    uint32_t dst_ip;            // host order
    uint16_t dst_port, scps;
    uint32_t scpd;
} chcfg_t;

typedef struct {
    uint8_t  kind, ch;          // 0 start (ch = mask), 1 stop (mask), 2 resend
    uint32_t first, last;
    uint64_t bid;
} cmd_t;

typedef struct {
    bool     valid;
    uint8_t  ch;
    uint64_t bid;
    uint32_t slot, seq, h, crc, rows_bad;
} hist_t;

static QueueHandle_t s_q;
static chcfg_t  s_cfg[GVSP_CHANNELS];           // set by the GVCP task, read here (critical section)
static volatile bool s_run[GVSP_CHANNELS];
static uint32_t s_base[GVSP_CHANNELS][VIDEO_RING_SLOTS];
static uint32_t s_last[GVSP_CHANNELS];
static uint32_t s_blocks[GVSP_CHANNELS], s_resends[GVSP_CHANNELS], s_errors[GVSP_CHANNELS];
static hist_t   s_hist[HIST];
static uint32_t s_hist_i;
static frame_udp_gvsp_job_t s_job;

// Profile streams (Linescan3D): the GVCP task's settings, read here at start.
typedef struct {
    bool    profiles, mm, chunks;
    uint8_t lines, comps;
} pmode_t;
static pmode_t  s_pmode[GVSP_CHANNELS];
static bool     s_prun[GVSP_CHANNELS];          // a profile record is on
static frame_udp_pstream_t s_ps[GVSP_CHANNELS];
static uint32_t s_pnext[GVSP_CHANNELS], s_pseen_ms[GVSP_CHANNELS], s_presync[GVSP_CHANNELS];

// Guaranteed delivery (design § 5.5, B3): the gate's decisions are gvsp_gate.c's, here the
// reads they need and the actions.
typedef struct {
    bool        on;                 /* Guaranteed for the running stream */
    gvsp_gate_t g;
    bool        ev_due;
    uint64_t    ev_bid;
} gate_t;
static gate_t   s_gate[GVSP_CHANNELS];
static volatile bool     s_gmode[GVSP_CHANNELS];         /* StreamDeliveryMode, from the GVCP task */
static volatile uint64_t s_gack[GVSP_CHANNELS];          /* StreamAckedBlockId, from the GVCP task */

void gvsp_init(void)
{
    // 64: a host recovering from a stall asks many blocks at once (16 overflowed on the board, 28.10)
    if (s_q == NULL) s_q = xQueueCreate(64, sizeof(cmd_t));
}

void gvsp_channel_set(uint8_t ch, uint32_t dst_ip, uint16_t dst_port, uint16_t scps, uint32_t scpd)
{
    if (ch >= GVSP_CHANNELS) return;
    taskENTER_CRITICAL();
    s_cfg[ch].dst_ip = dst_ip;
    s_cfg[ch].dst_port = dst_port;
    s_cfg[ch].scps = scps;
    s_cfg[ch].scpd = scpd;
    taskEXIT_CRITICAL();
}

static void post(const cmd_t *c)
{
    if (s_q == NULL || xQueueSend(s_q, c, 0) != pdTRUE)
        printf("[GVSP] command queue full: %u dropped\r\n", (unsigned)c->kind);
}

void gvsp_start(uint8_t mask) { cmd_t c = { 0, mask, 0, 0, 0 }; post(&c); }
void gvsp_stop(uint8_t mask)  { cmd_t c = { 1, mask, 0, 0, 0 }; post(&c); }
bool gvsp_running(uint8_t ch)  { return ch < GVSP_CHANNELS && s_run[ch]; }

void gvsp_resend(uint8_t ch, uint64_t block_id, uint32_t first, uint32_t last)
{
    cmd_t c = { 2, ch, first, last, block_id };
    post(&c);
}

bool gvsp_frame_chunks(bool chunk_mode)
{
    // without the FPGA's no-chunk mode (B2 and older) a job of 270 H words is refused (code 2):
    // frames keep their chunks whatever ChunkModeActive says
    return chunk_mode || !(spi_flash_fpga_features() & SPI_FLASH_FEAT_FRAME_NOCHUNK);
}

uint32_t gvsp_frame_payload(uint32_t h, bool chunks)
{
    return VIDEO_FRAME_WIDTH * h + (chunks ? GVSP_CHUNK_BYTES : 0U);
}

void gvsp_profile_set(uint8_t ch, bool profiles, uint8_t lines, uint8_t comps, bool mm, bool chunks)
{
    pmode_t m = { profiles, mm, chunks, lines, comps };

    if (ch >= GVSP_CHANNELS) return;
    taskENTER_CRITICAL();
    s_pmode[ch] = m;
    taskEXIT_CRITICAL();
}

// The parts of a profile block in order (Range, Reflectance, Confidence, chunk part) and
// their bytes. udp_rd.v sends exactly these: 1080 elements a line, the chunk part being the
// payload chunk's tag, 96 B ProfileInfo a line and its own tag.
static uint8_t layout(uint8_t lines, uint8_t comps, bool mm, bool chunks, uint8_t *kind, uint32_t *bytes)
{
    uint8_t n = 0;
    uint32_t px = VIDEO_FRAME_WIDTH * lines;

    if (comps & GVSP_COMP_RANGE) {
        kind[n] = mm ? FRAME_UDP_PART_AC16 : FRAME_UDP_PART_C16;
        bytes[n++] = px * (mm ? 4U : 2U);
    }
    if (comps & GVSP_COMP_REFLECTANCE) { kind[n] = FRAME_UDP_PART_MONO8; bytes[n++] = px; }
    if (comps & GVSP_COMP_CONFIDENCE)  { kind[n] = FRAME_UDP_PART_CONF8; bytes[n++] = px; }
    if (chunks && n != 0U) { kind[n] = FRAME_UDP_PART_CHUNK; bytes[n++] = 16U + 96U * lines; }
    return n;
}

uint32_t gvsp_profile_payload(uint8_t lines, uint8_t comps, bool mm, bool chunks)
{
    uint8_t kind[FRAME_UDP_PARTS_MAX], n, i;
    uint32_t bytes[FRAME_UDP_PARTS_MAX], sum = 0;

    n = layout(lines, comps, mm, chunks, kind, bytes);
    for (i = 0; i < n; ++i) sum += bytes[i];
    return sum;
}

void gvsp_counters(uint8_t ch, uint32_t *blocks, uint32_t *resends, uint32_t *errors)
{
    if (ch >= GVSP_CHANNELS) return;
    if (s_pmode[ch].profiles) {
        frame_udp_pstat_t ps;
        frame_udp_pstream_status(ch, &ps);
        *blocks = ps.valid ? ps.blocks : 0U;
        *resends = s_resends[ch];
        *errors = s_errors[ch];
        return;
    }
    *blocks = s_blocks[ch];
    *resends = s_resends[ch];
    *errors = s_errors[ch];
}

// The next hop's MAC from the lwIP ARP table; one request and up to 200 ms otherwise (as
// the former VMSP 0x0224 peer lookup, by address instead of a socket).
static bool peer_mac(uint32_t ip_host, uint8_t ip_wire[4], uint8_t mac[6])
{
    ip4_addr_t peer, hop;
    struct eth_addr *eth = NULL;
    const ip4_addr_t *ipret = NULL;
    ssize_t idx = -1;

    peer.addr = lwip_htonl(ip_host);
    memcpy(ip_wire, &peer.addr, 4);
    hop = peer;
    if (!ip4_addr_netcmp(&peer, netif_ip4_addr(&gnetif), netif_ip4_netmask(&gnetif)))
        ip4_addr_copy(hop, *netif_ip4_gw(&gnetif));
    for (int i = 0; i < 20 && idx < 0; ++i) {
        LOCK_TCPIP_CORE();
        idx = etharp_find_addr(&gnetif, &hop, &eth, &ipret);
        if (idx >= 0) memcpy(mac, eth->addr, 6);
        else if (i == 0) (void)etharp_query(&gnetif, &hop, NULL);
        UNLOCK_TCPIP_CORE();
        if (idx < 0) vTaskDelay(pdMS_TO_TICKS(10));
    }
    return idx >= 0;
}

static void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

// The job for frame (slot, seq, h) on channel ch: addresses, packet size, template, the whole
// block as range. false: channel closed or the host's MAC unknown.
static bool job_for(uint8_t ch, uint32_t slot, uint32_t seq, uint32_t h)
{
    frame_udp_gvsp_job_t *g = &s_job;
    camera_params_t p;
    chcfg_t c;
    uint8_t *t = g->tpl;
    bool chunks;

    taskENTER_CRITICAL();
    c = s_cfg[ch];
    taskEXIT_CRITICAL();
    if (c.dst_port == 0U || c.scps < 576U) return false;
    memset(g, 0, sizeof(*g));
    if (!peer_mac(c.dst_ip, g->dst_ip, g->dst_mac)) return false;
    g->cam = ch;
    memcpy(g->src_mac, gnetif.hwaddr, 6);
    memcpy(g->src_ip, &ip_2_ip4(&gnetif.ip_addr)->addr, 4);
    g->dst_port = c.dst_port;
    g->src_port = GVSP_SRC_PORT;
    g->gap = (uint32_t)(((uint64_t)c.scpd * 10U) / 8U);         /* 10 ns ticks -> 8 ns MAC clocks */
    g->lw = (uint16_t)((c.scps - 48U) / 4U);
    g->frame_addr = slot;
    g->frame_seq = seq;
    chunks = gvsp_frame_chunks(s_pmode[ch].chunks);      /* ChunkModeActive at AcquisitionStart */
    g->nochunk = chunks ? 0U : 1U;
    g->t = ROW_WORDS * h + (chunks ? CHUNK_WORDS : 0U);
    g->ndata = (uint16_t)((g->t + g->lw - 1U) / g->lw);
    g->block_id = (uint64_t)seq + 1U;
    g->pid0 = 0;
    g->pid1 = (uint16_t)(g->ndata + 1U);
    (void)camera_cached_params(ch, &p);
    // Leader body (GEV fig. 25-2): field 0, reserved, payload type 0x4001 (0x0001 without
    // chunks), timestamp (the FPGA puts the strobe there), Mono8, size, offsets, padding.
    // Trailer body (fig. 25-4 + § 25.1): reserved, payload type, size_y, and with chunks the
    // chunk data payload length (the whole payload: image, its tag and the chunks) and the chunk
    // layout id; without chunks the FPGA sends the first two words only.
    memset(t, 0, sizeof(g->tpl));
    t[2] = chunks ? 0x40 : 0x00; t[3] = 0x01;
    put_be32(t + 12, 0x01080001U);
    put_be32(t + 16, VIDEO_FRAME_WIDTH);
    put_be32(t + 20, h);
    put_be32(t + 28, p.offset_y);
    t[38] = chunks ? 0x40 : 0x00; t[39] = 0x01;
    put_be32(t + 40, h);
    if (chunks) {
        put_be32(t + 44, gvsp_frame_payload(h, true));
        put_be32(t + 48, 1U);
    }
    return true;
}

// Where a range starting at packet first begins in the data stream.
static void job_range(uint32_t first, uint32_t last, uint32_t h)
{
    frame_udp_gvsp_job_t *g = &s_job;
    uint32_t ps = first ? first : 1U;

    if (last > g->ndata + 1U) last = g->ndata + 1U;
    g->pid0 = (uint16_t)first;
    g->pid1 = (uint16_t)last;
    g->w0 = (ps - 1U) * g->lw;
    if (g->w0 >= ROW_WORDS * h) {                        /* inside the chunks: no row */
        g->row0 = (uint16_t)h;
        g->word0 = 0;
    } else {
        g->row0 = (uint16_t)(g->w0 / ROW_WORDS);
        g->word0 = (uint16_t)(g->w0 % ROW_WORDS);
    }
}

static void send_new(uint8_t ch, uint32_t slot, uint32_t seq)
{
    frame_udp_status_t st;
    uint16_t job_id;
    uint32_t h = video_slot_height(slot);
    hist_t *e;
    int rc;

    if (!job_for(ch, slot, seq, h)) { ++s_errors[ch]; return; }
    rc = frame_udp_gvsp_run(&s_job, &job_id, &st, JOB_TIMEOUT_MS);
    if (rc != 0) {
        ++s_errors[ch];
        printf("[GVSP] ch %u frame %u: job rc %d code %u\r\n", (unsigned)ch, (unsigned)seq, rc, (unsigned)st.code);
        return;
    }
    ++s_blocks[ch];
    e = &s_hist[s_hist_i++ % HIST];
    e->valid = true;
    e->ch = ch;
    e->bid = s_job.block_id;
    e->slot = slot;
    e->seq = seq;
    e->h = h;
    e->crc = st.cycles;                                  /* GVSP: the image CRC */
    e->rows_bad = st.rows_bad;
}

// A stopped profile stream's record keeps its destination; a host that reconnected after a
// heartbeat loss opens the channel anew (design § 5.5 rule 5). false: the channel is closed.
static bool pdest(uint8_t ch)
{
    frame_udp_pstream_t *p = &s_ps[ch];
    chcfg_t c;
    uint8_t ip[4], mac[6];

    taskENTER_CRITICAL();
    c = s_cfg[ch];
    taskEXIT_CRITICAL();
    if (c.dst_port == 0U || !peer_mac(c.dst_ip, ip, mac)) return false;
    if (p->dst_port != c.dst_port || memcmp(ip, p->dst_ip, 4) != 0 || memcmp(mac, p->dst_mac, 6) != 0) {
        memcpy(p->dst_ip, ip, 4);
        memcpy(p->dst_mac, mac, 6);
        p->dst_port = c.dst_port;
        (void)frame_udp_pstream_set(ch, p);
    }
    return true;
}

static void resend(const cmd_t *c)
{
    frame_udp_status_t st;
    uint16_t job_id;
    hist_t *e = NULL;
    uint32_t i, last;
    int rc;

    if (s_pmode[c->ch].profiles) {
        // A profile block's slots follow from its id; the FPGA checks each line and sends a
        // line the ring no longer holds as zeros, flagged. The record stays after a stop.
        if (s_ps[c->ch].lw == 0U || c->bid == 0U || c->first > c->last || c->first > s_ps[c->ch].ndata + 1U) return;
        if (!s_prun[c->ch] && !pdest(c->ch)) {         /* stopped: to whoever holds the channel now */
            printf("[GVSP] ch %u: resend of block %u ignored: stopped and no destination\r\n",
                   (unsigned)c->ch, (unsigned)c->bid);
            return;
        }
        last = (c->last > s_ps[c->ch].ndata + 1U) ? s_ps[c->ch].ndata + 1U : c->last;
        ++s_resends[c->ch];
        rc = frame_udp_prof_resend(c->ch, c->bid, (uint16_t)c->first, (uint16_t)last, &job_id, &st, JOB_TIMEOUT_MS);
        if (rc != 0) {
            ++s_errors[c->ch];
            printf("[GVSP] ch %u: resend of block %u packets %u..%u failed: rc %d, status code %u\r\n",
                   (unsigned)c->ch, (unsigned)c->bid, (unsigned)c->first, (unsigned)last, rc, (unsigned)st.code);
        }
        return;
    }
    for (i = 0; i < HIST; ++i)
        if (s_hist[i].valid && s_hist[i].ch == c->ch && s_hist[i].bid == c->bid) e = &s_hist[i];
    if (e == NULL || c->first > c->last) return;         /* gone from the history: nothing to send */
    if (!job_for(c->ch, e->slot, e->seq, e->h)) return;
    if (c->first > s_job.ndata + 1U) return;
    job_range(c->first, c->last, e->h);
    s_job.resend = 1;
    s_job.crc_ovr = e->crc;
    s_job.rows_bad_ovr = e->rows_bad;
    ++s_resends[c->ch];
    if (frame_udp_gvsp_run(&s_job, &job_id, &st, JOB_TIMEOUT_MS) != 0) {
        ++s_errors[c->ch];
        if (st.code == 2U) e->valid = false;              /* the slot was overwritten */
    }
}

// ---- profile streams ------------------------------------------------------------------

static void put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v;
}

// 48-bit big-endian part length
static void put_be48(uint8_t *p, uint32_t v)
{
    put_be16(p, 0);
    put_be32(p + 2, v);
}

// The newest frame number in camera ch's telemetry block (word 1, refreshed every 0.5 s).
static uint32_t frame_now(uint8_t ch)
{
    volatile uint32_t *d = (volatile uint32_t *)VIDEO_DIAG_ADDR(ch);

    ae350_dcache_flush_range(VIDEO_DIAG_ADDR(ch), 32U);
    return d[1];
}

// The record of channel ch: parts, packets, templates (GEV 2.2 § 25.10: multi-part 0x000A,
// 0x400A with the chunk part), the first frame. false: channel closed, the host's MAC
// unknown, or no component enabled.
static bool pstream_for(uint8_t ch, const pmode_t *m, uint32_t start)
{
    static const uint16_t k_dtype[5]  = { 0x0005, 0x0005, 0x0001, 0x0009, 0x000A };   /* 3D, 3D, 2D, conf., chunk */
    static const uint32_t k_pfnc[5]   = { 0x011000B8U, 0x022000BBU, 0x01080001U, 0x010800C6U, 0 };
    static const uint16_t k_purpose[5] = { 4, 4, 5, 6, 0xFFFF };   /* ComponentIDValue: Range, Reflectance, Confidence */
    frame_udp_pstream_t *p = &s_ps[ch];
    uint32_t bytes[FRAME_UDP_PARTS_MAX], total = 0, words, nlog = 0, i;
    uint8_t *t;
    chcfg_t c;

    taskENTER_CRITICAL();
    c = s_cfg[ch];
    taskEXIT_CRITICAL();
    if (c.dst_port == 0U || c.scps < 576U) return false;
    memset(p, 0, sizeof(*p));
    if ((p->nparts = layout(m->lines, m->comps, m->mm, m->chunks, p->kind, bytes)) == 0U) return false;
    if (!peer_mac(c.dst_ip, p->dst_ip, p->dst_mac)) return false;
    while ((1U << nlog) < m->lines) ++nlog;
    p->on = true;
    p->nlog = (uint8_t)nlog;
    memcpy(p->src_mac, gnetif.hwaddr, 6);
    memcpy(p->src_ip, &ip_2_ip4(&gnetif.ip_addr)->addr, 4);
    p->dst_port = c.dst_port;
    p->src_port = GVSP_SRC_PORT;
    p->gap = (uint32_t)(((uint64_t)c.scpd * 10U) / 8U);
    p->lw = (uint16_t)((c.scps - 56U) / 4U);             /* multi-part header: 20 + 8 B */
    p->ring = DENSE_RING_ADDR(ch);
    p->start = start;
    for (i = 0; i < p->nparts; ++i) {
        words = bytes[i] / 4U;
        p->npk[i] = (uint16_t)((words + p->lw - 1U) / p->lw);
        p->off[i] = total;
        total += bytes[i];
        if (p->kind[i] != FRAME_UDP_PART_CHUNK) p->lenp += bytes[i];
        p->ndata = (uint16_t)(p->ndata + p->npk[i]);
    }
    // Leader body: reserved, payload type, timestamp (the FPGA puts line 0's strobe there),
    // then 48 B per part: data type, part length, format, source, zones, purpose, region,
    // size_x / size_y (the chunk part: zeros but type and length).
    t = p->lead;
    put_be16(t + 2, m->chunks ? 0x400AU : 0x000AU);
    for (i = 0; i < p->nparts; ++i) {
        uint8_t *e = t + 12U + 48U * i;
        uint8_t k = p->kind[i];
        put_be16(e, k_dtype[k]);
        put_be48(e + 2, bytes[i]);
        put_be32(e + 8, k_pfnc[k]);
        e[14] = (k == FRAME_UDP_PART_CHUNK) ? 0xFFU : ch;
        put_be16(e + 20, k_purpose[k]);
        put_be16(e + 22, (k == FRAME_UDP_PART_CHUNK) ? 0xFFFFU : 1U);   /* RegionSelector Scan3dExtraction0 */
        if (k != FRAME_UDP_PART_CHUNK) {
            put_be32(e + 24, VIDEO_FRAME_WIDTH);
            put_be32(e + 28, m->lines);
        }
    }
    p->lb = (uint8_t)((12U + 48U * p->nparts) / 4U);
    // Trailer body: reserved, payload type, 16 B per part (type, valid length, size_y), and in
    // extended chunk mode the payload length and the chunk layout id (§ 25.1).
    t = p->trail;
    put_be16(t + 2, m->chunks ? 0x400AU : 0x000AU);
    for (i = 0; i < p->nparts; ++i) {
        uint8_t *e = t + 4U + 16U * i;
        put_be16(e, k_dtype[p->kind[i]]);
        put_be48(e + 2, bytes[i]);
        if (p->kind[i] != FRAME_UDP_PART_CHUNK) put_be32(e + 8, m->lines);
    }
    i = 4U + 16U * p->nparts;
    if (m->chunks) {
        put_be32(t + i, total);
        put_be32(t + i + 4U, 0x100U | (m->mm ? 0x10U : 0U) | m->comps);
        i += 8U;
    }
    p->tb = (uint8_t)(i / 4U);
    return true;
}

// The first frame of a new stream: a whole block whose every frame comes after the dense
// switch (the telemetry frame may be up to 0.5 s old, it was read after the FPGA's echo).
static uint32_t pstart_frame(uint8_t ch, uint32_t lines)
{
    return (frame_now(ch) + 2U + lines - 1U) & ~(lines - 1U);
}

// After an FPGA reboot its frame numbers start from 1 again while DDR3 keeps the previous
// session's blocks (laser.h, laser_block_erase): a stale slot whose header says "frame f" is
// taken for the new frame f - a first stream after boot sent old points and failed its CRCs
// (10-174 blocks in 10-30 s, 03.11.2026; a VMSP stream before it cured it because 0x0212
// erases the whole ring, vmsp_service.c). Done before the dense mode so the FPGA is not
// writing the ring meanwhile.
static void pring_erase(uint8_t ch)
{
    uint32_t i;

    for (i = 0; i < LASER_DRY_SLOTS; ++i) laser_block_erase(LASER_DRY_BLOCK_ADDR(ch, i));
}

static bool pstart(uint8_t ch)
{
    pmode_t m;
    int rc;

    taskENTER_CRITICAL();
    m = s_pmode[ch];
    taskEXIT_CRITICAL();
    pring_erase(ch);
    rc = laser_no_frame_set((uint8_t)(1U << ch), true, LASER_ACK_TIMEOUT_MS);
    if (rc != 2) {
        printf("[GVSP] ch %u: profiles need the dense ring (no-frame echo %d)\r\n", (unsigned)ch, rc);
        ++s_errors[ch];
        return false;
    }
    if (!pstream_for(ch, &m, pstart_frame(ch, m.lines))) {
        printf("[GVSP] ch %u: profile stream not started (channel closed, host MAC, or no component)\r\n",
               (unsigned)ch);
        (void)laser_no_frame_set((uint8_t)(1U << ch), false, LASER_ACK_TIMEOUT_MS);
        ++s_errors[ch];
        return false;
    }
    taskENTER_CRITICAL();
    memset(&s_gate[ch], 0, sizeof(s_gate[ch]));
    s_gate[ch].on = s_gmode[ch];
    gvsp_gate_init(&s_gate[ch].g, s_ps[ch].start);
    s_gack[ch] = 0;
    taskEXIT_CRITICAL();
    (void)frame_udp_pstream_set(ch, &s_ps[ch]);
    s_pnext[ch] = s_ps[ch].start;
    s_pseen_ms[ch] = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_prun[ch] = true;
    printf("[GVSP] ch %u: profiles from frame %u, %u lines a block, %u parts, %u data packets%s\r\n",
           (unsigned)ch, (unsigned)s_ps[ch].start, (unsigned)m.lines, (unsigned)s_ps[ch].nparts, (unsigned)s_ps[ch].ndata,
           s_gate[ch].on ? ", guaranteed delivery" : "");
    return true;
}

// Stop: the profiles already taken go out first - wait until the FPGA's next frame has not
// moved for 50 ms (or 1 s), then the record off and the frame mode back.
static void pstop(uint8_t ch)
{
    frame_udp_pstat_t ps;
    uint32_t last = 0xFFFFFFFFU, quiet = 0, t;

    for (t = 0; t < 1000U && quiet < 50U; t += 5U) {
        frame_udp_pstream_status(ch, &ps);
        quiet = (ps.next_frame == last) ? quiet + 5U : 0U;
        last = ps.next_frame;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    s_ps[ch].on = false;
    (void)frame_udp_pstream_set(ch, &s_ps[ch]);
    s_prun[ch] = false;
    // The dense ring is not written after this: a host that comes back reads Head / Tail and
    // takes the rest by resend (design § 5.5 rule 5).
    s_gate[ch].g.gated = s_gate[ch].g.resuming = false;
    (void)laser_no_frame_set((uint8_t)(1U << ch), false, LASER_ACK_TIMEOUT_MS);
    frame_udp_pstream_status(ch, &ps);
    printf("[GVSP] ch %u: profiles stopped at frame %u: blocks %u, lines not delivered %u, skips %u, "
           "record errors %u, restarts %u\r\n", (unsigned)ch, (unsigned)ps.next_frame, (unsigned)ps.blocks,
           (unsigned)ps.lines_bad, (unsigned)ps.skips, (unsigned)ps.rec_errors, (unsigned)s_presync[ch]);
}

// A frame the FPGA waits for and that never comes (a skipped frame number) would stop the
// stream for good: when the camera is 64 frames past the FPGA's next one and that has not
// moved for 500 ms, start the record again from the current frame.
static void pwatch(uint8_t ch)
{
    frame_udp_pstat_t ps;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS, f;
    pmode_t m;

    frame_udp_pstream_status(ch, &ps);
    if (!ps.valid || ps.next_frame != s_pnext[ch] || s_gate[ch].g.gated) {   /* the gate restarts it itself */
        s_pnext[ch] = ps.valid ? ps.next_frame : s_pnext[ch];
        s_pseen_ms[ch] = now;
        return;
    }
    if (now - s_pseen_ms[ch] < 500U) return;
    f = frame_now(ch);
    if ((int32_t)(f - s_pnext[ch]) < 64) { s_pseen_ms[ch] = now; return; }
    taskENTER_CRITICAL();
    m = s_pmode[ch];
    taskEXIT_CRITICAL();
    if (!pstream_for(ch, &m, pstart_frame(ch, m.lines))) return;
    (void)frame_udp_pstream_set(ch, &s_ps[ch]);
    ++s_presync[ch];
    printf("[GVSP] ch %u: the FPGA waited at frame %u while the camera is at %u: restarted from %u\r\n",
           (unsigned)ch, (unsigned)s_pnext[ch], (unsigned)f, (unsigned)s_ps[ch].start);
    s_pnext[ch] = s_ps[ch].start;
    s_pseen_ms[ch] = now;
}

// ---- guaranteed delivery ---------------------------------------------------------------

void gvsp_delivery_set(uint8_t ch, bool guaranteed)
{
    if (ch < GVSP_CHANNELS) s_gmode[ch] = guaranteed;
}

void gvsp_ack(uint8_t ch, uint64_t block_id)
{
    if (ch >= GVSP_CHANNELS) return;
    taskENTER_CRITICAL();
    s_gack[ch] = block_id;
    taskEXIT_CRITICAL();
}

// Whether frame f's dense slot holds f (its block header; written as the frame starts).
static bool dense_has(void *ctx, uint32_t f)
{
    uint8_t ch = (uint8_t)(uintptr_t)ctx;
    uint32_t b = LASER_DRY_BLOCK_ADDR(ch, f & (LASER_DRY_SLOTS - 1U));
    volatile uint32_t *h = (volatile uint32_t *)b;

    ae350_dcache_invalidate_range(b, 32U);               /* only the FPGA writes the ring */
    return h[LASER_H_MAGIC] == LASER_BLOCK_MAGIC && h[LASER_H_FRAME_ID] == f;
}

// The newest frame in camera ch's frame ring: where frames go while the gate is shut.
static uint32_t ring_newest(void *ctx)
{
    uint8_t ch = (uint8_t)(uintptr_t)ctx;
    uint32_t s, id, best = 0;

    for (s = 0; s < VIDEO_RING_SLOTS; ++s) {
        id = video_slot_frame_id(ch, s);
        if (id != 0U && (best == 0U || (int32_t)(id - best) > 0)) best = id;
    }
    return best;
}

static void pgate(uint8_t ch)
{
    gate_t *g = &s_gate[ch];
    gvsp_gate_io_t io = { dense_has, ring_newest, (void *)(uintptr_t)ch };
    uint32_t n = 1U << s_ps[ch].nlog, act, lost;
    uint64_t ack;
    frame_udp_pstat_t ps;

    taskENTER_CRITICAL();
    ack = s_gack[ch];
    taskEXIT_CRITICAL();
    frame_udp_pstream_status(ch, &ps);
    lost = g->g.overflow;
    act = gvsp_gate_step(&g->g, n, ack, ps.valid, ps.next_frame, xTaskGetTickCount() * portTICK_PERIOD_MS, &io);
    if (act & GVSP_GATE_SHUT) {
        laser_no_frame_post((uint8_t)(1U << ch), false);
        printf("[GVSP] ch %u: ring full up to the acked frame %u: new frames held off the dense ring\r\n",
               (unsigned)ch, (unsigned)g->g.acked_end);
    }
    if (act & GVSP_GATE_EVENT) {
        taskENTER_CRITICAL();
        g->ev_due = true;
        g->ev_bid = (uint64_t)(g->g.acked_end / n) + 1U;
        taskEXIT_CRITICAL();
    }
    if (act & GVSP_GATE_FLUSH) {                         /* the block the shut cut, whole, as a resend */
        frame_udp_status_t st;
        uint16_t job_id;
        ++s_resends[ch];
        if (frame_udp_prof_resend(ch, g->g.flush_bid, 0, (uint16_t)(s_ps[ch].ndata + 1U), &job_id, &st, JOB_TIMEOUT_MS) != 0)
            ++s_errors[ch];
    }
    if (act & GVSP_GATE_OPEN) laser_no_frame_post((uint8_t)(1U << ch), true);
    if (act & GVSP_GATE_RESTART) {
        s_ps[ch].start = g->g.resume;
        (void)frame_udp_pstream_set(ch, &s_ps[ch]);
        printf("[GVSP] ch %u: dense ring again from frame %u, %u blocks lost (%u in all)\r\n",
               (unsigned)ch, (unsigned)g->g.resume, (unsigned)(g->g.overflow - lost), (unsigned)g->g.overflow);
        s_pnext[ch] = g->g.resume;
        s_pseen_ms[ch] = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
}

void gvsp_delivery_status(uint8_t ch, gvsp_delivery_t *d)
{
    frame_udp_pstat_t ps;
    uint32_t n, next, start, keep, acked;

    memset(d, 0, sizeof(*d));
    if (ch >= GVSP_CHANNELS || s_ps[ch].lw == 0U) return;
    n = 1U << s_ps[ch].nlog;
    start = s_ps[ch].start;
    frame_udp_pstream_status(ch, &ps);
    next = ps.valid ? ps.next_frame : start;
    taskENTER_CRITICAL();
    acked = s_gate[ch].g.acked_end;
    d->overflow = s_gate[ch].g.overflow;
    taskEXIT_CRITICAL();
    // shortcut: the newest frame in the dense ring is taken as the FPGA's next one, so a
    // best-effort stream that lags the camera shows a tail older than what the ring keeps.
    keep = ((int32_t)(next - start) > (int32_t)LASER_DRY_SLOTS) ? next - LASER_DRY_SLOTS + n - 1U : start;
    d->head = next / n;
    d->tail = (uint64_t)((keep & ~(n - 1U)) / n) + 1U;
    if (d->tail > d->head) d->tail = d->head;
    d->ring_free = ((int32_t)(acked + LASER_DRY_SLOTS - next) > 0) ? (acked + LASER_DRY_SLOTS - next) / n : 0U;
    d->resends = s_resends[ch];
}

bool gvsp_overflow_event(uint8_t ch, uint64_t *block_id)
{
    bool due;

    if (ch >= GVSP_CHANNELS) return false;
    taskENTER_CRITICAL();
    due = s_gate[ch].ev_due;
    s_gate[ch].ev_due = false;
    *block_id = s_gate[ch].ev_bid;
    taskEXIT_CRITICAL();
    return due;
}

static void start(uint8_t mask)
{
    uint8_t ch;

    if (!video_ring_lock(5000U)) {
        printf("[GVSP] start: ring busy\r\n");
        return;
    }
    video_invalidate_frames();                           /* the ring's frames are the stream's now */
    for (ch = 0; ch < GVSP_CHANNELS; ++ch) {
        if (!(mask & (1U << ch))) continue;
        if (s_pmode[ch].profiles) {                      /* the FPGA sends them by itself */
            if (!s_prun[ch]) (void)pstart(ch);
            continue;
        }
        video_stream_baseline(ch, s_base[ch]);
        s_last[ch] = 0;
        s_run[ch] = true;
    }
    video_ring_unlock();
    printf("[GVSP] start, channels 0x%X\r\n", (unsigned)mask);
}

void task_gvsp_entry(void *pvParameters)
{
    cmd_t c;
    uint32_t slot, id;
    TickType_t t_watch = xTaskGetTickCount();
    uint8_t ch;
    bool sent;
    (void)pvParameters;

    for (;;) {
        sent = false;
        while (s_q != NULL && xQueueReceive(s_q, &c, 0) == pdTRUE) {
            if (c.kind == 0U) start(c.ch);
            else if (c.kind == 1U) {
                for (ch = 0; ch < GVSP_CHANNELS; ++ch) {
                    if (!(c.ch & (1U << ch))) continue;
                    s_run[ch] = false;
                    if (s_prun[ch]) pstop(ch);
                }
                printf("[GVSP] stop, channels 0x%X: blocks %u/%u, resends %u/%u, errors %u/%u\r\n",
                       (unsigned)c.ch, (unsigned)s_blocks[0], (unsigned)s_blocks[1], (unsigned)s_resends[0],
                       (unsigned)s_resends[1], (unsigned)s_errors[0], (unsigned)s_errors[1]);
            } else if (c.ch < GVSP_CHANNELS) {
                resend(&c);
                sent = true;
            }
        }
        for (ch = 0; ch < GVSP_CHANNELS; ++ch) {
            if (!s_run[ch] || !video_stream_newest(ch, s_base[ch], &slot, &id) || id <= s_last[ch]) continue;
            s_last[ch] = id;
            send_new(ch, slot, id);
            sent = true;
        }
        for (ch = 0; ch < GVSP_CHANNELS; ++ch) if (s_prun[ch] && s_gate[ch].on) pgate(ch);
        if ((xTaskGetTickCount() - t_watch) >= pdMS_TO_TICKS(100)) {
            t_watch = xTaskGetTickCount();
            for (ch = 0; ch < GVSP_CHANNELS; ++ch) if (s_prun[ch]) pwatch(ch);
        }
        if (!sent) vTaskDelay(1);
    }
}
