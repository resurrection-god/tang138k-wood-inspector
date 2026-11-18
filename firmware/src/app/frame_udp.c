/*
 * frame_udp.c - the UDP frame delivery job for udp_tx in the FPGA (frame_udp.h).
 *
 * The control blocks (descriptor, result, stream records) are in DDR3 on bitstreams before
 * 30.10.2026 and in the FPGA control block on the extended APB since (apb_ctl.v, flash_ctl
 * FEATURES bit 1): the same layouts, written word by word through fu_addr() (frame_udp_map.h).
 * A block is built in RAM first and written out in two phases, word 0 (seq) first and the
 * last word (the seq copy) last, so the FPGA never takes a torn one. DDR3 adds the cache
 * barrier and flush of each phase; the APB is uncached (~60 ns an access), so the hot paths
 * read only the words they need.
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cache.h"
#include "spi_flash.h"
#include "frame_udp.h"
#include "frame_udp_map.h"

#define DESC_WORDS      (FRAME_UDP_DESC_BYTES / 4U)    /* 24 */
#define REC_WORDS       24U                            /* beats 0..2 of a record, the CPU's */
#define PSTAT_WORD      24U                            /* +0x60: the FPGA's block status */

static uint32_t s_seq;          /* seq of the last published descriptor */
static uint16_t s_job_id;
static bool     s_apb;          /* control blocks on the APB (FEATURES bit 1) */
// One descriptor shared by GVSP image jobs and profile resends (gvsp.c): a job holds it
// from publication to its result. VMSP 0x0224 / 0x0226 (UDPD jobs) were removed on 03.11.2026.
static SemaphoreHandle_t s_lock;
static uint8_t  s_desc[FRAME_UDP_DESC_BYTES];          /* the descriptor being built */
static uint8_t  s_rec[2][REC_WORDS * 4U];              /* the records being built */

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_le32(uint8_t *p, uint32_t v)
{
    put_le16(p, (uint16_t)v);
    put_le16(p + 2, (uint16_t)(v >> 16));
}

static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// DDR3: barrier and flush of the WHOLE cache, as in mailbox_publish(). On the board on
// 13.10.2026 a ranged flush (flush_range over 96 B) did not achieve this: with parallel 0x0300
// the FPGA picked up the descriptor 6..9 s late once every few jobs (result TIMEOUT), and even
// a 0x0306 dump of the same place (also flush_range) did not release the job, while a retry
// with a barrier and flush_all released it at once (ERRORS 3.13). APB: the barrier only.
static void publish(void)
{
    __asm__ volatile ("fence rw, rw" ::: "memory");
    if (!s_apb) ae350_dcache_flush_all();
    __asm__ volatile ("fence rw, rw" ::: "memory");
}

// Words w0 .. w1 - 1 of block blk from the image b.
static void put_words(uint32_t blk, const uint8_t *b, uint32_t w0, uint32_t w1)
{
    for (; w0 < w1; ++w0) *(volatile uint32_t *)fu_addr(s_apb, blk, w0) = get_le32(b + 4U * w0);
}

// Word w of a block the FPGA writes; DDR3: invalidate first (only the FPGA writes it,
// invariant 4) - the caller does it once for a run of words (fresh_words()).
static uint32_t get_word(uint32_t blk, uint32_t w)
{
    return *(volatile uint32_t *)fu_addr(s_apb, blk, w);
}

static void fresh_words(uint32_t blk, uint32_t w0, uint32_t n, bool cpu_wrote_line)
{
    uint32_t a = fu_addr(false, blk, w0);

    if (s_apb) return;
    // the record's status beat shares no line with the CPU's beats, but the record word 0
    // does: write back and drop rather than drop only there
    if (cpu_wrote_line) ae350_dcache_flush_range(a, 4U * n);
    else                ae350_dcache_invalidate_range(a, 4U * n);
}

void frame_udp_init(void)
{
    uint32_t feat;

    spi_flash_init();                                    /* FEATURES lives in flash_ctl */
    feat = spi_flash_fpga_features();
    s_apb = (feat & SPI_FLASH_FEAT_CTL_APB) != 0U;
    fresh_words(FU_BLK_STAT, 0, FRAME_UDP_STAT_BYTES / 4U, false);
    s_seq = (get_word(FU_BLK_STAT, 7) == FRAME_UDP_STAT_MAGIC) ? get_word(FU_BLK_STAT, 0) : 0U;

    // The CPU writes the descriptor. Zeros carry no magic; an FPGA not rebooted together with
    // the firmware will take it for a rejected job with seq 0, which is harmless.
    memset(s_desc, 0, sizeof(s_desc));
    put_words(FU_BLK_DESC, s_desc, 0, DESC_WORDS);
    publish();
    if (s_lock == NULL) s_lock = xSemaphoreCreateMutex();
}

bool frame_udp_ctl_apb(void)
{
    return s_apb;
}

// The addressing part both job kinds share: bytes 0..31 and the job id.
static void put_addr(uint8_t *d, uint8_t cam, const uint8_t dmac[6], uint16_t dport, const uint8_t smac[6],
                     uint16_t sport, const uint8_t dip[4], const uint8_t sip[4], uint32_t gap)
{
    d[4] = cam;
    memcpy(d + 8, dmac, 6);
    put_le16(d + 14, dport);
    memcpy(d + 16, smac, 6);
    put_le16(d + 22, sport);
    memcpy(d + 24, dip, 4);
    memcpy(d + 28, sip, 4);
    put_le32(d + 32, gap);
    put_le16(d + 36, s_job_id);
}

static int run_desc(uint8_t *d, uint32_t seq, frame_udp_status_t *st, uint32_t timeout_ms);

int frame_udp_gvsp_run(const frame_udp_gvsp_job_t *g, uint16_t *job_id, frame_udp_status_t *st,
                       uint32_t timeout_ms)
{
    uint8_t *d = s_desc;
    uint32_t seq;
    int rc;

    if (s_lock == NULL || xSemaphoreTake(s_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return -1;
    seq = s_seq + 1U;
    if (seq == 0U) seq = 1U;
    if (++s_job_id == 0U) s_job_id = 1U;
    *job_id = s_job_id;

    // The template first (DDR3 on every bitstream): the FPGA reads it after taking the
    // descriptor, and run_desc() publishes before the descriptor is whole.
    memcpy((void *)FRAME_UDP_GVSP_TPL_ADDR, g->tpl, sizeof(g->tpl));
    if (s_apb) ae350_dcache_flush_all();
    memset(d + 4, 0, FRAME_UDP_DESC_BYTES - 8U);
    put_le32(d + 0, seq);
    put_addr(d, g->cam, g->dst_mac, g->dst_port, g->src_mac, g->src_port, g->dst_ip, g->src_ip, g->gap);
    d[5] = FRAME_UDP_KIND_GVSP_IMAGE;
    d[7] = (uint8_t)((g->resend ? 1U : 0U) | (g->nochunk ? 2U : 0U));
    put_le16(d + 38, g->lw);
    put_le32(d + 40, g->frame_addr);
    put_le32(d + 44, g->frame_seq);
    put_le32(d + 48, FRAME_UDP_GVSP_MAGIC);
    put_le16(d + 52, g->pid0);
    put_le16(d + 54, g->pid1);
    put_le32(d + 56, g->w0);
    put_le32(d + 60, g->t);
    put_le32(d + 64, (uint32_t)g->block_id);
    put_le32(d + 68, (uint32_t)(g->block_id >> 32));
    put_le16(d + 72, g->row0);
    put_le16(d + 74, g->word0);
    put_le32(d + 76, FRAME_UDP_GVSP_TPL_ADDR);
    put_le32(d + 80, g->crc_ovr);
    put_le32(d + 84, g->rows_bad_ovr);
    put_le16(d + 88, g->ndata);
    rc = run_desc(d, seq, st, timeout_ms);
    xSemaphoreGive(s_lock);
    return rc;
}

// ---- profile streams (udp_rd.v kind 2, ARCHITECTURE § 8.1) -----------------------

// Bits [lsb, lsb + n) of the little-endian bit field at p (the record's part entries).
static void put_bits(uint8_t *p, uint32_t lsb, uint32_t n, uint32_t v)
{
    uint32_t i;

    for (i = 0; i < n; ++i, ++lsb) {
        if ((v >> i) & 1U) p[lsb >> 3] |= (uint8_t)(1U << (lsb & 7U));
        else               p[lsb >> 3] &= (uint8_t)~(1U << (lsb & 7U));
    }
}

uint32_t frame_udp_pstream_set(uint8_t cam, const frame_udp_pstream_t *p)
{
    uint32_t blk = cam ? FU_BLK_REC1 : FU_BLK_REC0;
    uint8_t *r = s_rec[cam & 1U];
    uint8_t *t = (uint8_t *)FRAME_UDP_PTPL_ADDR(cam);
    uint32_t seq, i;

    // Continue from the record the FPGA holds: it keeps the seq it saw across a firmware
    // restart, and an equal one would not be read. Only the CPU writes beats 0..2.
    fresh_words(blk, 0, 1, true);
    seq = get_word(blk, 0);
    if (++seq == 0U) seq = 1U;

    memcpy(t, p->lead, FRAME_UDP_PTPL_BYTES);            /* templates: DDR3 on every bitstream */
    memcpy(t + FRAME_UDP_PTPL_BYTES, p->trail, FRAME_UDP_PTPL_BYTES);
    if (s_apb) ae350_dcache_flush_all();
    // Phase 1: everything and seq at the start; the end stays old, so the FPGA sees it torn.
    memset(r, 0, sizeof(s_rec[0]));
    put_le32(r + 0, seq);
    r[4] = cam;
    r[5] = FRAME_UDP_KIND_GVSP_PROFILE;
    r[6] = p->nlog;
    r[7] = p->on ? 1U : 0U;
    memcpy(r + 8, p->dst_mac, 6);
    put_le16(r + 14, p->dst_port);
    memcpy(r + 16, p->src_mac, 6);
    put_le16(r + 22, p->src_port);
    memcpy(r + 24, p->dst_ip, 4);
    memcpy(r + 28, p->src_ip, 4);
    put_le32(r + 32, p->gap);
    r[36] = p->nparts;
    put_le16(r + 38, p->lw);
    put_le32(r + 40, p->ring);
    put_le32(r + 44, p->start);
    put_le32(r + 48, FRAME_UDP_PCFG_MAGIC);
    put_le32(r + 52, FRAME_UDP_PTPL_ADDR(cam));
    r[56] = p->lb;
    r[57] = p->tb;
    put_le32(r + 58, p->lenp & 0xFFFFFFU);               /* 24 bits, byte 61 stays 0 */
    for (i = 0; i < p->nparts && i < FRAME_UDP_PARTS_MAX; ++i) {
        put_bits(r + 64, 43U * i, 3U, p->kind[i]);
        put_bits(r + 64, 43U * i + 3U, 16U, p->npk[i]);
        put_bits(r + 64, 43U * i + 19U, 24U, p->off[i]);
    }
    put_le16(r + 86, p->ndata);
    put_words(blk, r, 0, REC_WORDS - 1U);
    publish();
    put_le32(r + 92, seq);                               /* phase 2: whole */
    put_words(blk, r, REC_WORDS - 1U, REC_WORDS);
    publish();
    return seq;
}

void frame_udp_pstream_status(uint8_t cam, frame_udp_pstat_t *s)
{
    uint32_t blk = cam ? FU_BLK_REC1 : FU_BLK_REC0;

    // The FPGA writes this beat; on DDR3 the line may hold record bytes the CPU wrote, so write
    // back and drop rather than drop only (invariant 4).
    fresh_words(blk, PSTAT_WORD, 8, true);
    s->valid      = (get_word(blk, PSTAT_WORD + 0U) == FRAME_UDP_PSTAT_MAGIC);
    s->next_frame = get_word(blk, PSTAT_WORD + 1U);
    s->blocks     = get_word(blk, PSTAT_WORD + 2U);
    s->lines_bad  = get_word(blk, PSTAT_WORD + 3U);
    s->skips      = get_word(blk, PSTAT_WORD + 4U) & 0xFFFFU;
    s->rec_errors = get_word(blk, PSTAT_WORD + 5U) >> 16;
    s->last_bid   = get_word(blk, PSTAT_WORD + 6U);
    s->seq        = get_word(blk, PSTAT_WORD + 7U);
}

int frame_udp_prof_resend(uint8_t cam, uint64_t block_id, uint16_t pid0, uint16_t pid1, uint16_t *job_id,
                          frame_udp_status_t *st, uint32_t timeout_ms)
{
    uint8_t *d = s_desc;
    uint32_t seq;
    int rc;

    if (s_lock == NULL || xSemaphoreTake(s_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return -1;
    seq = s_seq + 1U;
    if (seq == 0U) seq = 1U;
    if (++s_job_id == 0U) s_job_id = 1U;
    *job_id = s_job_id;
    memset(d + 4, 0, FRAME_UDP_DESC_BYTES - 8U);
    put_le32(d + 0, seq);
    d[4] = cam;
    d[5] = FRAME_UDP_KIND_GVSP_PROFILE;
    d[7] = 1U;                                           /* resend flag in the packets */
    put_le16(d + 36, s_job_id);
    put_le32(d + 48, FRAME_UDP_GVSP_MAGIC);
    put_le16(d + 52, pid0);
    put_le16(d + 54, pid1);
    put_le32(d + 64, (uint32_t)block_id);
    put_le32(d + 68, (uint32_t)(block_id >> 32));
    rc = run_desc(d, seq, st, timeout_ms);
    xSemaphoreGive(s_lock);
    return rc;
}

// Publish a filled descriptor (seq at the start already) and wait for its result. Under s_lock.
static int run_desc(uint8_t *d, uint32_t seq, frame_udp_status_t *st, uint32_t timeout_ms)
{
    uint32_t s[FRAME_UDP_STAT_BYTES / 4U], i;
    TickType_t t0, t_pub;

    put_words(FU_BLK_DESC, d, 0, DESC_WORDS - 1U);
    publish();

    // Phase 2: seq at the end; now the descriptor is whole, and everything else is
    // already where the FPGA reads it.
    put_le32(d + 92, seq);
    put_words(FU_BLK_DESC, d, DESC_WORDS - 1U, DESC_WORDS);
    publish();
    s_seq = seq;

    t0 = xTaskGetTickCount();
    t_pub = t0;
    st->republish = 0;
    for (;;) {
        fresh_words(FU_BLK_STAT, 0, FRAME_UDP_STAT_BYTES / 4U, false);
        // three words a poll: seq, magic, the seq copy the FPGA writes last
        if (get_word(FU_BLK_STAT, 0) == seq && get_word(FU_BLK_STAT, 7) == FRAME_UDP_STAT_MAGIC &&
            get_word(FU_BLK_STAT, 15) == seq) break;
        st->wait_ms = (uint32_t)(xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;
        if ((uint32_t)(xTaskGetTickCount() - t0) >= pdMS_TO_TICKS(timeout_ms)) return -1;
        // Safety net: every 20 ms without a result, seq to both ends once more. The FPGA
        // does not take a job it has already taken (same seq), so this is harmless even
        // during a long job; st->republish counts how many times.
        if ((uint32_t)(xTaskGetTickCount() - t_pub) >= pdMS_TO_TICKS(20)) {
            put_words(FU_BLK_DESC, d, 0, 1);
            put_words(FU_BLK_DESC, d, DESC_WORDS - 1U, DESC_WORDS);
            publish();
            t_pub = xTaskGetTickCount();
            ++st->republish;
        }
        vTaskDelay(1);
    }
    st->wait_ms = (uint32_t)(xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;
    for (i = 0; i < FRAME_UDP_STAT_BYTES / 4U; ++i) s[i] = get_word(FU_BLK_STAT, i);

    st->job_id      = (uint16_t)s[1];
    st->frames      = (uint16_t)(s[1] >> 16);
    st->packets     = s[2];
    st->rows_bad    = s[3];
    st->frames_bad  = (uint16_t)s[4];
    st->axi_err     = (uint16_t)(s[4] >> 16);
    st->code        = s[5];
    st->cycles      = s[6];
    st->n_jobs      = s[8];
    st->n_pkts      = s[9];
    st->n_sent      = s[10];
    st->n_stat_to   = (uint16_t)s[11];
    st->n_underrun  = (uint16_t)(s[11] >> 16);
    st->mux_stat_to = (uint16_t)s[12];
    st->mux_abort   = (uint16_t)(s[12] >> 16);
    st->mux_n_cpu   = s[13];
    st->n_bad_ctrl  = (uint16_t)s[14];
    st->max_hold_us = (uint16_t)(s[14] >> 16);
    return (st->code == 0U) ? 0 : -2;
}
