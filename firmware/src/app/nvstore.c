/*
 * nvstore.c - settings store in SPI flash (nvstore.h): format 2 (tag-length-value in a 4 KB
 * sector per copy) with the legacy 1 KB space mirrored into format 1.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "spi_flash.h"
#include "nvstore.h"

#define NV1_MAGIC   0x3153564EU     /* "NVS1" little-endian in memory */
#define NV2_MAGIC   0x3253564EU     /* "NVS2" */

typedef struct {
    uint32_t magic, seq, len, crc;  // CRC-32 of seq + data
} nv1_hdr_t;

typedef struct {
    uint32_t magic, seq, len, crc;  // CRC-32 of seq, legacy_seq, len and data[len]
    uint32_t legacy_seq;            // the format-1 seq this image's legacy record equals
} nv2_hdr_t;

// Format 2 data: records {u16 tag, u16 len, data, padded to 4}, back to back.
#define NV2_MAX     (SPI_FLASH_SECTOR - sizeof(nv2_hdr_t))
#define REC_HDR     4U
#define PAD4(n)     (((n) + 3U) & ~3U)

static SemaphoreHandle_t s_lock;
static bool     s_loaded;
static bool     s_present;          // flash_ctl.v in the bitstream: the store exists
static uint8_t  s_img[NV_SIZE];     // legacy space as it is now
static uint8_t  s_tmp[NV_SIZE];     // legacy space being written
static uint8_t  s_v2[NV2_MAX];      // format-2 records as they are now
static uint8_t  s_v2tmp[NV2_MAX];   // ... being written
static uint32_t s_v2len;
static uint32_t s_seq1, s_seq2;     // newest copies; 0 = none
static int      s_slot1 = -1, s_slot2 = -1;
static uint32_t s_legacy_seq;       // of the newest format-2 image
static uint32_t s_writes, s_errors;

static const uint32_t k_slot1[2] = { NV_FLASH_A, NV_FLASH_B };
static const uint32_t k_slot2[2] = { NV2_FLASH_A, NV2_FLASH_B };

static uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t n)
{
    crc = ~crc;
    while (n--) {
        crc ^= *p++;
        for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

static uint32_t crc1(uint32_t seq, const uint8_t *img)
{
    return crc32_update(crc32_update(0, (const uint8_t *)&seq, 4), img, NV_SIZE);
}

static uint32_t crc2(const nv2_hdr_t *h, const uint8_t *data)
{
    uint32_t c = crc32_update(0, (const uint8_t *)&h->seq, 4);
    c = crc32_update(c, (const uint8_t *)&h->legacy_seq, 4);
    c = crc32_update(c, (const uint8_t *)&h->len, 4);
    return crc32_update(c, data, h->len);
}

// ---- records ------------------------------------------------------------------

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

// Offset of the record's data, or -1; *n its length. Stops at a malformed record.
static int rec_find(const uint8_t *img, uint32_t len, uint16_t tag, uint16_t *n)
{
    uint32_t off = 0, rl;

    while (off + REC_HDR <= len) {
        rl = rd16(img + off + 2);
        if (off + REC_HDR + rl > len) return -1;
        if (rd16(img + off) == tag) { *n = (uint16_t)rl; return (int)(off + REC_HDR); }
        off += REC_HDR + PAD4(rl);
    }
    return -1;
}

// dst = src without `tag`, then `tag` = data (len 0: deleted). Returns the new length, 0 if too big.
static uint32_t rec_put(uint8_t *dst, const uint8_t *src, uint32_t len, uint16_t tag,
                        const uint8_t *data, uint16_t dlen)
{
    uint32_t off = 0, out = 0, rl;

    while (off + REC_HDR <= len) {
        rl = rd16(src + off + 2);
        if (off + REC_HDR + rl > len) break;
        if (rd16(src + off) != tag) {
            memcpy(dst + out, src + off, REC_HDR + PAD4(rl));
            out += REC_HDR + PAD4(rl);
        }
        off += REC_HDR + PAD4(rl);
    }
    if (dlen == 0U) return out;
    if (out + REC_HDR + PAD4(dlen) > NV2_MAX) return 0;
    wr16(dst + out, tag);
    wr16(dst + out + 2, dlen);
    memset(dst + out + REC_HDR, 0, PAD4(dlen));
    memcpy(dst + out + REC_HDR, data, dlen);
    return out + REC_HDR + PAD4(dlen);
}

// ---- copies ---------------------------------------------------------------------

static uint32_t read_slot1_once(int k, uint8_t *out, bool *magic)
{
    nv1_hdr_t h;

    if (spi_flash_read(k_slot1[k], (uint8_t *)&h, sizeof(h)) != SPI_FLASH_OK) return 0;
    *magic = (h.magic == NV1_MAGIC);
    if (h.magic != NV1_MAGIC || h.len != NV_SIZE || h.seq == 0U) return 0;
    if (spi_flash_read(k_slot1[k] + sizeof(h), out, NV_SIZE) != SPI_FLASH_OK) return 0;
    return crc1(h.seq, out) == h.crc ? h.seq : 0U;
}

static uint32_t read_slot2_once(int k, uint8_t *out, uint32_t *len, uint32_t *legacy_seq, bool *magic)
{
    nv2_hdr_t h;

    if (spi_flash_read(k_slot2[k], (uint8_t *)&h, sizeof(h)) != SPI_FLASH_OK) return 0;
    *magic = (h.magic == NV2_MAGIC);
    if (h.magic != NV2_MAGIC || h.len > NV2_MAX || h.seq == 0U) return 0;
    if (h.len && spi_flash_read(k_slot2[k] + sizeof(h), out, h.len) != SPI_FLASH_OK) return 0;
    if (crc2(&h, out) != h.crc) return 0;
    *len = h.len;
    *legacy_seq = h.legacy_seq;
    return h.seq;
}

// A copy that carries its magic and still fails is read once more: a misread is not a damaged
// copy (ERRORS 5.15). Damaged on both reads: the alarm - the other, older copy will be used.
static uint32_t read_slot1(int k, uint8_t *out)
{
    bool magic = false;
    uint32_t seq = read_slot1_once(k, out, &magic);

    if (seq == 0U && magic && (seq = read_slot1_once(k, out, &magic)) == 0U) {
        spi_flash_alarm_set(SPI_FLASH_ALARM_SETTINGS);
        printf("[NV] ALARM: format-1 copy %c damaged on two reads\r\n", 'A' + k);
    }
    return seq;
}

static uint32_t read_slot2(int k, uint8_t *out, uint32_t *len, uint32_t *legacy_seq)
{
    bool magic = false;
    uint32_t seq = read_slot2_once(k, out, len, legacy_seq, &magic);

    if (seq == 0U && magic && (seq = read_slot2_once(k, out, len, legacy_seq, &magic)) == 0U) {
        spi_flash_alarm_set(SPI_FLASH_ALARM_SETTINGS);
        printf("[NV] ALARM: format-2 copy %c damaged on two reads\r\n", 'A' + k);
    }
    return seq;
}

// Erase the sector not holding the newest copy, data first, header last.
static int write_sector(uint32_t addr, const void *hdr, uint32_t hlen, const uint8_t *data, uint32_t dlen)
{
    int rc = spi_flash_erase(addr, SPI_FLASH_SECTOR);
    if (rc == SPI_FLASH_OK && dlen) rc = spi_flash_write(addr + hlen, data, dlen);
    if (rc == SPI_FLASH_OK) rc = spi_flash_write(addr, (const uint8_t *)hdr, hlen);
    ++s_writes;
    if (rc != SPI_FLASH_OK) ++s_errors;
    return rc;
}

static int write1(const uint8_t *img)
{
    int k = (s_slot1 == 0) ? 1 : 0;
    nv1_hdr_t h = { NV1_MAGIC, s_seq1 + 1U, NV_SIZE, 0 };
    int rc;

    h.crc = crc1(h.seq, img);
    if ((rc = write_sector(k_slot1[k], &h, sizeof(h), img, NV_SIZE)) != SPI_FLASH_OK) {
        printf("[NV] format-1 copy %c write failed: rc %d\r\n", 'A' + k, rc);
        return rc;
    }
    s_seq1 = h.seq;
    s_slot1 = k;
    return 0;
}

static int write2(const uint8_t *data, uint32_t len, uint32_t legacy_seq)
{
    int k = (s_slot2 == 0) ? 1 : 0;
    nv2_hdr_t h = { NV2_MAGIC, s_seq2 + 1U, len, 0, legacy_seq };
    int rc;

    h.crc = crc2(&h, data);
    if ((rc = write_sector(k_slot2[k], &h, sizeof(h), data, len)) != SPI_FLASH_OK) {
        printf("[NV] format-2 copy %c write failed: rc %d\r\n", 'A' + k, rc);
        return rc;
    }
    s_seq2 = h.seq;
    s_slot2 = k;
    s_legacy_seq = legacy_seq;
    return 0;
}

// The legacy space into format 2 as a record, stamped with the format-1 seq it equals.
static int put_legacy(const uint8_t *img)
{
    uint32_t n = rec_put(s_v2tmp, s_v2, s_v2len, NV_TAG_LEGACY, img, NV_SIZE);
    int rc;

    if (n == 0U) return -1;
    if ((rc = write2(s_v2tmp, n, s_seq1)) != 0) return rc;
    memcpy(s_v2, s_v2tmp, n);
    s_v2len = n;
    return 0;
}

// Under s_lock.
static void load(void)
{
    uint32_t sa, sb, la = 0, lb = 0, lsa = 0, lsb = 0;
    uint16_t n = 0;
    int off;

    s_loaded = true;
    s_present = spi_flash_present();
    if (!s_present) {
        printf("[NV] no flash_ctl in this bitstream: no settings store, defaults only\r\n");
        return;
    }
    // Format 1: the legacy space as firmware without format 2 last left it.
    sa = read_slot1(0, s_img);
    sb = read_slot1(1, s_tmp);
    if (sb > sa) { memcpy(s_img, s_tmp, NV_SIZE); s_seq1 = sb; s_slot1 = 1; }
    else if (sa != 0U) { s_seq1 = sa; s_slot1 = 0; }
    else memset(s_img, 0xFF, NV_SIZE);
    // Format 2.
    sa = read_slot2(0, s_v2, &la, &lsa);
    sb = read_slot2(1, s_v2tmp, &lb, &lsb);
    if (sb > sa) { memcpy(s_v2, s_v2tmp, lb); s_v2len = lb; s_seq2 = sb; s_slot2 = 1; s_legacy_seq = lsb; }
    else if (sa != 0U) { s_v2len = la; s_seq2 = sa; s_slot2 = 0; s_legacy_seq = lsa; }

    if (s_seq2 == 0U) {
        // First boot with format 2: the legacy space moves in (format 1 stays as the mirror).
        s_v2len = 0;
        if (s_seq1 != 0U && put_legacy(s_img) == 0)
            printf("[NV] format 2 created from format-1 seq %u\r\n", (unsigned)s_seq1);
        else if (s_seq1 == 0U)
            printf("[NV] no copy in flash: empty store, defaults until the first save\r\n");
    } else if (s_seq1 != 0U && s_seq1 != s_legacy_seq) {
        // Format 1 was written by firmware without format 2 (a rollback) since: it is newer.
        printf("[NV] format 1 changed since (seq %u, mirrored %u): legacy space taken from it\r\n",
               (unsigned)s_seq1, (unsigned)s_legacy_seq);
        (void)put_legacy(s_img);
    } else if ((off = rec_find(s_v2, s_v2len, NV_TAG_LEGACY, &n)) >= 0 && n == NV_SIZE) {
        memcpy(s_img, s_v2 + off, NV_SIZE);
    }
    printf("[NV] settings in flash: format 2 copy %c seq %u (%u B of records), format 1 copy %c seq %u\r\n",
           s_slot2 >= 0 ? 'A' + s_slot2 : '-', (unsigned)s_seq2, (unsigned)s_v2len,
           s_slot1 >= 0 ? 'A' + s_slot1 : '-', (unsigned)s_seq1);
}

static bool take(void)
{
    if (s_lock == NULL || xSemaphoreTake(s_lock, pdMS_TO_TICKS(5000)) != pdTRUE) return false;
    if (!s_loaded) load();
    return true;
}

void nv_init(void)
{
    if (s_lock == NULL) s_lock = xSemaphoreCreateMutex();
}

int nv_read(uint16_t addr, uint8_t *buf, uint16_t len)
{
    int rc = -1;                              // no flash_ctl: no store, callers fall back to defaults

    if ((uint32_t)addr + len > NV_SIZE) return -1;
    if (!take()) return -1;
    if (s_present) {
        memcpy(buf, s_img + addr, len);
        rc = 0;
    }
    xSemaphoreGive(s_lock);
    return rc;
}

// Format 1 first (the mirror), then format 2 stamped with its seq. A failure of the second
// leaves format 1 newer than the stamp: the next boot takes the legacy space from it.
int nv_write(uint16_t addr, const uint8_t *buf, uint16_t len)
{
    int rc = -1;

    if ((uint32_t)addr + len > NV_SIZE) return -1;
    if (!take()) return -1;
    if (s_present) {
        memcpy(s_tmp, s_img, NV_SIZE);
        memcpy(s_tmp + addr, buf, len);
        rc = write1(s_tmp);
        if (rc == 0) {
            memcpy(s_img, s_tmp, NV_SIZE);
            rc = put_legacy(s_img);
        }
    }
    xSemaphoreGive(s_lock);
    return rc;
}

int nv_get(uint16_t tag, uint8_t *buf, uint16_t cap, uint16_t *n)
{
    int rc = -1, off;
    uint16_t rl = 0;

    if (!take()) return -1;
    if (s_present) {
        off = rec_find(s_v2, s_v2len, tag, &rl);
        if (off < 0) {
            rc = 1;
        } else {
            memcpy(buf, s_v2 + off, (rl < cap) ? rl : cap);
            if (n) *n = rl;
            rc = 0;
        }
    }
    xSemaphoreGive(s_lock);
    return rc;
}

int nv_set(uint16_t tag, const uint8_t *data, uint16_t len)
{
    int rc = -1;
    uint32_t n;

    if (tag == NV_TAG_LEGACY) return -1;      // by address only (nv_write), so the mirror follows
    if (!take()) return -1;
    if (s_present) {
        n = rec_put(s_v2tmp, s_v2, s_v2len, tag, data, len);
        if ((n != 0U || len == 0U) && write2(s_v2tmp, n, s_legacy_seq) == 0) {
            memcpy(s_v2, s_v2tmp, n);
            s_v2len = n;
            rc = 0;
        }
    }
    xSemaphoreGive(s_lock);
    return rc;
}

void nv_print(void)
{
    if (!take()) { printf("[NV] busy\r\n"); return; }
    if (s_present)
        printf("[NV] format 2 at 0x%06X/0x%06X copy %c seq %u (%u B, legacy = format-1 seq %u); "
               "format 1 at 0x%06X/0x%06X copy %c seq %u; writes %u, failed %u\r\n",
               (unsigned)NV2_FLASH_A, (unsigned)NV2_FLASH_B, s_slot2 >= 0 ? 'A' + s_slot2 : '-',
               (unsigned)s_seq2, (unsigned)s_v2len, (unsigned)s_legacy_seq,
               (unsigned)NV_FLASH_A, (unsigned)NV_FLASH_B, s_slot1 >= 0 ? 'A' + s_slot1 : '-',
               (unsigned)s_seq1, (unsigned)s_writes, (unsigned)s_errors);
    else
        printf("[NV] no settings store: the bitstream has no flash_ctl\r\n");
    xSemaphoreGive(s_lock);
}
