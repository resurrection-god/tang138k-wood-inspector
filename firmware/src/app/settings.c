/*
 * settings.c - settings storage (nvstore.h): partitions, image, copies A/B.
 * What is in a partition is known by its owner (settings_section_t): the mode
 * set by work_mode.c, the calibration by laser.c via main.c. Until 24.09.2026
 * the mode sets lived here.
 *
 * Part of net_stack.c until 24.09.2026.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "video.h"
#include "settings.h"
#include "i2c_bus.h"
#include "nvstore.h"
#include "vmsp_protocol.h"   // vmsp_crc32_update(): CRC of the settings image in the store
#include "cache.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "arch/sys_arch.h"
#include "arch/ethernetif.h"



// ===========================================================================
// Settings in the store (nvstore.h) and mode sets
// ===========================================================================
//
// docs/ARCHITECTURE.md § 7; the store layout and the image format are in
// settings.h. Two layers here:
//
//   image           header, CRC and partition data (the data is given and
//                   accepted by the partition owner, settings_section_t);
//   store          two copies of each partition, a write goes to the one not
//                   holding the last whole image, and a read-back is mandatory.
//
// Everything runs under video_ring_lock(): the lock order is the same as
// capture's, the ring first, then i2c_bus_lock(). That is why the work buffers
// are static.

#define SET_RING_WAIT_MS    2000U


// The first page 0x000..0x00F is not ours: before the first write on 23.09.2026
// 0x000 held byte 0x89 with the other 1023 at 0xFF; where it came from is
// unknown (a factory mark?), and there is only one copy of it. So the
// partitions start at 0x010.
static const uint16_t k_sec_base[SETTINGS_SEC_COUNT][2] = { { 0x010U, 0x100U },
                                                            { 0x1F0U, 0x2E0U } };
static const uint16_t k_sec_slot[SETTINGS_SEC_COUNT]  = { SETTINGS_SLOT_BYTES, SETTINGS_SLOT_BYTES };
static const uint32_t k_sec_magic[SETTINGS_SEC_COUNT] = { SETTINGS_MAGIC_CFG,
                                                          SETTINGS_MAGIC_CALIB };

// Compile-time layout: the copies do not overlap, do not touch page 0x000 and
// the 'W' command field (0x3E0..0x3FF). That the partition data fits in a copy
// is checked by the partition owners (work_mode.c, main.c).
typedef char settings_layout_check[
    (SETTINGS_SLOT_BYTES == 240U &&
     0x010U + 240U <= 0x100U && 0x100U + 240U <= 0x1F0U &&
     0x1F0U + 240U <= 0x2E0U && 0x2E0U + 240U <= 0x3E0U &&
     SETTINGS_IMAGE_MAX >= 240U) ? 1 : -1];

static const settings_section_t *s_sec[SETTINGS_SEC_COUNT];   /* partition owners */
static bool (*s_writable)(void);                              /* NULL = always allowed */
static bool       s_boot_done = false;

static settings_info_t s_info[SETTINGS_SEC_COUNT];
// Partition data as it lies in the store (last read or write): RAM is
// compared with it for the mismatch flag.
static uint8_t  s_saved[SETTINGS_SEC_COUNT][SETTINGS_IMAGE_MAX];
static uint16_t s_saved_len[SETTINGS_SEC_COUNT];
static bool     s_saved_valid[SETTINGS_SEC_COUNT];
static bool     s_dirty_last[SETTINGS_SEC_COUNT];

static uint8_t    s_payload[SETTINGS_IMAGE_MAX];   /* partition data from RAM */
static uint8_t    s_img[SETTINGS_IMAGE_MAX];       /* whole partition image   */
static uint8_t    s_scratch[SETTINGS_IMAGE_MAX];   /* reading copies from the chip */



// ---- image -------------------------------------------------------------------


// CRC over header bytes 0..7 (magic, version, length) and 12..15 (seq) and the
// data: everything except the CRC field itself.
static uint32_t set_image_crc(const uint8_t *img, uint16_t len)
{
    uint32_t c = VMSP_CRC32_INIT;

    c = vmsp_crc32_update(c, img, 8);
    c = vmsp_crc32_update(c, img + 12, 4);
    c = vmsp_crc32_update(c, img + SETTINGS_HDR_BYTES, len);
    return vmsp_crc32_final(c);
}

static uint32_t set_image_build(uint8_t sec, const uint8_t *payload, uint16_t len,
                                uint32_t seq, uint8_t *out)
{
    memmove(out + SETTINGS_HDR_BYTES, payload, len);
    settings_wr32(out + 0, k_sec_magic[sec]);
    settings_wr16(out + 4, SETTINGS_VERSION);
    settings_wr16(out + 6, len);
    settings_wr32(out + 12, seq);
    settings_wr32(out + 8, set_image_crc(out, len));
    return SETTINGS_HDR_BYTES + len;
}

// Partition image check. A VCP_ST_* code: DENIED = foreign magic (on an erased
// store it is 0xFFFFFFFF, i.e. "empty"), UNSUPPORTED = a version from the
// future, BAD_LENGTH, BAD_CRC.
static int set_image_check(uint8_t sec, const uint8_t *img, uint32_t avail,
                           uint16_t *len, uint32_t *seq, uint16_t *ver)
{
    uint16_t v, l;

    if (avail < SETTINGS_HDR_BYTES) return SETTINGS_ST_BAD_LENGTH;
    if (settings_rd32(img) != k_sec_magic[sec]) return SETTINGS_ST_DENIED;
    v = settings_rd16(img + 4);
    l = settings_rd16(img + 6);
    if (v == 0U || v > SETTINGS_VERSION) return SETTINGS_ST_UNSUPPORTED;
    if ((uint32_t)SETTINGS_HDR_BYTES + l > k_sec_slot[sec] ||
        (uint32_t)SETTINGS_HDR_BYTES + l > avail) return SETTINGS_ST_BAD_LENGTH;
    if (s_sec[sec] != NULL && s_sec[sec]->fixed_len != 0U && l != s_sec[sec]->fixed_len)
        return SETTINGS_ST_BAD_LENGTH;
    if (settings_rd32(img + 8) != set_image_crc(img, l)) return SETTINGS_ST_BAD_CRC;
    if (len) *len = l;
    if (seq) *seq = settings_rd32(img + 12);
    if (ver) *ver = v;
    return SETTINGS_ST_OK;
}

// Partition data from RAM, from the owner. 0 = nothing to save (no calibration).
static uint16_t set_current_payload(uint8_t sec, uint8_t *out)
{
    return (s_sec[sec] != NULL) ? s_sec[sec]->payload(out) : 0U;
}

// Accept partition data into RAM. strict: any rejected section cancels
// everything (import from the PC: let the person know at once); otherwise a
// rejected part stays at the default (store read: the board must come up).
// *bad: how many were rejected.
static int set_take_payload(uint8_t sec, const uint8_t *p, uint16_t len, bool strict,
                            uint32_t *bad)
{
    if (bad) *bad = 0;
    if (s_sec[sec] == NULL) return SETTINGS_ST_UNSUPPORTED;
    return s_sec[sec]->take(p, len, strict, bad);
}

// The image is older than the partition owner understands (min_version, 0 = any).
static bool set_version_too_old(uint8_t sec, uint16_t ver)
{
    return s_sec[sec] != NULL && s_sec[sec]->min_version != 0U &&
           ver < s_sec[sec]->min_version;
}

// Apply what was accepted (load, reset, import); a partition without apply gets OK.
static int set_apply(uint8_t sec)
{
    return (s_sec[sec] != NULL && s_sec[sec]->apply != NULL) ? s_sec[sec]->apply()
                                                            : SETTINGS_ST_OK;
}

// In WORK the storage is read-only (docs/ARCHITECTURE.md § 7): work_mode gives
// the rule.
static bool set_writable(void)
{
    return (s_writable == NULL) || s_writable();
}

static bool set_dirty(uint8_t sec)
{
    uint16_t n = set_current_payload(sec, s_payload);
    bool d;

    if (!s_saved_valid[sec])
        d = (n != 0U);          /* not in the store, present in RAM (the mode set always is) */
    else
        d = (n != s_saved_len[sec]) || (memcmp(s_payload, s_saved[sec], n) != 0);
    s_dirty_last[sec] = d;
    return d;
}

static void set_fill_info(uint8_t sec, settings_info_t *info, bool dirty)
{
    if (info == NULL) return;
    *info = s_info[sec];
    if (dirty) info->flags |= SETTINGS_F_DIRTY;
}

// ---- store ------------------------------------------------------------------

// Read both copies of the partition and pick the whole one with the larger seq;
// its image goes to out. Returns the flags SETTINGS_F_PRESENT / _DAMAGED /
// _READ_FAIL.
static uint8_t set_store_read(uint8_t sec, uint8_t *out, uint8_t *copy,
                               uint16_t *len, uint32_t *seq, uint16_t *ver)
{
    uint8_t flags = 0;
    bool found = false;
    uint8_t k;

    *copy = 0xFFU;
    if (!i2c_bus_lock(2000))
        return SETTINGS_F_READ_FAIL;
    for (k = 0; k < 2U; ++k)
    {
        uint16_t l = 0, v = 0;
        uint32_t sq = 0;
        int st;

        if (nv_read(k_sec_base[sec][k], s_scratch, k_sec_slot[sec]) != 0)
        {
            flags |= SETTINGS_F_READ_FAIL;
            continue;
        }
        st = set_image_check(sec, s_scratch, k_sec_slot[sec], &l, &sq, &v);
        if (st == SETTINGS_ST_DENIED) continue;                 /* empty */
        if (st != SETTINGS_ST_OK)
        {
            printf("[SETTINGS] section %u copy %c at 0x%03X damaged (status 0x%02X)\r\n",
                   (unsigned int)sec, 'A' + k, (unsigned int)k_sec_base[sec][k],
                   (unsigned int)st);
            flags |= SETTINGS_F_DAMAGED;
            continue;
        }
        if (!found || (int32_t)(sq - *seq) > 0)
        {
            found = true;
            *copy = k;
            *len  = l;
            *seq  = sq;
            *ver  = v;
            memcpy(out, s_scratch, SETTINGS_HDR_BYTES + l);
        }
    }
    i2c_bus_unlock();
    if (found) flags |= SETTINGS_F_PRESENT;
    return flags;
}

// Write the partition data as a new image into the copy that does NOT hold the
// last whole image, and read it back: this chip acknowledges a forbidden write
// with ACK (ERRORS former 31.50), only a read-back is proof.
static int set_store_write(uint8_t sec, const uint8_t *payload, uint16_t len)
{
    uint8_t copy, target, fl;
    uint16_t l = 0, v = 0;
    uint32_t seq = 0, n;
    int rc = -1;

    fl = set_store_read(sec, s_img, &copy, &l, &seq, &v);
    if ((fl & SETTINGS_F_READ_FAIL) && !(fl & SETTINGS_F_PRESENT))
    {
        printf("[SETTINGS] store does not answer, nothing written\r\n");
        return SETTINGS_ST_HW_ERROR;
    }
    target = (copy == 0U) ? 1U : 0U;
    seq = (fl & SETTINGS_F_PRESENT) ? seq + 1U : 1U;
    n = set_image_build(sec, payload, len, seq, s_img);

    if (!i2c_bus_lock(2000))
        return SETTINGS_ST_BUSY;
    if (nv_write(k_sec_base[sec][target], s_img, (uint16_t)n) == 0 &&
        nv_read(k_sec_base[sec][target], s_scratch, (uint16_t)n) == 0)
        rc = (memcmp(s_img, s_scratch, n) == 0) ? 0 : -2;
    i2c_bus_unlock();

    if (rc != 0)
    {
        printf("[SETTINGS] section %u copy %c: %s\r\n", (unsigned int)sec, 'A' + target,
               (rc == -2) ? "read-back differs - WRITE PROTECTED? (ch. 31.50)"
                          : "transfer failed (NACK/timeout)");
        return SETTINGS_ST_HW_ERROR;
    }
    memcpy(s_saved[sec], payload, len);
    s_saved_len[sec]   = len;
    s_saved_valid[sec] = true;
    s_info[sec].flags   = SETTINGS_F_PRESENT;
    s_info[sec].copy    = target;
    s_info[sec].version = SETTINGS_VERSION;
    s_info[sec].length  = len;
    s_info[sec].seq     = seq;
    printf("[SETTINGS] section %u saved: copy %c at 0x%03X, %u bytes, seq %u\r\n",
           (unsigned int)sec, 'A' + target, (unsigned int)k_sec_base[sec][target],
           (unsigned int)n, (unsigned int)seq);
    return SETTINGS_ST_OK;
}

// Partition from the chip -> RAM. Without a whole copy RAM is not touched: at
// start-up the defaults stay there, and NOTHING is written back; a damaged image
// may turn out to be the only evidence (docs/DECISIONS.md R-14).
static int set_load_section(uint8_t sec, bool boot)
{
    settings_info_t *in = &s_info[sec];
    uint8_t copy, fl;
    uint16_t len = 0, ver = 0;
    uint32_t seq = 0, bad = 0;
    int st;

    fl = set_store_read(sec, s_img, &copy, &len, &seq, &ver);
    in->flags = fl;
    in->copy  = copy;
    if (!(fl & SETTINGS_F_PRESENT))
    {
        in->version = 0;
        in->length  = 0;
        in->seq     = 0;
        if (boot && sec == SETTINGS_SEC_CFG) in->flags |= SETTINGS_F_DEFAULTS;
        printf("[SETTINGS] section %u: %s%s\r\n", (unsigned int)sec,
               (fl & SETTINGS_F_READ_FAIL) ? "store read failed" :
               (fl & SETTINGS_F_DAMAGED)   ? "!!! no intact copy, image left untouched" :
                                             "empty",
               (boot && sec == SETTINGS_SEC_CFG) ? " - compile-time defaults" : "");
        return (fl & SETTINGS_F_READ_FAIL) ? SETTINGS_ST_HW_ERROR : SETTINGS_ST_DENIED;
    }
    in->version = ver;
    in->length  = len;
    in->seq     = seq;
    // An image of the old layout is not parsed at all: RAM keeps the defaults, the
    // "rejected" flag is set, and nothing is written to the store; the old image
    // stays until an explicit save (0x0160) overwrites it.
    if (set_version_too_old(sec, ver))
    {
        // At start-up RAM already holds the defaults; on an explicit load (0x0162),
        // reset to them, the caller applies them.
        if (!boot && s_sec[sec]->defaults != NULL) s_sec[sec]->defaults();
        in->flags |= SETTINGS_F_REJECTED;
        memcpy(s_saved[sec], s_img + SETTINGS_HDR_BYTES, len);
        s_saved_len[sec]   = len;
        s_saved_valid[sec] = true;
        printf("[SETTINGS] section %u: image version %u is older than %u - rejected, defaults used\r\n",
               (unsigned int)sec, (unsigned int)ver, (unsigned int)s_sec[sec]->min_version);
        return SETTINGS_ST_OK;
    }
    st = set_take_payload(sec, s_img + SETTINGS_HDR_BYTES, len, false, &bad);
    if (st != SETTINGS_ST_OK) return st;
    if (bad != 0U)
    {
        in->flags |= SETTINGS_F_REJECTED;
        printf("[SETTINGS] section %u: %u item(s) out of range, defaults used for them\r\n",
               (unsigned int)sec, (unsigned int)bad);
    }
    memcpy(s_saved[sec], s_img + SETTINGS_HDR_BYTES, len);
    s_saved_len[sec]   = len;
    s_saved_valid[sec] = true;
    printf("[SETTINGS] section %u loaded: copy %c, %u bytes, seq %u%s\r\n",
           (unsigned int)sec, 'A' + copy, (unsigned int)len, (unsigned int)seq,
           (fl & SETTINGS_F_DAMAGED) ? " (other copy damaged)" : "");
    return SETTINGS_ST_OK;
}

// ---- public ------------------------------------------------------------------

void settings_boot_load(void)
{
    uint8_t sec;

    if (s_boot_done) return;
    if (!video_ring_lock(5000U))
    {
        printf("[SETTINGS] boot: ring busy, settings not loaded yet\r\n");
        return;                                   /* the next camera task will retry */
    }
    if (!s_boot_done)
    {
        for (sec = 0; sec < SETTINGS_SEC_COUNT; ++sec) (void)set_load_section(sec, true);
        // What to do with what was loaded at start-up is decided by the partition owner
        // (mode set: SERVICE into the synchroniser and the finder, work_mode.c).
        for (sec = 0; sec < SETTINGS_SEC_COUNT; ++sec)
            if (s_sec[sec] != NULL && s_sec[sec]->boot != NULL) s_sec[sec]->boot();
        s_boot_done = true;
    }
    video_ring_unlock();
}

int settings_save(uint8_t section, settings_info_t *info)
{
    uint16_t n;
    int st;

    if (section >= SETTINGS_SEC_COUNT) return SETTINGS_ST_UNSUPPORTED;
    if (!video_ring_lock(SET_RING_WAIT_MS)) return SETTINGS_ST_BUSY;
    // In WORK: refusal; ~0.1 s of I2C writes in the middle of the pipeline
    // (docs/ARCHITECTURE.md § 7).
    if (!set_writable())
        st = SETTINGS_ST_DENIED;
    else if ((n = set_current_payload(section, s_payload)) == 0U)
        st = SETTINGS_ST_DENIED;                       /* no calibration, nothing to save */
    else
        st = set_store_write(section, s_payload, n);
    set_fill_info(section, info, set_dirty(section));
    video_ring_unlock();
    return st;
}

int settings_load(uint8_t section, settings_info_t *info)
{
    int st;

    if (section >= SETTINGS_SEC_COUNT) return SETTINGS_ST_UNSUPPORTED;
    if (!video_ring_lock(SET_RING_WAIT_MS)) return SETTINGS_ST_BUSY;
    if (!set_writable())
        st = SETTINGS_ST_DENIED;
    else
    {
        st = set_load_section(section, false);
        if (st == SETTINGS_ST_OK) st = set_apply(section);
    }
    set_fill_info(section, info, set_dirty(section));
    video_ring_unlock();
    return st;
}

int settings_defaults(uint32_t guard)
{
    int st;

    // Guard word: the command erases a setting, and a random packet must not.
    if (guard != SETTINGS_RESET_GUARD) return SETTINGS_ST_DENIED;
    if (!video_ring_lock(SET_RING_WAIT_MS)) return SETTINGS_ST_BUSY;
    if (!set_writable())
        st = SETTINGS_ST_DENIED;
    else
    {
        // RAM only: the defaults reach the store by an explicit save.
        if (s_sec[SETTINGS_SEC_CFG] != NULL && s_sec[SETTINGS_SEC_CFG]->defaults != NULL)
            s_sec[SETTINGS_SEC_CFG]->defaults();
        s_info[SETTINGS_SEC_CFG].flags |= SETTINGS_F_DEFAULTS;
        printf("[SETTINGS] RAM reset to compile-time defaults (store untouched)\r\n");
        st = set_apply(SETTINGS_SEC_CFG);
    }
    video_ring_unlock();
    return st;
}

int settings_get_info(uint8_t section, settings_info_t *info)
{
    if (section >= SETTINGS_SEC_COUNT) return SETTINGS_ST_UNSUPPORTED;
    if (video_ring_lock(500U))
    {
        set_fill_info(section, info, set_dirty(section));
        video_ring_unlock();
    }
    else
    {
        set_fill_info(section, info, s_dirty_last[section]);   /* capture in progress */
    }
    return SETTINGS_ST_OK;
}

uint32_t settings_status_word(void)
{
    uint32_t v = 0;
    uint8_t sec;

    // Do not wait: 0x0300 is called during capture too, the flag is then the previous one.
    if (video_ring_lock(0U))
    {
        for (sec = 0; sec < SETTINGS_SEC_COUNT; ++sec) (void)set_dirty(sec);
        video_ring_unlock();
    }
    for (sec = 0; sec < SETTINGS_SEC_COUNT; ++sec)
    {
        uint32_t f = s_info[sec].flags;
        if (s_dirty_last[sec]) f |= SETTINGS_F_DIRTY;
        v |= (f & 0xFFU) << (8U * sec);
    }
    return v;
}

int settings_export(uint8_t section, bool from_store, uint8_t *out, uint32_t max,
                    uint32_t *len)
{
    uint32_t n = 0;
    int st = SETTINGS_ST_OK;

    if (len) *len = 0;
    if (section >= SETTINGS_SEC_COUNT) return SETTINGS_ST_UNSUPPORTED;
    if (out == NULL) return SETTINGS_ST_BAD_LENGTH;
    if (!video_ring_lock(SET_RING_WAIT_MS)) return SETTINGS_ST_BUSY;
    if (from_store)
    {
        uint8_t copy;
        uint16_t l = 0, v = 0;
        uint32_t seq = 0;
        uint8_t fl = set_store_read(section, s_img, &copy, &l, &seq, &v);

        if (fl & SETTINGS_F_PRESENT)
            n = SETTINGS_HDR_BYTES + l;
        else
            st = (fl & SETTINGS_F_READ_FAIL) ? SETTINGS_ST_HW_ERROR : SETTINGS_ST_DENIED;
    }
    else
    {
        uint16_t pl = set_current_payload(section, s_payload);

        if (pl == 0U)
            st = SETTINGS_ST_DENIED;
        else
            n = set_image_build(section, s_payload, pl, s_info[section].seq, s_img);
    }
    if (st == SETTINGS_ST_OK)
    {
        if (n > max)
            st = SETTINGS_ST_BAD_LENGTH;
        else
        {
            memcpy(out, s_img, n);
            if (len) *len = n;
        }
    }
    video_ring_unlock();
    return st;
}

int settings_import(uint8_t section, const uint8_t *img, uint32_t len, bool save,
                    settings_info_t *info)
{
    uint16_t l = 0, v = 0;
    uint32_t seq = 0, bad = 0;
    int st;

    if (section >= SETTINGS_SEC_COUNT) return SETTINGS_ST_UNSUPPORTED;
    if (img == NULL || len > SETTINGS_IMAGE_MAX) return SETTINGS_ST_BAD_LENGTH;
    st = set_image_check(section, img, len, &l, &seq, &v);
    if (st != SETTINGS_ST_OK) return st;
    if ((uint32_t)SETTINGS_HDR_BYTES + l != len) return SETTINGS_ST_BAD_LENGTH;
    if (set_version_too_old(section, v)) return SETTINGS_ST_DENIED;
    if (!video_ring_lock(SET_RING_WAIT_MS)) return SETTINGS_ST_BUSY;
    if (!set_writable())
        st = SETTINGS_ST_DENIED;
    else
    {
        // The data is copied: img is the network task's buffer, and set_store_write()
        // below builds the image in s_img.
        memcpy(s_payload, img + SETTINGS_HDR_BYTES, l);
        st = set_take_payload(section, s_payload, l, true, &bad);
        if (st != SETTINGS_ST_OK)
            printf("[SETTINGS] import of section %u refused: %u item(s) out of range\r\n",
                   (unsigned int)section, (unsigned int)bad);
        if (st == SETTINGS_ST_OK)
        {
            printf("[SETTINGS] section %u imported from host (%u bytes, host seq %u)\r\n",
                   (unsigned int)section, (unsigned int)l, (unsigned int)seq);
            s_info[section].flags &= (uint8_t)~SETTINGS_F_DEFAULTS;
            st = set_apply(section);
            if (save)
            {
                int st_save = set_store_write(section, s_payload, l);
                if (st == SETTINGS_ST_OK) st = st_save;
            }
        }
    }
    set_fill_info(section, info, set_dirty(section));
    video_ring_unlock();
    return st;
}

void settings_print(void)
{
    static const char *names[SETTINGS_SEC_COUNT] = { "settings", "calibration" };
    settings_info_t in;
    uint8_t sec;

    printf("\r\n[SETTINGS] settings store\r\n");
    for (sec = 0; sec < SETTINGS_SEC_COUNT; ++sec)
    {
        (void)settings_get_info(sec, &in);
        printf("  %-11s flags 0x%02X [%s%s%s%s%s%s] copy %c, v%u, %u bytes, seq %u\r\n",
               names[sec], (unsigned int)in.flags,
               (in.flags & SETTINGS_F_PRESENT)   ? " present"   : "",
               (in.flags & SETTINGS_F_DIRTY)     ? " UNSAVED"   : "",
               (in.flags & SETTINGS_F_DAMAGED)   ? " damaged"   : "",
               (in.flags & SETTINGS_F_READ_FAIL) ? " read-fail" : "",
               (in.flags & SETTINGS_F_DEFAULTS)  ? " defaults"  : "",
               (in.flags & SETTINGS_F_REJECTED)  ? " rejected"  : "",
               (in.copy == 0xFFU) ? '-' : (char)('A' + in.copy),
               (unsigned int)in.version, (unsigned int)in.length, (unsigned int)in.seq);
    }
    for (sec = 0; sec < SETTINGS_SEC_COUNT; ++sec)
        if (s_sec[sec] != NULL && s_sec[sec]->print != NULL) s_sec[sec]->print();
}

void settings_register(uint8_t section, const settings_section_t *owner)
{
    if (section < SETTINGS_SEC_COUNT) s_sec[section] = owner;
}

void settings_set_guard(bool (*writable)(void))
{
    s_writable = writable;
}
