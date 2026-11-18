/*
 * settings.h - settings storage (nvstore.h) and the mode settings sets
 *
 * Part of net_stack.h until 24.09.2026.
 */

#ifndef APP_SETTINGS_H_
#define APP_SETTINGS_H_

#include <stdint.h>
#include <stdbool.h>

// ---- Settings in the store (nvstore.h) ------------------------------------------
//
// docs/ARCHITECTURE.md § 7, docs/DECISIONS.md R-14. Two partitions, each with
// two copies (A/B): a write goes to the copy that does NOT hold the last whole
// image, so a power failure in the middle of a save leaves the previous one.
// On read the whole copy with the larger seq is taken.
//
//   0x000  16 B - do not touch: byte 0x89 of unknown origin (net_stack.c)
//   0x010  settings partition, copy A    (240 B)
//   0x100  settings partition, copy B    (240 B)
//   0x1F0  calibration partition, copy A (240 B)
//   0x2E0  calibration partition, copy B (240 B)
//   0x3D0  16 B - board IPv4 address, mask, gateway (net_config.h, since 15.10.2026)
//   0x3E0  16 B - free (was the field of the EEPROM write check 'W', removed 22.10.2026)
//
// This is the legacy 1 KB space: since 23.10.2026 one record of the store's format 2, mirrored
// into format 1 (nvstore.h, R-41). New fields go into format-2 records of their own, not here.
//   0x3F0  16 B - board MAC address (net_config.h, since 22.10.2026)
//
// Since 22.10.2026 this address space lives in SPI flash (nvstore.h, R-40); until then it was the
// EEPROM U24, which the project no longer uses.
//
// Partition image: a 16 B Big-Endian header (magic "SCFG"/"SCAL", u16 version,
// u16 payload length, u32 CRC-32, u32 seq) and the data. The CRC covers header
// bytes 0..7 and 12..15 and the data: a corrupted seq or length is visible too.
// The same image travels to the PC in 0x0168/0x016A.
//
// The settings partition data is "tag, length, data" sections, tag =
// (mode << 4) | item: mode 1 WORK / 2 SERVICE; item 1 camera 0, 2 camera 1,
// 3 synchroniser. Mode 0xF is outside the sets: item 4 is the line finder of
// camera 0, 5 of camera 1 (since version 2, 24.09.2026; in version 1 the finder
// was item 4 of each set). An unknown tag is skipped, a missing one gets the
// default. The calibration partition is the 72 B of 0x0130 as is.
//
// Never saved (docs/DECISIONS.md R-12): the operating mode, the laser enable,
// laser mode 2 ("always 1" is written as 1), the synchroniser enable bit.
#define SETTINGS_SEC_CFG        0
#define SETTINGS_SEC_CALIB      1
#define SETTINGS_SEC_COUNT      2
// The version this firmware writes. Which versions a partition accepts on read
// is the min_version of its owner (settings_section_t).
#define SETTINGS_VERSION        2U
#define SETTINGS_HDR_BYTES      16U
#define SETTINGS_IMAGE_MAX      256U
#define SETTINGS_SLOT_BYTES     240U   /* one partition copy in the store */
#define SETTINGS_MAGIC_CFG      0x53434647U   /* "SCFG" */
#define SETTINGS_MAGIC_CALIB    0x5343414CU   /* "SCAL" */
#define SETTINGS_RESET_GUARD    0x52534554U   /* "RSET": body of 0x0166 */

#define SETTINGS_F_PRESENT      0x01   /* a whole copy was read from the store    */
#define SETTINGS_F_DIRTY        0x02   /* RAM differs from what is in the store   */
#define SETTINGS_F_DAMAGED      0x04   /* there is a copy with a magic, but broken */
#define SETTINGS_F_READ_FAIL    0x08   /* the store did not answer over I2C       */
#define SETTINGS_F_DEFAULTS     0x10   /* running on compile-time defaults         */
#define SETTINGS_F_REJECTED     0x20   /* some values from the store rejected     */

typedef struct {
    uint8_t  flags;      // SETTINGS_F_*
    uint8_t  copy;       // whole copy: 0 = A, 1 = B, 0xFF = none
    uint16_t version;
    uint16_t length;     // payload bytes
    uint32_t seq;
} settings_info_t;

// Read the store at start-up and apply the SERVICE set (except the cameras:
// camera_boot_reset() sets them up). Idempotent; both camera tasks call it
// BEFORE the multiplexer scan; it takes video_ring_lock and i2c_bus_lock() itself.
void settings_boot_load(void);

// Everything below returns a VCP_ST_* code (0 = OK) and takes video_ring_lock
// itself. Save, load, reset and import in WORK answer DENIED
// (docs/ARCHITECTURE.md § 7).
int  settings_save(uint8_t section, settings_info_t *info);
int  settings_load(uint8_t section, settings_info_t *info);
int  settings_defaults(uint32_t guard);
int  settings_get_info(uint8_t section, settings_info_t *info);
// Partition image: the current one from RAM (from_store = false) or a whole
// copy from the chip.
int  settings_export(uint8_t section, bool from_store, uint8_t *out, uint32_t max,
                     uint32_t *len);
// Accept an image from the PC: check the magic, version, CRC and EVERY value;
// with save, also write it to the store. The current mode's set is applied at once.
int  settings_import(uint8_t section, const uint8_t *img, uint32_t len, bool save,
                     settings_info_t *info);
// Word 33 of the 0x0300 response: [7:0] settings partition flags, [15:8]
// calibration. Does not wait for the ring: if capture holds it, the mismatch
// flag is the previous one.
uint32_t settings_status_word(void);
// Partition state and both sets to the UART (command 'K').
void settings_print(void);




// Return codes (the same numbers as the protocol's VCP_ST_*).
#define SETTINGS_ST_OK            0x00
#define SETTINGS_ST_BUSY          0x02
#define SETTINGS_ST_BAD_LENGTH    0x06
#define SETTINGS_ST_BAD_CRC       0x07
#define SETTINGS_ST_DENIED        0x09
#define SETTINGS_ST_HW_ERROR      0x0A
#define SETTINGS_ST_UNSUPPORTED   0x0C

// Partition owner: the storage does not know what is in the data and asks it.
// The functions are called under video_ring_lock(), held by the storage.
typedef struct {
    // Partition data from RAM into out (at most SETTINGS_IMAGE_MAX). 0 = nothing to save.
    uint16_t (*payload)(uint8_t *out);
    // Accept data into RAM. strict: any rejected part cancels everything (import
    // from the PC); otherwise a rejected part stays at the default (store read).
    // *bad: how many parts were rejected. A SETTINGS_ST_* code.
    int      (*take)(const uint8_t *p, uint16_t len, bool strict, uint32_t *bad);
    // Apply what was accepted to the system (after load, import, reset). NULL = nothing.
    int      (*apply)(void);
    // Reset RAM to compile-time defaults. NULL = not supported.
    void     (*defaults)(void);
    // What to do with what was loaded at start-up (settings_boot_load). NULL = nothing.
    void     (*boot)(void);
    // Lines for the 'K' command. NULL = nothing to print.
    void     (*print)(void);
    // Data length if it is fixed (calibration is 72 B); 0 = any.
    uint16_t fixed_len;
    // The oldest image version the partition understands; 0 = any. An older image
    // is rejected whole on load (defaults and flag SETTINGS_F_REJECTED), on
    // import it gets DENIED.
    uint16_t min_version;
} settings_section_t;

// Hook up a partition owner and the rule "may the storage be changed now"
// (NULL = always). main() calls it before the tasks start.
void settings_register(uint8_t section, const settings_section_t *owner);
void settings_set_guard(bool (*writable)(void));

// Big-endian fields of partition data (the image travels to the PC and back).
static inline void settings_wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static inline void settings_wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
static inline uint16_t settings_rd16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline uint32_t settings_rd32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

#endif /* APP_SETTINGS_H_ */
