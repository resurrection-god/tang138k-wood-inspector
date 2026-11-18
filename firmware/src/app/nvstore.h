/*
 * nvstore.h - the board's settings store in SPI flash (22.10.2026, R-40; format 2 since
 * 23.10.2026, R-41).
 *
 * Format 2 (NV2_FLASH_A/B): a whole 4 KB sector per copy, A/B with seq and CRC-32, holding
 * tag-length-value records: a new field is a new tag, no further migrations. Every write
 * puts the whole image with seq + 1 into the sector not holding the newest copy, header
 * last, so a power cut mid-write leaves the previous image.
 *
 * The legacy 1 KB address space of settings.h (the settings and calibration partitions,
 * the IP and MAC records of net_config.h) is one record, NV_TAG_LEGACY, read and written
 * by address with nv_read / nv_write as before. It is also mirrored into the format-1
 * sectors (NV_FLASH_A/B, the 1 KB image of 22.10.2026), so firmware without format 2
 * (rollback to `main`) boots with the current settings, address and MAC; when such firmware
 * has written format 1 since (its seq differs from the one format 2 mirrored), the next boot
 * takes the legacy area from format 1 again. First boot with format 2: migrated from format 1.
 *
 * A bitstream without flash_ctl.v (spi_flash_present() false) has no store: reads and
 * writes fail, every owner falls back to its defaults.
 */

#ifndef APP_NVSTORE_H_
#define APP_NVSTORE_H_

#include <stdint.h>

#define NV_SIZE         1024U          /* the legacy address space of settings.h */
#define NV_FLASH_A      0x00F40000U    /* format 1 (legacy mirror), after the mm tables (0xF00000..0xF3FFFF) */
#define NV_FLASH_B      0x00F41000U
#define NV2_FLASH_A     0x00F42000U    /* format 2 */
#define NV2_FLASH_B     0x00F43000U

// Record tags of format 2 - one list, so two owners never share a number.
#define NV_TAG_LEGACY       0x0001U    /* the 1 KB legacy space (nv_read / nv_write) */
#define NV_TAG_GEV_IPCFG    0x0010U    /* net_config.c: GigE Vision IP configuration flags */
#define NV_TAG_USER_NAME    0x0011U    /* net_config.c: GigE Vision user-defined name */

void nv_init(void);                    // creates the lock; before the scheduler
// Legacy space by address. 0 OK, otherwise an error (no store, out of range, or the flash
// write failed: nothing changed).
int  nv_read(uint16_t addr, uint8_t *buf, uint16_t len);
int  nv_write(uint16_t addr, const uint8_t *buf, uint16_t len);
// Format 2 records. nv_get: 0 found (*n = its length, at most cap bytes copied), 1 no such
// record, -1 no store. nv_set: len 0 deletes the record; 0 OK, -1 no store, too big or the
// flash write failed (nothing changed).
int  nv_get(uint16_t tag, uint8_t *buf, uint16_t cap, uint16_t *n);
int  nv_set(uint16_t tag, const uint8_t *data, uint16_t len);
void nv_print(void);                   // console: active copies, seq

#endif /* APP_NVSTORE_H_ */
