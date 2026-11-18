/*
 * spi_flash.h - the board's SPI flash (16.10.2026, docs/DECISIONS.md R-39)
 *
 * The flash holds the bitstream (0x000000), the firmware (0x600000) and the data
 * area (0xF00000..). It is served by our own controller in the FPGA,
 * fpga/src/flash_ctl.v: the boot window 0x80000000 (the CPU's instruction memory)
 * and a command port at 0xF8000000 (ERRORS 5.12). Without the command port a CPU
 * access there hangs the core, so spi_flash_init() first reads the controller's
 * identity word through the boot window (SPI_FLASH_ID_ADDR); without it reads go
 * through the boot window and every other call answers SPI_FLASH_ERR_ABSENT.
 *
 * Erase and program are refused outside the data area [SPI_FLASH_DATA_BASE,
 * SPI_FLASH_DATA_END) here and, independently, by flash_ctl.v itself: a wrong
 * address can never reach the bitstream or the firmware. Reads are allowed
 * anywhere.
 */

#ifndef APP_SPI_FLASH_H_
#define APP_SPI_FLASH_H_

#include <stdint.h>
#include <stdbool.h>

#define SPI_FLASH_SECTOR        4096U
#define SPI_FLASH_PAGE          256U
// Data area: 1 MB at the top of the 16 MB chip, far from the bitstream (to
// 0x509E00 as the programmer writes it) and the firmware (0x600000, ~160 KB);
// both programmer writes leave it intact - docs/WORKFLOWS.md § 5.
#define SPI_FLASH_DATA_BASE     0x00F00000U
#define SPI_FLASH_DATA_END      0x01000000U
// The boot window: the CPU sees flash 0x600000 at 0x80000000, and the 16 MB wrap
// around (flash 0x000000 at 0x80A00000 - the bitstream preamble, checked over JTAG
// 16.10.2026). Read-only; reads work on every bitstream.
#define SPI_FLASH_WINDOW(a)     (0x80000000U + (((uint32_t)(a) - 0x00600000U) & 0x00FFFFFFU))
// flash_ctl.v's identity word "FCT1" in the ROM window (no flash access behind it).
#define SPI_FLASH_ID_ADDR       0x8F000000U

#define SPI_FLASH_OK            0
#define SPI_FLASH_ERR_RANGE     (-1)   /* outside the data area or the chip        */
#define SPI_FLASH_ERR_TIMEOUT   (-2)   /* the controller or the chip did not finish */
#define SPI_FLASH_ERR_VERIFY    (-3)   /* read back differs                        */
#define SPI_FLASH_ERR_BUSY      (-4)   /* the flash lock was not free              */
#define SPI_FLASH_ERR_ABSENT    (-5)   /* the bitstream has no flash command port  */
#define SPI_FLASH_ERR_REFUSED   (-6)   /* flash_ctl.v refused the command          */

// Probes for flash_ctl.v. Before the scheduler.
void spi_flash_init(void);
bool spi_flash_present(void);
// The bitstream's build id (fpga/src/build_id.v at flash_ctl offset 0x014): CRC-32 of its RTL
// sources. 0 = unknown (no flash_ctl, or one from before 23.10.2026).
uint32_t spi_flash_build_id(void);
// Features of the bitstream the firmware must not assume (flash_ctl offset 0x018; 0 on older
// bitstreams): SPI_FLASH_FEAT_* bits.
#define SPI_FLASH_FEAT_FRAME_NOCHUNK   0x1U   /* udp_rd: frames as 0x0001 without chunks (ERRORS 3.23) */
#define SPI_FLASH_FEAT_CTL_APB         0x2U   /* control blocks on the APB (apb_ctl.v, 0xF8001000); no UDPD jobs */
uint32_t spi_flash_fpga_features(void);

// Stored data the board fell back from (28.10.2026, ERRORS 5.15): a record's newest copy did not
// read back whole twice and an older copy was used, or a copy was found damaged. Not normal - a
// misreading bitstream or a dying flash; reported on VMSP 0x0300 word 33 [18:16] and GEV
// DeviceStoredDataAlarm. Cleared only by a reboot.
#define SPI_FLASH_ALARM_SETTINGS    0x1U    /* settings store (nvstore.c) */
#define SPI_FLASH_ALARM_MM(cam)     (0x2U << (cam))   /* pixel -> mm table of camera cam */
void     spi_flash_alarm_set(uint32_t bits);
uint32_t spi_flash_alarm_get(void);
// JEDEC ID {manufacturer, type, capacity} as 0x00MMTTCC; 0 or 0xFFFFFF - no answer.
uint32_t spi_flash_id(void);
int spi_flash_read(uint32_t addr, uint8_t *dst, uint32_t len);
// Erase the 4 KB sectors covering [addr, addr + len) - data area only.
int spi_flash_erase(uint32_t addr, uint32_t len);
// Program and verify (data area only, the range erased before).
int spi_flash_write(uint32_t addr, const uint8_t *src, uint32_t len);

#endif /* APP_SPI_FLASH_H_ */
