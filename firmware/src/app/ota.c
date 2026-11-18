/*
 * ota.c - Over-The-Air safe firmware and bitstream updates (Dual-Slot A/B, Zero-Brick).
 */

#include "ota.h"
#include "spi_flash.h"
#include "vmsp_protocol.h"
#include "cache.h"
#include "platform.h"
#include "ethernet.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#define SMU_CMD_SW_RESET    0x3CU
#define READ_CHUNK          256U

static bool         s_session_active = false;
static uint8_t      s_target = 0;
static uint32_t     s_total_size = 0;
static uint32_t     s_expected_crc = 0;
static uint32_t     s_received_bytes = 0;
static uint32_t     s_running_crc = VMSP_CRC32_INIT;

static ota_meta_t   s_meta;
static bool         s_meta_loaded = false;

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

static int write_metadata(const ota_meta_t *m)
{
    int rc = spi_flash_erase(OTA_META_FLASH_ADDR, SPI_FLASH_SECTOR);
    if (rc == SPI_FLASH_OK) {
        rc = spi_flash_write(OTA_META_FLASH_ADDR, (const uint8_t *)m, sizeof(ota_meta_t));
    }
    return rc;
}

void ota_init(void)
{
    int rc = spi_flash_read(OTA_META_FLASH_ADDR, (uint8_t *)&s_meta, sizeof(ota_meta_t));
    if (rc == SPI_FLASH_OK && s_meta.magic == OTA_MAGIC_VAL) {
        uint32_t crc = vmsp_crc32((const uint8_t *)&s_meta, sizeof(ota_meta_t) - 4U);
        if (crc == s_meta.header_crc) {
            s_meta_loaded = true;
            printf("[OTA] Metadata valid: active slot %u, status %u, attempts %u, B size %u B\r\n",
                   (unsigned int)s_meta.active_slot, (unsigned int)s_meta.boot_status,
                   (unsigned int)s_meta.boot_attempts, (unsigned int)s_meta.slot_b_size);
            return;
        }
    }

    s_meta_loaded = false;
    memset(&s_meta, 0, sizeof(s_meta));
    printf("[OTA] No valid metadata found. Running default Slot A.\r\n");
}

void ota_confirm_boot(void)
{
    volatile uint32_t *trk = (volatile uint32_t *)OTA_DDR3_TRACKER_ADDR;
    trk[1] = 0U;  /* reset boot attempt counter */

    if (s_meta_loaded && s_meta.active_slot == 1U && s_meta.boot_status == OTA_BOOT_STATUS_TENTATIVE) {
        s_meta.boot_status = OTA_BOOT_STATUS_CONFIRMED;
        s_meta.boot_attempts = 0U;
        s_meta.header_crc = vmsp_crc32((const uint8_t *)&s_meta, sizeof(ota_meta_t) - 4U);
        if (write_metadata(&s_meta) == SPI_FLASH_OK) {
            printf("[OTA] Firmware Slot B confirmed healthy and marked permanent!\r\n");
        } else {
            printf("[OTA] Failed to write confirmation metadata!\r\n");
        }
    }
}

uint8_t ota_current_slot(void)
{
    return (s_meta_loaded && s_meta.active_slot == 1U) ? 1U : 0U;
}

const char *ota_get_version_string(void)
{
#ifdef OTA_LITE_PROFILE
    return (ota_current_slot() == 1U) ? FW_VERSION_STR_LITE_B : FW_VERSION_STR_LITE_A;
#else
    return (ota_current_slot() == 1U) ? FW_VERSION_STR_SLOT_B : FW_VERSION_STR_SLOT_A;
#endif
}

uint8_t ota_get_led_pattern(void)
{
    return (ota_current_slot() == 1U) ? OTA_LED_PATTERN_DOUBLE : OTA_LED_PATTERN_CALM;
}

int ota_get_status(uint8_t *out_payload, uint16_t *out_len)
{
    volatile uint32_t *trk = (volatile uint32_t *)OTA_DDR3_TRACKER_ADDR;
    uint8_t attempts = (trk[0] == OTA_TRACKER_MAGIC) ? (uint8_t)trk[1] : (s_meta_loaded ? (uint8_t)s_meta.boot_attempts : 0U);
    uint8_t fallback = (trk[0] == OTA_TRACKER_MAGIC) ? (uint8_t)trk[3] : 0U;
    uint8_t slot = ota_current_slot();
    const char *ver = ota_get_version_string();

    out_payload[0] = 0U;  /* status OK */
    out_payload[1] = slot;
    out_payload[2] = s_meta_loaded ? (uint8_t)s_meta.boot_status : 0U;
    out_payload[3] = attempts;
    wr_be32(out_payload + 4, OTA_SLOT_A_FLASH_ADDR);
    wr_be32(out_payload + 8, OTA_SLOT_B_FLASH_ADDR);
    wr_be32(out_payload + 12, s_meta.slot_b_crc);
    wr_be32(out_payload + 16, s_meta.slot_b_size);
    wr_be32(out_payload + 20, spi_flash_build_id());
    out_payload[24] = fallback;
    out_payload[25] = ota_get_led_pattern();
    out_payload[26] = (slot == 1U) ? 2U : 1U;  /* Version Major */
    out_payload[27] = (slot == 1U) ? 1U : 0U;  /* Version Minor */

    /* 36-byte null-padded firmware version banner */
    memset(out_payload + 28, 0, 36);
    strncpy((char *)(out_payload + 28), ver, 35);
    *out_len = 64U;
    return 0;
}

int ota_set_active_slot(uint8_t slot, uint8_t *out_payload, uint16_t *out_len)
{
    if (slot > 1U) {
        out_payload[0] = 1U;  /* Invalid slot */
        *out_len = 1U;
        return 0;
    }

    s_meta.magic = OTA_MAGIC_VAL;
    s_meta.active_slot = (uint32_t)slot;
    s_meta.boot_status = OTA_BOOT_STATUS_CONFIRMED;
    s_meta.boot_attempts = 0U;
    s_meta.header_crc = vmsp_crc32((const uint8_t *)&s_meta, sizeof(ota_meta_t) - 4U);

    if (write_metadata(&s_meta) != SPI_FLASH_OK) {
        out_payload[0] = 2U;  /* Flash write failed */
        *out_len = 1U;
        return 0;
    }

    s_meta_loaded = true;
    volatile uint32_t *trk = (volatile uint32_t *)OTA_DDR3_TRACKER_ADDR;
    trk[0] = OTA_TRACKER_MAGIC;
    trk[1] = 0U;
    trk[2] = (uint32_t)slot;
    trk[3] = 0U;

    printf("[OTA] Active slot manually changed to Slot %c (%s)\r\n",
           slot ? 'B' : 'A', slot ? "Candidate 0xF60000" : "Golden 0x600000");

    out_payload[0] = 0U;  /* OK */
    out_payload[1] = slot;
    *out_len = 2U;
    return 0;
}

int ota_start(uint8_t target, uint32_t total_size, uint32_t expected_crc, uint8_t *out_payload, uint16_t *out_len)
{
    if (target != OTA_TARGET_FIRMWARE) {
        out_payload[0] = 1U;  /* Unsupported target */
        *out_len = 1U;
        return 0;
    }

    if (total_size < 1024U || total_size > OTA_MAX_FW_BYTES) {
        out_payload[0] = 2U;  /* Bad size */
        *out_len = 1U;
        return 0;
    }

    s_session_active = true;
    s_target = target;
    s_total_size = total_size;
    s_expected_crc = expected_crc;
    s_received_bytes = 0U;
    s_running_crc = VMSP_CRC32_INIT;

    printf("[OTA] Session started: target %u, size %u B, CRC %08X\r\n",
           (unsigned int)target, (unsigned int)total_size, (unsigned int)expected_crc);

    out_payload[0] = 0U;  /* OK */
    out_payload[1] = 1U;  /* Target slot = Slot B */
    wr_be16(out_payload + 2, 1024U);
    wr_be32(out_payload + 4, OTA_DDR3_STAGING_ADDR);
    *out_len = 8U;
    return 0;
}

int ota_data(uint32_t offset, const uint8_t *data, uint16_t len, uint8_t *out_payload, uint16_t *out_len)
{
    if (!s_session_active) {
        out_payload[0] = 1U;  /* Not in session */
        *out_len = 1U;
        return 0;
    }

    if (offset != s_received_bytes || offset + len > s_total_size) {
        out_payload[0] = 2U;  /* Bad offset or overflow */
        *out_len = 1U;
        return 0;
    }

    memcpy((uint8_t *)(OTA_DDR3_STAGING_ADDR + offset), data, len);
    s_running_crc = vmsp_crc32_update(s_running_crc, data, len);
    s_received_bytes += len;

    out_payload[0] = 0U;  /* OK */
    out_payload[1] = 0U;
    wr_be16(out_payload + 2, len);
    wr_be32(out_payload + 4, s_received_bytes);
    wr_be32(out_payload + 8, vmsp_crc32_final(s_running_crc));
    *out_len = 12U;
    return 0;
}

int ota_commit(uint32_t expected_crc, uint8_t *out_payload, uint16_t *out_len)
{
    uint8_t read_chk[READ_CHUNK];
    uint32_t rb_crc = VMSP_CRC32_INIT;
    uint32_t off;
    uint32_t num_sectors;
    uint32_t erase_len;
    uint32_t final_crc;
    int rc;

    if (!s_session_active || s_received_bytes != s_total_size) {
        out_payload[0] = 1U;  /* Incomplete */
        *out_len = 1U;
        return 0;
    }

    final_crc = vmsp_crc32_final(s_running_crc);
    if (final_crc != expected_crc || final_crc != s_expected_crc) {
        printf("[OTA] CRC mismatch in RAM: calc %08X != exp %08X\r\n",
               (unsigned int)final_crc, (unsigned int)expected_crc);
        out_payload[0] = 2U;  /* CRC mismatch */
        *out_len = 1U;
        return 0;
    }

    /* Flush D-cache of DDR3 staging buffer before SPI flash write */
    ae350_dcache_flush_range(OTA_DDR3_STAGING_ADDR, s_total_size);

    num_sectors = (s_total_size + SPI_FLASH_SECTOR - 1U) / SPI_FLASH_SECTOR;
    erase_len = num_sectors * SPI_FLASH_SECTOR;

    printf("[OTA] Erasing %u sectors (%u B) at Slot B (0x%06X)...\r\n",
           (unsigned int)num_sectors, (unsigned int)erase_len, (unsigned int)OTA_SLOT_B_FLASH_ADDR);

    rc = spi_flash_erase(OTA_SLOT_B_FLASH_ADDR, erase_len);
    if (rc != SPI_FLASH_OK) {
        printf("[OTA] Flash erase failed: rc %d\r\n", rc);
        out_payload[0] = 3U;  /* Erase failed */
        *out_len = 1U;
        return 0;
    }

    printf("[OTA] Programming %u B to Slot B (0x%06X)...\r\n",
           (unsigned int)s_total_size, (unsigned int)OTA_SLOT_B_FLASH_ADDR);

    rc = spi_flash_write(OTA_SLOT_B_FLASH_ADDR, (const uint8_t *)OTA_DDR3_STAGING_ADDR, s_total_size);
    if (rc != SPI_FLASH_OK) {
        printf("[OTA] Flash write failed: rc %d\r\n", rc);
        out_payload[0] = 4U;  /* Write failed */
        *out_len = 1U;
        return 0;
    }

    /* Read back and verify CRC */
    printf("[OTA] Verifying flash read-back CRC...\r\n");
    for (off = 0; off < s_total_size; off += READ_CHUNK) {
        uint32_t chunk = (s_total_size - off > READ_CHUNK) ? READ_CHUNK : (s_total_size - off);
        rc = spi_flash_read(OTA_SLOT_B_FLASH_ADDR + off, read_chk, chunk);
        if (rc != SPI_FLASH_OK) {
            printf("[OTA] Flash read-back failed at +%u: rc %d\r\n", (unsigned int)off, rc);
            out_payload[0] = 5U;  /* Read verify failed */
            *out_len = 1U;
            return 0;
        }
        rb_crc = vmsp_crc32_update(rb_crc, read_chk, chunk);
    }
    rb_crc = vmsp_crc32_final(rb_crc);

    if (rb_crc != expected_crc) {
        printf("[OTA] Read-back CRC mismatch: %08X != %08X\r\n",
               (unsigned int)rb_crc, (unsigned int)expected_crc);
        out_payload[0] = 6U;  /* Verify CRC mismatch */
        *out_len = 1U;
        return 0;
    }

    /* Write metadata activating Slot B */
    s_meta.magic = OTA_MAGIC_VAL;
    s_meta.active_slot = 1U;  /* Slot B */
    s_meta.slot_b_crc = expected_crc;
    s_meta.slot_b_size = s_total_size;
    s_meta.boot_status = OTA_BOOT_STATUS_TENTATIVE;
    s_meta.boot_attempts = 0U;
    s_meta.header_crc = vmsp_crc32((const uint8_t *)&s_meta, sizeof(ota_meta_t) - 4U);

    rc = write_metadata(&s_meta);
    if (rc != SPI_FLASH_OK) {
        printf("[OTA] Metadata write failed: rc %d\r\n", rc);
        out_payload[0] = 7U;  /* Metadata write failed */
        *out_len = 1U;
        return 0;
    }

    s_meta_loaded = true;
    s_session_active = false;
    printf("[OTA] Update committed successfully! Slot B ready for reboot.\r\n");

    out_payload[0] = 0U;  /* OK */
    out_payload[1] = 1U;  /* Slot B */
    wr_be16(out_payload + 2, (uint16_t)num_sectors);
    wr_be32(out_payload + 4, rb_crc);
    *out_len = 8U;
    return 0;
}

int ota_reboot(uint32_t delay_ms)
{
    printf("[OTA] Rebooting SoC in %u ms...\r\n", (unsigned int)delay_ms);
    vTaskDelay(pdMS_TO_TICKS(delay_ms ? delay_ms : 100U));

    /* Disable interrupts globally */
    asm volatile ("csrci mstatus, 8");

    /* Stop Ethernet RX/TX to prevent DMA / interrupts across reset */
    DEV_ETHERNET->ETH_RX_IE = 0U;
    DEV_ETHERNET->ETH_TX_IE = 0U;
    DEV_ETHERNET->ETH_MODE = 0U;

    /* Flush and invalidate caches, clear mcause to prevent NMI trap */
    asm volatile ("fence iorw, iorw\n\t"
                  "fence rw, rw\n\t"
                  "li t0, 6\n\t"
                  "csrw 0x7cc, t0\n\t"
                  "csrw mcause, zero\n\t"
                  "fence.i");

    for (volatile uint32_t spin = 0; spin < 100000U; ++spin) { }

    DEV_SMU->HART0_RESET_VECTOR = 0x80000000UL;
    DEV_SMU->SMUCR = SMU_CMD_SW_RESET;
    for (;;) { }
    return 0;
}
