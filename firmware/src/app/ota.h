/*
 * ota.h - Over-The-Air safe firmware and bitstream updates (Dual-Slot A/B, Zero-Brick).
 */

#ifndef APP_OTA_H_
#define APP_OTA_H_

#include <stdint.h>
#include <stdbool.h>

#define OTA_MAGIC_VAL           0x4F544131UL    /* "OTA1" */
#define OTA_SLOT_A_FLASH_ADDR   0x00600000UL    /* Golden firmware (read-only) */
#define OTA_SLOT_B_FLASH_ADDR   0x00F60000UL    /* Upgrade firmware */
#define OTA_META_FLASH_ADDR     0x00F50000UL    /* OTA metadata sector (4 KB) */
#define OTA_DDR3_STAGING_ADDR   0x11000000UL    /* 1 MB staging buffer in DDR3 */
#define OTA_MAX_FW_BYTES        (512U * 1024U)  /* Max supported firmware size (512 KB) */

#define OTA_BOOT_STATUS_CONFIRMED 0U
#define OTA_BOOT_STATUS_TENTATIVE 1U
#define OTA_BOOT_STATUS_FAILED    2U

#define OTA_TARGET_FIRMWARE     0U
#define OTA_TARGET_BITSTREAM    1U

#define OTA_MAX_BOOT_ATTEMPTS   3U

#define OTA_DDR3_TRACKER_ADDR   0x06000780UL
#define OTA_TRACKER_MAGIC       0x4F544154UL    /* "OTAT" */

#define FW_VERSION_STR_SLOT_A   "TangMega-v1.0.0-GOLDEN (Slot A)"
#define FW_VERSION_STR_SLOT_B   "TangMega-v2.1.0-CANDIDATE (Slot B)"
#define FW_VERSION_STR_LITE_A   "TM-v1.0.0-LITE-GOLDEN (Slot A)"
#define FW_VERSION_STR_LITE_B   "TM-v2.1.0-LITE-CANDIDATE (Slot B)"

#define OTA_LED_PATTERN_CALM    0U  /* Calm 1 Hz alternating (LED0 / LED1) */
#define OTA_LED_PATTERN_DOUBLE  1U  /* Synchronous double-strobe */

typedef struct {
    uint32_t magic;           /* 0x4F544131 ("OTA1") */
    uint32_t active_slot;     /* 0 = Slot A, 1 = Slot B */
    uint32_t slot_a_crc;
    uint32_t slot_a_size;
    uint32_t slot_b_crc;
    uint32_t slot_b_size;
    uint32_t boot_status;     /* OTA_BOOT_STATUS_* */
    uint32_t boot_attempts;   /* boot attempts since switch */
    uint32_t header_crc;      /* CRC-32 of first 8 words */
} ota_meta_t;

void ota_init(void);
void ota_confirm_boot(void);
uint8_t ota_current_slot(void);
const char *ota_get_version_string(void);
uint8_t ota_get_led_pattern(void);

/* VMSP service handlers */
int ota_get_status(uint8_t *out_payload, uint16_t *out_len);
int ota_start(uint8_t target, uint32_t total_size, uint32_t expected_crc, uint8_t *out_payload, uint16_t *out_len);
int ota_data(uint32_t offset, const uint8_t *data, uint16_t len, uint8_t *out_payload, uint16_t *out_len);
int ota_commit(uint32_t expected_crc, uint8_t *out_payload, uint16_t *out_len);
int ota_reboot(uint32_t delay_ms);
int ota_set_active_slot(uint8_t slot, uint8_t *out_payload, uint16_t *out_len);

#endif /* APP_OTA_H_ */
