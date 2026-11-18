/*
 * spi_flash.c - the board's SPI flash (R-39). See spi_flash.h.
 *
 * The flash is served by our own controller in the FPGA, fpga/src/flash_ctl.v:
 * its ROM side is the CPU's instruction memory 0x80000000 (the boot window), its
 * command side sits on the extended APB slave at 0xF8000000 (ERRORS 5.12 - the
 * SoC's own flash controller is gone from the bitstream). One SPI command per
 * write of CMD, data through a 256-byte buffer; the controller itself refuses
 * erase and program outside 0xF00000..0xFFFFFF and every other command but read,
 * status, JEDEC id and write enable / disable.
 *
 * Without that controller (an older bitstream) nothing here touches 0xF8000000 -
 * a CPU access there never completes and hangs the core - and reads go through
 * the boot window.
 */

#include "spi_flash.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "cache.h"
#include <stdio.h>
#include <string.h>

// flash_ctl.v register map.
#define FC_BASE         0xF8000000UL
#define FC_REG(o)       (*(volatile uint32_t *)(FC_BASE + (o)))
#define FC_MAGIC        FC_REG(0x000)
#define FC_CMD          FC_REG(0x004)
#define FC_ADDR         FC_REG(0x008)
#define FC_STATUS       FC_REG(0x00C)
#define FC_ROMCNT       FC_REG(0x010)
#define FC_BUF(i)       FC_REG(0x100 + 4U * (i))
#define FC_MAGIC_VAL    0x46435431UL        /* "FCT1" */
#define FC_ST_BUSY      (1UL << 0)
#define FC_ST_REFUSED   (1UL << 1)
#define FC_CMD_ADDR     (1UL << 8)
#define FC_CMD_READ     (1UL << 9)
#define FC_CMD_N(n)     ((uint32_t)(n) << 10)

#define CMD_READ        0x03U
#define CMD_PP          0x02U
#define CMD_SE          0x20U   /* 4 KB sector erase */
#define CMD_WREN        0x06U
#define CMD_RDSR        0x05U
#define CMD_RDID        0x9FU

#define SPIN_LIMIT      2000000UL      /* status polls before giving up (~100 ms) */
#define CHUNK           256U           /* the controller's buffer */

static SemaphoreHandle_t s_lock;
static bool s_present;
static volatile uint32_t s_alarm;

void spi_flash_alarm_set(uint32_t bits)
{
    taskENTER_CRITICAL();
    s_alarm |= bits;
    taskEXIT_CRITICAL();
}

uint32_t spi_flash_alarm_get(void)
{
    return s_alarm;
}

// flash_ctl.v answers a ROM read at 0x8F000000 with its magic, without touching
// the flash; the SoC's own flash controller returns flash data there (0x600000,
// the boot loader's first word). The ROM window exists in every bitstream, so
// this probe is safe where an access to 0xF8000000 would hang the core.
void spi_flash_init(void)
{
    static bool probed = false;
    uint32_t id;

    if (probed) return;                           /* frame_udp_init() asks first, then mm_init() */
    probed = true;
    if (s_lock == NULL) s_lock = xSemaphoreCreateMutex();
    ae350_dcache_flush_all();                     /* the window is cacheable */
    id = *(volatile const uint32_t *)SPI_FLASH_ID_ADDR;
    s_present = (id == FC_MAGIC_VAL);
    if (s_present) {
        printf("[FLASH] flash_ctl command port: magic %08X, ROM words %u\r\n",
               (unsigned int)FC_MAGIC, (unsigned int)FC_ROMCNT);
    } else {
        printf("[FLASH] no flash_ctl in the bitstream (0x%08X at 0x8F000000): reads through the boot window, no writes\r\n",
               (unsigned int)id);
    }
}

bool spi_flash_present(void)
{
    return s_present;
}

uint32_t spi_flash_build_id(void)
{
    return s_present ? FC_REG(0x014) : 0U;      /* 0 on a flash_ctl from before 23.10.2026 */
}

uint32_t spi_flash_fpga_features(void)
{
    return s_present ? FC_REG(0x018) : 0U;      /* 0 on a flash_ctl from before 29.10.2026 */
}

static bool lock(void)
{
    if (s_lock == NULL) return false;
    return xSemaphoreTake(s_lock, pdMS_TO_TICKS(2000)) == pdTRUE;
}

static void unlock(void)
{
    xSemaphoreGive(s_lock);
}

static int wait_idle(void)
{
    uint32_t n = 0;
    while (FC_STATUS & FC_ST_BUSY)
        if (++n > SPIN_LIMIT) return SPI_FLASH_ERR_TIMEOUT;
    return SPI_FLASH_OK;
}

// One command: opcode, [3-byte address], n data bytes out of / into the buffer.
static int fc_cmd(uint8_t op, bool has_addr, bool rd, uint32_t addr, uint32_t n)
{
    int rc;

    if (n > CHUNK) return SPI_FLASH_ERR_RANGE;
    if ((rc = wait_idle()) != SPI_FLASH_OK) return rc;
    FC_ADDR = addr & 0x00FFFFFFUL;
    FC_CMD  = op | (has_addr ? FC_CMD_ADDR : 0UL) | (rd ? FC_CMD_READ : 0UL) | FC_CMD_N(n);
    if ((rc = wait_idle()) != SPI_FLASH_OK) return rc;
    return (FC_STATUS & FC_ST_REFUSED) ? SPI_FLASH_ERR_REFUSED : SPI_FLASH_OK;
}

static void buf_put(const uint8_t *src, uint32_t n)
{
    uint32_t i, w;

    for (i = 0; i < n; i += 4U) {
        w = src[i];
        if (i + 1U < n) w |= (uint32_t)src[i + 1U] << 8;
        if (i + 2U < n) w |= (uint32_t)src[i + 2U] << 16;
        if (i + 3U < n) w |= (uint32_t)src[i + 3U] << 24;
        FC_BUF(i / 4U) = w;
    }
}

static void buf_get(uint8_t *dst, uint32_t n)
{
    uint32_t i, w = 0;

    for (i = 0; i < n; ++i) {
        if ((i & 3U) == 0U) w = FC_BUF(i / 4U);
        dst[i] = (uint8_t)(w >> (8U * (i & 3U)));
    }
}

// Wait for the chip's write-in-progress bit; erase of a 4 KB sector takes up to
// ~400 ms, a page program ~3 ms.
static int wait_ready(uint32_t timeout_ms)
{
    uint32_t t;
    int rc;

    for (t = 0; ; ++t) {
        if ((rc = fc_cmd(CMD_RDSR, false, true, 0, 1)) != SPI_FLASH_OK) return rc;
        if (!(FC_BUF(0) & 0x01UL)) return SPI_FLASH_OK;
        if (t >= timeout_ms) return SPI_FLASH_ERR_TIMEOUT;
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            for (volatile int spin = 0; spin < 10000; spin++) { }
        }
    }
}

static bool in_data(uint32_t addr, uint32_t len)
{
    return len > 0U && addr >= SPI_FLASH_DATA_BASE && addr < SPI_FLASH_DATA_END &&
           len <= SPI_FLASH_DATA_END - addr;
}

uint32_t spi_flash_id(void)
{
    uint8_t id[3] = { 0, 0, 0 };

    if (!s_present || !lock()) return 0U;
    if (fc_cmd(CMD_RDID, false, true, 0, 3) == SPI_FLASH_OK) buf_get(id, 3);
    unlock();
    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

int spi_flash_read(uint32_t addr, uint8_t *dst, uint32_t len)
{
    uint32_t n;
    int rc = SPI_FLASH_OK;

    if (dst == NULL || addr >= SPI_FLASH_DATA_END || len > SPI_FLASH_DATA_END - addr)
        return SPI_FLASH_ERR_RANGE;
    if (!s_present) {
        // Through the boot window, which every bitstream has. Its lines are never
        // dirty, so dropping the cache cannot lose data.
        ae350_dcache_flush_all();
        memcpy(dst, (const void *)SPI_FLASH_WINDOW(addr), len);
        return SPI_FLASH_OK;
    }
    // Through the command port: no cache in the way, right after a program too.
    if (!lock()) return SPI_FLASH_ERR_BUSY;
    while (len && rc == SPI_FLASH_OK) {
        n = (len < CHUNK) ? len : CHUNK;
        if ((rc = fc_cmd(CMD_READ, true, true, addr, n)) == SPI_FLASH_OK) buf_get(dst, n);
        addr += n; dst += n; len -= n;
    }
    unlock();
    return rc;
}

static uint8_t read_sr(void)
{
    return (fc_cmd(CMD_RDSR, false, true, 0, 1) == SPI_FLASH_OK) ? (uint8_t)FC_BUF(0) : 0xEEU;
}

// A verify mismatch: what differs, whether a second read agrees, and the chip's status.
// "1->0" bits are zeros the program did not ask for (the erase did not take, or wrong data
// went out); "0->1" bits are zeros it asked for and did not get.
static void verify_report(uint32_t addr, const uint8_t *exp, const uint8_t *got, uint32_t n)
{
    static uint8_t again[CHUNK];
    uint32_t i, diff = 0, first = 0, b10 = 0, b01 = 0, rr_exp = 0, rr_got = 0;
    uint8_t x;

    for (i = 0; i < n; ++i) {
        if (exp[i] == got[i]) continue;
        if (diff++ == 0U) first = i;
        for (x = exp[i] & (uint8_t)~got[i]; x; x &= (uint8_t)(x - 1U)) ++b10;
        for (x = got[i] & (uint8_t)~exp[i]; x; x &= (uint8_t)(x - 1U)) ++b01;
    }
    if (fc_cmd(CMD_READ, true, true, addr, n) == SPI_FLASH_OK) {
        buf_get(again, n);
        for (i = 0; i < n; ++i) { rr_exp += again[i] != exp[i]; rr_got += again[i] != got[i]; }
    } else {
        rr_exp = rr_got = 0xFFFFFFFFUL;
    }
    printf("[FLASH] verify 0x%06X n %u: %u B differ, first +%u exp %02X got %02X, bits 1->0 %u 0->1 %u; "
           "re-read differs from exp %u, from 1st read %u; SR %02X FC %08X\r\n",
           (unsigned)addr, (unsigned)n, (unsigned)diff, (unsigned)first, exp[first], got[first],
           (unsigned)b10, (unsigned)b01, (unsigned)rr_exp, (unsigned)rr_got, read_sr(), (unsigned)FC_STATUS);
}

int spi_flash_erase(uint32_t addr, uint32_t len)
{
    static uint8_t chk[CHUNK];
    uint32_t a, end, o, i;
    int rc = SPI_FLASH_OK;

    if (!in_data(addr, len)) return SPI_FLASH_ERR_RANGE;
    a   = addr & ~(SPI_FLASH_SECTOR - 1U);
    end = addr + len;
    if (!s_present) return SPI_FLASH_ERR_ABSENT;
    if (!lock()) return SPI_FLASH_ERR_BUSY;
    for (; a < end && rc == SPI_FLASH_OK; a += SPI_FLASH_SECTOR) {
        if (a < SPI_FLASH_DATA_BASE) { rc = SPI_FLASH_ERR_RANGE; break; }
        if ((rc = fc_cmd(CMD_WREN, false, false, 0, 0)) != SPI_FLASH_OK) break;
        if ((rc = fc_cmd(CMD_SE, true, false, a, 0)) != SPI_FLASH_OK) break;
        if ((rc = wait_ready(1000U)) != SPI_FLASH_OK) break;
        // Blank check: a program onto a sector the erase missed only clears bits, silently.
        for (o = 0; o < SPI_FLASH_SECTOR && rc == SPI_FLASH_OK; o += CHUNK) {
            if ((rc = fc_cmd(CMD_READ, true, true, a + o, CHUNK)) != SPI_FLASH_OK) break;
            buf_get(chk, CHUNK);
            for (i = 0; i < CHUNK && chk[i] == 0xFFU; ++i) {}
            if (i < CHUNK) {
                printf("[FLASH] erase 0x%06X not blank: +%u = %02X; SR %02X FC %08X\r\n",
                       (unsigned)a, (unsigned)(o + i), chk[i], read_sr(), (unsigned)FC_STATUS);
                rc = SPI_FLASH_ERR_VERIFY;
            }
        }
    }
    unlock();
    return rc;
}

int spi_flash_write(uint32_t addr, const uint8_t *src, uint32_t len)
{
    static uint8_t chk[CHUNK];
    uint32_t n;
    uint8_t sr;
    int rc = SPI_FLASH_OK;

    if (src == NULL || !in_data(addr, len)) return SPI_FLASH_ERR_RANGE;
    if (!s_present) return SPI_FLASH_ERR_ABSENT;
    if (!lock()) return SPI_FLASH_ERR_BUSY;
    while (len && rc == SPI_FLASH_OK) {
        n = SPI_FLASH_PAGE - (addr & (SPI_FLASH_PAGE - 1U));   /* never across a page */
        if (n > len) n = len;
        if ((rc = fc_cmd(CMD_WREN, false, false, 0, 0)) != SPI_FLASH_OK) break;
        if (((sr = read_sr()) & 0x03U) != 0x02U)                /* WEL set, WIP clear */
            printf("[FLASH] 0x%06X: SR %02X after WREN\r\n", (unsigned)addr, sr);
        buf_put(src, n);                                        /* RDSR above used byte 0 */
        buf_get(chk, n);
        if (memcmp(chk, src, n) != 0) {
            for (sr = 0; chk[sr] == src[sr]; ++sr) {}
            printf("[FLASH] 0x%06X: buffer reads back wrong before the program: +%u exp %02X got %02X\r\n",
                   (unsigned)addr, (unsigned)sr, src[sr], chk[sr]);
            rc = SPI_FLASH_ERR_VERIFY;
            break;
        }
        if ((rc = fc_cmd(CMD_PP, true, false, addr, n)) != SPI_FLASH_OK) break;
        if ((rc = wait_ready(50U)) != SPI_FLASH_OK) break;
        if ((rc = fc_cmd(CMD_READ, true, true, addr, n)) != SPI_FLASH_OK) break;
        buf_get(chk, n);
        if (memcmp(chk, src, n) != 0) { verify_report(addr, src, chk, n); rc = SPI_FLASH_ERR_VERIFY; break; }
        addr += n; src += n; len -= n;
    }
    unlock();
    return rc;
}
