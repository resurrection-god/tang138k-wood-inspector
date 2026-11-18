/*
 * mailbox.c - CPU -> FPGA mailbox (mailbox.h).
 */

#include "mailbox.h"
#include "memory_map.h"
#include "cache.h"
#include "spi_flash.h"
#include "frame_udp_map.h"     // FU_APB_BASE: apb_ctl.v, the mailbox is its words 0..63
#include <stddef.h>

// DDR3 (MAILBOX_ADDR) or apb_ctl.v's registers (FEATURES bit 1), decided on the first call:
// spi_flash_init() is idempotent and only reads the controller's identity and FEATURES.
static volatile uint32_t *s_mbx;
static bool               s_apb;

static volatile uint32_t *mbx(void)
{
    if (s_mbx == NULL) {
        spi_flash_init();
        s_apb = (spi_flash_fpga_features() & SPI_FLASH_FEAT_CTL_APB) != 0U;
        s_mbx = (volatile uint32_t *)(s_apb ? FU_APB_BASE : MAILBOX_ADDR);
    }
    return s_mbx;
}

bool mailbox_on_apb(void)
{
    (void)mbx();
    return s_apb;
}

void mailbox_put(uint32_t word, uint32_t value)
{
    mbx()[word] = value;
}

// On the APB the words are registers: no cache to flush, only the order of the stores
// (the counter last) must hold.
void mailbox_publish(uint32_t seq_word, uint32_t seq)
{
    volatile uint32_t *m = mbx();

    if (s_apb) {
        __asm__ volatile ("fence" ::: "memory");
        m[seq_word] = seq;
        __asm__ volatile ("fence" ::: "memory");
        return;
    }
    ae350_dcache_flush_all();
    m[seq_word] = seq;
    ae350_dcache_flush_all();
}

void mailbox_flush(void)
{
    if (mailbox_on_apb()) {
        __asm__ volatile ("fence" ::: "memory");
        return;
    }
    ae350_dcache_flush_all();
}

// A ranged invalidate drops cache lines WITHOUT writing them out (CLAUDE.md
// invariant 4), while the mailbox is written by the CPU. That is safe only
// because every mailbox write ends with mailbox_publish() or mailbox_flush() in
// the same function: there are no dirty mailbox lines by the time of
// mailbox_load(). Rule for all users: mailbox_put() without a flush at the end
// of the same operation is not allowed.
void mailbox_load(uint32_t bytes)
{
    if (mailbox_on_apb()) return;           // registers: read directly
    ae350_dcache_invalidate_range(MAILBOX_ADDR, bytes);
}

uint32_t mailbox_get(uint32_t word)
{
    return mbx()[word];
}
