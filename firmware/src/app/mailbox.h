/*
 * mailbox.h - the mailbox at 0x06000100 (MAILBOX_ADDR): the CPU writes, the
 * FPGA reads it over Lane 5 every ~10 ms. The word layout is kept by the owner
 * modules: TRIG 0..5 (cam_trig.h), SYNC 6..20 (sync.h), GEOM 29..31 (video.h),
 * LASR 32..40 (laser.h).
 *
 * One protocol for everyone: block magic and parameters, CACHE FLUSH, and only
 * then the command counter, flush again. The FPGA applies the block when it
 * sees the counter change, and by then the parameters must already be in DDR3,
 * otherwise the new command goes out with the old parameters. Without the flush
 * after the counter it may stay in the CPU cache, and the FPGA never sees the
 * command at all. Until 24.09.2026 this sequence was rewritten in four modules.
 *
 * Since 30.10.2026 a bitstream with FEATURES bit 1 (spi_flash.h) has the mailbox
 * as registers of apb_ctl.v at FU_APB_BASE + 0x000..0x0FF instead (same word
 * layout; the FPGA reads them directly, the DDR3 copy is not used). The calls
 * below pick the place on their own; on the APB "flush" is only a fence, and the
 * registers keep their values over a CPU reboot as DDR3 did (not over an FPGA
 * reload, which also resets the FPGA's counters).
 */

#ifndef APP_MAILBOX_H_
#define APP_MAILBOX_H_

#include <stdint.h>
#include <stdbool.h>

// Write a mailbox word (into the CPU cache: it goes out on a flush).
void     mailbox_put(uint32_t word, uint32_t value);
// Cache flush, command counter seq into word seq_word, flush again: see above.
void     mailbox_publish(uint32_t seq_word, uint32_t seq);
// Only flush the cache out.
void     mailbox_flush(void);

// Re-read the first `bytes` bytes of the mailbox from DDR3 (drop them from the
// cache). The mailbox lives in DDR3 and survives a CPU reboot: the command
// counters continue from what the FPGA has already seen, otherwise the first
// command after a restart would match the old value and not be applied.
// bytes is at most 256 (a mailbox burst).
void     mailbox_load(uint32_t bytes);
uint32_t mailbox_get(uint32_t word);
// The mailbox is apb_ctl.v's registers (FEATURES bit 1): the mm band is then pushed by
// the CPU (mm_table.c), not fetched from DDR3.
bool     mailbox_on_apb(void);

#endif /* APP_MAILBOX_H_ */
