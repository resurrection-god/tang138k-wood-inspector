/*
 * sync.c - sync block: shadow of mailbox words 6..20, software trigger, telemetry
 *
 * Part of net_stack.c until 24.09.2026.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "board.h"
#include "memory_map.h"
#include "mailbox.h"
#include "sync.h"
#include "cache.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "arch/sys_arch.h"
#include "arch/ethernetif.h"



// ---- Sync block: the DDR3 mailbox --------------------------------------------
//
// Shadow of mailbox words 6..20. DDR3 keeps the mailbox across a CPU reboot,
// and the FPGA has already seen its counters: they must continue from what is
// in memory, otherwise the first command after a firmware restart would match
// the old value and not be applied.
static uint32_t s_sync[SYNC_W_COUNT];
static bool     s_sync_init = false;

// sync_block values after an FPGA reset: TIMER, cameras 1 ms, laser off
// (always 0), filter 1 us. In lane4_clk clocks (board.h); they must match the
// DEF_* of the u_sync_block instance in scanner_top.v. They are also the default
// of the mode settings sets when the store holds nothing.
void sync_default_words(uint32_t *w)
{
    memset(w, 0, SYNC_W_COUNT * sizeof(uint32_t));
    w[SYNC_W_MAGIC]       = SYNC_MAGIC;
    w[SYNC_W_CTRL]        = SYNC_CTRL_ENABLE | (1U << SYNC_CTRL_SRC_SHIFT);
    w[SYNC_W_CAM0_WIDTH]  = (uint32_t)LANE_US_TO_TICKS(1000U);   /* 1 ms */
    w[SYNC_W_CAM1_WIDTH]  = (uint32_t)LANE_US_TO_TICKS(1000U);
    // The laser is on for the whole exposure (docs/ARCHITECTURE.md § 5, ERRORS 4.2):
    // the default width is 6 ms, the default exposure 5 ms plus margin.
    // It used to be 100 ns, a value computed for a short strobe, which with the
    // laser on gave nothing in the frame (measured 18.09.2026: 200 us of light in a
    // 5 ms exposure is indistinguishable from the background).
    //
    // The mode stays 1 ("always 0") meanwhile, so after a board reset the laser is
    // guaranteed off; the width by itself does not light it.
    w[SYNC_W_LASER_WIDTH] = LANE_NS_TO_TICKS(6000000U);   /* 6 ms */
    w[SYNC_W_LASER_MODE]  = 1U;
    w[SYNC_W_EXT_FILTER]  = LANE_NS_TO_TICKS(1000U);           /* 1 us */
}

static void sync_init(void)
{
    uint32_t w;

    if (s_sync_init) return;
    mailbox_load(128);
    if (mailbox_get(SYNC_W_MAGIC) == SYNC_MAGIC)
    {
        for (w = SYNC_W_MAGIC; w < SYNC_W_COUNT; ++w) s_sync[w] = mailbox_get(w);
    }
    else
    {
        sync_default_words(s_sync);
    }
    s_sync_init = true;
}

uint32_t sync_get(uint32_t word)
{
    sync_init();
    return (word < SYNC_W_COUNT) ? s_sync[word] : 0U;
}

void sync_set(uint32_t word, uint32_t value)
{
    sync_init();
    if (word > SYNC_W_MAGIC && word < SYNC_W_APPLY_SEQ) s_sync[word] = value;
}

// Magic and parameters, then the APPLY_SEQ counter: the mailbox.h order. SW_SEQ
// is rewritten with the same value: the block in memory must be whole.
void sync_apply(void)
{
    uint32_t w;

    sync_init();
    mailbox_put(SYNC_W_MAGIC, SYNC_MAGIC);
    for (w = SYNC_W_CTRL; w < SYNC_W_APPLY_SEQ; ++w) mailbox_put(w, s_sync[w]);
    mailbox_put(SYNC_W_SW_SEQ, s_sync[SYNC_W_SW_SEQ]);
    mailbox_publish(SYNC_W_APPLY_SEQ, ++s_sync[SYNC_W_APPLY_SEQ]);
}


// Software trigger. The FPGA polls the mailbox every ~10 ms: up to 10 ms delay,
// and two calls within one poll give one pulse.
void sync_fire(void)
{
    sync_init();
    // No parameters, only the magic and the counter, so one flush at the end.
    mailbox_put(SYNC_W_MAGIC, SYNC_MAGIC);
    mailbox_put(SYNC_W_SW_SEQ, ++s_sync[SYNC_W_SW_SEQ]);
    mailbox_flush();
}

bool sync_read_telemetry(sync_telemetry_t *t)
{
    volatile uint32_t *d = (volatile uint32_t *)VIDEO_DIAG0_ADDR;

    ae350_dcache_invalidate_range(VIDEO_DIAG0_ADDR, 128);
    t->valid     = (d[0] == VIDEO_FRAME_MAGIC) && (d[SYNC_T_ID] == SYNC_MAGIC);
    t->status    = d[SYNC_T_STATUS];
    t->accepted  = d[SYNC_T_ACCEPTED];
    t->overrun   = d[SYNC_T_OVERRUN];
    t->ext_edges = d[SYNC_T_EXT_EDGES];
    t->last_ts   = d[SYNC_T_LAST_TS];
    t->apply_seq = d[SYNC_T_APPLY_SEQ];
    t->sw_seq    = d[SYNC_T_SW_SEQ];
    return t->valid;
}
