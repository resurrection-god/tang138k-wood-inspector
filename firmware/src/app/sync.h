/*
 * sync.h - sync block: mailbox words 6..20 and telemetry
 *
 * Part of net_stack.h until 24.09.2026.
 */

#ifndef APP_SYNC_H_
#define APP_SYNC_H_

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "memory_map.h"

// ---- Sync block (stage 2, docs/ARCHITECTURE.md § 5) -------------------------
//
// Control goes through the same mailbox MAILBOX_ADDR, words 6..20; the FPGA
// reads it over Lane 5 every ~10 ms. Status is in words 23..30 of the telemetry
// block 0x06000000 (updated every 0.5 s). There are no registers on the AHB
// bus: they did not get along with the MAC timing (docs/DECISIONS.md R-13,
// ERRORS former 31.35).
#define SYNC_W_MAGIC            6
#define SYNC_W_CTRL             7
#define SYNC_W_CAM0_DELAY       8
#define SYNC_W_CAM0_WIDTH       9
#define SYNC_W_CAM1_DELAY       10
#define SYNC_W_CAM1_WIDTH       11
#define SYNC_W_LASER_DELAY      12
#define SYNC_W_LASER_WIDTH      13
#define SYNC_W_LASER_MODE       14
#define SYNC_W_PRE_LEAD         15
#define SYNC_W_PRE_WIDTH        16
#define SYNC_W_POLARITY         17
#define SYNC_W_EXT_FILTER       18
#define SYNC_W_APPLY_SEQ        19   /* changed: apply words 7..18 */
#define SYNC_W_SW_SEQ           20   /* changed: software trigger (CORE) */
#define SYNC_W_COUNT            21
#define SYNC_MAGIC              0x53594E43U   /* "SYNC" */

#define SYNC_T_ID               23
#define SYNC_T_STATUS           24   /* [0] enabled [1] exposure_done [2] active [3] overrun [8] loaded */
#define SYNC_T_ACCEPTED         25
#define SYNC_T_OVERRUN          26
#define SYNC_T_EXT_EDGES        27
#define SYNC_T_LAST_TS          28
#define SYNC_T_APPLY_SEQ        29
#define SYNC_T_SW_SEQ           30

#define SYNC_CTRL_ENABLE        0x001U
#define SYNC_CTRL_SRC_SHIFT     2
#define SYNC_CTRL_SRC_MASK      0x00CU
#define SYNC_CTRL_EXT_FALL      0x010U
#define SYNC_CTRL_LASER_EN      0x100U

typedef struct {
    bool     valid;                 // the telemetry has the block's words
    uint32_t status, accepted, overrun, ext_edges, last_ts, apply_seq, sw_seq;
} sync_telemetry_t;

uint32_t sync_get(uint32_t word);             // firmware shadow
void     sync_set(uint32_t word, uint32_t value);   // has no effect until sync_apply()
void     sync_apply(void);                    // write the mailbox and APPLY_SEQ + 1
void     sync_fire(void);                     // SW_SEQ + 1
bool     sync_read_telemetry(sync_telemetry_t *t);


// Word defaults after an FPGA reset; also the default of the mode sets
// (settings.c).
void     sync_default_words(uint32_t *w);

#endif /* APP_SYNC_H_ */
