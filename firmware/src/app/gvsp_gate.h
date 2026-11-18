/*
 * gvsp_gate.h - the "do not overwrite an unacknowledged slot" gate of guaranteed delivery
 * (docs/protocol/GEV_PROTOCOL_DESIGN.md § 5.5, B3), as a pure decision: no RTOS, no memory
 * access of its own, so firmware/tests/gate_bench.py runs it against a model of the FPGA.
 *
 * The FPGA writes every frame's profile block into the dense ring (8192 slots, slot = frame
 * mod 8192) unconditionally; the only lever is the frame mode (no-frame bit, ~10 ms through
 * the mailbox, taken at a frame's slot open): with it off the frame goes to the 32-slot frame
 * ring and the dense ring is not touched. So: shut (no-frame off) when the head reaches
 * GVSP_GATE_MARGIN frames short of the oldest unacknowledged slot. The FPGA then waits at
 * the block the shut cut; once the host has acknowledged everything before it, that block
 * goes out whole as a resend (its held-off lines flagged), and with it acknowledged the
 * dense ring holds nothing unacknowledged: open (no-frame on) and restart the stream record
 * at the first block whose frames all went to the dense ring. Blocks in the gap are lost
 * and counted; a resend of one sends its lines as not delivered (udp_rd.v checks every
 * line's frame number). A host that never acknowledges keeps the gate shut: nothing it has
 * not confirmed is destroyed.
 */
#ifndef APP_GVSP_GATE_H_
#define APP_GVSP_GATE_H_

#include <stdint.h>
#include <stdbool.h>

#define GVSP_GATE_SLOTS     8192U       /* LASER_DRY_SLOTS */
#define GVSP_GATE_MARGIN    512U        /* frames: > firmware + mailbox latency at 2 kHz */
#define GVSP_GATE_EVENT_MS  1000U

// What the caller reads for the gate: whether frame f's dense slot holds f, and the newest
// frame in the frame ring (0 = none).
typedef struct {
    bool     (*dense_has)(void *ctx, uint32_t frame);
    uint32_t (*ring_newest)(void *ctx);
    void     *ctx;
} gvsp_gate_io_t;

typedef struct {
    bool     gated, resuming, flushed;
    uint32_t acked_end;             /* frames below are acknowledged or gone (mod 2^32) */
    uint32_t resume;                /* candidate first frame after the gap */
    uint32_t overflow;              /* blocks lost to the gate */
    uint32_t ev_ms;
    uint64_t flush_bid;             /* the block the shut cut, sent as a resend */
} gvsp_gate_t;

// Actions for the caller, a bit mask.
#define GVSP_GATE_SHUT      0x01U   /* no-frame off: new frames to the frame ring */
#define GVSP_GATE_OPEN      0x02U   /* no-frame on: the dense ring again */
#define GVSP_GATE_RESTART   0x04U   /* the stream record from frame g->resume */
#define GVSP_GATE_EVENT     0x08U   /* StreamOverflow due (at the shut, then every second) */
#define GVSP_GATE_FLUSH     0x10U   /* send block g->flush_bid whole, as a resend */

void gvsp_gate_init(gvsp_gate_t *g, uint32_t start);
// One step: n lines a block, ack = the host's StreamAckedBlockId (0 none), next = the FPGA's
// next frame of the stream (ps_valid false: unknown).
uint32_t gvsp_gate_step(gvsp_gate_t *g, uint32_t n, uint64_t ack, bool ps_valid, uint32_t next,
                        uint32_t now_ms, const gvsp_gate_io_t *io);

#endif /* APP_GVSP_GATE_H_ */
