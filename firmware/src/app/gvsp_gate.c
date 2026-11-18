/*
 * gvsp_gate.c - the guaranteed-delivery gate (gvsp_gate.h).
 */

#include <string.h>

#include "gvsp_gate.h"

void gvsp_gate_init(gvsp_gate_t *g, uint32_t start)
{
    memset(g, 0, sizeof(*g));
    g->acked_end = start;                                /* nothing acknowledged yet */
}

uint32_t gvsp_gate_step(gvsp_gate_t *g, uint32_t n, uint64_t ack, bool ps_valid, uint32_t next,
                        uint32_t now_ms, const gvsp_gate_io_t *io)
{
    uint32_t a = (uint32_t)(ack * n), f, sent = next;    /* block b covers frames (b-1)N..bN-1 */
    bool held_last;

    if (!ps_valid) return 0;
    if (g->gated && g->flushed) sent = next + n;         /* the stuck block went out by itself */
    // An ack is only taken forward and never past what has been sent.
    if (ack != 0U && (int32_t)(a - g->acked_end) > 0 && (int32_t)(sent - a) >= 0) g->acked_end = a;
    if (!g->gated) {
        f = g->acked_end + GVSP_GATE_SLOTS - GVSP_GATE_MARGIN;
        // The head is past f: its slot holds it, or the FPGA has sent past it.
        if (!io->dense_has(io->ctx, f) && (int32_t)(next - f) < 0) return 0;
        g->gated = true;
        g->resuming = g->flushed = false;
        g->ev_ms = now_ms;
        return GVSP_GATE_SHUT | GVSP_GATE_EVENT;
    }
    // The newest held-off frame (0: none yet; older than the stream: a capture before it).
    f = io->ring_newest(io->ctx);
    held_last = f != 0U && (int32_t)(f - (next + n - 1U)) >= 0 && !io->dense_has(io->ctx, next + n - 1U);
    if (!g->resuming) {
        // The FPGA waits at the block the shut cut - once its last frame went to the frame
        // ring (before that the shut has not taken effect and the block is still filling).
        // That block goes out as a resend, its held-off lines flagged; with it acknowledged
        // nothing in the dense ring is unacknowledged, and the ring opens again.
        if (!g->flushed && held_last) {
            g->flushed = true;
            g->flush_bid = next / n + 1U;
            return GVSP_GATE_FLUSH;
        }
        if (!g->flushed || (int32_t)(g->acked_end - (next + n)) < 0) {
            if (now_ms - g->ev_ms < GVSP_GATE_EVENT_MS) return 0;
            g->ev_ms = now_ms;
            return GVSP_GATE_EVENT;
        }
        // From the first block after the newest held-off frame: past the flushed block, as
        // that block's last frame was held off.
        g->resuming = true;
        g->resume = (f + n) & ~(n - 1U);
        return GVSP_GATE_OPEN;
    }
    // Held-off frames first: once a frame has gone to the dense ring after the open, every
    // later one does too, so the resume frame in its slot means the whole block is there.
    if ((int32_t)(f - g->resume) >= 0) {
        g->resume = (f + n) & ~(n - 1U);                 /* that frame went to the frame ring too */
        return 0;
    }
    if (!io->dense_has(io->ctx, g->resume)) return 0;
    g->gated = g->resuming = false;
    // The blocks after the flushed one up to the resume frame are lost; below it every
    // frame is acknowledged or gone, so the ring is protected from there on.
    g->overflow += (g->resume - next) / n;
    g->acked_end = g->resume;
    return GVSP_GATE_RESTART;
}
