/*
 * gate_bench.c - the guaranteed-delivery gate (src/app/gvsp_gate.c) against a model of the
 * FPGA, udp_rd and a host that stops acknowledging (firmware/tests/gate_bench.py builds and
 * runs it, and its mutants).
 *
 * Per frame tick: the FPGA writes frame f to the dense ring (slot f mod 8192) or, with the
 * no-frame bit off, to the frame ring; a bit posted by the firmware takes effect LAT frames
 * later (mailbox read + slot open). udp_rd sends block [next, next+N) when the slot of its
 * last frame holds it (or jumps ahead when the ring overtook it), each line delivered if its
 * slot still holds it. The host takes blocks in order, resends a gap at once (lines not in
 * the ring come back as not delivered), and acknowledges every ACK_EVERY frames except
 * during stalls. The gate steps every frame.
 *
 * Checks: no frame the host has not acknowledged is ever overwritten; every frame from the
 * start is delivered once or reported not delivered; frames lost <= the gate's overflow
 * count x N; no loss without a stall; blocks never go backwards; the stream runs on after
 * the last stall. Output "PASS" or "FAIL <why>", exit code 0 / 1.
 *
 *   gate_bench N LAT ACK_EVERY STALLS STALL_LEN [STALE [START]]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gvsp_gate.h"

#define SLOTS   GVSP_GATE_SLOTS
#define FRAMES  400000U
#define NONE    0xFFFFFFFFU

static uint32_t dense[SLOTS];
static bool     dense_ok[SLOTS];       /* the slot was ever written */
static uint32_t ring_new;               /* newest frame in the frame ring, 0 none */

static bool held(uint32_t f) { return dense_ok[f % SLOTS] && dense[f % SLOTS] == f; }
static bool io_dense_has(void *ctx, uint32_t f) { (void)ctx; return held(f); }
static uint32_t io_ring_newest(void *ctx) { (void)ctx; return ring_new; }

static int fail(const char *why, unsigned long a, unsigned long b)
{
    printf("FAIL %s (%lu, %lu)\n", why, a, b);
    return 1;
}

int main(int argc, char **argv)
{
    uint32_t n, lat, ack_every, stalls, stall_len, stale;
    uint32_t start, next, f, t, i, act, host_end, host_ack_bid = 0, delivered = 0, lost = 0;
    uint32_t last_first, stall_at[16], pend_on = 0, pend_at = NONE;
    uint32_t last_stall_end = 0, delivered_at_last_stall = 0, shuts = 0;
    bool dense_mode = true, stalled;
    gvsp_gate_t g;
    gvsp_gate_io_t io = { io_dense_has, io_ring_newest, NULL };

    if (argc < 6) return fprintf(stderr, "usage: gate_bench N LAT ACK_EVERY STALLS STALL_LEN [STALE [START]]\n"), 2;
    n = (uint32_t)atoi(argv[1]); lat = (uint32_t)atoi(argv[2]); ack_every = (uint32_t)atoi(argv[3]);
    stalls = (uint32_t)atoi(argv[4]); stall_len = (uint32_t)atoi(argv[5]);
    stale = (argc > 6) ? (uint32_t)atoi(argv[6]) : 0U;
    start = (argc > 7) ? (uint32_t)strtoul(argv[7], NULL, 0) : 1024U;   /* a multiple of every N, as pstart_frame() */
    if (stalls > 16U) stalls = 16U;
    for (i = 0; i < stalls; ++i) stall_at[i] = 20000U + i * (FRAMES / (stalls + 1U));   /* ticks */
    ring_new = stale ? start - 100U : 0U;              /* a capture before the stream */
    if (ring_new == 0U) ring_new = 1U;                  /* 0 means "none" */
    if (!stale) ring_new = 0U;
    gvsp_gate_init(&g, start);
    next = start;
    host_end = start;                                    /* frames below: the host has them */
    last_first = start - n;

    // Frame numbers wrap (a month unattended at 1 kHz passes 2^31): every compare is modular.
    for (t = 0; t < FRAMES; ++t) {
        f = start + t;
        // the FPGA: a posted mode change takes effect at this frame's slot open
        if (pend_at != NONE && (int32_t)(f - pend_at) >= 0) { dense_mode = pend_on != 0U; pend_at = NONE; }
        if (dense_mode) {
            uint32_t old = dense[f % SLOTS], acked = host_ack_bid ? host_ack_bid * n : start;
            // rule 1 of § 5.5: a slot is free only once the host has acknowledged its block
            if (dense_ok[f % SLOTS] && (int32_t)(old - acked) >= 0 && (int32_t)(old - start) >= 0)
                return fail("an unacknowledged frame overwritten", old, f);
            dense[f % SLOTS] = f;
            dense_ok[f % SLOTS] = true;
        } else {
            ring_new = f;
        }
        // udp_rd: whole blocks in order
        for (;;) {
            uint32_t last = next + n - 1U, h = dense[last % SLOTS], bid, k;
            if ((int32_t)(last - f) > 0) break;
            if (!held(last)) {
                if (dense_ok[last % SLOTS] && (int32_t)(h - last) > 0 && (int32_t)(h - last) < (1 << 20)) {
                    next = h & ~(n - 1U);                /* overtaken: on from that frame's block */
                    continue;
                }
                break;
            }
            bid = next / n + 1U;
            // by frame: block_id64 from a 32-bit frame number wraps (OPEN_ISSUES 1.4), the gate does not
            if ((int32_t)(next - last_first) <= 0) return fail("blocks went backwards", next, last_first);
            // the host: a gap is resent now, lines the ring no longer holds come back flagged
            for (; (int32_t)(host_end - next) < 0; ++host_end) {
                if (held(host_end)) ++delivered; else ++lost;
            }
            for (k = 0; k < n; ++k, ++host_end) {
                if (held(host_end)) ++delivered; else ++lost;
            }
            last_first = next;
            (void)bid;
            next += n;
        }
        // the host acknowledges, except during a stall
        stalled = false;
        for (i = 0; i < stalls; ++i) if (t >= stall_at[i] && t < stall_at[i] + stall_len) stalled = true;
        for (i = 0; i < stalls; ++i) if (t == stall_at[i] + stall_len) { last_stall_end = t; delivered_at_last_stall = delivered; }
        if (!stalled && (t % ack_every) == 0U && host_end != start) host_ack_bid = host_end / n;
        // the firmware
        act = gvsp_gate_step(&g, n, host_ack_bid, true, next, f, &io);
        if (act & GVSP_GATE_SHUT) { pend_on = 0; pend_at = f + lat; ++shuts; }
        if (act & GVSP_GATE_OPEN) { pend_on = 1; pend_at = f + lat; }
        if (act & GVSP_GATE_FLUSH) {                     /* a resend of the whole cut block */
            uint32_t b0 = (uint32_t)(g.flush_bid - 1U) * n, k;
            if ((int32_t)(b0 - last_first) <= 0) return fail("flushed block already sent", b0, last_first);
            if (b0 != next) return fail("flushed block is not the one the FPGA waits at", b0, next);
            for (; (int32_t)(host_end - b0) < 0; ++host_end) {
                if (held(host_end)) ++delivered; else ++lost;
            }
            for (k = 0; k < n; ++k, ++host_end) {
                if (held(host_end)) ++delivered; else ++lost;
            }
            last_first = b0;
        }
        if (act & GVSP_GATE_RESTART) {
            if (g.resume % n) return fail("restart not at a block start", g.resume, n);
            if ((int32_t)(g.resume - next) < 0) return fail("restart before the stream's next frame", g.resume, next);
            next = g.resume;
        }
    }
    if (delivered + lost != host_end - start) return fail("frames neither delivered nor reported", delivered + lost, host_end - start);
    if (lost > g.overflow * n) return fail("lost frames not counted", lost, g.overflow * n);
    if (stalls == 0U && lost != 0U) return fail("loss without a stall", lost, 0);
    if (shuts > stalls) return fail("more shuts than stalls", shuts, stalls);
    if (start + FRAMES - host_end > 4U * GVSP_GATE_MARGIN + n) return fail("the stream did not run on", host_end, start + FRAMES);
    if (stalls != 0U && stall_len > SLOTS && lost == 0U) return fail("a stall past the ring lost nothing", stall_len, g.overflow);
    if (last_stall_end && delivered - delivered_at_last_stall < (FRAMES - last_stall_end) / 2U)
        return fail("little delivered after the last stall", delivered - delivered_at_last_stall, FRAMES - last_stall_end);
    printf("PASS delivered %u lost %u overflow blocks %u\n", (unsigned)delivered, (unsigned)lost, (unsigned)g.overflow);
    return 0;
}
