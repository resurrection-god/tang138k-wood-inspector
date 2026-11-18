/*
 * frame_udp.h - frame delivery over UDP straight from the FPGA (udp_tx,
 * docs/ARCHITECTURE.md § 8.1, spec § 4.3 and § 5, since 13.10.2026).
 *
 * The CPU puts a job descriptor into DDR3 (FRAME_UDP_DESC_ADDR): camera, ring
 * slots, PC and board addresses and ports, pause. The FPGA polls it every
 * 50 us, reads the frames from the ring itself, sends them as UDP packets
 * through the MAC and writes the result (FRAME_UDP_STAT_ADDR). The layouts have
 * one source of truth in docs/ARCHITECTURE.md § 8.1; their mirrors:
 * fpga/src/udp_rd.v, tools/common/memory_map.py.
 *
 * The FPGA takes the descriptor only when the seq at its start and at its end
 * match and differ from the previous job, and the magic is in place. So the
 * write has two phases: all fields and seq at the start, cache flush, seq at
 * the end, flush. While the end is old, the FPGA sees a torn descriptor and
 * waits.
 */

#ifndef APP_FRAME_UDP_H_
#define APP_FRAME_UDP_H_

#include <stdbool.h>
#include <stdint.h>

#define FRAME_UDP_DESC_ADDR     0x06000400U     /* 96 B, written by the CPU */
#define FRAME_UDP_DESC_BYTES    96U
#define FRAME_UDP_STAT_ADDR     0x06000480U     /* 64 B, written by the FPGA */
#define FRAME_UDP_STAT_BYTES    64U
#define FRAME_UDP_STAT_MAGIC    0x53504455U     /* 'UDPS' */
// GVSP job (GigE Vision, docs/protocol/GEV_PROTOCOL_DESIGN.md § 5.7, since 23.10.2026): the same
// descriptor with its own magic; the leader / trailer bodies in a template next to it.
#define FRAME_UDP_GVSP_MAGIC    0x44535647U     /* 'GVSD' */
#define FRAME_UDP_KIND_GVSP_IMAGE 1U
#define FRAME_UDP_GVSP_TPL_ADDR 0x06001000U     /* 52 B, written by the CPU */
#define FRAME_UDP_GVSP_TPL_BYTES 52U
// Profile blocks (GEV_PROTOCOL_DESIGN.md § 5.8, since 27.10.2026): per camera a stream record
// the FPGA reads between descriptor polls and sends blocks from by itself; the descriptor's
// kind 2 asks for a packet range of one block (resend). The record's last beat (+0x60) is
// the FPGA's status. Leader body at the template address, trailer body 256 B after it.
#define FRAME_UDP_KIND_GVSP_PROFILE 2U
#define FRAME_UDP_PCFG_ADDR(cam) (0x06001100U + 0x80U * (uint32_t)(cam))
#define FRAME_UDP_PCFG_MAGIC    0x43535647U     /* 'GVSC' */
#define FRAME_UDP_PSTAT_MAGIC   0x53505647U     /* 'GVPS' */
#define FRAME_UDP_PTPL_ADDR(cam) (0x06001200U + 0x200U * (uint32_t)(cam))
#define FRAME_UDP_PTPL_BYTES    256U
#define FRAME_UDP_PARTS_MAX     4U
#define FRAME_UDP_PART_C16      0U              /* part kinds as udp_rd.v: Range, x_sub        */
#define FRAME_UDP_PART_AC16     1U              /*   Range, mm (bit 15 of both halves flipped) */
#define FRAME_UDP_PART_MONO8    2U              /*   Reflectance, i                            */
#define FRAME_UDP_PART_CONF8    3U              /*   Confidence, q                             */
#define FRAME_UDP_PART_CHUNK    4U              /*   chunk part: ProfileInfo, 96 B per line    */

// One frame as a GVSP block, or a packet range of it (resend). The packet layout is
// gvsp.c's: a payload packet carries lw data words of the stream "image rows, then 12 chunk
// words"; w0 is the first data word sent, (row0, word0) the same as a row and a word in it.
typedef struct {
    uint8_t  cam, resend;
    uint8_t  nochunk;                     /* image only, payload type 0x0001 (udp_rd.v g_nock) */
    uint8_t  dst_mac[6], src_mac[6];      /* in wire order */
    uint8_t  dst_ip[4], src_ip[4];        /* in wire order */
    uint16_t dst_port, src_port;
    uint32_t gap;                         /* 8 ns clocks after each packet */
    uint16_t lw;                          /* data words per full payload packet: (SCPS - 48) / 4 */
    uint32_t frame_addr, frame_seq;       /* the ring slot and the VID0 seq seen in it */
    uint16_t pid0, pid1;                  /* packets to send, 0 = leader, ndata + 1 = trailer */
    uint32_t w0, t;                       /* first data word sent; data words of the block */
    uint64_t block_id;
    uint16_t row0, word0;
    uint32_t crc_ovr, rows_bad_ovr;       /* chunk values for a range starting after word 0 */
    uint16_t ndata;
    uint8_t  tpl[FRAME_UDP_GVSP_TPL_BYTES];   /* leader body 36 B + trailer body 16 B, wire order */
} frame_udp_gvsp_job_t;

// A camera's profile stream: what the FPGA needs to build and address its blocks.
typedef struct {
    bool     on;
    uint8_t  nlog;                        /* lines per block = 1 << nlog, 0..4 */
    uint8_t  dst_mac[6], src_mac[6];      /* in wire order */
    uint8_t  dst_ip[4], src_ip[4];        /* in wire order */
    uint16_t dst_port, src_port;
    uint32_t gap;                         /* 8 ns clocks after each packet */
    uint16_t lw;                          /* data words per full payload packet: (SCPS - 56) / 4 */
    uint32_t ring;                        /* the camera's dense ring */
    uint32_t start;                       /* first frame, a multiple of the lines per block */
    uint8_t  nparts;
    uint8_t  kind[FRAME_UDP_PARTS_MAX];   /* FRAME_UDP_PART_* */
    uint16_t npk[FRAME_UDP_PARTS_MAX];    /* data packets of each part */
    uint32_t off[FRAME_UDP_PARTS_MAX];    /* each part's first byte in the block */
    uint16_t ndata;                       /* data packets of the block */
    uint32_t lenp;                        /* bytes of the parts before the chunk part */
    uint8_t  lb, tb;                      /* leader / trailer body words */
    uint8_t  lead[FRAME_UDP_PTPL_BYTES], trail[FRAME_UDP_PTPL_BYTES];   /* wire order */
} frame_udp_pstream_t;

// The FPGA's status beat of a profile stream.
typedef struct {
    bool     valid;
    uint32_t next_frame, blocks, lines_bad, skips, rec_errors, last_bid, seq;
} frame_udp_pstat_t;

typedef struct {
    // result word 0: this job
    uint16_t job_id, frames;
    uint32_t packets, rows_bad;
    uint16_t frames_bad, axi_err;         /* axi_err since the FPGA started */
    uint32_t code;                        /* 0 = done, 1 = descriptor rejected, 2 = GVSP: the slot
                                             no longer holds the frame (nothing sent) */
    uint32_t cycles;                      /* clk_x1 clocks, 10 ns; GVSP: CRC-32 of the image */
    // result word 1: counters since the FPGA started
    uint32_t n_jobs, n_pkts, n_sent;
    uint16_t n_stat_to, n_underrun;       /* udp_send: no MAC statistics; hole in a packet */
    uint16_t mux_stat_to, mux_abort;      /* eth_tx_mux */
    uint32_t mux_n_cpu;                   /* CPU packets through eth_tx_mux */
    uint16_t n_bad_ctrl;                  /* udp_send: control word out of place */
    uint16_t max_hold_us;                 /* longest wait of a CPU packet for the port, ~us */
    // firmware, not FPGA: how long we waited for the result and how many times seq was republished
    uint32_t wait_ms;
    uint32_t republish;
} frame_udp_status_t;

// At start-up, before the end of the FPGA's quiet window (10 s): zero the
// descriptor (after power-up DDR3 holds garbage, after a firmware restart the
// previous job) and continue the job numbers from what the FPGA has already seen.
void frame_udp_init(void);
// Control blocks on the APB (FEATURES bit 1) rather than in DDR3.
bool frame_udp_ctl_apb(void);

// Publish a GVSP job and wait for the result. 0 = done, -1 = no result within timeout_ms (FPGA
// without udp_tx, or hung), -2 = the FPGA rejected the descriptor or code 2 (st->code); st is
// filled on 0 and -2. Jobs share the descriptor: one waits (up to timeout_ms) for another to end.
int frame_udp_gvsp_run(const frame_udp_gvsp_job_t *g, uint16_t *job_id, frame_udp_status_t *st,
                       uint32_t timeout_ms);
// Publish camera cam's profile stream record (templates first, then the record in two phases
// like the descriptor); the FPGA takes it within ~100 us. Returns the record's seq.
uint32_t frame_udp_pstream_set(uint8_t cam, const frame_udp_pstream_t *p);
// The FPGA's status beat of camera cam's stream.
void frame_udp_pstream_status(uint8_t cam, frame_udp_pstat_t *s);
// Resend packets pid0..pid1 of profile block block_id of camera cam (addresses from its record).
// Codes as frame_udp_gvsp_run; -2 also for code 3 (no valid record, or a range past the trailer).
int frame_udp_prof_resend(uint8_t cam, uint64_t block_id, uint16_t pid0, uint16_t pid1, uint16_t *job_id,
                          frame_udp_status_t *st, uint32_t timeout_ms);

#endif /* APP_FRAME_UDP_H_ */
