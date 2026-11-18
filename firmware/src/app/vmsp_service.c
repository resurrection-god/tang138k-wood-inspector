/*
 * VCP-IP v0.2 per STP-PAK-026-NET-MVP with the project's deviations, the
 * board's only network protocol; normative is
 * docs/protocol/VMSP_PROTOCOL_SPECIFICATION.md (deviations: docs/DECISIONS.md
 * R-03). The text port 5001 from the STP was removed (R-01): all its working
 * functions and a minimum of debugging were moved here.
 *
 *   UDP 39150  discovery 0x0001/0x0002, IP assignment 0x0003/0x0004,
 *              only in STANDBY (since 02.10.2026), in WORK/SERVICE the port is silent
 *   TCP 39151  control:
 *                0x0110 mode           0x0112 camera configuration
 *                0x0120 exposure       0x0130 calibration (72 B in RAM)
 *                0x0132 / 0x0134 / 0x0136 "pixel -> mm" table: write, commit, info (R-38)
 *                0x0140 synchroniser channel   0x0142 synchroniser control
 *                0x0145 software trigger       0x0150 line finder configuration
 *                0x0160..0x016A settings in the store: save, load, state,
 *                reset, download the image to the PC, upload from the PC
 *                0x0300 status         0x0302 burst generator (debug)
 *   TCP 39152  data:
 *                0x0220 -> N frames in 0x0221 chunks, one row each
 *                0x0210 -> N 3D slices as 0x0211 messages
 *                0x0222 capture into memory
 *                0x0212 -> profile stream in 0x0213 batches until 0x0214 (stage 2)
 *                0x0306 memory dump (debug)
 *
 * One stream: laser line profiles over TCP (0x0212, stage 2, 01.10.2026: in the working mode the FPGA does not write the frame at all, and
 * a profile is 4 KB). Frames over UDP (0x0224 / 0x0226) were removed on
 * 03.11.2026: frames over UDP go only as GVSP (gvsp.c).
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cache.h"
#include "platform.h"
#include "board.h"
#include "memory_map.h"
#include "cam_trig.h"
#include "sync.h"
#include "video.h"
#include "laser.h"
#include "mm_table.h"
#include "work_mode.h"
#include "spi_flash.h"
#include "settings.h"
#include "camera.h"
#include "i2c_bus.h"
#include "vmsp_service.h"
#include "vmsp_protocol.h"
#include "frame_udp.h"
#include "net_config.h"
#include "ota.h"
#include "lcd_hud.h"
#include "arch/ethernetif.h"

#include "lwip/inet.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/stats.h"
#include "lwip/priv/sockets_priv.h"   /* lwip_socket_dbg_get_socket: profile stream measurement */


#define VCP_NODE_ID             1U
#define VCP_MAX_PAYLOAD         4096U
#define VCP_REPLY_MAX           300U
#define VCP_DUMP_MAX            (16U * 1024U * 1024U)
#define VCP_DUMP_CHUNK          8192U     /* 256 CCTL per piece is safe (ERRORS 5.2, former 30.2) */
// How long a command after which the camera may write the ring (0x0142, 0x0145,
// 0x0302) waits for the end of frame delivery before answering BUSY.
#define VCP_RING_GUARD_MS       200U

// Opcodes
#define VCP_OP_DISCOVERY_PROBE  0x0001
#define VCP_OP_DISCOVERY_INFO   0x0002
#define VCP_OP_ASSIGN_IP        0x0003
#define VCP_OP_ASSIGN_IP_ACK    0x0004
#define VCP_OP_ASSIGN_MAC       0x0005
#define VCP_OP_ASSIGN_MAC_ACK   0x0006
#define VCP_OP_WORK_MODE        0x0110
#define VCP_OP_WORK_MODE_ACK    0x0111
#define VCP_OP_CAMERA_CONFIG    0x0112
#define VCP_OP_EXPOSURE         0x0120
#define VCP_OP_EXPOSURE_ACK     0x0121
#define VCP_OP_GET_CAM_CONFIG       0x0122
#define VCP_OP_GET_CAM_CONFIG_ACK   0x0123
#define VCP_OP_SPATIAL_CALIB    0x0130
#define VCP_OP_SYNC_CHANNEL     0x0140
#define VCP_OP_SYNC_CONTROL     0x0142
#define VCP_OP_SW_TRIGGER       0x0145
#define VCP_OP_SW_TRIGGER_ACK   0x0146
// Project opcode: laser line finder configuration (the STP has no such number,
// no conflict). Spec § 4.2, design in docs/ARCHITECTURE.md § 6.
#define VCP_OP_LASER_CONFIG     0x0150
#define VCP_OP_LASER_CONFIG_ACK 0x0151
// Settings storage (project opcodes, spec § 4.2,
// docs/ARCHITECTURE.md § 7): the response is "request | 1".
#define VCP_OP_SETTINGS_SAVE    0x0160
#define VCP_OP_SETTINGS_LOAD    0x0162
#define VCP_OP_SETTINGS_INFO    0x0164
#define VCP_OP_SETTINGS_RESET   0x0166
#define VCP_OP_SETTINGS_EXPORT  0x0168
#define VCP_OP_SETTINGS_IMPORT  0x016A
// OTA firmware and bitstream update opcodes (0x0170..0x0179)
#define VCP_OP_OTA_START        0x0170
#define VCP_OP_OTA_START_ACK    0x0171
#define VCP_OP_OTA_DATA         0x0172
#define VCP_OP_OTA_DATA_ACK     0x0173
#define VCP_OP_OTA_COMMIT       0x0174
#define VCP_OP_OTA_COMMIT_ACK   0x0175
#define VCP_OP_OTA_REBOOT       0x0176
#define VCP_OP_OTA_REBOOT_ACK   0x0177
#define VCP_OP_OTA_STATUS       0x0178
#define VCP_OP_OTA_STATUS_ACK   0x0179
#define VCP_OP_OTA_SET_SLOT     0x017A
#define VCP_OP_OTA_SET_SLOT_ACK 0x017B
#define VCP_OP_OTA_VERSION      0x017C
#define VCP_OP_OTA_VERSION_ACK  0x017D
#define VCP_OP_FETCH_3D         0x0210
#define VCP_OP_SLICE_DATA       0x0211
// Profile stream (stage 2, 01.10.2026, project opcodes): start, batch, stop and
// result. See the "Profile stream" section below.
#define VCP_OP_PSTREAM          0x0212
#define VCP_OP_PBATCH           0x0213
#define VCP_OP_PSTOP            0x0214
#define VCP_OP_PSTOP_ACK        0x0215
#define VCP_OP_CAPTURE_IMAGE    0x0220
#define VCP_OP_IMAGE_DATA       0x0221
#define VCP_OP_ACQUIRE          0x0222
#define VCP_CAM_STEREO          0xFFU    /* 0x0222: both cameras from one pulse */
#define VCP_OP_STATUS           0x0300
#define VCP_OP_TRIG_SERIES      0x0302
#define VCP_OP_MEM_DUMP         0x0306

// Status codes (spec § 3; the STP has only 0x00)
#define VCP_ST_OK               0x00
#define VCP_ST_BUSY             0x02
#define VCP_ST_BAD_VERSION      0x04
#define VCP_ST_BAD_OPCODE       0x05
#define VCP_ST_BAD_LENGTH       0x06
#define VCP_ST_BAD_CRC          0x07
#define VCP_ST_BAD_TARGET       0x08
#define VCP_ST_DENIED           0x09
#define VCP_ST_HW_ERROR         0x0A
#define VCP_ST_TIMEOUT          0x0B
#define VCP_ST_UNSUPPORTED      0x0C

#define VCP_MODE_STANDBY        0
#define VCP_MODE_WORK           1
#define VCP_MODE_SERVICE        2

// Flags in byte 3 of the 0x0112 body (formerly "u8 0").
#define VCP_CAMCFG_FREE_RUN     0x01U   /* in stream mode, start capture */

#if (VCP_MODE_STANDBY != WORK_MODE_STANDBY) || (VCP_MODE_WORK != WORK_MODE_WORK) || \
    (VCP_MODE_SERVICE != WORK_MODE_SERVICE)
#error "Коды режимов СТП и автомата в net_stack.h разошлись"
#endif

#define VCP_CHUNK_META_BYTES    34U
// Slot header word holding the hw_timestamp of the last strobe edge at the
// time the header was written (HDR_EXT_BASE + 5 in cam_channel.v).
#define VIDEO_HDR_W_STROBE_TS   21U
// It was 29 (words 0..28); words 29..32 were added together with the modes and
// the line finder, word 33 with the settings storage. The client reads by plen,
// the old fields are in their old places.
#define VCP_STATUS_WORDS        39U

// 0x0211 slice header: 28 B. Fields 0..17 per STP 4.6, timestamp_us and
// status/flags are deviations (docs/DECISIONS.md R-03).
#define VCP_SLICE_HDR_BYTES     28U

TaskHandle_t TASK_VMSP_CONTROL_Handler = NULL;
TaskHandle_t TASK_VMSP_VIDEO_Handler = NULL;

// The system mode (0x0110) lives in work_mode.c (formerly net_stack.c;
// work_mode_get/set): it controls not only frame delivery but also the sync
// block, the laser and the mailbox. Only packet parsing is here. VCP_MODE_* and
// WORK_MODE_* are the same STP numbers; the check below keeps them from
// drifting apart.

// ---- encoding ---------------------------------------------------------------

static uint16_t rd_be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t rd_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static void wr_be16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void wr_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
static void wr_be64(uint8_t *p, uint64_t v)
{
    wr_be32(p, (uint32_t)(v >> 32));
    wr_be32(p + 4, (uint32_t)v);
}

static int send_all(int fd, const uint8_t *p, size_t len)
{
    while (len > 0) {
        int n = lwip_send(fd, p, (int)len, 0);
        if (n <= 0) return -1;
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

// Exactly len bytes or an error: a partial header is never handed to the parser.
static int recv_exact(int fd, uint8_t *p, size_t len)
{
    while (len > 0) {
        int n = lwip_recv(fd, p, (int)len, 0);
        if (n <= 0) return -1;
        p += n;
        len -= (size_t)n;
    }
    return 0;
}

static void build_header(uint8_t *h, uint16_t opcode, uint32_t seq, uint32_t target,
                         uint32_t plen, uint32_t crc)
{
    memcpy(h, "VMSP", 4);
    wr_be16(h + 4, VMSP_PROTOCOL_VERSION);
    wr_be16(h + 6, opcode);
    wr_be32(h + 8, seq);
    wr_be32(h + 12, VCP_NODE_ID);
    wr_be32(h + 16, target);
    wr_be32(h + 20, plen);
    wr_be32(h + 24, crc);
}

// vmsp_call() (GVCP, gvcp.c) runs a control handler with this fd: reply() hands the
// answer body to the caller instead of a socket. One caller task only.
#define VCP_FD_LOCAL            (-2)
static struct {
    uint8_t *buf;
    uint32_t cap, len;
} s_local;

// Response: the request's seq, addressed to the request's sender, in one buffer.
static int reply(int fd, const vmsp_header_t *req, uint16_t opcode,
                 const uint8_t *payload, uint32_t plen)
{
    uint8_t buf[VMSP_HEADER_SIZE + VCP_REPLY_MAX];

    if (plen > VCP_REPLY_MAX) return -1;
    if (fd == VCP_FD_LOCAL) {
        (void)opcode;
        if (plen > s_local.cap) return -1;
        if (plen) memcpy(s_local.buf, payload, plen);
        s_local.len = plen;
        return 0;
    }
    build_header(buf, opcode, req->seq_num, req->sender_id, plen,
                 plen ? vmsp_crc32(payload, plen) : 0U);
    if (plen) memcpy(buf + VMSP_HEADER_SIZE, payload, plen);
    return send_all(fd, buf, VMSP_HEADER_SIZE + plen);
}

// Common 4 B StatusAck on the response code "request | 1".
static int reply_status(int fd, const vmsp_header_t *req, uint8_t status)
{
    uint8_t p[4] = {status, 0, 0, 0};
    return reply(fd, req, (uint16_t)(req->opcode | 1U), p, sizeof(p));
}

// StatusAck + int32 return code (for commands that have one).
static int reply_status_rc(int fd, const vmsp_header_t *req, uint8_t status, int32_t rc)
{
    uint8_t p[8] = {status, 0, 0, 0, 0, 0, 0, 0};
    wr_be32(p + 4, (uint32_t)rc);
    return reply(fd, req, (uint16_t)(req->opcode | 1U), p, sizeof(p));
}

// STP deviation (docs/DECISIONS.md R-03): a failed frame request is one 0x0221
// with status != 0 and total_chunks = 0, without pixels.
static int reply_capture_error(int fd, const vmsp_header_t *req, uint8_t status)
{
    uint8_t m[VCP_CHUNK_META_BYTES];

    memset(m, 0, sizeof(m));
    wr_be16(m + 16, VIDEO_FRAME_WIDTH);
    wr_be16(m + 18, VIDEO_FRAME_HEIGHT);
    m[20] = 1;                       /* Mono8 */
    m[21] = status;
    printf("[VCP] 0x0220 refused: status 0x%02X\r\n", (unsigned int)status);
    return reply(fd, req, VCP_OP_IMAGE_DATA, m, sizeof(m));
}

// ---- time ---------------------------------------------------------------------

// The PLMT counts at 100 MHz (CLAUDE.md invariant 1). The hw_timestamp timer in
// the FPGA runs on lane4_clk (clock LANE_TICK_PS, board.h): since 29.09.2026
// also 100 MHz, from 17 to 29.09 it was 400/3 MHz.
static uint64_t plmt_ticks(void)
{
    uint32_t hi, lo;
    do {
        hi = DEV_PLMT->MTIME[1];
        lo = DEV_PLMT->MTIME[0];
    } while (hi != DEV_PLMT->MTIME[1]);
    return ((uint64_t)hi << 32) | lo;
}

// STP deviation (docs/DECISIONS.md R-03): timestamp_us is the moment of the
// camera strobe, not of sending.
// The counter is 32 bits (42.9 s at a 10 ns clock); the wrap number comes from
// the PLMT converted to FPGA clocks (10 ns PLMT / LANE_TICK_PS): both counters
// start from the board reset.
//
// The wrap number is taken from a reference moment ref (PLMT): the strobe is
// assumed to lie within +-21.5 s of it. The reference "now" is good only for a
// fresh strobe (profile stream); a frame or slice from the ring takes the moment
// capture first saw the frame whole (video_frame_seen_time, 05.10.2026,
// docs/OPEN_ISSUES.md 1.4): before that, a frame taken more than 21.5 s after
// capture got a timestamp 42.9 s larger than the real one (the strobe variant
// closest to "now" is 2^32 clocks later).
static uint64_t strobe_to_us_at(uint32_t strobe_ticks, uint64_t ref_plmt)
{
    int64_t ref = (int64_t)(ref_plmt * 10000ULL / LANE_TICK_PS);
    int64_t d = ref - (int64_t)strobe_ticks + 0x80000000LL;
    uint64_t wraps = (d > 0) ? ((uint64_t)d >> 32) : 0U;
    return LANE_TICKS_TO_US((wraps << 32) + strobe_ticks);
}

static uint64_t strobe_to_us(uint32_t strobe_ticks)
{
    return strobe_to_us_at(strobe_ticks, plmt_ticks());
}

// Slot frame strobe: the reference is the frame detection by capture, otherwise "now".
static uint64_t frame_strobe_us(uint32_t slot, uint32_t frame_id, uint32_t strobe_ticks)
{
    uint64_t ref;

    if (!video_frame_seen_time(slot, frame_id, &ref)) ref = plmt_ticks();
    return strobe_to_us_at(strobe_ticks, ref);
}

// ---- frame in chunks ----------------------------------------------------------

// One frame: VIDEO_FRAME_HEIGHT (1280) 0x0221 messages, a chunk = a row, and the
// chunk status is the row trailer (STP deviations, docs/DECISIONS.md R-03).
//
// The header and the row go out in TWO sends, the row straight from DDR3
// without a copy. This is not carelessness but the best of three measured
// variants (ERRORS 3.4, former 31.49):
//
//   two sends per row (here)             0.30...0.95 s per frame
//   accumulator, pieces of 1 MSS         0.87...2.77 s
//   accumulator, pieces of 4 MSS         0.90...4.60 s
//   32 KB accumulator (17.09.2026)       slower than the baseline, former 31.44
//
// The temptation to gather chunks into a buffer is great: on the wire one sees
// that the 1142 B chunks do not fit the 1460 MSS and lwIP now and then emits a
// stubby segment (328, 390, 690, 752 B), and Windows acknowledges such a segment
// not at once but by delayed ACK. But the measurement showed that this is not
// the issue: **the number of blocking sends is the same in all variants, 63...87
// per frame**, i.e. exactly 1 382 400 / TCP_SND_BUF (17520) = 79. The
// application stalls once per send window fill, however many sends it spends on
// it, and larger pieces only lengthen each stall. The bottleneck is not the
// sending scheme.
//
// Call under video_ring_lock(), sensor stopped.
//
// VCP_FRAME_ZEROCOPY (05.10.2026, ERRORS 3.8): the row is not copied at all;
// lwip_vmsp_write3() queues the header with metadata (a static array per row)
// and the row itself straight from the ring by reference (PBUF_ROM), one segment
// per chunk: 302 -> 196 ms per frame on the PC (ERRORS 3.9). The win came not
// from dropping the row copy but from avoiding mem_malloc: the copy in tcp_write
// looked for room in the heap ~157 us per chunk. The references live in the
// stack until acknowledged, so the caller MUST wait for vcp_frames_acked()
// before video_ring_unlock(); on an error the connection is aborted
// (lwip_vmsp_abort), not closed, otherwise the stack would keep sending rows of
// an already released ring. (The copying path, switch 0, was removed 22.10.2026: dead since 05.10.)
//
// Diagnostics: ONE line per request, after acknowledgement: printf goes to the
// UART at 115200 with waiting (~87 us per character), and three lines per frame
// cost ~20 ms of ~183 (10 %) right between frames (05.10.2026).
#define VCP_TX_TIMEOUT_MS       10000U      /* like the data port's SO_SNDTIMEO */

static void cpu_share_start(void);
static void cpu_share_tail(void);

// Request totals for its one diagnostics line (PLMT ticks, 10 ns).
static struct {
    uint64_t t_send, t_ack;
    uint32_t n_slow;
} s_vtx;

// All rows given by reference were acknowledged by the host: the ring may be
// released. Otherwise the connection is aborted and -1 is returned.
static int vcp_frames_acked(int fd)
{
    uint64_t t0 = plmt_ticks();

    if (lwip_vmsp_wait_acked(fd, VCP_TX_TIMEOUT_MS) < 0) {
        printf("[VCP] frame data not acknowledged, connection aborted\r\n");
        lwip_vmsp_abort(fd);
        return -1;
    }
    s_vtx.t_ack = plmt_ticks() - t0;
    return 0;
}

static int send_frame_chunks(int fd, const vmsp_header_t *req, uint32_t slot,
                             uint32_t frame_id, uint32_t *bad_rows)
{
    // The VMSP header + metadata also go by reference, so each row has its own.
    // Overwriting row r of the next frame is safe: there are ~1.4 MB between them,
    // and unacknowledged data is at most TCP_SND_BUF (128 KB).
    static uint8_t s_hm[VIDEO_FRAME_HEIGHT][VMSP_HEADER_SIZE + VCP_CHUNK_META_BYTES];
    static const uint8_t crc_tail[4] = {0, 0, 0, 0};    /* frame CRC disabled */
    const uint32_t w = VIDEO_FRAME_WIDTH;
    const uint32_t h = (video_slot_height(slot) <= VIDEO_FRAME_HEIGHT)
                           ? video_slot_height(slot) : VIDEO_FRAME_HEIGHT;
    volatile uint32_t *hw = (volatile uint32_t *)slot;
    uint32_t bad = 0;
    uint64_t ts_us;
    uint64_t t_b, t_c;

    ae350_dcache_flush_all();
    ts_us = frame_strobe_us(slot, frame_id, hw[VIDEO_HDR_W_STROBE_TS]);

    for (uint32_t r = 0; r < h; ++r) {
        const uint8_t *row = (const uint8_t *)VIDEO_ROW_ADDR(slot, r);
        volatile uint32_t *tag =
            (volatile uint32_t *)(VIDEO_ROW_ADDR(slot, r) + VIDEO_ROW_TAG_OFFSET);
        bool last = (r == h - 1U);
        uint32_t plen = VCP_CHUNK_META_BYTES + w + (last ? 4U : 0U);
        uint8_t st = VCP_ST_OK;
        uint8_t *hm = s_hm[r];
        uint8_t *m = hm + VMSP_HEADER_SIZE;

        if (tag[0] != (((uint32_t)VIDEO_ROW_TAG_MAGIC << 16) | r) || tag[1] != frame_id) {
            st = VCP_ST_HW_ERROR;
            ++bad;
        }

        wr_be64(m + 0, frame_id);
        wr_be64(m + 8, ts_us);
        wr_be16(m + 16, (uint16_t)w);
        wr_be16(m + 18, (uint16_t)h);
        m[20] = 1;                                   /* Mono8 */
        m[21] = st;
        wr_be32(m + 22, w * h);                      /* total bytes in the frame */
        wr_be32(m + 26, r * w);                      /* offset of this chunk in the frame */
        wr_be16(m + 30, (uint16_t)r);                /* chunk index 0..h-1 */
        wr_be16(m + 32, (uint16_t)h);                /* number of chunks in the frame */
        build_header(hm, VCP_OP_IMAGE_DATA, req->seq_num, req->sender_id, plen, 0);

        t_b = plmt_ticks();
        if (lwip_vmsp_write3(fd, hm, (u16_t)sizeof(s_hm[0]), row, (u16_t)w,
                             last ? crc_tail : NULL, (u16_t)(last ? 4U : 0U),
                             VCP_TX_TIMEOUT_MS) < 0) {
            printf("[VCP] frame %u: send failed at chunk %u, connection aborted\r\n",
                   (unsigned int)frame_id, (unsigned int)r);
            lwip_vmsp_abort(fd);
            return -1;
        }
        t_c = plmt_ticks();
        if (t_c - t_b > 20000ULL) ++s_vtx.n_slow;
        s_vtx.t_send += t_c - t_b;
    }
    *bad_rows = bad;
    return 0;
}

// The camera's frames from its ring: first what is in memory, and if there are
// fewer than N whole frames, a capture by trigger right here. Returns how many
// were gathered. 0x0220, 0x0222 and 0x0210 need the same order, and it used to
// be written separately in each of them. Call under video_ring_lock().
static uint32_t frames_from_ring(uint8_t cam, uint32_t want, bool laser_on,
                                 uint32_t *slots, uint32_t *ids)
{
    // Frames "already in memory" are usable only if taken with the same
    // illumination: returning a frame without laser to a request with laser would
    // silently lie to the client.
    uint32_t n = (video_frames_ready(cam) && (video_frames_had_laser(cam) == laser_on))
                     ? video_collect_frames(cam, want, slots, ids) : 0U;

    if (n < want && video_capture_frames(cam, want, laser_on) > 0)
        n = video_collect_frames(cam, want, slots, ids);
    return n;
}

// 0x0220: "give N frames". cam_id, laser_pulse_en, uint16 N (the STP reserved
// field, 0 -> 1). The frames are the N last whole ones from the ring, in
// ascending number order.
static int handle_capture(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    static uint32_t slots[VIDEO_RING_SLOTS];
    static uint32_t ids[VIDEO_RING_SLOTS];
    uint8_t cam = 0;
    uint32_t want = 1;
    uint32_t n, i;
    bool laser_on = false;
    int rc = 0;

    if (plen != 0 && plen != 4) return reply_capture_error(fd, req, VCP_ST_BAD_LENGTH);
    if (plen == 4) {
        cam = p[0];
        // laser_pulse_en per the STP (scenario 4.1): capture with illumination. Until
        // 21.09.2026 the byte was parsed and ignored, and the laser had to be turned on
        // with separate 0x0140/0x0142 and not forgotten to turn off.
        laser_on = (p[1] != 0U);
        want = rd_be16(p + 2);
    }
    if (want == 0) want = 1;
    if (want > VIDEO_RING_SLOTS) want = VIDEO_RING_SLOTS;
    if (cam >= VIDEO_CAM_COUNT) return reply_capture_error(fd, req, VCP_ST_UNSUPPORTED);
    if (work_mode_get() == VCP_MODE_STANDBY) return reply_capture_error(fd, req, VCP_ST_DENIED);

    if (!video_ring_lock(30000U)) return reply_capture_error(fd, req, VCP_ST_BUSY);

    n = frames_from_ring(cam, want, laser_on, slots, ids);
    if (n == 0) {
        video_ring_unlock();
        return reply_capture_error(fd, req, VCP_ST_TIMEOUT);
    }

    {
        uint64_t t0 = plmt_ticks();
        uint32_t bad_all = 0, sent = 0, c;

        cpu_share_start();
        memset(&lwip_vmsp_stats, 0, sizeof(lwip_vmsp_stats));
        memset(&s_vtx, 0, sizeof(s_vtx));
        for (i = 0; i < n; ++i) {
            uint32_t bad = 0;
            if (send_frame_chunks(fd, req, slots[i], ids[i], &bad) < 0) {
                rc = -1;
                break;
            }
            bad_all += bad;
            ++sent;
        }
        // The rows went out as references into the ring: it may be released only after
        // acknowledgement (on an error the connection is already aborted).
        if (rc == 0 && vcp_frames_acked(fd) < 0) rc = -1;
        c = lwip_vmsp_stats.n ? lwip_vmsp_stats.n : 1U;
        // One line: frames, time (total / per frame / in sending, waiting for
        // acknowledgement), bad trailers, per chunk tcp_write and tcp_output in 10 ns,
        // waits for room in 1 ms units, CPU shares.
        printf("[VCP] 0x0220 cam %u: %u/%u frames %u..%u, %u ms (%u/frame, send %u, ack %u),"
               " bad rows %u | chunk write %u out %u x10ns, slow %u, waits %u |",
               (unsigned int)cam, (unsigned int)sent, (unsigned int)n,
               (unsigned int)ids[0], (unsigned int)ids[n - 1U],
               (unsigned int)((plmt_ticks() - t0) / 100000ULL),
               (unsigned int)(sent ? (plmt_ticks() - t0) / 100000ULL / sent : 0U),
               (unsigned int)(s_vtx.t_send / 100000ULL),
               (unsigned int)(s_vtx.t_ack / 100000ULL),
               (unsigned int)bad_all,
               (unsigned int)(lwip_vmsp_stats.t_write / c),
               (unsigned int)(lwip_vmsp_stats.t_output / c),
               (unsigned int)s_vtx.n_slow, (unsigned int)lwip_vmsp_stats.waits);
        cpu_share_tail();
    }
    video_ring_unlock();
    return rc;
}

// 0x0222: capture into memory (the former TRIGGER of port 5001), N frames by
// trigger. The body is empty (1 frame) or uint16 N. On the data channel, so
// that capture does not hold the control channel. Response: StatusAck + uint32
// whole frames in memory.
static int handle_acquire(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    static uint32_t slots[VIDEO_RING_SLOTS];
    static uint32_t ids[VIDEO_RING_SLOTS];
    uint8_t out[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t want = 1;
    uint32_t n = 0;
    uint8_t  cam = 0;
    bool     laser_on = false;

    // The body grew from 2 to 4 B: `u16 N, u8 camera, u8 laser`. The old two bytes
    // are read as before: camera 0, laser off.
    //
    // Camera VCP_CAM_STEREO (0xFF, 30.09.2026) is stereo capture: each of the N
    // pulses captures both cameras (video_capture_stereo), and the response holds
    // the number of whole PAIRS (the smaller of the rings). The frames are fetched
    // afterwards with two 0x0220, one per camera; the host finds the pair by strobe
    // (timestamp_us), camera 1 later by the cam1 delay.
    if (plen != 0 && plen != 2 && plen != 4) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    if (work_mode_get() == VCP_MODE_STANDBY) return reply_status(fd, req, VCP_ST_DENIED);
    if (plen >= 2) want = rd_be16(p);
    if (plen == 4) { cam = p[2]; laser_on = (p[3] != 0U); }
    if (cam >= VIDEO_CAM_COUNT && cam != VCP_CAM_STEREO) return reply_status(fd, req, VCP_ST_UNSUPPORTED);
    if (want == 0) want = 1;
    if (want > VIDEO_RING_SLOTS - 1U) want = VIDEO_RING_SLOTS - 1U;
    if (!video_ring_lock(30000U)) return reply_status(fd, req, VCP_ST_BUSY);
    if (cam == VCP_CAM_STEREO)
    {
        if (video_capture_stereo(want, laser_on) > 0)
        {
            uint32_t n1;
            n  = video_collect_frames(0, VIDEO_RING_SLOTS, slots, ids);
            n1 = video_collect_frames(1, VIDEO_RING_SLOTS, slots, ids);
            if (n1 < n) n = n1;
        }
    }
    else if (video_capture_frames(cam, want, laser_on) > 0)
        n = video_collect_frames(cam, VIDEO_RING_SLOTS, slots, ids);
    video_ring_unlock();

    out[0] = (n > 0) ? VCP_ST_OK : VCP_ST_TIMEOUT;
    out[1] = cam;
    out[2] = laser_on ? 1U : 0U;
    wr_be32(out + 4, n);
    printf("[VCP] 0x0222 acquire: cam %u, laser %s, %u whole frames in memory\r\n",
           (unsigned int)cam, laser_on ? "ON" : "off", (unsigned int)n);
    return reply(fd, req, (uint16_t)(VCP_OP_ACQUIRE | 1U), out, sizeof(out));
}

extern struct netif gnetif;

// ---- 3D slices (0x0210 / 0x0211) -------------------------------------------------
//
// The norm is STP 4.6 with the project's deviations (docs/DECISIONS.md R-03):
//   * one 0x0211 message per slice (as a frame goes in 0x0221 chunks), not the
//     whole batch in one body: 10 slices in one body is 65 KB in one message;
//   * the slice header gets timestamp_us (otherwise a slice cannot be tied to a
//     frame) and a status byte (the STP has none, and a refusal must be
//     described somehow);
//   * encoder_coord carries the number of the received trigger: the product has
//     no encoder.
//
// Slice header, 28 B Big-Endian:
//   +0  u32 num_slices      total slices in the batch (0 = refusal)
//   +4  u32 slice_id        slice number = frame number
//   +8  u16 points_count
//   +10 u64 encoder_coord
//   +18 u64 timestamp_us    exposure strobe
//   +26 u8  status          VCP_ST_*
//   +27 u8  flags           [0] calibration present [1] block from the FPGA
//                           [2] axis [3] the frame was incomplete
// Then points_count points of 6 B: u16 X_sub (1/16 px), u16 Z (1/64 mm),
// u8 brightness, u8 quality.
#define VCP_SLICE_F_CALIB       0x01
#define VCP_SLICE_F_FPGA        0x02
#define VCP_SLICE_F_AXIS_COL    0x04
#define VCP_SLICE_F_PARTIAL     0x08

// A refusal is one 0x0211 with num_slices = 0, points_count = 0 and
// status != 0. The same shape as a 0x0220 refusal (docs/DECISIONS.md R-03): the
// host does not hang.
static int reply_slice_error(int fd, const vmsp_header_t *req, uint8_t status)
{
    uint8_t b[VCP_SLICE_HDR_BYTES];

    memset(b, 0, sizeof(b));
    b[26] = status;
    printf("[VCP] 0x0210 refused: status 0x%02X\r\n", (unsigned int)status);
    return reply(fd, req, VCP_OP_SLICE_DATA, b, sizeof(b));
}

// One slice from the slot's profile block. Call under video_ring_lock().
static int send_slice(int fd, const vmsp_header_t *req, uint32_t slot,
                      const laser_block_hdr_t *bh, uint32_t num_slices,
                      uint64_t encoder)
{
    // 7708 B static rather than on the stack: the data task has 4096 words, and
    // 7.7 KB of automatic memory would wreck it.
    static uint8_t body[VCP_SLICE_HDR_BYTES + LASER_POINTS_MAX * 6U];
    uint8_t hdr[VMSP_HEADER_SIZE];
    uint32_t n = bh->points;
    uint32_t blen, i;
    uint8_t flags = 0;

    if (n > LASER_POINTS_MAX) n = LASER_POINTS_MAX;
    if (laser_calib_valid())                    flags |= VCP_SLICE_F_CALIB;
    if (bh->kind_flags & LASER_KIND_F_FPGA)     flags |= VCP_SLICE_F_FPGA;
    if (bh->kind_flags & LASER_KIND_F_PARTIAL)  flags |= VCP_SLICE_F_PARTIAL;
    if (bh->axis == LASER_AXIS_COL)             flags |= VCP_SLICE_F_AXIS_COL;

    memset(body, 0, VCP_SLICE_HDR_BYTES);
    wr_be32(body + 0, num_slices);
    wr_be32(body + 4, bh->frame_id);
    wr_be16(body + 8, (uint16_t)n);
    wr_be64(body + 10, encoder);
    wr_be64(body + 18, frame_strobe_us(slot, bh->frame_id, bh->strobe_ts));
    body[26] = VCP_ST_OK;
    body[27] = flags;

    for (i = 0; i < n; ++i) {
        uint8_t *q = body + VCP_SLICE_HDR_BYTES + i * 6U;
        laser_point_t pt;
        laser_rec_t rec;

        if (bh->has_points) {
            // Block version 2: the FPGA computed the point (laser_prof.v), the centre of
            // gravity in 1/16 pixel and the flags; the CPU only repacks.
            (void)laser_block_point(slot, i, &pt);
        } else {
            memset(&rec, 0, sizeof(rec));
            (void)laser_block_rec(slot, i, &rec);
            laser_refine(&rec, i, &pt);           /* algorithm hook point */
        }
        wr_be16(q + 0, pt.x_sub);
        wr_be16(q + 2, pt.z);
        q[4] = pt.i;
        q[5] = pt.q;
    }

    blen = VCP_SLICE_HDR_BYTES + n * 6U;
    build_header(hdr, VCP_OP_SLICE_DATA, req->seq_num, req->sender_id,
                 blen, vmsp_crc32(body, blen));
    if (send_all(fd, hdr, sizeof(hdr)) < 0 || send_all(fd, body, blen) < 0) {
        printf("[VCP] 0x0211: send failed on slice %u\r\n", (unsigned int)bh->frame_id);
        return -1;
    }
    printf("[VCP] 0x0211: slice %u sent, %u points, flags 0x%02X\r\n",
           (unsigned int)bh->frame_id, (unsigned int)n, (unsigned int)flags);
    return 0;
}

// 0x0210: "give up to N slices, starting from number S". Body 0 or 8 B:
// u32 max_slices, u32 start_slice_seq. Like 0x0220, it captures by itself if
// there are not enough.
//
// The profile block is taken from the slot: since 30.09.2026 the FPGA writes it
// for both cameras, version 2 with ready points (centre of gravity in 1/16 pixel
// and flags, laser_prof.v), and send_slice() only repacks them. If there is no
// block, compute it in firmware (laser_profile_build), but that branch is a stub
// and honestly answers UNSUPPORTED.
static int handle_fetch_3d(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    static uint32_t slots[VIDEO_RING_SLOTS];
    static uint32_t ids[VIDEO_RING_SLOTS];
    static laser_block_hdr_t bh[VIDEO_RING_SLOTS];
    static uint32_t ready[VIDEO_RING_SLOTS];
    sync_telemetry_t st;
    uint32_t want = 1, start = 0;
    uint32_t n = 0, have_n = 0, i;
    uint8_t  fail = VCP_ST_TIMEOUT;
    uint8_t  cam = 0;
    int rc = 0;

    // The STP body is 8 B; 12 B is a project extension: plus a u32 camera number.
    if (plen != 0 && plen != 8 && plen != 12)
        return reply_slice_error(fd, req, VCP_ST_BAD_LENGTH);
    if (plen >= 8) {
        want  = rd_be32(p);
        start = rd_be32(p + 4);
    }
    if (plen == 12) cam = (uint8_t)rd_be32(p + 8);
    if (cam >= VIDEO_CAM_COUNT) return reply_slice_error(fd, req, VCP_ST_UNSUPPORTED);
    if (want == 0) want = 1;
    if (want > VIDEO_RING_SLOTS) want = VIDEO_RING_SLOTS;
    if (work_mode_get() == VCP_MODE_STANDBY) return reply_slice_error(fd, req, VCP_ST_DENIED);

    if (!video_ring_lock(30000U)) return reply_slice_error(fd, req, VCP_ST_BUSY);

    // A laser line profile without the laser makes no sense: capture for 0x0210 is
    // always with illumination.
    // A laser line profile without the laser makes no sense, so capture for 0x0210
    // always runs with illumination; there is no separate flag in the request on
    // purpose.
    n = frames_from_ring(cam, want, true, slots, ids);

    // Selection: a frame not older than start, with a ready profile block, either
    // written by the FPGA or computed right here.
    for (i = 0; i < n; ++i) {
        if (ids[i] < start) continue;
        if (!laser_block_valid(slots[i], ids[i], &bh[have_n])) {
            if (laser_profile_build(slots[i], ids[i]) != 0) {
                fail = VCP_ST_UNSUPPORTED;      /* no algorithm yet */
                continue;
            }
            if (!laser_block_valid(slots[i], ids[i], &bh[have_n])) {
                fail = VCP_ST_HW_ERROR;
                continue;
            }
        }
        ready[have_n++] = slots[i];
    }

    if (have_n == 0) {
        video_ring_unlock();
        return reply_slice_error(fd, req, (n == 0) ? VCP_ST_TIMEOUT : fail);
    }

    // Conveyor coordinate surrogate (STP deviation, docs/DECISIONS.md R-03): the
    // number of the received trigger.
    (void)sync_read_telemetry(&st);
    for (i = 0; i < have_n; ++i) {
        if (send_slice(fd, req, ready[i], &bh[i], have_n, st.accepted) < 0) {
            rc = -1;
            break;
        }
    }
    video_ring_unlock();
    return rc;
}

// ---- Profile stream (0x0212 .. 0x0215, stage 2, 01.10.2026) ---------------------
//
// Working mode: the FPGA writes only the profile block into the slot (a frame
// without rows, LASR_CTRL_NO_FRAME, cam_channel.v dry_frame), the firmware takes
// the points of every new block and sends them in batches until the host says
// "stop". The camera, ROI, generator and laser are set up by the host with the
// usual commands (0x0112, 0x0140, 0x0142, 0x0302); the stream only reads the
// ring.
//
// Request 0x0212, 8 B: u8 camera mask (1, 2, 3), u8 flags (VCP_PS_F_*),
// u16 profiles per batch (1..VCP_PS_BATCH_MAX, 0 -> 8), u32 how many profiles
// to deliver before finishing (0 = until stop).
//
// Every stream message is 0x0213 with the request's seq, body:
//   +0  u8  status          VCP_ST_*; a refusal is the only message, count = 0
//   +1  u8  cam
//   +2  u16 count           profiles in the batch; 0 = start (the first) or pulse
//   +4  u32 lost            profiles of this camera missed since the stream start
//   +8  u16 points          points per profile
//   +10 u16 profile_bytes   16 + 4 * points
//   +12 u32 info            for a batch, the stream's batch number; for start,
//                           the VCP_PS_A_* flags; for a pulse, profiles of the
//                           camera delivered
//   then count profiles:
//   +0  u32 frame_id        this camera's FPGA frame number
//   +4  u8  flags           the block's LASER_KIND_F_* (0x01 FPGA, 0x02 frame incomplete)
//   +5  u8  0
//   +6  u16 rows_used       frame rows that passed the finder
//   +8  u64 ts_us           exposure strobe, us (as in 0x0211 / 0x0221)
//   +16 points x u32        {q[31:24], i[23:16], x_sub[15:0]}, the FPGA word
//
// Stop: any message from the host (0x0214 by meaning); the board answers 0x0215
// and returns to normal requests; a dropped connection is a stop too. Body of
// 0x0215, 20 B: u8 status, 3 reserved, then delivered and missed for cameras 0
// and 1 (u32 x 4). The same 0x0215 comes by itself when the ordered number has
// been delivered.
//
// Body CRC is 0, as for frame chunks (vmsp.py then does not check it); flag
// VCP_PS_F_CRC enables it. TCP carries its own checksum anyway.
//
// Flag VCP_PS_F_MM (16.10.2026, R-38): every profile also carries its mm words
// (laser_mm.v), profile_bytes = 24 + 8 * points:
//   +0  u32 frame_id, +4 u8 flags, +5 u8 mm flags, +6 u16 rows_used, +8 u64 ts_us
//       (as above); mm flags: [0] the mm are valid for the current table and ROI,
//       [1] the FPGA marked the table bad, [2] the block's band is not the one of
//       the current ROI (its mm were replaced by "no mm", reason 2)
//   +16 u32 table id, +20 u32 band CRC - as the block's header says
//   +24 points x u32 {q, i, x_sub}, then points x u32 mm {depth[31:16], along[15:0]}
// With VCP_PS_F_MM_ONLY as well: no point words, profile_bytes = 24 + 4 * points - the
// bytes of a stream without mm (the 8 B form halved the rate on the office network,
// 16.10.2026: 2 x 900 Hz gave ~365 profiles/s per camera against ~745).
// A bitstream without the table loader refuses the request with UNSUPPORTED.
#define VCP_PS_BATCH_MAX        16U
#define VCP_PS_BATCH_DEFAULT    8U
#define VCP_PS_HDR_BYTES        16U
#define VCP_PS_PROF_HDR_BYTES   16U
#define VCP_PS_PROF_MM_HDR_BYTES 24U
#define VCP_PS_PROF_CRC_BYTES   8U     /* VCP_PS_A_PCRC: u32 CRC of the point words, u32 of the mm words */
#define VCP_PS_PROF_MAX_BYTES   (VCP_PS_PROF_MM_HDR_BYTES + VCP_PS_PROF_CRC_BYTES + LASER_POINTS_MAX * 2U * LASER_PT_BYTES)
// One message = one profile (count = 1), built in one buffer that stays in the
// L1D cache (32 KB) all the time (4380 B; 10.3 KB since the mm words, R-38). The former batch buffers of 82 KB per
// camera were bigger than the cache: copying the points from the ring wrote to
// a "cold" destination, and a write with line allocation read every destination
// line from DDR3 first: 500 us per profile against 241 into a hot one
// (measurement 'M', 01.10.2026, ERRORS 1.15). The protocol is unchanged:
// count <= the requested batch.
#define VCP_PS_MSG_BYTES        (VMSP_HEADER_SIZE + VCP_PS_HDR_BYTES + VCP_PS_PROF_MAX_BYTES)
#define VCP_PS_FLUSH_MS         10U    /* an incomplete batch goes out after this long */
#define VCP_PS_PULSE_MS         1000U  /* a pulse if there are no profiles */
#define VCP_PS_DRAIN_QUIET_MS   300U   /* after a stop: caught up when nothing new for this long */
#define VCP_PS_DRAIN_MAX_MS     60000U /* after a stop: the tail is sent for at most this long */
#define VCP_PS_RESCAN_MS        50U    /* the block is not in the expected slot: search all */
#define VCP_PS_RESCAN_DENSE_MS  500U   /* the dense ring before its first block */
#define VCP_PS_ACK_MS           1500U  /* LASR echo: telemetry every 0.5 s */

#define VCP_PS_F_KEEP_FRAME     0x01U  /* keep writing the frame (checking, debugging) */
#define VCP_PS_F_CRC            0x02U  /* batch body CRC */
#define VCP_PS_F_MM             0x04U  /* mm words too (R-38) */
#define VCP_PS_F_MM_ONLY        0x08U  /* with F_MM: mm words only, no point words */
#define VCP_PS_A_NO_FRAME       0x01U  /* the FPGA confirmed frames without rows */
#define VCP_PS_A_KEEP_FRAME     0x02U  /* the frame is written on request */
#define VCP_PS_A_MM             0x04U  /* profiles carry mm words */
#define VCP_PS_A_MM_ONLY        0x08U  /* ... and no point words */
#define VCP_PS_A_DENSE          0x10U  /* blocks in the dense ring: 2048 slots of 32 KB (LASER_DRY_*) */
#define VCP_PS_F_LE             0x10U  /* the host takes point and mm words LE, as they lie in DDR3 */
#define VCP_PS_A_LE             0x20U  /* ... and gets them so: sent by reference, no copy */
// FPGA CRC per profile (20.10.2026, GOAL): the profile header grows by
// VCP_PS_PROF_CRC_BYTES, the CRC-32 the FPGA computed over the point words and over
// the mm words (laser_prof.v, block header words 19, 20); profile flag
// LASER_KIND_F_CRC says they are real. The host checks them on what it received, so a
// fault in DDR3, the copy, lwIP, the wire, or a retransmission by reference of a slot
// the FPGA has meanwhile rewritten shows instead of passing with a good TCP checksum.
#define VCP_PS_F_PCRC           0x20U
#define VCP_PS_A_PCRC           0x40U
// Zero-copy (19.10.2026): with the dense ring and F_LE (and no body CRC) the point
// and mm words go to lwIP by reference straight from the ring (lwip_vmsp_write3,
// as frames do, ERRORS 3.9): no copy into s_ps_msg, no byte swap. A reference must
// not outlive its slot: the FPGA comes back to it after 8192 frames, >= 6.7 s even
// at the camera's free-run 1211 fps (2048 and 1.69 s until 20.10.2026). Data
// unacknowledged for VCP_PS_ZC_AGE_MS aborts the connection: a loud stop instead of a
// retransmission that would carry a newer frame's words under the old header with a
// valid TCP checksum. With the FPGA CRC (VCP_PS_A_PCRC) such a profile is dropped by
// the host anyway, so the guard only bounds the damage and waits out longer stalls:
// on 20.10.2026 a 1 s guard aborted a 10-minute 2 x 1000 Hz run at 596 s (ERRORS 1.26).
#define VCP_PS_ZC_AGE_MS        (s_ps_pcrc ? 4000U : 1000U)
#define VCP_PS_ZC_Q             256U   /* by-reference profiles in flight; TCP_SND_BUF holds ~30 */
#define VCP_PS_ZC_HDR_BYTES     (VMSP_HEADER_SIZE + VCP_PS_HDR_BYTES + VCP_PS_PROF_MM_HDR_BYTES + \
                                 VCP_PS_PROF_CRC_BYTES)
#define VCP_PS_MM_VALID         0x01U  /* mm flags of a profile */
#define VCP_PS_MM_BAD           0x02U
#define VCP_PS_MM_STALE         0x04U

typedef struct {
    uint32_t   next_slot;   // slot 0..s_ps_slots-1 where we expect the next block
    uint32_t   last_id;     // number of the last frame taken (0 = none yet)
    uint32_t   lost;        // number gaps + blocks rewritten during the read + frames the FPGA dropped
    uint32_t   dropped;     // ... of which frames the FPGA dropped (slot header word 6 [23:16])
    uint8_t    last_ab;     // that counter in the last taken slot's header
    bool       have_ab;
    uint32_t   sent;
    uint32_t   n;           // profiles in the buffer
    uint32_t   batch_no;
    uint16_t   points;      // points in the buffer's profiles
    TickType_t t_first;     // when the buffer's first profile arrived
    TickType_t t_found;     // when a block was last found
    TickType_t t_pulse;
    // zero-copy: the message header (per slot, lives as long as the slot) and the
    // two by-reference pieces after it (points / mm, or mm only)
    uint8_t     *zc_hdr;
    const void  *zc_b, *zc_c;
    uint16_t     zc_blen, zc_clen;
} pstream_cam_t;

static uint8_t       s_ps_msg[VCP_PS_MSG_BYTES] __attribute__((aligned(32)));
static pstream_cam_t s_ps[VIDEO_CAM_COUNT];
static bool          s_ps_mm;      /* the stream carries mm words (VCP_PS_F_MM) */
static bool          s_ps_mm_only; /* ... and no point words (VCP_PS_F_MM_ONLY) */
static bool          s_ps_pcrc;    /* profiles carry the FPGA CRCs (VCP_PS_A_PCRC) */
// The ring the blocks are in: the dense one (VCP_PS_A_DENSE, ~2 s at 1000 Hz) when
// the FPGA confirmed it, else the frame ring (32 slots, 32 ms at 1000 Hz).
static bool          s_ps_dense;
static uint32_t      s_ps_slots;
static bool          s_ps_zc;      /* by reference (VCP_PS_A_LE) */
static uint8_t       s_ps_zc_hdr[VIDEO_CAM_COUNT][LASER_DRY_SLOTS][VCP_PS_ZC_HDR_BYTES]
                         __attribute__((aligned(4)));
static uint32_t      s_ps_mm_none[LASER_POINTS_MAX];   /* "no mm" words for a stale band */
static struct { uint32_t seq_end, t_ms; } s_ps_zcq[VCP_PS_ZC_Q];
static uint32_t      s_ps_zcq_h, s_ps_zcq_n;

static uint32_t ps_blk(uint8_t cam, uint32_t i)
{
    return s_ps_dense ? LASER_DRY_BLOCK_ADDR(cam, i)
                      : LASER_BLOCK_ADDR(VIDEO_RING_BASE(cam) + i * VIDEO_SLOT_BYTES);
}

// Frames the FPGA dropped (20.10.2026, ERRORS 1.26): an abandoned frame gives its
// number to the next one, so the profile numbers stay contiguous and the drop is
// silent. The slot's VID0 header word 6 [23:16] is the writer's cumulative abandon
// count when the slot was opened; its growth between taken profiles is counted lost.
static uint8_t ps_abandons(uint32_t blk)
{
    uint32_t hdr = blk - (s_ps_dense ? LASER_DRY_BLOCK_OFF : LASER_BLOCK_OFF);

    ae350_dcache_flush_range(hdr, 32);
    return (uint8_t)(((volatile uint32_t *)hdr)[6] >> 16);
}

// Profile header: the fields, then (VCP_PS_A_PCRC) the two CRCs.
static uint32_t ps_prof_hdr_bytes(void)
{
    return (s_ps_mm ? VCP_PS_PROF_MM_HDR_BYTES : VCP_PS_PROF_HDR_BYTES) +
           (s_ps_pcrc ? VCP_PS_PROF_CRC_BYTES : 0U);
}

static uint32_t ps_prof_bytes(uint32_t points)
{
    return ps_prof_hdr_bytes() + points * ((s_ps_mm && !s_ps_mm_only) ? 2U : 1U) * LASER_PT_BYTES;
}
static uint8_t       s_ps_rx[VMSP_HEADER_SIZE];
static uint32_t      s_ps_rx_n;

// ---- Stream measurement (01.10.2026, ERRORS 1.15) ------------------------------
// Where the time goes and why the stream stalls: two cameras at 300 Hz lost
// profiles in whole ring revolutions (32), in one ~1.4 s pause at the stream
// start. Time is in PLMT clocks (10 ns). The TCP state comes from the socket's
// control block: the receiver window (snd_wnd == 0 means the PC cannot read in
// time), the window probe (persist_backoff), retransmission (nrtx). The result
// goes to the UART and the tail of 0x0215.
#define VCP_PS_SLOW_SEND_TICKS  5000000U  /* 50 ms */
#define VCP_PS_DIAG_EVENTS      8U
typedef struct {
    uint64_t t0, t_take, t_send;
    uint64_t t_read, t_still, t_swap;  /* parts of a take: points (flush + copy), slot check, byte swap */
    uint64_t flush0, copy0;            /* g_laser_pt_t_* at the stream start */
    uint32_t n_take, n_send, n_slow, max_send;
    uint32_t zero_wnd, persist, nrtx_ev;
    uint32_t tcpdrop0, linkdrop0, linkrecv0;
    uint8_t  z, p, r;                  /* previous state for counting transitions */
    uint32_t ev_n;
    struct { uint32_t t_ms, dur_ms, wnd0, wnd1, flags; } ev[VCP_PS_DIAG_EVENTS];
} ps_diag_t;
static ps_diag_t       s_psd;
// lwIP counters (tcp.drop, link.*) only with LWIP_STATS (lwipopts.h); without
// it the tail of 0x0215 holds zeros. The driver does not maintain link.*, it is
// always 0.
#if LWIP_STATS
#define PS_STAT(x)              (lwip_stats.x)
#else
#define PS_STAT(x)              0U
#endif
static struct tcp_pcb *s_ps_pcb;
// FreeRTOS task times at the stream start (configGENERATE_RUN_TIME_STATS): at
// the end, CPU shares per task, including idle (IDLE).
#define VCP_PS_MAX_TASKS        24U
static TaskStatus_t    s_ps_ts0[VCP_PS_MAX_TASKS], s_ps_ts1[VCP_PS_MAX_TASKS];
static UBaseType_t     s_ps_nts0;
static uint32_t        s_ps_rt0;

static void cpu_share_start(void)
{
    s_ps_nts0 = uxTaskGetSystemState(s_ps_ts0, VCP_PS_MAX_TASKS, &s_ps_rt0);
}

// CPU shares per task since cpu_share_start(), from 0.5 %; appends to the line
// the caller started and closes it.
static void cpu_share_tail(void)
{
    uint32_t rt1 = 0, total;
    UBaseType_t n1 = uxTaskGetSystemState(s_ps_ts1, VCP_PS_MAX_TASKS, &rt1), a, b;

    total = rt1 - s_ps_rt0;
    if (total == 0U) {
        printf("\r\n");
        return;
    }
    printf(" cpu over %u ms:", (unsigned int)((uint64_t)total * 128U / 100000U));
    for (a = 0; a < n1; ++a) {
        uint32_t d = s_ps_ts1[a].ulRunTimeCounter;
        for (b = 0; b < s_ps_nts0; ++b)
            if (s_ps_ts0[b].xHandle == s_ps_ts1[a].xHandle) {
                d -= s_ps_ts0[b].ulRunTimeCounter;
                break;
            }
        uint32_t pm = (uint32_t)((uint64_t)d * 1000U / total);   /* per mille; 32 bits overflowed in ~40 s */
        if (pm >= 5U)
            printf(" %s %u.%u%%", s_ps_ts1[a].pcTaskName,
                   (unsigned int)(pm / 10U), (unsigned int)(pm % 10U));
    }
    printf("\r\n");
}

// Repeated searches over all slots (the block is not in the expected one): what
// was in the slot.
#define VCP_PS_RSC_EVENTS       16U
static struct {
    uint32_t t_ms, last_id, magic, hfid, ffid, f63, best_id;
    uint16_t slot, best_si;
    uint8_t  cam;
} s_ps_rsc[VCP_PS_RSC_EVENTS];
static uint32_t s_ps_rsc_n;

static void ps_diag_sample(void)
{
    struct tcp_pcb *pcb = s_ps_pcb;
    uint8_t z, p, r;

    if (pcb == NULL) return;
    z = (pcb->snd_wnd == 0U);
    p = (pcb->persist_backoff > 0U);
    r = (pcb->nrtx > 0U);
    if (z && !s_psd.z) ++s_psd.zero_wnd;
    if (p && !s_psd.p) ++s_psd.persist;
    if (r && !s_psd.r) ++s_psd.nrtx_ev;
    s_psd.z = z; s_psd.p = p; s_psd.r = r;
}

static void ps_body_hdr(uint8_t *b, uint8_t st, uint8_t cam, uint16_t count,
                        uint32_t lost, uint16_t points, uint32_t info)
{
    b[0] = st;
    b[1] = cam;
    wr_be16(b + 2, count);
    wr_be32(b + 4, lost);
    wr_be16(b + 8, points);
    wr_be16(b + 10, (uint16_t)ps_prof_bytes(points));
    wr_be32(b + 12, info);
}

static int ps_send(int fd, const vmsp_header_t *req, uint8_t *buf, uint32_t blen, bool crc)
{
    uint8_t *body = buf + VMSP_HEADER_SIZE;
    uint32_t wnd0 = (s_ps_pcb != NULL) ? (uint32_t)s_ps_pcb->snd_wnd : 0U;
    uint64_t t;
    uint32_t d;
    int rc;

    build_header(buf, VCP_OP_PBATCH, req->seq_num, req->sender_id, blen,
                 crc ? vmsp_crc32(body, blen) : 0U);
    t = plmt_ticks();
    rc = send_all(fd, buf, VMSP_HEADER_SIZE + blen);
    d = (uint32_t)(plmt_ticks() - t);
    s_psd.t_send += d;
    ++s_psd.n_send;
    if (d > s_psd.max_send) s_psd.max_send = d;
    if (d >= VCP_PS_SLOW_SEND_TICKS) {
        ++s_psd.n_slow;
        if (s_psd.ev_n < VCP_PS_DIAG_EVENTS) {
            struct tcp_pcb *pcb = s_ps_pcb;
            uint32_t i = s_psd.ev_n++;
            s_psd.ev[i].t_ms   = (uint32_t)((t - s_psd.t0) / 100000ULL);
            s_psd.ev[i].dur_ms = d / 100000U;
            s_psd.ev[i].wnd0   = wnd0;
            s_psd.ev[i].wnd1   = (pcb != NULL) ? (uint32_t)pcb->snd_wnd : 0U;
            // [7:0] nrtx, [15:8] persist_backoff, [23:16] rto (500 ms ticks), after the send
            s_psd.ev[i].flags  = (pcb != NULL)
                ? ((uint32_t)pcb->nrtx | ((uint32_t)pcb->persist_backoff << 8) |
                   (((uint32_t)pcb->rto & 0xFFU) << 16))
                : 0U;
        }
    }
    ps_diag_sample();
    return rc;
}

// Forget the acknowledged by-reference sends; -1 if the oldest one still
// unacknowledged is older than VCP_PS_ZC_AGE_MS (its slot may be rewritten soon).
static int ps_zc_guard(void)
{
    struct tcp_pcb *pcb = s_ps_pcb;
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    while (s_ps_zcq_n != 0U && pcb != NULL &&
           (int32_t)(pcb->lastack - s_ps_zcq[s_ps_zcq_h].seq_end) >= 0) {
        s_ps_zcq_h = (s_ps_zcq_h + 1U) % VCP_PS_ZC_Q;
        --s_ps_zcq_n;
    }
    return (s_ps_zcq_n != 0U && now - s_ps_zcq[s_ps_zcq_h].t_ms >= VCP_PS_ZC_AGE_MS) ? -1 : 0;
}

// One profile by reference: header (s->zc_hdr), then s->zc_b and s->zc_c from the ring.
static int ps_send_zc(int fd, const vmsp_header_t *req, pstream_cam_t *s, uint32_t blen)
{
    uint16_t hlen = (uint16_t)(VMSP_HEADER_SIZE + VCP_PS_HDR_BYTES + ps_prof_hdr_bytes());
    uint64_t t = plmt_ticks();
    uint32_t d;
    int rc = -1;

    build_header(s->zc_hdr, VCP_OP_PBATCH, req->seq_num, req->sender_id, blen, 0U);
    for (;;) {
        if (ps_zc_guard() < 0) {
            printf("[VCP] 0x0212: by-reference data unacknowledged for %u ms, connection aborted\r\n",
                   (unsigned int)VCP_PS_ZC_AGE_MS);
            return -1;
        }
        if (s_ps_zcq_n < VCP_PS_ZC_Q) {
            rc = lwip_vmsp_write3(fd, s->zc_hdr, hlen, s->zc_b, s->zc_blen, s->zc_c, s->zc_clen, 20U);
            if (rc != -2) break;                /* -2: no room yet, check the age again */
        } else {
            vTaskDelay(1);
        }
    }
    if (rc == 0 && s_ps_pcb != NULL) {
        uint32_t i = (s_ps_zcq_h + s_ps_zcq_n++) % VCP_PS_ZC_Q;
        s_ps_zcq[i].seq_end = s_ps_pcb->snd_lbb;
        s_ps_zcq[i].t_ms    = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    }
    d = (uint32_t)(plmt_ticks() - t);
    s_psd.t_send += d;
    ++s_psd.n_send;
    if (d > s_psd.max_send) s_psd.max_send = d;
    if (d >= VCP_PS_SLOW_SEND_TICKS) ++s_psd.n_slow;
    ps_diag_sample();
    return rc;
}

// A message without profiles: start, pulse or refusal.
static int ps_send_empty(int fd, const vmsp_header_t *req, uint8_t st, uint8_t cam,
                         uint32_t lost, uint32_t info)
{
    uint8_t m[VMSP_HEADER_SIZE + VCP_PS_HDR_BYTES];

    ps_body_hdr(m + VMSP_HEADER_SIZE, st, cam, 0, lost, 0, info);
    return ps_send(fd, req, m, VCP_PS_HDR_BYTES, true);
}

static int ps_flush(int fd, const vmsp_header_t *req, uint8_t cam, bool crc)
{
    pstream_cam_t *s = &s_ps[cam];
    uint32_t blen;
    int rc;

    if (s->n == 0U) return 0;
    blen = VCP_PS_HDR_BYTES + s->n * ps_prof_bytes(s->points);
    ps_body_hdr((s_ps_zc ? s->zc_hdr : s_ps_msg) + VMSP_HEADER_SIZE, VCP_ST_OK, cam, (uint16_t)s->n,
                s->lost, s->points, s->batch_no++);
    rc = s_ps_zc ? ps_send_zc(fd, req, s, blen) : ps_send(fd, req, s_ps_msg, blen, crc);
    s->sent += s->n;
    s->n = 0;
    s->t_pulse = xTaskGetTickCount();
    return rc;
}

// Take the slot's block into the buffer (the block was already checked by
// laser_block_peek()). false: the slot was rewritten during the read, the
// profile is counted as lost.
static bool ps_take(pstream_cam_t *s, uint8_t cam, uint32_t si, uint32_t blk,
                    const laser_block_hdr_t *bh)
{
    uint8_t  *p;
    uint32_t *w;
    uint32_t i;

    if (s->last_id != 0U && bh->frame_id > s->last_id + 1U)
        s->lost += bh->frame_id - s->last_id - 1U;
    {
        uint8_t ab = ps_abandons(blk), d = (uint8_t)(ab - s->last_ab);
        if (s->have_ab) {
            s->lost    += d;
            s->dropped += d;
        }
        s->last_ab = ab;
        s->have_ab = true;
    }
    s->last_id   = bh->frame_id;
    s->next_slot = (si + 1U) % s_ps_slots;
    s->t_found   = xTaskGetTickCount();

    if (s->n == 0U) {
        s->points  = bh->points;
        s->t_first = s->t_found;
    }
    // By reference the header lives in the slot's own buffer, as long as the slot.
    if (s_ps_zc) s->zc_hdr = s_ps_zc_hdr[cam][si];
    p = (s_ps_zc ? s->zc_hdr : s_ps_msg) + VMSP_HEADER_SIZE + VCP_PS_HDR_BYTES;   /* one profile per message */
    w = (uint32_t *)(p + ps_prof_hdr_bytes());
    {
        uint64_t t0 = plmt_ticks(), t1, t2;
        uint32_t nw = s->points;
        uint8_t  mmf = 0;
        bool     none = false;                              /* mm replaced by "no mm" */
        uint32_t *m = s_ps_mm_only ? w : w + s->points;     /* where the mm words go */
        // By reference only the cache is dropped: lwIP's checksum reads the ring.
        if (s_ps_zc) {
            if (!s_ps_mm_only) ae350_dcache_flush_range(blk + LASER_PTS_OFF, s->points * LASER_PT_BYTES);
        } else if (!s_ps_mm_only) laser_block_points_read(blk, s->points, w);
        if (s_ps_mm) {
            // mm words, and whether they belong to the table and the ROI of now: a
            // block whose band is not the current one gets "no mm" (R-38).
            uint32_t id = 0, crc = 0;
            bool cur = mm_expected(cam, &id, &crc);
            if (bh->has_mm && s_ps_zc) ae350_dcache_flush_range(blk + LASER_MM_OFF, s->points * 4U);
            else if (bh->has_mm) laser_block_mm_read(blk, s->points, m);
            if (bh->mm_bad) mmf |= VCP_PS_MM_BAD;
            if (bh->has_mm && bh->mm_on && cur && bh->mm_id == id && bh->mm_crc == crc) {
                mmf |= VCP_PS_MM_VALID;
            } else {
                if (bh->has_mm && bh->mm_on) mmf |= VCP_PS_MM_STALE;
                none = !bh->has_mm || bh->mm_on;
                if (none && !s_ps_zc) {
                    for (i = 0; i < s->points; ++i) m[i] = LASER_MM_NONE(2U);
                }
            }
            nw = s_ps_mm_only ? s->points : 2U * s->points;
            p[5] = mmf;
            wr_be32(p + 16, bh->mm_id);
            wr_be32(p + 20, bh->mm_crc);
        }
        t1 = plmt_ticks();
        // Ring overtake: while the points were being read, the FPGA may have started
        // this slot anew.
        if (!laser_block_still(blk, bh->frame_id)) {
            ++s->lost;
            return false;
        }
        t2 = plmt_ticks();
        if (s_ps_zc) {
            const void *mm = none ? (const void *)s_ps_mm_none : (const void *)(blk + LASER_MM_OFF);
            s->zc_b    = s_ps_mm_only ? mm : (const void *)(blk + LASER_PTS_OFF);
            s->zc_blen = (uint16_t)(s->points * 4U);
            s->zc_c    = (s_ps_mm && !s_ps_mm_only) ? mm : NULL;
            s->zc_clen = (uint16_t)(s->zc_c != NULL ? s->points * 4U : 0U);
        } else {
            for (i = 0; i < nw; ++i) w[i] = __builtin_bswap32(w[i]);   /* to BE */
        }
        s_psd.t_read  += t1 - t0;
        s_psd.t_still += t2 - t1;
        s_psd.t_swap  += plmt_ticks() - t2;
    }
    wr_be32(p + 0, bh->frame_id);
    p[4] = bh->kind_flags;
    if (!s_ps_mm) p[5] = 0;                 /* mm flags, set above */
    wr_be16(p + 6, bh->rows_used);
    wr_be64(p + 8, strobe_to_us(bh->strobe_ts));
    if (s_ps_pcrc) {
        uint8_t *q = p + (s_ps_mm ? VCP_PS_PROF_MM_HDR_BYTES : VCP_PS_PROF_HDR_BYTES);
        wr_be32(q + 0, bh->crc_pts);
        wr_be32(q + 4, bh->crc_mmw);
    }
    ++s->n;
    return true;
}

// New blocks of the camera: first the expected slot, and only if it has been
// empty for long, all slots (stream start, ring overtake): the earliest number
// newer than last_id is taken. 1 = a profile was taken, 2 = there was a block
// but it was rewritten during the read (lost), 0 = nothing new.
static int ps_poll(uint8_t cam)
{
    pstream_cam_t *s = &s_ps[cam];
    laser_block_hdr_t bh;
    uint32_t blk = ps_blk(cam, s->next_slot);
    uint32_t si, best_si = 0, best_id = 0;
    bool found = false;

    if (s->n != 0U) return 0;                       /* the message buffer is busy: send first */
    if (laser_block_peek(blk, &bh) && bh.frame_id > s->last_id) {
        uint64_t t;
        bool took;
        if (s->n != 0U && bh.points != s->points) return 0;   /* different geometry: after sending */
        t = plmt_ticks();
        took = ps_take(s, cam, s->next_slot, blk, &bh);
        s_psd.t_take += plmt_ticks() - t;
        ++s_psd.n_take;
        return took ? 1 : 2;
    }
    // While no block has been taken yet (stream start, generator not started), search
    // all slots on every poll: at 900 Hz the ring wraps in ~35 ms, and the
    // VCP_PS_RESCAN_MS pause would lose the first profiles unnoticed; before the
    // first number there is nothing to count gaps from. The dense ring holds ~8 s,
    // so there the pause loses nothing, and searching 8192 slots (~40 ms) often would
    // only burn the CPU while the generator is not started. Once a block has been
    // taken the dense ring needs no search at all: frame F is in slot F mod 8192, and
    // the expected slot holds the next frame or, after an overtake, a newer one.
    if (s_ps_dense && s->last_id != 0U) return 0;
    if ((xTaskGetTickCount() - s->t_found) <
        pdMS_TO_TICKS(s_ps_dense ? VCP_PS_RESCAN_DENSE_MS : VCP_PS_RESCAN_MS) &&
        (s->last_id != 0U || s_ps_dense)) return 0;
    s->t_found = xTaskGetTickCount();
    if (s->last_id != 0U && s_ps_rsc_n < VCP_PS_RSC_EVENTS) {
        // Why the expected slot is empty (measurement 01.10.2026): what lies in it.
        uint32_t b = blk;
        volatile uint32_t *h = (volatile uint32_t *)b;
        volatile uint32_t *f = (volatile uint32_t *)(b + LASER_FOOT_OFF);
        ae350_dcache_flush_range(b, 64);
        ae350_dcache_flush_range(b + LASER_FOOT_OFF, 256);
        s_ps_rsc[s_ps_rsc_n].t_ms    = (uint32_t)((plmt_ticks() - s_psd.t0) / 100000ULL);
        s_ps_rsc[s_ps_rsc_n].cam     = cam;
        s_ps_rsc[s_ps_rsc_n].slot    = (uint16_t)s->next_slot;
        s_ps_rsc[s_ps_rsc_n].last_id = s->last_id;
        s_ps_rsc[s_ps_rsc_n].magic   = h[LASER_H_MAGIC];
        s_ps_rsc[s_ps_rsc_n].hfid    = h[LASER_H_FRAME_ID];
        s_ps_rsc[s_ps_rsc_n].ffid    = f[0];
        s_ps_rsc[s_ps_rsc_n].f63     = f[63];
    }
    for (si = 0; si < s_ps_slots; ++si) {
        if (!laser_block_peek(ps_blk(cam, si), &bh) || bh.frame_id <= s->last_id) continue;
        if (!found || bh.frame_id < best_id) {
            best_id = bh.frame_id;
            best_si = si;
            found   = true;
        }
    }
    if (s->last_id != 0U && s_ps_rsc_n < VCP_PS_RSC_EVENTS) {
        s_ps_rsc[s_ps_rsc_n].best_si = found ? (uint16_t)best_si : 0xFFFFU;
        s_ps_rsc[s_ps_rsc_n].best_id = best_id;
        ++s_ps_rsc_n;
    }
    if (found) s->next_slot = best_si;
    return 0;
}

// A host message during the stream means stop. Non-blocking receive into an
// accumulator: the header may also arrive in parts. 1 = stop (header in *stop),
// 0 = nothing, -1 = the connection is closed or broken.
static int ps_check_stop(int fd, vmsp_header_t *stop)
{
    static uint8_t drain[VCP_MAX_PAYLOAD];
    int n = lwip_recv(fd, s_ps_rx + s_ps_rx_n, (int)(sizeof(s_ps_rx) - s_ps_rx_n), MSG_DONTWAIT);
    uint32_t plen;

    if (n == 0) return -1;
    if (n < 0) return (errno == EWOULDBLOCK || errno == EAGAIN) ? 0 : -1;
    s_ps_rx_n += (uint32_t)n;
    if (s_ps_rx_n < sizeof(s_ps_rx)) return 0;
    s_ps_rx_n = 0;
    if (memcmp(s_ps_rx, "VMSP", 4) != 0) return -1;
    stop->opcode    = rd_be16(s_ps_rx + 6);
    stop->seq_num   = rd_be32(s_ps_rx + 8);
    stop->sender_id = rd_be32(s_ps_rx + 12);
    plen            = rd_be32(s_ps_rx + 20);
    if (plen > VCP_MAX_PAYLOAD) return -1;
    if (plen && recv_exact(fd, drain, plen) < 0) return -1;
    return 1;
}

// Tail of 0x0215 with the measurement (01.10.2026), after the old 20 B, u32 BE:
//   +20 tail version (1), +24 ms taking profiles, +28 profiles taken,
//   +32 ms in sending, +36 sends, +40 sends >= 50 ms, +44 longest, ms,
//   +48 receiver window closures, +52 window probes (persist), +56 retransmission
//   episodes (nrtx), +60 tcp.drop (incoming segments dropped) over the stream,
//   +64 link.drop over the stream, +68 link.recv over the stream, +72 events n
//   (<= 8), +76 n x {t_ms, dur_ms, wnd0, wnd1, flags}: slow sends (flags: [7:0]
//   nrtx, [15:8] persist, [23:16] rto).
#define VCP_PS_SUMMARY_BYTES    (20U + 56U + VCP_PS_DIAG_EVENTS * 20U)

static int ps_send_summary(int fd, const vmsp_header_t *req, uint8_t st)
{
    uint8_t b[VCP_PS_SUMMARY_BYTES];
    uint8_t c;
    uint32_t i, n = s_psd.ev_n;

    memset(b, 0, sizeof(b));
    b[0] = st;
    for (c = 0; c < VIDEO_CAM_COUNT; ++c) {
        wr_be32(b + 4 + 8U * c, s_ps[c].sent);
        wr_be32(b + 8 + 8U * c, s_ps[c].lost);
    }
    wr_be32(b + 20, 1U);
    wr_be32(b + 24, (uint32_t)(s_psd.t_take / 100000ULL));
    wr_be32(b + 28, s_psd.n_take);
    wr_be32(b + 32, (uint32_t)(s_psd.t_send / 100000ULL));
    wr_be32(b + 36, s_psd.n_send);
    wr_be32(b + 40, s_psd.n_slow);
    wr_be32(b + 44, s_psd.max_send / 100000U);
    wr_be32(b + 48, s_psd.zero_wnd);
    wr_be32(b + 52, s_psd.persist);
    wr_be32(b + 56, s_psd.nrtx_ev);
    wr_be32(b + 60, PS_STAT(tcp.drop) - s_psd.tcpdrop0);
    wr_be32(b + 64, PS_STAT(link.drop) - s_psd.linkdrop0);
    wr_be32(b + 68, PS_STAT(link.recv) - s_psd.linkrecv0);
    wr_be32(b + 72, n);
    for (i = 0; i < n; ++i) {
        uint8_t *e = b + 76 + 20U * i;
        wr_be32(e + 0, s_psd.ev[i].t_ms);
        wr_be32(e + 4, s_psd.ev[i].dur_ms);
        wr_be32(e + 8, s_psd.ev[i].wnd0);
        wr_be32(e + 12, s_psd.ev[i].wnd1);
        wr_be32(e + 16, s_psd.ev[i].flags);
    }
    return reply(fd, req, VCP_OP_PSTOP_ACK, b, 76U + 20U * n);
}

static void ps_diag_print(void)
{
    uint32_t i;

    printf("[PSTREAM] take %u ms / %u prof (%u us each) | send %u ms / %u (slow %u, max %u ms)"
           " | zero_wnd %u persist %u nrtx %u | tcp.drop %u link.drop %u link.recv %u\r\n",
           (unsigned int)(s_psd.t_take / 100000ULL), (unsigned int)s_psd.n_take,
           (unsigned int)(s_psd.n_take ? (uint32_t)(s_psd.t_take / 100ULL / s_psd.n_take) : 0U),
           (unsigned int)(s_psd.t_send / 100000ULL), (unsigned int)s_psd.n_send,
           (unsigned int)s_psd.n_slow, (unsigned int)(s_psd.max_send / 100000U),
           (unsigned int)s_psd.zero_wnd, (unsigned int)s_psd.persist, (unsigned int)s_psd.nrtx_ev,
           (unsigned int)(PS_STAT(tcp.drop) - s_psd.tcpdrop0),
           (unsigned int)(PS_STAT(link.drop) - s_psd.linkdrop0),
           (unsigned int)(PS_STAT(link.recv) - s_psd.linkrecv0));
    if (s_psd.n_take)
        printf("[PSTREAM] take parts per profile, us: read %u (flush %u + copy %u), still %u, swap %u\r\n",
               (unsigned int)(s_psd.t_read / 100ULL / s_psd.n_take),
               (unsigned int)((g_laser_pt_t_flush - s_psd.flush0) / 100ULL / s_psd.n_take),
               (unsigned int)((g_laser_pt_t_copy - s_psd.copy0) / 100ULL / s_psd.n_take),
               (unsigned int)(s_psd.t_still / 100ULL / s_psd.n_take),
               (unsigned int)(s_psd.t_swap / 100ULL / s_psd.n_take));
    printf("[PSTREAM]");
    cpu_share_tail();
    for (i = 0; i < s_psd.ev_n; ++i)
        printf("[PSTREAM]   slow send at %u ms: %u ms, wnd %u -> %u, nrtx %u persist %u rto %u\r\n",
               (unsigned int)s_psd.ev[i].t_ms, (unsigned int)s_psd.ev[i].dur_ms,
               (unsigned int)s_psd.ev[i].wnd0, (unsigned int)s_psd.ev[i].wnd1,
               (unsigned int)(s_psd.ev[i].flags & 0xFFU),
               (unsigned int)((s_psd.ev[i].flags >> 8) & 0xFFU),
               (unsigned int)((s_psd.ev[i].flags >> 16) & 0xFFU));
    printf("[PSTREAM] rescans %u\r\n", (unsigned int)s_ps_rsc_n);
    for (i = 0; i < s_ps_rsc_n; ++i)
        printf("[PSTREAM]   rescan %u ms cam%u slot %u last %u: magic %08X hdr %u foot %u/%08X -> slot %u id %u\r\n",
               (unsigned int)s_ps_rsc[i].t_ms, (unsigned int)s_ps_rsc[i].cam,
               (unsigned int)s_ps_rsc[i].slot, (unsigned int)s_ps_rsc[i].last_id,
               (unsigned int)s_ps_rsc[i].magic, (unsigned int)s_ps_rsc[i].hfid,
               (unsigned int)s_ps_rsc[i].ffid, (unsigned int)s_ps_rsc[i].f63,
               (unsigned int)s_ps_rsc[i].best_si, (unsigned int)s_ps_rsc[i].best_id);
}

static int handle_profile_stream(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t  mask, flags, c;
    uint32_t batch, limit, total = 0;
    uint32_t ack_flags = 0;
    bool     crc;
    int      rc = 0, st_rc;
    vmsp_header_t stop;
    bool     draining = false;       /* stop received: sending the tail from the ring */
    TickType_t t_drain = 0, t_quiet = 0;

    if (plen != 8) return ps_send_empty(fd, req, VCP_ST_BAD_LENGTH, 0, 0, 0);
    mask  = p[0];
    flags = p[1];
    batch = rd_be16(p + 2);
    limit = rd_be32(p + 4);
    crc   = (flags & VCP_PS_F_CRC) != 0U;
    if (batch == 0U) batch = VCP_PS_BATCH_DEFAULT;
    if (batch > VCP_PS_BATCH_MAX) batch = VCP_PS_BATCH_MAX;
    if (mask == 0U || (mask & ~((1U << VIDEO_CAM_COUNT) - 1U)) != 0U)
        return ps_send_empty(fd, req, VCP_ST_UNSUPPORTED, 0, 0, 0);
    if (work_mode_get() == VCP_MODE_STANDBY)
        return ps_send_empty(fd, req, VCP_ST_DENIED, 0, 0, 0);
    s_ps_mm      = (flags & VCP_PS_F_MM) != 0U;
    s_ps_mm_only = s_ps_mm && (flags & VCP_PS_F_MM_ONLY) != 0U;
    s_ps_pcrc    = (flags & VCP_PS_F_PCRC) != 0U;
    if (s_ps_pcrc) ack_flags |= VCP_PS_A_PCRC;
    if (s_ps_mm) {
        // mm need the loader in the bitstream (telemetry word 38 'M'), for every camera asked.
        mm_echo_t e;
        for (c = 0; c < VIDEO_CAM_COUNT; ++c)
            if ((mask & (1U << c)) && !mm_read_echo(c, &e)) {
                s_ps_mm = s_ps_mm_only = false;
                return ps_send_empty(fd, req, VCP_ST_UNSUPPORTED, 0, 0, 0);
            }
        ack_flags |= VCP_PS_A_MM | (s_ps_mm_only ? VCP_PS_A_MM_ONLY : 0U);
    }
    if (!video_ring_lock(5000U)) return ps_send_empty(fd, req, VCP_ST_BUSY, 0, 0, 0);

    // The frames in the ring will stop being frames: 0x0220 after the stream captures anew.
    video_invalidate_frames();
    s_ps_dense = false;
    if (flags & VCP_PS_F_KEEP_FRAME) {
        ack_flags |= VCP_PS_A_KEEP_FRAME;
    } else {
        st_rc = laser_no_frame_set(mask, true, VCP_PS_ACK_MS);
        if (st_rc < 0) {
            video_ring_unlock();
            return ps_send_empty(fd, req, VCP_ST_UNSUPPORTED, 0, 0, 0);
        }
        // 0: the FPGA did not confirm (a bitstream before stage 2): the stream works,
        // but the frame is written, and the host sees that in the start flags.
        if (st_rc > 0) ack_flags |= VCP_PS_A_NO_FRAME;
        // 2: the blocks go to the dense ring (a bitstream since 19.10.2026).
        s_ps_dense = (st_rc == 2);
        if (s_ps_dense) ack_flags |= VCP_PS_A_DENSE;
    }
    s_ps_slots = s_ps_dense ? LASER_DRY_SLOTS : VIDEO_RING_SLOTS;
    // By reference only with the dense ring (a slot outlives VCP_PS_ZC_AGE_MS) and
    // without the body CRC (it would read the ring words once more).
    s_ps_zc = s_ps_dense && (flags & VCP_PS_F_LE) != 0U && !crc;
    if (s_ps_zc) {
        uint32_t i;
        ack_flags |= VCP_PS_A_LE;
        for (i = 0; i < LASER_POINTS_MAX; ++i) s_ps_mm_none[i] = LASER_MM_NONE(2U);
    }
    s_ps_zcq_h = s_ps_zcq_n = 0;
    memset(s_ps, 0, sizeof(s_ps));
    s_ps_rx_n = 0;
    memset(&s_psd, 0, sizeof(s_psd));
    s_ps_rsc_n = 0;
    {
        struct lwip_sock *sk = lwip_socket_dbg_get_socket(fd);
        s_ps_pcb = (sk != NULL && sk->conn != NULL) ? sk->conn->pcb.tcp : NULL;
    }
    s_psd.tcpdrop0   = PS_STAT(tcp.drop);
    s_psd.linkdrop0 = PS_STAT(link.drop);
    s_psd.linkrecv0 = PS_STAT(link.recv);
    s_psd.t0        = plmt_ticks();
    cpu_share_start();
    s_psd.flush0 = g_laser_pt_t_flush;
    s_psd.copy0  = g_laser_pt_t_copy;
    for (c = 0; c < VIDEO_CAM_COUNT; ++c) {
        uint32_t si;
        if (!(mask & (1U << c))) continue;
        for (si = 0; si < s_ps_slots; ++si) laser_block_erase(ps_blk(c, si));
        s_ps[c].t_found = 0;                 /* search all slots at once */
        s_ps[c].t_pulse = xTaskGetTickCount();
    }
    // The ring is held for the preparation only. It cannot be held for the whole
    // stream: 0x0142, 0x0145, 0x0302 and a mode change take it (VCP_RING_GUARD_MS)
    // and would answer BUSY; the host could neither start the generator nor stop
    // it. What the ring protects is frame delivery, and there is none during the
    // stream: the data port is single and sequential, and the firmware has no other
    // readers of the ring.
    video_ring_unlock();
    printf("[VCP] 0x0212: profile stream, cams 0x%X, batch %u, limit %u, flags 0x%02X -> ack 0x%02X\r\n",
           (unsigned int)mask, (unsigned int)batch, (unsigned int)limit,
           (unsigned int)flags, (unsigned int)ack_flags);
    if (ps_send_empty(fd, req, VCP_ST_OK, mask, 0, ack_flags) < 0) rc = -1;

    while (rc == 0) {
        bool progress = false;
        TickType_t now;

        for (c = 0; c < VIDEO_CAM_COUNT; ++c) {
            pstream_cam_t *s = &s_ps[c];
            int got;
            if (!(mask & (1U << c))) continue;
            // One profile per camera per pass, cameras in turn. It used to be "while the
            // camera has new ones": with slow sending it always has a new frame, and the
            // second camera stood for 80..340 ms, longer than its ring's revolution (71 ms
            // at 450 Hz), and lost 32 profiles at a time (ERRORS 1.15).
            if ((got = ps_poll(c)) != 0) {
                progress = true;
                if (got == 1) ++total;
                // The profile goes out at once: the message buffer is one for both cameras and
                // must stay in the cache (VCP_PS_MSG_BYTES).
                if (s->n != 0U && ps_flush(fd, req, c, crc) < 0) { rc = -1; break; }
            }
            if (limit != 0U && total >= limit) break;
            now = xTaskGetTickCount();
            if (s->n != 0U && (now - s->t_first) >= pdMS_TO_TICKS(VCP_PS_FLUSH_MS)) {
                if (ps_flush(fd, req, c, crc) < 0) { rc = -1; break; }
            } else if (s->n == 0U && (now - s->t_pulse) >= pdMS_TO_TICKS(VCP_PS_PULSE_MS)) {
                s->t_pulse = now;
                if (ps_send_empty(fd, req, VCP_ST_OK, c, s->lost, s->sent) < 0) { rc = -1; break; }
            }
        }
        if (rc < 0) break;

        if (limit != 0U && total >= limit) {
            for (c = 0; c < VIDEO_CAM_COUNT; ++c)
                if ((mask & (1U << c)) && ps_flush(fd, req, c, crc) < 0) rc = -1;
            if (rc == 0 && ps_send_summary(fd, req, VCP_ST_OK) < 0) rc = -1;
            break;
        }
        ps_diag_sample();
        if (s_ps_zc && ps_zc_guard() < 0) {
            printf("[VCP] 0x0212: by-reference data unacknowledged for %u ms, connection aborted\r\n",
                   (unsigned int)VCP_PS_ZC_AGE_MS);
            rc = -1;
            break;
        }
        if (!draining) {
            st_rc = ps_check_stop(fd, &stop);
            if (st_rc < 0) { rc = -1; break; }
            if (st_rc > 0) {
                // The stop does not drop what is still in the ring (22.10.2026): the host
                // has stopped the pulses first, so the profiles behind are a finite tail.
                // Send it all, then the summary; every profile of a scan is wanted.
                draining = true;
                t_drain = t_quiet = xTaskGetTickCount();
                printf("[VCP] 0x0212: stop - sending the tail from the ring\r\n");
            }
        }
        if (draining) {
            now = xTaskGetTickCount();
            if (progress) t_quiet = now;
            // Caught up: nothing new for VCP_PS_DRAIN_QUIET_MS. A generator left running
            // never lets it go quiet; VCP_PS_DRAIN_MAX_MS ends the stream anyway.
            if ((now - t_quiet) >= pdMS_TO_TICKS(VCP_PS_DRAIN_QUIET_MS) ||
                (now - t_drain) >= pdMS_TO_TICKS(VCP_PS_DRAIN_MAX_MS)) {
                for (c = 0; c < VIDEO_CAM_COUNT; ++c)
                    if ((mask & (1U << c)) && ps_flush(fd, req, c, crc) < 0) rc = -1;
                if (rc == 0 && ps_send_summary(fd, &stop, VCP_ST_OK) < 0) rc = -1;
                printf("[VCP] 0x0212: tail sent in %u ms%s\r\n", (unsigned int)((now - t_drain) * portTICK_PERIOD_MS),
                       (now - t_drain) >= pdMS_TO_TICKS(VCP_PS_DRAIN_MAX_MS) ? " (time limit: pulses still running?)" : "");
                break;
            }
        }
        if (!progress) vTaskDelay(1);
    }

    // References into the ring must not outlive the stream: the generator keeps
    // writing it. Everything acknowledged, or the connection is aborted, not closed
    // (a close would keep retransmitting from rewritten slots).
    if (s_ps_zc && (rc < 0 || lwip_vmsp_wait_acked(fd, VCP_PS_ZC_AGE_MS / 2U) < 0)) {
        lwip_vmsp_abort(fd);
        rc = -1;
    }
    ps_diag_print();
    s_ps_pcb = NULL;                  /* the socket will close, the control block will go */
    if (!(flags & VCP_PS_F_KEEP_FRAME))
        (void)laser_no_frame_set(mask, false, VCP_PS_ACK_MS);
    video_invalidate_frames();
    printf("[VCP] 0x0212: stream end (%s): cam0 sent %u lost %u (FPGA dropped %u), "
           "cam1 sent %u lost %u (FPGA dropped %u)\r\n",
           (rc < 0) ? "connection" : "stop",
           (unsigned int)s_ps[0].sent, (unsigned int)s_ps[0].lost, (unsigned int)s_ps[0].dropped,
           (unsigned int)s_ps[1].sent, (unsigned int)s_ps[1].lost, (unsigned int)s_ps[1].dropped);
    return rc;
}

// 0x0306: memory dump (the former DUMP_RAM, debug). Request: uint32 address,
// uint32 size (1..16 MB). Response: one 0x0307 message: status, 3 reserved,
// address, size, data. The body CRC is computed by a pass over memory before
// sending; if memory changed between the passes (the sensor writes), the CRC
// will not match, and that is the "torn dump" signal.
static int handle_mem_dump(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t hdr[VMSP_HEADER_SIZE + 12];
    uint8_t *pre = hdr + VMSP_HEADER_SIZE;
    uint32_t addr, size, off, crc;

    memset(pre, 0, 12);
    if (plen != 8) {
        pre[0] = VCP_ST_BAD_LENGTH;
        return reply(fd, req, (uint16_t)(VCP_OP_MEM_DUMP | 1U), pre, 12);
    }
    addr = rd_be32(p);
    size = rd_be32(p + 4);
    wr_be32(pre + 4, addr);
    if (size == 0 || size > VCP_DUMP_MAX) {
        pre[0] = VCP_ST_DENIED;
        return reply(fd, req, (uint16_t)(VCP_OP_MEM_DUMP | 1U), pre, 12);
    }
    wr_be32(pre + 8, size);

    // Cache drop is a flush (write back, then evict the line, CCTL_L1D_VA_WBINVAL),
    // not an invalidate (bare evict, CCTL_L1D_VA_INVAL). Invalidate is only fit
    // for memory written by the FPGA alone: on firmware working memory it throws
    // away changes not yet written to DDR3. That is how a 4 MB dump from
    // 0x00100000 corrupted an lwIP pbuf and crashed the board (assert in
    // do_writemore, then mcause=5 in lwip_standard_chksum, ERRORS 5.2, former
    // 31.37). For FPGA memory flush is no worse: a dirty line there exists only
    // if the CPU itself wrote it (CLAUDE.md invariant 4).
    crc = vmsp_crc32_update(VMSP_CRC32_INIT, pre, 12);
    for (off = 0; off < size; off += VCP_DUMP_CHUNK) {
        uint32_t chunk = (size - off > VCP_DUMP_CHUNK) ? VCP_DUMP_CHUNK : size - off;
        ae350_dcache_flush_range(addr + off, chunk);
        crc = vmsp_crc32_update(crc, (const uint8_t *)(addr + off), chunk);
    }
    build_header(hdr, (uint16_t)(VCP_OP_MEM_DUMP | 1U), req->seq_num, req->sender_id,
                 12U + size, vmsp_crc32_final(crc));
    if (send_all(fd, hdr, sizeof(hdr)) < 0) return -1;

    for (off = 0; off < size; off += VCP_DUMP_CHUNK) {
        uint32_t chunk = (size - off > VCP_DUMP_CHUNK) ? VCP_DUMP_CHUNK : size - off;
        ae350_dcache_flush_range(addr + off, chunk);
        if (send_all(fd, (const uint8_t *)(addr + off), chunk) < 0) {
            printf("[VCP] 0x0306: send failed at +%u\r\n", (unsigned int)off);
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    printf("[VCP] 0x0306: dumped %u bytes from 0x%08X\r\n", (unsigned int)size, (unsigned int)addr);
    return 0;
}

// ---- control -----------------------------------------------------------------

// 0x0110 per STP: mode uint16 Big-Endian (the "01 00" example in STP is a typo).
//
// Response extended from 2 to 4 B (spec § 4.2, docs/ARCHITECTURE.md § 7):
// status, actual mode, transition flags WORK_MODE_F_*, reserved. The first two
// bytes are as before; an old client reads them as it used to.
//
// The transition itself is work_mode_set() in work_mode.c (formerly
// net_stack.c): it turns the laser off, stops the pulses, writes the mode to
// the mailbox and checks the FPGA echo.
static int handle_work_mode(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[4] = {0, 0, 0, 0};
    uint8_t actual = work_mode_get(), flags = 0;
    uint16_t mode;
    int st;

    if (plen != 1 && plen != 2) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    mode = (plen == 2) ? rd_be16(p) : p[0];
    st = (mode > VCP_MODE_SERVICE) ? VCP_ST_UNSUPPORTED
                                   : work_mode_set((uint8_t)mode, &actual, &flags);
    out[0] = (uint8_t)st;
    out[1] = actual;
    out[2] = flags;
    return reply(fd, req, VCP_OP_WORK_MODE_ACK, out, sizeof(out));
}

// 0x0150: laser line finder setup (project opcode). Parameters are per camera
// and shared by WORK/SERVICE modes (since 24.09.2026, stage L2a,
// docs/ARCHITECTURE.md § 6).
//   14 B: u8 camera 0..1, u8 floor 0..threshold, then the 12 B below;
//   12 B (the old body): camera 0, floor stays as it was:
//   u8 axis (0 - point per row, 1 - point per column), u8 threshold 1..255,
//   u8 gap 1..15, u8 flags ([0] write frame, [1] write profile, [2] compute
//   profile on the CPU), u16 window from, u16 window to (0 - whole axis),
//   u16 min_sum, u16 width_max.
// Response 0x0151, 18 B: status, axis, threshold, gap, flags, CAMERA, window
// from/to, min_sum, width_max (as in the old 16 B, byte 5 is the camera),
// [14] FPGA has the finder, [15] its state: [0] mailbox read, [1] writing a
// profile block, [2] acknowledged this APPLY_SEQ; [16] floor, [17] 0.
static int handle_laser_config(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    laser_telemetry_t lt;
    laser_cfg_t cfg;
    uint8_t out[18];
    uint8_t cam = 0;
    int st, ack = 0;

    memset(out, 0, sizeof(out));
    if (plen != 12 && plen != 14) {
        out[0] = VCP_ST_BAD_LENGTH;
        return reply(fd, req, VCP_OP_LASER_CONFIG_ACK, out, sizeof(out));
    }
    if (plen == 14) {
        cam = p[0];
        if (cam >= LASER_CAM_COUNT) {
            out[0] = VCP_ST_DENIED;
            out[5] = cam;
            return reply(fd, req, VCP_OP_LASER_CONFIG_ACK, out, sizeof(out));
        }
    }
    laser_cfg_get(cam, &cfg);                 /* floor unchanged for 12 B */
    if (plen == 14) {
        cfg.offset = p[1];
        p += 2;
    }
    cfg.axis      = p[0];
    cfg.thresh    = p[1];
    cfg.gap       = p[2];
    cfg.flags     = p[3];
    cfg.win_lo    = rd_be16(p + 4);
    cfg.win_hi    = rd_be16(p + 6);
    cfg.min_sum   = rd_be16(p + 8);
    cfg.width_max = rd_be16(p + 10);

    st = laser_cfg_set(cam, &cfg);
    if (st == 0)
        ack = laser_fpga_ack((uint8_t)(1U << cam), LASER_ACK_TIMEOUT_MS);
    laser_cfg_get(cam, &cfg);
    out[0] = (uint8_t)st;
    out[1] = cfg.axis;
    out[2] = cfg.thresh;
    out[3] = cfg.gap;
    out[4] = cfg.flags;
    out[5] = cam;
    wr_be16(out + 6, cfg.win_lo);
    wr_be16(out + 8, cfg.win_hi);
    wr_be16(out + 10, cfg.min_sum);
    wr_be16(out + 12, cfg.width_max);
    if (laser_read_telemetry(cam, &lt)) {
        out[14] = 1U;
        out[15] = (uint8_t)((lt.loaded ? 1U : 0U) | (lt.enable ? 2U : 0U) |
                            ((ack > 0) ? 4U : 0U));
    }
    out[16] = cfg.offset;
    return reply(fd, req, VCP_OP_LASER_CONFIG_ACK, out, sizeof(out));
}

// 0x0130 per STP 3.4: 72 B of calibration (camera intrinsics, rotation
// matrix, translation vector, laser plane). Accepted into RAM; an explicit
// 0x0160 with section 1 writes it to the store (calibration section), and at
// startup it is read from there (since 23.09.2026; the SPI flash sector of
// the original plan was not needed, docs/DECISIONS.md R-14, R-22). Response:
// StatusAck + CRC-32 of the received body.
static int handle_spatial_calib(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[8];
    int st;

    memset(out, 0, sizeof(out));
    st = laser_calib_set(p, plen);
    out[0] = (uint8_t)st;
    if (st == 0) wr_be32(out + 4, vmsp_crc32(p, plen));
    return reply(fd, req, (uint16_t)(VCP_OP_SPATIAL_CALIB | 1U), out, sizeof(out));
}

// ---- "pixel -> mm" table (0x0132 .. 0x0137, 16.10.2026, R-38) -----------------
//
// The PC builds the table (tools/laser/mm_table.py) and uploads it in order:
//   0x0132 write, body: u8 cam, 3 B 0, u32 offset, then the table bytes (words
//          little-endian, as they lie in memory); offset 0 starts a new table and
//          switches the camera's mm off. Answer 0x0133: u8 status, u8 cam, 2 B 0,
//          u32 bytes received so far.
//   0x0134 commit, body: the 20 B header (mm_table.h mm_hdr_t, big-endian; byte
//          18 bit 0 - also save the table into SPI flash, R-39). The board checks
//          the geometry and the CRC of the table in DDR3, makes the band for the
//          camera's ROI and waits for the FPGA. Answer 0x0135, 20 B: u8 status,
//          u8 cam, u8 FPGA (1 confirmed, 0 not in time, 0xFF no loader in the
//          bitstream), u8 band off reason (MM_OFF_*), u32 table id, u32 band CRC,
//          u8 first node row, u8 node rows, i16 vofs, then u8 flash (0 not asked,
//          1 saved, 2 failed), u8 flash copy (0xFF none), i8 SPI_FLASH_* code, u8 0.
//          Saving needs flash_ctl.v in the bitstream (-5 without it, ERRORS 5.12).
//   0x0136 info, body: u8 cam. Answer 0x0137, 60 B: u8 status, u8 cam, u8 state
//          (MM_ST_*), u8 off reason, u32 received, u32 table id, u32 table CRC,
//          u8 unit shift, u8 frame, u8 first node row, u8 node rows, i16 vofs,
//          u16 kofs, u32 band CRC, u32 APPLY_SEQ, then the FPGA echo: u8 valid,
//          u8 seq, u8 flags [0] band valid [1] CRC mismatch [2] load error
//          [3] loading, u8 rows, u32 CRC loaded; then the flash copies: u8 mask
//          (whole copies, bit k), i8 copy in use (-1 none), 2 B 0, u32 seq[2],
//          u32 table id[2].
#define VCP_OP_MM_WRITE         0x0132
#define VCP_OP_MM_COMMIT        0x0134
#define VCP_OP_MM_INFO          0x0136
#define VCP_MM_FPGA_WAIT_MS     1500U   /* telemetry every 0.5 s + mailbox poll + load */

static int handle_mm_write(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[8];
    mm_info_t inf;
    int st;

    memset(out, 0, sizeof(out));
    if (plen < 8U) {
        st = VCP_ST_BAD_LENGTH;
    } else {
        out[1] = p[0];
        st = mm_write(p[0], rd_be32(p + 4), p + 8, plen - 8U);
    }
    out[0] = (uint8_t)st;
    if (plen >= 8U) {
        mm_info(p[0], &inf);
        wr_be32(out + 4, inf.received);
    }
    return reply(fd, req, (uint16_t)(VCP_OP_MM_WRITE | 1U), out, sizeof(out));
}

static int handle_mm_commit(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[20];
    uint8_t cam = 0, copy = 0;
    mm_info_t inf;
    int st, fpga = 0, frc = 0;
    bool save = (plen == MM_HDR_BYTES) && (p[18] & 0x01U);

    memset(out, 0, sizeof(out));
    st = mm_commit(p, plen, &cam);
    if (st == VCP_ST_OK) fpga = mm_wait_fpga(cam, VCP_MM_FPGA_WAIT_MS);
    // Into flash too (R-39): the table the board takes at start-up.
    if (st == VCP_ST_OK && save) {
        frc = mm_save(cam, &copy);
        out[16] = (frc == 0) ? 1U : 2U;
        out[17] = copy;
        out[18] = (uint8_t)(int8_t)frc;
    }
    mm_info(cam, &inf);
    out[0] = (uint8_t)st;
    out[1] = cam;
    out[2] = (fpga < 0) ? 0xFFU : (uint8_t)fpga;
    out[3] = inf.off_why;
    wr_be32(out + 4, inf.hdr.id);
    wr_be32(out + 8, inf.band_crc);
    out[12] = inf.r0;
    out[13] = inf.rows;
    wr_be16(out + 14, (uint16_t)inf.vofs);
    printf("[VCP] 0x0134: cam %u table %08X -> status 0x%02X, band %s, FPGA %d\r\n",
           (unsigned int)cam, (unsigned int)inf.hdr.id, (unsigned int)st,
           (inf.off_why == MM_OFF_NONE) ? "on" : "off", fpga);
    return reply(fd, req, (uint16_t)(VCP_OP_MM_COMMIT | 1U), out, sizeof(out));
}

static int handle_mm_info(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[60];
    mm_info_t inf;
    mm_flash_info_t fl;

    memset(out, 0, sizeof(out));
    if (plen != 1U || p[0] >= VIDEO_CAM_COUNT) {
        out[0] = (plen != 1U) ? VCP_ST_BAD_LENGTH : VCP_ST_BAD_TARGET;
        return reply(fd, req, (uint16_t)(VCP_OP_MM_INFO | 1U), out, sizeof(out));
    }
    mm_info(p[0], &inf);
    out[1] = p[0];
    out[2] = inf.state;
    out[3] = inf.off_why;
    wr_be32(out + 4, inf.received);
    wr_be32(out + 8, inf.hdr.id);
    wr_be32(out + 12, inf.hdr.crc);
    out[16] = inf.hdr.unit_shift;
    out[17] = inf.hdr.frame;
    out[18] = inf.r0;
    out[19] = inf.rows;
    wr_be16(out + 20, (uint16_t)inf.vofs);
    wr_be16(out + 22, inf.kofs);
    wr_be32(out + 24, inf.band_crc);
    wr_be32(out + 28, inf.seq);
    out[32] = inf.echo.valid ? 1U : 0U;
    out[33] = inf.echo.seq;
    out[34] = (uint8_t)((inf.echo.band_valid ? 1U : 0U) | (inf.echo.crc_fail ? 2U : 0U) |
                        (inf.echo.load_err ? 4U : 0U) | (inf.echo.loading ? 8U : 0U));
    out[35] = inf.echo.rows;
    wr_be32(out + 36, inf.echo.crc);
    mm_flash_info(p[0], &fl);
    out[40] = fl.mask;
    out[41] = (uint8_t)fl.used;
    wr_be32(out + 44, fl.seq[0]);
    wr_be32(out + 48, fl.seq[1]);
    wr_be32(out + 52, fl.id[0]);
    wr_be32(out + 56, fl.id[1]);
    return reply(fd, req, (uint16_t)(VCP_OP_MM_INFO | 1U), out, sizeof(out));
}

// 0x0112: camera setup (the former CFG and SET_CAM of port 5001). 12 or 20 B:
//   u8 cam, u8 mode (0 - stream, sensor runs after setup; 1 - hardware
//   trigger armed: 0x0404 = 1, 0x0408 = 1), u8 testpat, u8 0,
//   u32 exposure us, u16 gain 0.1 dB, u16 rate 0.01 Hz,
//   [u16 roi_w, u16 roi_h, u16 off_x, u16 off_y]  - optional tail (20 B),
//   [u32 rate 0.01 Hz] - one more optional tail (24 B).
//
// The second tail is needed because u16 hundredths of a hertz top out at
// 655.35 Hz, while the sensor runs faster on a narrow ROI: 518 frames/s
// measured at 256 rows, 818 at 128 (fpga/fps_probe, ERRORS former 31.54). A
// non-zero value in the tail overrides the u16 field; zero means "take the
// u16", so a 24-byte body can always be sent.
//
// Byte 3 used to be "u8 0", now it holds flags: bit 0 (VCP_CAMCFG_FREE_RUN)
// starts capture (0x0400 = 1) in stream mode. An old client sends zero there
// and gets the old behaviour.
//
// A 12-byte body means ROI VIDEO_FRAME_WIDTH x VIDEO_FRAME_HEIGHT
// (1080x1280) - that is how the opcode behaved before, and an old client
// keeps working.
//
// The 8-byte tail sets the sensor ROI explicitly. Frame rate is set ONLY by
// ROI height; width does not change it, though it is applied (ERRORS former
// 31.54). So, by the 23.09.2026 decision, height is cropped in the sensor,
// while sensor width stays 1080 and will later be cropped in the FPGA before
// the DDR3 write (docs/CONCEPT.md, docs/ARCHITECTURE.md § 3.3).
//
// ROI height reaches the FPGA as GEOM words (video_geom_set below): the
// bridge and the writer close the frame by it, readout serves 1080 x H.
// Height is one for both cameras and takes effect through a D-PHY watchdog
// reset (ERRORS 2.11, 2.14). Line width in the bridge is a constant 1080: the
// bridge rejects a ROI of other width line by line, so it is good only for
// rate measurement (fps_probe), not for serving frames.
//
// Response: StatusAck + int32 rc.
static bool settings_too_slow(uint8_t cam, uint32_t rows, uint32_t exp_us);   /* below, with 0x0302 */

static int handle_camera_config(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t cam, mode, testpat, cfg_flags;
    uint32_t exp_us;
    uint16_t gain;
    uint32_t fps;
    uint32_t roi_w = VIDEO_FRAME_WIDTH, roi_h = VIDEO_FRAME_HEIGHT;
    uint32_t off_x = 0, off_y = 0;
    int rc = -100;

    if (plen != 12 && plen != 20 && plen != 24)
        return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    cam = p[0];
    mode = p[1];
    testpat = p[2];
    cfg_flags = p[3];           /* bit 0 - free-running stream, see below */
    exp_us = rd_be32(p + 4);
    gain = rd_be16(p + 8);
    fps = rd_be16(p + 10);
    if (plen >= 20)
    {
        roi_w = rd_be16(p + 12);
        roi_h = rd_be16(p + 14);
        off_x = rd_be16(p + 16);
        off_y = rd_be16(p + 18);
        // Zero means "the default as before": this lets the client set only the
        // width without computing the other three fields.
        if (roi_w == 0) roi_w = VIDEO_FRAME_WIDTH;
        if (roi_h == 0) roi_h = VIDEO_FRAME_HEIGHT;
        if (roi_w > VIDEO_FRAME_WIDTH || roi_h > VIDEO_FRAME_HEIGHT)
            return reply_status_rc(fd, req, VCP_ST_DENIED, rc);
    }
    if (plen == 24)
    {
        uint32_t fps32 = rd_be32(p + 20);
        if (fps32 != 0U) fps = fps32;          /* lifts the 655.35 Hz ceiling */
    }
    if (cam > 1) return reply_status_rc(fd, req, VCP_ST_UNSUPPORTED, rc);
    if (mode > 2 || exp_us < 16U || exp_us > 1000000U || gain > 240U)
        return reply_status_rc(fd, req, VCP_ST_DENIED, rc);
    if (mode == 1 && settings_too_slow(cam, roi_h, exp_us))
        return reply_status_rc(fd, req, VCP_ST_DENIED, -220);   /* hardware trigger faster than its cycle */
    if (!i2c_bus_lock(1000))
        return reply_status_rc(fd, req, VCP_ST_BUSY, rc);
    // Mode 2 - software trigger: the sensor is armed (0x0404 = 1) with source
    // "over I2C" (0x0408 = 0) and gets one pulse by a write to 0x0414. Needed to
    // commission a camera that has no hardware trigger wire: it is the only way
    // to get a frame from it without soldering anything.
    rc = camera_configure(cam, (mode != 0) ? 1U : 0U, (mode == 1) ? 1U : 0U,
                          0, exp_us, gain, fps, (uint8_t)(testpat & 1U),
                          roi_w, roi_h, off_x, off_y);
    if (rc == 0 && mode == 2)
    {
        vTaskDelay(pdMS_TO_TICKS(VIDEO_MODE_SETTLE_MS));
        rc = camera_trigger_once(cam);
        printf("[VCP] camera %u: software trigger fired -> rc %d\r\n",
               (unsigned int)cam, rc);
    }
    // "Free-running stream" flag: start capture in stream mode too.
    //
    // camera_configure() deliberately leaves 0x0400 = 0 everywhere except the
    // trigger mode (camera.c, ERRORS former 31.47): the working path does not
    // need a free-running stream, it only overwrites the ring. But the maximum
    // frame rate for a given ROI can be measured ONLY in a free-running stream -
    // only there the camera runs at its maximum, unthrottled (ERRORS former
    // 31.54).
    //
    // Hence a flag rather than a mode: the default path is unchanged, and the
    // measurement gets what it needs without touching RTL or the bitstream.
    if (rc == 0 && mode == 0 && (cfg_flags & VCP_CAMCFG_FREE_RUN))
    {
        rc = camera_set_acquisition(cam, 1);
        printf("[VCP] camera %u: free-run acquisition started -> rc %d\r\n",
               (unsigned int)cam, rc);
    }
    video_invalidate_trigger_arming_cam(cam);   /* this sensor writes its ring again */
    i2c_bus_unlock();
    // The MIPI receiver must know the same geometry as the sensor: the bridge
    // checks every line length and rejects mismatched lines whole. The geometry
    // goes to the FPGA through the mailbox (GEOM words, ERRORS former 31.53).
    //
    // Since 24.09.2026 the bitstream understands GEOM, but only the height: line
    // length in the bridge is a constant 1080, width from the mailbox is not
    // applied (ERRORS former 31.53). The bridge and the writer apply the height
    // after the next D-PHY watchdog reset; firmware does not treat a frame as
    // whole until the height in the slot header equals the ROI, and re-arms the
    // camera (ERRORS 2.11, 2.14). Height is one for both cameras: the camera 1
    // channel has no mailbox Lane 5 and gets it by wire from the camera 0
    // channel, so GEOM is written only when camera 0 is set up.
    // Design - docs/ARCHITECTURE.md § 3.3; what is unfinished - docs/OPEN_ISSUES.md 2.6.
    //
    // GEOM follows the triggered camera (ERRORS 3.29): the camera 1 channel takes the
    // height from the camera 0 channel, so with camera 1 alone in trigger mode the
    // height must be camera 1's, or the writer waits for frames of the other height
    // and drops every one (the rig of a host that streams one camera: camera 0 stays
    // at the full frame). With both cameras, or camera 0 alone, nothing changes.
    if (rc == 0)
    {
        camera_params_t g0, g1;
        const camera_params_t *g = &g0;

        (void)camera_cached_params(0, &g0);
        if (camera_connected(1) && camera_cached_params(1, &g1) == 0 &&
            g1.trg_mode != 0U && g0.trg_mode == 0U)
            g = &g1;
        video_geom_set((uint16_t)g->roi_w, (uint16_t)g->roi_h);
    }
    printf("[VCP] camera %u config: mode %u exp %u gain %u fps %u -> rc %d\r\n",
           (unsigned int)cam, (unsigned int)mode, (unsigned int)exp_us, (unsigned int)gain, (unsigned int)fps, rc);
    return reply_status_rc(fd, req, (rc == 0) ? VCP_ST_OK : VCP_ST_HW_ERROR, rc);
}

// 0x0120 per STP: uint16 camera_id, uint32 exposure_us -> sensor over I2C.
// Project extension: one more uint16 gain in 0.1 dB (8 B body) -> 0x0C20.
// Response 8 B: status, camera_id, 0, 0, uint32 exposure read back.
//
// Camera mode is NOT touched: the camera applies 0x0C10/0x0C20 on the fly,
// and a camera armed for trigger stays armed. So exposure and gain before a
// shot are set with this command, not 0x0112: that one restarts the sensor,
// and after it the MIPI receiver does not always catch a frame even from 5
// pulses (ERRORS former 31.37 - 2 failures out of 5 on the frame_viewer
// button path).
static int handle_exposure(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[8];
    uint16_t cam;
    uint32_t exp_us, rb = 0, rb_gain = 0;
    uint32_t gain = 0;
    bool with_gain;
    int rc = -1, rc_gain = 0;

    memset(out, 0, sizeof(out));
    if (plen != 6 && plen != 8) {
        out[0] = VCP_ST_BAD_LENGTH;
        return reply(fd, req, VCP_OP_EXPOSURE_ACK, out, sizeof(out));
    }
    cam = rd_be16(p);
    exp_us = rd_be32(p + 2);
    with_gain = (plen == 8);
    if (with_gain) gain = rd_be16(p + 6);
    out[1] = (uint8_t)cam;
    if (cam > 1 && cam != 0xFFFF) {
        out[0] = VCP_ST_UNSUPPORTED;
    } else if (exp_us < 16U || exp_us > 1000000U || gain > 240U) {  /* 0x0C10, 0x0C20 VEYE */
        out[0] = VCP_ST_DENIED;
    } else if ((cam != 1 && camera_trigger_cycle_ns(0) && settings_too_slow(0, camera_cached_roi_h(0), exp_us)) ||
               (cam != 0 && camera_trigger_cycle_ns(1) && settings_too_slow(1, camera_cached_roi_h(1), exp_us))) {
        out[0] = VCP_ST_DENIED;              /* the running series would outpace the camera */
    } else if (!i2c_bus_lock(1000)) {
        out[0] = VCP_ST_BUSY;
    } else {
        uint8_t c_start = (cam == 0xFFFF) ? 0 : (uint8_t)cam;
        uint8_t c_end   = (cam == 0xFFFF) ? 1 : (uint8_t)cam;
        for (uint8_t c = c_start; c <= c_end; c++)
        {
            camera_select_channel(c);
            camera_write_reg(0x0C10, exp_us);
            // The camera MCU does not apply a write at once: without a pause the
            // read-back returns the old value (ERRORS former 31.34), and it loses
            // back-to-back transactions.
            vTaskDelay(pdMS_TO_TICKS(20));
            rc = camera_read_reg_status(0x0C10, &rb);
            if (with_gain)
            {
                vTaskDelay(pdMS_TO_TICKS(20));
                camera_write_reg(0x0C20, gain);
                vTaskDelay(pdMS_TO_TICKS(20));
                rc_gain = camera_read_reg_status(0x0C20, &rb_gain);
                if (rc_gain == 0 && rb_gain != gain) rc_gain = -1;
            }
        }
        // The cache is the source of "current settings" for saving to store.
        for (uint8_t c = c_start; c <= c_end; c++)
            camera_cache_set_exposure(c, exp_us, with_gain ? (int32_t)gain : -1);
        i2c_bus_unlock();
        out[0] = (rc == 0 && rb == exp_us && rc_gain == 0) ? VCP_ST_OK : VCP_ST_HW_ERROR;
        wr_be32(out + 4, rb);
    }
    return reply(fd, req, VCP_OP_EXPOSURE_ACK, out, sizeof(out));
}

// 0x0140: synchronizer channel (the former 0/1/10 of port 5001). 12 B:
//   u8 channel 0 cam0 / 1 cam1 / 2 laser, u8 laser mode (0 pulse, 1 always 0,
//   2 always 1, 3 pre-pulse), u16 0, u32 delay ns, u32 width ns.
// Tick is lane4_clk (LANE_TICK_PS, board.h; 10 ns), rounded down, width at
// least one tick. Applied at once (docs/ARCHITECTURE.md § 5).
static int handle_sync_channel(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    static const uint32_t w_delay[3] = {SYNC_W_CAM0_DELAY, SYNC_W_CAM1_DELAY, SYNC_W_LASER_DELAY};
    static const uint32_t w_width[3] = {SYNC_W_CAM0_WIDTH, SYNC_W_CAM1_WIDTH, SYNC_W_LASER_WIDTH};
    uint8_t ch;
    uint32_t delay_t, width_t;

    if (plen != 12) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    ch = p[0];
    if (ch > 2 || p[1] > 3) return reply_status(fd, req, VCP_ST_DENIED);
    delay_t = LANE_NS_TO_TICKS(rd_be32(p + 4));
    width_t = LANE_NS_TO_TICKS(rd_be32(p + 8));
    if (width_t == 0) width_t = 1;
    sync_set(w_delay[ch], delay_t);
    sync_set(w_width[ch], width_t);
    if (ch == 2) sync_set(SYNC_W_LASER_MODE, p[1]);
    sync_apply();
    return reply_status(fd, req, VCP_ST_OK);
}

// 0x0142: synchronizer control (the former SYNC EN/SRC/LASER). 12 B:
//   u8 enable, u8 source 0 CORE / 1 TIMER / 2 EXT, u8 laser_enable,
//   u8 flags ([0] EXT on falling edge), u32 EXT filter in ticks, u32 polarity
//   ([0] cam0 [1] cam1 [2] laser, 1 - active low).
static int handle_sync_control(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint32_t ctrl;

    if (plen != 12) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    if (p[1] > 2) return reply_status(fd, req, VCP_ST_DENIED);
    // A new source may immediately give pulses to an armed camera, and the camera
    // will write the ring: not allowed while frames are served (0x0220 holds the ring).
    if (!video_ring_lock(VCP_RING_GUARD_MS)) return reply_status(fd, req, VCP_ST_BUSY);
    ctrl = (p[0] ? SYNC_CTRL_ENABLE : 0U) |
           ((uint32_t)p[1] << SYNC_CTRL_SRC_SHIFT) |
           ((p[3] & 1U) ? SYNC_CTRL_EXT_FALL : 0U) |
           (p[2] ? SYNC_CTRL_LASER_EN : 0U);
    sync_set(SYNC_W_CTRL, ctrl);
    sync_set(SYNC_W_EXT_FILTER, rd_be32(p + 4));
    sync_set(SYNC_W_POLARITY, rd_be32(p + 8) & 7U);
    sync_apply();
    video_invalidate_frames();
    video_ring_unlock();
    return reply_status(fd, req, VCP_ST_OK);
}

// 0x0145 (as in spec v0.2): software trigger - the CORE source of the
// sync block. Body 0 or 4 B (ignored). Response 0x0146, 6 B:
// uint16 source (1 CORE), uint32 software trigger number.
static int handle_sw_trigger(int fd, const vmsp_header_t *req, uint32_t plen)
{
    uint8_t out[6];

    if (plen != 0 && plen != 4) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    // A pulse yields a frame from an armed camera - not while frames are served.
    if (!video_ring_lock(VCP_RING_GUARD_MS)) return reply_status(fd, req, VCP_ST_BUSY);
    sync_fire();
    video_invalidate_frames();
    video_ring_unlock();
    wr_be16(out, 1);
    wr_be32(out + 2, sync_get(SYNC_W_SW_SEQ));
    return reply(fd, req, VCP_OP_SW_TRIGGER_ACK, out, sizeof(out));
}

// 0x0300: status (the former STATUS, SYNC ?, TRIG STATUS). Response 0x0301 -
// VCP_STATUS_WORDS uint32 BE words, layout in spec § 4.2 and vmsp.py:
//   0 software bits (3 frames in memory, 4 capture running)
//   1 FPGA frame number  2 FPGA timer  3 lines/dropped  4 generator pulses
//   5 block state  6 received  7 overruns  8 EXT edges  9 trigger tick
//   10/11 APPLY_SEQ FPGA/firmware  12/13 SW_SEQ FPGA/firmware  14 CTRL
//   15/16 cam0 delay/width  17/18 cam1  19/20 laser  21 laser mode
//   22 EXT filter  23 polarity
//   24 0x0404 camera trigger mode  25 0x0408 source  26 camera triggers
//   27 lost by camera  28 "FPGA telemetry present" flag
//   29 system mode + camera 0 finder flags
//   30 camera 0 finder echo - word 36 of its telemetry as is: {'L',
//      APPLY_SEQ[7:0], [15] mailbox read, [14] writing block, gap[11:8], threshold}
//   31 camera 0: {profile blocks written 16, points without line in the last 16}
//   32 camera 1: same
//   33 settings store: [7:0] settings section, [15:8] calibration, flags
//      SETTINGS_F_* (0x02 - RAM differs from store)
//   34 Ethernet RX watchdog: [15:0] lost RX interrupts it completed in the last 20 s
//      (a sick boot, OPEN_ISSUES 1.1), [31:16] since boot; both saturate at 0xFFFF
//   35 [15:0] frames it drained, [31:16] MAC speed corrections
//   36, 37 camera 1: triggers seen, triggers it ignored (0x0418, as words 26, 27 of camera 0)
//   38 camera 1 FPGA frame number (as word 1 for camera 0)
// FPGA telemetry is refreshed every 0.5 s.
static int handle_status(int fd, const vmsp_header_t *req, uint32_t plen)
{
    uint8_t out[VCP_STATUS_WORDS * 4];
    uint32_t v[VCP_STATUS_WORDS];
    volatile uint32_t *d = (volatile uint32_t *)VIDEO_DIAG0_ADDR;
    laser_telemetry_t lt, lt1;
    laser_cfg_t lcfg;
    sync_telemetry_t t;
    uint32_t i;

    if (plen != 0) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    memset(v, 0, sizeof(v));
    (void)sync_read_telemetry(&t);          /* drops the cached telemetry block */
    v[0]  = video_status_bits();
    v[1]  = d[1];
    v[2]  = d[2];
    v[3]  = d[6];
    v[4]  = d[20];                          /* trig_pulses, HDR_EXT_BASE + 4 */
    v[5]  = t.status;
    v[6]  = t.accepted;
    v[7]  = t.overrun;
    v[8]  = t.ext_edges;
    v[9]  = t.last_ts;
    v[10] = t.apply_seq;
    v[11] = sync_get(SYNC_W_APPLY_SEQ);
    v[12] = t.sw_seq;
    v[13] = sync_get(SYNC_W_SW_SEQ);
    v[14] = sync_get(SYNC_W_CTRL);
    v[15] = sync_get(SYNC_W_CAM0_DELAY);
    v[16] = sync_get(SYNC_W_CAM0_WIDTH);
    v[17] = sync_get(SYNC_W_CAM1_DELAY);
    v[18] = sync_get(SYNC_W_CAM1_WIDTH);
    v[19] = sync_get(SYNC_W_LASER_DELAY);
    v[20] = sync_get(SYNC_W_LASER_WIDTH);
    v[21] = sync_get(SYNC_W_LASER_MODE);
    v[22] = sync_get(SYNC_W_EXT_FILTER);
    v[23] = sync_get(SYNC_W_POLARITY);
    v[28] = t.valid ? 1U : 0U;
    // Word 29: [7:0] system mode, [15:8] camera 0 finder setup flags,
    // [23:16] its axis, [24] calibration present, [25] FPGA has the finder (L2a).
    laser_cfg_get(0, &lcfg);
    v[29] = (uint32_t)work_mode_get() |
            ((uint32_t)lcfg.flags << 8) |
            ((uint32_t)lcfg.axis << 16) |
            (laser_calib_valid() ? (1U << 24) : 0U) |
            (laser_read_telemetry(0, &lt) ? (1U << 25) : 0U);
    (void)laser_read_telemetry(1, &lt1);
    v[30] = lt.valid ? d[LASR_T_ECHO] : 0U;
    v[31] = ((uint32_t)lt.blocks << 16) | lt.no_line;
    v[32] = ((uint32_t)lt1.blocks << 16) | lt1.no_line;
    v[33] = settings_status_word() | (spi_flash_alarm_get() << 16);   /* [18:16] stored-data fallback */
    // Camera 1's FPGA frame number, as word 1 for camera 0: pulses minus frames shows a
    // pulse a camera skipped (OPEN_ISSUES 2.8). FPGA-written memory: invalidate is safe.
    ae350_dcache_invalidate_range(VIDEO_DIAG1_ADDR + 4U, 4U);
    v[38] = ((volatile uint32_t *)VIDEO_DIAG1_ADDR)[1];
    {
        uint32_t recent, total, drain, speed;
        eth_rx_health(&recent, &total, &drain, &speed);
        v[34] = (recent > 0xFFFFU ? 0xFFFFU : recent) | ((total > 0xFFFFU ? 0xFFFFU : total) << 16);
        v[35] = (drain > 0xFFFFU ? 0xFFFFU : drain) | ((speed > 0xFFFFU ? 0xFFFFU : speed) << 16);
    }
    if (i2c_bus_lock(1000)) {
        uint32_t tot = 0, lost = 0;
        camera_select_channel(0);
        (void)camera_read_reg_status(0x0404, &v[24]);
        (void)camera_read_reg_status(0x0408, &v[25]);
        (void)camera_read_trigger_count(0, &tot, &lost);
        v[26] = tot;
        v[27] = lost;
        if (camera_read_trigger_count(1, &tot, &lost) == 0) {   /* camera 1: words 36, 37 */
            v[36] = tot;
            v[37] = lost;
        }
        i2c_bus_unlock();
    }
    for (i = 0; i < VCP_STATUS_WORDS; ++i) wr_be32(out + 4 * i, v[i]);
    return reply(fd, req, (uint16_t)(VCP_OP_STATUS | 1U), out, sizeof(out));
}

// 0x0302: burst generator (the former TRIG, debug; source TIMER). 12 B:
//   u32 rate 0.01 Hz, u16 pulse count (0 - continuous), u16 flags
//   ([0] invert [1] stop), u32 width us. Response: StatusAck + int32 rc.
// A camera ignores a trigger that comes before its cycle is over, and nothing downstream
// sees a pulse that never became a frame (OPEN_ISSUES 2.8). Refuse a rate (0.01 Hz) whose
// period is under any triggered camera's cycle + 2 %: rc -210 - camera.
static bool rate_beats_cycle(uint32_t rate, uint64_t cyc_ns, uint8_t cam)
{
    // period_ns = 1e11 / rate; too fast when period < 1.02 * cycle
    if (cyc_ns == 0U || rate == 0U || (uint64_t)rate * cyc_ns * 102ULL <= 10000000000000ULL) return false;
    printf("[TRIG] %u.%02u Hz refused: camera %u needs %u us per trigger\r\n",
           (unsigned)(rate / 100U), (unsigned)(rate % 100U), (unsigned)cam, (unsigned)(cyc_ns / 1000U));
    return true;
}

static int trig_too_fast(uint32_t rate)
{
    uint8_t c;

    for (c = 0; c < VIDEO_CAM_COUNT; ++c)
        if (rate_beats_cycle(rate, camera_trigger_cycle_ns(c), c)) return -210 - (int)c;
    return 0;
}

// The same check the other way round: new exposure / ROI rows of a triggered camera while a
// continuous series runs (a mutant at 1 kHz: camera 1 set to 400 us skipped 1647 pulses).
static bool settings_too_slow(uint8_t cam, uint32_t rows, uint32_t exp_us)
{
    uint32_t rate, count, width;

    cam_trig_get(&rate, &count, &width);
    if (!cam_trig_running() || count != 0U || !camera_connected(cam)) return false;   /* stopped or a short burst */
    return rate_beats_cycle(rate, camera_cycle_ns(rows, exp_us), cam);
}

static int handle_trig_series(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint16_t flags;
    int rc;

    if (plen != 12) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    flags = rd_be16(p + 6);
    if (!video_ring_lock(VCP_RING_GUARD_MS)) return reply_status_rc(fd, req, VCP_ST_BUSY, -100);
    if (flags & 2U)
        rc = cam_trig_stop();
    else if ((rc = trig_too_fast(rd_be32(p))) != 0)
        ;                                   /* refused: the camera would skip pulses silently */
    else
    {
        rc = cam_trig_apply(rd_be32(p), rd_be16(p + 4), rd_be32(p + 8),
                            (flags & 1U) ? CAM_TRIG_FLAG_INVERT : 0U);
        if (rc == 0) video_invalidate_trigger_arming();   /* the camera writes the ring */
    }
    video_ring_unlock();
    return reply_status_rc(fd, req, (rc == 0) ? VCP_ST_OK : VCP_ST_DENIED, rc);
}

// 0x0122: read camera configuration. 2 B: uint16 camera_id (0 or 1).
// Response 0x0123: 32 B (status, cam, connected, model_id, exp_us, gain, fps, trg_mode, trg_src, testpat, auto_exp, roi_w, roi_h, off_x, off_y, max_fps)
static int handle_get_camera_config(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[32];
    uint16_t cam;
    bool connected = false;
    uint32_t model_id = 0, sensor_w = 0, sensor_h = 0;
    uint8_t trg_mode = 0, trg_src = 0, auto_exp = 0;
    uint32_t exp_us = 0, gain = 0, fps = 0, max_fps = 0, err_code = 0;
    uint8_t testpat = 0;
    uint32_t roi_w = 0, roi_h = 0, off_x = 0, off_y = 0;
    int rc = -1;

    memset(out, 0, sizeof(out));
    if (plen < 2) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    cam = rd_be16(p);
    if (cam > 1) return reply_status(fd, req, VCP_ST_UNSUPPORTED);

    if (!i2c_bus_lock(1500))
        return reply_status(fd, req, VCP_ST_BUSY);

    rc = camera_read_config((uint8_t)cam, &connected, &model_id, &sensor_w, &sensor_h,
                            &trg_mode, &trg_src, &auto_exp, &exp_us, &gain, &fps,
                            &max_fps, &err_code, &testpat, &roi_w, &roi_h, &off_x, &off_y);
    i2c_bus_unlock();

    out[0] = (rc == 0 && connected) ? VCP_ST_OK : VCP_ST_HW_ERROR;
    out[1] = (uint8_t)cam;
    wr_be16(out + 2, connected ? 1 : 0);
    wr_be32(out + 4, model_id);
    wr_be32(out + 8, exp_us);
    wr_be16(out + 12, (uint16_t)gain);
    wr_be16(out + 14, (uint16_t)fps);
    out[16] = trg_mode;
    out[17] = trg_src;
    out[18] = testpat;
    out[19] = auto_exp;
    wr_be16(out + 20, (uint16_t)roi_w);
    wr_be16(out + 22, (uint16_t)roi_h);
    wr_be16(out + 24, (uint16_t)off_x);
    wr_be16(out + 26, (uint16_t)off_y);
    // Words 28..31: sensor maximum rate for the CURRENT ROI, 0.01 Hz.
    // It was read before but not returned, yet it is the only number that
    // checks the profile rate budget (docs/CONCEPT.md): does max_fps grow when
    // ROI width is cut. The body grew from 28 to 32 bytes - the client parses by
    // length, an old one reads the first 28 as before.
    wr_be32(out + 28, max_fps);

    return reply(fd, req, VCP_OP_GET_CAM_CONFIG_ACK, out, sizeof(out));
}

// ---- settings store (0x0160..0x016A, docs/DECISIONS.md R-14) -------
//
// Section record - 12 B, the same in all responses:
//   u8 section (0 settings, 1 calibration), u8 flags SETTINGS_F_*, u8 copy
//   (0 A, 1 B, 0xFF none), u8 0, u16 version, u16 data length, u32 seq.
#define VCP_SETTINGS_REC_BYTES  12U

static void put_settings_rec(uint8_t *p, uint8_t section, const settings_info_t *in)
{
    p[0] = section;
    p[1] = in->flags;
    p[2] = in->copy;
    p[3] = 0;
    wr_be16(p + 4, in->version);
    wr_be16(p + 6, in->length);
    wr_be32(p + 8, in->seq);
}

// 0x0160 save / 0x0162 load from store: body u8 section. Response 16 B:
// status, 0, 0, 0, section record. In WORK - DENIED (docs/ARCHITECTURE.md § 7).
static int handle_settings_save_load(int fd, const vmsp_header_t *req, const uint8_t *p,
                                     uint32_t plen)
{
    uint8_t out[4 + VCP_SETTINGS_REC_BYTES];
    settings_info_t in;
    int st;

    memset(out, 0, sizeof(out));
    memset(&in, 0, sizeof(in));
    if (plen != 1) {
        out[0] = VCP_ST_BAD_LENGTH;
    } else {
        st = (req->opcode == VCP_OP_SETTINGS_SAVE) ? settings_save(p[0], &in)
                                                   : settings_load(p[0], &in);
        out[0] = (uint8_t)st;
        put_settings_rec(out + 4, p[0], &in);
    }
    return reply(fd, req, (uint16_t)(req->opcode | 1U), out, sizeof(out));
}

// 0x0164: store state, empty body. Response 28 B: status, 0, 0, 0 and
// the records of sections 0 and 1.
static int handle_settings_info(int fd, const vmsp_header_t *req, uint32_t plen)
{
    uint8_t out[4 + SETTINGS_SEC_COUNT * VCP_SETTINGS_REC_BYTES];
    settings_info_t in;
    uint8_t sec;

    if (plen != 0) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    memset(out, 0, sizeof(out));
    for (sec = 0; sec < SETTINGS_SEC_COUNT; ++sec) {
        (void)settings_get_info(sec, &in);
        put_settings_rec(out + 4 + sec * VCP_SETTINGS_REC_BYTES, sec, &in);
    }
    return reply(fd, req, (uint16_t)(VCP_OP_SETTINGS_INFO | 1U), out, sizeof(out));
}

// 0x0166: reset RAM settings to compile-time defaults. Body u32
// guard word SETTINGS_RESET_GUARD ("RSET"). store is not touched -
// the defaults can be saved by a following 0x0160. Response: StatusAck.
static int handle_settings_reset(int fd, const vmsp_header_t *req, const uint8_t *p,
                                 uint32_t plen)
{
    if (plen != 4) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    return reply_status(fd, req, (uint8_t)settings_defaults(rd_be32(p)));
}

// 0x0168: upload a section image to the PC. Body: u8 section, u8 source (0 -
// current settings from RAM, 1 - a whole copy from the chip). Response: status,
// section, source, 0, u16 image length, u16 0 and the image (16 B header +
// data) - exactly what 0x016A accepts back.
static int handle_settings_export(int fd, const vmsp_header_t *req, const uint8_t *p,
                                  uint32_t plen)
{
    uint8_t out[8 + SETTINGS_IMAGE_MAX];
    uint32_t n = 0;
    int st;

    memset(out, 0, 8);
    if (plen != 2) {
        out[0] = VCP_ST_BAD_LENGTH;
        return reply(fd, req, (uint16_t)(VCP_OP_SETTINGS_EXPORT | 1U), out, 8);
    }
    st = settings_export(p[0], p[1] != 0U, out + 8, SETTINGS_IMAGE_MAX, &n);
    out[0] = (uint8_t)st;
    out[1] = p[0];
    out[2] = p[1];
    wr_be16(out + 4, (uint16_t)n);
    return reply(fd, req, (uint16_t)(VCP_OP_SETTINGS_EXPORT | 1U), out, 8U + n);
}

// 0x016A: load an image from the PC. Body: u8 section, u8 flags ([0] also
// write to store), u16 0, image. Tag, version, CRC and every value are
// checked; any one out of range - DENIED, and RAM is unchanged. The set of
// the current mode is applied at once. Response 16 B, as for 0x0160.
static int handle_settings_import(int fd, const vmsp_header_t *req, const uint8_t *p,
                                  uint32_t plen)
{
    uint8_t out[4 + VCP_SETTINGS_REC_BYTES];
    settings_info_t in;
    int st;

    memset(out, 0, sizeof(out));
    memset(&in, 0, sizeof(in));
    if (plen < 4U + SETTINGS_HDR_BYTES) {
        out[0] = VCP_ST_BAD_LENGTH;
    } else {
        st = settings_import(p[0], p + 4, plen - 4U, (p[1] & 1U) != 0U, &in);
        out[0] = (uint8_t)st;
        put_settings_rec(out + 4, p[0], &in);
    }
    return reply(fd, req, (uint16_t)(VCP_OP_SETTINGS_IMPORT | 1U), out, sizeof(out));
}

static int handle_ota_start(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[16];
    uint16_t out_len = 0;
    if (plen < 8U) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    uint8_t target = p[0];
    uint32_t total_size = rd_be32(p + 4);
    uint32_t expected_crc = (plen >= 12U) ? rd_be32(p + 8) : 0U;
    ota_start(target, total_size, expected_crc, out, &out_len);
    return reply(fd, req, VCP_OP_OTA_START_ACK, out, out_len);
}

static int handle_ota_data(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[16];
    uint16_t out_len = 0;
    if (plen < 4U) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    uint32_t offset = rd_be32(p);
    ota_data(offset, p + 4, (uint16_t)(plen - 4U), out, &out_len);
    return reply(fd, req, VCP_OP_OTA_DATA_ACK, out, out_len);
}

static int handle_ota_commit(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[16];
    uint16_t out_len = 0;
    if (plen < 4U) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    uint32_t expected_crc = rd_be32(p);
    ota_commit(expected_crc, out, &out_len);
    return reply(fd, req, VCP_OP_OTA_COMMIT_ACK, out, out_len);
}

static int handle_ota_reboot(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[4] = { 0 };
    uint32_t delay_ms = (plen >= 4U) ? rd_be32(p) : 100U;
    reply(fd, req, VCP_OP_OTA_REBOOT_ACK, out, 1);
    return ota_reboot(delay_ms);
}

static int handle_ota_status(int fd, const vmsp_header_t *req, uint32_t plen)
{
    uint8_t out[64];
    uint16_t out_len = 0;
    (void)plen;
    ota_get_status(out, &out_len);
    return reply(fd, req, VCP_OP_OTA_STATUS_ACK, out, out_len);
}

static int handle_ota_set_slot(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    uint8_t out[8];
    uint16_t out_len = 0;
    if (plen < 1) return reply_status(fd, req, VCP_ST_BAD_LENGTH);
    ota_set_active_slot(p[0], out, &out_len);
    return reply(fd, req, VCP_OP_OTA_SET_SLOT_ACK, out, out_len);
}

static int handle_ota_version(int fd, const vmsp_header_t *req, uint32_t plen)
{
    uint8_t out[64];
    uint8_t slot = ota_current_slot();
    const char *ver = ota_get_version_string();
    uint8_t ver_len = (uint8_t)strlen(ver);

    (void)plen;
    out[0] = 0U;                          /* status OK */
    out[1] = slot;                        /* active slot (0=A, 1=B) */
    out[2] = (slot == 1U) ? 2U : 1U;      /* Version Major */
    out[3] = (slot == 1U) ? 1U : 0U;      /* Version Minor */
    out[4] = 0U;                          /* Version Patch */
    out[5] = ota_get_led_pattern();       /* LED pattern code */
    wr_be32(out + 6, spi_flash_build_id());
    out[10] = ver_len;
    memset(out + 11, 0, 40);
    if (ver_len > 39) ver_len = 39;
    memcpy(out + 11, ver, ver_len);

    return reply(fd, req, VCP_OP_OTA_VERSION_ACK, out, 11U + ver_len);
}

static void log_vmsp_to_lcd(uint16_t op, uint32_t plen)
{
    static uint32_t s_ota_data_cnt = 0;
    const char *name = NULL;
    uint8_t col = LCD_COL_CYAN;
    char msg[48];

    if (!lcd_hud_is_active()) return;

    switch (op) {
    case VCP_OP_WORK_MODE:      name = "WORK_MODE"; break;
    case VCP_OP_CAMERA_CONFIG:  name = "CAM_CFG"; break;
    case VCP_OP_EXPOSURE:       name = "EXPOSURE"; break;
    case VCP_OP_GET_CAM_CONFIG: name = "GET_CAM"; break;
    case VCP_OP_SYNC_CHANNEL:   name = "SYNC_CH"; break;
    case VCP_OP_SYNC_CONTROL:   name = "SYNC_CTL"; break;
    case VCP_OP_SW_TRIGGER:     name = "SW_TRIG"; col = LCD_COL_GOLD; break;
    case VCP_OP_STATUS:         name = "STATUS"; break;
    case VCP_OP_LASER_CONFIG:   name = "LASER_CFG"; break;
    case VCP_OP_OTA_START:      name = "OTA_START"; col = LCD_COL_GOLD; s_ota_data_cnt = 0; break;
    case VCP_OP_OTA_DATA:
        if ((++s_ota_data_cnt & 0x0FU) != 1U) {
            lcd_hud_pulse_rx();
            return;
        }
        name = "OTA_DATA";
        col = LCD_COL_GREEN;
        break;
    case VCP_OP_OTA_COMMIT:     name = "OTA_COMMIT"; col = LCD_COL_GREEN; break;
    case VCP_OP_OTA_REBOOT:     name = "OTA_REBOOT"; col = LCD_COL_RED; break;
    case VCP_OP_OTA_STATUS:     name = "OTA_STATUS"; break;
    case VCP_OP_OTA_SET_SLOT:   name = "OTA_SLOT"; col = LCD_COL_GOLD; break;
    case VCP_OP_OTA_VERSION:    name = "OTA_VER"; break;
    case VCP_OP_CAPTURE_IMAGE:  name = "CAPTURE"; col = LCD_COL_MAGENTA; break;
    case VCP_OP_FETCH_3D:       name = "FETCH_3D"; col = LCD_COL_MAGENTA; break;
    case VCP_OP_PSTREAM:        name = "PSTREAM"; col = LCD_COL_MAGENTA; break;
    case VCP_OP_MEM_DUMP:       name = "MEM_DUMP"; break;
    default:                    name = "CMD"; break;
    }
    snprintf(msg, sizeof(msg), "0x%04X %-10s (%u B)", (unsigned int)op, name, (unsigned int)plen);
    lcd_hud_log_exchange(col, "VMSP", msg);
}

static int dispatch_control(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    if (fd != VCP_FD_LOCAL) log_vmsp_to_lcd(req->opcode, plen);
    switch (req->opcode) {
    case VCP_OP_WORK_MODE:     return handle_work_mode(fd, req, p, plen);
    case VCP_OP_CAMERA_CONFIG: return handle_camera_config(fd, req, p, plen);
    case VCP_OP_EXPOSURE:      return handle_exposure(fd, req, p, plen);
    case VCP_OP_GET_CAM_CONFIG: return handle_get_camera_config(fd, req, p, plen);
    case VCP_OP_SYNC_CHANNEL:  return handle_sync_channel(fd, req, p, plen);
    case VCP_OP_SYNC_CONTROL:  return handle_sync_control(fd, req, p, plen);
    case VCP_OP_SW_TRIGGER:    return handle_sw_trigger(fd, req, plen);
    case VCP_OP_STATUS:        return handle_status(fd, req, plen);
    case VCP_OP_TRIG_SERIES:   return handle_trig_series(fd, req, p, plen);
    case VCP_OP_LASER_CONFIG:  return handle_laser_config(fd, req, p, plen);
    case VCP_OP_SPATIAL_CALIB: return handle_spatial_calib(fd, req, p, plen);
    case VCP_OP_MM_WRITE:      return handle_mm_write(fd, req, p, plen);
    case VCP_OP_MM_COMMIT:     return handle_mm_commit(fd, req, p, plen);
    case VCP_OP_MM_INFO:       return handle_mm_info(fd, req, p, plen);
    case VCP_OP_SETTINGS_SAVE:
    case VCP_OP_SETTINGS_LOAD:   return handle_settings_save_load(fd, req, p, plen);
    case VCP_OP_SETTINGS_INFO:   return handle_settings_info(fd, req, plen);
    case VCP_OP_SETTINGS_RESET:  return handle_settings_reset(fd, req, p, plen);
    case VCP_OP_SETTINGS_EXPORT: return handle_settings_export(fd, req, p, plen);
    case VCP_OP_SETTINGS_IMPORT: return handle_settings_import(fd, req, p, plen);
    case VCP_OP_OTA_START:       return handle_ota_start(fd, req, p, plen);
    case VCP_OP_OTA_DATA:        return handle_ota_data(fd, req, p, plen);
    case VCP_OP_OTA_COMMIT:      return handle_ota_commit(fd, req, p, plen);
    case VCP_OP_OTA_REBOOT:      return handle_ota_reboot(fd, req, p, plen);
    case VCP_OP_OTA_STATUS:      return handle_ota_status(fd, req, plen);
    case VCP_OP_OTA_SET_SLOT:    return handle_ota_set_slot(fd, req, p, plen);
    case VCP_OP_OTA_VERSION:     return handle_ota_version(fd, req, plen);
    case VCP_OP_CAPTURE_IMAGE:
        // Frames, capture and dump go only over data channel 39152, so that long
        // operations do not hold up control.
        return reply_capture_error(fd, req, VCP_ST_DENIED);
    case VCP_OP_FETCH_3D:
        return reply_slice_error(fd, req, VCP_ST_DENIED);
    case VCP_OP_PSTREAM:
        return ps_send_empty(fd, req, VCP_ST_DENIED, 0, 0, 0);
    case VCP_OP_ACQUIRE:
    case VCP_OP_MEM_DUMP:
        return reply_status(fd, req, VCP_ST_DENIED);
    default:
        return reply_status(fd, req, VCP_ST_BAD_OPCODE);
    }
}

int vmsp_call(uint16_t opcode, const uint8_t *body, uint32_t len, uint8_t *out, uint32_t cap)
{
    vmsp_header_t req;

    memset(&req, 0, sizeof(req));
    req.opcode = opcode;
    req.protocol_ver = VMSP_PROTOCOL_VERSION;
    s_local.buf = out;
    s_local.cap = cap;
    s_local.len = 0;
    if (dispatch_control(VCP_FD_LOCAL, &req, body, len) < 0) return -1;
    return (int)s_local.len;
}

static int dispatch_data(int fd, const vmsp_header_t *req, const uint8_t *p, uint32_t plen)
{
    log_vmsp_to_lcd(req->opcode, plen);
    switch (req->opcode) {
    case VCP_OP_CAPTURE_IMAGE: return handle_capture(fd, req, p, plen);
    case VCP_OP_ACQUIRE:       return handle_acquire(fd, req, p, plen);
    case VCP_OP_FETCH_3D:      return handle_fetch_3d(fd, req, p, plen);
    case VCP_OP_PSTREAM:       return handle_profile_stream(fd, req, p, plen);
    case VCP_OP_MEM_DUMP:      return handle_mem_dump(fd, req, p, plen);
    default:                   return reply_status(fd, req, VCP_ST_BAD_OPCODE);
    }
}

// One connection: header and body read exactly, checks in the order of spec
// § 2.1. Bad signature or version, body over the limit - disconnect;
// bad CRC or addressee - error response, the connection lives on.
static void serve_client(int fd, bool data_port)
{
    static uint8_t payload_ctrl[VCP_MAX_PAYLOAD];
    static uint8_t payload_data[VCP_MAX_PAYLOAD];
    uint8_t *payload = data_port ? payload_data : payload_ctrl;
    uint8_t hdr[VMSP_HEADER_SIZE];
    vmsp_header_t req;

    for (;;) {
        if (recv_exact(fd, hdr, sizeof(hdr)) < 0) break;
        if (memcmp(hdr, "VMSP", 4) != 0) {
            printf("[VCP] bad magic, closing\r\n");
            break;
        }
        memcpy(req.magic, hdr, 4);
        req.protocol_ver = rd_be16(hdr + 4);
        req.opcode = rd_be16(hdr + 6);
        req.seq_num = rd_be32(hdr + 8);
        req.sender_id = rd_be32(hdr + 12);
        req.target_id = rd_be32(hdr + 16);
        req.payload_len = rd_be32(hdr + 20);
        req.payload_crc32 = rd_be32(hdr + 24);

        if (req.protocol_ver != VMSP_PROTOCOL_VERSION) {
            (void)reply_status(fd, &req, VCP_ST_BAD_VERSION);
            break;
        }
        if (req.payload_len > VCP_MAX_PAYLOAD) {
            printf("[VCP] payload_len %u too large, closing\r\n", (unsigned int)req.payload_len);
            break;
        }
        if (req.payload_len && recv_exact(fd, payload, req.payload_len) < 0) break;

        if (req.target_id != 0U && req.target_id != VCP_NODE_ID && req.target_id != 0xFFFFFFFFU) {
            if (reply_status(fd, &req, VCP_ST_BAD_TARGET) < 0) break;
            continue;
        }
        if (req.payload_len && vmsp_crc32(payload, req.payload_len) != req.payload_crc32) {
            int r = (req.opcode == VCP_OP_CAPTURE_IMAGE)
                        ? reply_capture_error(fd, &req, VCP_ST_BAD_CRC)
                        : reply_status(fd, &req, VCP_ST_BAD_CRC);
            if (r < 0) break;
            continue;
        }
        if ((data_port ? dispatch_data(fd, &req, payload, req.payload_len)
                       : dispatch_control(fd, &req, payload, req.payload_len)) < 0)
            break;
    }
    // Short pause before closing: let lwIP send the last response.
    // Do not use SO_LINGER={1,0} - it breaks the old NETCONN teardown.
    int err = errno;                      // why recv/send failed (ECONNABORTED: keepalive)
    vTaskDelay(pdMS_TO_TICKS(10));
    lwip_close(fd);
    printf("[VCP] %s client closed (errno %d)\r\n", data_port ? "data" : "control", err);
    if (lcd_hud_is_active()) {
        lcd_hud_set_pc_state("LISTENING (39151 / 3956)", LCD_COL_WHITEBLUE);
        lcd_hud_log_exchange(LCD_COL_GRAY, "TCP", data_port ? "Data client closed" : "Ctrl client closed");
    }
}

// ---- discovery ------------------------------------------------------------

// The running address (net_config.c: store or the board.h default), not the
// compile-time constant (until 15.10.2026).
static uint32_t own_ip_be(void)
{
    net_config_t c;
    net_config_current(&c);
    return ((uint32_t)c.ip[0] << 24) | ((uint32_t)c.ip[1] << 16) | ((uint32_t)c.ip[2] << 8) | c.ip[3];
}

static int build_discovery_info(uint8_t *packet, size_t size, uint32_t seq, uint32_t target)
{
    uint8_t payload[28];

    memset(payload, 0, sizeof(payload));
    wr_be32(payload + 12, 0xAE350001U);
    memcpy(payload + 16, gnetif.hwaddr, 6);       /* the running MAC (store or board.h) */
    wr_be32(payload + 22, own_ip_be());
    payload[26] = (uint8_t)VCP_NODE_ID;
    payload[27] = 0x02;
    return (int)vmsp_encode_packet(VCP_OP_DISCOVERY_INFO, seq, VCP_NODE_ID, target,
                                   payload, sizeof(payload), packet, size);
}

void task_vmsp_discovery_entry(void *pvParameters)
{
    const uint8_t *own_mac = gnetif.hwaddr;       /* the running MAC (net_config.c) */
    struct sockaddr_in bind_addr;
    uint8_t rx[512];
    uint8_t tx[VMSP_HEADER_SIZE + 28];
    int fd;
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(3500));
    fd = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        printf("[VCP] discovery socket failed\r\n");
        vTaskDelete(NULL);
        return;
    }
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(VMSP_PORT_DISCOVERY);
    if (lwip_bind(fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        printf("[VCP] discovery bind failed\r\n");
        lwip_close(fd);
        vTaskDelete(NULL);
        return;
    }
    {
        int on = 1;   // answers to a PC outside the subnet go by broadcast (below)
        (void)lwip_setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
    }
    printf("[VCP] UDP discovery listening on %u\r\n", VMSP_PORT_DISCOVERY);

    for (;;) {
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        vmsp_packet_t pkt;
        int n = lwip_recvfrom(fd, rx, sizeof(rx), 0, (struct sockaddr *)&src, &src_len);
        int len = 0;

        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        // Bad signature/version/CRC over UDP - silently (spec § 4.1).
        if (vmsp_decode_packet(rx, (size_t)n, VCP_NODE_ID, &pkt) != VMSP_DEC_OK) continue;
        // Since 02.10.2026 the board is discoverable only in STANDBY (owner's
        // decision): in WORK and SERVICE the port is fully silent, 0x0003 too. The
        // socket is not closed - the packet is simply dropped, there is no point
        // reopening the socket on every mode change. A client reaches a board in
        // WORK/SERVICE over TCP at a known address (0x0300 answers in any mode).
        if (work_mode_get() != VCP_MODE_STANDBY) continue;

        bool assign = false;
        net_config_t nc;
        if (pkt.header.opcode == VCP_OP_DISCOVERY_PROBE) {
            len = build_discovery_info(tx, sizeof(tx), pkt.header.seq_num, pkt.header.sender_id);
        } else if (pkt.header.opcode == VCP_OP_ASSIGN_IP && pkt.payload_len >= 22) {
            // STP 2.4: MAC 6, IP, mask, gateway, reserved 4. Someone else's MAC - not
            // for us, stay silent. Since 15.10.2026 the address is changed and stored
            // in the store (net_config.c, spec § 4.1); reserved byte 0 bit 0 = 1:
            // apply without storing (lost on reboot).
            uint8_t st;
            bool nostore = (pkt.payload[18] & 1U) != 0;
            int rc;
            if (memcmp(pkt.payload, own_mac, 6) != 0) continue;
            memcpy(nc.ip, pkt.payload + 6, 4);
            memcpy(nc.mask, pkt.payload + 10, 4);
            memcpy(nc.gw, pkt.payload + 14, 4);
            rc = !net_config_valid(&nc) ? 1 : nostore ? 0 : net_config_store(&nc);
            st = (rc == 0) ? VCP_ST_OK : (rc == 1) ? VCP_ST_DENIED : VCP_ST_HW_ERROR;
            assign = (rc == 0);
            printf("[VCP] 0x0003: %u.%u.%u.%u/%u.%u.%u.%u gw %u.%u.%u.%u%s -> status 0x%02X\r\n",
                   nc.ip[0], nc.ip[1], nc.ip[2], nc.ip[3], nc.mask[0], nc.mask[1], nc.mask[2], nc.mask[3],
                   nc.gw[0], nc.gw[1], nc.gw[2], nc.gw[3], nostore ? " (not stored)" : "", st);
            len = (int)vmsp_encode_packet(VCP_OP_ASSIGN_IP_ACK, pkt.header.seq_num, VCP_NODE_ID,
                                          pkt.header.sender_id, &st, 1, tx, sizeof(tx));
        } else if (pkt.header.opcode == VCP_OP_ASSIGN_MAC && pkt.payload_len >= 12) {
            // Since 22.10.2026: current MAC 6, new MAC 6 -> store, applies at the next boot
            // (changing it live would strand every ARP cache). New MAC all zero erases the record.
            static const uint8_t zero[6] = { 0 };
            const uint8_t *nm = pkt.payload + 6;
            bool erase = memcmp(nm, zero, 6) == 0;
            uint8_t st;
            int rc;
            if (memcmp(pkt.payload, own_mac, 6) != 0) continue;
            rc = erase ? net_mac_store(NULL) : net_mac_store(nm);
            st = (rc == 0) ? VCP_ST_OK : (rc == 1) ? VCP_ST_DENIED : VCP_ST_HW_ERROR;
            printf("[VCP] 0x0005: MAC %02X:%02X:%02X:%02X:%02X:%02X%s -> status 0x%02X (applies at the next boot)\r\n",
                   nm[0], nm[1], nm[2], nm[3], nm[4], nm[5], erase ? " (erase, default back)" : "", st);
            len = (int)vmsp_encode_packet(VCP_OP_ASSIGN_MAC_ACK, pkt.header.seq_num, VCP_NODE_ID,
                                          pkt.header.sender_id, &st, 1, tx, sizeof(tx));
        }
        if (len > 0) {
            // A PC outside the board's subnet (the board was moved to another
            // network) cannot get a unicast answer: the board would send it to its
            // gateway. Answer such a PC by broadcast to its port (15.10.2026).
            struct sockaddr_in dst = src;
            if (!net_config_on_link(ntohl(src.sin_addr.s_addr)))
                dst.sin_addr.s_addr = htonl(INADDR_BROADCAST);
            lwip_sendto(fd, tx, (size_t)len, 0, (struct sockaddr *)&dst, sizeof(dst));
        }
        if (assign) {
            vTaskDelay(pdMS_TO_TICKS(50));   // let the answer leave from the old address
            (void)net_config_set(&nc, false);
        }
    }
}

// ---- TCP services -----------------------------------------------------------

static int make_listener(uint16_t port, int backlog)
{
    int fd = lwip_socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr;
    int reuse = 1;

    if (fd < 0) {
        printf("[VCP] socket(%u) failed errno=%d\r\n", port, errno);
        return -1;
    }
    lwip_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (lwip_bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("[VCP] bind(%u) failed errno=%d\r\n", port, errno);
        lwip_close(fd);
        return -1;
    }
    if (lwip_listen(fd, backlog) < 0) {
        printf("[VCP] listen(%u) failed errno=%d\r\n", port, errno);
        lwip_close(fd);
        return -1;
    }
    return fd;
}

static void accept_loop(uint16_t port, bool data_port, uint32_t start_delay_ms)
{
    int listen_fd;

    vTaskDelay(pdMS_TO_TICKS(start_delay_ms));
    // The listener is created once and lives forever: re-creating sockets on a
    // timer is what corrupted the NETCONN pool (ERRORS 3.2, former 29.12).
    while ((listen_fd = make_listener(port, 2)) < 0) {
        printf("[VCP] listener %u failed, retrying\r\n", port);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    printf("[VCP] TCP %s listening on %u\r\n", data_port ? "data" : "control", port);

    for (;;) {
        struct sockaddr_in client;
        socklen_t client_len = sizeof(client);
        struct timeval snd;
        int on = 1, idle = 10, intvl = 2, cnt = 5;
        int fd = lwip_accept(listen_fd, (struct sockaddr *)&client, &client_len);

        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        printf("[VCP] %s client %s\r\n", data_port ? "data" : "control",
               inet_ntoa(client.sin_addr));
        if (lcd_hud_is_active()) {
            char cmsg[40];
            snprintf(cmsg, sizeof(cmsg), "PC %s (%s)", inet_ntoa(client.sin_addr), data_port ? "DATA" : "CTRL");
            lcd_hud_set_pc_state(cmsg, LCD_COL_GREEN);
            lcd_hud_log_exchange(LCD_COL_GREEN, "CONN", cmsg);
        }
        // An idle but live client keeps its connection as long as it likes; a vanished one
        // (cable, PC crash) is dropped by keepalive in ~20 s instead of holding the port
        // forever, and a send it stops acknowledging fails in 10 s (ERRORS 3.17).
        snd.tv_sec = 10;
        snd.tv_usec = 0;
        if (lwip_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd)) < 0 ||
            lwip_setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on)) < 0 ||
            lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle)) < 0 ||
            lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl)) < 0 ||
            lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt)) < 0)
            printf("[VCP] socket timeouts NOT set: a lost client may hold the port\r\n");
        if (data_port) {
            int nodelay = 1;
            lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        }
        serve_client(fd, data_port);
    }
}

void task_vmsp_control_entry(void *pvParameters)
{
    (void)pvParameters;
    accept_loop(NET_VMSP_CONTROL_PORT, false, 4000U);
}

void task_vmsp_video_entry(void *pvParameters)
{
    (void)pvParameters;
    accept_loop(NET_VMSP_VIDEO_PORT, true, 4500U);
}
