/*
 * gvsp.h - GigE Vision streaming (GVSP) scheduler, stage 3 of
 * docs/protocol/GEV_PROTOCOL_DESIGN.md (§ 5.7): the FPGA packet engine (udp_tx, GVSP job
 * kind) sends each frame as a GVSP block - leader, image + chunks, trailer, extended ID;
 * this task finds the frames, hands the jobs over and serves PACKETRESEND from the ring.
 * Stream channel x = camera x. DeviceScanType Areascan: frames, a job per frame. Linescan3D
 * (§ 5.8, since 27.10.2026): profile blocks of N lines as multi-part, which the FPGA sends by
 * itself from a stream record; here the record, the drain at stop, resends and a watchdog.
 *
 * The GVCP task (gvcp.c) configures the channels and starts / stops acquisition through
 * these calls; they only post to this task, never wait for a frame.
 */
#ifndef APP_GVSP_H_
#define APP_GVSP_H_

#include <stdint.h>
#include <stdbool.h>

#define GVSP_CHANNELS           2U
#define GVSP_SRC_PORT           39153U   /* the FPGA's UDP source port (GevSCSP) */
// The image's own chunk tag (GEV 2.2 § 25.1: the image is the first chunk) and the frame id,
// timestamp and frame CRC chunks with their tags.
#define GVSP_CHUNK_BYTES        56U
// Profile stream components (ComponentSelector index: 1 Range, 2 Reflectance, 3 Confidence).
#define GVSP_COMP_RANGE         0x02U
#define GVSP_COMP_REFLECTANCE   0x04U
#define GVSP_COMP_CONFIDENCE    0x08U
#define TASK_GVSP_PRIO          2
#define TASK_GVSP_STK_SIZE      2048

// Channel configuration (bootstrap SCP / SCDA / SCPS / SCPD). dst_port 0 = closed.
// dst_ip host order; scps = the whole IP packet size; scpd in 10 ns timestamp ticks.
void gvsp_channel_set(uint8_t ch, uint32_t dst_ip, uint16_t dst_port, uint16_t scps, uint32_t scpd);
// Start / stop sending the frames of the channels in mask (bit x = channel x). Start takes a
// baseline of the ring: only frames closed afterwards are sent.
void gvsp_start(uint8_t mask);
void gvsp_stop(uint8_t mask);
bool gvsp_running(uint8_t ch);
// PACKETRESEND (extended ID): packets first..last of block block_id on channel ch.
void gvsp_resend(uint8_t ch, uint64_t block_id, uint32_t first, uint32_t last);
// Bytes of a frame block's payload: 1080 x h image + chunks, or the image alone (payload type
// 0x0001 with ChunkModeActive off, for receivers without chunk support, ERRORS 3.23) (PayloadSize).
uint32_t gvsp_frame_payload(uint32_t h, bool chunks);
// Whether frames carry chunks: ChunkModeActive, or always on a bitstream without the no-chunk mode.
bool gvsp_frame_chunks(bool chunk_mode);
// Channel ch streams profiles (Linescan3D) or frames, set before gvsp_start: lines per block
// (1, 2, 4, 8, 16), components GVSP_COMP_*, mm (CalibratedAC) or pixels, the chunk part.
void gvsp_profile_set(uint8_t ch, bool profiles, uint8_t lines, uint8_t comps, bool mm, bool chunks);
// Bytes of such a block's payload, every part (PayloadSize); 0 = no component enabled.
uint32_t gvsp_profile_payload(uint8_t lines, uint8_t comps, bool mm, bool chunks);
// Guaranteed delivery of profile blocks (design § 5.5, vendor StreamDelivery* features): the
// mode is taken at the next start; the host acknowledges every block up to block_id.
void gvsp_delivery_set(uint8_t ch, bool guaranteed);
void gvsp_ack(uint8_t ch, uint64_t block_id);
typedef struct {
    uint64_t head, tail;                 // newest block sent, oldest block the ring still keeps
    uint32_t ring_free;                  // blocks that can still come before the gate shuts
    uint32_t overflow;                   // blocks lost to the gate since start
    uint32_t resends;
} gvsp_delivery_t;
void gvsp_delivery_status(uint8_t ch, gvsp_delivery_t *d);
// true once when the StreamOverflow event is due: the gate shut, then every second it stays
// shut; *block_id = the first block not acknowledged.
bool gvsp_overflow_event(uint8_t ch, uint64_t *block_id);
// Counters for the console / diagnostics.
void gvsp_counters(uint8_t ch, uint32_t *blocks, uint32_t *resends, uint32_t *errors);

void gvsp_init(void);                    // before the scheduler: the command queue
void task_gvsp_entry(void *pvParameters);

#endif /* APP_GVSP_H_ */
