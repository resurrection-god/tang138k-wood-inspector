/*
 * mm_table.h - "pixel -> mm" table: upload into DDR3, the band for the ROI, the
 * MMTB mailbox block (16.10.2026, docs/DECISIONS.md R-38, docs/ARCHITECTURE.md § 6.3)
 *
 * The PC builds the table from the calibration (tools/laser/mm_table.py) and
 * uploads it with 0x0132 / 0x0134; the FPGA turns every profile point into mm by
 * bilinear interpolation in it (fpga/src/laser_mm.v). Changing the model needs
 * neither a bitstream nor firmware.
 *
 * Here: the table of each camera in DDR3 (MM_TABLE_ADDR), checked by CRC-32 on
 * commit; the band of node rows under the camera's current ROI; the MMTB block of
 * the mailbox that makes the FPGA fetch the band (lane5_reader.v); the FPGA echo.
 * The table is kept in the SPI flash data area (two copies per camera, R-39) and
 * loaded from there at start-up; the PC writes the flash over JTAG
 * (tools/laser/mm_flash.py) or the board itself when 0x0134 asks to save (our own
 * flash controller, fpga/src/flash_ctl.v, ERRORS 5.12); an upload goes to RAM. With
 * no whole copy in flash every point carries "no mm" until the PC uploads a table.
 *
 * The ROI is followed through camera.c's ROI listener: "changing" switches the
 * camera's band off before the sensor is touched, "settled" computes the band
 * from the read-back ROI. The profile stream checks every block against
 * mm_expected() and blanks the mm of a block computed with another band.
 */

#ifndef APP_MM_TABLE_H_
#define APP_MM_TABLE_H_

#include <stdint.h>
#include <stdbool.h>
#include "camera.h"

// Table geometry - the same numbers as mm_table.py and laser_mm.v.
#define MM_GRID_K_LOG2      4                       /* a node every 16 columns */
#define MM_GRID_V_LOG2      3                       /* a node every 8 rows     */
#define MM_NODE_COLS        69
#define MM_NODE_ROWS        161
#define MM_TABLE_WORDS      (MM_NODE_COLS * MM_NODE_ROWS)
#define MM_TABLE_BYTES      (MM_TABLE_WORDS * 4U)  /* 44 436 */
#define MM_BAND_ROWS_MAX    28                      /* 4 BSRAM banks of 512: ROI up to 209 rows */
#define MM_HDR_VERSION      1U
#define MM_HDR_BYTES        20U                     /* 0x0134 body */

// MMTB block of the mailbox (lane5_reader.v): 50 magic, 51 + 5c base, 52 + 5c
// CRC of the band, 53 + 5c table id, 54 + 5c {kofs, vofs}, 55 + 5c {rows [12:8],
// enable [0]}, 61 APPLY_SEQ.
#define MMTB_W_MAGIC        50U
#define MMTB_W_CAM(c)       (51U + 5U * (uint32_t)(c))
#define MMTB_W_SEQ          61U
#define MMTB_MAGIC          0x4D4D5442U             /* "MMTB" */

// Telemetry of each camera: word 38 {'M', APPLY_SEQ low byte, [15] band valid,
// [14] CRC mismatch, [13] load error, [12] loading, [4:0] rows}, 39 - CRC loaded.
#define MM_T_ECHO           38
#define MM_T_CRC            39
#define MM_T_ID_BYTE        0x4DU                   /* 'M' */

// Why a camera's band is off.
#define MM_OFF_NONE         0   /* on */
#define MM_OFF_NO_TABLE     1   /* no table, or an upload in progress */
#define MM_OFF_ROI_TALL     2   /* the ROI needs more than MM_BAND_ROWS_MAX node rows */
#define MM_OFF_ROI_BAD      3   /* the ROI read back from the camera is not what was asked */
#define MM_OFF_CHANGING     4   /* the camera is being reconfigured */

// The table header (0x0134 body, big-endian on the wire).
typedef struct {
    uint8_t  cam, version, gk_log2, gv_log2;
    uint16_t cols, rows;
    uint32_t id, crc;
    uint8_t  unit_shift, frame;
} mm_hdr_t;

typedef struct {
    bool     valid;          // telemetry has the loader words (bitstream with R-38)
    uint8_t  seq;            // APPLY_SEQ low byte of the last job
    bool     band_valid, crc_fail, load_err, loading;
    uint8_t  rows;
    uint32_t crc;            // CRC of the last load
} mm_echo_t;

#define MM_ST_NONE          0
#define MM_ST_UPLOADING     1
#define MM_ST_READY         2

typedef struct {
    uint8_t   state;         // MM_ST_*
    uint32_t  received;      // bytes uploaded in order since offset 0
    mm_hdr_t  hdr;           // valid in MM_ST_READY
    uint8_t   off_why;       // MM_OFF_*
    uint8_t   r0, rows;      // band: first node row, node rows
    int16_t   vofs;          // 1/16 px
    uint16_t  kofs;
    uint16_t  roi_h, off_y;  // ROI the band is for
    uint32_t  band_crc;
    uint32_t  seq;           // last APPLY_SEQ published
    mm_echo_t echo;
} mm_info_t;

// At start-up, before the cameras: MMTB with both cameras off (a stale block in
// DDR3 would make the FPGA load a band nobody knows about), the ROI listener.
void mm_init(void);

// 0x0132: table bytes in order (offset 0 starts anew and switches the camera's
// band off at once). A VCP_ST_* code.
int  mm_write(uint8_t cam, uint32_t off, const uint8_t *data, uint32_t len);
// 0x0134: header, all bytes present, CRC of the table as it lies in DDR3; then the
// band for the current ROI goes to the FPGA. A VCP_ST_* code.
int  mm_commit(const uint8_t *body, uint32_t len, uint8_t *cam_out);

void mm_info(uint8_t cam, mm_info_t *out);
bool mm_read_echo(uint8_t cam, mm_echo_t *e);
// Wait until the FPGA reports the last published job for this camera (telemetry
// every 0.5 s). 1 = done and the band state is as expected, 0 = not in time or
// different, -1 = the bitstream has no loader.
int  mm_wait_fpga(uint8_t cam, uint32_t timeout_ms);

// What a profile block must carry for its mm to be trusted now: false - the
// camera's band is off (no block may carry mm), true - id and band CRC.
bool mm_expected(uint8_t cam, uint32_t *table_id, uint32_t *band_crc);

// ---- The table in SPI flash (16.10.2026, docs/DECISIONS.md R-39) ---------------
// Two copies per camera in the flash data area (spi_flash.h); mm_init() takes the
// newest whole one into DDR3 at start-up. mm_save() writes the camera's current
// table (MM_ST_READY) into the other copy; SPI_FLASH_* code; *copy - which.
int mm_save(uint8_t cam, uint8_t *copy);

typedef struct {
    uint8_t  mask;           // copies with a whole header (bit k)
    int8_t   used;           // copy the RAM table came from or was saved to, -1 none
    uint32_t seq[2];
    uint32_t id[2];          // table id in each copy
} mm_flash_info_t;
void mm_flash_info(uint8_t cam, mm_flash_info_t *out);

#endif /* APP_MM_TABLE_H_ */
