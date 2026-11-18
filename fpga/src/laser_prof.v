// ============================================================================
// Module      : laser_prof
// Laser line profile block of one camera (stage L2a, ERRORS 4.5;
// docs/ARCHITECTURE.md § 6.2).
//
// The finder laser_col_extract.v sees every pixel word that the writer puts into
// a frame row, and after the frame is closed this module unloads the profile
// block into the same slot, next to the frame:
//
//   slot + 0x1A0000   block header (256 B)
//   slot + 0x1A0100   16 B records, one point per column (COLS points)
//   slot + 0x1A5100   closing burst, last word 0x12345678
//   slot + 0x1A5200   4 B points (version 2, since 30.09.2026)
//   slot + 0x1A6600   4 B mm words (version 3, since 16.10.2026)
//
// The layout is normative and mirrored: firmware/src/app/laser.h and
// tools/laser/laser_block.py. There is room for records of 1280 points (the axis
// along the row fits too), so FOOT_OFF does not depend on the axis.
//
// Point (version 2): {q[31:24], i[23:16], x_sub[15:0]} - centre of gravity in 1/16
// pixel with rounding, brightness of the maximum and the LASER_Q_* quality byte;
// computed by the pipelined divider laser_cog_div.v right on the record unload
// path (one point per four record words), bit-exact contract -
// tools/laser/model.py refine_fixed(). The processor has nothing left to divide:
// at a high fps (small ROI) it could not compute the points for every frame.
//
// mm word (version 3): {depth[31:16], along[15:0]} from the "pixel -> mm" table
// (laser_mm.v, docs/DECISIONS.md R-38), computed while the mm bursts are filled
// from the point memory; along = 0x8000 - no mm, depth then holds the reason.
// The point word itself does not change (Q_UNCALIB stays: it describes the pixel
// word), so version 2 readers read version 3 blocks as before.
//
// CRC (20.10.2026, GOAL "every profile carries a CRC computed in the FPGA"):
// header word 19 is the CRC-32 (zlib) of the COLS point words, word 20 of the COLS
// mm words, each over the words as little-endian bytes, taken as they go into the
// bursts; kind flag 0x04 says the words are there. The host checks them on what it
// received, so DDR3, the CPU's copy, lwIP and the wire are all covered.
//
// Burst order: records first (68 bursts at 1080 columns), then points
// (17), mm words (17), then the header, the closing burst LAST. The header comes
// after the records because it holds the number of points without a line, which
// is known only after the unload. The "block complete" marker is the closing
// burst (laser_block.block_complete).
//
// The table state of a block is taken once, at the unload start, together with
// the table's id, CRC and offsets for the header: mm are written only if the
// band was valid then and no load was asked for. The loader (lane5_reader.v)
// writes the band only after mm_load_ack, which rises only between unloads and
// holds while mm_load_req does, so a block never mixes two bands.
//
// The module talks to the writer (cam_channel.v) in bursts, like the diag_burst
// telemetry: prof_ready - a whole burst is in the buffer and its address in
// prof_addr; the writer sends it and in ST_PAUSE / ST_WRITE takes the words from
// prof_q (a register: the word_in_burst -> wr_data path gains one mux input from a
// register, and nothing combinational from here); prof_done - the burst has left,
// the next one can be prepared. While the unload runs, prof_busy keeps the writer
// from opening the next slot: the finder shares its memory read port between rows
// and the unload, and a new frame before the unload ends would corrupt it (the
// epoch bit).
//
// Everything on lane4_clk. No compare of this module enters the combinational
// cone of the writer FSM, except the two registers prof_ready and prof_busy
// (that cone has no margin; docs/WORKFLOWS.md § 3).
// ============================================================================

`timescale 1ns / 1ps

module laser_prof #(
    parameter integer COLS      = 1080,          // profile points = frame columns, a multiple of 4
    parameter integer ROW_BITS  = 11,
    parameter [7:0]   CAM_ID    = 8'd0,
    parameter [31:0]  FOOT_OFF  = 32'h0000_5100, // from the block start: 256 + 1280 * 16
    parameter [31:0]  PTS_OFF   = 32'h0000_5200, // from the block start: after the closing burst
    parameter [31:0]  MM_OFF    = 32'h0000_6600  // from the block start: after the points (1280 * 4)
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [31:0]          hw_timestamp,

    // Search parameters (mailbox, LASR words). Taken on frame_start.
    input  wire [7:0]           cfg_thresh,
    input  wire [7:0]           cfg_offset,
    input  wire [3:0]           cfg_gap,
    input  wire [ROW_BITS-1:0]  cfg_win_lo,
    input  wire [ROW_BITS-1:0]  cfg_win_hi,
    input  wire                 cfg_enable,      // write a block for this frame
    input  wire [15:0]          cfg_min_sum,     // sum_i below this - "no line"
    input  wire [15:0]          cfg_width_max,   // spot wider - the WIDE bit

    // From the writer: slot opened, pixel words, frame closed.
    input  wire                 frame_start,
    input  wire                 pix_valid,
    input  wire [31:0]          pix_word,
    input  wire [ROW_BITS-1:0]  pix_row,
    input  wire [15:0]          pix_k,
    input  wire [15:0]          lines_dropped,   // cumulative bridge counter
    input  wire                 frame_close,     // pulse, the values below are of this frame
    input  wire [31:0]          close_frame_id,
    input  wire [31:0]          close_block_base,  // the writer picks the ring (cam_channel.v BLOCK_OFF / dense ring)
    input  wire [10:0]          close_rows,      // rows opened + 1 (frame_rows_cnt)
    input  wire [31:0]          strobe_ts,

    // Block bursts for the writer.
    output wire                 prof_busy,
    output reg                  prof_ready,
    output reg  [31:0]          prof_addr,
    input  wire                 prof_first,      // ST_PAUSE of a block burst
    input  wire                 prof_next,       // ST_WRITE of a block burst
    output reg  [31:0]          prof_q,
    input  wire                 prof_done,       // burst sent

    // "pixel -> mm" table (lane5_reader.v; for camera 1 by wire from the camera 0
    // channel). The bus: [15:0] vofs (signed, 1/16 px), [27:16] kofs, [32:28] node
    // rows, [33] band valid, [34] load request, [95:64] table id, [127:96] band CRC.
    input  wire                 tw_en,
    input  wire [1:0]           tw_bank,
    input  wire [8:0]           tw_addr,
    input  wire [31:0]          tw_data,
    input  wire [127:0]         mm_cfg,
    output reg                  mm_load_ack,

    // Telemetry.
    output reg  [15:0]          o_blocks,        // blocks completed, cumulative
    output reg  [15:0]          o_no_line        // points without a line in the last block
);

    localparam integer REC_BURSTS = (COLS * 16 + 255) / 256;   // 68 at 1080
    localparam integer PT_BURSTS  = (COLS * 4 + 255) / 256;    // 17 at 1080
    localparam [6:0]   SEQ_PT0    = REC_BURSTS[6:0];
    localparam [6:0]   SEQ_MM0    = REC_BURSTS[6:0] + PT_BURSTS[6:0];
    localparam [6:0]   SEQ_HDR    = SEQ_MM0 + PT_BURSTS[6:0];
    localparam [6:0]   SEQ_FOOT   = SEQ_HDR + 7'd1;

    localparam [31:0] BLOCK_MAGIC = 32'h5052_4F46;   // "PROF"
    localparam [31:0] INTEGRITY   = 32'h1234_5678;
    localparam [7:0]  KIND_F_FPGA = 8'h01, KIND_F_PARTIAL = 8'h02, KIND_F_CRC = 8'h04;
    localparam [7:0]  BLOCK_VER   = 8'd3;             // 2: + points (30.09.2026), 3: + mm (16.10.2026)
    localparam [7:0]  AXIS_COL    = 8'd1;
    // Point quality bits - LASER_Q_* (laser.h), Q_* (model.py).
    localparam [7:0]  Q_NO_LINE = 8'h01, Q_SATURATED = 8'h02, Q_WIDE = 8'h04,
                      Q_MULTI = 8'h08, Q_UNCALIB = 8'h20, Q_PARTIAL = 8'h40;

    // ---- finder --------------------------------------------------------------
    reg         rec_rd;
    reg  [15:0] rec_col;
    reg  [1:0]  rec_part;
    wire [31:0] rec_word;
    wire        rec_valid;
    wire [19:0] stat_words;

    laser_col_extract #(
        .COLS     (COLS),
        .ROW_BITS (ROW_BITS)
    ) u_extract (
        .clk         (clk),
        .rst_n       (rst_n),
        .cfg_thresh  (cfg_thresh),
        .cfg_gap     (cfg_gap),
        .cfg_offset  (cfg_offset),
        .cfg_win_lo  (cfg_win_lo),
        .cfg_win_hi  (cfg_win_hi),
        .frame_start (frame_start),
        .pix_valid   (pix_valid),
        .pix_word    (pix_word),
        .pix_row     (pix_row),
        .pix_k       (pix_k),
        .rec_rd      (rec_rd),
        .rec_col     (rec_col),
        .rec_part    (rec_part),
        .rec_word    (rec_word),
        .rec_valid   (rec_valid),
        .stat_words  (stat_words)
    );

    // ---- what goes into the header: captured on frame_start / frame_close ----
    // The parameters are the same the finder takes in the same cycle (offset clipped
    // by the threshold the same way), so the header says what the block was computed with.
    reg                en_f;
    reg [7:0]          thr_f, off_f;
    reg [3:0]          gap_f;
    reg [ROW_BITS-1:0] lo_f, hi_f;
    reg [15:0]         msum_f, wmax_f;
    reg [31:0]         ts_f;
    reg [15:0]         drop_f;
    reg [31:0]         fid_c, strobe_c;
    reg [31:0]         blk_base;
    reg [15:0]         rows_c, drop_c;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_f <= 1'b0;
        end else if (frame_start) begin
            en_f <= cfg_enable;
        end
    end

    // Header data without reset: read only after frame_close, and that
    // happens only after frame_start.
    always @(posedge clk) begin
        if (frame_start) begin
            thr_f  <= cfg_thresh;
            off_f  <= (cfg_offset > cfg_thresh) ? cfg_thresh : cfg_offset;
            gap_f  <= cfg_gap;
            lo_f   <= cfg_win_lo;
            hi_f   <= cfg_win_hi;
            msum_f <= cfg_min_sum;
            wmax_f <= cfg_width_max;
            ts_f   <= hw_timestamp;
            drop_f <= lines_dropped;
        end
        if (frame_close) begin
            fid_c    <= close_frame_id;
            strobe_c <= strobe_ts;
            blk_base <= close_block_base;
            rows_c   <= {5'd0, close_rows} - 16'd1;
            drop_c   <= lines_dropped - drop_f;
        end
    end

    // ---- "pixel -> mm" table: the state of this block --------------------------
    // The bus and the band writes pass one register here (camera 1's come from the
    // camera 0 channel, across the die). A band write without the acknowledge is a
    // loader fault: it marks the table bad until the next load request.
    reg [127:0] mmc;
    reg         tw_en_q;
    reg [1:0]   tw_bank_q;
    reg [8:0]   tw_addr_q;
    reg [31:0]  tw_data_q;
    reg         mm_bad;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mmc     <= 128'd0;
            tw_en_q <= 1'b0;
        end else begin
            mmc     <= mm_cfg;
            tw_en_q <= tw_en;
        end
    end
    always @(posedge clk) begin
        tw_bank_q <= tw_bank;
        tw_addr_q <= tw_addr;
        tw_data_q <= tw_data;
    end
    wire mm_req   = mmc[34];
    wire mm_valid = mmc[33];

    reg         mmb_en;              // this block gets mm
    reg         mmb_bad;
    reg [15:0]  mmb_vofs;
    reg [11:0]  mmb_kofs;
    reg [4:0]   mmb_rows;
    reg [31:0]  mmb_id, mmb_crc;
    reg [15:0]  n_mm, n_out, n_node;

    // ---- unload ----------------------------------------------------------------
    reg        busy;
    reg [6:0]  seq;          // 0..REC_BURSTS-1 records, then header, closing
    reg        filling;      // buffer fill in progress
    reg [6:0]  iss;          // reads issued / header words (0..64)
    reg [6:0]  wcnt;         // written into the buffer (0..64)
    reg [5:0]  ok_d;         // the column exists; delay = rec_valid delay
    reg [15:0] no_line_cnt;
    reg        start_fill;
    reg        pt_v0, pt_v1, pt_ok0, pt_ok1;   // point memory read: 2 cycles to pt_q

    assign prof_busy = busy;

    // Burst buffer. Synchronous read, the address is the rptr register: 0 in ST_PAUSE,
    // incremented on every ST_WRITE cycle, so prof_q already holds the right word for
    // the next write cycle.
    reg [31:0] pbuf [0:63];
    reg [5:0]  wptr, rptr;
    reg        pbuf_we;
    reg [31:0] pbuf_wd;
    // Header / closing words go to pbuf one clock after their index is issued, from hdr_q /
    // foot_q: iss -> 21-way word select -> pbuf_wd was a clk_x1 path (8823136F, +0.113).
    reg        hd_pend;
    reg [31:0] hdr_q, foot_q;

    always @(posedge clk) begin
        if (pbuf_we) pbuf[wptr] <= pbuf_wd;
        prof_q <= pbuf[rptr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rptr <= 6'd0;
        else if (prof_done)                rptr <= 6'd0;
        else if (prof_first || prof_next)  rptr <= rptr + 6'd1;
    end

    // CRC-32, reflected (zlib), one 32-bit word, bits LSB first = bytes little-endian
    // (the same function as in lane5_reader.v).
    function [31:0] crc32_word;
        input [31:0] c;
        input [31:0] d;
        integer b;
        reg [31:0] x;
        begin
            x = c;
            for (b = 0; b < 32; b = b + 1)
                x = (x >> 1) ^ ((x[0] ^ d[b]) ? 32'hEDB8_8320 : 32'd0);
            crc32_word = x;
        end
    endfunction
    reg [31:0] crc_pt, crc_mm;           // running, not inverted; reset at the unload start

    // Header / closing burst word by index.
    reg [31:0] hdr_w, foot_w;
    wire [5:0] hi_idx = iss[5:0];
    wire [15:0] rows_used = rows_c;
    always @(*) begin
        case (hi_idx)
            6'd0:  hdr_w = BLOCK_MAGIC;
            6'd1:  hdr_w = fid_c;
            6'd2:  hdr_w = ts_f;
            6'd3:  hdr_w = strobe_c;
            6'd4:  hdr_w = {KIND_F_FPGA | KIND_F_CRC | ((drop_c != 16'd0) ? KIND_F_PARTIAL : 8'h00),
                            CAM_ID, BLOCK_VER, AXIS_COL};
            6'd5:  hdr_w = {COLS[15:0], 16'd16};
            6'd6:  hdr_w = {thr_f, 4'd0, gap_f, off_f, 8'd0};
            6'd7:  hdr_w = {rows_used, no_line_cnt};
            6'd8:  hdr_w = {16'd0, drop_c};
            6'd9:  hdr_w = {{(16-ROW_BITS){1'b0}}, hi_f, {(16-ROW_BITS){1'b0}}, lo_f};
            6'd10: hdr_w = {12'd0, stat_words};
            6'd11: hdr_w = {PTS_OFF[15:0], 16'd4};                // points: offset, bytes per point
            6'd12: hdr_w = {wmax_f, msum_f};                      // what the point flags were computed with
            // Version 3: mm words, and which table and band they were computed with.
            6'd13: hdr_w = {MM_OFF[15:0], 16'd4};                 // mm: offset, bytes per point
            6'd14: hdr_w = mmb_id;
            6'd15: hdr_w = mmb_crc;                               // CRC of the band, checked at load
            6'd16: hdr_w = {4'd0, mmb_kofs, mmb_vofs};
            6'd17: hdr_w = {mmb_en, mmb_bad, 1'b0, mmb_rows, 8'd0, n_mm};
            6'd18: hdr_w = {n_node, n_out};                       // points: invalid node, outside the band
            6'd19: hdr_w = ~crc_pt;                               // CRC-32 of the point words
            6'd20: hdr_w = ~crc_mm;                               // CRC-32 of the mm words
            default: hdr_w = 32'd0;
        endcase
        case (hi_idx)
            6'd0:  foot_w = fid_c;
            6'd63: foot_w = INTEGRITY;
            default: foot_w = 32'd0;
        endcase
    end
    always @(posedge clk) begin
        hdr_q  <= hdr_w;
        foot_q <= foot_w;
    end

    wire [15:0] col_now = {5'd0, seq, iss[5:2]};        // seq * 16 + iss / 4
    wire        is_rec  = (seq < SEQ_PT0);
    wire        is_pt   = (seq >= SEQ_PT0) && (seq < SEQ_MM0);
    wire        is_mm   = (seq >= SEQ_MM0) && (seq < SEQ_HDR);
    wire [6:0]  pt_burst = seq - SEQ_PT0;                // point burst number
    wire [6:0]  mm_burst = seq - SEQ_MM0;                // mm burst number
    wire [4:0]  pidx     = is_pt ? pt_burst[4:0] : mm_burst[4:0];

    // ---- points: the divider on the record unload path -------------------------
    //
    // Records go into the burst buffer as words 0..3 of a column: sum_i, sum_iv,
    // {flags, i_plus, i_minus, max_i}, {n, argmax}. The first three are captured into
    // registers, on the fourth the column goes into the divider (once every 4 cycles
    // against the divider's rate of one point per cycle). The quotient lands in the
    // point memory by column number 16 cycles later, and the point bursts are
    // unloaded after all records - by then the last column has long been divided (a
    // record burst on the bus takes tens of cycles).
    reg  [31:0] r_si, r_siv, r_w2;
    reg         dv_in;
    reg  [34:0] dv_num;
    reg  [19:0] dv_den;
    reg  [26:0] dv_tag;                                   // {q, i, column}
    wire        dv_out;
    wire [14:0] dv_q;
    wire [26:0] dv_otag;
    wire [15:0] col_w  = {5'd0, seq, wcnt[5:2]};         // column of the word being written into the buffer
    wire [31:0] rec3   = rec_word;                        // {n, argmax}
    wire        no_ln  = (r_si < {16'd0, msum_f}) || (r_si == 32'd0);
    wire [7:0]  q_in   = Q_UNCALIB
                       | (no_ln                      ? Q_NO_LINE   : 8'h00)
                       | ((r_w2[7:0] == 8'hFF)       ? Q_SATURATED : 8'h00)
                       | ((rec3[31:16] > wmax_f)     ? Q_WIDE      : 8'h00)
                       | (r_w2[24]                   ? Q_MULTI     : 8'h00)   // SPOT_F_MULTI
                       | (r_w2[26]                   ? Q_PARTIAL   : 8'h00);  // SPOT_F_CLIPPED

    laser_cog_div #(.TAG_W(27)) u_div (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (dv_in),
        .in_num    (dv_num),
        .in_den    (dv_den),
        .in_tag    (dv_tag),
        .out_valid (dv_out),
        .out_q     (dv_q),
        .out_tag   (dv_otag)
    );

    // Point memory: column -> point word. Written by the divider, read by the
    // point bursts; every column of every frame passes the divider before its
    // point burst is read, so no reset is needed.
    reg [31:0] pts [0:COLS-1];
    reg [10:0] pt_ra, pt_rq;
    reg [31:0] pt_q;
    always @(posedge clk) begin
        if (dv_out)
            pts[dv_otag[10:0]] <= {dv_otag[26:19], dv_otag[18:11],
                                   (dv_otag[19] ? 16'd0 : {1'b0, dv_q})};   // no line -> x_sub 0
        pt_q  <= pts[pt_ra];
        pt_rq <= pt_ra;                                   // the column of pt_q
    end

    // ---- mm: the point memory through the table, while the mm bursts fill -------
    wire        mmo_valid, mmo_tag;
    wire [31:0] mmo_mm;
    laser_mm u_mm (
        .clk       (clk),
        .rst_n     (rst_n),
        .tw_en     (tw_en_q && mm_load_ack),
        .tw_bank   (tw_bank_q),
        .tw_addr   (tw_addr_q),
        .tw_data   (tw_data_q),
        .cfg_en    (mmb_en),
        .cfg_vofs  (mmb_vofs),
        .cfg_kofs  (mmb_kofs),
        .cfg_rows  (mmb_rows),
        .in_valid  (pt_v1 && is_mm),
        .in_px     (pt_q),
        .in_col    (pt_rq),
        .in_tag    (pt_ok1),
        .out_valid (mmo_valid),
        .out_mm    (mmo_mm),
        .out_tag   (mmo_tag)
    );

    wire start_unload = frame_close && en_f && !busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mm_load_ack <= 1'b0;
            mm_bad      <= 1'b0;
            mmb_en      <= 1'b0;
            mmb_bad     <= 1'b0;
        end else begin
            // The acknowledge rises only between unloads and holds while the request
            // does; an unload started meanwhile takes no mm (mmb_en below).
            if (!mm_req)                       mm_load_ack <= 1'b0;
            else if (!busy && !start_unload)   mm_load_ack <= 1'b1;
            if (tw_en_q && !mm_load_ack)       mm_bad <= 1'b1;
            else if (mm_req && !mm_load_ack)   mm_bad <= 1'b0;   // a new load starts clean
            if (start_unload) begin
                mmb_en  <= mm_valid && !mm_req && !mm_bad;
                mmb_bad <= mm_bad;
            end
        end
    end
    always @(posedge clk) begin
        if (start_unload) begin
            mmb_vofs <= mmc[15:0];
            mmb_kofs <= mmc[27:16];
            mmb_rows <= mmc[32:28];
            mmb_id   <= mmc[95:64];
            mmb_crc  <= mmc[127:96];
        end
    end

    // CRCs of exactly the words that go into the point / mm bursts of real columns.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_pt <= 32'hFFFF_FFFF;
            crc_mm <= 32'hFFFF_FFFF;
        end else if (start_unload) begin
            crc_pt <= 32'hFFFF_FFFF;
            crc_mm <= 32'hFFFF_FFFF;
        end else if (filling) begin
            if (is_pt && pt_v1 && pt_ok1)        crc_pt <= crc32_word(crc_pt, pt_q);
            if (is_mm && mmo_valid && mmo_tag)   crc_mm <= crc32_word(crc_mm, mmo_mm);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) dv_in <= 1'b0;
        else        dv_in <= filling && is_rec && rec_valid && ok_d[5] && (wcnt[1:0] == 2'd3);
    end
    always @(posedge clk) begin
        if (filling && is_rec && rec_valid && ok_d[5]) begin
            if (wcnt[1:0] == 2'd0) r_si  <= rec_word;
            if (wcnt[1:0] == 2'd1) r_siv <= rec_word;
            if (wcnt[1:0] == 2'd2) r_w2  <= rec_word;
        end
        // 16*sum_iv + sum_i/2: sum_iv < 2^30, sum_i < 2^19 (header of laser_cog_div.v).
        dv_num <= {r_siv[30:0], 4'd0} + {16'd0, r_si[19:1]};
        dv_den <= r_si[19:0];
        dv_tag <= {q_in, r_w2[7:0], col_w[10:0]};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy        <= 1'b0;
            seq         <= 7'd0;
            filling     <= 1'b0;
            iss         <= 7'd0;
            wcnt        <= 7'd0;
            ok_d        <= 6'd0;
            no_line_cnt <= 16'd0;
            n_mm        <= 16'd0;
            n_out       <= 16'd0;
            n_node      <= 16'd0;
            start_fill  <= 1'b0;
            prof_ready  <= 1'b0;
            prof_addr   <= 32'd0;
            rec_rd      <= 1'b0;
            rec_col     <= 16'd0;
            rec_part    <= 2'd0;
            wptr        <= 6'd0;
            pbuf_we     <= 1'b0;
            pbuf_wd     <= 32'd0;
            hd_pend     <= 1'b0;
            o_blocks    <= 16'd0;
            o_no_line   <= 16'd0;
            pt_ra       <= 11'd0;
            pt_v0       <= 1'b0;
            pt_v1       <= 1'b0;
            pt_ok0      <= 1'b0;
            pt_ok1      <= 1'b0;
        end else begin
            rec_rd     <= 1'b0;
            pbuf_we    <= 1'b0;
            hd_pend    <= 1'b0;
            if (hd_pend) begin
                pbuf_we <= 1'b1;
                pbuf_wd <= (seq == SEQ_HDR) ? hdr_q : foot_q;
            end
            start_fill <= 1'b0;
            ok_d       <= {ok_d[4:0], 1'b0};
            pt_v0      <= 1'b0;
            pt_v1      <= pt_v0;
            pt_ok1     <= pt_ok0;

            // Frame closed: a block - if it was enabled when the frame was opened.
            if (start_unload) begin
                busy        <= 1'b1;
                seq         <= 7'd0;
                no_line_cnt <= 16'd0;
                n_mm        <= 16'd0;
                n_out       <= 16'd0;
                n_node      <= 16'd0;
                start_fill  <= 1'b1;
            end

            // Burst sent: the next one or the end of the block.
            if (prof_done && busy) begin
                prof_ready <= 1'b0;
                if (seq == SEQ_FOOT) begin
                    busy      <= 1'b0;
                    o_blocks  <= o_blocks + 16'd1;
                    o_no_line <= no_line_cnt;
                end else begin
                    seq        <= seq + 7'd1;
                    start_fill <= 1'b1;
                end
            end

            if (start_fill) begin
                filling   <= 1'b1;
                iss       <= 7'd0;
                wcnt      <= 7'd0;
                wptr      <= 6'd0;
                prof_addr <= is_rec           ? blk_base + {17'd0, seq + 7'd1, 8'd0} :
                             is_pt            ? blk_base + PTS_OFF + {17'd0, pt_burst, 8'd0} :
                             is_mm            ? blk_base + MM_OFF + {17'd0, mm_burst, 8'd0} :
                             (seq == SEQ_HDR) ? blk_base :
                                                blk_base + FOOT_OFF;
            end else if (filling) begin
                if (is_rec) begin
                    // Records: 64 reads in a row (16 columns x 4 words). rec_valid
                    // arrives 5 cycles after the cycle in which rec_rd is high,
                    // i.e. 6 after the decision here - hence ok_d[5].
                    if (!iss[6]) begin
                        rec_rd   <= 1'b1;
                        rec_col  <= col_now;
                        rec_part <= iss[1:0];
                        ok_d[0]  <= ({16'd0, col_now} < COLS);
                        iss      <= iss + 7'd1;
                    end
                    if (rec_valid) begin
                        pbuf_we <= 1'b1;
                        pbuf_wd <= ok_d[5] ? rec_word : 32'd0;
                        if (ok_d[5] && (wcnt[1:0] == 2'd3) && (rec_word[31:16] == 16'd0))
                            no_line_cnt <= no_line_cnt + 16'd1;
                        wcnt    <= wcnt + 7'd1;
                    end
                end else if (is_pt || is_mm) begin
                    // Points: 64 point memory reads in a row, the word is ready 2 cycles
                    // later. mm: the same reads through laser_mm, 9 cycles more.
                    if (!iss[6]) begin
                        pt_ra  <= {pidx, iss[5:0]};
                        pt_v0  <= 1'b1;
                        pt_ok0 <= ({21'd0, pidx, iss[5:0]} < COLS);
                        iss    <= iss + 7'd1;
                    end
                    if (is_pt && pt_v1) begin
                        pbuf_we <= 1'b1;
                        pbuf_wd <= pt_ok1 ? pt_q : 32'd0;
                        wcnt    <= wcnt + 7'd1;
                    end
                    if (is_mm && mmo_valid) begin
                        pbuf_we <= 1'b1;
                        pbuf_wd <= mmo_tag ? mmo_mm : 32'd0;
                        wcnt    <= wcnt + 7'd1;
                        if (mmo_tag) begin
                            if (mmo_mm[15:0] != 16'h8000)        n_mm   <= n_mm + 16'd1;
                            else if (mmo_mm[18:16] == 3'd3)      n_out  <= n_out + 16'd1;
                            else if (mmo_mm[18:16] == 3'd4)      n_node <= n_node + 16'd1;
                        end
                    end
                end else if (!iss[6]) begin
                    hd_pend <= 1'b1;                    // hdr_q / foot_q hold this index's word next clock
                    iss     <= iss + 7'd1;
                    wcnt    <= wcnt + 7'd1;
                end
                if (pbuf_we) wptr <= wptr + 6'd1;
                if (wcnt[6] && !pbuf_we && !hd_pend) begin
                    filling    <= 1'b0;
                    prof_ready <= 1'b1;
                end
            end
        end
    end

endmodule
