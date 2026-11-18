// laser_col_extract.v - streaming laser-line search, one point per sensor COLUMN.
//
// Geometry (decision of 23.09.2026, docs/DECISIONS.md R-15, docs/ARCHITECTURE.md
// § 6.1): the laser line lies ALONG sensor rows, so the profile has one
// point per column (1080 points) and the search runs ACROSS rows. Rows arrive one
// at a time, four pixels per clock; each column keeps its own accumulators in
// block RAM and nothing of the frame is stored.
//
// This module is the bit-exact twin of tools/laser/model.py extract_raw(). The
// rules below are copied from there, and any difference shows up as a mismatch
// in the raw sums - which is the acceptance test (fpga/sim/laser_check.py):
//
//   * a pixel joins a spot when I >= thresh; below threshold it adds nothing and
//     only extends the gap counter;
//   * a spot closes after more than `gap` consecutive below-threshold pixels,
//     and at the end of the window;
//   * a new maximum must be strictly greater (ties keep the first);
//     i_minus is the pixel before the new maximum (0 on the first window row),
//     i_plus the pixel right after it, whatever its value (0 at the last row);
//   * the best spot is the one with strictly greater sum (ties keep the first);
//   * sums accumulate (I - offset), offset <= thresh; max/neighbours are raw I;
//   * flags: 0x01 more than one spot, 0x02 max == 255, 0x04 best spot was cut by
//     the end of the window.
//
// Pipeline, per pixel word (k = column / 4):
//   S0  word in, state read issued for address k; RAM output registered (qr);
//       meanwhile everything stateless: threshold, I - offset, (I - offset)*row
//   S1  state out of qr; state-dependent compares (new max, gap, best spot)
//   S2  add stage (sums, best-spot swap), result registered
//   S3  registered state written back to address k
// Address k is read again only on the next row, ROW_WORDS words later, so the
// read-modify-write never meets itself as long as a row has at least 5 words.
//
// Frames are separated by an epoch bit instead of a clearing pass: frame_start
// flips `epoch`, and a state word whose epoch differs reads as all zeros. The
// epoch lives in its own 1-bit-per-address array (ep_mem), not in the wide
// word: all four columns of an address are written together, and a separate
// small array gives the "stale" flag from a register one clock early instead
// of out of 46 BSRAMs (timing).
//
// Record readout (between frames only - it shares the RAM read port): present
// rec_col/rec_part, get rec_word five clocks later (rec_valid marks it). An open spot is closed on the
// way out, so no finalising pass is needed either. Record layout is
// tools/laser/laser_block.py REC ("<IIBBBBHH"), one 32-bit word per rec_part:
//   0 sum_i   1 sum_iv   2 {flags, i_plus, i_minus, max_i}   3 {n, argmax}
//
// Everything here runs on lane4_clk. No comparator of this module reaches the
// writer FSM's combinational cone (that cone has no timing margin;
// docs/WORKFLOWS.md § 3).

module laser_col_extract #(
    parameter integer COLS      = 1080,     // multiple of 4
    parameter integer ROW_BITS  = 11        // row index width (1280 rows)
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Search parameters. Sampled on frame_start and held for the frame.
    input  wire [7:0]           cfg_thresh,
    input  wire [3:0]           cfg_gap,
    input  wire [7:0]           cfg_offset,     // must be <= cfg_thresh
    input  wire [ROW_BITS-1:0]  cfg_win_lo,
    input  wire [ROW_BITS-1:0]  cfg_win_hi,     // inclusive

    // Pixel stream: one word = four pixels of one row, byte 0 = column 4k.
    input  wire                 frame_start,    // pulse before the first word
    input  wire                 pix_valid,
    input  wire [31:0]          pix_word,
    input  wire [ROW_BITS-1:0]  pix_row,
    input  wire [15:0]          pix_k,          // word index in the row

    // Record readout.
    input  wire                 rec_rd,
    input  wire [15:0]          rec_col,
    input  wire [1:0]           rec_part,
    output reg  [31:0]          rec_word,
    output reg                  rec_valid,

    output reg  [19:0]          stat_words      // pixel words taken this frame (saturates)
);

    localparam integer WORDS = COLS / 4;
    localparam integer AW    = (WORDS <= 2)    ? 1 :
                               (WORDS <= 4)    ? 2 :
                               (WORDS <= 8)    ? 3 :
                               (WORDS <= 16)   ? 4 :
                               (WORDS <= 32)   ? 5 :
                               (WORDS <= 64)   ? 6 :
                               (WORDS <= 128)  ? 7 :
                               (WORDS <= 256)  ? 8 :
                               (WORDS <= 512)  ? 9 : 10;

    // ---- one column's state word -------------------------------------------
    localparam integer SUM_W  = 19;     // 255 * 1280 = 326400 < 2^19
    localparam integer SUMV_W = 28;     // 255 * sum(0..1279) = 208.7M < 2^28
    localparam integer N_W    = 11;

    localparam integer F_OPEN   = 0;
    localparam integer F_GAP    = 1;                      // 5 bits
    localparam integer F_PREV   = 6;                      // 8
    localparam integer F_IPPEND = 14;
    localparam integer F_CSUM   = 15;                     // cur
    localparam integer F_CSUMV  = F_CSUM  + SUM_W;
    localparam integer F_CN     = F_CSUMV + SUMV_W;
    localparam integer F_CMAX   = F_CN    + N_W;
    localparam integer F_CARG   = F_CMAX  + 8;
    localparam integer F_CIM    = F_CARG  + ROW_BITS;
    localparam integer F_CIP    = F_CIM   + 8;
    localparam integer F_BSUM   = F_CIP   + 8;            // best
    localparam integer F_BSUMV  = F_BSUM  + SUM_W;
    localparam integer F_BN     = F_BSUMV + SUMV_W;
    localparam integer F_BMAX   = F_BN    + N_W;
    localparam integer F_BARG   = F_BMAX  + 8;
    localparam integer F_BIM    = F_BARG  + ROW_BITS;
    localparam integer F_BIP    = F_BIM   + 8;
    localparam integer F_BCLIP  = F_BIP   + 8;
    localparam integer F_SPOTS  = F_BCLIP + 1;            // 2 bits, saturating
    localparam integer LANE_W   = F_SPOTS + 2;
    localparam integer CUR_W    = F_BSUM - F_CSUM;        // cur block, same layout as best

    // ---- RAM: WORDS x (4 * LANE_W), synchronous read and write -------------
    // The RAM output is registered once more (qr) before any logic sees it:
    // BSRAM clock-to-out plus the lane select was the critical path (87 MHz).
    // The write therefore lands four clocks after the read of the same address
    // (qr, S1, S2, then the registered write), so address k may be read again
    // no sooner than five words later: rows of fewer than 5 words are not
    // supported.
    reg  [4*LANE_W-1:0] mem [0:WORDS-1];
    reg  [4*LANE_W-1:0] q, qr;
    reg  [AW-1:0]       rd_addr;
    reg                 wr_en;
    reg  [AW-1:0]       wr_addr;
    reg  [4*LANE_W-1:0] wr_data;       // combinational, from S2
    reg  [4*LANE_W-1:0] wr_data_q;     // what the RAM actually writes

    always @(posedge clk) begin
        if (wr_en) mem[wr_addr] <= wr_data_q;
        q  <= mem[rd_addr];
        qr <= q;
    end

    // Epoch per address, same read/write timing as mem; stale_r travels with qr.
    reg              ep_mem [0:WORDS-1];
    reg              ep_q, stale_r;
    reg              epoch;
    always @(posedge clk) begin
        if (wr_en) ep_mem[wr_addr] <= epoch;
        ep_q    <= ep_mem[rd_addr];
        stale_r <= (ep_q != epoch);
    end

    // Gowin BSRAM powers up zeroed after configuration; the epoch test relies on
    // that only for the very first frame. Simulation starts from X instead, so
    // give it the same zeros.
    // synthesis translate_off
    integer mi;
    initial for (mi = 0; mi < WORDS; mi = mi + 1) begin
        mem[mi]    = {4*LANE_W{1'b0}};
        ep_mem[mi] = 1'b0;
    end
    // synthesis translate_on

    // ---- frame parameters and epoch -----------------------------------------
    reg [7:0]           thr, off;
    reg [4:0]           gapmax;
    reg [ROW_BITS-1:0]  lo, hi;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            epoch  <= 1'b0;
            thr    <= 8'd40;
            off    <= 8'd0;
            gapmax <= 5'd2;
            lo     <= {ROW_BITS{1'b0}};
            hi     <= {ROW_BITS{1'b1}};
        end else if (frame_start) begin
            epoch  <= ~epoch;
            thr    <= cfg_thresh;
            off    <= (cfg_offset > cfg_thresh) ? cfg_thresh : cfg_offset;
            gapmax <= {1'b0, cfg_gap};
            lo     <= cfg_win_lo;
            hi     <= cfg_win_hi;
        end
    end

    // ---- S0: accept a word ---------------------------------------------------
    // Everything about a pixel that does not depend on the column's state is
    // done here, while the RAM read is in flight, so S1 is left with the
    // compares that need the state:
    //   input -> s0  threshold, I - offset, "pixel is non-zero", k < WORDS
    //   s0 -> s1     the (I - offset) * row product (a whole clock for the
    //                multiplier) and the row-window test
    // A word outside the window still reads its RAM address; s1_v drops it.
    // s0_* travel with q, s1_* with qr (one clock later).
    reg                s0_v, s1_v, s0_kok;
    reg [31:0]         s0_px, s1_px;
    reg [ROW_BITS-1:0] s0_row, s1_row;
    reg [AW-1:0]       s0_k, s1_k;
    reg [3:0]          s0_above, s0_pxnz, s1_above, s1_pxnz;
    reg [4*8-1:0]      s0_pxo, s1_pxo;
    reg [4*SUMV_W-1:0] s1_prod;

    integer            l0;
    reg [7:0]          px0;
    reg [3:0]          i_above, i_pxnz;
    reg [4*8-1:0]      i_pxo;
    reg [4*SUMV_W-1:0] n0_prod;

    always @(*) begin
        i_above = 4'd0; i_pxnz = 4'd0; i_pxo = 32'd0; px0 = 8'd0;
        for (l0 = 0; l0 < 4; l0 = l0 + 1) begin
            px0 = pix_word[l0*8 +: 8];
            i_above[l0]      = (px0 >= thr);
            i_pxnz[l0]       = (px0 != 8'd0);
            i_pxo[l0*8 +: 8] = px0 - off;
        end
    end

    always @(*) begin
        n0_prod = {4*SUMV_W{1'b0}};
        for (l0 = 0; l0 < 4; l0 = l0 + 1)
            if (s0_above[l0]) n0_prod[l0*SUMV_W +: SUMV_W] = s0_pxo[l0*8 +: 8] * s0_row;
    end

    wire s0_in_win = (s0_row >= lo) && (s0_row <= hi) && s0_kok;

    // Readout path shares the read port; it only runs between frames.
    reg                r0_v, r1_v;
    reg [1:0]          r0_lane, r1_lane, r0_part, r1_part;

    // The address mux looks at pix_valid only: no compare on the RAM address
    // path. Readout runs only between frames, when no pixel word is valid.
    always @(*) begin
        if (pix_valid) rd_addr = pix_k[AW-1:0];
        else           rd_addr = rec_col[AW+1:2];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_v <= 1'b0; s0_kok <= 1'b0; s0_px <= 32'd0; s0_row <= {ROW_BITS{1'b0}}; s0_k <= {AW{1'b0}};
            s0_above <= 4'd0; s0_pxnz <= 4'd0; s0_pxo <= 32'd0;
            s1_v <= 1'b0; s1_px <= 32'd0; s1_row <= {ROW_BITS{1'b0}}; s1_k <= {AW{1'b0}};
            s1_above <= 4'd0; s1_pxnz <= 4'd0; s1_pxo <= 32'd0; s1_prod <= {4*SUMV_W{1'b0}};
            r0_v <= 1'b0; r0_lane <= 2'd0; r0_part <= 2'd0;
            r1_v <= 1'b0; r1_lane <= 2'd0; r1_part <= 2'd0;
            stat_words <= 20'd0;
        end else begin
            s0_v     <= pix_valid;
            s0_kok   <= ({16'd0, pix_k} < WORDS);
            s0_px    <= pix_word;
            s0_row   <= pix_row;
            s0_k     <= pix_k[AW-1:0];
            s0_above <= i_above;
            s0_pxnz  <= i_pxnz;
            s0_pxo   <= i_pxo;
            s1_v     <= s0_v && s0_in_win;
            s1_px    <= s0_px;
            s1_row   <= s0_row;
            s1_k     <= s0_k;
            s1_above <= s0_above;
            s1_pxnz  <= s0_pxnz;
            s1_pxo   <= s0_pxo;
            s1_prod  <= n0_prod;
            r0_v    <= rec_rd && !pix_valid;
            r0_lane <= rec_col[1:0];
            r0_part <= rec_part;
            r1_v    <= r0_v;
            r1_lane <= r0_lane;
            r1_part <= r0_part;
            if (frame_start)                           stat_words <= 20'd0;
            else if (s1_v && stat_words != 20'hFFFFF)  stat_words <= stat_words + 20'd1;
        end
    end

    // ---- S1: compare stage, four lanes ---------------------------------------
    reg                s2_v;
    reg [AW-1:0]       s2_k;
    reg [ROW_BITS-1:0] s2_row;
    reg [4*LANE_W-1:0] s2_st;          // state, already zeroed if stale
    reg [31:0]         s2_px;
    reg [3:0]          s2_above, s2_newmax, s2_closing, s2_better;
    reg [4*5-1:0]      s2_gap;
    reg [4*8-1:0]      s2_pxo;         // pixel minus offset
    reg [4*SUMV_W-1:0] s2_prod;

    // Combinational compare stage; the clocked block below only registers it.
    integer            l1;
    reg [LANE_W-1:0]   st, raw;
    reg [7:0]          px;
    reg                open, start_c, above_c, stale, gt_raw;
    reg [4:0]          g;
    reg [4*LANE_W-1:0] n_st;
    reg [3:0]          n_newmax, n_closing, n_better;
    reg [4*5-1:0]      n_gap;

    always @(*) begin
        n_st = {4*LANE_W{1'b0}};
        n_newmax = 4'd0; n_closing = 4'd0; n_better = 4'd0; n_gap = 20'd0;
        st = {LANE_W{1'b0}}; raw = {LANE_W{1'b0}}; px = 8'd0;
        open = 1'b0; start_c = 1'b0; above_c = 1'b0; stale = 1'b0; gt_raw = 1'b0; g = 5'd0;
        for (l1 = 0; l1 < 4; l1 = l1 + 1) begin
            raw     = qr[l1*LANE_W +: LANE_W];
            stale   = stale_r;
            st      = stale ? {LANE_W{1'b0}} : raw;
            px      = s1_px[l1*8 +: 8];
            open    = st[F_OPEN];
            above_c = s1_above[l1];
            start_c = above_c && !open;
            // New maximum. On a new spot the running max is 0, so the test is
            // "px > 0"; otherwise it is px against the stored max. The stored
            // max is compared RAW, in parallel with the epoch test, and the
            // epoch only picks the answer (timing: 133.9 MHz with the compare
            // behind the epoch mux).
            gt_raw  = (px > raw[F_CMAX +: 8]);
            g       = above_c ? 5'd0 : (open ? st[F_GAP +: 5] + 5'd1 : 5'd0);
            // The state handed to S2 already has its cur block cleared on a new
            // spot and i_plus inserted, so S2 is only adders (timing: this mux
            // used to sit in front of the S2 adders).
            n_st[l1*LANE_W +: LANE_W] = st;
            if (start_c)
                n_st[l1*LANE_W + F_CSUM +: CUR_W] = {CUR_W{1'b0}};
            else
                n_st[l1*LANE_W + F_CIP +: 8] = (st[F_IPPEND] && open) ? px : st[F_CIP +: 8];
            n_newmax[l1]  = above_c && (start_c ? s1_pxnz[l1] : gt_raw);
            n_gap[l1*5 +: 5] = g;
            n_closing[l1] = !above_c && open && (g > gapmax);
            // "Current spot beats the best" - needed only when the spot closes,
            // and a spot closes only on a below-threshold pixel, which does not
            // touch the sums. So the compare can use the sums as READ and run
            // here, in parallel, instead of after the S2 adders (95 MHz -> ok).
            // Closing implies the state is fresh, so the raw word will do.
            n_better[l1] = (raw[F_CSUM +: SUM_W] > raw[F_BSUM +: SUM_W]);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_v <= 1'b0; s2_k <= {AW{1'b0}}; s2_row <= {ROW_BITS{1'b0}};
            s2_st <= {4*LANE_W{1'b0}}; s2_px <= 32'd0;
            s2_above <= 4'd0; s2_newmax <= 4'd0; s2_closing <= 4'd0;
            s2_better <= 4'd0; s2_gap <= 20'd0; s2_pxo <= 32'd0;
            s2_prod <= {4*SUMV_W{1'b0}};
        end else begin
            s2_v       <= s1_v;
            s2_k       <= s1_k;
            s2_row     <= s1_row;
            s2_px      <= s1_px;
            s2_st      <= n_st;
            s2_above   <= s1_above;
            s2_newmax  <= n_newmax;
            s2_closing <= n_closing;
            s2_better  <= n_better;
            s2_gap     <= n_gap;
            s2_pxo     <= s1_pxo;
            s2_prod    <= s1_prod;
        end
    end

    // ---- S2: add stage and write-back ----------------------------------------
    reg [LANE_W-1:0]   ns, os;
    reg [CUR_W-1:0]    cur;
    reg [SUM_W-1:0]    csum;
    reg [SUMV_W-1:0]   csumv;
    reg [N_W-1:0]      cn;
    reg [7:0]          px2, pxo2;
    reg [1:0]          sp;
    integer            l2;

    always @(*) begin
        wr_data = {4*LANE_W{1'b0}};
        os = {LANE_W{1'b0}}; ns = {LANE_W{1'b0}}; cur = {CUR_W{1'b0}};
        csum = {SUM_W{1'b0}}; csumv = {SUMV_W{1'b0}}; cn = {N_W{1'b0}};
        px2 = 8'd0; pxo2 = 8'd0; sp = 2'd0;
        for (l2 = 0; l2 < 4; l2 = l2 + 1) begin
            os  = s2_st[l2*LANE_W +: LANE_W];
            ns  = os;
            px2 = s2_px[l2*8 +: 8];
            pxo2 = s2_pxo[l2*8 +: 8];
            sp  = os[F_SPOTS +: 2];
            ns[F_PREV +: 8] = px2;
            ns[F_IPPEND] = s2_newmax[l2];
            ns[F_GAP +: 5] = s2_gap[l2*5 +: 5];
            // cur block: cleared on a new spot, i_plus as captured in S1.
            cur = os[F_CSUM +: CUR_W];        // cleared / i_plus set in S1
            csum  = cur[0 +: SUM_W] + {{(SUM_W-8){1'b0}}, pxo2};
            csumv = cur[SUM_W +: SUMV_W] + s2_prod[l2*SUMV_W +: SUMV_W];
            cn    = cur[SUM_W+SUMV_W +: N_W] + {{(N_W-1){1'b0}}, 1'b1};
            if (s2_above[l2]) begin
                cur[0 +: SUM_W]              = csum;
                cur[SUM_W +: SUMV_W]         = csumv;
                cur[SUM_W+SUMV_W +: N_W]     = cn;
                if (s2_newmax[l2]) begin
                    cur[F_CMAX - F_CSUM +: 8]        = px2;
                    cur[F_CARG - F_CSUM +: ROW_BITS] = s2_row;
                    cur[F_CIM  - F_CSUM +: 8]        = os[F_PREV +: 8];
                    cur[F_CIP  - F_CSUM +: 8]        = 8'd0;
                end
                ns[F_OPEN] = 1'b1;
            end
            ns[F_CSUM +: CUR_W] = cur;
            if (s2_closing[l2]) begin
                ns[F_OPEN] = 1'b0;
                ns[F_SPOTS +: 2] = (sp == 2'd3) ? 2'd3 : sp + 2'd1;
                if (s2_better[l2]) begin
                    ns[F_BSUM +: CUR_W] = cur;
                    ns[F_BCLIP]         = 1'b0;
                end
            end
            wr_data[l2*LANE_W +: LANE_W] = ns;
        end
    end

    // Register the new state before the RAM: the adders and the best-spot
    // compare end here, and the BSRAM input sees only a register.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_en   <= 1'b0;
            wr_addr <= {AW{1'b0}};
        end else begin
            wr_en   <= s2_v;
            wr_addr <= s2_k;
        end
    end
    always @(posedge clk) wr_data_q <= wr_data;

    // ---- readout: close the open spot on the way out --------------------------
    reg [LANE_W-1:0] rs;
    reg [SUM_W-1:0]  b_sum;
    reg [SUMV_W-1:0] b_sumv;
    reg [N_W-1:0]    b_n;
    reg [7:0]        b_max, b_im, b_ip;
    reg [ROW_BITS-1:0] b_arg;
    reg              b_clip;
    reg [1:0]        b_spots;
    reg [7:0]        b_flags;
    reg [31:0]       rec_next;
    // Stage RA: pick the column's lane out of the RAM word and apply the epoch.
    // Stage RB: close the open spot and pick the part. Split for timing.
    // RA is itself two registers: the 4:1 lane select alone, then the epoch
    // test. Together they were the critical path (85 MHz): an 820-bit RAM word
    // spread over 46 BSRAMs is mostly routing, and the epoch bit fans out to
    // every bit of the selected lane.
    reg [LANE_W-1:0] rsel, rs_q;
    reg              rsel_v, ra_v, rsel_stale;
    reg [1:0]        rsel_part, ra_part;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rsel <= {LANE_W{1'b0}}; rsel_v <= 1'b0; rsel_part <= 2'd0; rsel_stale <= 1'b0;
            rs_q <= {LANE_W{1'b0}}; ra_v <= 1'b0; ra_part <= 2'd0;
        end else begin
            rsel      <= qr[r1_lane*LANE_W +: LANE_W];
            rsel_v    <= r1_v;
            rsel_part <= r1_part;
            ra_v      <= rsel_v;
            ra_part   <= rsel_part;
            rsel_stale <= stale_r;
            rs_q      <= rsel_stale ? {LANE_W{1'b0}} : rsel;
        end
    end

    always @(*) begin
        rs = rs_q;
        b_sum   = rs[F_BSUM  +: SUM_W];
        b_sumv  = rs[F_BSUMV +: SUMV_W];
        b_n     = rs[F_BN    +: N_W];
        b_max   = rs[F_BMAX  +: 8];
        b_arg   = rs[F_BARG  +: ROW_BITS];
        b_im    = rs[F_BIM   +: 8];
        b_ip    = rs[F_BIP   +: 8];
        b_clip  = rs[F_BCLIP];
        b_spots = rs[F_SPOTS +: 2];
        if (rs[F_OPEN]) begin
            b_spots = (b_spots == 2'd3) ? 2'd3 : b_spots + 2'd1;
            if (rs[F_CSUM +: SUM_W] > b_sum) begin
                b_sum  = rs[F_CSUM  +: SUM_W];
                b_sumv = rs[F_CSUMV +: SUMV_W];
                b_n    = rs[F_CN    +: N_W];
                b_max  = rs[F_CMAX  +: 8];
                b_arg  = rs[F_CARG  +: ROW_BITS];
                b_im   = rs[F_CIM   +: 8];
                b_ip   = rs[F_CIP   +: 8];
                b_clip = 1'b1;
            end
        end
        b_flags = {5'd0, b_clip, (b_max == 8'd255), (b_spots > 2'd1)};
        case (ra_part)
            2'd0:    rec_next = {{(32-SUM_W){1'b0}}, b_sum};
            2'd1:    rec_next = {{(32-SUMV_W){1'b0}}, b_sumv};
            2'd2:    rec_next = {b_flags, b_ip, b_im, b_max};
            default: rec_next = {{(16-N_W){1'b0}}, b_n, {(16-ROW_BITS){1'b0}}, b_arg};
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rec_word <= 32'd0; rec_valid <= 1'b0;
        end else begin
            rec_valid <= ra_v;
            if (ra_v) rec_word <= rec_next;
        end
    end

endmodule
