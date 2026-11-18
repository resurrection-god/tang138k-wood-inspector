// ============================================================================
// Module      : l4_axi_wr
// Lane 4 write port -> AXI4 write master (docs/ARCHITECTURE.md § 4.1; out of
// the main build since stage 5, ERRORS 1.24). It plays the Shared DDR3 bridge's
// Lane 4 slave for one cam_channel
// writer, so the writer does not change: wr_go for 2 cycles with wr_addr and
// burstcount, one idle cycle, wr_en for exactly burstcount cycles carrying one
// 32-bit word each in wr_data[31:0], then the writer waits for wr_done; a new
// burst starts only while wr_wait is low (the Lane 4 contract, CLAUDE.md
// invariant 3).
//
// Lane 4 has no back-pressure inside a burst, so a burst is captured whole into
// one of two buffers and drained to AXI from there:
//
//   * Capture packs the 32-bit words into DW-bit beats by address: word k of a
//     burst at byte address A goes to lane ((A[4:2] + k) mod 8) of beat
//     ((A[4:2] + k) / 8). Lanes no word reached stay masked (wstrb 0), so any
//     word-aligned start and any length are written exactly.
//   * wr_done is pulsed DONE_LAT cycles after the last word is CAPTURED, not
//     after DDR3 answers - the same meaning the vendor bridge gives it ("the
//     bridge accepted the burst", lane5_reader.v). The writer may then start
//     the next burst into the other buffer while this one drains.
//   * wr_wait is high while the buffer the next burst would use is still
//     draining.
//   * Draining is in capture order, so DDR3 sees the writer's bursts in the
//     order it issued them. AXI bursts are INCR, full width (AxSIZE = beat),
//     and are split so that none crosses a 4 KB boundary.
// ============================================================================

`timescale 1ns / 1ps

module l4_axi_wr #(
    parameter integer DW       = 256,
    parameter integer AW       = 30,
    parameter integer MAXB_LOG2 = 6,    // beats per buffer = 2^MAXB_LOG2 (>= 33)
    parameter integer DONE_LAT = 2      // cycles from the last word to wr_done
)(
    input  wire             clk,
    input  wire             rst_n,

    // ---- Lane 4 slave side (towards cam_channel) ----------------------------
    input  wire             wr_go,
    input  wire [7:0]       burstcount,
    input  wire [31:0]      wr_addr,
    input  wire             wr_en,
    input  wire [3:0]       wr_mask,
    input  wire [63:0]      wr_data,
    output reg              wr_done,
    output wire             wr_wait,

    // ---- AXI4 write master ---------------------------------------------------
    output reg              m_awvalid,
    input  wire             m_awready,
    output wire [AW-1:0]    m_awaddr,
    output wire [7:0]       m_awlen,
    output wire             m_wvalid,
    input  wire             m_wready,
    output wire [DW-1:0]    m_wdata,
    output wire [DW/8-1:0]  m_wstrb,
    output wire             m_wlast,
    input  wire             m_bvalid,
    output wire             m_bready,
    input  wire [1:0]       m_bresp,

    // ---- diagnostics ---------------------------------------------------------
    output reg  [15:0]      o_err_proto,    // go while waiting, stray wr_en, odd address, mask
    output reg  [15:0]      o_err_bresp,    // non-OKAY write responses
    output reg  [31:0]      o_bursts        // Lane 4 bursts captured
);

    localparam integer WPB  = DW / 32;              // 32-bit words per beat (8)
    localparam integer LG   = 3;                    // log2(WPB)
    localparam integer MAXB = 1 << MAXB_LOG2;

    // ---- buffer RAM: {word-valid mask, data} per beat, two buffers ------------
    reg  [WPB+DW-1:0]   mem [0:2*MAXB-1];
    reg  [WPB+DW-1:0]   ram_q;

    reg  [1:0]          b_full;                     // buffer holds a burst not yet drained
    reg  [AW-1:0]       b_addr  [0:1];              // first beat's byte address (beat-aligned)
    reg  [MAXB_LOG2:0]  b_beats [0:1];

    // =========================================================================
    // Capture
    // =========================================================================
    localparam C_IDLE = 2'd0, C_DATA = 2'd1, C_DONE = 2'd2;

    reg  [1:0]          c_state;
    reg                 cw;                         // buffer the next/current burst goes to
    reg                 c_drop;                     // burst arrived on a busy buffer: discard
    reg  [7:0]          c_total;
    reg  [7:0]          c_cnt;
    reg  [LG-1:0]       c_lane;
    reg  [MAXB_LOG2-1:0] c_beat;
    reg  [DW-1:0]       c_data;
    reg  [WPB-1:0]      c_mask;
    reg  [AW-1:0]       c_addr;
    reg  [3:0]          c_lat;
    reg                 go_q;

    assign wr_wait = b_full[cw];

    wire [DW-1:0]  c_data_n = c_data | ({{(DW-32){1'b0}}, wr_data[31:0]} << (c_lane * 32));
    wire [WPB-1:0] c_mask_n = c_mask | ({{(WPB-1){1'b0}}, 1'b1} << c_lane);
    wire           c_last   = (c_cnt == c_total - 8'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_state     <= C_IDLE;
            cw          <= 1'b0;
            c_drop      <= 1'b0;
            c_total     <= 8'd0;
            c_cnt       <= 8'd0;
            c_lane      <= {LG{1'b0}};
            c_beat      <= {MAXB_LOG2{1'b0}};
            c_data      <= {DW{1'b0}};
            c_mask      <= {WPB{1'b0}};
            c_addr      <= {AW{1'b0}};
            c_lat       <= 4'd0;
            go_q        <= 1'b0;
            wr_done     <= 1'b0;
            o_err_proto <= 16'd0;
            o_bursts    <= 32'd0;
            b_addr[0]   <= {AW{1'b0}};
            b_addr[1]   <= {AW{1'b0}};
            b_beats[0]  <= {(MAXB_LOG2+1){1'b0}};
            b_beats[1]  <= {(MAXB_LOG2+1){1'b0}};
        end else begin
            wr_done <= 1'b0;
            go_q    <= wr_go;

            case (c_state)
                C_IDLE: begin
                    if (wr_go && !go_q) begin
                        c_total <= burstcount;
                        c_cnt   <= 8'd0;
                        c_lane  <= wr_addr[LG+1:2];
                        c_beat  <= {MAXB_LOG2{1'b0}};
                        c_data  <= {DW{1'b0}};
                        c_mask  <= {WPB{1'b0}};
                        c_addr  <= {wr_addr[AW-1:LG+2], {(LG+2){1'b0}}};
                        c_drop  <= b_full[cw] || (burstcount == 8'd0);
                        if ((b_full[cw] || wr_addr[1:0] != 2'b00 || burstcount == 8'd0) &&
                            o_err_proto != 16'hFFFF)
                            o_err_proto <= o_err_proto + 16'd1;
                        c_state <= (burstcount == 8'd0) ? C_IDLE : C_DATA;
                    end else if (wr_en && o_err_proto != 16'hFFFF) begin
                        o_err_proto <= o_err_proto + 16'd1;     // data outside a burst
                    end
                end

                C_DATA: begin
                    if (wr_en) begin
                        if (wr_mask != 4'b0000 && o_err_proto != 16'hFFFF)
                            o_err_proto <= o_err_proto + 16'd1;
                        c_cnt  <= c_cnt + 8'd1;
                        c_lane <= c_lane + 1'b1;
                        if (&c_lane || c_last) begin
                            // beat complete: the RAM write is in the block below
                            c_beat <= c_beat + 1'b1;
                            c_data <= {DW{1'b0}};
                            c_mask <= {WPB{1'b0}};
                        end else begin
                            c_data <= c_data_n;
                            c_mask <= c_mask_n;
                        end
                        if (c_last) begin
                            if (!c_drop) begin
                                b_addr[cw]  <= c_addr;
                                b_beats[cw] <= {1'b0, c_beat} + 1'b1;
                                o_bursts    <= o_bursts + 32'd1;
                            end
                            c_lat   <= 4'd0;
                            c_state <= C_DONE;
                        end
                    end
                end

                C_DONE: begin
                    if (c_lat + 4'd1 >= DONE_LAT[3:0]) begin
                        wr_done <= 1'b1;
                        if (!c_drop) cw <= ~cw;
                        c_state <= C_IDLE;
                    end else begin
                        c_lat <= c_lat + 4'd1;
                    end
                end

                default: c_state <= C_IDLE;
            endcase
        end
    end

    // RAM write: one beat whenever a lane-7 word or the burst's last word lands.
    always @(posedge clk) begin
        if (c_state == C_DATA && wr_en && !c_drop && (&c_lane || c_last))
            mem[{cw, c_beat}] <= {c_mask_n, c_data_n};
    end

    // =========================================================================
    // Drain
    // =========================================================================
    localparam D_IDLE = 2'd0, D_AW = 2'd1, D_W = 2'd2;

    // Within one AXI burst the beats stream one per cycle through two stages:
    // the RAM's registered output (q) and the W output register (o). A stage
    // takes a new beat only when it is empty or being emptied in the same cycle,
    // so WDATA never changes under a WVALID that has not been accepted.
    reg  [1:0]          d_state;
    reg                 dr;                         // buffer being drained
    reg  [AW-1:0]       d_addr;                     // byte address of the next AXI burst
    reg  [MAXB_LOG2:0]  d_left;                     // beats of the buffer not yet in an AXI burst
    reg  [MAXB_LOG2-1:0] f_beat;                    // next beat to fetch from the RAM
    reg  [8:0]          d_len;                      // beats in the current AXI burst
    reg  [8:0]          f_cnt;                      // beats of it fetched so far
    reg                 q_vld, q_last;              // ram_q holds a fetched beat
    reg                 o_vld, o_last;              // the W register holds a beat
    reg  [WPB+DW-1:0]   o_reg;

    // Beats from d_addr to the next 4 KB boundary.
    wire [8:0] to_4k  = 9'd128 - {2'b00, d_addr[11:LG+2]};
    wire [8:0] chunk  = ({{(9-MAXB_LOG2-1){1'b0}}, d_left} < to_4k) ?
                        {{(9-MAXB_LOG2-1){1'b0}}, d_left} : to_4k;

    wire w_take  = o_vld && m_wready;
    wire o_free  = !o_vld || w_take;
    wire q_move  = q_vld && o_free;
    wire q_free  = !q_vld || q_move;
    wire f_issue = (d_state == D_W) && (f_cnt < d_len) && q_free;

    always @(posedge clk) if (f_issue) ram_q <= mem[{dr, f_beat}];

    assign m_awaddr = d_addr;
    assign m_awlen  = d_len[7:0] - 8'd1;
    assign m_wvalid = o_vld;
    assign m_wdata  = o_reg[DW-1:0];
    assign m_wlast  = o_last;
    assign m_bready = 1'b1;

    genvar g;
    generate
        for (g = 0; g < WPB; g = g + 1) begin : g_strb
            assign m_wstrb[g*4 +: 4] = {4{o_reg[DW + g]}};
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_state   <= D_IDLE;
            dr        <= 1'b0;
            d_addr    <= {AW{1'b0}};
            d_left    <= {(MAXB_LOG2+1){1'b0}};
            f_beat    <= {MAXB_LOG2{1'b0}};
            d_len     <= 9'd0;
            f_cnt     <= 9'd0;
            q_vld     <= 1'b0;
            q_last    <= 1'b0;
            o_vld     <= 1'b0;
            o_last    <= 1'b0;
            o_reg     <= {(WPB+DW){1'b0}};
            m_awvalid <= 1'b0;
            b_full    <= 2'b00;
        end else begin
            // A burst finishing capture marks its buffer full (same edge as wr_done).
            if (c_state == C_DONE && c_lat + 4'd1 >= DONE_LAT[3:0] && !c_drop)
                b_full[cw] <= 1'b1;

            // ---- the two W stages ------------------------------------------
            if (q_move) begin
                o_reg  <= ram_q;
                o_last <= q_last;
                o_vld  <= 1'b1;
            end else if (w_take) begin
                o_vld  <= 1'b0;
            end
            if (f_issue) begin
                q_vld  <= 1'b1;
                q_last <= (f_cnt == d_len - 9'd1);
                f_beat <= f_beat + 1'b1;
                f_cnt  <= f_cnt + 9'd1;
            end else if (q_move) begin
                q_vld  <= 1'b0;
            end

            case (d_state)
                D_IDLE: begin
                    if (b_full[dr]) begin
                        d_addr  <= b_addr[dr];
                        d_left  <= b_beats[dr];
                        f_beat  <= {MAXB_LOG2{1'b0}};
                        d_state <= D_AW;
                    end
                end

                D_AW: begin
                    if (!m_awvalid) begin
                        d_len     <= chunk;
                        f_cnt     <= 9'd0;
                        m_awvalid <= 1'b1;
                    end else if (m_awready) begin
                        m_awvalid <= 1'b0;
                        d_addr    <= d_addr + {{(AW-9-LG-2){1'b0}}, d_len, {(LG+2){1'b0}}};
                        d_left    <= d_left - d_len[MAXB_LOG2:0];
                        d_state   <= D_W;
                    end
                end

                D_W: begin
                    if (w_take && o_last) begin
                        if (d_left == 0) begin
                            b_full[dr] <= 1'b0;
                            dr         <= ~dr;
                            d_state    <= D_IDLE;
                        end else begin
                            d_state    <= D_AW;
                        end
                    end
                end

                default: d_state <= D_IDLE;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            o_err_bresp <= 16'd0;
        else if (m_bvalid && m_bresp != 2'b00 && o_err_bresp != 16'hFFFF)
            o_err_bresp <= o_err_bresp + 16'd1;
    end

endmodule
