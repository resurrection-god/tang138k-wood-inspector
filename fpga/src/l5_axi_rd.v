// ============================================================================
// Module      : l5_axi_rd
// Lane 5 read port -> AXI4 read master (docs/ARCHITECTURE.md § 4.1). It plays
// the Shared DDR3 bridge's Lane 5 slave for lane5_reader, so
// the reader does not change: rd_go for one cycle with rd_addr and
// rd_burstcount, then the reader waits for rd_rdy, raises rd_en, and takes one
// 32-bit word per rd_valid (in rd_data[31:0]) until it has burstcount words.
//
//   * rd_go is a single-cycle pulse and is latched even while a previous read
//     is still being served, so it is never missed.
//   * The read goes out as INCR, full-width AXI bursts from the beat-aligned
//     address, split at 4 KB boundaries; rd_rdy is pulsed when the first AR is
//     accepted. Words before the start address and after the last one are
//     dropped.
//   * lane5_reader gives up on a burst after 2048 cycles and drops rd_en. If
//     rd_en falls before all words were delivered, or a new rd_go arrives
//     meanwhile, the rest of the read is drained from AXI and discarded.
//   * Every Lane 5 signal passes one register each way (09.10.2026): this
//     module sits by axi_mux at the DDR3 IP, lane5_reader by camera 0's D-PHY,
//     and three stage-5 builds in a row failed clk_x1 on logic spread between
//     the two (-0.043 / -0.887 ns, ERRORS 1.24). The reader sees each word one
//     cycle later and its rd_en arrives one cycle later; it reads rd_data only
//     with rd_valid and ignores rd_valid outside its read, so the extra word a
//     late rd_en can let through after a timeout is harmless (the read is then
//     abandoned as before).
// ============================================================================

`timescale 1ns / 1ps

module l5_axi_rd #(
    parameter integer DW = 256,
    parameter integer AW = 30
)(
    input  wire             clk,
    input  wire             rst_n,

    // ---- Lane 5 slave side (towards lane5_reader) ---------------------------
    input  wire             rd_go,
    input  wire [7:0]       rd_burstcount,
    input  wire [31:0]      rd_addr,
    input  wire             rd_en,
    output reg              rd_rdy,
    output reg              rd_valid,
    output reg  [63:0]      rd_data,

    // ---- AXI4 read master ----------------------------------------------------
    output reg              m_arvalid,
    input  wire             m_arready,
    output wire [AW-1:0]    m_araddr,
    output wire [7:0]       m_arlen,
    input  wire             m_rvalid,
    output wire             m_rready,
    input  wire [DW-1:0]    m_rdata,
    input  wire [1:0]       m_rresp,
    input  wire             m_rlast,

    // ---- diagnostics ---------------------------------------------------------
    output reg  [15:0]      o_aborts,       // reads abandoned by the reader
    output reg  [15:0]      o_overrun,      // rd_go while one was already pending
    output reg  [15:0]      o_err_rresp     // non-OKAY read responses
);

    localparam integer WPB = DW / 32;       // words per beat (8)
    localparam integer LG  = 3;             // log2(WPB)

    // ---- pending request -----------------------------------------------------
    reg             p_valid;
    reg  [31:0]     p_addr;
    reg  [7:0]      p_cnt;

    // ---- request in service --------------------------------------------------
    localparam S_IDLE = 2'd0, S_RUN = 2'd1;
    reg  [1:0]      state;
    reg  [AW-1:0]   a_addr;                 // next AR's byte address (beat-aligned)
    reg  [6:0]      a_left;                 // beats not yet requested
    reg  [7:0]      a_len;                  // beats of the AR being presented
    reg  [6:0]      r_left;                 // beats requested but not yet received
    reg  [8:0]      w_left;                 // words still to deliver
    reg             first_ar;               // rd_rdy not yet given
    reg             en_seen;                // rd_en has been high in this request
    reg             abort;

    // ---- Lane 5 port registers (see the header) ------------------------------
    reg             go_q, en_q;             // rd_go, rd_en as this module sees them
    reg  [31:0]     addr_q;
    reg  [7:0]      bc_q;
    reg             rdy_i, valid_i;         // rd_rdy, rd_valid one cycle before the port
    reg  [31:0]     data_i;

    // ---- the beat being unpacked ---------------------------------------------
    reg             hb_valid;
    // Loaded whenever no beat is held (enable = !hb_valid, one flop): the beat
    // accepted with m_rvalid && m_rready is the one that sets hb_valid, and the
    // register then holds it. Its enable used to be the R handshake itself,
    // axi_mux's slice state into 256 flops (-0.475 ns in a stage-5 build).
    reg  [DW-1:0]   hb_data;
    reg  [LG-1:0]   hb_w;                   // next word of the beat
    reg             first_beat;
    reg  [LG-1:0]   off;                    // start word inside the first beat

    wire [7:0]  to_4k   = 8'd128 - {1'b0, a_addr[11:LG+2]};
    wire [7:0]  a_chunk = ({1'b0, a_left} < to_4k) ? {1'b0, a_left} : to_4k;

    assign m_araddr = a_addr;
    assign m_arlen  = a_len - 8'd1;
    assign m_rready = (state == S_RUN) && (abort || !hb_valid);

    wire rd_go_new = go_q;      // one cycle wide by contract

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            go_q     <= 1'b0;
            en_q     <= 1'b0;
            addr_q   <= 32'd0;
            bc_q     <= 8'd0;
            rd_rdy   <= 1'b0;
            rd_valid <= 1'b0;
            rd_data  <= 64'd0;
        end else begin
            go_q     <= rd_go;
            en_q     <= rd_en;
            addr_q   <= rd_addr;
            bc_q     <= rd_burstcount;
            rd_rdy   <= rdy_i;
            rd_valid <= valid_i;
            rd_data  <= {32'd0, data_i};
        end
    end

    always @(posedge clk) if (!hb_valid) hb_data <= m_rdata;

    // Beats covering `cnt` words that start at word `off` of a beat.
    function [6:0] beats_of;
        input [LG-1:0] off;
        input [7:0]    cnt;
        reg   [8:0]    words;
        begin
            words    = {6'd0, off} + {1'b0, cnt} + 9'd7;
            beats_of = {1'b0, words[8:LG]};
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_valid     <= 1'b0;
            p_addr      <= 32'd0;
            p_cnt       <= 8'd0;
            state       <= S_IDLE;
            a_addr      <= {AW{1'b0}};
            a_left      <= 7'd0;
            a_len       <= 8'd0;
            r_left      <= 7'd0;
            w_left      <= 9'd0;
            first_ar    <= 1'b0;
            en_seen     <= 1'b0;
            abort       <= 1'b0;
            hb_valid    <= 1'b0;
            hb_w        <= {LG{1'b0}};
            first_beat  <= 1'b0;
            off         <= {LG{1'b0}};
            m_arvalid   <= 1'b0;
            rdy_i       <= 1'b0;
            valid_i     <= 1'b0;
            data_i      <= 32'd0;
            o_aborts    <= 16'd0;
            o_overrun   <= 16'd0;
            o_err_rresp <= 16'd0;
        end else begin
            rdy_i   <= 1'b0;
            valid_i <= 1'b0;
            // The word has no enable: it follows the beat's current word every
            // cycle and means something only together with rd_valid, which is
            // how lane5_reader and probe_verify read it. Its enable used to be
            // the delivery condition below (state, abort, hb_valid, rd_en from
            // the far-away reader, w_left != 0): two LUT levels into the CE of
            // 32 flops, -0.043 ns on clk_x1 in a stage-5 build (09.10.2026).
            data_i  <= hb_data[hb_w*32 +: 32];

            // latch a request
            if (rd_go_new) begin
                if (p_valid && o_overrun != 16'hFFFF) o_overrun <= o_overrun + 16'd1;
                p_valid <= 1'b1;
                p_addr  <= addr_q;
                p_cnt   <= bc_q;
            end

            case (state)
                S_IDLE: begin
                    if (p_valid && !rd_go_new) begin
                        p_valid    <= 1'b0;
                        a_addr     <= {p_addr[AW-1:LG+2], {(LG+2){1'b0}}};
                        a_left     <= beats_of(p_addr[LG+1:2], p_cnt);
                        r_left     <= 7'd0;
                        w_left     <= {1'b0, p_cnt};
                        off        <= p_addr[LG+1:2];
                        first_ar   <= 1'b1;
                        first_beat <= 1'b1;
                        en_seen    <= 1'b0;
                        abort      <= (p_cnt == 8'd0);
                        hb_valid   <= 1'b0;
                        state      <= S_RUN;
                    end
                end

                S_RUN: begin
                    // ---- issue ARs ------------------------------------------
                    if (!m_arvalid) begin
                        if (a_left != 7'd0 && !abort) begin
                            a_len     <= a_chunk;
                            m_arvalid <= 1'b1;
                        end
                    end else if (m_arready) begin
                        m_arvalid <= 1'b0;
                        a_addr    <= a_addr + {{(AW-8-LG-2){1'b0}}, a_len, {(LG+2){1'b0}}};
                        a_left    <= a_left - a_len[6:0];
                        if (first_ar) begin
                            rdy_i    <= 1'b1;
                            first_ar <= 1'b0;
                        end
                    end

                    // ---- accept beats ---------------------------------------
                    if (m_rvalid && m_rready) begin
                        if (m_rresp != 2'b00 && o_err_rresp != 16'hFFFF) o_err_rresp <= o_err_rresp + 16'd1;
                        if (!abort) begin
                            hb_valid   <= 1'b1;            // hb_data takes m_rdata now (!hb_valid)
                            hb_w       <= first_beat ? off : {LG{1'b0}};
                            first_beat <= 1'b0;
                        end
                    end

                    // r_left: beats requested minus beats received
                    r_left <= r_left
                              + ((m_arvalid && m_arready) ? a_len[6:0] : 7'd0)
                              - ((m_rvalid && m_rready)   ? 7'd1       : 7'd0);

                    // ---- deliver words --------------------------------------
                    if (en_q) en_seen <= 1'b1;
                    if (!abort && hb_valid && en_q && w_left != 9'd0) begin
                        valid_i  <= 1'b1;
                        hb_w     <= hb_w + 1'b1;
                        w_left   <= w_left - 9'd1;
                        if (&hb_w || w_left == 9'd1) hb_valid <= 1'b0;
                    end

                    // ---- abandon -------------------------------------------
                    if (!abort && w_left != 9'd0 && ((en_seen && !en_q) || p_valid || rd_go_new)) begin
                        abort    <= 1'b1;
                        hb_valid <= 1'b0;
                        if (o_aborts != 16'hFFFF) o_aborts <= o_aborts + 16'd1;
                    end

                    // ---- done: everything delivered (or abandoned) and drained
                    if ((w_left == 9'd0 || abort) && !m_arvalid &&
                        (a_left == 7'd0 || abort) && r_left == 7'd0 && !(m_rvalid && m_rready)) begin
                        abort    <= 1'b0;
                        hb_valid <= 1'b0;
                        state    <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
