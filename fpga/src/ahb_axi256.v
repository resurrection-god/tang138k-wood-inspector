// ============================================================================
// Module      : ahb_axi256
// AE350 data-memory port (AHB-Lite, 64-bit, "Customized Data Memory") ->
// AXI4 master, full width DW (256) - docs/ARCHITECTURE.md § 4.1.
// Every CPU access to DDR3 passes through here, so correctness comes
// before speed, and ordering is strict:
//
//   * The DDR3 IP accepts full-width transfers only (AxSIZE = beat, INCR/WRAP,
//     address bits below the beat ignored - IPUG281 and the IP's synthesis
//     warnings). A 64-bit AHB beat therefore travels as one
//     256-bit beat at its line address, with WSTRB selecting its bytes; a read
//     returns the 64-bit lane addr[4:3] of the line.
//   * A cache line is 32 bytes - one 256-bit beat. Reads: the first beat of a
//     burst fetches the line; later beats of the SAME burst that stay in that
//     line are served from it with one wait state. Nothing is kept across bursts: the
//     FPGA writes the camera rings behind the CPU's back, and a stale line here
//     would survive the CPU's own cache invalidation.
//   * Every data phase waits at least one cycle: HADDR is only registered at the
//     address phase and every decision is taken from the register (27.10.2026,
//     ERRORS 6.19 - the line compares straight from the hard SoC's HADDR set clk_x1's
//     limit).
//   * Writes: beats of a fixed-length burst (INCR4/8/16, WRAP4/8/16) are
//     collected into a line with one wait each; the line is written when the burst's
//     last beat arrives (that beat waits for the write response) or when the
//     burst moves to another line. SINGLE and undefined-length INCR beats are
//     each written on their own and wait for the response.
//   * Before any read, a collected but unwritten line is written first, so a
//     read never overtakes an earlier write. If a burst ends early (IDLE or
//     NONSEQ while a line is still collected) the line is written in the
//     background; IDLE itself is answered with no wait, as AHB requires, and
//     the next transfer waits for the write.
//   * HRESP is always OKAY; non-OKAY AXI responses are counted.
// ============================================================================

`timescale 1ns / 1ps

module ahb_axi256 #(
    parameter integer DW = 256,
    parameter integer AW = 30
)(
    input  wire             clk,
    input  wire             rst_n,

    // ---- AHB-Lite slave (point to point: hready is both HREADYOUT and HREADY)
    input  wire [31:0]      haddr,
    input  wire [1:0]       htrans,
    input  wire             hwrite,
    input  wire [2:0]       hsize,
    input  wire [2:0]       hburst,
    input  wire [63:0]      hwdata,
    output reg  [63:0]      hrdata,
    output reg              hready,
    output wire             hresp,

    // ---- AXI4 master (one port of axi_mux) ----------------------------------
    output reg              m_awvalid,
    input  wire             m_awready,
    output wire [AW-1:0]    m_awaddr,
    output wire [7:0]       m_awlen,
    output reg              m_wvalid,
    input  wire             m_wready,
    output wire [DW-1:0]    m_wdata,
    output wire [DW/8-1:0]  m_wstrb,
    output wire             m_wlast,
    input  wire             m_bvalid,
    output wire             m_bready,
    input  wire [1:0]       m_bresp,
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
    output reg  [15:0]      o_err_resp,     // non-OKAY B/R
    output reg  [31:0]      o_line_reads,   // AXI reads issued
    output reg  [31:0]      o_line_writes   // AXI writes issued
);

    localparam [1:0] T_IDLE = 2'b00, T_BUSY = 2'b01, T_NONSEQ = 2'b10, T_SEQ = 2'b11;
    localparam integer LB = 5;                      // log2(bytes per line)
    localparam integer LW = 32 - LB;                // line-number width

    assign hresp    = 1'b0;
    assign m_awlen  = 8'd0;
    assign m_arlen  = 8'd0;
    assign m_wlast  = 1'b1;
    assign m_bready = 1'b1;
    assign m_rready = 1'b1;

    // Beats in a fixed-length burst; 0 = undefined length (INCR), 1 = SINGLE.
    function [4:0] burst_beats;
        input [2:0] b;
        case (b)
            3'b000:          burst_beats = 5'd1;
            3'b010, 3'b011:  burst_beats = 5'd4;
            3'b100, 3'b101:  burst_beats = 5'd8;
            3'b110, 3'b111:  burst_beats = 5'd16;
            default:         burst_beats = 5'd0;
        endcase
    endfunction

    // Byte strobes of one AHB beat inside its 256-bit line.
    function [DW/8-1:0] beat_strb;
        input [31:0] a;
        input [2:0]  sz;
        reg   [7:0]  s8;
        begin
            case (sz)
                3'd0:    s8 = 8'b0000_0001 << a[2:0];
                3'd1:    s8 = 8'b0000_0011 << {a[2:1], 1'b0};
                3'd2:    s8 = 8'b0000_1111 << {a[2], 2'b00};
                default: s8 = 8'b1111_1111;
            endcase
            beat_strb = {{(DW/8-8){1'b0}}, s8} << (a[4:3] * 8);
        end
    endfunction

    function [DW-1:0] strb_to_bits;
        input [DW/8-1:0] s;
        integer k;
        begin
            for (k = 0; k < DW/8; k = k + 1)
                strb_to_bits[k*8 +: 8] = {8{s[k]}};
        end
    endfunction

    // ---- read line (valid only inside the burst that fetched it) -------------
    reg             lb_valid;
    reg  [LW-1:0]   lb_line;
    reg  [DW-1:0]   lb_data;

    // ---- write line being collected ------------------------------------------
    reg             wl_valid;
    reg  [LW-1:0]   wl_line;
    reg  [DW-1:0]   wl_data;
    reg  [DW/8-1:0] wl_strb;

    // ---- the transfer whose data phase is stalled ----------------------------
    reg             pd_valid;
    reg             pd_write;
    reg             pd_last;        // write: last beat of its burst -> write the line
    reg             pd_capt;        // write: data already merged into wl
    reg  [31:0]     pd_addr;
    reg  [2:0]      pd_size;
    reg  [DW/8-1:0] pd_strb;        // = beat_strb(pd_addr, pd_size), latched with them (RTL_RULES 3.1)

    reg  [4:0]      bt_rem;         // beats left in a fixed burst after the last sampled one
    reg             bt_undef;       // current burst is INCR of undefined length

    localparam F_IDLE = 3'd0, F_WR = 3'd1, F_WB = 3'd2, F_NEXT = 3'd3, F_RD = 3'd4, F_RR = 3'd5;
    reg  [2:0]      f_state;
    reg             aw_done, w_done;

    assign m_awaddr = {wl_line[AW-LB-1:0], {LB{1'b0}}};    // bits above AW are not memory
    assign m_wdata  = wl_data;
    assign m_wstrb  = wl_strb;
    assign m_araddr = {pd_addr[AW-1:LB], {LB{1'b0}}};

    // the new address phase
    wire            t_act    = htrans[1];
    wire            t_nonseq = (htrans == T_NONSEQ);
    wire [4:0]      t_blen   = burst_beats(hburst);
    wire            t_last   = t_nonseq ? (t_blen == 5'd1 || t_blen == 5'd0)
                                        : (bt_undef || bt_rem == 5'd1);

    // pending write data, merged in F_NEXT (hwdata is held while hready is low)
    // pd_strb is a flop: beat_strb() of pd_size/pd_addr used to drive the
    // per-byte enables of wl_data, a B4 worst path (RTL_RULES 3.1). It is built
    // from HSIZE/HADDR[4:0] only: no wide HADDR compare reaches it (ERRORS 6.19).
    wire [DW/8-1:0] h_strb   = beat_strb(haddr, hsize);
    wire [DW-1:0]   pd_bits  = strb_to_bits(pd_strb);
    wire [DW-1:0]   pd_wide  = {{(DW-64){1'b0}}, hwdata} << (pd_addr[4:3] * 64);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hready        <= 1'b1;
            hrdata        <= 64'd0;
            lb_valid      <= 1'b0;
            lb_line       <= {LW{1'b0}};
            lb_data       <= {DW{1'b0}};
            wl_valid      <= 1'b0;
            wl_line       <= {LW{1'b0}};
            wl_data       <= {DW{1'b0}};
            wl_strb       <= {(DW/8){1'b0}};
            pd_valid      <= 1'b0;
            pd_write      <= 1'b0;
            pd_last       <= 1'b0;
            pd_capt       <= 1'b0;
            pd_addr       <= 32'd0;
            pd_size       <= 3'd0;
            pd_strb       <= {{(DW/8-1){1'b0}}, 1'b1};   // = beat_strb(0, 0)
            bt_rem        <= 5'd0;
            bt_undef      <= 1'b0;
            f_state       <= F_IDLE;
            aw_done       <= 1'b0;
            w_done        <= 1'b0;
            m_awvalid     <= 1'b0;
            m_wvalid      <= 1'b0;
            m_arvalid     <= 1'b0;
            o_err_resp    <= 16'd0;
            o_line_reads  <= 32'd0;
            o_line_writes <= 32'd0;
        end else begin
            // =================================================================
            // Edge with HREADY high: the data phase in progress ends and the
            // next address phase is sampled.
            // =================================================================
            if (hready) begin
                // the new address phase
                if (t_act) begin
                    if (t_nonseq) begin
                        lb_valid <= 1'b0;
                        bt_undef <= (t_blen == 5'd0);
                        bt_rem   <= (t_blen == 5'd0) ? 5'd0 : t_blen - 5'd1;
                    end else if (!bt_undef && bt_rem != 5'd0) begin
                        bt_rem   <= bt_rem - 5'd1;
                    end

                    // Every data phase waits at least one cycle and is decided in F_NEXT from the
                    // registered pd_addr: the hard SoC's HADDR reaches only these flops, never the
                    // line compares (27 bits) and the enables behind them - that cone missed clk_x1's
                    // margin in every B2 placement (ERRORS 6.19). Costs one wait state on a read hit
                    // and on a collected write beat.
                    hready   <= 1'b0;
                    pd_valid <= 1'b1;
                    pd_write <= hwrite;
                    pd_last  <= t_last;
                    pd_capt  <= 1'b0;
                    pd_addr  <= haddr;
                    pd_size  <= hsize;
                    pd_strb  <= h_strb;
                    if (f_state == F_IDLE) f_state <= F_NEXT;
                end else if (htrans == T_IDLE && wl_valid && f_state == F_IDLE) begin
                    // burst ended early with a line collected: write it in the background
                    f_state <= F_WR;
                end
            end

            // =================================================================
            // AXI operations
            // =================================================================
            case (f_state)
                F_IDLE: begin
                    // A transfer sampled on the very edge a background write
                    // finished is left pending with no op running: pick it up.
                    if (pd_valid) f_state <= F_NEXT;
                end

                F_WR: begin
                    if (!aw_done && !m_awvalid && !w_done && !m_wvalid) begin
                        m_awvalid <= 1'b1;
                        m_wvalid  <= 1'b1;
                    end else begin
                        if (m_awvalid && m_awready) begin m_awvalid <= 1'b0; aw_done <= 1'b1; end
                        if (m_wvalid  && m_wready)  begin m_wvalid  <= 1'b0; w_done  <= 1'b1; end
                        if ((aw_done || (m_awvalid && m_awready)) && (w_done || (m_wvalid && m_wready)))
                            f_state <= F_WB;
                    end
                end

                F_WB: begin
                    aw_done <= 1'b0;
                    w_done  <= 1'b0;
                    if (m_bvalid) begin
                        if (m_bresp != 2'b00 && o_err_resp != 16'hFFFF) o_err_resp <= o_err_resp + 16'd1;
                        o_line_writes <= o_line_writes + 32'd1;
                        wl_valid      <= 1'b0;
                        wl_strb       <= {(DW/8){1'b0}};
                        f_state       <= F_NEXT;
                    end
                end

                F_NEXT: begin
                    if (!pd_valid) begin
                        f_state <= F_IDLE;
                    end else if (!pd_capt && wl_valid &&
                                 (!pd_write || pd_addr[31:LB] != wl_line)) begin
                        // a collected line must reach DDR3 before this transfer
                        f_state <= F_WR;
                    end else if (pd_write && !pd_capt) begin
                        // the stalled write's data is on hwdata now
                        wl_valid <= 1'b1;
                        wl_line  <= pd_addr[31:LB];
                        wl_data  <= ((wl_valid ? wl_data : {DW{1'b0}}) & ~pd_bits) | (pd_wide & pd_bits);
                        wl_strb  <= (wl_valid ? wl_strb : {(DW/8){1'b0}}) | pd_strb;
                        pd_capt  <= 1'b1;
                        if (pd_last) begin
                            f_state <= F_WR;
                        end else begin
                            pd_valid <= 1'b0;
                            hready   <= 1'b1;
                            f_state  <= F_IDLE;
                        end
                    end else if (pd_write) begin
                        // captured and written: complete the data phase
                        pd_valid <= 1'b0;
                        hready   <= 1'b1;
                        f_state  <= F_IDLE;
                    end else if (lb_valid && lb_line == pd_addr[31:LB]) begin
                        // read hit inside the burst (lb_valid is cleared at every NONSEQ)
                        hrdata   <= lb_data[pd_addr[4:3]*64 +: 64];
                        pd_valid <= 1'b0;
                        hready   <= 1'b1;
                        f_state  <= F_IDLE;
                    end else begin
                        f_state <= F_RD;
                    end
                end

                F_RD: begin
                    if (!m_arvalid) begin
                        m_arvalid <= 1'b1;
                    end else if (m_arready) begin
                        m_arvalid    <= 1'b0;
                        o_line_reads <= o_line_reads + 32'd1;
                        f_state      <= F_RR;
                    end
                end

                F_RR: begin
                    if (m_rvalid) begin
                        if (m_rresp != 2'b00 && o_err_resp != 16'hFFFF) o_err_resp <= o_err_resp + 16'd1;
                        lb_valid <= 1'b1;
                        lb_line  <= pd_addr[31:LB];
                        lb_data  <= m_rdata;
                        hrdata   <= m_rdata[pd_addr[4:3]*64 +: 64];
                        pd_valid <= 1'b0;
                        hready   <= 1'b1;
                        f_state  <= F_IDLE;
                    end
                end

                default: f_state <= F_IDLE;
            endcase
        end
    end

    // synthesis translate_off
    // The latched strobes must match the function of the latched address/size.
    wire [DW/8-1:0] chk_strb = beat_strb(pd_addr, pd_size);
    always @(posedge clk or negedge rst_n) begin
        if (rst_n && pd_strb !== chk_strb) begin
            $display("ERROR ahb_axi256 %m: pd_strb=%h expected %h at %0t", pd_strb, chk_strb, $time);
            $fatal(1);
        end
    end
    // synthesis translate_on

endmodule
