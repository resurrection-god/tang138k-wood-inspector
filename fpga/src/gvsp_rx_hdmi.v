// ============================================================================
// Module      : gvsp_rx_hdmi
// Wire-speed hardware GigE Vision Stream Protocol (GVSP) receiver targeting
// 360x270 Mono8 ping-pong video on Tang Mega 138K Pro (Stage 5 HDMI 720p60).
//
// Features:
//   - Taps TSE MAC 8-bit RX stream directly at 125 MHz (rx_mac_clk).
//   - Parses Ethernet/IPv4/UDP + GVSP Standard (42..49) & Extended (42..61) ID.
//   - Bresenham downsampling from arbitrary camera WxH (>= 360x270) to 360x270;
//     for Hikrobot MV-CS004-10GM 720x540 Mono8 this is exact 2:1 decimation.
//   - Packet-drop resilience: if a UDP packet drops mid-frame (pid > expected),
//     a 24-cycle divider re-synchronizes (row, col) from (pid - 1) * pkt_stride
//     instead of aborting the remainder of the frame.
//   - Missed-Trailer resilience: if a new Leader arrives while frame_active is
//     still asserted with pixels written, emits an inline EOF sentinel first.
//   - Ordering-safe CDC: sends {op[1:0], y[8:0], x[8:0], pix[7:0]} (28-bit)
//     through a single Gray-coded async FIFO (1 BSRAM) to clk_x1 (100 MHz):
//       op = 2'b01 : Start-Of-Frame (Leader)
//       op = 2'b00 : Pixel {y[8:0], x[8:0], data[7:0]}
//       op = 2'b10 : End-Of-Frame (Trailer)
// ============================================================================

`timescale 1ns / 1ps

module gvsp_rx_hdmi #(
    parameter integer DST_W = 360,
    parameter integer DST_H = 270
)(
    // Ethernet MAC RX domain (125 MHz)
    input  wire        rx_clk,
    input  wire        rx_rst_n_async,
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    input  wire        rx_last,
    input  wire        rx_error,

    // Video / SoC domain (100 MHz clk_x1)
    input  wire        sys_clk,
    input  wire        sys_rst_n,

    // Configuration (sys_clk domain)
    input  wire        cfg_enable,
    input  wire [15:0] cfg_udp_port,

    // Pixel / Frame stream output (sys_clk domain, 1-cycle registered)
    output reg         out_valid,
    output reg  [1:0]  out_op,       // 2'b01=SOF, 2'b00=PIX, 2'b10=EOF
    output reg  [8:0]  out_y,
    output reg  [8:0]  out_x,
    output reg  [7:0]  out_data,

    // Telemetry (sys_clk domain)
    output reg  [15:0] stat_frames,
    output reg  [15:0] stat_pkts,
    output reg  [15:0] stat_drops,
    output reg  [15:0] stat_w,
    output reg  [15:0] stat_h
);

    // ------------------------------------------------------------------------
    // 1. Reset & Config Synchronisation into rx_clk
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_sync;
    always @(posedge rx_clk or negedge rx_rst_n_async) begin
        if (!rx_rst_n_async) rst_sync <= 2'b00;
        else                 rst_sync <= {rst_sync[0], 1'b1};
    end
    wire rx_rst_n = rst_sync[1];

    (* ASYNC_REG = "TRUE" *) reg [1:0]  en_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] port_sync1, port_sync2;
    always @(posedge rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            en_sync    <= 2'b00;
            port_sync1 <= 16'd5557;
            port_sync2 <= 16'd5557;
        end else begin
            en_sync    <= {en_sync[0], cfg_enable};
            port_sync1 <= cfg_udp_port;
            port_sync2 <= port_sync1;
        end
    end
    wire        rx_en   = en_sync[1];
    wire [15:0] rx_port = port_sync2;

    // ------------------------------------------------------------------------
    // 2. Packet Parser & Drop-Resilient Bresenham Downsampler (rx_clk)
    // ------------------------------------------------------------------------
    reg [11:0] byte_idx;
    reg        pkt_ok;
    reg [15:0] udp_len;
    reg [11:0] udp_end_idx;

    reg        gvsp_ext;
    reg [3:0]  gvsp_fmt;
    reg [15:0] gvsp_bid;
    reg [23:0] gvsp_pid;

    reg        ldr_pf_ok;
    reg [15:0] ldr_w;
    reg [15:0] ldr_h;

    reg        frame_active;
    reg        frame_has_data;
    reg [15:0] active_bid;
    reg [11:0] active_w;
    reg [11:0] active_h;
    reg [11:0] active_w_m1;
    reg [23:0] expected_pid;
    reg        pid_match;

    reg [11:0] pkt_stride;
    reg        pkt_stride_valid;

    // Defer SOF emission by 1 cycle if previous frame missed its Trailer (EOF first)
    reg        pending_sof_push;

    // Packet-loss fast resync divider: offset = (gvsp_pid - 1) * pkt_stride
    // Stage 0 (resync_mul_q): register pid_m1 & pkt_stride multiply cleanly (1 clk)
    // Stage 1 (resync_busy):  computes row = offset / active_w, col = offset % active_w in 24 cycles.
    // Stage 2 (resync_fin):   maps (row, col + 25) to (src_x, img_x, img_y) cleanly (1 clk).
    reg        resync_mul_q;
    reg [23:0] resync_pid_m1;
    reg        resync_busy;
    reg        resync_fin_q;
    reg [4:0]  resync_cnt;
    reg [23:0] resync_num;
    reg [23:0] resync_q;
    reg [12:0] resync_rem;
    reg [11:0] resync_col_q;
    reg [11:0] resync_row_q;

    // Bresenham downsampler state
    reg [11:0] src_x;
    reg [11:0] acc_x;
    reg [11:0] acc_y;
    reg [8:0]  img_x;
    reg [8:0]  img_y;
    reg        take_x;
    reg        take_y;

    // Registered push to async_fifo_simple
    reg        fifo_wr_en_q;
    reg [27:0] fifo_wr_data_q;
    wire       fifo_wr_full;

    // Status toggles (rx_clk -> sys_clk)
    reg        rx_frame_tgl;
    reg        rx_pkt_tgl;
    reg        rx_drop_tgl;
    reg [15:0] rx_geom_w;
    reg [15:0] rx_geom_h;

    wire [11:0] body_start   = gvsp_ext ? 12'd62 : 12'd50;
    wire [11:0] hdr_udp_sub  = gvsp_ext ? 12'd28 : 12'd16;
    wire [11:0] ldr_rel      = byte_idx - body_start;
    wire        in_gvsp_body = pkt_ok && (byte_idx >= body_start) && (byte_idx < udp_end_idx);
    wire        pkt_complete = pkt_ok && !rx_error && (byte_idx >= 12'd49) &&
                               ((byte_idx + 12'd1) >= udp_end_idx);

    wire [12:0] next_acc_x = {1'b0, acc_x} + DST_W[12:0];
    wire [12:0] next_acc_y = {1'b0, acc_y} + DST_H[12:0];
    wire        step_x     = (next_acc_x >= {1'b0, active_w});
    wire        step_y     = (next_acc_y >= {1'b0, active_h});

    // Helper combinational next step for resync remainder
    wire [12:0] resync_sub_in = {resync_rem[11:0], resync_num[23]};
    wire        resync_ge     = (resync_sub_in >= {1'b0, active_w});
    wire [11:0] rec_col_next  = resync_ge ? (resync_sub_in[11:0] - active_w) : resync_sub_in[11:0];
    wire        gvsp_hdr_done = pkt_ok && ((!gvsp_ext && (byte_idx == 12'd49)) || (gvsp_ext && (byte_idx == 12'd61)));
    wire [23:0] gvsp_pid_full = {gvsp_pid[23:8], rx_data};

    always @(posedge rx_clk) begin
        fifo_wr_en_q <= 1'b0;

        if (!rx_rst_n) begin
            byte_idx         <= 12'd0;
            pkt_ok           <= 1'b0;
            udp_len          <= 16'd0;
            udp_end_idx      <= 12'd0;
            gvsp_ext         <= 1'b0;
            gvsp_fmt         <= 4'd0;
            gvsp_bid         <= 16'd0;
            gvsp_pid         <= 24'd0;
            ldr_pf_ok        <= 1'b0;
            ldr_w            <= 16'd0;
            ldr_h            <= 16'd0;
            frame_active     <= 1'b0;
            frame_has_data   <= 1'b0;
            active_bid       <= 16'd0;
            active_w         <= 12'd720;
            active_h         <= 12'd540;
            active_w_m1      <= 12'd719;
            expected_pid     <= 24'd1;
            pid_match        <= 1'b0;
            pkt_stride       <= 12'd1404;
            pkt_stride_valid <= 1'b0;
            pending_sof_push <= 1'b0;
            resync_mul_q     <= 1'b0;
            resync_pid_m1    <= 24'd0;
            resync_busy      <= 1'b0;
            resync_fin_q     <= 1'b0;
            resync_cnt       <= 5'd0;
            resync_num       <= 24'd0;
            resync_q         <= 24'd0;
            resync_rem       <= 13'd0;
            resync_col_q     <= 12'd0;
            resync_row_q     <= 12'd0;
            src_x            <= 12'd0;
            acc_x            <= 12'd0;
            acc_y            <= 12'd0;
            img_x            <= 9'd0;
            img_y            <= 9'd0;
            take_x           <= 1'b0;
            take_y           <= 1'b0;
            fifo_wr_data_q   <= 28'd0;
            rx_frame_tgl     <= 1'b0;
            rx_pkt_tgl       <= 1'b0;
            rx_drop_tgl      <= 1'b0;
            rx_geom_w        <= 16'd0;
            rx_geom_h        <= 16'd0;
        end else begin
            // Deferred SOF push (cycle right after forced EOF on missed Trailer)
            if (pending_sof_push) begin
                pending_sof_push <= 1'b0;
                if (!fifo_wr_full) begin
                    fifo_wr_en_q   <= 1'b1;
                    fifo_wr_data_q <= {2'b01, 9'd0, 9'd0, 8'd0}; // SOF
                end
            end

            // Stage 0 -> 1: pipelined resync multiply (cuts DSP->div path)
            if (resync_mul_q) begin
                resync_mul_q <= 1'b0;
                resync_busy  <= 1'b1;
                resync_cnt   <= 5'd24;
                resync_num   <= resync_pid_m1 * {12'd0, pkt_stride};
                resync_q     <= 24'd0;
                resync_rem   <= 13'd0;
            end

            // Stage 1: Resync divider (steps 1 bit per rx_clk cycle after header gap)
            if (resync_busy) begin
                resync_num <= {resync_num[22:0], 1'b0};
                resync_q   <= {resync_q[22:0], resync_ge};
                resync_rem <= resync_ge ? ({1'b0, resync_sub_in[11:0] - active_w}) : resync_sub_in;
                if (resync_cnt == 5'd1) begin
                    resync_busy  <= 1'b0;
                    resync_fin_q <= 1'b1;
                    resync_row_q <= {resync_q[10:0], resync_ge};
                    resync_col_q <= rec_col_next;
                end
                resync_cnt <= resync_cnt - 5'd1;
            end

            // Stage 2: Map recovered (row, col + 26) to output coords cleanly (1 clk)
            if (resync_fin_q) begin
                resync_fin_q <= 1'b0;
                pid_match    <= 1'b1;
                if ((resync_col_q + 12'd26) >= active_w) begin
                    src_x  <= resync_col_q + 12'd26 - active_w;
                    acc_x  <= 12'd0;
                    acc_y  <= 12'd0;
                    img_x  <= (resync_col_q[9:0] + 10'd26 - active_w[9:0]) >> 1;
                    img_y  <= (resync_row_q[9:0] + 10'd1) >> 1;
                    take_x <= ~(resync_col_q[0] ^ active_w[0]);
                    take_y <= resync_row_q[0];
                end else begin
                    src_x  <= resync_col_q + 12'd26;
                    acc_x  <= 12'd0;
                    acc_y  <= 12'd0;
                    img_x  <= (resync_col_q[9:0] + 10'd26) >> 1;
                    img_y  <= resync_row_q[9:1];
                    take_x <= ~resync_col_q[0];
                    take_y <= ~resync_row_q[0];
                end
            end

            if (rx_valid) begin
                // Start-of-packet init
                if (byte_idx == 12'd0) begin
                    pkt_ok      <= rx_en && !rx_error;
                    gvsp_ext    <= 1'b0;
                    gvsp_fmt    <= 4'd0;
                    gvsp_bid    <= 16'd0;
                    gvsp_pid    <= 24'd0;
                    ldr_pf_ok   <= 1'b1;
                    ldr_w       <= 16'd0;
                    ldr_h       <= 16'd0;
                    pid_match    <= 1'b0;
                    resync_mul_q <= 1'b0;
                    resync_busy  <= 1'b0;
                    resync_fin_q <= 1'b0;
                end else if (rx_error) begin
                    pkt_ok <= 1'b0;
                end

                // Ethernet / IPv4 / UDP header checks
                case (byte_idx)
                    12'd12: if (rx_data != 8'h08) pkt_ok <= 1'b0;
                    12'd13: if (rx_data != 8'h00) pkt_ok <= 1'b0;
                    12'd14: if (rx_data != 8'h45) pkt_ok <= 1'b0;
                    12'd23: if (rx_data != 8'h11) pkt_ok <= 1'b0;
                    12'd36: if (rx_data != rx_port[15:8]) pkt_ok <= 1'b0;
                    12'd37: if (rx_data != rx_port[7:0])  pkt_ok <= 1'b0;
                    12'd38: udp_len[15:8] <= rx_data;
                    12'd39: begin
                        udp_len[7:0] <= rx_data;
                        udp_end_idx  <= 12'd34 + {udp_len[11:8], rx_data};
                        if ({udp_len[15:8], rx_data} < 16'd16)
                            pkt_ok <= 1'b0;
                    end
                    default: ;
                endcase

                // GVSP header decode
                if (pkt_ok) begin
                    case (byte_idx)
                        12'd44: gvsp_bid[15:8] <= rx_data;
                        12'd45: gvsp_bid[7:0]  <= rx_data;
                        12'd46: begin
                            gvsp_ext <= rx_data[7];
                            gvsp_fmt <= rx_data[3:0];
                        end
                        12'd47: if (!gvsp_ext) gvsp_pid[23:16] <= rx_data;
                        12'd48: if (!gvsp_ext) gvsp_pid[15:8]  <= rx_data;
                        12'd49: if (!gvsp_ext) gvsp_pid[7:0]   <= rx_data;
                        12'd56: if (gvsp_ext)  gvsp_bid[15:8]  <= rx_data;
                        12'd57: if (gvsp_ext)  gvsp_bid[7:0]   <= rx_data;
                        12'd59: if (gvsp_ext)  gvsp_pid[23:16] <= rx_data;
                        12'd60: if (gvsp_ext)  gvsp_pid[15:8]  <= rx_data;
                        12'd61: if (gvsp_ext)  gvsp_pid[7:0]   <= rx_data;
                        default: ;
                    endcase

                    if (gvsp_hdr_done && (gvsp_fmt == 4'h3) && frame_active && (gvsp_bid == active_bid)) begin
                        if (gvsp_pid_full == 24'd1 && !pkt_stride_valid && (udp_len[11:0] > hdr_udp_sub)) begin
                            pkt_stride       <= udp_len[11:0] - hdr_udp_sub;
                            pkt_stride_valid <= 1'b1;
                        end
                        if (gvsp_pid_full == expected_pid) begin
                            pid_match    <= 1'b1;
                            expected_pid <= expected_pid + 24'd1;
                        end else if ((gvsp_pid_full > expected_pid) && pkt_stride_valid &&
                                     ((gvsp_pid_full - expected_pid) <= 24'd32)) begin
                            // Recover after dropped packet without aborting the frame
                            pid_match     <= 1'b0;
                            expected_pid  <= gvsp_pid_full + 24'd1;
                            rx_drop_tgl   <= ~rx_drop_tgl;
                            resync_mul_q  <= 1'b1;
                            resync_pid_m1 <= gvsp_pid_full - 24'd1;
                        end else begin
                            pid_match    <= 1'b0;
                            rx_drop_tgl  <= ~rx_drop_tgl;
                            frame_active <= 1'b0;
                        end
                    end
                end

                // Pre-register pixel payload unconditionally so udp_end_idx doesn't feed data bus
                if (!pending_sof_push) begin
                    fifo_wr_data_q <= {2'b00, img_y, img_x, rx_data};
                end

                // GVSP Body
                if (in_gvsp_body) begin
                    if (gvsp_fmt == 4'h1) begin
                        // Mono8 = 0x01080001
                        case (ldr_rel)
                            12'd12: if (rx_data != 8'h01) ldr_pf_ok <= 1'b0;
                            12'd13: if (rx_data != 8'h08) ldr_pf_ok <= 1'b0;
                            12'd14: if (rx_data != 8'h00) ldr_pf_ok <= 1'b0;
                            12'd15: if (rx_data != 8'h01) ldr_pf_ok <= 1'b0;
                            12'd18: ldr_w[15:8] <= rx_data;
                            12'd19: ldr_w[7:0]  <= rx_data;
                            12'd22: ldr_h[15:8] <= rx_data;
                            12'd23: ldr_h[7:0]  <= rx_data;
                            default: ;
                        endcase
                    end else if (gvsp_fmt == 4'h3 && frame_active && pid_match) begin
                        frame_has_data <= 1'b1;
                        if (take_x && take_y && (img_x < DST_W[8:0]) && (img_y < DST_H[8:0]) && !fifo_wr_full) begin
                            fifo_wr_en_q <= 1'b1;
                        end

                        if (src_x >= active_w_m1) begin
                            src_x  <= 12'd0;
                            acc_x  <= 12'd0;
                            img_x  <= 9'd0;
                            take_x <= 1'b1;
                            if (step_y) begin
                                acc_y  <= next_acc_y[11:0] - active_h;
                                img_y  <= img_y + 9'd1;
                                take_y <= 1'b1;
                            end else begin
                                acc_y  <= next_acc_y[11:0];
                                take_y <= 1'b0;
                            end
                        end else begin
                            src_x <= src_x + 12'd1;
                            if (step_x) begin
                                acc_x  <= next_acc_x[11:0] - active_w;
                                img_x  <= img_x + 9'd1;
                                take_x <= 1'b1;
                            end else begin
                                acc_x  <= next_acc_x[11:0];
                                take_x <= 1'b0;
                            end
                        end
                    end
                end

                // End-of-packet transitions
                if (rx_last) begin
                    byte_idx <= 12'd0;
                    if (pkt_complete) begin
                        if (gvsp_fmt == 4'h1 && ldr_pf_ok &&
                            (ldr_w >= DST_W[15:0]) && (ldr_w <= 16'd2048) &&
                            (ldr_h >= DST_H[15:0]) && (ldr_h <= 16'd2048)) begin
                            // If prior frame never got a Trailer, flush EOF first
                            if (frame_active && frame_has_data) begin
                                rx_drop_tgl <= ~rx_drop_tgl;
                                rx_frame_tgl <= ~rx_frame_tgl;
                                if (!fifo_wr_full) begin
                                    fifo_wr_en_q     <= 1'b1;
                                    fifo_wr_data_q   <= {2'b10, 9'd0, 9'd0, 8'd0}; // EOF
                                    pending_sof_push <= 1'b1;
                                end
                            end else if (!fifo_wr_full) begin
                                fifo_wr_en_q   <= 1'b1;
                                fifo_wr_data_q <= {2'b01, 9'd0, 9'd0, 8'd0}; // SOF
                            end

                            frame_active   <= 1'b1;
                            frame_has_data <= 1'b0;
                            active_bid     <= gvsp_bid;
                            active_w       <= ldr_w[11:0];
                            active_h       <= ldr_h[11:0];
                            active_w_m1    <= ldr_w[11:0] - 12'd1;
                            expected_pid   <= 24'd1;
                            src_x          <= 12'd0;
                            acc_x          <= 12'd0;
                            acc_y          <= 12'd0;
                            img_x          <= 9'd0;
                            img_y          <= 9'd0;
                            take_x         <= 1'b1;
                            take_y         <= 1'b1;
                            rx_geom_w      <= ldr_w;
                            rx_geom_h      <= ldr_h;
                        end else if (gvsp_fmt == 4'h3 && frame_active && (pid_match || resync_busy)) begin
                            rx_pkt_tgl <= ~rx_pkt_tgl;
                        end else if (gvsp_fmt == 4'h2 && frame_active && (gvsp_bid == active_bid)) begin
                            frame_active   <= 1'b0;
                            frame_has_data <= 1'b0;
                            rx_frame_tgl   <= ~rx_frame_tgl;
                            if (!fifo_wr_full) begin
                                fifo_wr_en_q   <= 1'b1;
                                fifo_wr_data_q <= {2'b10, 9'd0, 9'd0, 8'd0}; // EOF
                            end
                        end
                    end
                end else if (byte_idx != 12'hFFF) begin
                    byte_idx <= byte_idx + 12'd1;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // 3. Gray-coded Async FIFO (rx_clk 125 MHz -> sys_clk 100 MHz, 1 BSRAM)
    // ------------------------------------------------------------------------
    wire        fifo_rd_empty;
    wire [27:0] fifo_rd_data;
    wire        fifo_rd_en = cfg_enable && !fifo_rd_empty;

    async_fifo_simple #(
        .DATA_WIDTH(28),
        .ADDR_BITS(9)
    ) u_pix_fifo (
        .wr_clk(rx_clk),
        .wr_rst_n(rx_rst_n),
        .wr_en(fifo_wr_en_q),
        .wr_data(fifo_wr_data_q),
        .wr_full(fifo_wr_full),
        .wr_used(),

        .rd_clk(sys_clk),
        .rd_rst_n(sys_rst_n),
        .rd_en(fifo_rd_en),
        .rd_data(fifo_rd_data),
        .rd_empty(fifo_rd_empty),
        .rd_used()
    );

    reg fifo_rd_v1;
    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            fifo_rd_v1 <= 1'b0;
            out_valid  <= 1'b0;
            out_op     <= 2'b00;
            out_y      <= 9'd0;
            out_x      <= 9'd0;
            out_data   <= 8'd0;
        end else begin
            fifo_rd_v1 <= fifo_rd_en;
            out_valid  <= fifo_rd_v1;
            if (fifo_rd_v1) begin
                out_op   <= fifo_rd_data[27:26];
                out_y    <= fifo_rd_data[25:17];
                out_x    <= fifo_rd_data[16:8];
                out_data <= fifo_rd_data[7:0];
            end
        end
    end

    // ------------------------------------------------------------------------
    // 4. Status Toggle & Geometry Sync (rx_clk -> sys_clk)
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [2:0] frame_tgl_sync, pkt_tgl_sync, drop_tgl_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] w_sync1, w_sync2, h_sync1, h_sync2;

    always @(posedge sys_clk) begin
        if (!sys_rst_n) begin
            frame_tgl_sync <= 3'b000;
            pkt_tgl_sync   <= 3'b000;
            drop_tgl_sync  <= 3'b000;
            w_sync1        <= 16'd0;
            w_sync2        <= 16'd0;
            h_sync1        <= 16'd0;
            h_sync2        <= 16'd0;
            stat_frames    <= 16'd0;
            stat_pkts      <= 16'd0;
            stat_drops     <= 16'd0;
            stat_w         <= 16'd0;
            stat_h         <= 16'd0;
        end else begin
            frame_tgl_sync <= {frame_tgl_sync[1:0], rx_frame_tgl};
            pkt_tgl_sync   <= {pkt_tgl_sync[1:0],   rx_pkt_tgl};
            drop_tgl_sync  <= {drop_tgl_sync[1:0],  rx_drop_tgl};
            w_sync1        <= rx_geom_w;
            w_sync2        <= w_sync1;
            h_sync1        <= rx_geom_h;
            h_sync2        <= h_sync1;
            stat_w         <= w_sync2;
            stat_h         <= h_sync2;

            if (frame_tgl_sync[2] ^ frame_tgl_sync[1])
                stat_frames <= stat_frames + 16'd1;
            if (pkt_tgl_sync[2] ^ pkt_tgl_sync[1])
                stat_pkts <= stat_pkts + 16'd1;
            if (drop_tgl_sync[2] ^ drop_tgl_sync[1])
                stat_drops <= stat_drops + 16'd1;
        end
    end

endmodule
