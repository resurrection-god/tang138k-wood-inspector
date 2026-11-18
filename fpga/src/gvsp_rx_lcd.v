// ============================================================================
// Module      : gvsp_rx_lcd
// Hardware GigE Vision (GVSP) UDP Image Stream Receiver -> LCD Framebuffer FIFO
//
// Listens directly on the Triple-Speed Ethernet MAC client RX port (rx_mac_clk,
// 125 MHz) for IPv4/UDP GVSP packets addressed to cfg_udp_port. Supports both
// Standard (8 B) and Extended ID (20 B) GVSP headers for Mono8 image payloads,
// downscales on the fly via multiplier-free Bresenham accumulators to 200x120,
// and crosses into the AHB_CLK (100 MHz) domain through async_fifo_simple.
// ============================================================================

`timescale 1ns / 1ps

module gvsp_rx_lcd (
    // MAC RX domain (125 MHz)
    input  wire        rx_clk,
    input  wire        rx_rst_n_async,
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    input  wire        rx_last,
    input  wire        rx_error,

    // LCD / APB domain (100 MHz AHB_CLK)
    input  wire        lcd_clk,
    input  wire        lcd_rst_n,
    input  wire        cfg_enable,
    input  wire [15:0] cfg_udp_port,

    // Pixel write stream in lcd_clk domain
    output reg         pix_valid,
    output reg  [6:0]  pix_y,
    output reg  [7:0]  pix_x,
    output reg  [7:0]  pix_data,

    // Status counters in lcd_clk domain
    output reg  [15:0] stat_frames,
    output reg  [15:0] stat_pkts,
    output reg  [15:0] stat_w,
    output reg  [15:0] stat_h
);

    localparam [7:0] LCD_W = 8'd200;
    localparam [6:0] LCD_H = 7'd120;

    // ------------------------------------------------------------------------
    // 1. CDC: Reset & Configuration (lcd_clk -> rx_clk)
    // ------------------------------------------------------------------------
    (* async_reg = "true" *) reg [1:0] rx_rst_sync;
    always @(posedge rx_clk) begin
        if (!rx_rst_n_async)
            rx_rst_sync <= 2'b00;
        else
            rx_rst_sync <= {rx_rst_sync[0], 1'b1};
    end
    wire rx_rst_n = rx_rst_sync[1];

    (* async_reg = "true" *) reg [1:0]  en_sync;
    (* async_reg = "true" *) reg [15:0] port_sync1, port_sync2;
    always @(posedge rx_clk) begin
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
    // 2. Packet Header & GVSP Stream Parser (rx_clk domain)
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
    reg [15:0] active_bid;
    reg [11:0] active_w;
    reg [11:0] active_h;
    reg [11:0] active_w_m1;
    reg [23:0] expected_pid;
    reg        pid_match;

    // Bresenham downsampler state
    reg [11:0] src_x;
    reg [11:0] acc_x;
    reg [11:0] acc_y;
    reg [7:0]  lcd_x;
    reg [6:0]  lcd_y;
    reg        take_x;
    reg        take_y;

    // Registered push to async_fifo_simple (obeys RTL_RULES § 3.1)
    reg        fifo_wr_en_q;
    reg [22:0] fifo_wr_data_q;
    wire       fifo_wr_full;

    // Status toggles (rx_clk -> lcd_clk)
    reg        rx_frame_tgl;
    reg        rx_pkt_tgl;
    reg [15:0] rx_geom_w;
    reg [15:0] rx_geom_h;

    wire [11:0] body_start   = gvsp_ext ? 12'd62 : 12'd50;
    wire [11:0] ldr_rel      = byte_idx - body_start;
    wire        in_gvsp_body = pkt_ok && (byte_idx >= body_start) && (byte_idx < udp_end_idx);
    wire        pkt_complete = pkt_ok && !rx_error && (byte_idx >= 12'd49) &&
                               ((byte_idx + 12'd1) >= udp_end_idx);

    wire [12:0] next_acc_x = {1'b0, acc_x} + 13'd200;
    wire [12:0] next_acc_y = {1'b0, acc_y} + 13'd120;
    wire        step_x     = (next_acc_x >= {1'b0, active_w});
    wire        step_y     = (next_acc_y >= {1'b0, active_h});

    always @(posedge rx_clk) begin
        fifo_wr_en_q <= 1'b0;

        if (!rx_rst_n) begin
            byte_idx       <= 12'd0;
            pkt_ok         <= 1'b0;
            udp_len        <= 16'd0;
            udp_end_idx    <= 12'd0;
            gvsp_ext       <= 1'b0;
            gvsp_fmt       <= 4'd0;
            gvsp_bid       <= 16'd0;
            gvsp_pid       <= 24'd0;
            ldr_pf_ok      <= 1'b0;
            ldr_w          <= 16'd0;
            ldr_h          <= 16'd0;
            frame_active   <= 1'b0;
            active_bid     <= 16'd0;
            active_w       <= 12'd720;
            active_h       <= 12'd480;
            active_w_m1    <= 12'd719;
            expected_pid   <= 24'd1;
            pid_match      <= 1'b0;
            src_x          <= 12'd0;
            acc_x          <= 12'd0;
            acc_y          <= 12'd0;
            lcd_x          <= 8'd0;
            lcd_y          <= 7'd0;
            take_x         <= 1'b0;
            take_y         <= 1'b0;
            fifo_wr_data_q <= 23'd0;
            rx_frame_tgl   <= 1'b0;
            rx_pkt_tgl     <= 1'b0;
            rx_geom_w      <= 16'd0;
            rx_geom_h      <= 16'd0;
        end else if (rx_valid) begin
            // Start-of-packet init at byte_idx == 0
            if (byte_idx == 12'd0) begin
                pkt_ok    <= rx_en && !rx_error;
                gvsp_ext  <= 1'b0;
                gvsp_fmt  <= 4'd0;
                gvsp_bid  <= 16'd0;
                gvsp_pid  <= 24'd0;
                ldr_pf_ok <= 1'b1;
                ldr_w     <= 16'd0;
                ldr_h     <= 16'd0;
                pid_match <= 1'b0;
            end else if (rx_error) begin
                pkt_ok <= 1'b0;
            end

            // Header filters: EtherType=0x0800, IPv4 IHL=5 (0x45), Proto=UDP (0x11), DstPort
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

            // GVSP Header decode (starting at byte_idx == 42)
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
                    12'd49: if (!gvsp_ext) begin
                        gvsp_pid[7:0] <= rx_data;
                        if (gvsp_fmt == 4'h3 && frame_active && (gvsp_bid == active_bid)) begin
                            if ({gvsp_pid[23:8], rx_data} == expected_pid) begin
                                pid_match    <= 1'b1;
                                expected_pid <= expected_pid + 24'd1;
                            end else begin
                                pid_match    <= 1'b0;
                                frame_active <= 1'b0;
                            end
                        end
                    end
                    12'd56: if (gvsp_ext) gvsp_bid[15:8]  <= rx_data;
                    12'd57: if (gvsp_ext) gvsp_bid[7:0]   <= rx_data;
                    12'd59: if (gvsp_ext) gvsp_pid[23:16] <= rx_data;
                    12'd60: if (gvsp_ext) gvsp_pid[15:8]  <= rx_data;
                    12'd61: if (gvsp_ext) begin
                        gvsp_pid[7:0] <= rx_data;
                        if (gvsp_fmt == 4'h3 && frame_active && (gvsp_bid == active_bid)) begin
                            if ({gvsp_pid[23:8], rx_data} == expected_pid) begin
                                pid_match    <= 1'b1;
                                expected_pid <= expected_pid + 24'd1;
                            end else begin
                                pid_match    <= 1'b0;
                                frame_active <= 1'b0;
                            end
                        end
                    end
                    default: ;
                endcase
            end

            // GVSP Body processing (unified path for both Standard and Extended ID)
            if (in_gvsp_body) begin
                if (gvsp_fmt == 4'h1) begin
                    // GVSP Leader: PixelFormat == 0x01080001 (Mono8), SizeX, SizeY
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
                    if (take_x && take_y && (lcd_x < LCD_W) && (lcd_y < LCD_H) && !fifo_wr_full) begin
                        fifo_wr_en_q   <= 1'b1;
                        fifo_wr_data_q <= {lcd_y, lcd_x, rx_data};
                    end

                    if (src_x >= active_w_m1) begin
                        src_x  <= 12'd0;
                        acc_x  <= 12'd0;
                        lcd_x  <= 8'd0;
                        take_x <= 1'b1;
                        if (step_y) begin
                            acc_y  <= next_acc_y[11:0] - active_h;
                            lcd_y  <= lcd_y + 7'd1;
                            take_y <= 1'b1;
                        end else begin
                            acc_y  <= next_acc_y[11:0];
                            take_y <= 1'b0;
                        end
                    end else begin
                        src_x <= src_x + 12'd1;
                        if (step_x) begin
                            acc_x  <= next_acc_x[11:0] - active_w;
                            lcd_x  <= lcd_x + 8'd1;
                            take_x <= 1'b1;
                        end else begin
                            acc_x  <= next_acc_x[11:0];
                            take_x <= 1'b0;
                        end
                    end
                end
            end

            // End-of-packet state transitions
            if (rx_last) begin
                byte_idx <= 12'd0;
                if (pkt_complete) begin
                    if (gvsp_fmt == 4'h1 && ldr_pf_ok && (ldr_w >= 16'd200) && (ldr_w <= 16'd2048) &&
                        (ldr_h >= 16'd120) && (ldr_h <= 16'd2048)) begin
                        frame_active <= 1'b1;
                        active_bid   <= gvsp_bid;
                        active_w     <= ldr_w[11:0];
                        active_h     <= ldr_h[11:0];
                        active_w_m1  <= ldr_w[11:0] - 12'd1;
                        expected_pid <= 24'd1;
                        src_x        <= 12'd0;
                        acc_x        <= 12'd0;
                        acc_y        <= 12'd0;
                        lcd_x        <= 8'd0;
                        lcd_y        <= 7'd0;
                        take_x       <= 1'b1;
                        take_y       <= 1'b1;
                        rx_geom_w    <= ldr_w;
                        rx_geom_h    <= ldr_h;
                    end else if (gvsp_fmt == 4'h3 && frame_active && pid_match) begin
                        rx_pkt_tgl <= ~rx_pkt_tgl;
                    end else if (gvsp_fmt == 4'h2 && frame_active && (gvsp_bid == active_bid)) begin
                        frame_active <= 1'b0;
                        rx_frame_tgl <= ~rx_frame_tgl;
                    end
                end
            end else if (byte_idx != 12'hFFF) begin
                byte_idx <= byte_idx + 12'd1;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 3. Gray-coded Async FIFO (rx_clk 125 MHz -> lcd_clk 100 MHz, 1 BSRAM)
    // ------------------------------------------------------------------------
    wire        fifo_rd_empty;
    wire [22:0] fifo_rd_data;
    wire        fifo_rd_en = cfg_enable && !fifo_rd_empty;

    async_fifo_simple #(
        .DATA_WIDTH(23),
        .ADDR_BITS(9)
    ) u_pix_fifo (
        .wr_clk(rx_clk),
        .wr_rst_n(rx_rst_n),
        .wr_en(fifo_wr_en_q),
        .wr_data(fifo_wr_data_q),
        .wr_full(fifo_wr_full),
        .wr_used(),

        .rd_clk(lcd_clk),
        .rd_rst_n(lcd_rst_n),
        .rd_en(fifo_rd_en),
        .rd_data(fifo_rd_data),
        .rd_empty(fifo_rd_empty),
        .rd_used()
    );

    reg fifo_rd_v1;
    always @(posedge lcd_clk) begin
        if (!lcd_rst_n) begin
            fifo_rd_v1 <= 1'b0;
            pix_valid  <= 1'b0;
            pix_y      <= 7'd0;
            pix_x      <= 8'd0;
            pix_data   <= 8'd0;
        end else begin
            fifo_rd_v1 <= fifo_rd_en;
            pix_valid  <= fifo_rd_v1;
            if (fifo_rd_v1) begin
                pix_y    <= fifo_rd_data[22:16];
                pix_x    <= fifo_rd_data[15:8];
                pix_data <= fifo_rd_data[7:0];
            end
        end
    end

    // ------------------------------------------------------------------------
    // 4. CDC: Status Toggles (rx_clk -> lcd_clk)
    // ------------------------------------------------------------------------
    (* async_reg = "true" *) reg [2:0] ftgl_sync, ptgl_sync;
    always @(posedge lcd_clk) begin
        if (!lcd_rst_n) begin
            ftgl_sync   <= 3'b000;
            ptgl_sync   <= 3'b000;
            stat_frames <= 16'd0;
            stat_pkts   <= 16'd0;
            stat_w      <= 16'd0;
            stat_h      <= 16'd0;
        end else begin
            ftgl_sync <= {ftgl_sync[1:0], rx_frame_tgl};
            ptgl_sync <= {ptgl_sync[1:0], rx_pkt_tgl};
            if (ftgl_sync[2] != ftgl_sync[1]) begin
                stat_frames <= stat_frames + 16'd1;
                stat_w      <= rx_geom_w;
                stat_h      <= rx_geom_h;
            end
            if (ptgl_sync[2] != ptgl_sync[1]) begin
                stat_pkts <= stat_pkts + 16'd1;
            end
        end
    end

endmodule
