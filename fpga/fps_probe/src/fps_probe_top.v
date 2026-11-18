// ============================================================================
// fps_probe_top - a separate simplified FPGA project for one question:
// does the sensor frame rate grow when the ROI width (and height) shrinks.
//
// Why separately (ERRORS former 31.54 and 31.53; result in ERRORS 2.9): in the
// main project everything that could count frames is tied to the writer geometry
// and to the SoC, and every rebuild of the main project is 9 minutes and a risk
// for Ethernet. Here there is no SoC, no DDR3, no Ethernet: the build is short
// and the measurement direct.
//
// What is inside:
//   * its own I2C master: sets up camera 0 (connector J15, TWI multiplexer,
//     position 3) and reads back the ROI registers, 0x0814 (requested rate) and
//     0x0838 (rate limit for the ROI - the firmware considers it unreadable);
//   * the hard D-PHY RX and THE SAME CSI-2 bridge as in the main project, with the
//     row length set at run time (cfg_line_bytes);
//   * counts of camera strobes, rows, headers, byte clock frequency;
//   * UART 115200 on COM7: a command to set ROI/rate/exposure, a report once a
//     second. Host - tools/camera/fps_probe.py.
//
// The camera runs free-streaming (0x0404 = 0, 0x0400 = 1) at the rate from the
// command - usually well above the limit, so that the sensor settles at its maximum.
//
// ⚠ Load to SRAM only (programmer_cli --run 2), do not touch the flash: the main
// bitstream and firmware stay in flash, a cold start brings the board back as it was.
// ============================================================================
`timescale 1ns / 1ps

module fps_probe_top (
    input  wire       CLK,          // 50 MHz, P16
    output wire [4:0] LED,
    output wire       UART2_TXD,
    input  wire       UART2_RXD,
    inout  wire       I2C_SCL,
    inout  wire       I2C_SDA,
    output wire       TWI_MUX_S0,
    output wire       TWI_MUX_S1,
    output wire       TWI_MUX_S2,
    output wire       CAM_TRIG,
    input  wire       CAM_STROBE,
    inout  wire       CAM_CK_P,
    inout  wire       CAM_CK_N,
    inout  wire       CAM_D0_P,
    inout  wire       CAM_D0_N,
    inout  wire       CAM_D1_P,
    inout  wire       CAM_D1_N
);

    localparam integer CLK_HZ = 50_000_000;

    // ---------------------------------------------------------------- reset
    reg [19:0] por_cnt = 20'd0;
    reg        rst_n   = 1'b0;
    always @(posedge CLK) begin
        if (!(&por_cnt)) por_cnt <= por_cnt + 20'd1;
        rst_n <= &por_cnt;
    end

    // Camera 0 on position 3 of the 74HC4051 multiplexer: S0 = 1, S1 = 1, S2 = 0
    // (i2c_mux_select() in the firmware, formerly camera_set_mux; default camera_select_channel).
    assign TWI_MUX_S0 = 1'b1;
    assign TWI_MUX_S1 = 1'b1;
    assign TWI_MUX_S2 = 1'b0;
    assign CAM_TRIG   = 1'b0;      // free-streaming, no external trigger needed

    // -------------------------------------------------- one-second report tick
    reg [25:0] sec_cnt;
    reg        sec_tick;
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            sec_cnt <= 26'd0; sec_tick <= 1'b0;
        end else begin
            sec_tick <= 1'b0;
            if (sec_cnt == CLK_HZ - 1) begin sec_cnt <= 26'd0; sec_tick <= 1'b1; end
            else sec_cnt <= sec_cnt + 26'd1;
        end
    end

    // ------------------------------------------------------------- UART RX
    wire       rx_valid;
    wire [7:0] rx_byte;
    uart_rx #(.DIV(CLK_HZ / 115200)) u_rx (
        .clk(CLK), .rst_n(rst_n), .rx(UART2_RXD), .valid(rx_valid), .data(rx_byte));

    // Commands:
    //   0xA5 + 16 bytes: W, H, FPS (0.01 Hz), EXP (us) - all u32 big-endian.
    //        Set up the camera again and re-apply the receive geometry.
    //   0xA6        : only re-read the camera registers.
    reg [31:0] cfg_w, cfg_h, cfg_fps, cfg_exp;
    reg [7:0]  cmd_buf [0:15];
    reg [4:0]  cmd_cnt;
    reg        cmd_active;
    reg        req_apply, req_read;
    integer    k;
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            cfg_w <= 32'd1080; cfg_h <= 32'd1280;
            cfg_fps <= 32'd300000;          // 3000 frames/s - well above the limit
            cfg_exp <= 32'd200;
            cmd_cnt <= 5'd0; cmd_active <= 1'b0;
            req_apply <= 1'b0; req_read <= 1'b0;
        end else begin
            req_apply <= 1'b0; req_read <= 1'b0;
            if (rx_valid) begin
                if (!cmd_active) begin
                    if (rx_byte == 8'hA5) begin cmd_active <= 1'b1; cmd_cnt <= 5'd0; end
                    else if (rx_byte == 8'hA6) req_read <= 1'b1;
                end else begin
                    cmd_buf[cmd_cnt[3:0]] <= rx_byte;
                    if (cmd_cnt == 5'd15) begin
                        cmd_active <= 1'b0;
                        cfg_w   <= {cmd_buf[0],  cmd_buf[1],  cmd_buf[2],  cmd_buf[3]};
                        cfg_h   <= {cmd_buf[4],  cmd_buf[5],  cmd_buf[6],  cmd_buf[7]};
                        cfg_fps <= {cmd_buf[8],  cmd_buf[9],  cmd_buf[10], cmd_buf[11]};
                        cfg_exp <= {cmd_buf[12], cmd_buf[13], cmd_buf[14], rx_byte};
                        req_apply <= 1'b1;
                    end
                    cmd_cnt <= cmd_cnt + 5'd1;
                end
            end
        end
    end

    // ------------------------------------------------------- I2C: camera
    wire        i2c_busy;
    wire        cam_running;           // capture started (after 0x0400 = 1)
    wire [31:0] rb_id, rb_w, rb_h, rb_fps, rb_maxfps, rb_fps_run, rb_maxfps_run;
    wire [15:0] i2c_nacks;
    wire [7:0]  i2c_step;

    // The first setup runs by itself, half a second after reset.
    reg [25:0] boot_cnt;
    reg        boot_go;
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin boot_cnt <= 26'd0; boot_go <= 1'b0; end
        else begin
            boot_go <= 1'b0;
            if (boot_cnt != CLK_HZ / 2) begin
                boot_cnt <= boot_cnt + 26'd1;
                if (boot_cnt == CLK_HZ / 2 - 1) boot_go <= 1'b1;
            end
        end
    end

    wire i2c_scl_oe, i2c_sda_oe, i2c_scl_in, i2c_sda_in;
    assign I2C_SCL   = i2c_scl_oe ? 1'b0 : 1'bz;
    assign I2C_SDA   = i2c_sda_oe ? 1'b0 : 1'bz;
    assign i2c_scl_in = I2C_SCL;
    assign i2c_sda_in = I2C_SDA;

    cam_i2c_seq #(.CLK_HZ(CLK_HZ)) u_cam (
        .clk(CLK), .rst_n(rst_n),
        .go_apply(req_apply | boot_go), .go_read(req_read),
        .roi_w(cfg_w), .roi_h(cfg_h), .fps(cfg_fps), .exp_us(cfg_exp),
        .scl_oe(i2c_scl_oe), .sda_oe(i2c_sda_oe), .scl_in(i2c_scl_in), .sda_in(i2c_sda_in),
        .busy(i2c_busy), .running(cam_running), .step(i2c_step), .nacks(i2c_nacks),
        .rb_id(rb_id), .rb_w(rb_w), .rb_h(rb_h), .rb_fps(rb_fps), .rb_maxfps(rb_maxfps),
        .rb_fps_run(rb_fps_run), .rb_maxfps_run(rb_maxfps_run));

    // ----------------------------------------------------- D-PHY and CSI-2 bridge
    wire        byte_clk;
    wire [15:0] lane2_data, lane3_data;
    wire        lane2_vld, lane3_vld;

    // Receiver watchdog. The receiver is held in reset until the camera is started
    // and for 100 ms more after the start - the D-PHY must not be released into
    // silence, the aligner sticks on the preamble (ERRORS 2.5). After that: half a
    // second without a single row - a 10 ms reset. The bridge geometry changes only
    // under this reset (it is also held for the whole apply), so the domain
    // decoupling at the cfg_* inputs exists by construction.
    wire line_edge;
    reg  [25:0] wd_quiet;
    reg  [23:0] wd_hold;
    reg  [15:0] wd_resets;
    reg         dphy_rst;
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            wd_quiet <= 26'd0; wd_hold <= 24'd0; wd_resets <= 16'd0; dphy_rst <= 1'b1;
        end else if (!cam_running || i2c_busy) begin
            dphy_rst <= 1'b1;
            wd_quiet <= 26'd0;
            wd_hold  <= CLK_HZ / 10;       // 100 ms more of reset after the start
        end else if (wd_hold != 24'd0) begin
            dphy_rst <= 1'b1;
            wd_hold  <= wd_hold - 24'd1;
            wd_quiet <= 26'd0;
        end else if (line_edge) begin
            dphy_rst <= 1'b0;
            wd_quiet <= 26'd0;
        end else if (wd_quiet == CLK_HZ / 2) begin
            wd_hold   <= CLK_HZ / 100;     // 10 ms
            wd_resets <= wd_resets + 16'd1;
            dphy_rst  <= 1'b1;
        end else begin
            dphy_rst <= 1'b0;
            wd_quiet <= wd_quiet + 26'd1;
        end
    end
    wire dphy_rst_n = rst_n & ~dphy_rst;

    Gowin_MIPI_DPHY_RX4 u_dphy (
        .d0ln_hsrxd(), .d1ln_hsrxd(),
        .d2ln_hsrxd(lane2_data), .d3ln_hsrxd(lane3_data),
        .d0ln_hsrxd_vld(), .d1ln_hsrxd_vld(),
        .d2ln_hsrxd_vld(lane2_vld), .d3ln_hsrxd_vld(lane3_vld),
        .di_lprx0_n(), .di_lprx0_p(), .di_lprx1_n(), .di_lprx1_p(),
        .di_lprx2_n(), .di_lprx2_p(), .di_lprx3_n(), .di_lprx3_p(),
        .di_lprxck_n(), .di_lprxck_p(),
        .rx_clk_o(byte_clk),
        .deskew_error(), .d0ln_deskew_done(), .d1ln_deskew_done(),
        .d2ln_deskew_done(), .d3ln_deskew_done(),
        .ck_p(CAM_CK_P), .ck_n(CAM_CK_N),
        .rx0_p(), .rx0_n(), .rx1_p(), .rx1_n(),
        .rx2_p(CAM_D0_P), .rx2_n(CAM_D0_N),
        .rx3_p(CAM_D1_P), .rx3_n(CAM_D1_N),
        .lprx_en_ck(1'b1), .lprx_en_d0(1'b0), .lprx_en_d1(1'b0),
        .lprx_en_d2(1'b1), .lprx_en_d3(1'b1),
        .hsrx_odten_ck(1'b1), .hsrx_odten_d0(1'b0), .hsrx_odten_d1(1'b0),
        .hsrx_odten_d2(1'b1), .hsrx_odten_d3(1'b1),
        .d0ln_hsrx_dren(1'b0), .d1ln_hsrx_dren(1'b0),
        .d2ln_hsrx_dren(1'b1), .d3ln_hsrx_dren(1'b1),
        .hsrx_en_ck(1'b1),
        // 8-bit mode, as in the main project (cam_rx.v, formerly fpga_camera_ddr3_streamer.v).
        .hs_8bit_mode(1'b1), .rx_clk_1x(1'b1), .rx_invert(1'b0),
        .lalign_en(1'b0), .walign_by(1'b1),
        .do_lptx0_n(1'b0), .do_lptx0_p(1'b0), .do_lptx1_n(1'b0), .do_lptx1_p(1'b0),
        .do_lptx2_n(1'b0), .do_lptx2_p(1'b0), .do_lptx3_n(1'b0), .do_lptx3_p(1'b0),
        .do_lptxck_n(1'b0), .do_lptxck_p(1'b0),
        .lptx_en_ck(1'b0), .lptx_en_d0(1'b0), .lptx_en_d1(1'b0), .lptx_en_d2(1'b0), .lptx_en_d3(1'b0),
        .byte_lendian(1'b0), .hsrx_stop(1'b0),
        .pwron(dphy_rst_n), .reset(~dphy_rst_n),
        .deskew_lnsel(3'd0), .deskew_mth(13'd0), .deskew_owval(7'd0), .deskew_req(1'b0),
        .drst_n(rst_n), .one_byte0_match(1'b0), .word_lendian(1'b0), .fifo_rd_std(3'd0),
        .deskew_by(1'b0), .deskew_en_oedge(1'b0), .deskew_half_opening(6'd0),
        .deskew_lsb_mode(2'd0), .deskew_m(3'd0), .deskew_mset(7'd0), .deskew_oclkedg_en(1'b0),
        .eqcs_lane0(3'b100), .eqcs_lane1(3'b100), .eqcs_lane2(3'b100), .eqcs_lane3(3'b100), .eqcs_ck(3'b100),
        .eqrs_lane0(3'b100), .eqrs_lane1(3'b100), .eqrs_lane2(3'b100), .eqrs_lane3(3'b100), .eqrs_ck(3'b100),
        .hsrx_dlydir_lane0(1'b0), .hsrx_dlydir_lane1(1'b0), .hsrx_dlydir_lane2(1'b0), .hsrx_dlydir_lane3(1'b0), .hsrx_dlydir_ck(1'b0),
        .hsrx_dlyldn_lane0(1'b0), .hsrx_dlyldn_lane1(1'b0), .hsrx_dlyldn_lane2(1'b0), .hsrx_dlyldn_lane3(1'b0), .hsrx_dlyldn_ck(1'b0),
        .hsrx_dlymv_lane0(1'b0), .hsrx_dlymv_lane1(1'b0), .hsrx_dlymv_lane2(1'b0), .hsrx_dlymv_lane3(1'b0), .hsrx_dlymv_ck(1'b0),
        .walign_dvld(1'b0)
    );

    wire        tog_line, tog_hdr, tog_bad;
    wire [15:0] seen_wc;

    camera_csi2_raw8_stream_bridge #(
        .LINE_BYTES(16'd1080), .FRAME_LINES(16'd1280)
    ) u_bridge (
        .byte_clk(byte_clk), .rst_n(dphy_rst_n), .enable(1'b1),
        .cfg_line_bytes(cfg_w[15:0]), .cfg_frame_lines(cfg_h[15:0]),
        .frame_anchor(1'b0),
        .lane0_data(lane2_data), .lane1_data(lane3_data),
        .lane0_valid(lane2_vld), .lane1_valid(lane3_vld),
        .stream_sp_en(), .stream_lp_av_en(), .stream_dt(), .stream_wc(),
        .stream_payload_dv(), .stream_payload(),
        .raw8_header_toggle(tog_hdr), .raw8_line_ok_toggle(tog_line),
        .raw8_bad_wc_toggle(tog_bad), .frame_done_toggle(),
        .pixel_seen(), .overflow_seen(),
        .raw8_header_count(), .raw8_line_count(), .raw8_bad_wc_count(),
        .last_word_count(), .last_line_byte_count(), .last_payload_word(),
        .last_lane_swap(), .last_phase_high(), .line_index(),
        .seen_wc_last(seen_wc), .seen_wc_hits_expected(), .seen_wc_hits_alt()
    );

    // Byte clock: a divider in its domain, edges of the top bit are counted on CLK.
    reg [9:0] bclk_div = 10'd0;
    always @(posedge byte_clk) bclk_div <= bclk_div + 10'd1;

    // Synchronisation of one-bit signals into CLK and edge counting per second.
    reg [2:0] s_line, s_hdr, s_bad, s_bclk, s_stb;
    always @(posedge CLK) begin
        s_line <= {s_line[1:0], tog_line};
        s_hdr  <= {s_hdr[1:0],  tog_hdr};
        s_bad  <= {s_bad[1:0],  tog_bad};
        s_bclk <= {s_bclk[1:0], bclk_div[9]};
        s_stb  <= {s_stb[1:0],  CAM_STROBE};
    end
    assign line_edge = s_line[2] ^ s_line[1];
    wire hdr_edge  = s_hdr[2]  ^ s_hdr[1];
    wire bad_edge  = s_bad[2]  ^ s_bad[1];
    wire bclk_edge = s_bclk[2] & ~s_bclk[1];
    wire stb_rise  = ~s_stb[2] & s_stb[1];
    wire stb_fall  = s_stb[2] & ~s_stb[1];

    reg [31:0] n_line, n_hdr, n_bad, n_bclk, n_stb;
    reg [31:0] r_line, r_hdr, r_bad, r_bclk, r_stb;
    reg [31:0] stb_per_cnt, stb_per, stb_hi_cnt, stb_hi;
    reg [15:0] wc_s0, wc_s1, r_wc;
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            n_line <= 0; n_hdr <= 0; n_bad <= 0; n_bclk <= 0; n_stb <= 0;
            r_line <= 0; r_hdr <= 0; r_bad <= 0; r_bclk <= 0; r_stb <= 0;
            stb_per_cnt <= 0; stb_per <= 0; stb_hi_cnt <= 0; stb_hi <= 0;
            wc_s0 <= 0; wc_s1 <= 0; r_wc <= 0;
        end else begin
            wc_s0 <= seen_wc; wc_s1 <= wc_s0;
            if (wc_s1 == wc_s0) r_wc <= wc_s1;       // two equal samples in a row

            // The strobe period is the frame time directly, bypassing MIPI entirely.
            stb_per_cnt <= stb_per_cnt + 32'd1;
            if (stb_rise) begin stb_per <= stb_per_cnt; stb_per_cnt <= 32'd0; end
            if (s_stb[1]) stb_hi_cnt <= stb_hi_cnt + 32'd1;
            if (stb_fall) begin stb_hi <= stb_hi_cnt; end
            if (stb_rise) stb_hi_cnt <= 32'd0;

            if (sec_tick) begin
                r_line <= n_line; r_hdr <= n_hdr; r_bad <= n_bad; r_bclk <= n_bclk; r_stb <= n_stb;
                n_line <= {31'd0, line_edge}; n_hdr <= {31'd0, hdr_edge}; n_bad <= {31'd0, bad_edge};
                n_bclk <= {31'd0, bclk_edge}; n_stb <= {31'd0, stb_rise};
            end else begin
                n_line <= n_line + line_edge; n_hdr <= n_hdr + hdr_edge; n_bad <= n_bad + bad_edge;
                n_bclk <= n_bclk + bclk_edge; n_stb <= n_stb + stb_rise;
            end
        end
    end

    // ---------------------------------------------------------- UART report
    // Report frame: 0x5A 0xA5, then NW u32 words big-endian, then a byte sum of
    // all bytes of the words. Word order is in tools/camera/fps_probe.py (WORDS).
    localparam integer NW = 24;
    reg [31:0] rep_seq;
    function [31:0] rep_word(input [4:0] i);
        case (i)
            5'd0:  rep_word = rep_seq;
            5'd1:  rep_word = cfg_w;
            5'd2:  rep_word = cfg_h;
            5'd3:  rep_word = cfg_fps;
            5'd4:  rep_word = cfg_exp;
            5'd5:  rep_word = r_stb;          // strobes per second
            5'd6:  rep_word = stb_per;        // strobe period, 50 MHz cycles
            5'd7:  rep_word = stb_hi;         // strobe width, 50 MHz cycles
            5'd8:  rep_word = r_line;         // complete rows per second
            5'd9:  rep_word = r_hdr;          // headers accepted per second
            5'd10: rep_word = r_bad;          // candidates rejected (undercounted)
            5'd11: rep_word = {16'd0, r_wc};  // word count of the last candidate
            5'd12: rep_word = r_bclk;         // byte_clk/1024 edges per second
            5'd13: rep_word = rb_id;          // 0x0000, "VEYE" expected
            5'd14: rep_word = rb_w;           // 0x0818 after the write
            5'd15: rep_word = rb_h;           // 0x081C
            5'd16: rep_word = rb_fps;         // 0x0814 before the start
            5'd17: rep_word = rb_maxfps;      // 0x0838 before the start
            5'd18: rep_word = rb_fps_run;     // 0x0814 200 ms after the start
            5'd19: rep_word = rb_maxfps_run;  // 0x0838 200 ms after the start
            5'd20: rep_word = {16'd0, i2c_nacks};
            5'd21: rep_word = {16'd0, wd_resets};
            5'd22: rep_word = {22'd0, i2c_step, cam_running, i2c_busy};
            default: rep_word = 32'd0;
        endcase
    endfunction

    reg        tx_go;
    reg  [7:0] tx_data;
    wire       tx_busy;
    uart_tx #(.DIV(CLK_HZ / 115200)) u_tx (
        .clk(CLK), .rst_n(rst_n), .go(tx_go), .data(tx_data), .busy(tx_busy), .tx(UART2_TXD));

    reg  [7:0] rp_idx;          // 0,1 - header, 2..2+4*NW-1 - words, then the sum
    reg        rp_active;
    reg  [7:0] rp_sum;
    reg [31:0] rp_word;
    wire [7:0] rp_rel  = rp_idx - 8'd2;     // byte number within the words
    wire [4:0] rp_widx = rp_rel[6:2];       // word number
    wire [1:0] rp_bsel = rp_rel[1:0];       // byte within the word, 0 - most significant
    wire [31:0] rp_cur = rep_word(rp_widx);
    always @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            rep_seq <= 0; rp_idx <= 0; rp_active <= 0; rp_sum <= 0; tx_go <= 0; tx_data <= 0; rp_word <= 0;
        end else begin
            tx_go <= 1'b0;
            if (!rp_active) begin
                if (sec_tick) begin rp_active <= 1'b1; rp_idx <= 8'd0; rp_sum <= 8'd0; rep_seq <= rep_seq + 1; end
            end else if (!tx_busy && !tx_go) begin
                if (rp_idx == 8'd0) begin
                    tx_data <= 8'h5A; tx_go <= 1'b1; rp_idx <= rp_idx + 1;
                end else if (rp_idx == 8'd1) begin
                    tx_data <= 8'hA5; tx_go <= 1'b1; rp_idx <= rp_idx + 1;
                end else if (rp_idx < 8'd2 + 4 * NW) begin
                    // The word is latched on its first byte, so that all four
                    // bytes come from one value.
                    if (rp_bsel == 2'd0) begin
                        rp_word <= rp_cur;
                        tx_data <= rp_cur[31:24];
                        rp_sum  <= rp_sum + rp_cur[31:24];
                    end else begin
                        case (rp_bsel)
                            2'd1: begin tx_data <= rp_word[23:16]; rp_sum <= rp_sum + rp_word[23:16]; end
                            2'd2: begin tx_data <= rp_word[15:8];  rp_sum <= rp_sum + rp_word[15:8];  end
                            default: begin tx_data <= rp_word[7:0]; rp_sum <= rp_sum + rp_word[7:0]; end
                        endcase
                    end
                    tx_go  <= 1'b1;
                    rp_idx <= rp_idx + 1;
                end else begin
                    tx_data <= rp_sum; tx_go <= 1'b1; rp_active <= 1'b0;
                end
            end
        end
    end

    // --------------------------------------------------------------- LED
    reg hb;
    always @(posedge CLK) if (sec_tick) hb <= ~hb;
    assign LED[0] = hb;                        // alive
    assign LED[1] = (r_line != 0);             // rows are coming
    assign LED[2] = (i2c_nacks != 0);          // I2C got a NACK
    assign LED[3] = (r_stb != 0);              // strobe is coming
    assign LED[4] = i2c_busy;                  // camera setup in progress

endmodule
