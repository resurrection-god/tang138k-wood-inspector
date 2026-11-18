// ============================================================================
// Module      : hdmi_mono_vision
// Stage 5 Hikrobot Mono8 -> 720p60 HDMI Video, Sobel 3x3 & Telemetry HUD Core.
//
// Features:
//   - APB slave interface (0xF8000800..0xFFF) compatible with AE350 firmware.
//   - True dual-bank ping-pong 360x270 x 8-bit framebuffer (96 Gowin SDPB).
//     Write clock = sys_clk (100 MHz); Read clock = video_clk (74 MHz).
//     Tear-free VSYNC page flip synchronized via 2FF toggle handshake.
//   - 1280x720 @ 60.016 Hz timing (74.0 MHz pixel clock: 1644 x 750).
//   - Left 960x720 viewport: exact 8/3 upscale of 360x270 with real-time 3x3
//     Sobel edge detector computed per source row during blanking/sidebar pass
//     (hx = 960..1330), zero bank contention, full edge clamping.
//   - Visual modes (cfg_algo_mode):
//       0: Mono8 + Neon Cyan Sobel Edge Overlay + Crosshair/Fiducials
//       1: Pure Sobel Gradient Heatmap
//       2: Clean Unprocessed Mono8
//       3: Hardware Test Pattern (0..255 Ramp + Checker + Sobel Calibration Boxes)
//   - Right 320x720 sidebar: live 16x16 crisp bitmap HUD (FPS, PKTS, DROPS,
//     RES, ALGO, THR, EDGE%, LUMA MIN/MAX/AVG, 16-bin histogram bargraph).
//   - Drives BOTH Tang Mega 138K Pro HDMI connectors identically via svo_tmds
//     + OSER10 + ELVDS_OBUF.
// ============================================================================

`timescale 1ns / 1ps

module dp_fb_16k8 (
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [13:0] wr_addr,
    input  wire [7:0]  wr_data,
    input  wire        rd_clk,
    input  wire [13:0] rd_addr,
    output reg  [7:0]  rd_data
);
    (* syn_ramstyle = "block_ram" *) reg [7:0] mem [0:16383];
    always @(posedge wr_clk) begin
        if (wr_en) mem[wr_addr] <= wr_data;
    end
    always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
    end
endmodule

module fb_bank_360x270 (
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [16:0] wr_addr,
    input  wire [7:0]  wr_data,
    input  wire        rd_clk,
    input  wire [16:0] rd_addr,
    output reg  [7:0]  rd_data
);
    wire [7:0] slice_q [0:5];
    genvar i;
    generate
        for (i = 0; i < 6; i = i + 1) begin : gen_slices
            wire we_i = wr_en && (wr_addr[16:14] == i[2:0]);
            dp_fb_16k8 u_slice (
                .wr_clk  (wr_clk),
                .wr_en   (we_i),
                .wr_addr (wr_addr[13:0]),
                .wr_data (wr_data),
                .rd_clk  (rd_clk),
                .rd_addr (rd_addr[13:0]),
                .rd_data (slice_q[i])
            );
        end
    endgenerate

    reg [2:0] rd_sel_q;
    always @(posedge rd_clk) begin
        rd_sel_q <= rd_addr[16:14];
        case (rd_sel_q)
            3'd0: rd_data <= slice_q[0];
            3'd1: rd_data <= slice_q[1];
            3'd2: rd_data <= slice_q[2];
            3'd3: rd_data <= slice_q[3];
            3'd4: rd_data <= slice_q[4];
            3'd5: rd_data <= slice_q[5];
            default: rd_data <= 8'd0;
        endcase
    end
endmodule

// Small 512x8 line buffer (SSRAM/BSRAM)
module lb_512x8 (
    input  wire       clk,
    input  wire       wr_en,
    input  wire [8:0] wr_addr,
    input  wire [7:0] wr_data,
    input  wire [8:0] rd_addr,
    output reg  [7:0] rd_data
);
    (* syn_ramstyle = "block_ram" *) reg [7:0] mem [0:511];
    always @(posedge clk) begin
        if (wr_en) mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
    end
endmodule

// Display line buffer 512x16: {grad[7:0], luma[7:0]}
module lb_512x16 (
    input  wire        clk,
    input  wire        wr_en,
    input  wire [8:0]  wr_addr,
    input  wire [15:0] wr_data,
    input  wire [8:0]  rd_addr,
    output reg  [15:0] rd_data
);
    (* syn_ramstyle = "block_ram" *) reg [15:0] mem [0:511];
    always @(posedge clk) begin
        if (wr_en) mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
    end
endmodule

module hdmi_mono_vision #(
    parameter integer SRC_W = 360,
    parameter integer SRC_H = 270
)(
    input  wire        clk,             // 100 MHz AHB_CLK / clk_x1
    input  wire        rst_n,

    // Video clocks for HDMI 720p60
    input  wire        video_clk,       // 74.0 MHz pixel clock
    input  wire        serial_clk,      // 370.0 MHz 5x TMDS SERDES clock
    input  wire        video_rst_n,

    // APB3 Slave (0xF8000800..0xFFF)
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [10:0] paddr,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output reg         pready,

    // Streaming input from gvsp_rx_hdmi (clk domain)
    input  wire        hw_valid,
    input  wire [1:0]  hw_op,           // 2'b01=SOF, 2'b00=PIX, 2'b10=EOF
    input  wire [8:0]  hw_y,
    input  wire [8:0]  hw_x,
    input  wire [7:0]  hw_data,

    // Telemetry from gvsp_rx_hdmi (clk domain)
    input  wire [15:0] hw_stat_frames,
    input  wire [15:0] hw_stat_pkts,
    input  wire [15:0] hw_stat_drops,
    input  wire [15:0] hw_stat_w,
    input  wire [15:0] hw_stat_h,
    output reg         cfg_hw_rx_en,
    output reg  [15:0] cfg_hw_rx_port,

    // HDMI TMDS output (Port 0, Bank 2 HCLK local)
    output wire        tmds_clk_p_0,
    output wire        tmds_clk_n_0,
    output wire [2:0]  tmds_d_p_0,
    output wire [2:0]  tmds_d_n_0
);

    localparam [31:0] MAGIC = 32'h4844_4D49; // "HDMI"

    // ------------------------------------------------------------------------
    // 1. APB Register Interface (clk 100 MHz)
    // ------------------------------------------------------------------------
    reg        q_active, q_write;
    reg [8:0]  q_addr;
    reg [31:0] q_wdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_active <= 1'b0;
            q_write  <= 1'b0;
            q_addr   <= 9'd0;
            q_wdata  <= 32'd0;
            pready   <= 1'b0;
        end else begin
            pready <= 1'b0;
            if (psel && !penable && !q_active) begin
                q_active <= 1'b1;
                q_write  <= pwrite;
                q_addr   <= paddr[10:2];
                q_wdata  <= pwdata;
            end else if (q_active) begin
                q_active <= 1'b0;
                pready   <= 1'b1;
            end
        end
    end

    wire apb_wr = q_active &&  q_write;
    wire apb_rd = q_active && !q_write;

    reg        cfg_enable;
    reg        cfg_overlay_en;
    reg        cfg_invert;
    reg [1:0]  cfg_algo_mode;   // 0=Mono8+Sobel, 1=SobelMap, 2=RawMono8, 3=TestPat
    reg [7:0]  cfg_sobel_thresh;
    reg [1:0]  cfg_rot;         // 0=normal, 1=Hflip, 2=Vflip, 3=180
    reg        cfg_link_ok;
    reg [31:0] cfg_cam_ip;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_enable       <= 1'b1;
            cfg_overlay_en   <= 1'b1;
            cfg_invert       <= 1'b0;
            cfg_algo_mode    <= 2'd0;
            cfg_sobel_thresh <= 8'd36;
            cfg_rot          <= 2'd0;
            cfg_link_ok      <= 1'b0;
            cfg_cam_ip       <= 32'h0A00_0005;
            cfg_hw_rx_en     <= 1'b0;
            cfg_hw_rx_port   <= 16'd5557;
        end else if (apb_wr) begin
            case (q_addr)
                9'd1: begin
                    cfg_enable       <= q_wdata[0];
                    cfg_overlay_en   <= q_wdata[1];
                    cfg_invert       <= q_wdata[2];
                    cfg_algo_mode    <= q_wdata[4:3];
                    cfg_sobel_thresh <= (q_wdata[12:5] != 8'd0) ? q_wdata[12:5] : 8'd36;
                    cfg_rot          <= q_wdata[14:13];
                    cfg_link_ok      <= q_wdata[15];
                end
                9'd8:  cfg_hw_rx_en   <= q_wdata[0];
                9'd9:  cfg_hw_rx_port <= q_wdata[15:0];
                9'd14: cfg_cam_ip     <= q_wdata;
                default: ;
            endcase
        end
    end

    // Synced telemetry readback from video_clk
    (* ASYNC_REG = "TRUE" *) reg [15:0] v_f_sync1, v_f_sync2;
    (* ASYNC_REG = "TRUE" *) reg [15:0] cfps_sync1, cfps_sync2;
    (* ASYNC_REG = "TRUE" *) reg [15:0] edge_pct_sync1, edge_pct_sync2;
    reg [15:0] hdmi_frame_cnt;
    reg [15:0] meas_cam_fps_x10;
    reg [15:0] meas_edge_pct_x10;

    always @(posedge clk) begin
        v_f_sync1      <= hdmi_frame_cnt;
        v_f_sync2      <= v_f_sync1;
        cfps_sync1     <= meas_cam_fps_x10;
        cfps_sync2     <= cfps_sync1;
        edge_pct_sync1 <= meas_edge_pct_x10;
        edge_pct_sync2 <= edge_pct_sync1;
    end

    always @(posedge clk) begin
        if (apb_rd) begin
            case (q_addr)
                9'd0:  prdata <= MAGIC;
                9'd1:  prdata <= {16'd0, cfg_link_ok, cfg_rot, cfg_sobel_thresh, cfg_algo_mode, cfg_invert, cfg_overlay_en, cfg_enable};
                9'd2:  prdata <= {v_f_sync2, 1'b0, cfg_rot, cfg_sobel_thresh, cfg_algo_mode, cfg_invert, cfg_overlay_en, cfg_enable};
                9'd8:  prdata <= {31'd0, cfg_hw_rx_en};
                9'd9:  prdata <= {16'd0, cfg_hw_rx_port};
                9'd10: prdata <= {hw_stat_frames, hw_stat_pkts};
                9'd11: prdata <= {hw_stat_h, hw_stat_w};
                9'd12: prdata <= {16'd0, hw_stat_drops};
                9'd13: prdata <= {edge_pct_sync2, cfps_sync2};
                9'd14: prdata <= cfg_cam_ip;
                default: prdata <= 32'd0;
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // 2. Write Side + Ping-Pong Bank Controller (clk 100 MHz)
    // ------------------------------------------------------------------------
    reg        wr_bank;
    reg        wr_accept;
    reg        wr_has_pix;
    reg        wr_hold;
    reg        wr_swap_req_tgl;

    (* ASYNC_REG = "TRUE" *) reg [2:0] rd_ack_sync;
    wire rd_swap_ack_q2 = rd_ack_sync[2];

    // Linear address y*360 + x = ((y*45) << 3) + x, registered 1 cycle cleanly
    reg        wr_pix_we_q;
    reg [16:0] wr_pix_addr_q;
    reg [7:0]  wr_pix_data_q;
    reg        wr_pix_bank_q;

    wire [16:0] y_times_45 = ({8'd0, hw_y} << 5) + ({8'd0, hw_y} << 3) + ({8'd0, hw_y} << 2) + {8'd0, hw_y};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_bank         <= 1'b0;
            wr_accept       <= 1'b0;
            wr_has_pix      <= 1'b0;
            wr_hold         <= 1'b0;
            wr_swap_req_tgl <= 1'b0;
            rd_ack_sync     <= 3'b000;
            wr_pix_we_q     <= 1'b0;
            wr_pix_addr_q   <= 17'd0;
            wr_pix_data_q   <= 8'd0;
            wr_pix_bank_q   <= 1'b0;
        end else begin
            rd_ack_sync <= {rd_ack_sync[1:0], rd_swap_ack_tgl};
            wr_pix_we_q <= 1'b0;

            if (wr_hold && (rd_swap_ack_q2 == wr_swap_req_tgl)) begin
                wr_hold <= 1'b0;
                wr_bank <= ~wr_bank;
            end

            if (hw_valid) begin
                if (hw_op == 2'b01) begin
                    // SOF: accept only if current wr_bank is free (not waiting for VSYNC swap)
                    wr_accept  <= !wr_hold;
                    wr_has_pix <= 1'b0;
                end else if (hw_op == 2'b00 && wr_accept && (hw_x < SRC_W[8:0]) && (hw_y < SRC_H[8:0])) begin
                    wr_has_pix    <= 1'b1;
                    wr_pix_we_q   <= 1'b1;
                    wr_pix_bank_q <= wr_bank;
                    wr_pix_addr_q <= (y_times_45 << 3) + {8'd0, hw_x};
                    wr_pix_data_q <= hw_data;
                end else if (hw_op == 2'b10 && wr_accept) begin
                    wr_accept <= 1'b0;
                    if (wr_has_pix) begin
                        wr_hold         <= 1'b1;
                        wr_swap_req_tgl <= ~wr_swap_req_tgl;
                    end
                end
            end
        end
    end

    // Dual-bank 360x270 x 8-bit SDPB Framebuffers (96 BSRAM total)
    reg [16:0] fb_rd_addr_v;
    wire [7:0] fb0_rd_data_v, fb1_rd_data_v;
    reg        rd_bank_v;

    fb_bank_360x270 u_fb_bank0 (
        .wr_clk  (clk),
        .wr_en   (wr_pix_we_q && !wr_pix_bank_q),
        .wr_addr (wr_pix_addr_q),
        .wr_data (wr_pix_data_q),
        .rd_clk  (video_clk),
        .rd_addr (fb_rd_addr_v),
        .rd_data (fb0_rd_data_v)
    );

    fb_bank_360x270 u_fb_bank1 (
        .wr_clk  (clk),
        .wr_en   (wr_pix_we_q &&  wr_pix_bank_q),
        .wr_addr (wr_pix_addr_q),
        .wr_data (wr_pix_data_q),
        .rd_clk  (video_clk),
        .rd_addr (fb_rd_addr_v),
        .rd_data (fb1_rd_data_v)
    );

    wire [7:0] fb_rd_data_v = rd_bank_v ? fb1_rd_data_v : fb0_rd_data_v;

    // ------------------------------------------------------------------------
    // 3. Synchronize Config & Telemetry Snapshot into video_clk (74 MHz)
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0]  v_en_sync, v_ov_sync, v_inv_sync, v_lnk_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0]  v_algo_s1, v_algo_s2, v_rot_s1, v_rot_s2;
    (* ASYNC_REG = "TRUE" *) reg [7:0]  v_thr_s1, v_thr_s2;
    (* ASYNC_REG = "TRUE" *) reg [15:0] v_pkts_s1, v_pkts_s2, v_drops_s1, v_drops_s2;
    (* ASYNC_REG = "TRUE" *) reg [15:0] v_w_s1, v_w_s2, v_h_s1, v_h_s2;
    (* ASYNC_REG = "TRUE" *) reg [31:0] v_ip_s1, v_ip_s2;
    (* ASYNC_REG = "TRUE" *) reg [2:0]  wr_req_sync_v;

    reg rd_swap_ack_tgl;

    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            v_en_sync     <= 2'b00; v_ov_sync <= 2'b00; v_inv_sync <= 2'b00; v_lnk_sync <= 2'b00;
            v_algo_s1     <= 2'd0;  v_algo_s2 <= 2'd0;  v_rot_s1 <= 2'd0;   v_rot_s2 <= 2'd0;
            v_thr_s1      <= 8'd36; v_thr_s2  <= 8'd36;
            v_pkts_s1     <= 16'd0; v_pkts_s2 <= 16'd0; v_drops_s1 <= 16'd0; v_drops_s2 <= 16'd0;
            v_w_s1        <= 16'd0; v_w_s2    <= 16'd0; v_h_s1 <= 16'd0;     v_h_s2 <= 16'd0;
            v_ip_s1       <= 32'd0; v_ip_s2   <= 32'd0;
            wr_req_sync_v <= 3'b000;
        end else begin
            v_en_sync     <= {v_en_sync[0],  cfg_enable};
            v_ov_sync     <= {v_ov_sync[0],  cfg_overlay_en};
            v_inv_sync    <= {v_inv_sync[0], cfg_invert};
            v_lnk_sync    <= {v_lnk_sync[0], cfg_link_ok};
            v_algo_s1     <= cfg_algo_mode;    v_algo_s2  <= v_algo_s1;
            v_rot_s1      <= cfg_rot;          v_rot_s2   <= v_rot_s1;
            v_thr_s1      <= cfg_sobel_thresh; v_thr_s2   <= v_thr_s1;
            v_pkts_s1     <= hw_stat_pkts;     v_pkts_s2  <= v_pkts_s1;
            v_drops_s1    <= hw_stat_drops;    v_drops_s2 <= v_drops_s1;
            v_w_s1        <= hw_stat_w;        v_w_s2     <= v_w_s1;
            v_h_s1        <= hw_stat_h;        v_h_s2     <= v_h_s1;
            v_ip_s1       <= cfg_cam_ip;       v_ip_s2    <= v_ip_s1;
            wr_req_sync_v <= {wr_req_sync_v[1:0], wr_swap_req_tgl};
        end
    end

    wire       v_enable  = v_en_sync[1];
    wire       v_overlay = v_ov_sync[1];
    wire       v_invert  = v_inv_sync[1];
    wire       v_link_ok = v_lnk_sync[1];
    wire [1:0] v_algo    = v_algo_s2;
    wire [1:0] v_rot     = v_rot_s2;
    wire [7:0] v_thresh  = v_thr_s2;

    // ------------------------------------------------------------------------
    // 4. HDMI 720p60 Timing Generator (1648 x 750 @ 74.167 MHz = 60.005 Hz)
    //    Active area: x = 0..1279, y = 0..719
    //    H_FP=108 (1280..1387), H_SYNC=40 (1388..1427), H_BP=220 (1428..1647)
    //    V_FP=5   (720..724),   V_SYNC=5  (725..729),   V_BP=20  (730..749)
    // ------------------------------------------------------------------------
    localparam [11:0] H_ACTIVE = 12'd1280;
    localparam [11:0] H_FP     = 12'd108;
    localparam [11:0] H_SYNC   = 12'd40;
    localparam [11:0] H_TOTAL  = 12'd1648;
    localparam [9:0]  V_ACTIVE = 10'd720;
    localparam [9:0]  V_FP     = 10'd5;
    localparam [9:0]  V_SYNC   = 10'd5;
    localparam [9:0]  V_TOTAL  = 10'd750;

    reg [11:0] h_cnt;
    reg [9:0]  v_cnt;
    reg [8:0]  cur_sy;
    reg [3:0]  acc_y;
    reg        need_prefetch_next;

    // Camera FPS counter measured over 60 HDMI frames (~1.0 s window)
    reg [5:0]  fps_sec_cnt;
    reg [7:0]  cam_flip_acc;
    reg        cam_frame_seen;

    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            h_cnt              <= 12'd0;
            v_cnt              <= 10'd0;
            cur_sy             <= 9'd0;
            acc_y              <= 4'd0;
            need_prefetch_next <= 1'b0;
            rd_bank_v          <= 1'b1;
            rd_swap_ack_tgl    <= 1'b0;
            hdmi_frame_cnt     <= 16'd0;
            fps_sec_cnt        <= 6'd0;
            cam_flip_acc       <= 8'd0;
            meas_cam_fps_x10   <= 16'd0;
            cam_frame_seen     <= 1'b0;
        end else begin
            if (h_cnt == (H_TOTAL - 12'd1)) begin
                h_cnt <= 12'd0;
                if (v_cnt == (V_TOTAL - 10'd1)) begin
                    v_cnt  <= 10'd0;
                    cur_sy <= 9'd0;
                    acc_y  <= 4'd0;
                    need_prefetch_next <= 1'b0;
                end else begin
                    v_cnt <= v_cnt + 10'd1;
                    if (v_cnt < (V_ACTIVE - 10'd1)) begin
                        if ((acc_y + 4'd3) >= 4'd8) begin
                            acc_y  <= acc_y + 4'd3 - 4'd8;
                            cur_sy <= cur_sy + 9'd1;
                            // Compute whether line AFTER next (v_cnt+1) will step
                            need_prefetch_next <= ((acc_y + 4'd3 - 4'd8 + 4'd3) >= 4'd8);
                        end else begin
                            acc_y <= acc_y + 4'd3;
                            need_prefetch_next <= ((acc_y + 4'd3 + 4'd3) >= 4'd8);
                        end
                    end else begin
                        need_prefetch_next <= 1'b0;
                    end
                end
            end else begin
                h_cnt <= h_cnt + 12'd1;
                if (v_cnt == 10'd0 && h_cnt == 12'd0) begin
                    need_prefetch_next <= 1'b0; // acc_y=0 -> v_cnt=1 stays at sy=0
                end
            end

            // Tear-free VSYNC bank swap at v_cnt == 720, h_cnt == 0
            if ((v_cnt == V_ACTIVE) && (h_cnt == 12'd0)) begin
                hdmi_frame_cnt <= hdmi_frame_cnt + 16'd1;
                if (wr_req_sync_v[2] != rd_swap_ack_tgl) begin
                    rd_bank_v       <= ~rd_bank_v;
                    rd_swap_ack_tgl <= wr_req_sync_v[2];
                    cam_flip_acc    <= cam_flip_acc + 8'd1;
                    cam_frame_seen  <= 1'b1;
                end
                if (fps_sec_cnt == 6'd59) begin
                    fps_sec_cnt      <= 6'd0;
                    // Exact 60.016 Hz window -> report cam_fps_x10 (~600 for 60 Hz)
                    meas_cam_fps_x10 <= ({5'd0, cam_flip_acc, 3'b000}) + ({7'd0, cam_flip_acc, 1'b0});
                    cam_flip_acc     <= 8'd0;
                end else begin
                    fps_sec_cnt <= fps_sec_cnt + 6'd1;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // 5. Row Prefetch & 3x3 Sobel Engine (runs during v=747,748,749 & hx=960..1330)
    //    Line buffers: lb0 = row(sy-1), lb1 = row(sy)
    //    Read from rd_bank: row(min(sy+1, 269))
    //    Pipeline:
    //      t+0: issue addresses sx_run (0..360)
    //      t+1: slice sel (fb_bank_360x270) / lb read wait
    //      t+2: fb_rd_data_v & lb0_q & lb1_q valid; write lb0<=lb1_q, lb1<=fb_rd_data_v
    //      t+3: 3x3 shift window populated; clamp cols 0 & 359
    //      t+4: Gx, Gy computed (signed 11-bit)
    //      t+5: |Gx|+|Gy|>>2 clamped to [0..255], write disp_line[col]
    // ------------------------------------------------------------------------
    reg        lb0_we, lb1_we, disp_we;
    reg [8:0]  lb_wa, lb_ra, disp_wa;
    reg [7:0]  lb0_wd, lb1_wd;
    wire [7:0] lb0_q, lb1_q;
    reg [15:0] disp_wd;
    reg [8:0]  disp_ra;
    wire [15:0] disp_q;

    lb_512x8  u_lb0 (.clk(video_clk), .wr_en(lb0_we), .wr_addr(lb_wa), .wr_data(lb0_wd), .rd_addr(lb_ra), .rd_data(lb0_q));
    lb_512x8  u_lb1 (.clk(video_clk), .wr_en(lb1_we), .wr_addr(lb_wa), .wr_data(lb1_wd), .rd_addr(lb_ra), .rd_data(lb1_q));
    lb_512x16 u_disp (.clk(video_clk), .wr_en(disp_we), .wr_addr(disp_wa), .wr_data(disp_wd), .rd_addr(disp_ra), .rd_data(disp_q));

    // Decide when prefetch + Sobel runs for row target_sy
    wire pass_v747 = (v_cnt == 10'd747); // preload lb0 = row0, lb1 = row0
    wire pass_v748 = (v_cnt == 10'd748); // preload row1 + Sobel sy=0
    wire pass_step = (v_cnt < (V_ACTIVE - 10'd1)) && need_prefetch_next;
    wire pass_en   = pass_v747 || pass_v748 || pass_step;

    wire [11:0] pf_start_h = (v_cnt >= V_ACTIVE) ? 12'd0 : 12'd960;
    wire [8:0]  pf_sy_cur  = (v_cnt >= V_ACTIVE) ? 9'd0 : (cur_sy + 9'd1);
    wire [8:0]  pf_rd_row  = pass_v747 ? 9'd0 :
                             (pf_sy_cur < (SRC_H[8:0] - 9'd1)) ? (pf_sy_cur + 9'd1) : (SRC_H[8:0] - 9'd1);

    wire [11:0] pf_rel = h_cnt - pf_start_h;
    wire        pf_win = pass_en && (h_cnt >= pf_start_h) && (pf_rel < 12'd366);

    // Bank read address generation with horizontal/vertical rotation applied
    wire [8:0]  pf_col_raw  = (pf_rel < SRC_W[11:0]) ? pf_rel[8:0] : (SRC_W[8:0] - 9'd1);
    wire [8:0]  pf_col_rot  = (v_rot == 2'd1 || v_rot == 2'd3) ? (SRC_W[8:0] - 9'd1 - pf_col_raw) : pf_col_raw;
    wire [8:0]  pf_row_rot  = (v_rot == 2'd2 || v_rot == 2'd3) ? (SRC_H[8:0] - 9'd1 - pf_rd_row)  : pf_rd_row;
    wire [16:0] pf_row_mul45 = ({8'd0, pf_row_rot} << 5) + ({8'd0, pf_row_rot} << 3) + ({8'd0, pf_row_rot} << 2) + {8'd0, pf_row_rot};

    // Pipeline stage trackers for prefetch
    reg [8:0] pf_sx_d1, pf_sx_d2, pf_sx_d3, pf_sx_d4, pf_sx_d5;
    reg       pf_v_d1,  pf_v_d2,  pf_v_d3,  pf_v_d4;
    reg       pf_747_d1, pf_747_d2;
    reg       pf_sobel_d2, pf_sobel_d3, pf_sobel_d4;

    // Raw fetched pixels or test pattern at d2
    wire [7:0] tp_pix_d2 = ((pf_sx_d2[4] ^ pf_sy_cur[4]) && (pf_sx_d2 > 9'd40 && pf_sx_d2 < 9'd320 && pf_sy_cur > 9'd40 && pf_sy_cur < 9'd230))
                           ? 8'd230 : (pf_sx_d2[7:0]);
    wire [7:0] src_pix_raw = (v_algo == 2'd3) ? tp_pix_d2 : (cam_frame_seen ? fb_rd_data_v : tp_pix_d2);
    wire [7:0] bot_pix_d2  = v_invert ? ~src_pix_raw : src_pix_raw;

    // 3x3 shift window registers (updated on pf_sobel_d2)
    reg [7:0] r0c0, r0c1, r0c2;
    reg [7:0] r1c0, r1c1, r1c2;
    reg [7:0] r2c0, r2c1, r2c2;

    // Sobel stage 1 & 2
    reg signed [10:0] gx_q, gy_q;
    reg [7:0]         center_q1, center_q2;
    wire [10:0] abs_gx = gx_q[10] ? (~gx_q + 11'd1) : gx_q;
    wire [10:0] abs_gy = gy_q[10] ? (~gy_q + 11'd1) : gy_q;
    wire [11:0] g_sum  = {1'b0, abs_gx} + {1'b0, abs_gy};
    wire [9:0]  g_div4 = g_sum[11:2];
    wire [7:0]  grad_clamped = (|g_div4[9:8]) ? 8'd255 : g_div4[7:0];

    // Live per-frame image statistics (computed across the 270 Sobel passes)
    reg [7:0]  acc_min, acc_max;
    reg [25:0] acc_sum, acc_sum_snap;
    reg [16:0] acc_edge_cnt;
    reg [15:0] acc_hist [0:15];

    reg [7:0]  hud_min, hud_max, hud_avg;
    reg [15:0] hud_hist [0:15];
    reg [15:0] hud_pkts, hud_drops, hud_w, hud_h;
    reg [31:0] hud_ip;
    reg        hud_link_ok;
    reg [7:0]  hud_thresh;
    reg [1:0]  hud_algo;
    reg [15:0] hud_cam_fps_x10;
    reg [15:0] hud_edge_pct_x10;

    integer hi;
    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            fb_rd_addr_v <= 17'd0;
            lb_ra        <= 9'd0;
            lb_wa        <= 9'd0;
            lb0_we       <= 1'b0;
            lb1_we       <= 1'b0;
            lb0_wd       <= 8'd0;
            lb1_wd       <= 8'd0;
            disp_we      <= 1'b0;
            disp_wa      <= 9'd0;
            disp_wd      <= 16'd0;
            pf_v_d1      <= 1'b0; pf_v_d2 <= 1'b0; pf_v_d3 <= 1'b0; pf_v_d4 <= 1'b0;
            pf_747_d1    <= 1'b0; pf_747_d2 <= 1'b0;
            pf_sobel_d2  <= 1'b0; pf_sobel_d3 <= 1'b0; pf_sobel_d4 <= 1'b0;
            pf_sx_d1 <= 9'd0; pf_sx_d2 <= 9'd0; pf_sx_d3 <= 9'd0; pf_sx_d4 <= 9'd0; pf_sx_d5 <= 9'd0;
            r0c0 <= 8'd0; r0c1 <= 8'd0; r0c2 <= 8'd0;
            r1c0 <= 8'd0; r1c1 <= 8'd0; r1c2 <= 8'd0;
            r2c0 <= 8'd0; r2c1 <= 8'd0; r2c2 <= 8'd0;
            gx_q <= 11'sd0; gy_q <= 11'sd0;
            center_q1 <= 8'd0; center_q2 <= 8'd0;
            acc_min <= 8'd255; acc_max <= 8'd0; acc_sum <= 26'd0; acc_sum_snap <= 26'd0; acc_edge_cnt <= 17'd0;
            for (hi = 0; hi < 16; hi = hi + 1) begin
                acc_hist[hi] <= 16'd0;
                hud_hist[hi] <= 16'd0;
            end
            hud_min <= 8'd0; hud_max <= 8'd0; hud_avg <= 8'd0;
            hud_pkts <= 16'd0; hud_drops <= 16'd0; hud_w <= 16'd0; hud_h <= 16'd0;
            hud_ip <= 32'd0; hud_link_ok <= 1'b0; hud_thresh <= 8'd36; hud_algo <= 2'd0;
            hud_cam_fps_x10 <= 16'd0; hud_edge_pct_x10 <= 16'd0;
            meas_edge_pct_x10 <= 16'd0;
        end else begin
            lb0_we  <= 1'b0;
            lb1_we  <= 1'b0;
            disp_we <= 1'b0;

            // Stage 0: issue memory read (cols 0..360)
            if (pf_win && (pf_rel < 12'd361)) begin
                fb_rd_addr_v <= (pf_row_mul45 << 3) + {8'd0, pf_col_rot};
                lb_ra        <= pf_col_raw;
                pf_v_d1      <= 1'b1;
                pf_sx_d1     <= pf_rel[8:0];
                pf_747_d1    <= pass_v747;
            end else begin
                pf_v_d1   <= 1'b0;
                pf_747_d1 <= 1'b0;
            end

            // Stage 1: wait 1 cycle for slice mux & line buffer
            pf_v_d2     <= pf_v_d1;
            pf_sx_d2    <= pf_sx_d1;
            pf_747_d2   <= pf_747_d1;
            pf_sobel_d2 <= pf_v_d1 && !pf_747_d1;

            // Stage 2: data available! Update line buffers and 3x3 shift window
            if (pf_v_d2 && (pf_sx_d2 < SRC_W[8:0])) begin
                lb_wa <= pf_sx_d2;
                if (pf_747_d2) begin
                    // Row 0 duplicated into both lb0 & lb1 (top boundary clamp)
                    lb0_we <= 1'b1; lb0_wd <= bot_pix_d2;
                    lb1_we <= 1'b1; lb1_wd <= bot_pix_d2;
                end else begin
                    lb0_we <= 1'b1; lb0_wd <= lb1_q;
                    lb1_we <= 1'b1; lb1_wd <= bot_pix_d2;
                end
            end

            pf_sobel_d3 <= 1'b0;
            if (pf_sobel_d2) begin
                if (pf_sx_d2 == 9'd0) begin
                    // First pixel: duplicate left neighbor
                    r0c0 <= lb0_q;      r0c1 <= lb0_q;      r0c2 <= lb0_q;
                    r1c0 <= lb1_q;      r1c1 <= lb1_q;      r1c2 <= lb1_q;
                    r2c0 <= bot_pix_d2; r2c1 <= bot_pix_d2; r2c2 <= bot_pix_d2;
                end else if (pf_sx_d2 < SRC_W[8:0]) begin
                    r0c0 <= r0c1; r0c1 <= r0c2; r0c2 <= lb0_q;
                    r1c0 <= r1c1; r1c1 <= r1c2; r1c2 <= lb1_q;
                    r2c0 <= r2c1; r2c1 <= r2c2; r2c2 <= bot_pix_d2;
                end else begin
                    // sx == 360: flush col 359 with right border duplicated
                    r0c0 <= r0c1; r0c1 <= r0c2;
                    r1c0 <= r1c1; r1c1 <= r1c2;
                    r2c0 <= r2c1; r2c1 <= r2c2;
                end
                if (pf_sx_d2 >= 9'd1) begin
                    pf_sobel_d3 <= 1'b1;
                    pf_sx_d3    <= pf_sx_d2 - 9'd1;
                end
            end

            // Stage 3: Sobel Gx & Gy (for pixel pf_sx_d3 = 0..359)
            pf_sobel_d4 <= pf_sobel_d3;
            pf_sx_d4    <= pf_sx_d3;
            if (pf_sobel_d3) begin
                center_q1 <= r1c1;
                gx_q <= ($signed({3'd0, r0c2}) + $signed({2'd0, r1c2, 1'b0}) + $signed({3'd0, r2c2})) -
                        ($signed({3'd0, r0c0}) + $signed({2'd0, r1c0, 1'b0}) + $signed({3'd0, r2c0}));
                gy_q <= ($signed({3'd0, r2c0}) + $signed({2'd0, r2c1, 1'b0}) + $signed({3'd0, r2c2})) -
                        ($signed({3'd0, r0c0}) + $signed({2'd0, r0c1, 1'b0}) + $signed({3'd0, r0c2}));
            end

            // Stage 4: Write {grad, center_luma} to disp_line & accumulate frame stats
            if (pf_sobel_d4) begin
                disp_we <= 1'b1;
                disp_wa <= pf_sx_d4;
                disp_wd <= {grad_clamped, center_q1};

                // Update frame statistics only during active/row-0..269 passes (not v747)
                if (center_q1 < acc_min) acc_min <= center_q1;
                if (center_q1 > acc_max) acc_max <= center_q1;
                acc_sum <= acc_sum + {18'd0, center_q1};
                if (grad_clamped >= v_thresh) acc_edge_cnt <= acc_edge_cnt + 17'd1;
                acc_hist[center_q1[7:4]] <= acc_hist[center_q1[7:4]] + 16'd1;
            end

            // Snapshot stats cleanly at VSYNC (v_cnt == 720, h_cnt == 0)
            if ((v_cnt == V_ACTIVE) && (h_cnt == 12'd0)) begin
                hud_min      <= (acc_min <= acc_max) ? acc_min : 8'd0;
                hud_max      <= acc_max;
                acc_sum_snap <= acc_sum;
                for (hi = 0; hi < 16; hi = hi + 1) begin
                    hud_hist[hi] <= acc_hist[hi];
                    acc_hist[hi] <= 16'd0;
                end
                hud_pkts        <= v_pkts_s2;
                hud_drops       <= v_drops_s2;
                hud_w           <= v_w_s2;
                hud_h           <= v_h_s2;
                hud_ip          <= v_ip_s2;
                hud_link_ok     <= v_link_ok;
                hud_thresh      <= v_thresh;
                hud_algo        <= v_algo;
                hud_cam_fps_x10 <= meas_cam_fps_x10;
                meas_edge_pct_x10 <= (acc_edge_cnt[16:7] > 10'd972) ? 16'd1000 : (({11'd0, acc_edge_cnt} * 28'd1055) >> 17);
                hud_edge_pct_x10  <= (acc_edge_cnt[16:7] > 10'd972) ? 16'd1000 : (({11'd0, acc_edge_cnt} * 28'd1055) >> 17);
                acc_min <= 8'd255;
                acc_max <= 8'd0;
                acc_sum <= 26'd0;
                acc_edge_cnt <= 17'd0;
            end
            // Exact average luma at v_cnt == 720, h_cnt == 1: sum / 97200 = (sum * 674) >> 26
            if ((v_cnt == V_ACTIVE) && (h_cnt == 12'd1)) begin
                hud_avg <= ({8'd0, acc_sum_snap} * 34'd674) >> 26;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 6. VSYNC ASCII Formatter & 8x8 Font ROM (zero combinatorial div in active!)
    //    Prepares 14 dynamic 20-char lines during blanking v=721..740 -> char RAM
    // ------------------------------------------------------------------------
    // Minimal 64-entry 8x8 ASCII font (codes 0x20..0x5F)
    function [63:0] font_glyph;
        input [5:0] code; // ascii - 0x20
        begin
            case (code)
                6'h00: font_glyph = 64'h0000000000000000; // ' '
                6'h05: font_glyph = 64'h6264081026460000; // '%'
                6'h0B: font_glyph = 64'h0018187E18180000; // '+'
                6'h0D: font_glyph = 64'h0000007E00000000; // '-'
                6'h0E: font_glyph = 64'h0000000000181800; // '.'
                6'h0F: font_glyph = 64'h0204081020400000; // '/'
                6'h10: font_glyph = 64'h3C666E76663C0000; // '0'
                6'h11: font_glyph = 64'h18381818187E0000; // '1'
                6'h12: font_glyph = 64'h3C660C18307E0000; // '2'
                6'h13: font_glyph = 64'h3C660C1C06663C00; // '3'
                6'h14: font_glyph = 64'h0C1C3C6C7E0C0C00; // '4'
                6'h15: font_glyph = 64'h7E607C0606663C00; // '5'
                6'h16: font_glyph = 64'h3C607C66663C0000; // '6'
                6'h17: font_glyph = 64'h7E060C1830300000; // '7'
                6'h18: font_glyph = 64'h3C663C66663C0000; // '8'
                6'h19: font_glyph = 64'h3C663E060C380000; // '9'
                6'h1A: font_glyph = 64'h0018180018180000; // ':'
                6'h1D: font_glyph = 64'h00007E007E000000; // '='
                6'h1E: font_glyph = 64'h6030180C18306000; // '>'
                6'h21: font_glyph = 64'h183C667E66660000; // 'A'
                6'h22: font_glyph = 64'h7C667C66667C0000; // 'B'
                6'h23: font_glyph = 64'h3C666060663C0000; // 'C'
                6'h24: font_glyph = 64'h786C66666C780000; // 'D'
                6'h25: font_glyph = 64'h7E607C60607E0000; // 'E'
                6'h26: font_glyph = 64'h7E607C6060600000; // 'F'
                6'h27: font_glyph = 64'h3C66606E663E0000; // 'G'
                6'h28: font_glyph = 64'h66667E6666660000; // 'H'
                6'h29: font_glyph = 64'h3C181818183C0000; // 'I'
                6'h2B: font_glyph = 64'h666C786C66660000; // 'K'
                6'h2C: font_glyph = 64'h60606060607E0000; // 'L'
                6'h2D: font_glyph = 64'h63777F6B63630000; // 'M'
                6'h2E: font_glyph = 64'h66767E6E66660000; // 'N'
                6'h2F: font_glyph = 64'h3C666666663C0000; // 'O'
                6'h30: font_glyph = 64'h7C66667C60600000; // 'P'
                6'h32: font_glyph = 64'h7C66667C6C660000; // 'R'
                6'h33: font_glyph = 64'h3C603C06663C0000; // 'S'
                6'h34: font_glyph = 64'h7E18181818180000; // 'T'
                6'h35: font_glyph = 64'h66666666663C0000; // 'U'
                6'h36: font_glyph = 64'h666666663C180000; // 'V'
                6'h37: font_glyph = 64'h63636B7F77630000; // 'W'
                6'h38: font_glyph = 64'h663C183C66660000; // 'X'
                6'h39: font_glyph = 64'h66663C1818180000; // 'Y'
                default: font_glyph = 64'h0000000000000000;
            endcase
        end
    endfunction

    // Binary -> 5-digit BCD converter running during blanking (16 cycles per number)
    function [5:0] dig_code;
        input [3:0] d;
        begin
            dig_code = 6'h10 + {2'b00, d};
        end
    endfunction

    function [5:0] hex_code;
        input [3:0] h;
        begin
            hex_code = (h < 4'd10) ? (6'h10 + {2'b00, h}) : (6'h21 + {2'b00, h - 4'd10});
        end
    endfunction

    // 14 HUD rows x 20 chars stored in small LUT/reg array updated during v=722..729
    reg [5:0] hud_line_cam_fps [0:3];   // e.g. "60.0"
    reg [5:0] hud_line_pkts    [0:3];   // hex 16-bit
    reg [5:0] hud_line_drops   [0:4];   // decimal
    reg [5:0] hud_line_thr     [0:2];   // decimal
    reg [5:0] hud_line_edge    [0:3];   // xx.x%
    reg [5:0] hud_line_min     [0:2];
    reg [5:0] hud_line_max     [0:2];
    reg [5:0] hud_line_avg     [0:2];

    // Sequential double-dabble BCD converter (16 clocks)
    reg [15:0] bcd_bin;
    reg [19:0] bcd_out;
    reg [4:0]  bcd_step;
    reg [3:0]  bcd_target;

    wire [19:0] bcd_adj = {
        (bcd_out[19:16] >= 4'd5) ? (bcd_out[19:16] + 4'd3) : bcd_out[19:16],
        (bcd_out[15:12] >= 4'd5) ? (bcd_out[15:12] + 4'd3) : bcd_out[15:12],
        (bcd_out[11:8]  >= 4'd5) ? (bcd_out[11:8]  + 4'd3) : bcd_out[11:8],
        (bcd_out[7:4]   >= 4'd5) ? (bcd_out[7:4]   + 4'd3) : bcd_out[7:4],
        (bcd_out[3:0]   >= 4'd5) ? (bcd_out[3:0]   + 4'd3) : bcd_out[3:0]
    };

    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            bcd_bin <= 16'd0; bcd_out <= 20'd0; bcd_step <= 5'd0; bcd_target <= 4'd0;
            for (hi = 0; hi < 4; hi = hi + 1) begin
                hud_line_cam_fps[hi] <= 6'h10; hud_line_pkts[hi] <= 6'h10; hud_line_edge[hi] <= 6'h10;
            end
            for (hi = 0; hi < 5; hi = hi + 1) hud_line_drops[hi] <= 6'h10;
            for (hi = 0; hi < 3; hi = hi + 1) begin
                hud_line_thr[hi] <= 6'h10; hud_line_min[hi] <= 6'h10; hud_line_max[hi] <= 6'h10; hud_line_avg[hi] <= 6'h10;
            end
        end else begin
            if ((v_cnt >= 10'd722) && (v_cnt <= 10'd728) && (h_cnt == 12'd0)) begin
                bcd_out    <= 20'd0;
                bcd_step   <= 5'd16;
                bcd_target <= v_cnt[3:0] - 4'd2; // 0..6
                case (v_cnt[3:0] - 4'd2)
                    4'd0: bcd_bin <= hud_cam_fps_x10;
                    4'd1: bcd_bin <= hud_drops;
                    4'd2: bcd_bin <= {8'd0, hud_thresh};
                    4'd3: bcd_bin <= hud_edge_pct_x10;
                    4'd4: bcd_bin <= {8'd0, hud_min};
                    4'd5: bcd_bin <= {8'd0, hud_max};
                    4'd6: bcd_bin <= {8'd0, hud_avg};
                    default: bcd_bin <= 16'd0;
                endcase
                hud_line_pkts[0] <= hex_code(hud_pkts[15:12]);
                hud_line_pkts[1] <= hex_code(hud_pkts[11:8]);
                hud_line_pkts[2] <= hex_code(hud_pkts[7:4]);
                hud_line_pkts[3] <= hex_code(hud_pkts[3:0]);
            end else if (bcd_step != 5'd0) begin
                bcd_out  <= {bcd_adj[18:0], bcd_bin[15]};
                bcd_bin  <= {bcd_bin[14:0], 1'b0};
                bcd_step <= bcd_step - 5'd1;
                if (bcd_step == 5'd1) begin
                    // Final bit shifted in -> latch result
                    begin : blk_bcd_save
                        reg [19:0] fin;
                        fin = {bcd_adj[18:0], bcd_bin[15]};
                        case (bcd_target)
                            4'd0: begin // CAM FPS xx.x
                                hud_line_cam_fps[0] <= dig_code(fin[15:12]);
                                hud_line_cam_fps[1] <= dig_code(fin[11:8]);
                                hud_line_cam_fps[2] <= dig_code(fin[7:4]);
                                hud_line_cam_fps[3] <= dig_code(fin[3:0]);
                            end
                            4'd1: begin // DROPS
                                hud_line_drops[0] <= dig_code(fin[19:16]);
                                hud_line_drops[1] <= dig_code(fin[15:12]);
                                hud_line_drops[2] <= dig_code(fin[11:8]);
                                hud_line_drops[3] <= dig_code(fin[7:4]);
                                hud_line_drops[4] <= dig_code(fin[3:0]);
                            end
                            4'd2: begin // THR
                                hud_line_thr[0] <= dig_code(fin[11:8]);
                                hud_line_thr[1] <= dig_code(fin[7:4]);
                                hud_line_thr[2] <= dig_code(fin[3:0]);
                            end
                            4'd3: begin // EDGE xx.x
                                hud_line_edge[0] <= dig_code(fin[15:12]);
                                hud_line_edge[1] <= dig_code(fin[11:8]);
                                hud_line_edge[2] <= dig_code(fin[7:4]);
                                hud_line_edge[3] <= dig_code(fin[3:0]);
                            end
                            4'd4: begin
                                hud_line_min[0] <= dig_code(fin[11:8]);
                                hud_line_min[1] <= dig_code(fin[7:4]);
                                hud_line_min[2] <= dig_code(fin[3:0]);
                            end
                            4'd5: begin
                                hud_line_max[0] <= dig_code(fin[11:8]);
                                hud_line_max[1] <= dig_code(fin[7:4]);
                                hud_line_max[2] <= dig_code(fin[3:0]);
                            end
                            4'd6: begin
                                hud_line_avg[0] <= dig_code(fin[11:8]);
                                hud_line_avg[1] <= dig_code(fin[7:4]);
                                hud_line_avg[2] <= dig_code(fin[3:0]);
                            end
                            default: ;
                        endcase
                    end
                end
            end
        end
    end

    // HUD character selector (row 0..24, col 0..19 -> 6-bit ascii - 0x20)
    function [5:0] hud_char_at;
        input [4:0] row;
        input [4:0] col;
        begin
            hud_char_at = 6'h00;
            case (row)
                5'd1: case (col)
                    5'd1:hud_char_at=6'h28; 5'd2:hud_char_at=6'h29; 5'd3:hud_char_at=6'h2B; // HIK
                    5'd4:hud_char_at=6'h0D; 5'd5:hud_char_at=6'h2D; 5'd6:hud_char_at=6'h36; 5'd7:hud_char_at=6'h33; // -MVS
                    5'd9:hud_char_at=6'h17; 5'd10:hud_char_at=6'h12; 5'd11:hud_char_at=6'h10; 5'd12:hud_char_at=6'h30; // 720P
                    5'd13:hud_char_at=6'h16; 5'd14:hud_char_at=6'h10; // 60
                    default: ;
                endcase
                5'd3: case (col)
                    5'd1:hud_char_at=6'h2C; 5'd2:hud_char_at=6'h29; 5'd3:hud_char_at=6'h2E; 5'd4:hud_char_at=6'h2B; 5'd5:hud_char_at=6'h1A; // LINK:
                    5'd7:hud_char_at=hud_link_ok?6'h2F:6'h37;
                    5'd8:hud_char_at=hud_link_ok?6'h2B:6'h21;
                    5'd9:hud_char_at=hud_link_ok?6'h00:6'h29;
                    5'd10:hud_char_at=hud_link_ok?6'h00:6'h34; // OK / WAIT
                    default: ;
                endcase
                5'd5: case (col)
                    5'd1:hud_char_at=6'h32; 5'd2:hud_char_at=6'h25; 5'd3:hud_char_at=6'h33; 5'd4:hud_char_at=6'h1A; // RES:
                    5'd6:hud_char_at=6'h13; 5'd7:hud_char_at=6'h16; 5'd8:hud_char_at=6'h10; // 360
                    5'd9:hud_char_at=6'h38; // X
                    5'd10:hud_char_at=6'h12; 5'd11:hud_char_at=6'h17; 5'd12:hud_char_at=6'h10; // 270
                    5'd14:hud_char_at=6'h2D; 5'd15:hud_char_at=6'h18; // M8
                    default: ;
                endcase
                5'd7: case (col)
                    5'd1:hud_char_at=6'h23; 5'd2:hud_char_at=6'h21; 5'd3:hud_char_at=6'h2D; 5'd4:hud_char_at=6'h00; // CAM
                    5'd5:hud_char_at=6'h26; 5'd6:hud_char_at=6'h30; 5'd7:hud_char_at=6'h33; 5'd8:hud_char_at=6'h1A; // FPS:
                    5'd10:hud_char_at=hud_line_cam_fps[0]; 5'd11:hud_char_at=hud_line_cam_fps[1];
                    5'd12:hud_char_at=hud_line_cam_fps[2]; 5'd13:hud_char_at=6'h0E; 5'd14:hud_char_at=hud_line_cam_fps[3];
                    default: ;
                endcase
                5'd9: case (col)
                    5'd1:hud_char_at=6'h2F; 5'd2:hud_char_at=6'h35; 5'd3:hud_char_at=6'h34; 5'd4:hud_char_at=6'h00; // OUT
                    5'd5:hud_char_at=6'h26; 5'd6:hud_char_at=6'h30; 5'd7:hud_char_at=6'h33; 5'd8:hud_char_at=6'h1A; // FPS:
                    5'd10:hud_char_at=6'h16; 5'd11:hud_char_at=6'h10; 5'd12:hud_char_at=6'h0E; 5'd13:hud_char_at=6'h10; // 60.0
                    default: ;
                endcase
                5'd11: case (col)
                    5'd1:hud_char_at=6'h30; 5'd2:hud_char_at=6'h2B; 5'd3:hud_char_at=6'h34; 5'd4:hud_char_at=6'h33; 5'd5:hud_char_at=6'h1A; // PKTS:
                    5'd7:hud_char_at=6'h10; 5'd8:hud_char_at=6'h38; // 0X
                    5'd9:hud_char_at=hud_line_pkts[0]; 5'd10:hud_char_at=hud_line_pkts[1];
                    5'd11:hud_char_at=hud_line_pkts[2]; 5'd12:hud_char_at=hud_line_pkts[3];
                    default: ;
                endcase
                5'd13: case (col)
                    5'd1:hud_char_at=6'h24; 5'd2:hud_char_at=6'h32; 5'd3:hud_char_at=6'h2F; 5'd4:hud_char_at=6'h30; 5'd5:hud_char_at=6'h1A; // DROP:
                    5'd7:hud_char_at=hud_line_drops[0]; 5'd8:hud_char_at=hud_line_drops[1];
                    5'd9:hud_char_at=hud_line_drops[2]; 5'd10:hud_char_at=hud_line_drops[3];
                    5'd11:hud_char_at=hud_line_drops[4];
                    default: ;
                endcase
                5'd15: case (col)
                    5'd1:hud_char_at=6'h21; 5'd2:hud_char_at=6'h2C; 5'd3:hud_char_at=6'h27; 5'd4:hud_char_at=6'h2F; 5'd5:hud_char_at=6'h1A; // ALGO:
                    5'd7:hud_char_at=(hud_algo==2'd0)?6'h33:(hud_algo==2'd1)?6'h2D:(hud_algo==2'd2)?6'h32:6'h34;
                    5'd8:hud_char_at=(hud_algo==2'd0)?6'h2F:(hud_algo==2'd1)?6'h21:(hud_algo==2'd2)?6'h21:6'h25;
                    5'd9:hud_char_at=(hud_algo==2'd0)?6'h22:(hud_algo==2'd1)?6'h30:(hud_algo==2'd2)?6'h37:6'h33;
                    5'd10:hud_char_at=(hud_algo==2'd0)?6'h25:(hud_algo==2'd1)?6'h00:(hud_algo==2'd2)?6'h00:6'h34;
                    5'd11:hud_char_at=(hud_algo==2'd0)?6'h2C:6'h00; // SOBEL / MAP / RAW / TEST
                    default: ;
                endcase
                5'd17: case (col)
                    5'd1:hud_char_at=6'h34; 5'd2:hud_char_at=6'h28; 5'd3:hud_char_at=6'h32; 5'd4:hud_char_at=6'h1A; // THR:
                    5'd6:hud_char_at=hud_line_thr[0]; 5'd7:hud_char_at=hud_line_thr[1]; 5'd8:hud_char_at=hud_line_thr[2];
                    5'd10:hud_char_at=6'h25; 5'd11:hud_char_at=6'h24; 5'd12:hud_char_at=6'h27; 5'd13:hud_char_at=6'h1A; // EDG:
                    5'd14:hud_char_at=hud_line_edge[1]; 5'd15:hud_char_at=hud_line_edge[2]; 5'd16:hud_char_at=6'h0E;
                    5'd17:hud_char_at=hud_line_edge[3]; 5'd18:hud_char_at=6'h05;
                    default: ;
                endcase
                5'd19: case (col)
                    5'd1:hud_char_at=6'h2D; 5'd2:hud_char_at=6'h29; 5'd3:hud_char_at=6'h2E; 5'd4:hud_char_at=6'h1A; // MIN:
                    5'd5:hud_char_at=hud_line_min[0]; 5'd6:hud_char_at=hud_line_min[1]; 5'd7:hud_char_at=hud_line_min[2];
                    5'd10:hud_char_at=6'h2D; 5'd11:hud_char_at=6'h21; 5'd12:hud_char_at=6'h38; 5'd13:hud_char_at=6'h1A; // MAX:
                    5'd14:hud_char_at=hud_line_max[0]; 5'd15:hud_char_at=hud_line_max[1]; 5'd16:hud_char_at=hud_line_max[2];
                    default: ;
                endcase
                5'd21: case (col)
                    5'd1:hud_char_at=6'h21; 5'd2:hud_char_at=6'h36; 5'd3:hud_char_at=6'h27; 5'd4:hud_char_at=6'h1A; // AVG:
                    5'd6:hud_char_at=hud_line_avg[0]; 5'd7:hud_char_at=hud_line_avg[1]; 5'd8:hud_char_at=hud_line_avg[2];
                    default: ;
                endcase
                5'd24: case (col)
                    5'd1:hud_char_at=6'h28; 5'd2:hud_char_at=6'h29; 5'd3:hud_char_at=6'h33; 5'd4:hud_char_at=6'h34;
                    5'd5:hud_char_at=6'h2F; 5'd6:hud_char_at=6'h27; 5'd7:hud_char_at=6'h32; 5'd8:hud_char_at=6'h21;
                    5'd9:hud_char_at=6'h2D; // HISTOGRAM
                    default: ;
                endcase
                default: ;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------------
    // 7. Active Display Scanout + Overlay + HUD Renderer (video_clk 74 MHz)
    //    Stage A (combinational / reg): address disp_line[sx] & sync signals
    //    Stage B (1 clk later): disp_q valid; blend Sobel + Reticle + Sidebar
    // ------------------------------------------------------------------------
    reg [8:0] sx_cur;
    reg [3:0] acc_x;
    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            sx_cur  <= 9'd0;
            acc_x   <= 4'd0;
            disp_ra <= 9'd0;
        end else if (h_cnt == 12'd0) begin
            sx_cur  <= 9'd0;
            acc_x   <= 4'd3;
            disp_ra <= 9'd0;
        end else if (h_cnt < 12'd960) begin
            disp_ra <= sx_cur;
            if ((acc_x + 4'd3) >= 4'd8) begin
                acc_x  <= acc_x + 4'd3 - 4'd8;
                sx_cur <= (sx_cur < (SRC_W[8:0] - 9'd1)) ? (sx_cur + 9'd1) : sx_cur;
            end else begin
                acc_x <= acc_x + 4'd3;
            end
        end
    end

    // Pipeline alignment registers (2 clocks to match disp_q BSRAM + pixel reg)
    reg [11:0] hx_d1, hx_d2;
    reg [9:0]  vy_d1, vy_d2;
    reg        act_d1, act_d2, hs_d1, hs_d2, vs_d1, vs_d2;

    wire raw_de = (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);
    wire raw_hs = (h_cnt >= (H_ACTIVE + H_FP)) && (h_cnt < (H_ACTIVE + H_FP + H_SYNC));
    wire raw_vs = (v_cnt >= (V_ACTIVE + V_FP)) && (v_cnt < (V_ACTIVE + V_FP + V_SYNC));

    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            hx_d1  <= 12'd0; hx_d2  <= 12'd0;
            vy_d1  <= 10'd0; vy_d2  <= 10'd0;
            act_d1 <= 1'b0;  act_d2 <= 1'b0;
            hs_d1  <= 1'b0;  hs_d2  <= 1'b0;
            vs_d1  <= 1'b0;  vs_d2  <= 1'b0;
        end else begin
            hx_d1  <= h_cnt;  hx_d2  <= hx_d1;
            vy_d1  <= v_cnt;  vy_d2  <= vy_d1;
            act_d1 <= raw_de; act_d2 <= act_d1;
            hs_d1  <= raw_hs; hs_d2  <= hs_d1;
            vs_d1  <= raw_vs; vs_d2  <= vs_d1;
        end
    end

    // Pre-register font glyph row lookup at stage d1 for clean 74 MHz timing
    wire [11:0] hud_x_d1   = hx_d1 - 12'd960;
    wire [4:0]  hud_col_d1 = hud_x_d1[8:4]; // 0..19 (16px wide)
    wire [5:0]  hud_row_d1 = vy_d1[9:4];    // 0..44 (16px high)
    wire [63:0] glyph_bits_d1 = font_glyph(hud_char_at(hud_row_d1[4:0], hud_col_d1));
    // Glyph is 8x8 packed row-major MSB first (bit 63 = top-left of 8x8)
    wire [2:0]  gx_bit_d1 = 3'd7 - hud_x_d1[3:1];
    wire [2:0]  gy_row_d1 = 3'd7 - vy_d1[3:1];
    wire [5:0]  g_idx_d1  = {gy_row_d1, gx_bit_d1};
    reg         hud_txt_on_d2;
    reg [5:0]   hud_row_d2;

    always @(posedge video_clk) begin
        hud_txt_on_d2 <= glyph_bits_d1[g_idx_d1];
        hud_row_d2    <= hud_row_d1;
    end

    // Histogram bargraph in bottom sidebar: x in 976..1263 (16 bars * 18 px), y in 420..680
    wire [11:0] hbar_x = hx_d2 - 12'd976;
    wire [3:0]  hbar_bin = hbar_x[7:4]; // 16px per bin (16*16=256px, fit cleanly!)
    wire        in_hist_box = (hx_d2 >= 12'd976) && (hx_d2 < 12'd1232) && (vy_d2 >= 10'd420) && (vy_d2 < 10'd680);
    wire [9:0]  hist_val_scaled = (hud_hist[hbar_bin][15:6] > 10'd255) ? 10'd255 : hud_hist[hbar_bin][15:6];
    wire        hist_bar_px = in_hist_box && (hbar_x[3:0] < 4'd13) && ((10'd678 - vy_d2) <= hist_val_scaled);

    // Crosshair & corner fiducials on 960x720 video area (center = (480, 360))
    wire in_cross_h = (vy_d2 == 10'd360) && ((hx_d2 >= 12'd440 && hx_d2 < 12'd472) || (hx_d2 >= 12'd488 && hx_d2 < 12'd520));
    wire in_cross_v = (hx_d2 == 12'd480) && ((vy_d2 >= 10'd320 && vy_d2 < 10'd352) || (vy_d2 >= 10'd368 && vy_d2 < 10'd400));
    wire in_roi_box = ((hx_d2 == 12'd380 || hx_d2 == 12'd580) && (vy_d2 >= 10'd280 && vy_d2 <= 10'd440)) ||
                      ((vy_d2 == 10'd280 || vy_d2 == 10'd440) && (hx_d2 >= 12'd380 && hx_d2 <= 12'd580));
    wire on_reticle = v_overlay && (in_cross_h || in_cross_v || in_roi_box);

    // Pixel compositor at stage d2 -> registered into d3 for svo_tmds
    wire [7:0] px_luma = disp_q[7:0];
    wire [7:0] px_grad = disp_q[15:8];
    wire       px_edge = (px_grad >= v_thresh);

    reg        out_de, out_hs, out_vs;
    reg [7:0]  out_r, out_g, out_b;

    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            out_de <= 1'b0; out_hs <= 1'b0; out_vs <= 1'b0;
            out_r  <= 8'd0; out_g  <= 8'd0; out_b  <= 8'd0;
        end else begin
            out_de <= act_d2;
            out_hs <= hs_d2;
            out_vs <= vs_d2;

            if (!act_d2 || !v_enable) begin
                out_r <= 8'd0; out_g <= 8'd0; out_b <= 8'd0;
            end else if (hx_d2 < 12'd960) begin
                // Video viewport (960x720)
                if (on_reticle) begin
                    out_r <= 8'd255; out_g <= 8'd190; out_b <= 8'd20; // crisp amber reticle
                end else if (v_algo == 2'd0) begin
                    // Mono8 + Neon Cyan Sobel edge highlight
                    if (px_edge) begin
                        out_r <= 8'd0;
                        out_g <= (px_grad > 8'd128) ? 8'd255 : (8'd160 + {1'b0, px_grad[7:1]});
                        out_b <= 8'd255;
                    end else begin
                        out_r <= px_luma;
                        out_g <= px_luma;
                        out_b <= px_luma;
                    end
                end else if (v_algo == 2'd1) begin
                    // Pure Sobel Edge Map
                    out_r <= px_edge ? 8'd40 : 8'd0;
                    out_g <= px_grad;
                    out_b <= px_edge ? 8'd255 : {1'b0, px_grad[7:1]};
                end else begin
                    // Raw Mono8 / Test pattern clean grayscale
                    out_r <= px_luma;
                    out_g <= px_luma;
                    out_b <= px_luma;
                end
            end else begin
                // Telemetry HUD sidebar (x = 960..1279)
                if (hx_d2 == 12'd960 || hx_d2 == 12'd961 || hx_d2 == 12'd1279 || vy_d2 == 10'd0 || vy_d2 == 10'd719) begin
                    out_r <= 8'd0; out_g <= 8'd185; out_b <= 8'd240; // cyan border
                end else if (hist_bar_px) begin
                    out_r <= 8'd20; out_g <= 8'd220; out_b <= 8'd180;
                end else if (in_hist_box && (hx_d2 == 12'd976 || hx_d2 == 12'd1231 || vy_d2 == 10'd420 || vy_d2 == 10'd679)) begin
                    out_r <= 8'd40; out_g <= 8'd80; out_b <= 8'd110;
                end else if (hud_txt_on_d2 && (hud_row_d2 < 6'd25)) begin
                    if (hud_row_d2 == 6'd1) begin
                        out_r <= 8'd0; out_g <= 8'd235; out_b <= 8'd255; // header cyan
                    end else if (hud_row_d2 == 6'd3) begin
                        out_r <= hud_link_ok ? 8'd80 : 8'd255;
                        out_g <= hud_link_ok ? 8'd255 : 8'd90;
                        out_b <= hud_link_ok ? 8'd120 : 8'd60;
                    end else begin
                        out_r <= 8'd230; out_g <= 8'd240; out_b <= 8'd250;
                    end
                end else begin
                    out_r <= 8'd12; out_g <= 8'd20; out_b <= 8'd32; // deep slate
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // 8. TMDS 8b/10b Encoding + Dual-Port OSER10 + ELVDS_OBUF (HDMI 0 & HDMI 1)
    //    Channel 0 = Blue + {VSYNC, HSYNC}, Channel 1 = Green, Channel 2 = Red
    // ------------------------------------------------------------------------
    wire [2:0] tmds_d9, tmds_d8, tmds_d7, tmds_d6, tmds_d5;
    wire [2:0] tmds_d4, tmds_d3, tmds_d2, tmds_d1, tmds_d0;

    svo_tmds u_tmds_b (
        .clk    (video_clk),
        .resetn (video_rst_n),
        .de     (out_de),
        .ctrl   ({out_vs, out_hs}),
        .din    (out_b),
        .dout   ({tmds_d9[0], tmds_d8[0], tmds_d7[0], tmds_d6[0], tmds_d5[0],
                  tmds_d4[0], tmds_d3[0], tmds_d2[0], tmds_d1[0], tmds_d0[0]})
    );

    svo_tmds u_tmds_g (
        .clk    (video_clk),
        .resetn (video_rst_n),
        .de     (out_de),
        .ctrl   (2'b00),
        .din    (out_g),
        .dout   ({tmds_d9[1], tmds_d8[1], tmds_d7[1], tmds_d6[1], tmds_d5[1],
                  tmds_d4[1], tmds_d3[1], tmds_d2[1], tmds_d1[1], tmds_d0[1]})
    );

    svo_tmds u_tmds_r (
        .clk    (video_clk),
        .resetn (video_rst_n),
        .de     (out_de),
        .ctrl   (2'b00),
        .din    (out_r),
        .dout   ({tmds_d9[2], tmds_d8[2], tmds_d7[2], tmds_d6[2], tmds_d5[2],
                  tmds_d4[2], tmds_d3[2], tmds_d2[2], tmds_d1[2], tmds_d0[2]})
    );

    // Port 0 serializers + differential buffers
    wire       tmds_clk_s0;
    wire [2:0] tmds_data_s0;
    OSER10 u_ser_clk_0 (
        .Q(tmds_clk_s0),
        .D0(1'b1), .D1(1'b1), .D2(1'b1), .D3(1'b1), .D4(1'b1),
        .D5(1'b0), .D6(1'b0), .D7(1'b0), .D8(1'b0), .D9(1'b0),
        .PCLK(video_clk), .FCLK(serial_clk), .RESET(~video_rst_n)
    );
    OSER10 u_ser_d_0 [2:0] (
        .Q(tmds_data_s0),
        .D0(tmds_d0), .D1(tmds_d1), .D2(tmds_d2), .D3(tmds_d3), .D4(tmds_d4),
        .D5(tmds_d5), .D6(tmds_d6), .D7(tmds_d7), .D8(tmds_d8), .D9(tmds_d9),
        .PCLK(video_clk), .FCLK(serial_clk), .RESET(~video_rst_n)
    );
    ELVDS_OBUF u_obuf_0 [3:0] (
        .I  ({tmds_clk_s0, tmds_data_s0}),
        .O  ({tmds_clk_p_0, tmds_d_p_0}),
        .OB ({tmds_clk_n_0, tmds_d_n_0})
    );

endmodule
