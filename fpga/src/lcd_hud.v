// ============================================================================
// Module      : lcd_hud
// RGB666 Parallel LCD Controller with:
//   * 200x120 RGB332 Framebuffer (4x8192x8 BSRAM, 4-pixel burst APB write + auto-inc)
//   * Full 800x480 Hardware 60 FPS Plasma + Bouncing Neon Box engine (cfg_hw_plasma)
//   * Hardware 1px Calibration Border + 4-way Rotation + 4-step Brightness Dimming
// Clocked strictly on AHB_CLK (100 MHz).
//
// APB Register Map (0xF8000800..0xF8000FFF):
//   0x000  MAGIC     RO  0x4C434432 ("LCD2")
//   0x004  CTRL      RW  [0] enable, [1] mode_800, [2] inv_dclk, [3] show_border,
//                        [5:4] rot (00=0, 01=90, 10=180, 11=270),
//                        [7:6] dim (00=100%, 01=75%, 10=50%, 11=25%),
//                        [8] hw_plasma (1=native 800x480 60FPS HW animation),
//                        [13:9] hw_speed (1..20),
//                        [14] mono8 (1=fb_pix[7:2] across R/G/B for 64 gray levels)
//   0x008  STATUS    RO  [31:16] frame_cnt, [8:0] ctrl[8:0]
//   0x010  FB_WR     W   [14:0] addr ({y[6:0], x[7:0]}), [22:15] rgb332 pixel
//   0x014  PULSE     W   heartbeat
//   0x018  FB_ADDR   W   [12:6] y (0..119), [5:0] x_word (0..49) -> resets auto-inc pointer
//   0x01C  FB_DATA4  W   [31:0] 4 packed RGB332 pixels {px3, px2, px1, px0} + auto-inc
//   0x020  HW_RX_CFG RW  [0] hw_rx_en (1=accept hardware GVSP pixel stream)
//   0x024  HW_RX_PRT RW  [15:0] UDP port for hardware GVSP RX (default 5557)
//   0x028  HW_RX_ST  RO  [31:16] hw_frames, [15:0] hw_pkts
//   0x02C  HW_RX_GM  RO  [31:16] hw_h, [15:0] hw_w
// ============================================================================

`timescale 1ns / 1ps

module lcd_hud (
    input  wire        clk,             // AHB_CLK (100 MHz)
    input  wire        rst_n,           // rom_hrstn
    // APB slave
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [10:0] paddr,           // byte offset 0x000..0x7FF
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready,
    // Hardware GVSP pixel stream (from gvsp_rx_lcd in clk domain)
    input  wire        hw_pix_valid,
    input  wire [6:0]  hw_pix_y,
    input  wire [7:0]  hw_pix_x,
    input  wire [7:0]  hw_pix_data,
    input  wire [15:0] hw_stat_frames,
    input  wire [15:0] hw_stat_pkts,
    input  wire [15:0] hw_stat_w,
    input  wire [15:0] hw_stat_h,
    output reg         cfg_hw_rx_en,
    output reg  [15:0] cfg_hw_rx_port,
    // Physical RGB666 LCD panel interface
    output reg         lcd_dclk,
    output reg         lcd_de,
    output reg         lcd_hs,
    output reg         lcd_vs,
    output reg  [5:0]  lcd_r,
    output reg  [5:0]  lcd_g,
    output reg  [5:0]  lcd_b
);

    localparam [31:0] MAGIC = 32'h4C43_4432; // "LCD2"

    // ---- Registered APB Slave (with 4-cycle burst support for 0x01C FB_DATA4) ----
    reg [8:0]  q_addr;
    reg [31:0] q_wdata;
    reg        q_write, q_armed, q_go, q_rdy;
    reg        burst_active;
    reg [1:0]  burst_idx;
    reg [31:0] burst_word;
    assign pready = q_rdy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_addr       <= 9'd0;
            q_wdata      <= 32'd0;
            q_write      <= 1'b0;
            q_armed      <= 1'b0;
            q_go         <= 1'b0;
            q_rdy        <= 1'b0;
            burst_active <= 1'b0;
            burst_idx    <= 2'd0;
            burst_word   <= 32'd0;
        end else begin
            q_go  <= 1'b0;
            q_rdy <= 1'b0;

            if (burst_active) begin
                burst_word <= {8'd0, burst_word[31:8]};
                if (burst_idx == 2'd3) begin
                    burst_active <= 1'b0;
                    q_rdy        <= 1'b1;
                end else begin
                    burst_idx <= burst_idx + 2'd1;
                end
            end else begin
                if (q_go) begin
                    if (q_write && (q_addr == 9'd7)) begin
                        burst_active <= 1'b1;
                        burst_idx    <= 2'd0;
                        burst_word   <= q_wdata;
                    end else begin
                        q_rdy <= 1'b1;
                    end
                end
                if (psel && !penable) begin
                    q_addr  <= paddr[10:2];
                    q_wdata <= pwdata;
                    q_write <= pwrite;
                    q_armed <= 1'b1;
                end else if (q_armed && psel && penable) begin
                    q_go    <= 1'b1;
                    q_armed <= 1'b0;
                end
            end
        end
    end

    wire is_npu_space = (q_addr[8:6] == 3'b001); // 0x100..0x1FF (0xF8000900..0xF80009FF)
    wire apb_wr = q_go && q_write && !is_npu_space;
    wire apb_rd = q_go && !q_write;
    wire npu_wr = q_go && q_write && is_npu_space;
    wire [31:0] npu_rdata;

    npu_core u_npu_core (
        .clk       (clk),
        .rst_n     (rst_n),
        .reg_wr    (npu_wr),
        .reg_rd    (apb_rd && is_npu_space),
        .reg_addr  ({q_addr[5:0], 2'b00}),
        .reg_wdata (q_wdata),
        .reg_rdata (npu_rdata)
    );

    // ---- Control & Status Registers ----
    reg        cfg_enable;
    reg        cfg_mode_800;
    reg        cfg_inv_dclk;
    reg        cfg_show_border;
    reg [1:0]  cfg_rot;
    reg [1:0]  cfg_dim;
    reg        cfg_hw_plasma;
    reg [4:0]  cfg_hw_speed;
    reg        cfg_mono8;
    reg [15:0] frame_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_enable      <= 1'b0;    // Slot A keeps screen OFF
            cfg_mode_800    <= 1'b1;    // 800x480 default
            cfg_inv_dclk    <= 1'b1;
            cfg_show_border <= 1'b1;
            cfg_rot         <= 2'b10;   // 180 deg upright
            cfg_dim         <= 2'b00;   // 100%
            cfg_hw_plasma   <= 1'b1;    // Native 60 FPS HW plasma by default
            cfg_hw_speed    <= 5'd2;
            cfg_mono8       <= 1'b0;
            cfg_hw_rx_en    <= 1'b0;
            cfg_hw_rx_port  <= 16'd5557;
        end else if (apb_wr) begin
            case (q_addr)
                9'd1: begin
                    cfg_enable      <= q_wdata[0];
                    cfg_mode_800    <= q_wdata[1];
                    cfg_inv_dclk    <= q_wdata[2];
                    cfg_show_border <= q_wdata[3];
                    cfg_rot         <= q_wdata[5:4];
                    cfg_dim         <= q_wdata[7:6];
                    cfg_hw_plasma   <= q_wdata[8];
                    cfg_hw_speed    <= (q_wdata[13:9] != 5'd0) ? q_wdata[13:9] : 5'd2;
                    cfg_mono8       <= q_wdata[14];
                end
                9'd8: cfg_hw_rx_en   <= q_wdata[0];
                9'd9: cfg_hw_rx_port <= q_wdata[15:0];
                default: ;
            endcase
        end
    end

    always @(posedge clk) begin
        if (apb_rd) begin
            if (is_npu_space) begin
                prdata <= npu_rdata;
            end else begin
                case (q_addr)
                    9'd0: prdata <= MAGIC;
                    9'd1: prdata <= {17'd0, cfg_mono8, cfg_hw_speed, cfg_hw_plasma, cfg_dim, cfg_rot, cfg_show_border, cfg_inv_dclk, cfg_mode_800, cfg_enable};
                    9'd2: prdata <= {frame_cnt, 7'd0, cfg_hw_plasma, cfg_dim, cfg_rot, cfg_show_border, cfg_inv_dclk, cfg_mode_800, cfg_enable};
                    9'd8: prdata <= {31'd0, cfg_hw_rx_en};
                    9'd9: prdata <= {16'd0, cfg_hw_rx_port};
                    9'd10: prdata <= {hw_stat_frames, hw_stat_pkts};
                    9'd11: prdata <= {hw_stat_h, hw_stat_w};
                    default: prdata <= 32'd0;
                endcase
            end
        end
    end

    // ---- Single-bank 200x120 RGB332 Framebuffer (32768 x 8-bit BSRAM = 16 BSRAM) ----
    (* syn_ramstyle = "block_ram" *) reg [7:0] fb [0:32767];
    reg  [7:0]  fb_q;
    wire [14:0] fb_ra;

    // Auto-increment write pointer for 4-pixel burst APB writes (0x018 / 0x01C)
    reg [5:0] auto_x;
    reg [6:0] auto_y;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            auto_x <= 6'd0;
            auto_y <= 7'd0;
        end else if (apb_wr && (q_addr == 9'd6)) begin // 0x018 FB_ADDR
            auto_x <= q_wdata[5:0];
            auto_y <= q_wdata[12:6];
        end else if (burst_active && (burst_idx == 2'd3)) begin // end of 4-px burst
            if (auto_x >= 6'd49) begin
                auto_x <= 6'd0;
                auto_y <= auto_y + 7'd1;
            end else begin
                auto_x <= auto_x + 6'd1;
            end
        end
    end

    wire        fb_we = hw_pix_valid || burst_active || (apb_wr && (q_addr == 9'd4));
    wire [14:0] fb_wa = hw_pix_valid ? {hw_pix_y, hw_pix_x} :
                        burst_active ? {auto_y, auto_x, burst_idx} :
                                       q_wdata[14:0];
    wire [7:0]  fb_wd = hw_pix_valid ? hw_pix_data :
                        burst_active ? burst_word[7:0] :
                                       q_wdata[22:15];

    always @(posedge clk) begin
        if (fb_we) fb[fb_wa] <= fb_wd;
        fb_q <= fb[fb_ra];
    end

    // ---- Pixel Clock Divider on AHB_CLK (100 MHz) ----
    reg [3:0] div_cnt;
    reg       pix_stb;
    reg       dclk_wave;

    wire [3:0] div_max  = cfg_mode_800 ? 4'd2 : 4'd9;
    wire [3:0] div_half = cfg_mode_800 ? 4'd0 : 4'd4;

    always @(posedge clk) begin
        if (!cfg_enable) begin
            div_cnt   <= 4'd0;
            pix_stb   <= 1'b0;
            dclk_wave <= 1'b0;
        end else begin
            if (div_cnt >= div_max) begin
                div_cnt <= 4'd0;
                pix_stb <= 1'b1;
            end else begin
                div_cnt <= div_cnt + 4'd1;
                pix_stb <= 1'b0;
            end
            dclk_wave <= (div_cnt <= div_half);
        end
    end

    // ---- Video Timing Counters (800_480_screen reference) ----
    reg [10:0] h_cnt;
    reg [9:0]  v_cnt;
    reg        frame_pulse;

    wire [10:0] h_total = cfg_mode_800 ? 11'd1192 : 11'd524;
    wire [10:0] h_sync  = cfg_mode_800 ? 11'd48   : 11'd41;
    wire [10:0] h_start = cfg_mode_800 ? 11'd182  : 11'd43;
    wire [10:0] h_end   = cfg_mode_800 ? 11'd982  : 11'd523;

    wire [9:0]  v_total = cfg_mode_800 ? 10'd533  : 10'd285;
    wire [9:0]  v_sync  = cfg_mode_800 ? 10'd3    : 10'd10;
    wire [9:0]  v_start = cfg_mode_800 ? 10'd8    : 10'd12;
    wire [9:0]  v_end   = cfg_mode_800 ? 10'd488  : 10'd284;

    always @(posedge clk) begin
        frame_pulse <= 1'b0;
        if (!cfg_enable) begin
            h_cnt <= 11'd0;
            v_cnt <= 10'd0;
            if (!rst_n) frame_cnt <= 16'd0;
        end else if (pix_stb) begin
            if (h_cnt >= h_total) begin
                h_cnt <= 11'd0;
                if (v_cnt >= v_total) begin
                    v_cnt       <= 10'd0;
                    frame_cnt   <= frame_cnt + 16'd1;
                    frame_pulse <= 1'b1;
                end else begin
                    v_cnt <= v_cnt + 10'd1;
                end
            end else begin
                h_cnt <= h_cnt + 11'd1;
            end
        end
    end

    // ---- Hardware 60 FPS Animation State (Plasma phase + Bouncing Neon Box) ----
    reg [11:0] hw_phase;
    reg [9:0]  box_x;
    reg [8:0]  box_y;
    reg        box_dx, box_dy;
    reg [2:0]  box_col;

    always @(posedge clk) begin
        if (!cfg_enable || !rst_n) begin
            hw_phase <= 12'd0;
            box_x    <= 10'd120;
            box_y    <= 9'd100;
            box_dx   <= 1'b1;
            box_dy   <= 1'b1;
            box_col  <= 3'd2;
        end else if (frame_pulse) begin
            hw_phase <= hw_phase + {7'd0, cfg_hw_speed};
            if (box_dx) begin
                if (box_x >= 10'd650) begin box_dx <= 1'b0; box_col <= box_col + 3'd1; end
                else box_x <= box_x + {5'd0, cfg_hw_speed};
            end else begin
                if (box_x <= 10'd10)  begin box_dx <= 1'b1; box_col <= box_col + 3'd1; end
                else box_x <= box_x - {5'd0, cfg_hw_speed};
            end
            if (box_dy) begin
                if (box_y >= 9'd390)  begin box_dy <= 1'b0; box_col <= box_col + 3'd1; end
                else box_y <= box_y + {4'd0, cfg_hw_speed};
            end else begin
                if (box_y <= 9'd10)   begin box_dy <= 1'b1; box_col <= box_col + 3'd1; end
                else box_y <= box_y - {4'd0, cfg_hw_speed};
            end
        end
    end

    // ---- Stage 0: Active Region & Raw Coordinates ----
    reg        act_s0, hs_s0, vs_s0;
    reg [9:0]  px_x;
    reg [8:0]  px_y;
    wire [10:0] h_diff = h_cnt - h_start;
    wire [9:0]  v_diff = v_cnt - v_start;

    always @(posedge clk) begin
        hs_s0  <= (h_cnt >= h_sync);
        vs_s0  <= (v_cnt >= v_sync);
        act_s0 <= (h_cnt >= h_start) && (h_cnt < h_end) &&
                  (v_cnt >= v_start) && (v_cnt < v_end);
        px_x   <= (h_cnt >= h_start) ? h_diff[9:0] : 10'd0;
        px_y   <= (v_cnt >= v_start) ? v_diff[8:0] : 9'd0;
    end

    // ---- Stage 1: Full 10-bit 4-Way Rotation ----
    wire [9:0] lim_x = cfg_mode_800 ? 10'd799 : 10'd479;
    wire [9:0] lim_y = cfg_mode_800 ? 10'd479 : 10'd271;

    reg [9:0] map_x;
    reg [9:0] map_y;

    always @(*) begin
        case (cfg_rot)
            2'b00: begin map_x = px_x; map_y = {1'b0, px_y}; end
            2'b01: begin map_x = {1'b0, px_y}; map_y = (px_x <= lim_x) ? (lim_x - px_x) : 10'd0; end
            2'b10: begin map_x = (px_x <= lim_x) ? (lim_x - px_x) : 10'd0; map_y = ({1'b0, px_y} <= lim_y) ? (lim_y - {1'b0, px_y}) : 10'd0; end
            2'b11: begin map_x = ({1'b0, px_y} <= lim_y) ? (lim_y - {1'b0, px_y}) : 10'd0; map_y = px_x; end
        endcase
    end

    wire [7:0] fb_x = cfg_rot[0] ? map_y[9:2] : map_x[9:2];
    wire [6:0] fb_y = cfg_rot[0] ? map_x[8:2] : map_y[8:2];
    assign fb_ra = {fb_y, fb_x};
    wire [7:0] fb_pix = fb_q;

    // ---- Stage 1B: Native 800x480 60 FPS Hardware Plasma + Bouncing Box ----
    wire [7:0] wx = map_x[8:1] + hw_phase[7:0];
    wire [7:0] wy = map_y[8:1] + {hw_phase[6:0], 1'b0};
    wire [7:0] wz = map_x[9:2] - map_y[8:1] + hw_phase[7:0];

    wire [5:0] tri_r = wx[6] ? ~wx[5:0] : wx[5:0];
    wire [5:0] tri_g = wy[6] ? ~wy[5:0] : wy[5:0];
    wire [5:0] tri_b = wz[6] ? ~wz[5:0] : wz[5:0];

    wire in_hw_box = (map_x >= box_x) && (map_x < box_x + 10'd120) &&
                     (map_y >= {1'b0, box_y}) && (map_y < {1'b0, box_y} + 10'd72);
    wire is_hw_box_edge = in_hw_box && (
                     (map_x < box_x + 10'd4) || (map_x >= box_x + 10'd116) ||
                     (map_y < {1'b0, box_y} + 10'd4) || (map_y >= {1'b0, box_y} + 10'd68));

    reg [17:0] hw_plasma_rgb_q;
    always @(posedge clk) begin
        if (is_hw_box_edge)
            hw_plasma_rgb_q <= 18'b111111_111111_111111;
        else if (in_hw_box)
            hw_plasma_rgb_q <= { (box_col[0] ? 6'd58 : 6'd10), (box_col[1] ? 6'd58 : 6'd14), (box_col[2] ? 6'd58 : 6'd20) };
        else
            hw_plasma_rgb_q <= { {1'b0, tri_r[5:1]}, {1'b0, tri_g[5:1]}, ({1'b0, tri_b[5:1]} + 6'd10) };
    end

    // ---- Hardware 1px Calibration Border Overlay ----
    wire [9:0] b_lim_x = cfg_rot[0] ? lim_y : lim_x;
    wire [9:0] b_lim_y = cfg_rot[0] ? lim_x : lim_y;

    wire b_ring0 = (map_x == 10'd0) || (map_x == b_lim_x) || (map_y == 10'd0) || (map_y == b_lim_y);
    wire b_ring1 = !b_ring0 && ((map_x == 10'd1) || (map_x == b_lim_x - 10'd1) || (map_y == 10'd1) || (map_y == b_lim_y - 10'd1));
    wire b_ring2 = !b_ring0 && !b_ring1 && ((map_x == 10'd2) || (map_x == b_lim_x - 10'd2) || (map_y == 10'd2) || (map_y == b_lim_y - 10'd2));
    wire b_tl    = (map_x >= 10'd3) && (map_x < 10'd23) && (map_y >= 10'd3) && (map_y < 10'd23) &&
                   ((map_x < 10'd7) || (map_y < 10'd7));

    reg [2:0] border_type_q;
    always @(posedge clk) begin
        if (!cfg_show_border)
            border_type_q <= 3'd0;
        else if (b_ring0)
            border_type_q <= 3'd1;
        else if (b_ring1)
            border_type_q <= 3'd2;
        else if (b_ring2)
            border_type_q <= 3'd3;
        else if (b_tl)
            border_type_q <= 3'd4;
        else
            border_type_q <= 3'd0;
    end

    // ---- Stage 2: Select HW Plasma, Mono8 6-bit Grayscale, or RGB332 + Dimming + Border ----
    wire [5:0] raw_r6 = cfg_hw_plasma ? hw_plasma_rgb_q[17:12] :
                        cfg_mono8     ? fb_pix[7:2]            : {fb_pix[7:5], fb_pix[7:5]};
    wire [5:0] raw_g6 = cfg_hw_plasma ? hw_plasma_rgb_q[11:6]  :
                        cfg_mono8     ? fb_pix[7:2]            : {fb_pix[4:2], fb_pix[4:2]};
    wire [5:0] raw_b6 = cfg_hw_plasma ? hw_plasma_rgb_q[5:0]   :
                        cfg_mono8     ? fb_pix[7:2]            : {fb_pix[1:0], fb_pix[1:0], fb_pix[1:0]};

    reg [5:0] dim_r6, dim_g6, dim_b6;
    always @(*) begin
        case (cfg_dim)
            2'b00: begin dim_r6 = raw_r6; dim_g6 = raw_g6; dim_b6 = raw_b6; end
            2'b01: begin
                dim_r6 = {1'b0, raw_r6[5:1]} + {2'b00, raw_r6[5:2]};
                dim_g6 = {1'b0, raw_g6[5:1]} + {2'b00, raw_g6[5:2]};
                dim_b6 = {1'b0, raw_b6[5:1]} + {2'b00, raw_b6[5:2]};
            end
            2'b10: begin
                dim_r6 = {1'b0, raw_r6[5:1]};
                dim_g6 = {1'b0, raw_g6[5:1]};
                dim_b6 = {1'b0, raw_b6[5:1]};
            end
            2'b11: begin
                dim_r6 = {2'b00, raw_r6[5:2]};
                dim_g6 = {2'b00, raw_g6[5:2]};
                dim_b6 = {2'b00, raw_b6[5:2]};
            end
        endcase
    end

    reg [17:0] final_rgb;
    always @(*) begin
        case (border_type_q)
            3'd1: final_rgb = 18'b111111_000000_000000;
            3'd2: final_rgb = 18'b000000_111111_000000;
            3'd3: final_rgb = 18'b000000_000000_111111;
            3'd4: final_rgb = 18'b000000_111111_000000;
            default: final_rgb = {dim_r6, dim_g6, dim_b6};
        endcase
    end

    // ---- Final Registered Pad Outputs ----
    always @(posedge clk) begin
        if (!cfg_enable) begin
            lcd_dclk <= 1'b0;
            lcd_de   <= 1'b0;
            lcd_hs   <= 1'b0;
            lcd_vs   <= 1'b0;
            lcd_r    <= 6'd0;
            lcd_g    <= 6'd0;
            lcd_b    <= 6'd0;
        end else begin
            lcd_dclk <= cfg_inv_dclk ? ~dclk_wave : dclk_wave;
            if (pix_stb) begin
                lcd_de <= act_s0;
                lcd_hs <= hs_s0;
                lcd_vs <= vs_s0;
                lcd_r  <= act_s0 ? final_rgb[17:12] : 6'd0;
                lcd_g  <= act_s0 ? final_rgb[11:6]  : 6'd0;
                lcd_b  <= act_s0 ? final_rgb[5:0]   : 6'd0;
            end
        end
    end

endmodule
