// ============================================================================
// Module      : apb_ctl
// FPGA control registers on the extended APB slave (0xF8001000..), in place of
// the DDR3 mailboxes the FPGA used to poll (design "Регистры управления ПЛИС на
// APB", 30.10.2026; revises DECISIONS R-13). The layouts stay those of the DDR3
// blocks; only the transport changes. The consumers keep their parsers and
// read the same words through local ports that behave like the ones they had:
//
//   * Lane 5 slave (rd_go / rd_rdy / rd_en / rd_valid / rd_data), as l5_axi_rd
//     gave lane5_reader: bursts from the mailbox, CFG_ADDR words 0..63.
//   * An AXI slave for udp_rd's descriptor, status and stream-record traffic
//     (udp_tx routes those addresses here, frames still go to DDR3): one
//     256-bit beat RAM of 16 beats, beat = {addr[12], addr[7:5]}:
//       0..2  descriptor   (DDR3 0x06000400)       APB 0x200..0x25F
//       4..5  status       (DDR3 0x06000480)       APB 0x280..0x2BF
//       8..11 stream record camera 0 (0x06001100)  APB 0x300..0x37F
//       12..15 stream record camera 1 (0x06001180) APB 0x380..0x3FF
//     Record beat 3 (+0x60) is the block status udp_rd writes.
//
// APB block map (byte offsets from 0xF8001000):
//   0x000..0x0FF  mailbox, words 0..63 (TRIG, SYNC, GEOM, LASR, MMTB)   R/W
//   0x200..0x3FF  the beat RAM above, word w = beat * 8 + lane         R/W
//   0x410         capabilities: [0] control on APB (feature bits: flash_ctl  R
//                 FEATURES 0x018, read there first: old bitstreams do not
//                 decode PADDR[12])
//   0x414         Lane 5 reads outside the mailbox (o_l5_stray)          R
//   0x420         mm band word: each write is one word to the loader     W
//                 (lane5_reader.v; reads 0)
//   0x424         mm loader state (lane5_reader o_ld_stat)               R
//   other         reads 0, writes ignored
//
// The memories are block RAM (the device has no distributed RAM), one write
// port and one read address per copy; the APB read-back uses a copy of its own.
// The clk_x1 side is reached only through apb_ctl_bridge.
// ============================================================================

`timescale 1ns / 1ps

module apb_ctl #(
    parameter [31:0] CFG_ADDR = 32'h0600_0100,   // the mailbox's old DDR3 address (lane5_reader)
    parameter [31:0] CAPS     = 32'h0000_0001
)(
    // ---- APB (AHB_CLK) -------------------------------------------------------
    input  wire          pclk,
    input  wire          prst_n,
    input  wire          psel,
    input  wire          penable,
    input  wire          pwrite,
    input  wire [11:0]   paddr,
    input  wire [31:0]   pwdata,
    output wire [31:0]   prdata,
    output wire          pready,

    // ---- clk_x1 ----------------------------------------------------------------
    input  wire          clk,
    input  wire          rst_n,

    // Lane 5 slave towards lane5_reader
    input  wire          rd_go,
    input  wire [7:0]    rd_burstcount,
    input  wire [31:0]   rd_addr,
    input  wire          rd_en,
    output reg           rd_rdy,
    output reg           rd_valid,
    output reg  [63:0]   rd_data,
    output reg  [15:0]   o_l5_stray,       // Lane 5 reads outside the mailbox (returned as zeros)

    // AXI slave towards udp_rd (descriptor / status / stream records only)
    input  wire          s_arvalid,
    output wire          s_arready,
    input  wire [29:0]   s_araddr,
    input  wire [7:0]    s_arlen,
    output reg           s_rvalid,
    input  wire          s_rready,
    output reg  [255:0]  s_rdata,
    output reg           s_rlast,
    input  wire          s_awvalid,
    output wire          s_awready,
    input  wire [29:0]   s_awaddr,
    input  wire [7:0]    s_awlen,
    input  wire          s_wvalid,
    output wire          s_wready,
    input  wire [255:0]  s_wdata,
    input  wire          s_wlast,
    output reg           s_bvalid,
    input  wire          s_bready,

    // mm band words to lane5_reader's loader, one clock each, and its state
    output reg           o_bw_v,
    output reg  [31:0]   o_bw_d,
    input  wire [31:0]   i_ld_stat
);

    // ------------------------------------------------------------- the bridge
    wire        c_go, c_we;
    wire [9:0]  c_addr;
    wire [31:0] c_wdata;
    wire [31:0] c_rdata;

    apb_ctl_bridge #(.AW(10)) u_bridge (
        .pclk(pclk), .prst_n(prst_n), .psel(psel), .penable(penable), .pwrite(pwrite),
        .paddr(paddr), .pwdata(pwdata), .prdata(prdata), .pready(pready),
        .clk(clk), .rst_n(rst_n), .c_go(c_go), .c_we(c_we), .c_addr(c_addr),
        .c_wdata(c_wdata), .c_rdata(c_rdata)
    );

    wire c_mbx  = (c_addr[9:6] == 4'd0);          // 0x000..0x0FF
    wire c_beat = (c_addr[9:7] == 3'd1);          // 0x200..0x3FF
    wire c_caps = (c_addr == 10'd260);            // 0x410
    wire c_strw = (c_addr == 10'd261);            // 0x414
    wire c_band = (c_addr == 10'd264);            // 0x420
    wire c_ldst = (c_addr == 10'd265);            // 0x424

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_bw_v <= 1'b0; o_bw_d <= 32'd0;
        end else begin
            o_bw_v <= c_go && c_we && c_band;
            if (c_go) o_bw_d <= c_wdata;
        end
    end

    // --------------------------------------------------------------- memories
    // GW5AST has no distributed RAM (synthesis IF0005; every syn_ramstyle =
    // "distributed_ram" array became flip-flops), so all three memories are block
    // RAM with a registered read (SDPB, one write and one read port each):
    //   l5m   64 x 32   mailbox, read by the Lane 5 slave
    //   um   128 x 32   beat RAM, word {beat, lane}, read by udp_rd's AXI reads
    //   am   256 x 32   APB read-back: 0..63 mailbox, 128..255 beat RAM
    // Every write goes to each copy that holds the word. Writes come from the APB
    // (one word) or from udp_rd's W beats, which a serialiser writes one lane per
    // clock; it gives way to an APB write in the same clock.
    reg  [255:0] w_buf;                           // W beat being written lane by lane
    reg          w_ser;                           // w_buf holds lanes still to write
    reg  [2:0]   w_lane;
    reg  [3:0]   w_beat;
    reg          w_act, w_last;
    wire        apb_we      = c_go && c_we;
    wire        mbx_we      = apb_we && c_mbx;
    wire        beat_apb_we = apb_we && c_beat;
    wire        ser_we      = w_ser && !apb_we;

    wire [6:0]  ub_wa = beat_apb_we ? c_addr[6:0] : {w_beat, w_lane};
    wire [31:0] ser_wd = w_buf[32*w_lane +: 32];
    wire [31:0] ub_wd = beat_apb_we ? c_wdata : ser_wd;
    wire        ub_we = beat_apb_we || ser_we;
    wire [7:0]  am_wa = mbx_we ? {2'b00, c_addr[5:0]} : {1'b1, ub_wa};
    wire        am_we = mbx_we || ub_we;

    (* syn_ramstyle = "block_ram" *) reg [31:0] l5m [0:63];
    (* syn_ramstyle = "block_ram" *) reg [31:0] um  [0:127];
    (* syn_ramstyle = "block_ram" *) reg [31:0] am  [0:255];
    reg  [31:0] l5m_q, um_q, am_q;
    wire [5:0]  l5m_ra;
    wire [6:0]  um_ra;
    wire [7:0]  am_ra = c_beat ? {1'b1, c_addr[6:0]} : {2'b00, c_addr[5:0]};
    always @(posedge clk) begin
        if (mbx_we) l5m[c_addr[5:0]] <= c_wdata;
        l5m_q <= l5m[l5m_ra];
    end
    always @(posedge clk) begin
        if (ub_we) um[ub_wa] <= ub_wd;
        um_q <= um[um_ra];
    end
    always @(posedge clk) begin
        if (am_we) am[am_wa] <= mbx_we ? c_wdata : ub_wd;
        am_q <= am[am_ra];
    end

    // ---------------------------------------------------------- APB read-back
    // c_addr is stable from c_go on, so am_q (read at the clock after c_go) is the
    // word asked for; the bridge takes c_rdata in that clock.
    reg        c_from_am;
    reg [31:0] c_misc;
    always @(posedge clk) begin
        c_from_am <= c_mbx || c_beat;
        c_misc    <= c_caps ? CAPS :
                     c_strw ? {16'd0, o_l5_stray} :
                     c_ldst ? i_ld_stat : 32'd0;
    end
    assign c_rdata = c_from_am ? am_q : c_misc;

    // ---------------------------------------------------------- Lane 5 slave
    // rd_go latches the burst; rd_rdy one clock later; then one word per clock
    // while rd_en is high, until burstcount words. A word is read in the clock it
    // is issued and rd_valid follows two clocks later (block RAM read + output
    // flop); lane5_reader waits for rd_valid and the slave never issues more than
    // the burst. Addresses outside the mailbox read as zeros and are counted.
    reg        l_pend, l_act, l_iss, l_iss_in;
    reg [31:0] l_addr;
    reg [7:0]  l_left;
    wire [31:0] l_off  = l_addr - CFG_ADDR;
    wire        l_in   = (l_off[31:8] == 24'd0);
    assign l5m_ra = l_off[7:2];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            l_pend <= 1'b0; l_act <= 1'b0; l_iss <= 1'b0; l_iss_in <= 1'b0;
            l_addr <= 32'd0; l_left <= 8'd0;
            rd_rdy <= 1'b0; rd_valid <= 1'b0; rd_data <= 64'd0; o_l5_stray <= 16'd0;
        end else begin
            rd_rdy   <= 1'b0;
            l_iss    <= 1'b0;
            rd_valid <= l_iss;
            if (l_iss) rd_data <= {32'd0, l_iss_in ? l5m_q : 32'd0};
            if (rd_go) begin
                // a new burst replaces one in progress (l5_axi_rd drained the old one)
                l_pend <= 1'b1; l_act <= 1'b0; rd_valid <= 1'b0;
                l_addr <= rd_addr; l_left <= rd_burstcount;
            end else if (l_pend) begin
                l_pend <= 1'b0; l_act <= 1'b1;
                rd_rdy <= 1'b1;
                if (!l_in && o_l5_stray != 16'hFFFF) o_l5_stray <= o_l5_stray + 16'd1;
            end else if (l_act) begin
                if (l_left == 8'd0) begin
                    l_act <= 1'b0;
                end else if (rd_en) begin
                    l_iss    <= 1'b1;
                    l_iss_in <= l_in;
                    l_addr   <= l_addr + 32'd4;
                    l_left   <= l_left - 8'd1;
                end
            end
        end
    end

    // ------------------------------------------------------------- AXI slave
    // One read and one write at a time (udp_rd keeps one of each in flight). A
    // read beat is gathered lane by lane from um into s_rdata (8 reads, ~10
    // clocks a beat) and shown once complete; these are a few beats per frame.
    reg        r_act, r_get, r_done;
    reg [3:0]  r_beat;
    reg [7:0]  r_left;
    reg [2:0]  r_lane, r_lane_q;
    assign um_ra = {r_beat, r_lane};
    assign s_arready = !r_act && !s_rvalid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_act <= 1'b0; r_get <= 1'b0; r_done <= 1'b0; r_beat <= 4'd0; r_left <= 8'd0;
            r_lane <= 3'd0; r_lane_q <= 3'd0;
            s_rvalid <= 1'b0; s_rdata <= 256'd0; s_rlast <= 1'b0;
        end else begin
            if (s_rvalid && s_rready) s_rvalid <= 1'b0;
            r_get    <= 1'b0;
            r_lane_q <= r_lane;
            // um_q is lane r_lane_q of the beat when r_get is high
            if (r_get) s_rdata[32*r_lane_q +: 32] <= um_q;
            if (s_arvalid && s_arready) begin
                r_act  <= 1'b1;
                r_beat <= {s_araddr[12], s_araddr[7:5]};
                r_left <= s_arlen;
                r_lane <= 3'd0;
                r_done <= 1'b0;
            end else if (r_act && !s_rvalid && !r_done) begin
                // read lane r_lane now; it lands in s_rdata next clock
                r_get  <= 1'b1;
                r_lane <= r_lane + 3'd1;
                if (r_lane == 3'd7) r_done <= 1'b1;
            end else if (r_act && r_done && !r_get) begin
                // all eight lanes are in s_rdata
                s_rvalid <= 1'b1;
                s_rlast  <= (r_left == 8'd0);
                r_beat   <= r_beat + 4'd1;
                r_left   <= r_left - 8'd1;
                r_done   <= 1'b0;
                if (r_left == 8'd0) r_act <= 1'b0;
            end
        end
    end

    // A W beat is taken whole into w_buf, then written one lane per clock; the
    // response comes after its last lane, so a read that follows sees it.
    assign s_awready = !w_act && !s_bvalid;
    assign s_wready  = w_act && !w_ser;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_act <= 1'b0; w_beat <= 4'd0; s_bvalid <= 1'b0;
            w_ser <= 1'b0; w_lane <= 3'd0; w_last <= 1'b0; w_buf <= 256'd0;
        end else begin
            if (s_bvalid && s_bready) s_bvalid <= 1'b0;
            if (s_awvalid && s_awready) begin
                w_act  <= 1'b1;
                w_beat <= {s_awaddr[12], s_awaddr[7:5]};
            end else if (w_act && s_wvalid && !w_ser) begin
                w_buf  <= s_wdata;
                w_last <= s_wlast;
                w_ser  <= 1'b1;
                w_lane <= 3'd0;
            end else if (ser_we) begin
                w_lane <= w_lane + 3'd1;
                if (w_lane == 3'd7) begin
                    w_ser  <= 1'b0;
                    w_beat <= w_beat + 4'd1;
                    if (w_last) begin
                        w_act    <= 1'b0;
                        s_bvalid <= 1'b1;
                    end
                end
            end
        end
    end

    // synthesis translate_off
    integer mi;
    initial begin
        for (mi = 0; mi < 64; mi = mi + 1) l5m[mi] = 32'd0;
        for (mi = 0; mi < 128; mi = mi + 1) um[mi] = 32'd0;
        for (mi = 0; mi < 256; mi = mi + 1) am[mi] = 32'd0;
    end
    // synthesis translate_on

endmodule
