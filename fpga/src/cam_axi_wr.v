// ============================================================================
// Module      : cam_axi_wr
// The camera writer's AXI4 write port (cam_channel.v), in place of its Lane 4
// port and the l4_axi_wr adapter behind it since 09.10.2026
// (AXI4 migration, stage 5: ERRORS 1.24; docs/ARCHITECTURE.md § 4.1). There is
// no Lane 4 handshake any more - no wr_go, no idle cycle, no wr_done to wait for.
//
// The writer FSM hands over one 32-bit word per cycle. A 256-bit beat takes
// eight of them, so a burst is issued on AXI only once all of its data is
// buffered: W is shared by every master through axi_mux, and a writer that
// raised AW and then trickled a beat every eight cycles would hold the channel
// for the CPU and the other camera that long.
//
//   * Words: s_en with s_data; s_first marks a burst's first word, whose byte
//     address is s_addr (word aligned, held by the writer while its words
//     flow), s_last the burst's last word. Word k of a burst at address A goes
//     to lane (A[4:2] + k) mod 8 of beat (A[4:2] + k) / 8, and lanes no word
//     reached are masked, so any word-aligned start and any length are written
//     exactly.
//   * Beats go into a FIFO of 2^WQ_LOG2. The beat that closes an AXI burst -
//     the writer's last word, or the last beat before a 4 KB boundary - also
//     queues that burst's {address, length}. AW is issued from that queue and
//     W from the beat FIFO, both in the same order.
//   * s_room (a register) says that a burst of up to MAX_WORDS words fits in
//     both queues; the writer starts a burst only while it is high (CLAUDE.md
//     invariant 3). Nothing
//     else is handed back: a burst taken here is written. B is counted, not
//     waited for (o_aw_cnt / o_b_cnt / o_bresp_err: telemetry words 7 and 11).
// ============================================================================

`timescale 1ns / 1ps

module cam_axi_wr #(
    parameter integer DW        = 256,
    parameter integer AW        = 30,
    parameter integer MAX_WORDS = 128,  // longest burst the writer starts
    parameter integer WQ_LOG2   = 7,    // beat FIFO: 128 x 265 bits, one BSRAM row
    parameter integer AQ_LOG2   = 5,    // AW queue: 32 bursts
    // AXI bursts this port may have issued whose last W beat has not left it
    // yet. 1 is the pattern l4_axi_wr had (AW, all of its W, then the next AW);
    // without the limit up to ~8 AWs could wait in axi_mux / the IP ahead of
    // their data. Experiment of 09.10.2026 (ERRORS 1.24): Ethernet receive froze
    // under load with the unlimited port - does the IP's write queue depth,
    // through the CPU's read latency, reach the MAC?
    parameter integer AW_AHEAD  = 1
)(
    input  wire             clk,
    input  wire             rst_n,

    // ---- words from the writer FSM ------------------------------------------
    input  wire             s_en,
    input  wire [31:0]      s_data,
    input  wire             s_first,
    input  wire             s_last,
    input  wire [31:0]      s_addr,
    output reg              s_room,

    // ---- AXI4 write master (one axi_mux port) -------------------------------
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
    output reg  [15:0]      o_aw_cnt,       // AXI bursts issued, wrapping
    output reg  [15:0]      o_b_cnt,        // write responses received, wrapping
    output reg  [2:0]       o_bresp_err     // non-OKAY responses, saturating at 7
);

    localparam integer WPB  = DW / 32;              // 32-bit words per beat (8)
    localparam integer LG   = 3;                    // log2(WPB)
    localparam integer BA   = AW - LG - 2;          // beat address width
    localparam integer WQD  = 1 << WQ_LOG2;
    localparam integer AQD  = 1 << AQ_LOG2;
    localparam integer WE   = DW + WPB + 1;         // beat FIFO entry: {wlast, mask, data}
    // A burst of MAX_WORDS spans at most this many beats, and at most two AXI
    // bursts (it is far shorter than a 4 KB page).
    localparam integer MAXB = MAX_WORDS / WPB + 1;
    // s_room looks at queue levels that lag what the writer has already handed
    // over: the previous burst's last two beats and two AW entries may still be
    // on their way in when the writer decides to start the next one, and the
    // flag itself is one cycle old. The margins cover that.
    localparam integer     WQ_ROOM_I = WQD - MAXB - 4;
    localparam integer     AQ_ROOM_I = AQD - 2 - 3;
    localparam [WQ_LOG2:0] WQ_ROOM   = WQ_ROOM_I[WQ_LOG2:0];
    localparam [AQ_LOG2:0] AQ_ROOM   = AQ_ROOM_I[AQ_LOG2:0];

    // =========================================================================
    // Packing: words -> beats
    // =========================================================================
    reg  [DW-1:0]   acc;                // the beat being assembled
    reg  [WPB-1:0]  msk;                // its lanes written so far
    reg  [LG-1:0]   lane;               // lane of the next word of the burst
    reg  [BA-1:0]   baddr;              // address of the beat being assembled

    reg             p_vld;              // acc/msk hold a finished beat: push it now
    reg             p_end;              // ... the burst's last
    reg             p_page;             // ... the last beat of a 4 KB page
    reg  [BA-1:0]   p_baddr;            // ... its address

    wire [LG-1:0]   lane_cur  = s_first ? s_addr[LG+1:2] : lane;
    wire [BA-1:0]   baddr_cur = s_first ? s_addr[AW-1:LG+2] : baddr;
    wire            beat_done = s_en && ((&lane_cur) || s_last);

    // Each lane register loads only its own word; a finished beat leaves acc
    // the cycle after (it is read by the FIFO write on the same edge that the
    // next word lands in lane 0), so no wide shift or clear is needed - lanes
    // a beat did not write are masked off.
    genvar g;
    generate
        for (g = 0; g < WPB; g = g + 1) begin : g_lane
            always @(posedge clk)
                if (s_en && lane_cur == g) acc[g*32 +: 32] <= s_data;
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msk     <= {WPB{1'b0}};
            lane    <= {LG{1'b0}};
            baddr   <= {BA{1'b0}};
            p_vld   <= 1'b0;
            p_end   <= 1'b0;
            p_page  <= 1'b0;
            p_baddr <= {BA{1'b0}};
        end else begin
            msk   <= (p_vld ? {WPB{1'b0}} : msk) |
                     (s_en ? ({{(WPB-1){1'b0}}, 1'b1} << lane_cur) : {WPB{1'b0}});
            p_vld <= beat_done;
            if (s_en) begin
                lane  <= lane_cur + 1'b1;
                baddr <= beat_done ? baddr_cur + 1'b1 : baddr_cur;
            end
            if (beat_done) begin
                p_end   <= s_last;
                p_page  <= &baddr_cur[11-LG-2:0];   // beat 127 of a 4 KB page
                p_baddr <= baddr_cur;
            end
        end
    end

    // =========================================================================
    // Beat FIFO and AW queue
    // =========================================================================
    reg  [WE-1:0]       wmem [0:WQD-1];
    reg  [WQ_LOG2:0]    wq_wp, wq_fp;       // write, fetch
    reg  [WQ_LOG2:0]    wq_cnt;             // beats here, output stages included

    reg  [BA+7:0]       aq [0:AQD-1];       // {beat address, AWLEN}
    reg  [AQ_LOG2:0]    aq_wp, aq_rp;

    // The AXI burst a pushed beat belongs to: open until a beat closes it.
    wire                p_wlast  = p_end || p_page;
    reg                 ch_open;
    reg  [BA-1:0]       ch_start;
    reg  [7:0]          ch_n;
    wire [BA-1:0]       ch_first = ch_open ? ch_start : p_baddr;
    wire [7:0]          ch_cnt   = ch_open ? ch_n + 8'd1 : 8'd1;

    always @(posedge clk) begin
        if (p_vld) wmem[wq_wp[WQ_LOG2-1:0]] <= {p_wlast, msk, acc};
        if (p_vld && p_wlast) aq[aq_wp[AQ_LOG2-1:0]] <= {ch_first, ch_cnt - 8'd1};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch_open  <= 1'b0;
            ch_start <= {BA{1'b0}};
            ch_n     <= 8'd0;
            aq_wp    <= {(AQ_LOG2+1){1'b0}};
        end else if (p_vld) begin
            if (p_wlast) begin
                ch_open <= 1'b0;
                aq_wp   <= aq_wp + 1'b1;
            end else begin
                ch_open  <= 1'b1;
                ch_start <= ch_first;
                ch_n     <= ch_cnt;
            end
        end
    end

    // =========================================================================
    // W: the FIFO's registered read (q) and the output register (o)
    // =========================================================================
    // A stage takes a beat only when it is empty or being emptied in the same
    // cycle, so WDATA never changes under a WVALID that was not accepted.
    reg  [WE-1:0]   ram_q, o_reg;
    reg             q_vld, o_vld;
    wire            w_take  = o_vld && m_wready;
    wire            o_free  = !o_vld || w_take;
    wire            q_move  = q_vld && o_free;
    wire            q_free  = !q_vld || q_move;
    wire            f_issue = (wq_fp != wq_wp) && q_free;

    always @(posedge clk) if (f_issue) ram_q <= wmem[wq_fp[WQ_LOG2-1:0]];

    assign m_wvalid = o_vld;
    assign m_wdata  = o_reg[DW-1:0];
    assign m_wlast  = o_reg[WE-1];
    generate
        for (g = 0; g < WPB; g = g + 1) begin : g_strb
            assign m_wstrb[g*4 +: 4] = {4{o_reg[DW + g]}};
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wq_wp  <= {(WQ_LOG2+1){1'b0}};
            wq_fp  <= {(WQ_LOG2+1){1'b0}};
            wq_cnt <= {(WQ_LOG2+1){1'b0}};
            q_vld  <= 1'b0;
            o_vld  <= 1'b0;
            o_reg  <= {WE{1'b0}};
        end else begin
            if (p_vld)   wq_wp <= wq_wp + 1'b1;
            if (f_issue) wq_fp <= wq_fp + 1'b1;
            wq_cnt <= wq_cnt + {{WQ_LOG2{1'b0}}, p_vld} - {{WQ_LOG2{1'b0}}, w_take};

            if (f_issue)     q_vld <= 1'b1;
            else if (q_move) q_vld <= 1'b0;

            if (q_move) begin
                o_reg <= ram_q;
                o_vld <= 1'b1;
            end else if (w_take) begin
                o_vld <= 1'b0;
            end
        end
    end

    // =========================================================================
    // AW
    // =========================================================================
    reg  [BA-1:0]   aw_b;
    reg  [7:0]      aw_l;
    wire            aq_any = (aq_wp != aq_rp);

    // AWs loaded minus WLASTs taken at this port (AW_AHEAD above). Signed as a
    // guard only: W of the next burst could reach axi_mux's slice before its AW
    // is loaded, and an unsigned count would then wrap and stop AW for good. With
    // the present timing it cannot - the next AW loads the cycle after the last
    // WLAST, before the next burst's WLAST can leave (an unsigned mutant passes
    // dualaxi, 09.10.2026) - but nothing in the protocol promises that.
    localparam signed [5:0] AHEAD_MAX = AW_AHEAD[5:0];
    reg  signed [5:0] ahead;
    wire            w_end   = m_wvalid && m_wready && m_wlast;
    wire            aw_load = (!m_awvalid || m_awready) && aq_any && (ahead < AHEAD_MAX);

    assign m_awaddr = {aw_b, {(LG+2){1'b0}}};
    assign m_awlen  = aw_l;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_awvalid <= 1'b0;
            aw_b      <= {BA{1'b0}};
            aw_l      <= 8'd0;
            aq_rp     <= {(AQ_LOG2+1){1'b0}};
            ahead     <= 6'sd0;
        end else begin
            ahead <= ahead + (aw_load ? 6'sd1 : 6'sd0) - (w_end ? 6'sd1 : 6'sd0);
            if (!m_awvalid || m_awready) begin
                m_awvalid <= aw_load;
                if (aw_load) begin
                    {aw_b, aw_l} <= aq[aq_rp[AQ_LOG2-1:0]];
                    aq_rp        <= aq_rp + 1'b1;
                end
            end
        end
    end

    // =========================================================================
    // Room for the next burst, B, counters
    // =========================================================================
    wire [AQ_LOG2:0] aq_cnt = aq_wp - aq_rp;

    assign m_bready = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_room      <= 1'b0;
            o_aw_cnt    <= 16'd0;
            o_b_cnt     <= 16'd0;
            o_bresp_err <= 3'd0;
        end else begin
            s_room <= (wq_cnt <= WQ_ROOM) && (aq_cnt <= AQ_ROOM);
            if (m_awvalid && m_awready) o_aw_cnt <= o_aw_cnt + 16'd1;
            if (m_bvalid) begin
                o_b_cnt <= o_b_cnt + 16'd1;
                if (m_bresp != 2'b00 && o_bresp_err != 3'd7)
                    o_bresp_err <= o_bresp_err + 3'd1;
            end
        end
    end

endmodule
