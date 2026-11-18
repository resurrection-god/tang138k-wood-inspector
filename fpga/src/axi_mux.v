// ============================================================================
// Module      : axi_mux
// N:1 AXI4 interconnect in front of the DDR3 Memory Interface IP's AXI4 port
// (docs/ARCHITECTURE.md § 4.1; back-end rules: CLAUDE.md invariant 7).
//
// There is exactly one slave, so this is a pair of arbiters, not a crossbar:
// NW write masters share AW/W/B and NR read masters share AR/R, independently.
//
//   * Arbitration is round-robin per transaction, and the grant is REGISTERED:
//     once s_awvalid/s_arvalid is up it stays on the same master with the same
//     payload until the handshake, as AXI requires - a combinational pick could
//     switch masters under a waiting valid. Address and length are latched at
//     the grant (a master must hold them while its valid is up anyway).
//   * Every channel towards and from the IP is registered: AW/AR payload at the
//     grant, W and B through skid buffers (+1 cycle each), R through an input
//     register and a FIFO per read port (+2 cycles).
//   * RREADY towards the IP is constantly high: the IP mishandles a stalled R
//     channel (see the read-data section). An AR is granted only when its
//     port's FIFO can take the whole response, so read bursts are limited to
//     2^RQ_LOG2 beats and each master gets its own read-data bus.
//   * The transaction ID on the slave side is the master's port index; masters
//     themselves carry no ID. B and R are routed back by BID/RID, so several
//     transactions may be in flight and the slave may interleave read data of
//     different IDs beat by beat.
//   * W data follows AW order: every accepted AW pushes its port index into a
//     small FIFO, and W is taken from the head until that master's WLAST. A
//     master must therefore not wait for WREADY before raising AWVALID (none of
//     ours does).
//   * Burst size and type are fixed for the IP: AxSIZE = log2(DW/8) (full width
//     only, IPUG281: "must be 1/8 of s_axi_wdata width") and INCR.
// ============================================================================

`timescale 1ns / 1ps

module axi_mux #(
    parameter integer NW      = 2,      // write masters
    parameter integer NR      = 2,      // read masters
    parameter integer DW      = 256,    // data width
    parameter integer AW      = 30,     // address width
    parameter integer IDW     = 4,      // slave ID width, >= clog2(max(NW, NR))
    parameter integer WQ_LOG2 = 3,      // W-order FIFO depth = 2^WQ_LOG2 AWs in flight
    parameter integer RQ_LOG2 = 6       // R FIFO per read port: 2^RQ_LOG2 beats = longest read burst (<= 8)
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // ---- write masters (flattened, port k at [k*W +: W]) --------------------
    input  wire [NW-1:0]        m_awvalid,
    output wire [NW-1:0]        m_awready,
    input  wire [NW*AW-1:0]     m_awaddr,
    input  wire [NW*8-1:0]      m_awlen,
    input  wire [NW-1:0]        m_wvalid,
    output wire [NW-1:0]        m_wready,
    input  wire [NW*DW-1:0]     m_wdata,
    input  wire [NW*DW/8-1:0]   m_wstrb,
    input  wire [NW-1:0]        m_wlast,
    output wire [NW-1:0]        m_bvalid,
    input  wire [NW-1:0]        m_bready,
    output wire [1:0]           m_bresp,        // shared, qualified by m_bvalid

    // ---- read masters --------------------------------------------------------
    input  wire [NR-1:0]        m_arvalid,
    output wire [NR-1:0]        m_arready,
    input  wire [NR*AW-1:0]     m_araddr,
    input  wire [NR*8-1:0]      m_arlen,
    output wire [NR-1:0]        m_rvalid,
    input  wire [NR-1:0]        m_rready,
    output wire [NR*DW-1:0]     m_rdata,        // per port
    output wire [NR*2-1:0]      m_rresp,
    output wire [NR-1:0]        m_rlast,

    // ---- slave (the DDR3 IP) -------------------------------------------------
    output wire                 s_awvalid,
    input  wire                 s_awready,
    output wire [IDW-1:0]       s_awid,
    output wire [AW-1:0]        s_awaddr,
    output wire [7:0]           s_awlen,
    output wire [2:0]           s_awsize,
    output wire [1:0]           s_awburst,
    output wire                 s_wvalid,
    input  wire                 s_wready,
    output wire [DW-1:0]        s_wdata,
    output wire [DW/8-1:0]      s_wstrb,
    output wire                 s_wlast,
    input  wire                 s_bvalid,
    output wire                 s_bready,
    input  wire [IDW-1:0]       s_bid,
    input  wire [1:0]           s_bresp,
    output wire                 s_arvalid,
    input  wire                 s_arready,
    output wire [IDW-1:0]       s_arid,
    output wire [AW-1:0]        s_araddr,
    output wire [7:0]           s_arlen,
    output wire [2:0]           s_arsize,
    output wire [1:0]           s_arburst,
    input  wire                 s_rvalid,
    output wire                 s_rready,
    input  wire [IDW-1:0]       s_rid,
    input  wire [DW-1:0]        s_rdata,
    input  wire [1:0]           s_rresp,
    input  wire                 s_rlast,

    // ---- diagnostics ---------------------------------------------------------
    output reg  [15:0]          o_bad_id        // B/R beats whose ID names no master
);

    // log2 of the byte width: 256 bits -> 5
    function integer clog2;
        input integer v;
        integer t;
        begin
            clog2 = 0;
            t = v - 1;
            while (t > 0) begin
                clog2 = clog2 + 1;
                t = t >> 1;
            end
        end
    endfunction

    localparam integer SIZE = clog2(DW / 8);
    localparam integer WQD  = 1 << WQ_LOG2;
    localparam integer   NW_M1   = NW - 1;
    localparam integer   NR_M1   = NR - 1;
    localparam [IDW-1:0] NW_LAST = NW_M1[IDW-1:0];
    localparam [IDW-1:0] NR_LAST = NR_M1[IDW-1:0];

    assign s_awsize  = SIZE[2:0];
    assign s_arsize  = SIZE[2:0];
    assign s_awburst = 2'b01;           // INCR
    assign s_arburst = 2'b01;

    // Round-robin pick: the first requester at or after `start`, cyclically.
    function [IDW-1:0] rr_pick;
        input [31:0]    req;
        input integer   n;
        input [IDW-1:0] start;
        integer k, idx;
        reg     found;
        begin
            rr_pick = {IDW{1'b0}};
            found   = 1'b0;
            for (k = 0; k < n; k = k + 1) begin
                idx = ({{(32-IDW){1'b0}}, start} + k) % n;
                if (!found && req[idx]) begin
                    rr_pick = idx[IDW-1:0];
                    found   = 1'b1;
                end
            end
        end
    endfunction

    // One-hot of an ID, so routing never indexes a vector with a wider value.
    function [31:0] onehot;
        input [IDW-1:0] id;
        begin
            onehot = 32'd1 << id;
        end
    endfunction

    // =========================================================================
    // Write address: registered grant
    // =========================================================================
    reg                 aw_act;
    reg  [IDW-1:0]      aw_idx;
    reg  [IDW-1:0]      aw_rr;
    reg  [AW-1:0]       aw_addr_r;      // payload latched at the grant: the master holds it
    reg  [7:0]          aw_len_r;       // until the handshake, and no port mux sits on the IP's path

    // W-order FIFO: port indices of accepted AWs whose data has not finished.
    reg  [IDW-1:0]      wq [0:WQD-1];
    reg  [WQ_LOG2:0]    wq_wr, wq_rd;
    wire                wq_full  = (wq_wr[WQ_LOG2] != wq_rd[WQ_LOG2]) &&
                                   (wq_wr[WQ_LOG2-1:0] == wq_rd[WQ_LOG2-1:0]);
    // The queue head is kept in flops, computed one clock ahead from the next
    // pointers: wq_rd -> wq[] read -> port select used to drive the clock enable
    // of the 288 wo_d/wo_s flops, a B4 worst path (RTL_RULES 3.1). The sim-only
    // check at the end of the module compares them with the plain read each clock.
    reg                 wq_nemp;        // = (wq_wr != wq_rd)
    reg  [IDW-1:0]      wq_head;        // = wq[wq_rd] while wq_nemp
    reg  [NW-1:0]       head_oh;        // = one-hot of wq_head

    wire                aw_hs    = s_awvalid && s_awready;

    wire [IDW-1:0]      aw_pick  = rr_pick({{(32-NW){1'b0}}, m_awvalid}, NW, aw_rr);

    assign s_awvalid = aw_act;
    assign s_awid    = aw_idx;
    assign s_awaddr  = aw_addr_r;
    assign s_awlen   = aw_len_r;

    // The master's AWREADY is a registered pulse the cycle after its grant -
    // the payload is latched by then - not the IP's AWREADY passed through: a
    // master may sit far from the IP (a camera writer next to its D-PHY), and
    // the IP's ready arrives late (full build, 08.10.2026).
    reg  [NW-1:0]       aw_rdy_q;
    wire [31:0]         aw_pick_oh = onehot(aw_pick);
    assign m_awready = aw_rdy_q;

    genvar g;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            aw_act <= 1'b0;
            aw_idx <= {IDW{1'b0}};
            aw_rr  <= {IDW{1'b0}};
            aw_addr_r <= {AW{1'b0}};
            aw_len_r  <= 8'd0;
            aw_rdy_q  <= {NW{1'b0}};
            wq_wr  <= {(WQ_LOG2+1){1'b0}};
        end else begin
            aw_rdy_q <= {NW{1'b0}};
            if (aw_hs) begin
                wq[wq_wr[WQ_LOG2-1:0]] <= aw_idx;
                wq_wr  <= wq_wr + 1'b1;
                aw_act <= 1'b0;
                aw_rr  <= (aw_idx == NW_LAST) ? {IDW{1'b0}} : aw_idx + 1'b1;
            end else if (!aw_act && (|(m_awvalid & ~aw_rdy_q)) && !wq_full) begin
                aw_idx    <= aw_pick;
                aw_addr_r <= m_awaddr[aw_pick*AW +: AW];
                aw_len_r  <= m_awlen[aw_pick*8 +: 8];
                aw_act    <= 1'b1;
                aw_rdy_q  <= aw_pick_oh[NW-1:0];
            end
        end
    end

    // =========================================================================
    // Write data: from the head of the W-order FIFO until its WLAST
    // =========================================================================
    // Register slice (skid buffer) towards the IP: s_w* come from flip-flops,
    // and the masters' wready depends only on the slice's own state. Without
    // it the head-of-queue selection ran combinationally into the IP's write
    // logic and missed 100 MHz (axi_probe, 08.10.2026: -0.42 ns).
    // Each master's W enters a two-entry register slice of its own first
    // (axi_mux_rs): the mux's head-of-queue decision then steers only the
    // slice's pointers, never a master's data path. Without it the decision
    // ran combinationally into l4_axi_wr's 290-flop drain enable across the
    // die and missed 100 MHz by 4.8 ns in the full build (08.10.2026).
    wire [NW-1:0]       i_wvalid, i_wready, i_wlast;
    wire [NW*DW-1:0]    i_wdata;
    wire [NW*DW/8-1:0]  i_wstrb;
    generate
        for (g = 0; g < NW; g = g + 1) begin : g_wrs
            axi_mux_rs #(.W(DW + DW/8 + 1)) u_rs (
                .clk(clk), .rst_n(rst_n),
                .i_valid(m_wvalid[g]), .i_ready(m_wready[g]),
                .i_data({m_wlast[g], m_wstrb[g*(DW/8) +: DW/8], m_wdata[g*DW +: DW]}),
                .o_valid(i_wvalid[g]), .o_ready(i_wready[g]),
                .o_data({i_wlast[g], i_wstrb[g*(DW/8) +: DW/8], i_wdata[g*DW +: DW]})
            );
        end
    endgenerate

    wire            u_wvalid = wq_nemp && |(i_wvalid & head_oh);
    wire [DW-1:0]   u_wdata  = i_wdata[wq_head*DW +: DW];
    wire [DW/8-1:0] u_wstrb  = i_wstrb[wq_head*(DW/8) +: (DW/8)];
    wire            u_wlast  = |(i_wlast & head_oh);

    reg             ws_v, ws_l, wo_v, wo_l;
    reg  [DW-1:0]   ws_d, wo_d;
    reg  [DW/8-1:0] ws_s, wo_s;
    wire            u_wready = !ws_v;
    wire            u_whs    = u_wvalid && u_wready;
    wire            wo_load  = !wo_v || s_wready;

    assign s_wvalid = wo_v;
    assign s_wdata  = wo_d;
    assign s_wstrb  = wo_s;
    assign s_wlast  = wo_l;

    generate
        for (g = 0; g < NW; g = g + 1) begin : g_w
            assign i_wready[g] = wq_nemp && head_oh[g] && u_wready;
        end
    endgenerate

    // Next queue pointers and the head they will point at. An AW accepted on
    // this edge into the slot the head moves to is bypassed: wq[] still holds
    // the old entry there.
    wire [WQ_LOG2:0]    wq_rd_n   = wq_rd + {{WQ_LOG2{1'b0}}, (u_whs && u_wlast)};
    wire [WQ_LOG2:0]    wq_wr_n   = wq_wr + {{WQ_LOG2{1'b0}}, aw_hs};
    wire                wq_byp    = aw_hs && (wq_wr[WQ_LOG2-1:0] == wq_rd_n[WQ_LOG2-1:0]);
    wire [IDW-1:0]      wq_head_n = wq_byp ? aw_idx : wq[wq_rd_n[WQ_LOG2-1:0]];
    wire [31:0]         head_oh_n = onehot(wq_head_n);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wq_nemp <= 1'b0;
            wq_head <= {IDW{1'b0}};
            head_oh <= {NW{1'b0}};
        end else begin
            wq_nemp <= (wq_wr_n != wq_rd_n);
            wq_head <= wq_head_n;
            head_oh <= head_oh_n[NW-1:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wq_rd <= {(WQ_LOG2+1){1'b0}};
            ws_v <= 1'b0; ws_l <= 1'b0; ws_d <= {DW{1'b0}}; ws_s <= {(DW/8){1'b0}};
            wo_v <= 1'b0; wo_l <= 1'b0; wo_d <= {DW{1'b0}}; wo_s <= {(DW/8){1'b0}};
        end else begin
            if (u_whs && u_wlast) wq_rd <= wq_rd + 1'b1;
            if (wo_load) begin
                if (ws_v) begin
                    wo_v <= 1'b1; wo_d <= ws_d; wo_s <= ws_s; wo_l <= ws_l;
                    ws_v <= 1'b0;
                end else begin
                    wo_v <= u_wvalid;
                    if (u_wvalid) begin wo_d <= u_wdata; wo_s <= u_wstrb; wo_l <= u_wlast; end
                end
            end else if (u_whs) begin
                ws_v <= 1'b1; ws_d <= u_wdata; ws_s <= u_wstrb; ws_l <= u_wlast;
            end
        end
    end

    // =========================================================================
    // Write response: routed by BID
    // =========================================================================
    // Register slice from the IP, as on R: the IP's BID leaves its AW-ID FIFO
    // late in the cycle, and routing it straight into a master's B counter
    // missed 100 MHz (axi_probe, 08.10.2026: -0.44 ns). Routing by BID works
    // on the registered copy; one cycle of write-response latency.
    reg             bs_v, bo_v;
    reg  [IDW-1:0]  bs_id, bo_id;
    reg  [1:0]      bs_r, bo_r;
    wire [31:0]     bid_oh   = onehot(bo_id);
    wire            bid_ok   = |bid_oh[NW-1:0];
    wire            d_bready = bid_ok ? |(m_bready & bid_oh[NW-1:0]) : 1'b1;   // drain a stray ID
    wire            bo_load  = !bo_v || d_bready;
    wire            s_bhs    = s_bvalid && s_bready;

    assign s_bready = !bs_v;
    generate
        for (g = 0; g < NW; g = g + 1) begin : g_b
            assign m_bvalid[g] = bo_v && (bo_id == g);
        end
    endgenerate
    assign m_bresp  = bo_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bs_v <= 1'b0; bs_id <= {IDW{1'b0}}; bs_r <= 2'b00;
            bo_v <= 1'b0; bo_id <= {IDW{1'b0}}; bo_r <= 2'b00;
        end else begin
            if (bo_load) begin
                if (bs_v) begin
                    bo_v <= 1'b1; bo_id <= bs_id; bo_r <= bs_r;
                    bs_v <= 1'b0;
                end else begin
                    bo_v <= s_bvalid;
                    if (s_bvalid) begin bo_id <= s_bid; bo_r <= s_bresp; end
                end
            end else if (s_bhs) begin
                bs_v <= 1'b1; bs_id <= s_bid; bs_r <= s_bresp;
            end
        end
    end

    // =========================================================================
    // Read address: registered grant
    // =========================================================================
    reg                 ar_act;
    reg  [IDW-1:0]      ar_idx;
    reg  [IDW-1:0]      ar_rr;
    reg  [AW-1:0]       ar_addr_r;      // latched at the grant, as on AW
    reg  [7:0]          ar_len_r;
    wire [NR-1:0]       ar_ok;          // the port's R FIFO has room for this burst (below)
    wire                ar_hs   = s_arvalid && s_arready;
    wire [IDW-1:0]      ar_pick = rr_pick({{(32-NR){1'b0}}, m_arvalid & ar_ok}, NR, ar_rr);

    assign s_arvalid = ar_act;
    assign s_arid    = ar_idx;
    assign s_araddr  = ar_addr_r;
    assign s_arlen   = ar_len_r;

    // ARREADY towards the master: a registered pulse after the grant, as on AW.
    reg  [NR-1:0]       ar_rdy_q;
    wire [31:0]         ar_pick_oh = onehot(ar_pick);
    assign m_arready = ar_rdy_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ar_act <= 1'b0;
            ar_idx <= {IDW{1'b0}};
            ar_rr  <= {IDW{1'b0}};
            ar_addr_r <= {AW{1'b0}};
            ar_len_r  <= 8'd0;
            ar_rdy_q  <= {NR{1'b0}};
        end else begin
            ar_rdy_q <= {NR{1'b0}};
            if (ar_hs) begin
                ar_act <= 1'b0;
                ar_rr  <= (ar_idx == NR_LAST) ? {IDW{1'b0}} : ar_idx + 1'b1;
            end else if (!ar_act && (|(m_arvalid & ar_ok))) begin
                ar_idx    <= ar_pick;
                ar_addr_r <= m_araddr[ar_pick*AW +: AW];
                ar_len_r  <= m_arlen[ar_pick*8 +: 8];
                ar_act    <= 1'b1;
                ar_rdy_q  <= ar_pick_oh[NR-1:0];
            end
        end
    end

    // =========================================================================
    // Read data: never back-pressured, one FIFO per read port
    // =========================================================================
    // The IP must never see RREADY low. On the board (axi_probe, 08.10.2026)
    // its R channel, stalled by RREADY = 0, offered a phantom beat - the ID
    // and RLAST of the next response with the data of the previous one - and
    // a response went missing later: its ID and data FIFOs (rinfo_axi_fifo,
    // rdata_axi_fifo) slip against each other. So every beat is taken the
    // cycle it is offered: registered (the IP's data leave a block RAM with no
    // output register), then written into the FIFO of its port. An AR is
    // granted only while its port's FIFO has room for the whole response
    // (credits counted in beats, reserved at the grant, returned as the
    // master takes them), so the FIFO cannot overflow; a burst longer than
    // 2^RQ_LOG2 beats is never granted. The FIFOs also end head-of-line
    // blocking: a master that is slow to take its data (l5_axi_rd unpacks a
    // beat into eight Lane 5 words) holds up no other port.
    // Latency: IP -> input register -> FIFO -> output register, 2 cycles more
    // than a straight wire.
    localparam integer RQD = 1 << RQ_LOG2;
    localparam [9:0]   RQD_L = RQD[9:0];

    assign s_rready = 1'b1;

    reg             rin_v, rin_l;
    reg  [IDW-1:0]  rin_id;
    reg  [1:0]      rin_r;
    reg  [DW-1:0]   rin_d;
    wire [31:0]     rin_oh = onehot(rin_id);
    wire            rin_ok = |rin_oh[NR-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rin_v <= 1'b0; rin_l <= 1'b0; rin_id <= {IDW{1'b0}}; rin_r <= 2'b00;
        end else begin
            rin_v <= s_rvalid; rin_l <= s_rlast; rin_id <= s_rid; rin_r <= s_rresp;
        end
    end
    always @(posedge clk) rin_d <= s_rdata;

    wire [NR-1:0]   r_ovf;                  // a beat arrived for a full FIFO: credits are wrong

    generate
        for (g = 0; g < NR; g = g + 1) begin : g_r
            reg  [DW+2:0]       mem [0:RQD-1];      // {last, resp, data}
            reg  [RQ_LOG2:0]    wp, rp;
            reg  [DW+2:0]       q;
            reg                 qv;
            reg  [RQ_LOG2:0]    res;                // beats reserved: granted, not yet taken
            wire                we     = rin_v && rin_id == g;
            wire                ram_ne = (wp != rp);
            wire                rs_ready;           // the output slice can take q
            wire                pop    = qv && rs_ready;
            wire                ld     = (!qv || pop) && ram_ne;
            // The reservation is booked the cycle after the grant (ar_rdy_q),
            // from the length the grant latched - the master's ARLEN comes from
            // far away. A granted burst fits (arlen + 1 <= RQD), so its length
            // fits in RQ_LOG2 bits.
            wire [RQ_LOG2:0]    res_add = ar_rdy_q[g] ? {1'b0, ar_len_r[RQ_LOG2-1:0]} + 1'b1
                                                       : {(RQ_LOG2+1){1'b0}};
            reg                 ok_q;

            always @(posedge clk) begin
                if (we) mem[wp[RQ_LOG2-1:0]] <= {rin_l, rin_r, rin_d};
                if (ld) q <= mem[rp[RQ_LOG2-1:0]];
            end

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    wp <= {(RQ_LOG2+1){1'b0}}; rp <= {(RQ_LOG2+1){1'b0}}; qv <= 1'b0;
                    res <= {(RQ_LOG2+1){1'b0}}; ok_q <= 1'b0;
                end else begin
                    if (we) wp <= wp + 1'b1;
                    if (ld)       begin rp <= rp + 1'b1; qv <= 1'b1; end
                    else if (pop) qv <= 1'b0;
                    res <= res + res_add - {{RQ_LOG2{1'b0}}, pop};
                    // Room for the whole response (res + arlen + 1 <= RQD), decided
                    // a cycle ahead from the request the master is holding. Not in
                    // the cycle its previous request is taken (ar_rdy_q): the next
                    // one may bring a new length, and this one's reservation is not
                    // booked yet. Pops in between only make room.
                    ok_q <= m_arvalid[g] && !ar_rdy_q[g] &&
                            (({{(9-RQ_LOG2){1'b0}}, res} + {2'b00, m_arlen[g*8 +: 8]} + 10'd1) <= RQD_L);
                end
            end

            assign ar_ok[g]   = ok_q;
            assign r_ovf[g]   = we && (wp[RQ_LOG2] != rp[RQ_LOG2]) && (wp[RQ_LOG2-1:0] == rp[RQ_LOG2-1:0]);
            // Output slice: the master's RREADY steers only its pointers, not
            // the FIFO's block RAM (a master may sit far away, as on W).
            axi_mux_rs #(.W(DW + 3)) u_rs (
                .clk(clk), .rst_n(rst_n),
                .i_valid(qv), .i_ready(rs_ready), .i_data(q),
                .o_valid(m_rvalid[g]), .o_ready(m_rready[g]),
                .o_data({m_rlast[g], m_rresp[g*2 +: 2], m_rdata[g*DW +: DW]})
            );
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            o_bad_id <= 16'd0;
        else if (((bo_v && bo_load && !bid_ok) || (rin_v && !rin_ok) || (|r_ovf)) && o_bad_id != 16'hFFFF)
            o_bad_id <= o_bad_id + 16'd1;
    end

    // synthesis translate_off
    // Registered queue head == the plain FIFO read it replaces (RTL_RULES 3.1).
    wire [31:0] chk_oh = onehot(wq[wq_rd[WQ_LOG2-1:0]]);
    always @(posedge clk or negedge rst_n) begin
        if (rst_n && (wq_nemp !== (wq_wr != wq_rd) ||
                      (wq_nemp && (wq_head !== wq[wq_rd[WQ_LOG2-1:0]] ||
                                   head_oh !== chk_oh[NW-1:0])))) begin
            $display("ERROR axi_mux %m: wq_nemp=%b wq_head=%0d head_oh=%b wq_rd=%0d wq_wr=%0d at %0t",
                     wq_nemp, wq_head, head_oh, wq_rd, wq_wr, $time);
            $fatal(1);
        end
    end
    // synthesis translate_on

endmodule


// ============================================================================
// axi_mux_rs - two-entry register slice for one valid/ready channel, used by
// axi_mux on every master-side W and R channel. Both directions come from
// flip-flops: i_ready and o_valid depend only on the occupancy count, o_data
// is a 2:1 choice between two registers. o_ready therefore steers three flops
// (count, read pointer), and i_valid only the data registers' enables - a far
// partner never reaches the wide data path combinationally. Full throughput:
// one transfer per cycle with o_ready held high. +1 cycle of latency.
// ============================================================================
module axi_mux_rs #(
    parameter integer W = 32
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         i_valid,
    output wire         i_ready,
    input  wire [W-1:0] i_data,
    output wire         o_valid,
    input  wire         o_ready,
    output wire [W-1:0] o_data
);
    reg  [W-1:0] d0, d1;
    reg          wp, rp;
    reg  [1:0]   cnt;
    wire         push = i_valid && i_ready;
    wire         pop  = o_valid && o_ready;

    assign i_ready = (cnt != 2'd2);
    assign o_valid = (cnt != 2'd0);
    assign o_data  = rp ? d1 : d0;

    always @(posedge clk) begin
        if (push && !wp) d0 <= i_data;
        if (push &&  wp) d1 <= i_data;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp <= 1'b0; rp <= 1'b0; cnt <= 2'd0;
        end else begin
            if (push) wp <= ~wp;
            if (pop)  rp <= ~rp;
            cnt <= cnt + {1'b0, push} - {1'b0, pop};
        end
    end
endmodule
