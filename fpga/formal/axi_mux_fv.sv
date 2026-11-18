// Formal harness for axi_mux (fpga/src/axi_mux.v): the N:1 AXI4 interconnect
// in front of the DDR3 IP (CLAUDE.md invariant 7, ERRORS 1.19, 1.20).
//
// Two write and two read masters, narrow data (the mux never looks at data;
// 8 free bits per beat tell beats apart), bursts of 1..4 beats, a W-order
// FIFO of 4 AWs, a read FIFO of 4 beats per port.
//
// Masters (assumptions) - AXI-legal and nothing more:
//   * AW / W / AR: a valid not taken stays, payload unchanged;
//   * each write master sends, per burst, exactly AWLEN+1 W beats with WLAST
//     on the last, in the order of its AWs; W may run ahead of its AW (it
//     does in cam_axi_wr) - the burst length is fixed by whichever of the two
//     shows first;
//   * a read burst is at most 2^RQ_LOG2 beats (a longer one is never granted
//     by design, axi_mux header);
//   * BREADY / RREADY: anything.
// Slave = the DDR3 IP (assumptions):
//   * AWREADY / WREADY / ARREADY: anything;
//   * B only for a burst whose last W beat it has taken, in that order, with
//     its ID; held until BREADY;
//   * R: per ID in AR order, IDs interleaved beat by beat as the slave likes,
//     RLAST on the burst's last beat; data and response free.
// Properties (scoreboards per master: filled on one side, checked on the
// other):
//   S1  the slave sees each master's AWs / ARs in order, with their address
//       and length, tagged with the master's index;
//   S2  W beats reach the slave in AW order, each from the master the
//       current AW belongs to, with its data, strobe and WLAST; WLAST closes
//       exactly AWLEN+1 beats; no W beat before its AW has been taken;
//   S3  every B / R beat reaches the master its ID names, in order, intact;
//   S4  the slave side is AXI-legal (valid held, payload stable), AxSIZE /
//       AxBURST fixed, RREADY never low (invariant 7, ERRORS 1.19);
//   S5  the master side is AXI-legal for B and R (held until ready), at most
//       one master sees BVALID;
//   S6  o_bad_id stays 0: no response is dropped as "no such master" and no
//       read FIFO overflows (the AR credit scheme holds).
// With LIVE = 1 the slave and the masters are always ready: then no AW / AR
// waits 12 cycles or more for its grant (L1).

`timescale 1ns / 1ps

module axi_mux_fv #(
    parameter integer LIVE = 0
)(
    input  wire              clk,
    input  wire              rst_n,
    // write masters
    input  wire [1:0]        m_awvalid,
    input  wire [15:0]       m_awaddr,      // 2 x 8
    input  wire [3:0]        m_awlen2,      // 2 x 2: AWLEN 0..3
    input  wire [1:0]        m_wvalid,
    input  wire [15:0]       m_wdata,       // 2 x 8
    input  wire [1:0]        m_wstrb,       // 2 x 1
    input  wire [1:0]        m_wlast,
    input  wire [1:0]        m_bready,
    input  wire [3:0]        w_pick,        // 2 x 2: length - 1 of a burst whose W shows first
    // read masters
    input  wire [1:0]        m_arvalid,
    input  wire [15:0]       m_araddr,
    input  wire [3:0]        m_arlen2,
    input  wire [1:0]        m_rready,
    // slave
    input  wire              s_awready,
    input  wire              s_wready,
    input  wire              s_bvalid,
    input  wire [1:0]        s_bid,
    input  wire [1:0]        s_bresp,
    input  wire              s_arready,
    input  wire              s_rvalid,
    input  wire [1:0]        s_rid,
    input  wire [7:0]        s_rdata,
    input  wire [1:0]        s_rresp,
    input  wire              s_rlast
);
    localparam integer NW = 2, NR = 2, DW = 8, AW = 8, IDW = 2;
    localparam integer WQ_LOG2 = 2, RQ_LOG2 = 2;

    wire [15:0] m_awlen = {6'd0, m_awlen2[3:2], 6'd0, m_awlen2[1:0]};
    wire [15:0] m_arlen = {6'd0, m_arlen2[3:2], 6'd0, m_arlen2[1:0]};

    wire [1:0]      m_awready, m_wready, m_bvalid, m_arready, m_rvalid, m_rlast;
    wire [1:0]      m_bresp;
    wire [15:0]     m_rdata;
    wire [3:0]      m_rresp;
    wire            s_awvalid, s_wvalid, s_wlast, s_bready, s_arvalid, s_rready;
    wire [IDW-1:0]  s_awid, s_arid;
    wire [AW-1:0]   s_awaddr, s_araddr;
    wire [7:0]      s_awlen, s_arlen;
    wire [2:0]      s_awsize, s_arsize;
    wire [1:0]      s_awburst, s_arburst;
    wire [DW-1:0]   s_wdata;
    wire [DW/8-1:0] s_wstrb;
    wire [15:0]     o_bad_id;

    axi_mux #(
        .NW(NW), .NR(NR), .DW(DW), .AW(AW), .IDW(IDW),
        .WQ_LOG2(WQ_LOG2), .RQ_LOG2(RQ_LOG2)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .m_awvalid(m_awvalid), .m_awready(m_awready), .m_awaddr(m_awaddr), .m_awlen(m_awlen),
        .m_wvalid(m_wvalid), .m_wready(m_wready), .m_wdata(m_wdata), .m_wstrb(m_wstrb),
        .m_wlast(m_wlast), .m_bvalid(m_bvalid), .m_bready(m_bready), .m_bresp(m_bresp),
        .m_arvalid(m_arvalid), .m_arready(m_arready), .m_araddr(m_araddr), .m_arlen(m_arlen),
        .m_rvalid(m_rvalid), .m_rready(m_rready), .m_rdata(m_rdata), .m_rresp(m_rresp),
        .m_rlast(m_rlast),
        .s_awvalid(s_awvalid), .s_awready(s_awready), .s_awid(s_awid), .s_awaddr(s_awaddr),
        .s_awlen(s_awlen), .s_awsize(s_awsize), .s_awburst(s_awburst),
        .s_wvalid(s_wvalid), .s_wready(s_wready), .s_wdata(s_wdata), .s_wstrb(s_wstrb),
        .s_wlast(s_wlast), .s_bvalid(s_bvalid), .s_bready(s_bready), .s_bid(s_bid),
        .s_bresp(s_bresp), .s_arvalid(s_arvalid), .s_arready(s_arready), .s_arid(s_arid),
        .s_araddr(s_araddr), .s_arlen(s_arlen), .s_arsize(s_arsize), .s_arburst(s_arburst),
        .s_rvalid(s_rvalid), .s_rready(s_rready), .s_rid(s_rid), .s_rdata(s_rdata),
        .s_rresp(s_rresp), .s_rlast(s_rlast),
        .o_bad_id(o_bad_id)
    );

    // ---- reset ------------------------------------------------------------------
    reg first = 1'b1;
    always @(posedge clk) first <= 1'b0;
    always @(*) begin
        if (first) assume (!rst_n && m_awvalid == 2'b00 && m_wvalid == 2'b00 &&
                           m_arvalid == 2'b00 && !s_bvalid && !s_rvalid);
        else       assume (rst_n);
    end

    // handshakes
    wire [1:0] maw_hs = m_awvalid & m_awready;
    wire [1:0] mw_hs  = m_wvalid  & m_wready;
    wire [1:0] mar_hs = m_arvalid & m_arready;
    wire [1:0] mb_hs  = m_bvalid  & m_bready;
    wire [1:0] mr_hs  = m_rvalid  & m_rready;
    wire       saw_hs = s_awvalid && s_awready;
    wire       sw_hs  = s_wvalid  && s_wready;
    wire       sb_hs  = s_bvalid  && s_bready;
    wire       sar_hs = s_arvalid && s_arready;
    wire       sr_hs  = s_rvalid  && s_rready;

    // =========================================================================
    // Slave model and its bookkeeping
    // =========================================================================
    // AWs taken by the slave, in order: {id[0], len} - W must follow this order
    reg  [2:0] awq [0:7];
    reg  [3:0] awq_wp = 4'd0, awq_rp = 4'd0;
    reg  [1:0] swb    = 2'd0;           // beats of the head burst taken
    wire [2:0] awq_head = awq[awq_rp[2:0]];
    wire       sw_k     = awq_head[2];  // master of the slave's current W burst
    // write bursts complete at the slave, awaiting B: ids in order
    reg        bq [0:7];
    reg  [3:0] bq_wp = 4'd0, bq_rp = 4'd0;
    reg        sbv_q = 1'b0, sbr_q = 1'b0;
    reg  [1:0] sbid_q, sbresp_q;
    always @(posedge clk) begin
        sbv_q <= !first && s_bvalid; sbr_q <= s_bready; sbid_q <= s_bid; sbresp_q <= s_bresp;
    end

    always @(*) if (!first) begin
        if (s_bvalid) assume (bq_wp != bq_rp && s_bid == {1'b0, bq[bq_rp[2:0]]});
        if (s_rvalid) assume (s_rid < NR);
        // the IP's queues are finite: it holds at most 6 write bursts between AW and B
        if ({1'b0, awq_wp - awq_rp} + {1'b0, bq_wp - bq_rp} >= 5'd6) assume (!s_awready);
        if (sbv_q && !sbr_q) assume (s_bvalid && s_bid == sbid_q && s_bresp == sbresp_q);
    end

    always @(posedge clk) if (!first) begin
        if (saw_hs) begin
            awq[awq_wp[2:0]] <= {s_awid[0], s_awlen[1:0]};
            awq_wp <= awq_wp + 4'd1;
        end
        if (sw_hs) begin
            swb <= s_wlast ? 2'd0 : swb + 2'd1;
            if (s_wlast) begin
                awq_rp <= awq_rp + 4'd1;
                bq[bq_wp[2:0]] <= sw_k;
                bq_wp <= bq_wp + 4'd1;
            end
        end
        if (sb_hs) bq_rp <= bq_rp + 4'd1;
    end

    // =========================================================================
    // Per master: the master model, its scoreboards, the slave's read
    // queue for its ID
    // =========================================================================
    // ---- views of the mux's write-data path used by the invariants
    wire [3:0] awq_n   = awq_wp - awq_rp;
    wire [3:0] bq_n    = bq_wp - bq_rp;
    wire [2:0] wq_n    = dut.wq_wr - dut.wq_rd;
    wire       wo_wl   = dut.wo_v && dut.wo_l;
    wire       ws_wl   = dut.ws_v && dut.ws_l;
    wire [2:0] stage_l = {2'd0, wo_wl} + {2'd0, ws_wl};
    wire [2:0] awq_h1  = awq[awq_rp[2:0] + 3'd1];
    wire       own_wo  = awq_head[2];                       // master of the beat in wo
    wire       own_ws  = wo_wl ? awq_h1[2] : awq_head[2];   // ... in ws
    wire [1:0] ws_pos  = wo_wl ? 2'd0 : swb + 2'd1;
    wire [1:0] ws_len  = wo_wl ? awq_h1[1:0] : awq_head[1:0];

    genvar gk;
    generate for (gk = 0; gk < 2; gk = gk + 1) begin : mst
        // ---- write master: burst lengths decided by its AW or first W beat
        reg  [1:0] dl [0:7];            // lengths - 1, ring indexed by burst number
        reg  [3:0] dcnt = 4'd0;         // bursts decided
        reg  [3:0] awn  = 4'd0;         // AWs taken
        reg  [3:0] wn   = 4'd0;         // W bursts finished
        reg  [3:0] sbd  = 4'd0;         // W bursts finished at the slave
        reg  [1:0] wb   = 2'd0;         // beats of the current W burst taken
        reg        awv_q = 1'b0, wv_q = 1'b0, arv_q = 1'b0;
        reg        awr_q, wr_q, arr_q;
        reg  [9:0] aw_q, ar_q;
        reg  [9:0] w_q;

        wire       awv  = m_awvalid[gk];
        wire       wv   = m_wvalid[gk];
        wire       arv  = m_arvalid[gk];
        wire [9:0] awp  = {m_awaddr[gk*8 +: 8], m_awlen2[gk*2 +: 2]};
        wire [9:0] arp  = {m_araddr[gk*8 +: 8], m_arlen2[gk*2 +: 2]};
        wire [9:0] wp   = {m_wlast[gk], m_wstrb[gk], m_wdata[gk*8 +: 8]};
        wire [1:0] wpick = w_pick[gk*2 +: 2];

        always @(posedge clk) begin
            awv_q <= !first && awv; awr_q <= m_awready[gk]; aw_q <= awp;
            wv_q  <= !first && wv;  wr_q  <= m_wready[gk];  w_q  <= wp;
            arv_q <= !first && arv; arr_q <= m_arready[gk]; ar_q <= arp;
        end

        always @(*) if (!first) begin
            if (awv_q && !awr_q) assume (awv && awp == aw_q);
            if (wv_q  && !wr_q)  assume (wv  && wp  == w_q);
            if (arv_q && !arr_q) assume (arv && arp == ar_q);
            if (awv) begin
                if (awn != dcnt) assume (awp[1:0] == dl[awn[2:0]]);
                else             assume ((dcnt - wn) < 4'd4 && (dcnt - sbd) < 4'd7);
            end
            if (wv) begin
                if (wn != dcnt) assume (wp[9] == (wb == dl[wn[2:0]]));
                else begin
                    assume (wb == 2'd0 && wp[9] == (wpick == 2'd0) && (dcnt - awn) < 4'd4 && (dcnt - sbd) < 4'd7);
                    if (awv && awn == dcnt) assume (wpick == awp[1:0]);
                end
            end
        end

        always @(posedge clk) if (!first) begin
            if (awv && awn == dcnt) begin
                dl[dcnt[2:0]] <= awp[1:0];
                dcnt <= dcnt + 4'd1;
            end else if (wv && wn == dcnt) begin
                dl[dcnt[2:0]] <= wpick;
                dcnt <= dcnt + 4'd1;
            end
            if (maw_hs[gk]) awn <= awn + 4'd1;
            if (sw_hs && s_wlast && sw_k == gk) sbd <= sbd + 4'd1;
            if (mw_hs[gk]) begin
                wb <= wp[9] ? 2'd0 : wb + 2'd1;
                if (wp[9]) wn <= wn + 4'd1;
            end
        end

        // ---- scoreboards
        // AWs / ARs taken at the master side, not yet at the slave
        reg  [9:0] eaw [0:7];
        reg  [3:0] eaw_wp = 4'd0, eaw_rp = 4'd0;
        reg  [9:0] ear [0:7];
        reg  [3:0] ear_wp = 4'd0, ear_rp = 4'd0;
        // W beats taken at the master side, not yet at the slave
        reg  [9:0] ew [0:7];
        reg  [3:0] ew_wp = 4'd0, ew_rp = 4'd0;
        // B responses / R beats taken at the slave side, not yet at the master
        reg  [1:0]  eb [0:7];
        reg  [3:0]  eb_wp = 4'd0, eb_rp = 4'd0;
        reg  [10:0] er [0:7];           // {last, resp, data}
        reg  [3:0]  er_wp = 4'd0, er_rp = 4'd0;
        // the slave's read bursts for this ID: lengths in AR order
        reg  [1:0]  rq [0:7];
        reg  [3:0]  rq_wp = 4'd0, rq_rp = 4'd0;
        reg  [1:0]  rbt = 2'd0;         // beats of the head read burst sent

        wire aw_here = saw_hs && s_awid == gk;
        wire ar_here = sar_hs && s_arid == gk;
        wire aw_e    = (eaw_wp == eaw_rp);
        wire ar_e    = (ear_wp == ear_rp);
        // the AW the slave takes is the queue head, or the master's AW being
        // taken in this very cycle
        wire [9:0] aw_exp = aw_e ? awp : eaw[eaw_rp[2:0]];
        wire [9:0] ar_exp = ar_e ? arp : ear[ear_rp[2:0]];
        wire       r_here = s_rvalid && s_rid == gk;

        always @(*) if (!first && r_here)
            assume (rq_wp != rq_rp && s_rlast == (rbt == rq[rq_rp[2:0]]));

        always @(posedge clk) if (!first) begin
            if (maw_hs[gk] && !(aw_here && aw_e)) begin
                eaw[eaw_wp[2:0]] <= awp;
                eaw_wp <= eaw_wp + 4'd1;
            end
            if (aw_here && !aw_e) eaw_rp <= eaw_rp + 4'd1;
            if (mar_hs[gk] && !(ar_here && ar_e)) begin
                ear[ear_wp[2:0]] <= arp;
                ear_wp <= ear_wp + 4'd1;
            end
            if (ar_here && !ar_e) ear_rp <= ear_rp + 4'd1;
            if (mw_hs[gk]) begin
                ew[ew_wp[2:0]] <= wp;
                ew_wp <= ew_wp + 4'd1;
            end
            if (sw_hs && sw_k == gk) ew_rp <= ew_rp + 4'd1;
            if (sb_hs && s_bid == gk) begin
                eb[eb_wp[2:0]] <= s_bresp;
                eb_wp <= eb_wp + 4'd1;
            end
            if (mb_hs[gk]) eb_rp <= eb_rp + 4'd1;
            if (r_here && s_rready) begin
                er[er_wp[2:0]] <= {s_rlast, s_rresp, s_rdata};
                er_wp <= er_wp + 4'd1;
                rbt <= s_rlast ? 2'd0 : rbt + 2'd1;
                if (s_rlast) rq_rp <= rq_rp + 4'd1;
            end
            if (mr_hs[gk]) er_rp <= er_rp + 4'd1;
            if (ar_here) begin
                rq[rq_wp[2:0]] <= s_arlen[1:0];
                rq_wp <= rq_wp + 4'd1;
            end
        end

        // ---- master-side B / R stability
        reg         b_st = 1'b0, r_st = 1'b0;
        reg  [1:0]  b_q;
        reg  [10:0] r_q;
        wire [10:0] rp_now = {m_rlast[gk], m_rresp[gk*2 +: 2], m_rdata[gk*8 +: 8]};
        always @(posedge clk) begin
            b_st <= !first && m_bvalid[gk] && !m_bready[gk];
            r_st <= !first && m_rvalid[gk] && !m_rready[gk];
            b_q  <= m_bresp;
            r_q  <= rp_now;
        end

        // ---- bounded liveness counters
        reg  [4:0] aw_wait = 5'd0, ar_wait = 5'd0;
        always @(posedge clk) if (!first) begin
            aw_wait <= (awv && !m_awready[gk]) ? aw_wait + 5'd1 : 5'd0;
            ar_wait <= (arv && !m_arready[gk]) ? ar_wait + 5'd1 : 5'd0;
        end


        // ---- invariants of this master (k-induction strengthening)
        reg        okc = 1'b0;          // the mux's ok_q recomputed one cycle back
        always @(posedge clk)
            okc <= !first && arv && !dut.ar_rdy_q[gk] &&
                   ({3'd0, dut.g_r[gk].res} + {4'd0, arp[1:0]} + 6'd1 <= 6'd4);

        wire [3:0] eaw_n = eaw_wp - eaw_rp;
        wire [3:0] ear_n = ear_wp - ear_rp;
        wire [3:0] ew_n  = ew_wp - ew_rp;
        wire [3:0] eb_n  = eb_wp - eb_rp;
        wire [3:0] er_n  = er_wp - er_rp;
        wire [3:0] rq_n  = rq_wp - rq_rp;
        wire       aw_pend = dut.aw_act && dut.aw_idx == gk && !dut.aw_rdy_q[gk];
        wire       ar_pend = dut.ar_act && dut.ar_idx == gk && !dut.ar_rdy_q[gk];
        wire       w_wo  = dut.wo_v && own_wo == gk;
        wire       w_ws  = dut.ws_v && own_ws == gk;
        wire [1:0] s_cnt = dut.g_wrs[gk].u_rs.cnt;
        wire [9:0] s_old = dut.g_wrs[gk].u_rs.rp ? dut.g_wrs[gk].u_rs.d1 : dut.g_wrs[gk].u_rs.d0;
        wire [9:0] s_new = dut.g_wrs[gk].u_rs.rp ? dut.g_wrs[gk].u_rs.d0 : dut.g_wrs[gk].u_rs.d1;
        wire       rin_k = dut.rin_v && dut.rin_id == gk;
        wire [2:0] f_n   = dut.g_r[gk].wp - dut.g_r[gk].rp;
        wire [1:0] r_cnt = dut.g_r[gk].u_rs.cnt;
        wire [10:0] r_old = dut.g_r[gk].u_rs.rp ? dut.g_r[gk].u_rs.d1 : dut.g_r[gk].u_rs.d0;
        wire [10:0] r_new = dut.g_r[gk].u_rs.rp ? dut.g_r[gk].u_rs.d0 : dut.g_r[gk].u_rs.d1;

        // the beats this master has handed over and the slave has not taken,
        // oldest first: wo, ws, then its register slice
        reg  [9:0] wch [0:3];
        reg  [2:0] wch_n;
        // the R beats on their way to this master, oldest first: output
        // slice, q, the FIFO, the input register
        reg  [10:0] rch [0:7];
        reg  [3:0]  rch_n;
        // walks
        reg  [3:0] wb_b, aq_b;
        reg  [1:0] wb_p;
        reg        w_ok, aq_ok;
        reg  [4:0] outst;
        integer    j;
        always @(*) begin
            wch_n = 3'd0;
            for (j = 0; j < 4; j = j + 1) wch[j] = 10'd0;
            if (w_wo) begin wch[wch_n[1:0]] = {dut.wo_l, dut.wo_s, dut.wo_d}; wch_n = wch_n + 3'd1; end
            if (w_ws) begin wch[wch_n[1:0]] = {dut.ws_l, dut.ws_s, dut.ws_d}; wch_n = wch_n + 3'd1; end
            if (s_cnt != 2'd0) begin wch[wch_n[1:0]] = s_old; wch_n = wch_n + 3'd1; end
            if (s_cnt == 2'd2) begin wch[wch_n[1:0]] = s_new; wch_n = wch_n + 3'd1; end

            rch_n = 4'd0;
            for (j = 0; j < 8; j = j + 1) rch[j] = 11'd0;
            if (r_cnt != 2'd0) begin rch[rch_n[2:0]] = r_old; rch_n = rch_n + 4'd1; end
            if (r_cnt == 2'd2) begin rch[rch_n[2:0]] = r_new; rch_n = rch_n + 4'd1; end
            if (dut.g_r[gk].qv) begin rch[rch_n[2:0]] = dut.g_r[gk].q; rch_n = rch_n + 4'd1; end
            for (j = 0; j < 4; j = j + 1)
                if (j < f_n) begin
                    rch[rch_n[2:0]] = dut.g_r[gk].mem[dut.g_r[gk].rp[1:0] + j[1:0]];
                    rch_n = rch_n + 4'd1;
                end
            if (rin_k) begin rch[rch_n[2:0]] = {dut.rin_l, dut.rin_r, dut.rin_d}; rch_n = rch_n + 4'd1; end

            // WLAST positions of this master's pending beats follow its burst lengths
            wb_b = sbd;
            wb_p = (awq_n != 4'd0 && awq_head[2] == gk) ? swb : 2'd0;
            w_ok = 1'b1;
            for (j = 0; j < 8; j = j + 1)
                if (j < ew_n) begin
                    if (ew[ew_rp[2:0] + j[2:0]][9] != (wb_p == dl[wb_b[2:0]])) w_ok = 1'b0;
                    if (ew[ew_rp[2:0] + j[2:0]][9]) begin wb_b = wb_b + 4'd1; wb_p = 2'd0; end
                    else                            wb_p = wb_p + 2'd1;
                end
            // this master's AWs at the slave carry its burst lengths, in order
            aq_b  = sbd;
            aq_ok = 1'b1;
            for (j = 0; j < 8; j = j + 1)
                if (j < awq_n && awq[awq_rp[2:0] + j[2:0]][2] == gk) begin
                    if (awq[awq_rp[2:0] + j[2:0]][1:0] != dl[aq_b[2:0]]) aq_ok = 1'b0;
                    aq_b = aq_b + 4'd1;
                end
            // read beats the slave still owes this master
            outst = 5'd0;
            for (j = 0; j < 8; j = j + 1)
                if (j < rq_n) outst = outst + {3'd0, rq[rq_rp[2:0] + j[2:0]]} + 5'd1;
            outst = outst - {3'd0, rbt};
        end

        always @(posedge clk) if (!first && rst_n) begin
            // the two register slices of this master: at most 2 entries, write pointer
            // one ahead of the read pointer per entry (proved alone in axi_mux_rs)
            assert (s_cnt <= 2'd2 && dut.g_wrs[gk].u_rs.wp == (dut.g_wrs[gk].u_rs.rp ^ s_cnt[0]));
            assert (r_cnt <= 2'd2 && dut.g_r[gk].u_rs.wp == (dut.g_r[gk].u_rs.rp ^ r_cnt[0]));
            assert (f_n <= 3'd4);
            // AW / AR grant
            if (dut.aw_rdy_q[gk]) assert (dut.aw_act && dut.aw_idx == gk && awv_q && !awr_q &&
                                          aw_q == {dut.aw_addr_r, dut.aw_len_r[1:0]});
            assert (eaw_n == {3'd0, aw_pend});
            if (aw_pend) assert (eaw[eaw_rp[2:0]] == {dut.aw_addr_r, dut.aw_len_r[1:0]});
            if (dut.ar_rdy_q[gk]) assert (dut.ar_act && dut.ar_idx == gk && arv_q && !arr_q &&
                                          ar_q == {dut.ar_addr_r, dut.ar_len_r[1:0]});
            assert (ear_n == {3'd0, ar_pend});
            if (ar_pend) assert (ear[ear_rp[2:0]] == {dut.ar_addr_r, dut.ar_len_r[1:0]});
            // master bookkeeping
            assert ((dcnt - sbd) <= 4'd7 && (wn - sbd) <= (dcnt - sbd) && (awn - sbd) <= (dcnt - sbd));
            if (wb != 2'd0) assert (wn != dcnt);
            assert (awn == aq_b + eaw_n);
            assert (aq_ok);
            if (dut.aw_act && dut.aw_idx == gk) assert (dut.aw_len_r[1:0] == dl[aq_b[2:0]] && aq_b != dcnt);
            // W beats in flight
            assert (ew_n == {1'b0, wch_n});
            for (j = 0; j < 4; j = j + 1)
                if (j < wch_n) assert (wch[j] == ew[ew_rp[2:0] + j[2:0]]);
            assert (w_ok && wn == wb_b && wb == wb_p);
            // B in flight
            assert (eb_n == {3'd0, dut.bo_v && dut.bo_id == gk} + {3'd0, dut.bs_v && dut.bs_id == gk});
            if (dut.bo_v && dut.bo_id == gk) assert (eb[eb_rp[2:0]] == dut.bo_r);
            if (dut.bs_v && dut.bs_id == gk)
                assert (eb[eb_rp[2:0] + {2'd0, dut.bo_v && dut.bo_id == gk}] == dut.bs_r);
            // R beats in flight and the credit scheme
            assert (er_n == rch_n);
            for (j = 0; j < 8; j = j + 1)
                if (j < rch_n) assert (rch[j] == er[er_rp[2:0] + j[2:0]]);
            assert (rq_n <= 4'd4);
            if (rq_n == 4'd0) assert (rbt == 2'd0);
            else              assert (rbt <= rq[rq_rp[2:0]]);
            assert ({2'd0, dut.g_r[gk].res} + (dut.ar_rdy_q[gk] ? {3'd0, dut.ar_len_r[1:0]} + 5'd1 : 5'd0) ==
                    outst + (dut.ar_act && dut.ar_idx == gk ? {3'd0, dut.ar_len_r[1:0]} + 5'd1 : 5'd0) +
                    {4'd0, rin_k} + {2'd0, f_n} + {4'd0, dut.g_r[gk].qv});
            assert ({2'd0, dut.g_r[gk].res} + (dut.ar_rdy_q[gk] ? {3'd0, dut.ar_len_r[1:0]} + 5'd1 : 5'd0) <= 5'd4);
            assert (dut.g_r[gk].ok_q == okc);
            // ok_q looks one cycle back at a request the master still holds, and the
            // credits have only been returned since
            if (dut.g_r[gk].ok_q) assert (arv_q && !arr_q && {3'd0, dut.g_r[gk].res} + {4'd0, ar_q[1:0]} + 6'd1 <= 6'd4);
        end
        // ---- properties of this master
        always @(posedge clk) if (!first && rst_n) begin
            // S1
            if (aw_here) begin
                assert (!aw_e || maw_hs[gk]);
                assert ({s_awaddr, s_awlen[1:0]} == aw_exp && s_awlen[7:2] == 6'd0);
            end
            if (ar_here) begin
                assert (!ar_e || mar_hs[gk]);
                assert ({s_araddr, s_arlen[1:0]} == ar_exp && s_arlen[7:2] == 6'd0);
            end
            // S2: this master's beat at the slave
            if (sw_hs && sw_k == gk) begin
                assert (ew_wp != ew_rp);
                assert ({s_wlast, s_wstrb, s_wdata} == ew[ew_rp[2:0]]);
            end
            // S3
            if (mb_hs[gk]) begin
                assert (eb_wp != eb_rp);
                assert (m_bresp == eb[eb_rp[2:0]]);
            end
            if (mr_hs[gk]) begin
                assert (er_wp != er_rp);
                assert (rp_now == er[er_rp[2:0]]);
            end
            // S5
            if (b_st) assert (m_bvalid[gk] && m_bresp == b_q);
            if (r_st) assert (m_rvalid[gk] && rp_now == r_q);
            // L1
            if (LIVE) assert (aw_wait < 5'd12 && ar_wait < 5'd12);
        end
    end endgenerate

    // =========================================================================
    // Properties of the shared channels
    // =========================================================================
    reg            saw_st = 1'b0, sw_st = 1'b0, sar_st = 1'b0;
    reg  [12:0]    saw_q, sar_q;
    reg  [9:0]     sw_q;
    always @(posedge clk) begin
        saw_st <= !first && s_awvalid && !s_awready;
        sw_st  <= !first && s_wvalid  && !s_wready;
        sar_st <= !first && s_arvalid && !s_arready;
        saw_q  <= {s_awid, s_awaddr, s_awlen[2:0]};
        sar_q  <= {s_arid, s_araddr, s_arlen[2:0]};
        sw_q   <= {s_wlast, s_wstrb, s_wdata};
    end

    always @(posedge clk) if (!first && rst_n) begin
        // S4
        assert (s_rready);
        assert (s_awsize == 3'd0 && s_arsize == 3'd0 && s_awburst == 2'b01 && s_arburst == 2'b01);
        if (saw_st) assert (s_awvalid && {s_awid, s_awaddr, s_awlen[2:0]} == saw_q);
        if (sar_st) assert (s_arvalid && {s_arid, s_araddr, s_arlen[2:0]} == sar_q);
        if (sw_st)  assert (s_wvalid  && {s_wlast, s_wstrb, s_wdata} == sw_q);
        // S1: only real masters
        if (saw_hs) assert (s_awid < NW);
        if (sar_hs) assert (s_arid < NR);
        // S2: no W before its AW, WLAST after AWLEN+1 beats
        if (s_wvalid) assert (awq_wp != awq_rp);
        if (sw_hs)    assert (s_wlast == (swb == awq_head[1:0]));
        // S5
        assert (m_bvalid != 2'b11);
        // S6
        assert (o_bad_id == 16'd0);
    end

    generate if (LIVE) begin : g_live
        always @(*) if (!first)
            assume (s_awready && s_wready && s_arready && m_bready == 2'b11 && m_rready == 2'b11);
    end endgenerate

    // =========================================================================
    // Invariants of the shared state (k-induction strengthening)
    // =========================================================================
    integer i;
    always @(posedge clk) if (!first && rst_n) begin
        if (dut.aw_act) assert (dut.aw_idx < NW && dut.aw_len_r[7:2] == 6'd0 && wq_n < 3'd4);
        if (dut.ar_act) assert (dut.ar_idx < NR && dut.ar_len_r[7:2] == 6'd0);
        assert (dut.aw_rdy_q != 2'b11 && dut.ar_rdy_q != 2'b11);
        assert (wq_n <= 3'd4);
        assert (awq_n == {1'b0, wq_n} + {1'b0, stage_l});
        for (i = 0; i < 4; i = i + 1)
            if (i < wq_n) assert (dut.wq[dut.wq_rd[1:0] + i[1:0]] == {1'b0, awq[awq_rp[2:0] + stage_l + i[2:0]][2]});
        if (dut.ws_v) assert (dut.wo_v);
        if (dut.wo_v) assert (awq_n != 4'd0 && dut.wo_l == (swb == awq_head[1:0]));
        if (dut.ws_v) assert (dut.ws_l == (ws_pos == ws_len));
        if (wo_wl && dut.ws_v) assert (awq_n >= 4'd2);
        if (awq_n == 4'd0) assert (swb == 2'd0);
        else               assert (swb <= awq_head[1:0]);
        if (dut.bs_v) assert (dut.bo_v);
        if (dut.bo_v) assert (dut.bo_id < NW);
        if (dut.bs_v) assert (dut.bs_id < NW);
        if (dut.rin_v) assert (dut.rin_id < NR);
        assert (bq_n <= 4'd6 && awq_n <= 4'd6 && {1'b0, bq_n} + {1'b0, awq_n} <= 5'd6);
    end

    // the reset cycle can only be the first one: bookkeeping at its initial values
    always @(posedge clk) if (first) begin
        assert (awq_wp == 4'd0 && awq_rp == 4'd0 && swb == 2'd0 && bq_wp == 4'd0 && bq_rp == 4'd0);
        assert (!sbv_q && !saw_st && !sw_st && !sar_st);
        assert (mst[0].dcnt == 4'd0 && mst[0].awn == 4'd0 && mst[0].wn == 4'd0 && mst[0].sbd == 4'd0 && mst[0].wb == 2'd0);
        assert (mst[1].dcnt == 4'd0 && mst[1].awn == 4'd0 && mst[1].wn == 4'd0 && mst[1].sbd == 4'd0 && mst[1].wb == 2'd0);
        assert (mst[0].eaw_wp == mst[0].eaw_rp && mst[0].ear_wp == mst[0].ear_rp && mst[0].ew_wp == mst[0].ew_rp &&
                mst[0].eb_wp == mst[0].eb_rp && mst[0].er_wp == mst[0].er_rp && mst[0].rq_wp == mst[0].rq_rp && mst[0].rbt == 2'd0);
        assert (mst[1].eaw_wp == mst[1].eaw_rp && mst[1].ear_wp == mst[1].ear_rp && mst[1].ew_wp == mst[1].ew_rp &&
                mst[1].eb_wp == mst[1].eb_rp && mst[1].er_wp == mst[1].er_rp && mst[1].rq_wp == mst[1].rq_rp && mst[1].rbt == 2'd0);
        assert (!mst[0].okc && !mst[1].okc && !mst[0].awv_q && !mst[1].awv_q && !mst[0].arv_q && !mst[1].arv_q &&
                !mst[0].wv_q && !mst[1].wv_q && !mst[0].b_st && !mst[1].b_st && !mst[0].r_st && !mst[1].r_st);
    end
    // =========================================================================
    // Reachability
    // =========================================================================
    always @(posedge clk) if (!first && rst_n) begin
        cover (sw_hs && sw_k && awq_rp == 4'd2 && mst[0].ew_rp != 4'd0);             // W of both masters, in AW order
        cover (sr_hs && s_rid == 2'd1 && mst[0].rq_rp != mst[0].rq_wp && mst[0].rbt != 2'd0); // R of two ids interleaved
        cover (mst[0].r_st && mst[0].er_wp - mst[0].er_rp == 4'd4);                  // a read FIFO full, master stalling
        cover (sb_hs && m_bvalid[0] && !m_bready[0]);                                  // B piling up
        cover (m_wvalid[1] && mst[1].wn == mst[1].dcnt && !m_awvalid[1]);              // W shown before its AW
    end
endmodule
