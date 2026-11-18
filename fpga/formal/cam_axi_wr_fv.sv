// Formal harness for cam_axi_wr (fpga/src/cam_axi_wr.v): the camera writer's
// AXI4 write port (CLAUDE.md invariant 3, ERRORS 1.24).
//
// The writer side is modelled from what cam_channel's FSM does, not from the
// port's own code:
//   * a burst is 1..MAX_WORDS words, s_first on the first, s_last on the last,
//     any word-aligned start address; words may have gaps (the FSM has none,
//     so this is weaker than reality - a stronger proof);
//   * a burst starts only after s_room was seen high at some cycle after the
//     previous burst's last word, at least GAP cycles before its first word.
//     The FSM samples axi_room one cycle after the last word (ST_BURST_WAIT /
//     ST_IDLE) and the first word follows 4 cycles after the sample (ST_GO x2,
//     ST_WRITE, the wd_* register): GAP = 4 is the real writer, GAP = 1 a
//     harsher one;
//   * s_first / s_last / s_addr are free whenever s_en is low (dry rows raise
//     wd_first without wd_en).
// The AXI side is free: any AWREADY / WREADY / BVALID pattern.
//
// Scoreboard. The expected AXI traffic is built from the port's
// specification (its header), not from its code: word k of a burst at A goes
// to lane (A[4:2]+k) mod 8 of beat (A[4:2]+k)/8, lanes no word reached are
// masked, an AXI burst ends at the writer's last word or at a 4 KB page end.
// Each expected beat {WLAST, lanes, data} is queued the moment its last word
// arrives, each expected AW {address, AWLEN} the moment its burst closes.
//   D1  every W beat taken is the next expected one: WLAST, strobes (whole
//       nibbles, exactly the written lanes) and the data of written lanes;
//   D2  every AW taken is the next expected one, address and AWLEN; an AW
//       never leaves before its burst has closed, a beat never before it is
//       complete (both queues must be non-empty) - invariant 3;
//   A1  AW and W are AXI-legal: a valid not taken stays, payload unchanged;
//   A2  no burst crosses a 4 KB page;
//   A3  AWs issued minus WLASTs taken never exceeds AW_AHEAD (ERRORS 1.24);
//   A4  BREADY is always high.
// Nothing about the port's queue levels is assumed: an overflow of either
// queue breaks D1/D2. The I* assertions below tie the port's internal state
// to the scoreboard; they are what lets k-induction close (an unbounded
// proof), and each of them is checked like any other assertion.

`timescale 1ns / 1ps

module cam_axi_wr_fv #(
    parameter integer MAX_WORDS = 16,
    parameter integer WQ_LOG2   = 3,
    parameter integer AQ_LOG2   = 3,
    parameter integer AW_AHEAD  = 1,
    parameter integer GAP       = 1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        s_en,
    input  wire [1:0]  s_dbits,
    input  wire        s_first,
    input  wire        s_last,
    input  wire [31:0] s_addr,
    input  wire        m_awready,
    input  wire        m_wready,
    input  wire        m_bvalid,
    input  wire [1:0]  m_bresp
);
    localparam integer DW   = 256;
    localparam integer AW   = 16;       // 64 KB: 16 pages, enough to cross some
    localparam integer BA   = AW - 5;   // beat address width
    localparam integer WQD  = 1 << WQ_LOG2;
    localparam integer AQD  = 1 << AQ_LOG2;
    localparam integer MAXB = MAX_WORDS / 8 + 1;
    localparam integer SWI  = WQ_LOG2 + 1;           // expected-beat queue: 2*WQD entries
    localparam integer SAI  = AQ_LOG2 + 1;           // expected-AW queue: 2*AQD entries

    // Data abstraction: the port moves whole 32-bit words and never looks at
    // their contents, so two free bits per word (the lowest and the highest)
    // tell words apart and catch a word shifted by a lane or turned around;
    // the other 30 are tied to 0, and the .sby script removes the constant
    // flops. With all 32 bits free, BMC needed minutes per step past step 12.
    wire [31:0] s_data = {s_dbits[1], 30'd0, s_dbits[0]};

    wire            s_room, m_awvalid, m_wvalid, m_wlast, m_bready;
    wire [AW-1:0]   m_awaddr;
    wire [7:0]      m_awlen;
    wire [DW-1:0]   m_wdata;
    wire [DW/8-1:0] m_wstrb;
    wire [15:0]     o_aw_cnt, o_b_cnt;
    wire [2:0]      o_bresp_err;

    cam_axi_wr #(
        .DW(DW), .AW(AW), .MAX_WORDS(MAX_WORDS),
        .WQ_LOG2(WQ_LOG2), .AQ_LOG2(AQ_LOG2), .AW_AHEAD(AW_AHEAD)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .s_en(s_en), .s_data(s_data), .s_first(s_first), .s_last(s_last),
        .s_addr(s_addr), .s_room(s_room),
        .m_awvalid(m_awvalid), .m_awready(m_awready), .m_awaddr(m_awaddr),
        .m_awlen(m_awlen), .m_wvalid(m_wvalid), .m_wready(m_wready),
        .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast),
        .m_bvalid(m_bvalid), .m_bready(m_bready), .m_bresp(m_bresp),
        .o_aw_cnt(o_aw_cnt), .o_b_cnt(o_b_cnt), .o_bresp_err(o_bresp_err)
    );

    // two data bits per lane of a 256-bit beat
    function automatic [15:0] bits16(input [DW-1:0] d);
        integer l;
        begin
            for (l = 0; l < 8; l = l + 1) bits16[2*l +: 2] = {d[l*32 + 31], d[l*32]};
        end
    endfunction

    // lanes -> data-bit mask (2 bits per lane)
    function automatic [15:0] lanes2(input [7:0] m);
        integer l;
        begin
            for (l = 0; l < 8; l = l + 1) lanes2[2*l +: 2] = {2{m[l]}};
        end
    endfunction

    // a beat inside the port {wlast, lanes, data} against an expected one
    // {wlast, lanes, data bits}: data compared in written lanes only
    function automatic beat_ok(input [DW+8:0] b, input [24:0] e);
        begin
            beat_ok = (b[DW+8] == e[24]) && (b[DW+7:DW] == e[23:16]) &&
                      (((bits16(b[DW-1:0]) ^ e[15:0]) & lanes2(e[23:16])) == 16'd0);
        end
    endfunction

    // ---- reset: low in the first cycle, high afterwards ----------------------
    reg first = 1'b1;
    always @(posedge clk) first <= 1'b0;
    always @(*) begin
        if (first) assume (!rst_n && !s_en);
        else       assume (rst_n);
    end

    // =========================================================================
    // Writer model (assumptions)
    // =========================================================================
    reg        in_burst = 1'b0;         // between a burst's first and last word
    reg  [7:0] wcnt     = 8'd0;         // words of the current burst so far
    reg        seen     = 1'b0;         // s_room seen high since the last burst ended
    reg  [3:0] since    = 4'd0;         // cycles since that sample

    always @(*) if (!first && s_en) begin
        assume (s_first == !in_burst);
        assume ({24'd0, wcnt} < MAX_WORDS);
        if ({24'd0, wcnt} == MAX_WORDS - 1) assume (s_last);
        if (!in_burst) begin
            assume (s_addr[1:0] == 2'b00 && s_addr[31:AW] == {(32-AW){1'b0}});
            assume (seen && {28'd0, since} >= GAP);
        end
    end

    always @(posedge clk) if (!first) begin
        if (s_en) begin
            in_burst <= !s_last;
            wcnt     <= s_last ? 8'd0 : wcnt + 8'd1;
        end
        if (s_en && s_last) begin
            seen  <= 1'b0;
            since <= 4'd0;
        end else if (!seen && !in_burst && !s_en && s_room) begin
            seen  <= 1'b1;
            since <= 4'd1;
        end else if (seen && since != 4'hF) begin
            since <= since + 4'd1;
        end
    end

    // =========================================================================
    // Specification model: expected beats and AWs
    // =========================================================================
    reg  [AW-1:0] w_addr   = {AW{1'b0}};   // byte address of the burst's next word
    reg  [AW-1:0] b_start  = {AW{1'b0}};   // the burst's start address
    reg  [7:0]    cur_mask = 8'd0;         // lanes of the open beat written so far
    reg  [15:0]   cur_data = 16'd0;        // their data bits
    reg  [7:0]    ab_len   = 8'd0;         // beats started in the open AXI burst
    reg  [BA-1:0] ab_b     = {BA{1'b0}};   // its start beat address
    reg           ab_open  = 1'b0;
    reg  [BA-1:0] fin_b    = {BA{1'b0}};   // beat address of the last finished beat
    reg  [7:0]    bb       = 8'd0;         // beats finished in the current burst
    reg  [1:0]    abb      = 2'd0;         // AXI bursts closed in the current burst

    reg  [24:0]   sw [0:2*WQD-1];          // expected beats {wlast, lanes, data bits}
    reg  [SWI:0]  sw_wp = {(SWI+1){1'b0}}, sw_rp = {(SWI+1){1'b0}};
    reg  [BA+7:0] sa [0:2*AQD-1];          // expected AWs {beat address, AWLEN}
    reg  [SAI:0]  sa_wp = {(SAI+1){1'b0}}, sa_rp = {(SAI+1){1'b0}};

    wire [AW-1:0] a_cur      = s_first ? s_addr[AW-1:0] : w_addr;
    wire [2:0]    lane       = a_cur[4:2];
    wire          new_beat   = s_first || (lane == 3'd0);
    wire          new_ab     = s_first || (lane == 3'd0 && a_cur[11:5] == 7'd0);
    wire          beat_end   = s_last || (lane == 3'd7);
    wire          ab_end     = s_last || (&a_cur[11:2]);
    wire [7:0]    mask_now   = (new_beat ? 8'd0 : cur_mask) | (8'd1 << lane);
    wire [15:0]   data_now   = ((new_beat ? 16'd0 : cur_data) & ~(16'd3 << {lane, 1'b0})) |
                               ({14'd0, s_dbits} << {lane, 1'b0});
    wire [7:0]    ab_len_now = new_ab ? 8'd1 : (new_beat ? ab_len + 8'd1 : ab_len);
    wire [BA-1:0] ab_b_now   = new_ab ? a_cur[AW-1:5] : ab_b;

    wire w_hs  = m_wvalid && m_wready;
    wire aw_hs = m_awvalid && m_awready;

    // AWs issued minus WLASTs taken, from the scoreboard: every closed burst
    // queued one AW and one WLAST beat, so this is (WLAST beats still
    // expected) - (AWs not yet taken) + (the AW now presented).
    wire [SWI:0]  w_occ_e = sw_wp - sw_rp;
    reg  [7:0]    nlast;
    integer       j;
    always @(*) begin
        nlast = 8'd0;
        for (j = 0; j < 2*WQD; j = j + 1)
            if (j < w_occ_e)
                nlast = nlast + {7'd0, sw[(sw_rp[SWI-1:0] + j[SWI-1:0])][24]};
    end
    wire [SAI:0]  a_occ_e   = sa_wp - sa_rp;
    wire [7:0]    ahead_now = nlast + {7'd0, m_awvalid} - {{(7-SAI){1'b0}}, a_occ_e};

    always @(posedge clk) if (!first) begin
        if (s_en) begin
            w_addr   <= a_cur + 4;
            if (s_first) b_start <= s_addr[AW-1:0];
            cur_mask <= beat_end ? 8'd0  : mask_now;
            cur_data <= beat_end ? 16'd0 : data_now;
            ab_len   <= ab_len_now;
            ab_b     <= ab_b_now;
            ab_open  <= !ab_end;
            bb       <= (s_first ? 8'd0 : bb)  + {7'd0, beat_end};
            abb      <= (s_first ? 2'd0 : abb) + {1'b0, ab_end};
            if (beat_end) begin
                sw[sw_wp[SWI-1:0]] <= {ab_end, mask_now, data_now};
                sw_wp <= sw_wp + 1'b1;
                fin_b <= a_cur[AW-1:5];
            end
            if (ab_end) begin
                sa[sa_wp[SAI-1:0]] <= {ab_b_now, ab_len_now - 8'd1};
                sa_wp <= sa_wp + 1'b1;
            end
        end
        if (w_hs)  sw_rp <= sw_rp + 1'b1;
        if (aw_hs) sa_rp <= sa_rp + 1'b1;
    end

    // =========================================================================
    // AXI side
    // =========================================================================
    reg  [7:0] owb = 8'd0;              // W bursts finished (WLAST taken)
    reg  [7:0] oaw = 8'd0;              // AW handshakes
    always @(posedge clk) if (!first) begin
        if (w_hs && m_wlast) owb <= owb + 8'd1;
        if (aw_hs)           oaw <= oaw + 8'd1;
    end

    reg             aw_stall = 1'b0, w_stall = 1'b0;
    reg  [AW-1:0]   awaddr_q;
    reg  [7:0]      awlen_q;
    reg  [DW-1:0]   wdata_q;
    reg  [DW/8-1:0] wstrb_q;
    reg             wlast_q;
    always @(posedge clk) begin
        aw_stall <= !first && m_awvalid && !m_awready;
        w_stall  <= !first && m_wvalid && !m_wready;
        awaddr_q <= m_awaddr;  awlen_q <= m_awlen;
        wdata_q  <= m_wdata;   wstrb_q <= m_wstrb;  wlast_q <= m_wlast;
    end

    reg        strb_whole;
    reg  [7:0] w_lanes;
    integer    i;
    always @(*) begin
        strb_whole = 1'b1;
        for (i = 0; i < 8; i = i + 1) begin
            if (m_wstrb[i*4 +: 4] != 4'h0 && m_wstrb[i*4 +: 4] != 4'hF) strb_whole = 1'b0;
            w_lanes[i] = m_wstrb[i*4];
        end
    end

    wire [24:0]   sw_head  = sw[sw_rp[SWI-1:0]];
    wire [BA+7:0] sa_head  = sa[sa_rp[SAI-1:0]];
    wire [8:0]    aw_end   = {1'b0, m_awaddr[11:5]} + {1'b0, m_awlen};

    // =========================================================================
    // Properties
    // =========================================================================
    always @(posedge clk) if (!first && rst_n) begin
        // A1
        if (aw_stall) begin
            assert (m_awvalid);
            assert (m_awaddr == awaddr_q && m_awlen == awlen_q);
        end
        if (w_stall) begin
            assert (m_wvalid);
            assert (m_wdata == wdata_q && m_wstrb == wstrb_q && m_wlast == wlast_q);
        end
        // A2
        if (m_awvalid) assert (aw_end <= 9'd127);
        // A3
        assert ($signed(ahead_now) <= AW_AHEAD);
        // A4
        assert (m_bready);
        // D1
        if (w_hs) begin
            assert (sw_rp != sw_wp);
            assert (strb_whole);
            assert (beat_ok({m_wlast, w_lanes, m_wdata}, sw_head));
        end
        // D2
        if (aw_hs) begin
            assert (sa_rp != sa_wp);
            assert ({m_awaddr[AW-1:5], m_awlen} == sa_head && m_awaddr[4:0] == 5'd0);
        end
    end

    // =========================================================================
    // Invariants (k-induction strengthening; checked like everything else)
    // =========================================================================
    wire          pw    = dut.p_vld;
    wire          pwl   = dut.p_vld && (dut.p_end || dut.p_page);
    wire [SWI:0]  w_occ = sw_wp - sw_rp;            // expected beats not yet taken
    wire [SAI:0]  a_occ = sa_wp - sa_rp;            // expected AWs not yet taken
    wire [SWI:0]  w_pushed  = sw_wp - {{SWI{1'b0}}, pw};
    wire [SWI:0]  w_fetched = sw_rp + {{SWI{1'b0}}, dut.o_vld} + {{SWI{1'b0}}, dut.q_vld};
    wire [SAI:0]  a_pushed  = sa_wp - {{SAI{1'b0}}, pwl};
    wire [SAI:0]  a_loaded  = sa_rp + {{SAI{1'b0}}, m_awvalid};
    wire [AW-1:0] w_last_word = w_addr - 4;        // the last word's address
    wire [SWI:0]  w_inq     = w_pushed - w_fetched;      // beats in the FIFO memory
    wire [SWI:0]  w_notk    = w_pushed - sw_rp;          // pushed, not taken
    wire [SAI:0]  a_inq     = a_pushed - a_loaded;       // AWs in the queue memory
    wire [SWI:0]  w_last_i  = sw_wp - 1'b1;
    wire [SAI:0]  a_last_i  = sa_wp - 1'b1;
    wire [24:0]   sw_last   = sw[w_last_i[SWI-1:0]];
    wire [BA+7:0] sa_last   = sa[a_last_i[SAI-1:0]];
    // complete beats of the open AXI burst
    wire [7:0]    cb = ab_open ? ab_len - {7'd0, cur_mask != 8'd0} : 8'd0;
    wire [10:0]   lane_sum = {3'd0, b_start[4:2]} + {3'd0, wcnt};
    wire [7:0]    lane_lo  = (bb == 8'd0) ? (8'd1 << b_start[4:2]) - 8'd1 : 8'd0;

    // The port's own room thresholds (not this file's copies): a changed margin
    // is then judged on whether it is still safe, not on whether it moved.
    wire [31:0] wq_room = dut.WQ_ROOM_I;
    wire [31:0] aq_room = dut.AQ_ROOM_I;

    reg room_q = 1'b0;                  // s_room recomputed: the queue levels one cycle back
    always @(posedge clk)
        room_q <= !first && (dut.wq_cnt <= dut.WQ_ROOM) && (dut.aq_cnt <= dut.AQ_ROOM);

    // The reset cycle can only be the first one: the bookkeeping is at its
    // initial values then (k-induction may otherwise start from a "first"
    // cycle with stale bookkeeping and a DUT about to be reset).
    always @(posedge clk) if (first) begin
        assert (sw_wp == 0 && sw_rp == 0 && sa_wp == 0 && sa_rp == 0 && oaw == 8'd0 && owb == 8'd0);
        assert (!in_burst && !seen && since == 4'd0 && wcnt == 8'd0 && cur_mask == 8'd0 && !ab_open);
        assert (!room_q && !aw_stall && !w_stall);
    end
    integer k;
    always @(posedge clk) if (!first && rst_n) begin
        // writer / specification bookkeeping
        assert (in_burst || (cur_mask == 8'd0 && wcnt == 8'd0));
        assert (seen || since == 4'd0);
        assert (ab_open == (in_burst && w_addr[11:2] != 10'd0));
        if (in_burst) begin
            assert (wcnt != 8'd0 && {24'd0, wcnt} < MAX_WORDS);
            assert (w_addr == b_start + {wcnt, 2'b00});
            assert ({bb[7:0], w_addr[4:2]} == lane_sum);
            assert (cur_mask == ((8'd1 << w_addr[4:2]) - 8'd1) - lane_lo);
            assert (abb == {1'b0, w_addr[AW-1:12] != b_start[AW-1:12]});
            if (ab_open) begin
                if (cur_mask != 8'd0) assert (w_addr[AW-1:5] == ab_b + {{(BA-8){1'b0}}, ab_len} - 1'b1);
                else                  assert (w_addr[AW-1:5] == ab_b + {{(BA-8){1'b0}}, ab_len});
                assert (ab_len != 8'd0 && ab_len <= MAXB);
                if (abb == 2'd0) begin
                    assert (ab_b == b_start[AW-1:5]);
                    assert (ab_len == bb + {7'd0, cur_mask != 8'd0});
                end else begin
                    assert (ab_b == {w_addr[AW-1:12], 7'd0});
                    assert (ab_len == {1'b0, w_addr[11:5]} + {7'd0, cur_mask != 8'd0});
                end
                assert ({2'b0, ab_b[6:0]} + {1'b0, ab_len} <= 9'd128);
            end
        end

        // queue levels: the room guarantee
        assert (w_occ <= WQD);
        assert (a_occ <= AQD);
        if (s_room && !in_burst) begin
            assert (w_occ <= wq_room + 2);
            assert (a_occ <= aq_room + 3);
        end
        if (seen && !in_burst) begin
            assert (w_occ <= wq_room + 2);
            assert (a_occ <= aq_room + 3);
        end
        if (in_burst) begin
            assert (w_occ <= wq_room + 2 + bb);
            assert (a_occ <= aq_room + 3 + abb);
        end
        assert (s_room == room_q);

        // packing stage
        assert (dut.p_page == (&dut.p_baddr[6:0]));
        if (in_burst) begin
            assert (dut.lane  == w_addr[4:2]);
            assert (dut.baddr == w_addr[AW-1:5]);
        end
        if (pw) begin
            assert (cur_mask == 8'd0);
            assert (beat_ok({dut.p_end || dut.p_page, dut.msk, dut.acc}, sw_last));
            assert (dut.p_baddr == fin_b);
            assert (in_burst == !dut.p_end);
            assert (sw_last[24] == !ab_open);
            assert (fin_b == w_last_word[AW-1:5]);
            if (pwl) assert (sa_last[BA+7:8] + {{(BA-8){1'b0}}, sa_last[7:0]} == fin_b);
        end else begin
            assert (dut.msk == cur_mask);
            assert (((bits16(dut.acc) ^ cur_data) & lanes2(cur_mask)) == 16'd0);
        end

        // the open AXI burst as the port has pushed it
        if (!pw) begin
            assert (dut.ch_open == (cb != 8'd0));
            if (cb != 8'd0) assert (dut.ch_n == cb && dut.ch_start == ab_b);
        end else if (pwl) begin
            assert (dut.ch_open == (sa_last[7:0] != 8'd0));
            if (sa_last[7:0] != 8'd0) assert (dut.ch_n == sa_last[7:0] && dut.ch_start == sa_last[BA+7:8]);
        end else begin
            assert (cb != 8'd0);
            assert (dut.ch_open == (cb != 8'd1));
            if (cb != 8'd1) assert (dut.ch_n == cb - 8'd1 && dut.ch_start == ab_b);
        end

        // beat FIFO and its two output stages
        assert (dut.wq_wp  == w_pushed[WQ_LOG2:0]);
        assert (dut.wq_fp  == w_fetched[WQ_LOG2:0]);
        assert (dut.wq_cnt == w_pushed[WQ_LOG2:0] - sw_rp[WQ_LOG2:0]);
        assert (w_inq <= WQD);
        assert (w_notk <= WQD);
        if (dut.o_vld) assert (beat_ok(dut.o_reg, sw_head));
        if (dut.q_vld) assert (beat_ok(dut.ram_q, sw[(sw_rp[SWI-1:0] + {{(SWI-1){1'b0}}, dut.o_vld})]));
        for (k = 0; k < WQD; k = k + 1)
            if (k < w_inq)
                assert (beat_ok(dut.wmem[(dut.wq_fp[WQ_LOG2-1:0] + k[WQ_LOG2-1:0])],
                                sw[(w_fetched[SWI-1:0] + k[SWI-1:0])]));

        // AW queue and the AW register
        assert (dut.aq_wp == a_pushed[AQ_LOG2:0]);
        assert (dut.aq_rp == a_loaded[AQ_LOG2:0]);
        assert (a_inq <= AQD);
        if (m_awvalid) assert ({dut.aw_b, dut.aw_l} == sa_head);
        for (k = 0; k < AQD; k = k + 1)
            if (k < a_inq)
                assert (dut.aq[(dut.aq_rp[AQ_LOG2-1:0] + k[AQ_LOG2-1:0])] ==
                        sa[(a_loaded[SAI-1:0] + k[SAI-1:0])]);

        // every queued AW stays inside its 4 KB page
        for (k = 0; k < 2*AQD; k = k + 1)
            if (k < a_occ_e)
                assert ({2'b0, sa[(sa_rp[SAI-1:0] + k[SAI-1:0])][14:8]} +
                        {1'b0, sa[(sa_rp[SAI-1:0] + k[SAI-1:0])][7:0]} <= 9'd127);

        // AW_AHEAD bookkeeping
        assert (dut.ahead == ahead_now[5:0]);
        assert ($signed(ahead_now) >= -(AQD + 2));
        assert (oaw[SAI:0] == sa_rp);
        assert (owb[SAI:0] == sa_wp - nlast[SAI:0]);
    end

    // =========================================================================
    // Reachability
    // =========================================================================
    always @(posedge clk) if (!first && rst_n) begin
        cover (aw_hs && m_awaddr[11:5] == 7'd0 && sa_rp != 0 && oaw > 8'd1);   // a burst split at a page
        cover (!s_room && !in_burst && sw_wp != 0);
        cover (aw_stall && w_stall && owb > 8'd2);
        cover (dut.wq_cnt == wq_room + 2 + MAXB);                              // the fullest the queue gets
        cover ($signed(ahead_now) < 0);                                        // W ahead of its AW
    end
endmodule
