// ============================================================================
// Module      : udp_rd
// The clk_x1 half of the FPGA frame stream (docs/ARCHITECTURE.md § 8.1):
// it polls the job descriptor the firmware
// leaves in DDR3, reads the frame of the job out of a camera ring through its
// own read port of axi_mux, builds each GVSP packet into the packet FIFO towards
// udp_send (tx_mac_clk), and writes a status record back through its own write
// port. The VMSP UDP frame job (magic 'UDPD', commands 0x0224 / 0x0226) was
// removed on 30.10.2026 (owner: GVSP is the frame transport); such a descriptor
// is rejected with code 1 like any other unknown one.
//
//   * Doorbell by polling: one beat of the descriptor every POLL_CYCLES; a new
//     seq makes it read all three beats and take the job only if the seq at
//     the start and at the end agree and the magic is there (the CPU flushes
//     the descriptor a cache line at a time, and DDR3 holds rubbish at power-up).
//   * One AXI read in flight at a time (ERRORS 1.24: a deep queue in the DDR3
//     IP reaches the CPU's read latency and, through it, Ethernet receive).
//   * FIFO word = {ctrl, 4 bytes}, byte 0 in bits 7:0 = first on the wire. A
//     packet is [ctrl: udp_len][words]; the tx side starts a packet only once
//     pkt_tgl says all of it is in.
//   * CRC-32 is zlib's (reflected, 0xEDB88320), 32 bits per cycle over each
//     word as it enters the FIFO; the word itself comes from a register.
//   * Frame job (magic 'GVSD' kind 1, docs/protocol/GEV_PROTOCOL_DESIGN.md
//     § 5.7): one frame as a GigE Vision GVSP block in extended-ID mode -
//     leader, image payload packets of a fixed size cut at word offsets across
//     rows, trailer; image then chunks (frame id, timestamp, image CRC + bad
//     rows). A packet range of the block can be asked for (resend): where the
//     first data packet starts (row, word, global word) is the firmware's
//     division, the RTL only counts. Leader / trailer bodies come from a
//     template the firmware leaves in DDR3; the RTL patches only the header,
//     the timestamp and the chunk values. One FSM serves frames and profile
//     blocks, so the FIFO, udp_send and the AXI ports need no arbitration.
//   * Profile blocks (kind 2, GEV_PROTOCOL_DESIGN.md § 5.8): N = 2^nlog
//     consecutive dense-ring profiles as one multi-part block (0x400A),
//     sent autonomously: per camera the firmware leaves a stream record at
//     PCFG_ADDR + 0x80 * cam (network fields, parts, templates, start frame)
//     and the FSM, between descriptor polls, reads the final burst of the
//     block's last frame - "finished" by the same rule as laser_block_valid()
//     - and sends the block when it is in. A descriptor of kind 2 asks for a
//     packet range of one block (resend). Before the data every line's header
//     and final burst are checked; a line that fails goes out as zeros and is
//     flagged in its ProfileInfo, never as good data. Each part re-reads the
//     line's point (or mm) words and keeps its byte lanes (word-run engine
//     below); mm words leave with bit 15 of both halves flipped (owner's
//     decisions 2 and 6). No BSRAM: data comes from DDR3 per packet.
// ============================================================================

`timescale 1ns / 1ps

module udp_rd #(
    parameter [29:0]  DESC_ADDR    = 30'h0600_0400,
    parameter [29:0]  STAT_ADDR    = 30'h0600_0480,
    parameter [29:0]  PCFG_ADDR    = 30'h0600_1100,  // profile stream records, 0x80 per camera
    parameter integer POLL_CYCLES  = 5000,           // 50 us at 100 MHz
    parameter [29:0]  PIX_BASE_OFF = 30'h100,        // rows start after the 256-byte header burst
    parameter [29:0]  ROW_STRIDE   = 30'd1280,       // cam_channel row_slot_off, memory_map.h
    parameter [15:0]  HMAX         = 16'd1280
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          enable,         // fpga_streaming_active: leave DDR3 alone while the CPU boots

    // AXI read port of axi_mux
    output reg           m_arvalid,
    input  wire          m_arready,
    output reg  [29:0]   m_araddr,
    output reg  [7:0]    m_arlen,
    input  wire          m_rvalid,
    output wire          m_rready,
    input  wire [255:0]  m_rdata,
    input  wire [1:0]    m_rresp,
    input  wire          m_rlast,

    // AXI write port of axi_mux (the status record, one beat per job)
    output reg           m_awvalid,
    input  wire          m_awready,
    output wire [29:0]   m_awaddr,
    output wire [7:0]    m_awlen,
    output reg           m_wvalid,
    input  wire          m_wready,
    output reg  [255:0]  m_wdata,
    output wire [31:0]   m_wstrb,
    output wire          m_wlast,
    input  wire          m_bvalid,
    output wire          m_bready,
    input  wire [1:0]    m_bresp,

    // packet FIFO, write side
    output wire          f_wr,
    output wire [32:0]   f_wdata,
    input  wire          f_full,

    // to udp_send: a whole packet is in the FIFO / a job starts (fields held)
    output reg           pkt_tgl,
    output reg           job_tgl,
    output reg  [47:0]   j_dmac,
    output reg  [47:0]   j_smac,
    output reg  [31:0]   j_dip,
    output reg  [31:0]   j_sip,
    output reg  [15:0]   j_dport,
    output reg  [15:0]   j_sport,
    output reg  [31:0]   j_gap,
    // from udp_send, synchronised: one pulse per packet on the wire
    input  wire          sent_pulse,
    // tx-side counters, snapshotted there at every packet sent: still by the
    // time a job has drained (docs/ARCHITECTURE.md § 8.1, result word 1)
    input  wire [159:0]  tx_snap,

    // counters (cumulative)
    output reg  [31:0]   n_jobs,
    output reg  [31:0]   n_pkts,
    output reg  [15:0]   n_axi_err,
    output wire          busy
);

    localparam [31:0] GVSD_MAGIC = 32'h4453_5647;   // 'GVSD': a GVSP job in the same descriptor
    localparam [31:0] STAT_MAGIC = 32'h5350_4455;   // 'UDPS'
    localparam [31:0] VID0_MAGIC = 32'h5649_4430;   // 'VID0'
    localparam [31:0] PCFG_MAGIC = 32'h4353_5647;   // 'GVSC': a profile stream record
    localparam [31:0] PST_MAGIC  = 32'h5350_5647;   // 'GVPS': its status beat (+0x60)
    localparam [31:0] PROF_MAGIC = 32'h5052_4F46;   // laser.h LASER_BLOCK_MAGIC
    localparam [31:0] FIN_MAGIC  = 32'h1234_5678;   // last word of the final burst
    localparam [31:0] CID_PAY    = 32'h0000_1000;   // tag of the payload chunk (the image / the parts)
    localparam [31:0] CID_PINFO  = 32'h0000_1004;   // ChunkPortProfileInfo (gev_features.py): 96 B per line
    // dense-ring block (laser.h): the block at slot + 0x100, then these offsets from it
    localparam [29:0] BLK_OFF = 30'h100, FIN_OFF = 30'h5100, PTS_OFF = 30'h5200, MM_OFF = 30'h6600;
    // word-run transforms: part kinds 0..3 as they are in the record, 4 = raw words
    localparam [2:0]  XF_C16 = 3'd0, XF_AC16 = 3'd1, XF_M8 = 3'd2, XF_C8 = 3'd3, XF_RAW = 3'd4;
    localparam [2:0]  PK_CHUNK = 3'd4;              // part kind of the chunk part
    localparam [8:0]  ROW_WORDS  = 9'd270;          // 1080 / 4
    localparam [5:0]  ROW_BEATS  = 6'd34;           // 1088 bytes: pixels + 8-byte trailer

    // zlib CRC-32 over the four bytes of w, byte 0 (bits 7:0) first
    function [31:0] crc32_w(input [31:0] c, input [31:0] w);
        integer i;
        reg [31:0] x;
        begin
            x = c ^ w;
            for (i = 0; i < 32; i = i + 1)
                x = x[0] ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
            crc32_w = x;
        end
    endfunction

    // a big-endian 32-bit field as a FIFO word: its most significant byte goes first
    function [31:0] be4(input [31:0] v);
        be4 = {v[7:0], v[15:8], v[23:16], v[31:24]};
    endfunction

    // ------------------------------------------------------------------ state
    localparam [6:0] S_IDLE = 7'd0, S_POLL = 7'd1, S_POLL_R = 7'd66, S_DESC = 7'd3, S_DESC_R = 7'd68,
                     S_CHECK = 7'd5, S_FHDR = 7'd7, S_FHDR_R = 7'd72,
                     S_ROW = 7'd12, S_ROW_AR = 7'd13, S_ROW_R = 7'd78, S_ROW_END = 7'd15,
                     S_DRAIN = 7'd20, S_STAT = 7'd21, S_STAT_B = 7'd22, S_CHECK0 = 7'd23,
                     // GVSP frame job
                     S_GTPL = 7'd25, S_GTPL_R = 7'd90, S_GCHK = 7'd27, S_GCHK2 = 7'd28, S_GPKT = 7'd29,
                     S_GCTRL = 7'd30, S_GHDR = 7'd31, S_GBODY = 7'd32, S_GDATA = 7'd33, S_GPEND = 7'd34,
                     S_GFLUSH = 7'd99,
                     // profile stream: scheduler, job set-up, line checks, seek, data, word runs
                     S_AUTO = 7'd36, S_ACFG_R = 7'd101, S_ARDY = 7'd38, S_ARDY2 = 7'd39, S_ARDY_R = 7'd104,
                     S_ANEXT = 7'd41, S_PCFG = 7'd42, S_PCFG_R = 7'd107, S_PCHK0 = 7'd44, S_PCHK = 7'd45,
                     S_PPRE = 7'd46, S_PPRE1 = 7'd47, S_PPRE_H = 7'd112, S_PPRE_F = 7'd113, S_PPRE_C = 7'd50,
                     S_SK0 = 7'd51, S_SK1 = 7'd52, S_SK2 = 7'd53, S_SK3 = 7'd54, S_SK4 = 7'd55,
                     S_PSEL = 7'd56, S_GPKT2 = 7'd57, S_PDATA = 7'd58, S_PLB = 7'd59, S_PLB2 = 7'd60,
                     S_PRUN = 7'd61, S_RUNA = 7'd62, S_RUNR = 7'd127, S_RUNZ = 7'd63, S_PTPL = 7'd2,
                     S_PSTAT = 7'd4;
    // Fanout (RTL_RULES 2.2): the binary st drove the 256-bit d0..d2 / rb / ws loads through a
    // shared decode, st[4] fanout 725, st[5] 635 (B4). The R-beat flag the encoding used to carry in
    // st[6] is st_rd, a flop of its own written with every st, so nothing reads a bit of st directly
    // and synthesis is free to re-encode it. syn_encoding = "onehot" was tried on 30.10.2026 and
    // dropped: udp_rd grew 7389 -> 9275 LUT and that build never finished routing.
    reg [6:0] st;
    reg         st_rd;              // = st[6] of the binary encoding: the state takes R beats
    function rd_st(input [6:0] s);
        rd_st = s[6];
    endfunction
    // st_dl: the state loads R beats into d0..d2 (S_DESC_R, S_PCFG_R), a flop written with every
    // st like st_rd. The d0..d2 load enable was a decode of st feeding one LUT with fanout 736
    // (8823136F, st_0 -> d2_*, 5.4 ns net); now it is st_dl, rb_v and rbeat only.
    reg         st_dl;
    function dl_st(input [6:0] s);
        dl_st = (s == S_DESC_R) || (s == S_PCFG_R);
    endfunction
    reg [15:0]  poll_cnt;
    reg [31:0]  last_seq;
    reg [255:0] d0, d1, d2;          // descriptor beats
    reg [1:0]   rbeat;               // beat number within a short (descriptor / frame header) read

    // job
    reg [15:0]  j_id;
    reg [31:0]  j_seq;
    reg [29:0]  frame_base;

    // frame header
    reg [31:0]  f_magic, f_seq, f_strobe;
    reg [15:0]  f_height;
    reg [15:0]  row, row_end;
    reg [29:0]  row_addr;

    // word source -> FIFO
    reg         ws_v, ws_ctrl, ws_crc;
    reg [31:0]  ws;
    assign f_wr    = ws_v && !f_full;
    assign f_wdata = {ws_ctrl, ws};
    wire        take   = f_wr;
    wire        ws_free = !ws_v || take;
    reg [31:0]  crc;
    reg [2:0]   hw_i;                // header word index
    // S_GHDR's exit, registered in S_GCTRL / S_GHDR: hw_last = (hw_i == the header's last word),
    // gh_nst = the state after the header. The exit was wr_full_q -> hw_i compare -> 5 LUTs ->
    // st_rd (8823136F, +0.017).
    reg         hw_last;
    reg [6:0]   gh_nst;
    reg [2:0]   gh_last;

    // R beat buffer and its words. Every beat lands in rb first; the FSM works
    // only from rb. RVALID/RDATA come out of axi_mux's slice through its mux,
    // and the first builds that compared RDATA and loaded wide registers from
    // it directly failed clk_x1 on exactly those paths (13.10.2026, the rule of
    // invariant 7: flop -> <= 2 LUTs -> flop across a module boundary). So rb
    // loads on RREADY alone - a function of this FSM's flops - and RVALID
    // reaches only the one-bit rb_v and rb_fresh.
    reg         rb_v, rb_last, rb_err, rb_fresh;
    reg [255:0] rb;
    reg [2:0]   rw_i;                // word within the beat
    reg [8:0]   rw_n;                // word within the row: 0..271
    reg         tag0_ok, tag1_ok;    // the row trailer's words as compared when taken (clk_x1, ERRORS 6.19)
    reg [5:0]   beats_left;          // beats of the row still to request
    reg         ar_busy;             // a row burst requested and its last beat not yet in rb
    // RREADY only where a beat is consumed: a beat accepted in any other state
    // would leave axi_mux's FIFO and be lost to the FSM.
    // The states that take R beats are exactly those with bit 6 set in the codes above; st_rd holds
    // that bit, so RREADY is one flop and rb_v - the 12-state decode it used to be was the clk_x1
    // path into axi_mux.
    assign m_rready = st_rd && !rb_v;
    // descriptor check, registered (S_CHECK0) before it enables the job's fields
    reg         chk_new, chk_ok;

    // status write: two beats - this job, then cumulative counters; an
    // autonomous profile block writes one beat to its camera's record + 0x60
    reg         wbeat;
    reg         j_auto;              // the job is an autonomous profile block (no descriptor)
    reg         j_cam0;              // ... of camera j_cam0
    assign m_awaddr = j_auto ? PCFG_ADDR + {22'd0, j_cam0, 7'h60} : STAT_ADDR;
    assign m_awlen  = j_auto ? 8'd0 : 8'd1;
    assign m_wstrb  = 32'hFFFF_FFFF;
    assign m_wlast  = wbeat;
    assign m_bready = 1'b1;

    reg [31:0]  pkts_pushed, pkts_sent;
    reg [15:0]  frames_done;
    reg [31:0]  rows_bad;
    reg [31:0]  code;
    assign busy = (st != S_IDLE) && (st != S_POLL) && (st != S_POLL_R);

    // the descriptor's fields
    wire [31:0] dz_seq    = d0[31:0];
    wire [7:0]  dz_cam    = d0[39:32];
    wire [47:0] dz_dmac   = {d0[71:64], d0[79:72], d0[87:80], d0[95:88], d0[103:96], d0[111:104]};
    wire [15:0] dz_dport  = d0[127:112];
    wire [47:0] dz_smac   = {d0[135:128], d0[143:136], d0[151:144], d0[159:152], d0[167:160], d0[175:168]};
    wire [15:0] dz_sport  = d0[191:176];
    wire [31:0] dz_dip    = {d0[199:192], d0[207:200], d0[215:208], d0[223:216]};
    wire [31:0] dz_sip    = {d0[231:224], d0[239:232], d0[247:240], d0[255:248]};
    wire [31:0] dz_gap    = d1[31:0];
    wire [15:0] dz_id     = d1[47:32];
    wire [29:0] dz_ring   = d1[93:64];       // stream record: the camera's dense ring
    wire [31:0] dz_magic  = d1[159:128];
    wire [31:0] dz_seq2   = d2[255:224];

    // the GVSP job's fields (magic GVSD; d0 as above but byte 5 = kind, byte 7 bit 0 = resend)
    wire [7:0]  dz_kind   = d0[47:40];
    wire [15:0] dz_lw     = d1[63:48];       // data words of a full payload packet: (SCPS - 48) / 4
    wire [29:0] dz_fbase  = d1[93:64];       // the frame's ring slot
    wire [31:0] dz_fseq   = d1[127:96];      // VID0 seq the firmware saw in it
    wire [15:0] dz_pid0   = d1[175:160];     // packets to send: dz_pid0 .. dz_pid1
    wire [15:0] dz_pid1   = d1[191:176];
    wire [31:0] dz_w0     = d1[223:192];     // global data word of the first data packet sent
    wire [31:0] dz_t      = d1[255:224];     // data words of the block: 270 * H + 12 chunk words
    wire [63:0] dz_bid    = d2[63:0];
    wire [15:0] dz_row0   = d2[79:64];       // dz_w0 as (row, word in row); row = H inside the chunks
    wire [15:0] dz_word0  = d2[95:80];
    wire [29:0] dz_tpl    = d2[125:96];      // leader body (9 words) and trailer body (4 words)
    wire [31:0] dz_crcovr = d2[159:128];     // chunk values when dz_w0 != 0 (a resend after the image start)
    wire [31:0] dz_rbovr  = d2[191:160];
    wire [15:0] dz_ndata  = d2[207:192];     // data packets in the block; the trailer is dz_ndata + 1
    wire        dz_gok    = (dz_magic == GVSD_MAGIC) && (dz_kind == 8'd1) && (dz_cam <= 8'd1)
                            && (dz_lw >= 16'd132) && (dz_lw <= 16'd2164) && (dz_ndata != 16'd0)
                            && (dz_pid0 <= dz_pid1) && (dz_pid1 <= dz_ndata + 16'd1)
                            && (dz_word0 < 16'd270) && (dz_t != 32'd0) && (dz_row0 <= HMAX);

    reg         j_resend;
    reg         g_nock;              // frame without chunks (payload type 0x0001, ERRORS 3.23): d0 byte 7 bit 1
    reg  [15:0] g_lw, g_pid, g_pid1, g_ndata, g_row0, g_words, pkt_left;
    reg  [31:0] g_fseq, g_w0, g_t, g_wcur, g_crcovr, g_rbovr;
    reg  [63:0] g_bid;
    reg  [8:0]  g_skip;              // words of the current row not sent (the first row of a resend)
    reg  [29:0] g_tpl;
    reg  [255:0] t0, t1;             // template beats
    reg  [18:0] g_t270;              // 270 * H
    reg  [3:0]  g_ck, g_bw;          // chunk word, body word
    reg  [1:0]  g_kind;              // 0 leader, 1 payload, 2 trailer
    reg         in_row;              // a row is being read (a packet may end in the middle of it)
    reg         g_bad1, g_w0z;       // the slot no longer the frame's; the range starts at word 0
    reg  [31:0] g_tm;                // data words of the block less the 14 chunk words
    wire [31:0] g_crc_v  = g_w0z ? ~crc : g_crcovr;
    wire [31:0] g_rbad_v = g_w0z ? rows_bad : g_rbovr;
    wire [31:0] g_left   = g_t - g_wcur;
    // template word i: the body bytes in wire order, as the firmware wrote them
    // Body word i, taken one word ahead into g_tw (g_bw1 = g_bw + 1): the template mux feeds its
    // own register, not ws (clk_x1, B4 29.10: g_bw -> ws was a critical path)
    reg  [3:0]  g_bw1;
    reg  [31:0] g_tw;
    function [31:0] body_w(input [3:0] i, input lead);
        reg [2:0] k;
        begin
            k = i[2:0] + 3'd1;
            if (lead) body_w = (i == 4'd1) ? 32'd0 : (i == 4'd2) ? be4(f_strobe)        // timestamp
                             : (i < 4'd8) ? t0[32 * i[2:0] +: 32] : t1[31:0];
            else      body_w = t1[32 * k +: 32];
        end
    endfunction

    // ---- profile blocks (kind 2) ----------------------------------------------------
    // A descriptor of kind 2 = resend of a packet range of block dz_bid of camera dz_cam
    // (d0 byte 7 bit 0 = resend flag, as kind 1); the rest comes from the camera's record.
    wire        dz_pok    = (dz_magic == GVSD_MAGIC) && (dz_kind == 8'd2) && (dz_cam <= 8'd1)
                            && (dz_pid0 <= dz_pid1);
    // The stream record, read into d0..d2 (ARCHITECTURE § 8.1): d0 as a descriptor (seq, cam,
    // kind 2, byte 6 = log2 N, byte 7 bit 0 = streaming on, the network fields), d1 = gap,
    // parts, lw, dense ring, start frame, magic 'GVSC', template, leader / trailer words,
    // payload chunk length; d2 = four part entries {offset 24, packets 16, kind 3}, the data
    // packet count and seq again.
    wire [2:0]  pc_nlog   = d0[50:48];
    wire        pc_on     = d0[56];
    wire [2:0]  pc_nparts = d1[34:32];
    wire [31:0] pc_start  = d1[127:96];
    wire [29:0] pc_tpl    = d1[189:160];
    wire [7:0]  pc_lb     = d1[199:192];
    wire [7:0]  pc_tb     = d1[207:200];
    wire [23:0] pc_lenp   = d1[231:208];
    wire [15:0] pc_ndata  = d2[191:176];
    wire        pc_ok     = (dz_magic == PCFG_MAGIC) && (dz_seq == dz_seq2) && (dz_kind == 8'd2)
                            && (pc_nlog <= 3'd4) && (pc_nparts != 3'd0) && (pc_nparts <= 3'd4)
                            && (dz_lw >= 16'd130) && (dz_lw <= 16'd2162) && (pc_ndata != 16'd0)
                            && (pc_lb != 8'd0) && (pc_tb != 8'd0);

    reg         j_prof;              // the job is a profile block
    reg         chk_rng;             // resend range past the trailer (registered in S_PCHK0)
    reg         ph_ok, pf_ok, pfin_ok;   // line check: header magic and frame; final-burst frame, end word
    reg         chk_pv, p_cfg_only;  // descriptor of kind 2; the record read only to learn the stream
    // scheduler, per camera: streaming on, record seq seen, next frame to send, N, ring
    reg  [1:0]  a_on;
    reg  [31:0] a_seq0, a_seq1, a_next0, a_next1;
    reg  [2:0]  a_nlog0, a_nlog1;
    reg  [29:0] a_ring0, a_ring1;
    reg         ac;                  // the camera the scheduler looks at
    reg  [31:0] a_flast;             // its block's last frame
    reg  [29:0] a_addr;
    reg  [31:0] a_f0;
    reg  [31:0] pc_blocks0, pc_blocks1, pc_bad0, pc_bad1;
    reg  [15:0] pc_skip0, pc_skip1, pc_err0, pc_err1;
    wire [31:0] a_next  = ac ? a_next1 : a_next0;
    wire [2:0]  a_nlog  = ac ? a_nlog1 : a_nlog0;
    wire [29:0] a_ring  = ac ? a_ring1 : a_ring0;
    wire [31:0] a_seq   = ac ? a_seq1 : a_seq0;
    wire [31:0] a_df    = a_f0 - a_flast;      // the frame in the slot against the one awaited
    reg         a_eq, a_ahead;       // a_df == 0; a_df in 1 .. 2^20 - 1 (registered at beat 1)
    reg  [31:0] a_seqc;              // the visited camera's record seq (registered in S_AUTO)
    // job
    reg  [2:0]  p_nlog, p_nparts;
    reg  [4:0]  p_n;                 // lines = 2^p_nlog
    reg  [31:0] p_first;             // frame of line 0
    reg  [29:0] p_ring, p_tpl;
    reg  [7:0]  p_lb, p_tb;
    reg  [23:0] p_lenp;
    reg  [171:0] p_parts;
    reg  [15:0] p_good;              // lines that passed the check before the data
    reg  [15:0] p_bad;               // lines of this block not delivered / changed while sent
    reg  [4:0]  p_l;                 // line
    reg  [31:0] p_fl;                // its frame
    reg  [29:0] p_bb;                // its block base
    reg  [2:0]  pb;                  // beat of the final burst
    // packets: part k, words of it done, where the next word comes from
    reg  [1:0]  p_k;
    reg  [2:0]  cur_kind;
    reg  [15:0] cur_np;
    reg  [23:0] cur_off;
    reg  [15:0] cur_tot, p_pw, p_left, p_rem;
    reg  [23:0] p_aoff;
    reg         p_eop;
    reg  [1:0]  p_seg;               // chunk part: 0 payload tag, 1 lines, 2 own tag
    reg         p_ci;                // word of a tag
    reg  [10:0] p_iw0;               // first input word of the next run (a resend inside a line)
    reg  [15:0] p_x;                 // seek: words still to skip
    reg  [12:0] p_slot;
    wire [42:0] part_k   = p_parts[43 * p_k +: 43];
    // part sizes: output words per line, input words per output word
    wire [10:0] wpl_k    = (cur_kind == XF_C16) ? 11'd540 : (cur_kind == XF_AC16) ? 11'd1080 :
                           (cur_kind == PK_CHUNK) ? 11'd24 : 11'd270;
    wire [2:0]  ipo_k    = (cur_kind == XF_C16) ? 3'd2 : (cur_kind == XF_M8 || cur_kind == XF_C8) ? 3'd4 : 3'd1;
    // the word-run engine: r_n input words from r_base (beat aligned), from word r_iw on,
    // through transform r_xf into output words; r_zero sends the words without reading
    reg         r_act, r_zero;
    reg  [2:0]  r_xf;
    reg  [1:0]  r_patch;             // 1 leader timestamp, 2 ProfileInfo status / frame
    reg  [29:0] r_base, r_addr;
    reg  [10:0] r_iw, r_n;
    reg  [7:0]  r_bl;                // beats still to request
    reg  [1:0]  r_grp;               // input words in the output word being packed
    reg  [23:0] r_acc;
    reg  [31:0] r_fexp;              // frame the run's line must hold (ProfileInfo)
    reg         r_tpl;               // the run is a leader / trailer body
    reg         r_first;             // its first burst address not computed yet
    reg         p_seeking;           // S_PSEL returns into the seek
    wire [31:0] r_w      = rb[32 * rw_i +: 32];
    wire [2:0]  r_ipo    = (r_xf == XF_C16) ? 3'd2 : (r_xf == XF_M8 || r_xf == XF_C8) ? 3'd4 : 3'd1;
    // r_xf is C16; Mono8 / Confidence8 (registered with r_xf). Kept as flops of their own: synthesis
    // merged them back into the r_xf decode (place 3 on 1c7c562: r_xf -> st +0.029 ns).
    (* syn_preserve = 1 *) reg r_c16, r_b8;
    wire        r_emit   = r_c16 ? (r_grp == 2'd1) : r_b8 ? (r_grp == 2'd3) : 1'b1;
    wire        r_lastw  = (r_iw + 11'd1 == r_n);
    wire [7:0]  r_lane   = (r_xf == XF_M8) ? r_w[23:16] : r_w[31:24];
    // ProfileInfo word 0: 1 = the line passed both checks, 4 = it changed while being sent
    wire [31:0] r_pstat  = (rb[31:0] == PROF_MAGIC && rb[63:32] == r_fexp) ? 32'd1 : 32'd4;
    reg  [31:0] r_out;
    always @(*) begin
        case (r_xf)
        XF_C16:  r_out = {r_w[15:0], r_acc[15:0]};
        XF_AC16: r_out = r_w ^ 32'h8000_8000;                 // A and C unsigned (decisions 2, 6)
        XF_M8, XF_C8: r_out = {r_lane, r_acc[23:0]};
        default: r_out = (r_patch == 2'd1 && r_iw == 11'd1) ? 32'd0 :
                         (r_patch == 2'd1 && r_iw == 11'd2) ? be4(f_strobe) :
                         (r_patch == 2'd2 && r_iw == 11'd0) ? r_pstat : r_w;
        endcase
    end
    // a zero run: line not delivered - ProfileInfo word 0 = 2, word 1 = the frame awaited
    wire [31:0] r_zout   = (r_patch == 2'd2 && r_iw == 11'd0) ? 32'd2 :
                           (r_patch == 2'd2 && r_iw == 11'd1) ? r_fexp : 32'd0;
    // beats to the end of the 4 KB page from r_addr, and the next burst (<= 32 beats)
    wire [7:0]  r_to_page = 8'd128 - {1'b0, r_addr[11:5]};
    wire [7:0]  r_m1      = (r_to_page < r_bl) ? r_to_page : r_bl;
    wire [7:0]  r_len     = (r_m1 < 8'd32) ? r_m1 : 8'd32;
    // beats to the end of the 4 KB page from row_addr (row_addr is beat-aligned)
    wire [7:0]  to_page    = 8'd128 - {1'b0, row_addr[11:5]};

    // d0..d2: the beat in rb while the FSM takes it in S_DESC_R / S_PCFG_R (it clears rb_v there).
    wire d_we = st_dl && rb_v;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d0 <= 256'd0; d1 <= 256'd0; d2 <= 256'd0;
        end else if (d_we) begin
            case (rbeat)
                2'd0: d0 <= rb;
                2'd1: d1 <= rb;
                default: d2 <= rb;
            endcase
        end
    end

    // Seek step (S_SK1), registered: p_k's enable was p_k -> part_k mux -> 16-bit compare ->
    // its own CE, 10 levels (8823136F, +0.007). S_SK1 now takes two clocks per part: sk_ph 0
    // lets sk_more / sk_np settle from the current p_k and p_rem, sk_ph 1 acts on them.
    reg         sk_ph, sk_more;
    reg  [15:0] sk_np;
    always @(posedge clk) begin
        sk_np   <= part_k[18:3];
        sk_more <= ({1'b0, p_k} + 3'd1 < p_nparts) && (p_rem >= part_k[18:3]);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            begin st <= S_IDLE; st_rd <= rd_st(S_IDLE); st_dl <= dl_st(S_IDLE); end poll_cnt <= 16'd0; last_seq <= 32'd0;
            rbeat <= 2'd0;
            m_arvalid <= 1'b0; m_araddr <= 30'd0; m_arlen <= 8'd0;
            m_awvalid <= 1'b0; m_wvalid <= 1'b0; m_wdata <= 256'd0; wbeat <= 1'b0;
            pkt_tgl <= 1'b0; job_tgl <= 1'b0;
            j_dmac <= 48'd0; j_smac <= 48'd0; j_dip <= 32'd0; j_sip <= 32'd0; j_dport <= 16'd0; j_sport <= 16'd0;
            j_gap <= 32'd0; j_id <= 16'd0; j_seq <= 32'd0; frame_base <= 30'd0;
            f_magic <= 32'd0; f_seq <= 32'd0; f_strobe <= 32'd0; f_height <= 16'd0;
            row <= 16'd0; row_end <= 16'd0; row_addr <= 30'd0;
            ws_v <= 1'b0; ws_ctrl <= 1'b0; ws_crc <= 1'b0; ws <= 32'd0; crc <= 32'hFFFF_FFFF; hw_i <= 3'd0; hw_last <= 1'b0; gh_nst <= 7'd0; gh_last <= 3'd0;
            rb_v <= 1'b0; rb <= 256'd0; rw_i <= 3'd0; rw_n <= 9'd0; tag0_ok <= 1'b0; tag1_ok <= 1'b0;
            rb_last <= 1'b0; rb_err <= 1'b0; rb_fresh <= 1'b0; chk_new <= 1'b0; chk_ok <= 1'b0;
            beats_left <= 6'd0; ar_busy <= 1'b0;
            pkts_pushed <= 32'd0; pkts_sent <= 32'd0; frames_done <= 16'd0;
            rows_bad <= 32'd0; code <= 32'd0;
            n_jobs <= 32'd0; n_pkts <= 32'd0; n_axi_err <= 16'd0;
            j_resend <= 1'b0; g_nock <= 1'b0;
            g_lw <= 16'd0; g_pid <= 16'd0; g_pid1 <= 16'd0; g_ndata <= 16'd0; g_row0 <= 16'd0;
            g_words <= 16'd0; pkt_left <= 16'd0; g_fseq <= 32'd0; g_w0 <= 32'd0; g_t <= 32'd0;
            g_wcur <= 32'd0; g_crcovr <= 32'd0; g_rbovr <= 32'd0; g_bid <= 64'd0; g_skip <= 9'd0;
            g_tpl <= 30'd0; t0 <= 256'd0; t1 <= 256'd0; g_t270 <= 19'd0; g_ck <= 4'd0; g_bw <= 4'd0; g_bw1 <= 4'd1; g_tw <= 32'd0;
            g_kind <= 2'd0; in_row <= 1'b0;
            j_auto <= 1'b0; j_cam0 <= 1'b0; j_prof <= 1'b0; chk_pv <= 1'b0; p_cfg_only <= 1'b0;
            a_on <= 2'b00; a_seq0 <= 32'd0; a_seq1 <= 32'd0; a_next0 <= 32'd0; a_next1 <= 32'd0;
            a_nlog0 <= 3'd0; a_nlog1 <= 3'd0; a_ring0 <= 30'd0; a_ring1 <= 30'd0; ac <= 1'b0;
            a_flast <= 32'd0; a_addr <= 30'd0; a_f0 <= 32'd0;
            pc_blocks0 <= 32'd0; pc_blocks1 <= 32'd0; pc_bad0 <= 32'd0; pc_bad1 <= 32'd0;
            pc_skip0 <= 16'd0; pc_skip1 <= 16'd0;
            pc_err0 <= 16'd0; pc_err1 <= 16'd0;
            p_nlog <= 3'd0; p_nparts <= 3'd0; p_n <= 5'd0; p_first <= 32'd0; p_ring <= 30'd0; p_tpl <= 30'd0;
            p_lb <= 8'd0; p_tb <= 8'd0; p_lenp <= 24'd0; p_parts <= 172'd0; p_good <= 16'd0; p_bad <= 16'd0;
            p_l <= 5'd0; p_fl <= 32'd0; p_bb <= 30'd0; ph_ok <= 1'b0; pf_ok <= 1'b0; pfin_ok <= 1'b0; pb <= 3'd0;
            p_k <= 2'd0; cur_kind <= 3'd0; cur_np <= 16'd0; cur_off <= 24'd0; cur_tot <= 16'd0;
            p_pw <= 16'd0; p_left <= 16'd0; p_rem <= 16'd0; p_aoff <= 24'd0; p_eop <= 1'b0;
            p_seg <= 2'd0; p_ci <= 1'b0; p_iw0 <= 11'd0; p_x <= 16'd0; p_slot <= 13'd0;
            r_act <= 1'b0; r_zero <= 1'b0; r_xf <= 3'd0; r_patch <= 2'd0; r_base <= 30'd0; r_addr <= 30'd0;
            r_iw <= 11'd0; r_n <= 11'd0; r_bl <= 8'd0; r_grp <= 2'd0; r_acc <= 24'd0; r_fexp <= 32'd0;
            r_tpl <= 1'b0; r_first <= 1'b0; p_seeking <= 1'b0; sk_ph <= 1'b0;
            chk_rng <= 1'b0; g_bad1 <= 1'b0; g_w0z <= 1'b0; g_tm <= 32'd0;
            a_eq <= 1'b0; a_ahead <= 1'b0; a_seqc <= 32'd0; r_c16 <= 1'b0; r_b8 <= 1'b0;
        end else begin
            // ---- FIFO side: a taken word leaves the source, and the CRC follows it
            if (take) begin
                ws_v <= 1'b0;
                if (ws_crc) crc <= crc32_w(crc, ws);
            end
            if (sent_pulse) pkts_sent <= pkts_sent + 32'd1;
            // ---- R beat into rb on RREADY alone; the FSM below clears rb_v as it
            // consumes the beat (RREADY is low then, so the two never meet)
            rb_fresh <= m_rready && m_rvalid;
            if (m_rready) begin
                rb <= m_rdata; rb_v <= m_rvalid; rb_last <= m_rlast; rb_err <= (m_rresp != 2'b00);
            end
            if (rb_fresh && rb_err && n_axi_err != 16'hFFFF) n_axi_err <= n_axi_err + 16'd1;

            case (st)
            // ---------------------------------------------------------- doorbell
            S_IDLE: begin
                if (enable) begin
                    if (poll_cnt == POLL_CYCLES[15:0]) begin
                        poll_cnt <= 16'd0;
                        m_arvalid <= 1'b1; m_araddr <= DESC_ADDR; m_arlen <= 8'd0;
                        begin st <= S_POLL; st_rd <= rd_st(S_POLL); st_dl <= dl_st(S_POLL); end
                    end else begin
                        poll_cnt <= poll_cnt + 16'd1;
                    end
                end
            end
            S_POLL: if (m_arready) begin m_arvalid <= 1'b0; begin st <= S_POLL_R; st_rd <= rd_st(S_POLL_R); st_dl <= dl_st(S_POLL_R); end end
            S_POLL_R: if (rb_v) begin
                rb_v <= 1'b0;
                if (rb[31:0] != last_seq) begin
                    m_arvalid <= 1'b1; m_araddr <= DESC_ADDR; m_arlen <= 8'd2; rbeat <= 2'd0;
                    begin st <= S_DESC; st_rd <= rd_st(S_DESC); st_dl <= dl_st(S_DESC); end
                end else begin
                    begin st <= S_AUTO; st_rd <= rd_st(S_AUTO); st_dl <= dl_st(S_AUTO); end                       // no job: is a profile block due?
                end
            end
            S_DESC: if (m_arready) begin m_arvalid <= 1'b0; begin st <= S_DESC_R; st_rd <= rd_st(S_DESC_R); st_dl <= dl_st(S_DESC_R); end end
            S_DESC_R: if (rb_v) begin
                rb_v <= 1'b0;
                rbeat <= rbeat + 2'd1;
                if (rb_last) begin st <= S_CHECK0; st_rd <= rd_st(S_CHECK0); st_dl <= dl_st(S_CHECK0); end
            end
            S_CHECK0: begin
                // the checks in a cycle of their own: two 32-bit compares and the
                // field ranges would otherwise enable ~300 bits of job fields
                chk_new <= (dz_seq == dz_seq2) && (dz_seq != last_seq);
                chk_ok  <= dz_gok || dz_pok;
                chk_pv  <= dz_pok;
                begin st <= S_CHECK; st_rd <= rd_st(S_CHECK); st_dl <= dl_st(S_CHECK); end
            end
            S_CHECK: begin
                if (!chk_new) begin
                    begin st <= S_IDLE; st_rd <= rd_st(S_IDLE); st_dl <= dl_st(S_IDLE); end                       // torn or stale: look again next poll
                end else begin
                    last_seq <= dz_seq; j_seq <= dz_seq; j_id <= dz_id;
                    frames_done <= 16'd0; rows_bad <= 32'd0;
                    pkts_pushed <= 32'd0; pkts_sent <= 32'd0;
                    j_auto <= 1'b0; j_prof <= 1'b0;
                    if (chk_pv) begin
                        // resend of a profile block: the rest comes from the camera's record
                        n_jobs <= n_jobs + 32'd1;
                        code <= 32'd0;
                        j_prof <= 1'b1; j_cam0 <= dz_cam[0];
                        j_resend <= d0[56]; g_pid <= dz_pid0; g_pid1 <= dz_pid1; g_bid <= dz_bid;
                        p_cfg_only <= 1'b0;
                        begin st <= S_PCFG; st_rd <= rd_st(S_PCFG); st_dl <= dl_st(S_PCFG); end
                    end else if (chk_ok) begin
                        j_dmac <= dz_dmac; j_smac <= dz_smac; j_dip <= dz_dip; j_sip <= dz_sip;
                        j_dport <= dz_dport; j_sport <= dz_sport; j_gap <= dz_gap;
                        job_tgl <= ~job_tgl;
                        n_jobs <= n_jobs + 32'd1;
                        code <= 32'd0;
                        j_resend <= d0[56]; g_lw <= dz_lw; frame_base <= dz_fbase; g_fseq <= dz_fseq;
                        g_pid <= dz_pid0; g_pid1 <= dz_pid1; g_w0 <= dz_w0; g_t <= dz_t; g_bid <= dz_bid;
                        g_w0z <= (dz_w0 == 32'd0); g_nock <= d0[57];
                        // the image words alone: with chunks 14 more (their tag and the three chunks)
                        g_tm <= dz_t - (d0[57] ? 32'd0 : 32'd14);
                        g_row0 <= dz_row0; g_skip <= dz_word0[8:0]; g_tpl <= dz_tpl;
                        g_crcovr <= dz_crcovr; g_rbovr <= dz_rbovr; g_ndata <= dz_ndata;
                        begin st <= S_GTPL; st_rd <= rd_st(S_GTPL); st_dl <= dl_st(S_GTPL); end
                    end else begin
                        code <= 32'd1;                  // rejected: report it and wait for the next seq
                        begin st <= S_STAT; st_rd <= rd_st(S_STAT); st_dl <= dl_st(S_STAT); end
                    end
                end
            end

            // ---------------------------------------------------------- frame header
            S_FHDR: begin
                // VID0 words 0..23: magic, seq, ..., height (4), strobe (21)
                m_arvalid <= 1'b1; m_araddr <= frame_base; m_arlen <= 8'd2; rbeat <= 2'd0;
                begin st <= S_FHDR_R; st_rd <= rd_st(S_FHDR_R); st_dl <= dl_st(S_FHDR_R); end
            end
            S_FHDR_R: begin
                if (m_arvalid && m_arready) m_arvalid <= 1'b0;
                if (rb_v) begin
                    rb_v <= 1'b0;
                    if (rbeat == 2'd0) begin
                        f_magic  <= rb[31:0];
                        f_seq    <= rb[63:32];
                        f_height <= (rb[159:144] < 16'd64 || rb[159:144] > HMAX) ? HMAX : rb[159:144];
                    end
                    if (rbeat == 2'd2) f_strobe <= rb[191:160];       // word 21 = word 5 of beat 2
                    rbeat <= rbeat + 2'd1;
                    if (rb_last) begin
                        row <= 16'd0;
                        row_addr <= frame_base + PIX_BASE_OFF;
                        begin st <= S_GCHK; st_rd <= rd_st(S_GCHK); st_dl <= dl_st(S_GCHK); end
                    end
                end
            end
            // ---------------------------------------------------------- rows
            S_ROW: begin
                // one row: 34 beats from row_addr, split at the 4 KB page
                beats_left <= ROW_BEATS;
                rw_n <= 9'd0; rw_i <= 3'd0;
                begin st <= S_ROW_AR; st_rd <= rd_st(S_ROW_AR); st_dl <= dl_st(S_ROW_AR); end
            end
            S_ROW_AR: if (!ar_busy) begin
                m_arvalid <= 1'b1; m_araddr <= row_addr;
                m_arlen   <= ((to_page < {2'd0, beats_left}) ? to_page : {2'd0, beats_left}) - 8'd1;
                ar_busy   <= 1'b1;
                begin st <= S_ROW_R; st_rd <= rd_st(S_ROW_R); st_dl <= dl_st(S_ROW_R); end
            end
            S_ROW_R: begin
                if (m_arvalid && m_arready) m_arvalid <= 1'b0;
                // words of the buffered beat: pixels to the FIFO, the trailer kept;
                // the beat is counted as its first word goes. GVSP: a full packet
                // ends here in the middle of the row; the row goes on after the
                // next packet's header (its beats wait in axi_mux, RREADY low).
                if (rb_v && pkt_left == 16'd0 && rw_n < ROW_WORDS && rw_n >= g_skip) begin
                    begin st <= S_GPEND; st_rd <= rd_st(S_GPEND); st_dl <= dl_st(S_GPEND); end
                end else if (rb_v && ws_free) begin
                    if (rw_i == 3'd0) begin
                        row_addr   <= row_addr + 30'd32;
                        beats_left <= beats_left - 6'd1;
                        if (rb_last) ar_busy <= 1'b0;
                    end
                    if (rw_n < ROW_WORDS) begin
                        if (rw_n >= g_skip) begin
                            ws_v <= 1'b1; ws_ctrl <= 1'b0; ws_crc <= 1'b1;
                            ws   <= rb[32 * rw_i +: 32];
                            pkt_left <= pkt_left - 16'd1;
                        end
                    end else if (rw_n == ROW_WORDS) begin
                        tag0_ok <= (rb[32 * rw_i +: 32] == {16'hA5A5, row});
                    end else begin
                        tag1_ok <= (rb[32 * rw_i +: 32] == f_seq);
                    end
                    rw_i <= rw_i + 3'd1;
                    rw_n <= rw_n + 9'd1;
                    if (rw_i == 3'd7) rb_v <= 1'b0;
                    if (rw_n == 9'd271) begin st <= S_ROW_END; st_rd <= rd_st(S_ROW_END); st_dl <= dl_st(S_ROW_END); end
                end
                // the row crossed a page: the second burst once the first is done
                // (its last beat counted and rb drained - nothing of it in flight)
                if (!ar_busy && !rb_v && beats_left != 6'd0 && !m_arvalid)
                    begin st <= S_ROW_AR; st_rd <= rd_st(S_ROW_AR); st_dl <= dl_st(S_ROW_AR); end
            end
            S_ROW_END: begin
                // trailer as the firmware checks it: {0xA5A5, row} and the frame number
                if (!(tag0_ok && tag1_ok) && rows_bad != 32'hFFFF_FFFF) rows_bad <= rows_bad + 32'd1;
                row_addr <= row_addr + ROW_STRIDE - 30'd1088;   // the stride's unwritten padding
                row   <= row + 16'd1;
                g_skip <= 9'd0; in_row <= 1'b0;
                begin st <= S_GDATA; st_rd <= rd_st(S_GDATA); st_dl <= dl_st(S_GDATA); end
            end
            // ---------------------------------------------------------- end of job
            S_DRAIN: if (pkts_sent == pkts_pushed) begin st <= j_auto ? S_PSTAT : S_STAT; st_rd <= rd_st(j_auto ? S_PSTAT : S_STAT); st_dl <= dl_st(j_auto ? S_PSTAT : S_STAT); end
            S_STAT: begin
                m_awvalid <= 1'b1; m_wvalid <= 1'b1; wbeat <= 1'b0;
                // GVSP: word 6 is the image CRC instead of the cycle count (the chunk
                // value a later resend of the chunk packet must repeat)
                m_wdata <= {STAT_MAGIC, ~crc, code, {n_axi_err, 16'd0},
                            j_prof ? {16'd0, p_bad} : rows_bad, pkts_pushed, {frames_done, j_id}, j_seq};
                begin st <= S_STAT_B; st_rd <= rd_st(S_STAT_B); st_dl <= dl_st(S_STAT_B); end
            end
            S_STAT_B: begin
                if (m_awready) m_awvalid <= 1'b0;
                if (m_wvalid && m_wready) begin
                    if (!wbeat) begin
                        // word 1: seq again at the end, as in the descriptor
                        wbeat   <= 1'b1;
                        m_wdata <= {j_seq, tx_snap[159:128], tx_snap[127:96], tx_snap[95:64],
                                    tx_snap[63:32], tx_snap[31:0], n_pkts, n_jobs};
                    end else begin
                        m_wvalid <= 1'b0;
                    end
                end
                if (m_bvalid && !m_awvalid && !m_wvalid) begin st <= S_IDLE; st_rd <= rd_st(S_IDLE); st_dl <= dl_st(S_IDLE); end
            end

            // ---------------------------------------------------------- GVSP block
            S_GTPL: begin                               // the leader / trailer template, 2 beats
                m_arvalid <= 1'b1; m_araddr <= g_tpl; m_arlen <= 8'd1; rbeat <= 2'd0;
                begin st <= S_GTPL_R; st_rd <= rd_st(S_GTPL_R); st_dl <= dl_st(S_GTPL_R); end
            end
            S_GTPL_R: begin
                if (m_arvalid && m_arready) m_arvalid <= 1'b0;
                if (rb_v) begin
                    rb_v <= 1'b0;
                    if (rbeat == 2'd0) t0 <= rb; else t1 <= rb;
                    rbeat <= rbeat + 2'd1;
                    if (rb_last) begin st <= S_FHDR; st_rd <= rd_st(S_FHDR); st_dl <= dl_st(S_FHDR); end          // then the frame's VID0 header
                end
            end
            S_GCHK: begin
                g_t270 <= ({3'd0, f_height} << 8) + ({3'd0, f_height} << 3) + ({3'd0, f_height} << 2)
                        + ({3'd0, f_height} << 1);
                g_bad1 <= (f_magic != VID0_MAGIC) || (f_seq != g_fseq);
                begin st <= S_GCHK2; st_rd <= rd_st(S_GCHK2); st_dl <= dl_st(S_GCHK2); end
            end
            S_GCHK2: begin
                // the slot must still hold the frame the firmware saw, of the height it
                // counted the block for; otherwise nothing is sent (code 2)
                if (g_bad1 || {13'd0, g_t270} != g_tm) begin
                    code <= 32'd2;
                    begin st <= S_STAT; st_rd <= rd_st(S_STAT); st_dl <= dl_st(S_STAT); end
                end else begin
                    frames_done <= 16'd1;
                    row <= g_row0; row_end <= f_height; in_row <= 1'b0;
                    row_addr <= frame_base + PIX_BASE_OFF + ({14'd0, g_row0} << 10) + ({14'd0, g_row0} << 8);
                    g_ck <= (g_w0 > {13'd0, g_t270}) ? (g_w0[3:0] - g_t270[3:0]) : 4'd0;
                    g_wcur <= g_w0;
                    crc <= 32'hFFFF_FFFF;
                    begin st <= S_GPKT; st_rd <= rd_st(S_GPKT); st_dl <= dl_st(S_GPKT); end
                end
            end
            S_GPKT: begin
                if (g_pid > g_pid1) begin
                    begin st <= S_GFLUSH; st_rd <= rd_st(S_GFLUSH); st_dl <= dl_st(S_GFLUSH); end
                end else begin
                    g_kind  <= (g_pid == 16'd0) ? 2'd0 : (g_pid == g_ndata + 16'd1) ? 2'd2 : 2'd1;
                    g_words <= j_prof ? ((g_pid == 16'd0) ? {8'd0, p_lb} : {8'd0, p_tb})
                                      : (g_left < {16'd0, g_lw}) ? g_left[15:0] : g_lw;
                    p_left  <= cur_tot - p_pw;
                    begin st <= (j_prof && g_pid != 16'd0 && g_pid != g_ndata + 16'd1) ? S_GPKT2 : S_GCTRL; st_rd <= rd_st((j_prof && g_pid != 16'd0 && g_pid != g_ndata + 16'd1) ? S_GPKT2 : S_GCTRL); st_dl <= dl_st((j_prof && g_pid != 16'd0 && g_pid != g_ndata + 16'd1) ? S_GPKT2 : S_GCTRL); end
                end
            end
            S_GPKT2: begin                              // profile data packet: size, end of part, offset
                g_words <= (p_left < g_lw) ? p_left : g_lw;
                p_eop   <= (p_left <= g_lw);
                p_aoff  <= cur_off + {6'd0, p_pw, 2'b00};
                begin st <= S_GCTRL; st_rd <= rd_st(S_GCTRL); st_dl <= dl_st(S_GCTRL); end
            end
            S_GCTRL: if (ws_free) begin
                // multi-part data packets carry 8 more header bytes: part, flags, address offset
                ws_v <= 1'b1; ws_ctrl <= 1'b1; ws_crc <= 1'b0;
                ws   <= {16'd0, j_prof ? (((g_kind == 2'd1) ? 16'd28 : 16'd20) + {g_words[13:0], 2'b00})
                                       : (g_kind == 2'd0) ? 16'd56 : (g_kind == 2'd2) ? (g_nock ? 16'd28 : 16'd36)
                                       : (16'd20 + {g_words[13:0], 2'b00})};
                pkt_left <= (g_kind == 2'd1 || j_prof) ? g_words : 16'd0;
                hw_i <= 3'd0; g_bw <= 4'd0; g_bw1 <= 4'd1;
                hw_last <= 1'b0;                        // the header has at least 5 words
                gh_last <= (j_prof && g_kind == 2'd1) ? 3'd6 : 3'd4;
                gh_nst  <= !j_prof ? ((g_kind == 2'd1) ? S_GDATA : S_GBODY) : (g_kind == 2'd1) ? S_PDATA : S_PTPL;
                g_tw <= body_w(4'd0, g_kind == 2'd0);
                begin st <= S_GHDR; st_rd <= rd_st(S_GHDR); st_dl <= dl_st(S_GHDR); end
            end
            S_GHDR: if (ws_free) begin                  // extended-ID GVSP header, 20 B (+8 multi-part)
                ws_v <= 1'b1; ws_ctrl <= 1'b0; ws_crc <= 1'b0;
                case (hw_i)
                    3'd0: ws <= be4({16'd0, 15'd0, j_resend});                         // status, flag
                    3'd1: ws <= be4({1'b1, 3'd0, (g_kind == 2'd0) ? 4'd1 : (g_kind == 2'd2) ? 4'd2 : j_prof ? 4'd7 : 4'd3,
                                     16'd0, (j_prof && g_kind == 2'd0) ? {5'd0, p_nparts} : 8'd0});   // nr_parts
                    3'd2: ws <= be4(g_bid[63:32]);
                    3'd3: ws <= be4(g_bid[31:0]);
                    3'd4: ws <= be4({16'd0, g_pid});                                    // packet_id32
                    3'd5: ws <= be4({6'd0, p_k, 1'b0, p_eop, 5'd0, p_eop, 16'd0});       // part_id, EOP, zone 0, EOZ
                    default: ws <= be4({8'd0, p_aoff});                                 // address_offset
                endcase
                hw_i <= hw_i + 3'd1;
                hw_last <= (hw_i + 3'd1 == gh_last);
                if (hw_last)
                    begin st <= gh_nst; st_rd <= rd_st(gh_nst); st_dl <= dl_st(gh_nst); end
            end
            S_GBODY: if (ws_free) begin                 // leader body (9 words) or trailer body (4)
                ws_v <= 1'b1; ws_ctrl <= 1'b0; ws_crc <= 1'b0;
                ws <= g_tw;
                g_tw <= body_w(g_bw1, g_kind == 2'd0);
                g_bw <= g_bw1; g_bw1 <= g_bw1 + 4'd1;
                // trailer body: 2 words for 0x0001 (reserved + type, size_y), 4 with the chunk fields
                if ((g_kind == 2'd0) ? (g_bw == 4'd8) : (g_bw == (g_nock ? 4'd1 : 4'd3))) begin st <= S_GPEND; st_rd <= rd_st(S_GPEND); st_dl <= dl_st(S_GPEND); end
            end
            S_GDATA: begin                              // the next data word's source
                if (pkt_left == 16'd0) begin
                    begin st <= S_GPEND; st_rd <= rd_st(S_GPEND); st_dl <= dl_st(S_GPEND); end
                end else if (in_row) begin
                    begin st <= S_ROW_R; st_rd <= rd_st(S_ROW_R); st_dl <= dl_st(S_ROW_R); end
                end else if (row < row_end) begin
                    in_row <= 1'b1;
                    begin st <= S_ROW; st_rd <= rd_st(S_ROW); st_dl <= dl_st(S_ROW); end
                end else if (ws_free) begin             // chunks after the image
                    // the image is the first chunk and carries its own tag (GEV 2.2 § 25.1)
                    ws_v <= 1'b1; ws_ctrl <= 1'b0; ws_crc <= 1'b0;
                    case (g_ck)
                        4'd0:       ws <= be4(CID_PAY);
                        4'd1:       ws <= be4({11'd0, g_t270, 2'b00});   // 1080 * H bytes
                        4'd2, 4'd6: ws <= 32'd0;
                        4'd3:       ws <= be4(f_seq);           // ChunkFrameID: the FPGA frame number
                        4'd4:       ws <= be4(32'h0000_1001);
                        4'd7:       ws <= be4(f_strobe);        // ChunkTimestamp
                        4'd8:       ws <= be4(32'h0000_1002);
                        4'd10:      ws <= be4(g_crc_v);         // ChunkFrameCRC: image CRC, bad rows
                        4'd11:      ws <= be4(g_rbad_v);
                        4'd12:      ws <= be4(32'h0000_1005);
                        default:    ws <= be4(32'd8);           // 5, 9, 13: chunk lengths
                    endcase
                    g_ck <= g_ck + 4'd1;
                    pkt_left <= pkt_left - 16'd1;
                end
            end
            S_GFLUSH: begin
                // a range that ended inside a row leaves the rest of its burst in
                // axi_mux: take and drop it, or the next job would read it as its own
                if (m_arvalid && m_arready) m_arvalid <= 1'b0;
                if (rb_v) begin
                    rb_v <= 1'b0;
                    // a beat not started counts its last flag (frames); a word run counts a
                    // beat only when it is done, so its held beat always does
                    if ((rw_i == 3'd0 || j_prof) && rb_last) ar_busy <= 1'b0;
                    rw_i <= 3'd0;
                end else if (!ar_busy && !m_arvalid) begin
                    in_row <= 1'b0; r_act <= 1'b0;
                    begin st <= S_DRAIN; st_rd <= rd_st(S_DRAIN); st_dl <= dl_st(S_DRAIN); end
                end
            end
            S_GPEND: if (!ws_v) begin                   // last word taken: the packet is whole
                pkt_tgl <= ~pkt_tgl;
                pkts_pushed <= pkts_pushed + 32'd1;
                n_pkts <= n_pkts + 32'd1;
                if (g_kind == 2'd1) g_wcur <= g_wcur + {16'd0, g_words};
                g_pid <= g_pid + 16'd1;
                begin st <= S_GPKT; st_rd <= rd_st(S_GPKT); st_dl <= dl_st(S_GPKT); end
                if (j_prof && g_kind == 2'd1) begin
                    if (p_eop) begin                    // next part from its start
                        p_k <= p_k + 2'd1; p_pw <= 16'd0; p_l <= 5'd0; p_seg <= 2'd0; p_ci <= 1'b0;
                        p_iw0 <= 11'd0;
                        begin st <= S_PSEL; st_rd <= rd_st(S_PSEL); st_dl <= dl_st(S_PSEL); end
                    end else begin
                        p_pw <= p_pw + g_words;
                    end
                end
            end
            // ---------------------------------------------------------- profile stream: scheduler
            // Between descriptor polls, one camera per visit: its record's seq (a change =
            // read it whole), then the final burst of its next block's last frame.
            S_AUTO: begin
                m_arvalid <= 1'b1; m_araddr <= PCFG_ADDR + {22'd0, ac, 7'd0}; m_arlen <= 8'd0;
                a_seqc <= a_seq;
                begin st <= S_ACFG_R; st_rd <= rd_st(S_ACFG_R); st_dl <= dl_st(S_ACFG_R); end
            end
            S_ACFG_R: begin
                if (m_arvalid && m_arready) m_arvalid <= 1'b0;
                if (rb_v) begin
                    rb_v <= 1'b0;
                    if (rb[31:0] != a_seqc) begin
                        j_cam0 <= ac; p_cfg_only <= 1'b1;
                        begin st <= S_PCFG; st_rd <= rd_st(S_PCFG); st_dl <= dl_st(S_PCFG); end
                    end else begin
                        begin st <= a_on[ac] ? S_ARDY : S_ANEXT; st_rd <= rd_st(a_on[ac] ? S_ARDY : S_ANEXT); st_dl <= dl_st(a_on[ac] ? S_ARDY : S_ANEXT); end
                    end
                end
            end
            S_ARDY: begin
                a_flast <= a_next + ((32'd1 << a_nlog) - 32'd1);
                begin st <= S_ARDY2; st_rd <= rd_st(S_ARDY2); st_dl <= dl_st(S_ARDY2); end
            end
            S_ARDY2: begin
                m_arvalid <= 1'b1; m_araddr <= a_ring + {2'd0, a_flast[12:0], 15'd0} + BLK_OFF + FIN_OFF;
                m_arlen <= 8'd7; pb <= 3'd0;
                begin st <= S_ARDY_R; st_rd <= rd_st(S_ARDY_R); st_dl <= dl_st(S_ARDY_R); end
            end
            S_ARDY_R: begin
                if (m_arvalid && m_arready) m_arvalid <= 1'b0;
                if (rb_v) begin
                    rb_v <= 1'b0;
                    pb <= pb + 3'd1;
                    if (pb == 3'd0) a_f0 <= rb[31:0];
                    if (pb == 3'd1) begin a_eq <= (a_df == 32'd0); a_ahead <= !a_df[31] && a_df[31:20] == 12'd0; end
                    if (rb_last) begin
                        if (rb[255:224] == FIN_MAGIC && a_eq) begin
                            // the block's last line is finished: send the block
                            j_cam0 <= ac; j_auto <= 1'b1; j_prof <= 1'b1;
                            j_resend <= 1'b0; p_cfg_only <= 1'b0; code <= 32'd0;
                            pkts_pushed <= 32'd0; pkts_sent <= 32'd0;
                            begin st <= S_PCFG; st_rd <= rd_st(S_PCFG); st_dl <= dl_st(S_PCFG); end
                        end else begin
                            // a newer frame in the slot (within 2^20 frames, not power-up
                            // rubbish): the ring overtook the stream - go on from that frame's block
                            if (rb[255:224] == FIN_MAGIC && a_ahead) begin
                                if (ac) begin a_next1 <= a_f0 & ~((32'd1 << a_nlog) - 32'd1); pc_skip1 <= pc_skip1 + 16'd1; end
                                else    begin a_next0 <= a_f0 & ~((32'd1 << a_nlog) - 32'd1); pc_skip0 <= pc_skip0 + 16'd1; end
                            end
                            begin st <= S_ANEXT; st_rd <= rd_st(S_ANEXT); st_dl <= dl_st(S_ANEXT); end
                        end
                    end
                end
            end
            S_ANEXT: begin
                ac <= ~ac;
                begin st <= S_IDLE; st_rd <= rd_st(S_IDLE); st_dl <= dl_st(S_IDLE); end
            end

            // ---------------------------------------------------------- profile job set-up
            S_PCFG: begin                               // the camera's record, 3 beats into d0..d2
                m_arvalid <= 1'b1; m_araddr <= PCFG_ADDR + {22'd0, j_cam0, 7'd0}; m_arlen <= 8'd2; rbeat <= 2'd0;
                begin st <= S_PCFG_R; st_rd <= rd_st(S_PCFG_R); st_dl <= dl_st(S_PCFG_R); end
            end
            S_PCFG_R: begin
                if (m_arvalid && m_arready) m_arvalid <= 1'b0;
                if (rb_v) begin
                    rb_v <= 1'b0;
                    rbeat <= rbeat + 2'd1;
                    if (rb_last) begin st <= S_PCHK0; st_rd <= rd_st(S_PCHK0); st_dl <= dl_st(S_PCHK0); end
                end
            end
            S_PCHK0: begin
                // an autonomous block also needs the record the scheduler learned (seq unchanged);
                // a resend range past the trailer, compared here so S_PCHK's enables stay shallow
                chk_ok  <= pc_ok && (dz_cam == {7'd0, j_cam0}) && (p_cfg_only || !j_auto || dz_seq == a_seqc);
                chk_rng <= !j_auto && (g_pid1 > pc_ndata + 16'd1);
                begin st <= S_PCHK; st_rd <= rd_st(S_PCHK); st_dl <= dl_st(S_PCHK); end
            end
            S_PCHK: begin
                if (p_cfg_only) begin
                    // learn the stream; a record that fails the check turns it off. A torn
                    // one (the CPU between its two phases) is left alone: read again next visit.
                    p_cfg_only <= 1'b0;
                    if (dz_seq == dz_seq2) begin
                        a_on[j_cam0] <= chk_ok && pc_on;
                        a_seqc <= dz_seq;            // a block found ready in this visit checks against it
                        if (j_cam0) begin
                            a_seq1 <= dz_seq; a_nlog1 <= pc_nlog; a_ring1 <= dz_ring;
                            a_next1 <= pc_start & ~((32'd1 << pc_nlog) - 32'd1);
                        end else begin
                            a_seq0 <= dz_seq; a_nlog0 <= pc_nlog; a_ring0 <= dz_ring;
                            a_next0 <= pc_start & ~((32'd1 << pc_nlog) - 32'd1);
                        end
                    end
                    begin st <= (chk_ok && pc_on) ? S_ARDY : S_ANEXT; st_rd <= rd_st((chk_ok && pc_on) ? S_ARDY : S_ANEXT); st_dl <= dl_st((chk_ok && pc_on) ? S_ARDY : S_ANEXT); end
                end else if (!chk_ok || chk_rng) begin
                    // record changed under the block, or a range beyond it
                    if (j_auto) begin
                        a_on[j_cam0] <= 1'b0;
                        if (j_cam0) begin a_seq1 <= ~a_seq1; pc_err1 <= pc_err1 + 16'd1; end
                        else        begin a_seq0 <= ~a_seq0; pc_err0 <= pc_err0 + 16'd1; end
                        j_auto <= 1'b0; j_prof <= 1'b0;
                        begin st <= S_ANEXT; st_rd <= rd_st(S_ANEXT); st_dl <= dl_st(S_ANEXT); end
                    end else begin
                        code <= 32'd3;
                        begin st <= S_STAT; st_rd <= rd_st(S_STAT); st_dl <= dl_st(S_STAT); end
                    end
                end else begin
                    j_dmac <= dz_dmac; j_smac <= dz_smac; j_dip <= dz_dip; j_sip <= dz_sip;
                    j_dport <= dz_dport; j_sport <= dz_sport; j_gap <= dz_gap;
                    job_tgl <= ~job_tgl;
                    g_lw <= dz_lw; g_ndata <= pc_ndata;
                    p_nlog <= pc_nlog; p_nparts <= pc_nparts; p_n <= 5'd1 << pc_nlog;
                    p_ring <= dz_ring; p_tpl <= pc_tpl; p_lb <= pc_lb; p_tb <= pc_tb; p_lenp <= pc_lenp;
                    p_parts <= d2[171:0];
                    if (j_auto) begin
                        p_first <= a_next; p_fl <= a_next;
                        g_bid <= {32'd0, a_next >> pc_nlog} + 64'd1;   // block id = first frame / N + 1
                        g_pid <= 16'd0; g_pid1 <= pc_ndata + 16'd1;
                    end else begin
                        p_first <= (g_bid[31:0] - 32'd1) << pc_nlog;
                        p_fl    <= (g_bid[31:0] - 32'd1) << pc_nlog;
                    end
                    p_good <= 16'd0; p_bad <= 16'd0; p_l <= 5'd0; f_strobe <= 32'd0; frames_done <= 16'd1;
                    begin st <= S_PPRE; st_rd <= rd_st(S_PPRE); st_dl <= dl_st(S_PPRE); end
                end
            end

            // ---------------------------------------------------------- line checks
            // Every line: header (magic, frame) and final burst (frame, end word) before any
            // data; a line that fails is sent as zeros.
            S_PPRE: begin
                if (p_l == p_n) begin
                    begin st <= S_SK0; st_rd <= rd_st(S_SK0); st_dl <= dl_st(S_SK0); end
                end else begin
                    p_bb <= p_ring + {2'd0, p_fl[12:0], 15'd0} + BLK_OFF;
                    begin st <= S_PPRE1; st_rd <= rd_st(S_PPRE1); st_dl <= dl_st(S_PPRE1); end
                end
            end
            S_PPRE1: begin
                m_arvalid <= 1'b1; m_araddr <= p_bb; m_arlen <= 8'd0;
                begin st <= S_PPRE_H; st_rd <= rd_st(S_PPRE_H); st_dl <= dl_st(S_PPRE_H); end
            end
            S_PPRE_H: begin
                if (m_arvalid && m_arready) m_arvalid <= 1'b0;
                if (rb_v) begin
                    rb_v <= 1'b0;
                    ph_ok <= (rb[31:0] == PROF_MAGIC) && (rb[63:32] == p_fl);
                    if (p_l == 5'd0) f_strobe <= rb[127:96];     // header word 3: the leader timestamp
                    m_arvalid <= 1'b1; m_araddr <= p_bb + FIN_OFF; m_arlen <= 8'd7; pb <= 3'd0;
                    begin st <= S_PPRE_F; st_rd <= rd_st(S_PPRE_F); st_dl <= dl_st(S_PPRE_F); end
                end
            end
            S_PPRE_F: begin
                if (m_arvalid && m_arready) m_arvalid <= 1'b0;
                if (rb_v) begin
                    rb_v <= 1'b0;
                    pb <= pb + 3'd1;
                    if (pb == 3'd0) pf_ok <= (rb[31:0] == p_fl);
                    if (rb_last) begin pfin_ok <= (rb[255:224] == FIN_MAGIC); begin st <= S_PPRE_C; st_rd <= rd_st(S_PPRE_C); st_dl <= dl_st(S_PPRE_C); end end   // word 63
                end
            end
            S_PPRE_C: begin
                if (ph_ok && pf_ok && pfin_ok) p_good[p_l[3:0]] <= 1'b1;
                else p_bad <= p_bad + 16'd1;
                p_l <= p_l + 5'd1; p_fl <= p_fl + 32'd1;
                begin st <= S_PPRE; st_rd <= rd_st(S_PPRE); st_dl <= dl_st(S_PPRE); end
            end

            // ---------------------------------------------------------- seek (resend inside a block)
            // Packet id -> part (subtract the parts' packet counts), word in the part (lw per
            // packet), line and input word in it (subtract the words per line): counters only.
            S_SK0: begin
                p_l <= 5'd0; p_seg <= 2'd0; p_ci <= 1'b0; p_pw <= 16'd0; p_iw0 <= 11'd0; p_k <= 2'd0;
                p_x <= 16'd0;
                if (g_pid == 16'd0 || g_pid > g_ndata) begin
                    p_seeking <= 1'b0;
                    begin st <= S_PSEL; st_rd <= rd_st(S_PSEL); st_dl <= dl_st(S_PSEL); end
                end else begin
                    p_rem <= g_pid - 16'd1;
                    begin st <= S_SK1; st_rd <= rd_st(S_SK1); st_dl <= dl_st(S_SK1); end
                end
            end
            S_SK1: begin
                sk_ph <= !sk_ph;
                if (sk_ph) begin
                    if (sk_more) begin
                        p_rem <= p_rem - sk_np;
                        p_k <= p_k + 2'd1;
                    end else begin
                        p_seeking <= 1'b1;
                        begin st <= S_PSEL; st_rd <= rd_st(S_PSEL); st_dl <= dl_st(S_PSEL); end
                    end
                end
            end
            S_SK2: begin
                if (p_rem != 16'd0) begin
                    p_x <= p_x + g_lw;
                    p_rem <= p_rem - 16'd1;
                end else begin
                    p_pw <= p_x;
                    begin st <= S_SK3; st_rd <= rd_st(S_SK3); st_dl <= dl_st(S_SK3); end
                end
            end
            S_SK3: begin
                if (cur_kind == PK_CHUNK && p_x < 16'd2) begin
                    p_ci <= p_x[0]; p_seeking <= 1'b0;
                    begin st <= S_GPKT; st_rd <= rd_st(S_GPKT); st_dl <= dl_st(S_GPKT); end
                end else begin
                    if (cur_kind == PK_CHUNK) begin p_x <= p_x - 16'd2; p_seg <= 2'd1; end
                    begin st <= S_SK4; st_rd <= rd_st(S_SK4); st_dl <= dl_st(S_SK4); end
                end
            end
            S_SK4: begin
                if (p_l != p_n && p_x >= {5'd0, wpl_k}) begin
                    p_x <= p_x - {5'd0, wpl_k};
                    p_l <= p_l + 5'd1;
                end else begin
                    if (cur_kind == PK_CHUNK && p_l == p_n) begin p_seg <= 2'd2; p_ci <= p_x[0]; end
                    else p_iw0 <= (ipo_k == 3'd2) ? {p_x[9:0], 1'b0} : (ipo_k == 3'd4) ? {p_x[8:0], 2'b00} : p_x[10:0];
                    p_seeking <= 1'b0;
                    begin st <= S_GPKT; st_rd <= rd_st(S_GPKT); st_dl <= dl_st(S_GPKT); end
                end
            end
            S_PSEL: begin                               // part p_k: kind, packets, offset, words
                cur_kind <= part_k[2:0];
                cur_np   <= part_k[18:3];
                cur_off  <= part_k[42:19];
                cur_tot  <= (part_k[2:0] == PK_CHUNK) ? 16'd4 + (16'd24 << p_nlog) :
                            (part_k[2:0] == XF_C16)   ? (16'd540 << p_nlog) :
                            (part_k[2:0] == XF_AC16)  ? (16'd1080 << p_nlog) : (16'd270 << p_nlog);
                begin st <= p_seeking ? S_SK2 : S_GPKT; st_rd <= rd_st(p_seeking ? S_SK2 : S_GPKT); st_dl <= dl_st(p_seeking ? S_SK2 : S_GPKT); end
            end

            // ---------------------------------------------------------- data of a part
            S_PDATA: begin
                if (pkt_left == 16'd0) begin
                    begin st <= S_GPEND; st_rd <= rd_st(S_GPEND); st_dl <= dl_st(S_GPEND); end
                end else if (r_act) begin
                    begin st <= r_zero ? S_RUNZ : S_RUNR; st_rd <= rd_st(r_zero ? S_RUNZ : S_RUNR); st_dl <= dl_st(r_zero ? S_RUNZ : S_RUNR); end     // the run of the previous packet goes on
                end else if (cur_kind == PK_CHUNK && p_seg != 2'd1) begin
                    // chunk part: the payload chunk's tag, the lines, ChunkProfileInfo's own tag
                    if (ws_free) begin
                        ws_v <= 1'b1; ws_ctrl <= 1'b0; ws_crc <= 1'b0;
                        ws <= (p_seg == 2'd0) ? (p_ci ? be4({8'd0, p_lenp}) : be4(CID_PAY))
                                              : (p_ci ? be4({16'd0, 16'd96 << p_nlog}) : be4(CID_PINFO));
                        pkt_left <= pkt_left - 16'd1;
                        p_ci <= ~p_ci;
                        if (p_seg == 2'd0 && p_ci) begin p_seg <= 2'd1; p_l <= 5'd0; end
                    end
                end else if (p_l == p_n) begin
                    p_seg <= 2'd2; p_ci <= 1'b0;        // chunk part: its lines are done
                end else begin
                    p_fl <= p_first + {27'd0, p_l};
                    begin st <= S_PLB; st_rd <= rd_st(S_PLB); st_dl <= dl_st(S_PLB); end
                end
            end
            S_PLB: begin
                p_bb <= p_ring + {2'd0, p_fl[12:0], 15'd0} + BLK_OFF;
                begin st <= S_PLB2; st_rd <= rd_st(S_PLB2); st_dl <= dl_st(S_PLB2); end
            end
            S_PLB2: begin st <= S_PRUN; st_rd <= rd_st(S_PRUN); st_dl <= dl_st(S_PRUN); end
            S_PRUN: begin                               // a run over line p_l of the current part
                r_act <= 1'b1; r_grp <= 2'd0; r_iw <= p_iw0; r_fexp <= p_fl; r_tpl <= 1'b0; r_first <= 1'b1;
                p_iw0 <= 11'd0;
                r_zero <= !p_good[p_l[3:0]];
                if (cur_kind == PK_CHUNK) begin
                    r_base <= p_bb; r_n <= 11'd24; r_xf <= XF_RAW; r_patch <= 2'd2; r_c16 <= 1'b0; r_b8 <= 1'b0;
                end else begin
                    r_base <= p_bb + ((cur_kind == XF_AC16) ? MM_OFF : PTS_OFF);
                    r_n <= 11'd1080; r_xf <= cur_kind; r_patch <= 2'd0;
                    r_c16 <= (cur_kind == XF_C16); r_b8 <= (cur_kind == XF_M8 || cur_kind == XF_C8);
                end
                begin st <= p_good[p_l[3:0]] ? S_RUNA : S_RUNZ; st_rd <= rd_st(p_good[p_l[3:0]] ? S_RUNA : S_RUNZ); st_dl <= dl_st(p_good[p_l[3:0]] ? S_RUNA : S_RUNZ); end
            end
            S_PTPL: begin                               // leader / trailer body from the template
                r_act <= 1'b1; r_grp <= 2'd0; r_iw <= 11'd0; r_zero <= 1'b0; r_xf <= XF_RAW; r_tpl <= 1'b1;
                r_c16 <= 1'b0; r_b8 <= 1'b0;
                r_first <= 1'b1;
                r_patch <= (g_kind == 2'd0) ? 2'd1 : 2'd0;
                r_base  <= p_tpl + ((g_kind == 2'd0) ? 30'd0 : 30'h100);
                r_n     <= {3'd0, (g_kind == 2'd0) ? p_lb : p_tb};
                begin st <= S_RUNA; st_rd <= rd_st(S_RUNA); st_dl <= dl_st(S_RUNA); end
            end

            // ---------------------------------------------------------- word-run engine
            S_RUNA: begin
                if (r_first) begin
                    r_addr  <= r_base + {17'd0, r_iw[10:3], 5'd0};
                    r_bl    <= ((r_n + 11'd7) >> 3) - {3'd0, r_iw[10:3]};
                    rw_i    <= r_iw[2:0];
                    r_first <= 1'b0;
                end else if (!ar_busy) begin
                    m_arvalid <= 1'b1; m_araddr <= r_addr; m_arlen <= r_len - 8'd1;
                    ar_busy <= 1'b1;
                    r_bl <= r_bl - r_len;
                    r_addr <= r_addr + {17'd0, r_len, 5'd0};
                    begin st <= S_RUNR; st_rd <= rd_st(S_RUNR); st_dl <= dl_st(S_RUNR); end
                end
            end
            S_RUNR: begin
                if (m_arvalid && m_arready) m_arvalid <= 1'b0;
                if (rb_v) begin
                    if (pkt_left == 16'd0 && r_grp == 2'd0) begin
                        begin st <= S_GPEND; st_rd <= rd_st(S_GPEND); st_dl <= dl_st(S_GPEND); end                  // packet full: the run goes on in the next one
                    end else if (ws_free || !r_emit) begin
                        if (r_emit) begin
                            ws_v <= 1'b1; ws_ctrl <= 1'b0; ws_crc <= 1'b0;
                            ws <= r_out;
                            pkt_left <= pkt_left - 16'd1;
                            r_grp <= 2'd0;
                        end else begin
                            if (r_c16) r_acc[15:0] <= r_w[15:0];
                            else                r_acc[8 * r_grp +: 8] <= r_lane;
                            r_grp <= r_grp + 2'd1;
                        end
                        if (r_patch == 2'd2 && r_iw == 11'd0 && r_pstat != 32'd1) p_bad <= p_bad + 16'd1;
                        r_iw <= r_iw + 11'd1;
                        rw_i <= rw_i + 3'd1;
                        if (rw_i == 3'd7 || r_lastw) begin
                            rb_v <= 1'b0;
                            if (rb_last) ar_busy <= 1'b0;
                        end
                        if (r_lastw) begin
                            r_act <= 1'b0;
                            if (!r_tpl) p_l <= p_l + 5'd1;
                            begin st <= r_tpl ? S_GPEND : S_PDATA; st_rd <= rd_st(r_tpl ? S_GPEND : S_PDATA); st_dl <= dl_st(r_tpl ? S_GPEND : S_PDATA); end
                        end
                    end
                end else if (!ar_busy && r_bl != 8'd0 && !m_arvalid) begin
                    begin st <= S_RUNA; st_rd <= rd_st(S_RUNA); st_dl <= dl_st(S_RUNA); end                       // the next burst of the run
                end
            end
            S_RUNZ: begin                               // a line not delivered: zeros, no reads
                if (pkt_left == 16'd0) begin
                    begin st <= S_GPEND; st_rd <= rd_st(S_GPEND); st_dl <= dl_st(S_GPEND); end
                end else if (ws_free) begin
                    ws_v <= 1'b1; ws_ctrl <= 1'b0; ws_crc <= 1'b0;
                    ws <= r_zout;
                    pkt_left <= pkt_left - 16'd1;
                    r_iw <= r_iw + {8'd0, r_ipo};
                    if (r_iw + {8'd0, r_ipo} >= r_n) begin
                        r_act <= 1'b0;
                        p_l <= p_l + 5'd1;
                        begin st <= S_PDATA; st_rd <= rd_st(S_PDATA); st_dl <= dl_st(S_PDATA); end
                    end
                end
            end

            // ---------------------------------------------------------- end of an autonomous block
            S_PSTAT: begin
                // camera record + 0x60: magic, next frame, blocks, lines not delivered or
                // changed while sent, {0, overtaken skips}, {record errors, 0}, last block id, record seq
                m_awvalid <= 1'b1; m_wvalid <= 1'b1; wbeat <= 1'b1;
                if (j_cam0) begin
                    a_next1 <= a_next1 + {27'd0, p_n}; pc_blocks1 <= pc_blocks1 + 32'd1;
                    pc_bad1 <= pc_bad1 + {16'd0, p_bad};
                    m_wdata <= {a_seq1, g_bid[31:0], {pc_err1, 16'd0}, {16'd0, pc_skip1}, pc_bad1 + {16'd0, p_bad},
                                pc_blocks1 + 32'd1, a_next1 + {27'd0, p_n}, PST_MAGIC};
                end else begin
                    a_next0 <= a_next0 + {27'd0, p_n}; pc_blocks0 <= pc_blocks0 + 32'd1;
                    pc_bad0 <= pc_bad0 + {16'd0, p_bad};
                    m_wdata <= {a_seq0, g_bid[31:0], {pc_err0, 16'd0}, {16'd0, pc_skip0}, pc_bad0 + {16'd0, p_bad},
                                pc_blocks0 + 32'd1, a_next0 + {27'd0, p_n}, PST_MAGIC};
                end
                ac <= ~ac;
                poll_cnt <= POLL_CYCLES[15:0];          // look at the descriptor right away
                begin st <= S_STAT_B; st_rd <= rd_st(S_STAT_B); st_dl <= dl_st(S_STAT_B); end
            end
            default: begin st <= S_IDLE; st_rd <= rd_st(S_IDLE); st_dl <= dl_st(S_IDLE); end
            endcase
        end
    end

    // synthesis translate_off
    // st_rd must follow bit 6 of the state code on every clock.
    always @(posedge clk or negedge rst_n) begin
        if (rst_n && st_rd !== st[6]) begin
            $display("ERROR udp_rd %m: st_rd=%b st=%0d at %0t", st_rd, st, $time);
            $fatal(1);
        end
    end
    // synthesis translate_on

endmodule
