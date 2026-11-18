// ============================================================================
// Module      : udp_tx
// The FPGA frame stream (docs/ARCHITECTURE.md § 8.1, packet format: spec § 5):
// udp_rd on clk_x1 (descriptor, DDR3 reads, payload and CRC, status), a
// 4096 x 33 packet FIFO (8 BSRAM) across to tx_mac_clk, udp_send there
// (Ethernet/IPv4/UDP headers, bytes to eth_tx_mux).
//
// Since 30.10.2026 the descriptor, the status and the stream records are not
// in DDR3 any more but in apb_ctl (FPGA control registers on APB): udp_rd keeps
// its AXI master, and the router below sends its reads in those two windows
// (DESC_ADDR..+0xFF, PCFG_ADDR..+0xFF) and all its writes to the c_* port;
// only frame, block and template reads reach DDR3 (m_*). udp_rd has one read
// in flight at a time (ERRORS 1.24), so a read's R channel is chosen when its
// AR is accepted.
//
// Crossings: the FIFO, and three toggles with held data - job_tgl (the job's
// fields are held by udp_rd from before the toggle until the job has
// drained), pkt_tgl (a whole packet is in the FIFO) and sent_tgl (a packet is
// on the wire). Packets are microseconds apart, so no toggle can flip twice
// within its synchroniser.
// ============================================================================

`timescale 1ns / 1ps

module udp_tx #(
    parameter [29:0]  DESC_ADDR   = 30'h0600_0400,
    parameter [29:0]  STAT_ADDR   = 30'h0600_0480,
    parameter [29:0]  PCFG_ADDR   = 30'h0600_1100,
    parameter integer POLL_CYCLES = 5000
) (
    // clk_x1 side
    input  wire          clk,
    input  wire          rst_n,
    input  wire          enable,
    output wire          m_arvalid,
    input  wire          m_arready,
    output wire [29:0]   m_araddr,
    output wire [7:0]    m_arlen,
    input  wire          m_rvalid,
    output wire          m_rready,
    input  wire [255:0]  m_rdata,
    input  wire [1:0]    m_rresp,
    input  wire          m_rlast,
    // control windows -> apb_ctl
    output wire          c_arvalid,
    input  wire          c_arready,
    output wire [29:0]   c_araddr,
    output wire [7:0]    c_arlen,
    input  wire          c_rvalid,
    output wire          c_rready,
    input  wire [255:0]  c_rdata,
    input  wire          c_rlast,
    output wire          c_awvalid,
    input  wire          c_awready,
    output wire [29:0]   c_awaddr,
    output wire [7:0]    c_awlen,
    output wire          c_wvalid,
    input  wire          c_wready,
    output wire [255:0]  c_wdata,
    output wire          c_wlast,
    input  wire          c_bvalid,
    output wire          c_bready,
    output wire [31:0]   n_jobs,
    output wire [31:0]   n_pkts,
    output wire [15:0]   n_axi_err,
    output wire          busy,

    // tx_mac_clk side
    input  wire          tx_clk,
    input  wire          tx_rst_n,         // synchronised to tx_clk
    output wire          tx_valid,
    output wire [7:0]    tx_data,
    output wire          tx_last,
    input  wire          tx_ready,
    input  wire          stat_valid,
    output wire [31:0]   n_sent,
    output wire [15:0]   n_stat_to,
    output wire [15:0]   n_underrun,
    output wire [15:0]   n_bad_ctrl,
    // eth_tx_mux's counters (tx_clk), for status word 1
    input  wire [31:0]   mux_n_a,
    input  wire [31:0]   mux_max_hold_a,
    input  wire [15:0]   mux_n_stat_to,
    input  wire [15:0]   mux_n_abort
);

    wire        f_wr, f_full, f_rd, f_empty;
    wire [32:0] f_wdata, f_rdata;
    wire        pkt_tgl, job_tgl, sent_tgl;
    reg  [159:0] tx_snap;            // tx_clk; see "tx counters -> status" below
    reg          sent_d;
    wire [47:0] j_dmac, j_smac;
    wire [31:0] j_dip, j_sip, j_gap;
    wire [15:0] j_dport, j_sport;

    // ---------------------------------------------------------------- sent -> clk_x1
    reg [2:0] s_sent;
    reg       sent_pulse;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin s_sent <= 3'd0; sent_pulse <= 1'b0; end
        else begin s_sent <= {s_sent[1:0], sent_tgl}; sent_pulse <= s_sent[2] ^ s_sent[1]; end

    // ---------------------------------------------------------------- read router
    wire         u_arvalid, u_arready, u_rvalid, u_rready, u_rlast;
    wire [29:0]  u_araddr;
    wire [7:0]   u_arlen;
    wire [255:0] u_rdata;
    wire [1:0]   u_rresp;
    wire         u_ctl = (u_araddr[29:8] == DESC_ADDR[29:8]) || (u_araddr[29:8] == PCFG_ADDR[29:8]);
    reg          r_ctl;                  // the read in flight goes to apb_ctl
    always @(posedge clk or negedge rst_n)
        if (!rst_n) r_ctl <= 1'b0;
        else if (u_arvalid && u_arready) r_ctl <= u_ctl;
    assign m_arvalid = u_arvalid && !u_ctl;
    assign c_arvalid = u_arvalid &&  u_ctl;
    assign m_araddr  = u_araddr;
    assign c_araddr  = u_araddr;
    assign m_arlen   = u_arlen;
    assign c_arlen   = u_arlen;
    assign u_arready = u_ctl ? c_arready : m_arready;
    assign u_rvalid  = r_ctl ? c_rvalid : m_rvalid;
    assign u_rdata   = r_ctl ? c_rdata  : m_rdata;
    assign u_rlast   = r_ctl ? c_rlast  : m_rlast;
    assign u_rresp   = r_ctl ? 2'b00    : m_rresp;
    assign m_rready  = u_rready && !r_ctl;
    assign c_rready  = u_rready &&  r_ctl;

    wire [31:0] u_wstrb_unused;
    udp_rd #(.DESC_ADDR(DESC_ADDR), .STAT_ADDR(STAT_ADDR), .PCFG_ADDR(PCFG_ADDR), .POLL_CYCLES(POLL_CYCLES)) u_rd (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .m_arvalid(u_arvalid), .m_arready(u_arready), .m_araddr(u_araddr), .m_arlen(u_arlen),
        .m_rvalid(u_rvalid), .m_rready(u_rready), .m_rdata(u_rdata), .m_rresp(u_rresp), .m_rlast(u_rlast),
        .m_awvalid(c_awvalid), .m_awready(c_awready), .m_awaddr(c_awaddr), .m_awlen(c_awlen),
        .m_wvalid(c_wvalid), .m_wready(c_wready), .m_wdata(c_wdata), .m_wstrb(u_wstrb_unused), .m_wlast(c_wlast),
        .m_bvalid(c_bvalid), .m_bready(c_bready), .m_bresp(2'b00),
        .f_wr(f_wr), .f_wdata(f_wdata), .f_full(f_full),
        .pkt_tgl(pkt_tgl), .job_tgl(job_tgl),
        .j_dmac(j_dmac), .j_smac(j_smac), .j_dip(j_dip), .j_sip(j_sip),
        .j_dport(j_dport), .j_sport(j_sport), .j_gap(j_gap),
        .sent_pulse(sent_pulse), .tx_snap(tx_snap),
        .n_jobs(n_jobs), .n_pkts(n_pkts), .n_axi_err(n_axi_err), .busy(busy));

    // ---------------------------------------------------------------- tx counters -> status
    // Snapshotted one cycle after each packet is on the wire, so the register
    // is still from that edge until the next packet of the stream: udp_rd
    // writes the status only after the last sent_pulse of a job has crossed
    // (>= 3 clk_x1 cycles later), when nothing here moves. The longest wait of
    // the CPU's packets goes in units of 128 cycles (1.024 us), saturated.
    wire [15:0] hold_us = (mux_max_hold_a[31:23] != 9'd0) ? 16'hFFFF : mux_max_hold_a[22:7];
    always @(posedge tx_clk or negedge tx_rst_n)
        if (!tx_rst_n) begin
            sent_d <= 1'b0; tx_snap <= 160'd0;
        end else begin
            sent_d <= sent_tgl;
            if (sent_tgl != sent_d)
                tx_snap <= {hold_us, n_bad_ctrl, mux_n_a, mux_n_abort, mux_n_stat_to, n_underrun, n_stat_to, n_sent};
        end

    async_fifo_simple #(.DATA_WIDTH(33), .ADDR_BITS(12)) u_fifo (
        .wr_clk(clk), .wr_rst_n(rst_n), .wr_en(f_wr), .wr_data(f_wdata), .wr_full(f_full), .wr_used(),
        .rd_clk(tx_clk), .rd_rst_n(tx_rst_n), .rd_en(f_rd), .rd_data(f_rdata), .rd_empty(f_empty), .rd_used());

    // ---------------------------------------------------------------- clk_x1 -> tx
    reg [2:0] s_job, s_pkt;
    reg       job_e, job_go, pkt_go;
    always @(posedge tx_clk or negedge tx_rst_n)
        if (!tx_rst_n) begin
            s_job <= 3'd0; s_pkt <= 3'd0; job_e <= 1'b0; job_go <= 1'b0; pkt_go <= 1'b0;
        end else begin
            s_job  <= {s_job[1:0], job_tgl};
            s_pkt  <= {s_pkt[1:0], pkt_tgl};
            job_e  <= s_job[2] ^ s_job[1];
            job_go <= job_e;                 // one more cycle: the held fields are long settled
            pkt_go <= s_pkt[2] ^ s_pkt[1];
        end

    udp_send u_send (
        .clk(tx_clk), .rst_n(tx_rst_n),
        .job_go(job_go), .dmac(j_dmac), .smac(j_smac), .dip(j_dip), .sip(j_sip),
        .dport(j_dport), .sport(j_sport), .gap(j_gap), .pkt_go(pkt_go),
        .f_rd(f_rd), .f_rdata(f_rdata), .f_empty(f_empty),
        .tx_valid(tx_valid), .tx_data(tx_data), .tx_last(tx_last), .tx_ready(tx_ready), .stat_valid(stat_valid),
        .sent_tgl(sent_tgl), .n_sent(n_sent), .n_stat_to(n_stat_to), .n_underrun(n_underrun),
        .n_bad_ctrl(n_bad_ctrl));

endmodule
