// ============================================================================
// Module      : apb_ctl_bridge
// APB slave on the SoC clock (AHB_CLK) -> one register access at a time in the
// clk_x1 domain, for the FPGA control registers that replace the DDR3 mailboxes
// (docs/RTL_RULES.md 4.2 and 7.1; design: "Регистры управления ПЛИС на APB",
// 30.10.2026, revises DECISIONS R-13).
//
//   * The APB side is fully registered, as flash_ctl (ERRORS 5.12): PSEL/PADDR/
//     PWDATA reach only the q_* flops. Setup captures, access holds PREADY low
//     until the clk_x1 side has done the access.
//   * Crossing: q_* stay stable from the request toggle until the acknowledge
//     toggle comes back (toggle + held data, RTL_RULES 7.1). Each toggle passes
//     two flops. A read returns c_rdata, captured by the clk_x1 side in the
//     cycle it acknowledges and stable until the next request.
//   * One access costs about 2 + 2 + 2 clocks of each side, ~60 ns: settings
//     are written rarely, a 2047-word mm band takes ~0.15 ms.
//   * The clk_x1 side gives c_go for one clock with c_we / c_addr / c_wdata and
//     expects c_rdata in the NEXT clock (one-cycle registered read).
// ============================================================================

`timescale 1ns / 1ps

module apb_ctl_bridge #(
    parameter integer AW = 10                // word address bits behind the bridge
)(
    // ---- APB (pclk = AHB_CLK) ----------------------------------------------
    input  wire          pclk,
    input  wire          prst_n,
    input  wire          psel,               // already decoded for this block
    input  wire          penable,
    input  wire          pwrite,
    input  wire [AW+1:0] paddr,              // byte address inside the block
    input  wire [31:0]   pwdata,
    output reg  [31:0]   prdata,
    output wire          pready,

    // ---- register side (clk = clk_x1) ----------------------------------------
    input  wire          clk,
    input  wire          rst_n,
    output reg           c_go,
    output reg           c_we,
    output reg  [AW-1:0] c_addr,
    output reg  [31:0]   c_wdata,
    input  wire [31:0]   c_rdata
);

    // ------------------------------------------------------------- APB side
    reg [AW-1:0] q_addr;
    reg [31:0]   q_wdata;
    reg          q_write, q_armed, q_busy, q_rdy, req_tgl;
    reg [1:0]    ack_s;                      // ack toggle, synchronised to pclk
    reg          ack_seen;
    reg [31:0]   x_rdata;                    // clk_x1 data, stable while ack is seen

    assign pready = q_rdy;

    always @(posedge pclk or negedge prst_n) begin
        if (!prst_n) begin
            q_addr <= {AW{1'b0}}; q_wdata <= 32'd0; q_write <= 1'b0;
            q_armed <= 1'b0; q_busy <= 1'b0; q_rdy <= 1'b0; req_tgl <= 1'b0;
            ack_s <= 2'b00; ack_seen <= 1'b0; prdata <= 32'd0;
        end else begin
            ack_s <= {ack_s[0], ack_tgl};
            q_rdy <= 1'b0;
            if (psel && !penable && !q_busy) begin
                q_addr <= paddr[AW+1:2]; q_wdata <= pwdata; q_write <= pwrite; q_armed <= 1'b1;
            end else if (q_armed && psel && penable) begin
                q_armed <= 1'b0; q_busy <= 1'b1;
                req_tgl <= ~req_tgl;
            end
            if (q_busy && (ack_s[1] != ack_seen)) begin
                ack_seen <= ack_s[1];
                q_busy   <= 1'b0;
                q_rdy    <= 1'b1;                // PREADY one clock: the access phase ends
                prdata   <= x_rdata;
            end
        end
    end

    // ---------------------------------------------------------- clk_x1 side
    reg [1:0] req_s;
    reg       req_seen, ack_tgl, rd_pend;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_s <= 2'b00; req_seen <= 1'b0; ack_tgl <= 1'b0; rd_pend <= 1'b0;
            c_go <= 1'b0; c_we <= 1'b0; c_addr <= {AW{1'b0}}; c_wdata <= 32'd0;
            x_rdata <= 32'd0;
        end else begin
            req_s <= {req_s[0], req_tgl};
            c_go  <= 1'b0;
            rd_pend <= 1'b0;
            if (req_s[1] != req_seen) begin
                // q_* have been stable since the toggle left the APB side
                req_seen <= req_s[1];
                c_go    <= 1'b1;
                c_we    <= q_write;
                c_addr  <= q_addr;
                c_wdata <= q_wdata;
            end
            if (c_go) rd_pend <= 1'b1;           // c_rdata is valid the clock after c_go
            if (rd_pend) begin
                x_rdata <= c_rdata;
                ack_tgl <= ~ack_tgl;
            end
        end
    end

endmodule
