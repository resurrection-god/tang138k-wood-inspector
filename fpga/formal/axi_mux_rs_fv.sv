// Formal harness for axi_mux_rs (fpga/src/axi_mux.v): the two-entry register
// slice on every master-side W and R channel of axi_mux.
//
// What is proved, for any input pattern (no assumption on the upstream side):
//   * the output channel is AXI-legal: a valid that was not taken stays up the
//     next cycle with the same data;
//   * nothing is lost, duplicated or reordered: the n-th beat accepted at the
//     input is the n-th beat leaving the output, with its data (n chosen by
//     the solver - "chosen beat" scoreboard);
//   * the slice is full throughput: it takes a beat whenever it holds < 2.
// Read with read_slang: the harness looks inside the DUT by hierarchical name.

`timescale 1ns / 1ps

module axi_mux_rs_fv #(
    parameter integer W = 8
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         i_valid,
    input  wire [W-1:0] i_data,
    input  wire         o_ready
);
    wire         i_ready, o_valid;
    wire [W-1:0] o_data;

    axi_mux_rs #(.W(W)) dut (
        .clk(clk), .rst_n(rst_n),
        .i_valid(i_valid), .i_ready(i_ready), .i_data(i_data),
        .o_valid(o_valid), .o_ready(o_ready), .o_data(o_data)
    );

    // ---- reset: low in the first cycle, high afterwards ----------------------
    reg first = 1'b1;
    always @(posedge clk) first <= 1'b0;
    always @(*) if (first) assume (!rst_n); else assume (rst_n);

    wire push = i_valid && i_ready;
    wire pop  = o_valid && o_ready;

    // ---- chosen beat ----------------------------------------------------------
    // n: a free constant (a register with no reset and no load keeps whatever
    // value the solver starts it with). pc/qc count beats in and out.
    reg  [7:0]   n;
    always @(posedge clk) n <= n;
    reg  [7:0]   pc = 8'd0, qc = 8'd0;
    reg  [W-1:0] nd;                    // the chosen beat's data, once it went in
    wire [7:0]   occ = pc - qc;
    wire         n_in   = (n - qc) < occ;       // the chosen beat is inside now
    wire         n_head = n_in && (n == qc);

    always @(posedge clk) begin
        if (!first) begin
            pc <= pc + {7'd0, push};
            qc <= qc + {7'd0, pop};
            if (push && pc == n) nd <= i_data;
        end
    end

    // ---- properties -----------------------------------------------------------
    reg  [W-1:0] o_data_q;
    reg          stall_q = 1'b0;
    always @(posedge clk) begin
        stall_q  <= !first && o_valid && !o_ready;
        o_data_q <= o_data;
    end

    always @(posedge clk) if (!first && rst_n) begin
        // output channel AXI-legal
        if (stall_q) begin
            assert (o_valid);
            assert (o_data == o_data_q);
        end
        // scoreboard
        if (pop && qc == n) assert (o_data == nd);
        // full throughput
        assert (i_ready == (occ < 8'd2));
        assert (o_valid == (occ != 8'd0));
        // invariants for induction: the counters agree with the DUT, and the
        // chosen beat sits where the read pointer will find it
        assert (dut.cnt == occ[1:0]);
        assert (occ <= 8'd2);
        assert (dut.wp == (dut.rp ^ occ[0]));
        if (n_head)                    assert ((dut.rp ? dut.d1 : dut.d0) == nd);
        if (n_in && !n_head)           assert ((dut.rp ? dut.d0 : dut.d1) == nd);
    end

    // ---- reachability ----------------------------------------------------------
    always @(posedge clk) if (!first) begin
        cover (pop && qc == n && n == 8'd3 && stall_q);
        cover (occ == 8'd2 && push == 1'b0 && pop);
    end
endmodule
