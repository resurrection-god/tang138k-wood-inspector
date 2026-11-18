// ============================================================================
// Module      : npu_apb_wrapper
// Headless APB slave wrapper (0xF8000800..0xF8000FFF) hosting npu_core v3
// at 0xF8000900 on AHB_CLK (100 MHz). Zero display/LCD/HDMI resources used.
// ============================================================================

`timescale 1ns / 1ps

module npu_apb_wrapper (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [10:0] paddr,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready
);

    reg [8:0]  q_addr;
    reg [31:0] q_wdata;
    reg        q_write, q_armed, q_go, q_rdy;
    assign pready = q_rdy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_addr  <= 9'd0;
            q_wdata <= 32'd0;
            q_write <= 1'b0;
            q_armed <= 1'b0;
            q_go    <= 1'b0;
            q_rdy   <= 1'b0;
        end else begin
            q_go  <= 1'b0;
            q_rdy <= 1'b0;

            if (q_go) begin
                q_rdy <= 1'b1;
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

    wire is_npu_space = (q_addr[8:6] == 3'b001); // 0x100..0x1FF -> 0xF8000900..0xF80009FF
    wire npu_wr       = q_go &&  q_write && is_npu_space;
    wire npu_rd       = q_go && !q_write && is_npu_space;
    wire [31:0] npu_rdata;

    npu_core u_npu_core (
        .clk       (clk),
        .rst_n     (rst_n),
        .reg_wr    (npu_wr),
        .reg_rd    (npu_rd),
        .reg_addr  ({q_addr[5:0], 2'b00}),
        .reg_wdata (q_wdata),
        .reg_rdata (npu_rdata)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prdata <= 32'd0;
        end else if (q_go && !q_write) begin
            prdata <= is_npu_space ? npu_rdata : 32'h574F_4F44; // "WOOD" magic on 0x800
        end
    end

endmodule
