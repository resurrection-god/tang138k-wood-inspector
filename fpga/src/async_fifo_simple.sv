`timescale 1ns / 1ps

module async_fifo_simple #(
	parameter DATA_WIDTH = 8,
	parameter ADDR_BITS = 12
)(
	input wr_clk,
	input wr_rst_n,
	input wr_en,
	input [DATA_WIDTH-1:0] wr_data,
	output wr_full,
	output [ADDR_BITS:0] wr_used,

	input rd_clk,
	input rd_rst_n,
	input rd_en,
	output reg [DATA_WIDTH-1:0] rd_data,
	output rd_empty,
	output [ADDR_BITS:0] rd_used
);

	localparam DEPTH = (1 << ADDR_BITS);

	(* ram_style = "block", syn_ramstyle = "block_ram" *) reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

	reg [ADDR_BITS:0] wr_bin;
	reg [ADDR_BITS:0] wr_gray;
	reg [ADDR_BITS:0] rd_bin;
	reg [ADDR_BITS:0] rd_gray;

	(* async_reg = "true" *) reg [ADDR_BITS:0] rd_gray_wr1;
	(* async_reg = "true" *) reg [ADDR_BITS:0] rd_gray_wr2;
	(* async_reg = "true" *) reg [ADDR_BITS:0] wr_gray_rd1;
	(* async_reg = "true" *) reg [ADDR_BITS:0] wr_gray_rd2;

	wire [ADDR_BITS:0] wr_bin_inc = wr_bin + {{ADDR_BITS{1'b0}}, 1'b1};
	wire [ADDR_BITS:0] rd_bin_inc = rd_bin + {{ADDR_BITS{1'b0}}, 1'b1};
	wire [ADDR_BITS:0] wr_gray_inc = bin_to_gray(wr_bin_inc);
	wire [ADDR_BITS:0] rd_gray_inc = bin_to_gray(rd_bin_inc);
	wire [ADDR_BITS:0] rd_bin_in_wr = gray_to_bin(rd_gray_wr2);
	wire [ADDR_BITS:0] wr_bin_in_rd = gray_to_bin(wr_gray_rd2);

	// The full flag comes from a flip-flop (16.10.2026, ERRORS 6.15). Before, the
	// path wr_bin -> +1 -> gray -> compare -> write enable of every BSRAM of the
	// memory was the worst mipi_byte_clk path of the pixel FIFO (+0.306 ns on
	// 4BC4E342, +0.076 ns with 16 more BSRAM elsewhere). The flag is computed one
	// cycle ahead from the values both registers will have then: wr_bin after this
	// cycle's write and rd_gray_wr1, which becomes rd_gray_wr2 - so it is the same
	// function of the same values as the combinational one, cycle for cycle.
	reg  wr_full_q;
	wire wr_do       = wr_en && !wr_full_q;
	wire [ADDR_BITS:0] wr_bin_nx      = wr_do ? wr_bin_inc : wr_bin;
	wire [ADDR_BITS:0] wr_gray_inc_nx = bin_to_gray(wr_bin_nx + {{ADDR_BITS{1'b0}}, 1'b1});
	assign wr_full = wr_full_q;
	assign rd_empty = (rd_gray == wr_gray_rd2);
	assign wr_used = wr_bin - rd_bin_in_wr;
	assign rd_used = wr_bin_in_rd - rd_bin;

	function [ADDR_BITS:0] bin_to_gray;
		input [ADDR_BITS:0] bin_value;
		begin
			bin_to_gray = (bin_value >> 1) ^ bin_value;
		end
	endfunction

	function [ADDR_BITS:0] gray_to_bin;
		input [ADDR_BITS:0] gray_value;
		integer i;
		begin
			gray_to_bin[ADDR_BITS] = gray_value[ADDR_BITS];
			for(i = ADDR_BITS - 1; i >= 0; i = i - 1)
				gray_to_bin[i] = gray_to_bin[i + 1] ^ gray_value[i];
		end
	endfunction

	always @(posedge wr_clk or negedge wr_rst_n) begin
		if(!wr_rst_n) begin
			wr_bin <= {ADDR_BITS+1{1'b0}};
			wr_gray <= {ADDR_BITS+1{1'b0}};
			wr_full_q <= 1'b0;
		end else begin
			if(wr_do) begin
				mem[wr_bin[ADDR_BITS-1:0]] <= wr_data;
				wr_bin <= wr_bin_inc;
				wr_gray <= wr_gray_inc;
			end
			wr_full_q <= (wr_gray_inc_nx == {~rd_gray_wr1[ADDR_BITS:ADDR_BITS-1], rd_gray_wr1[ADDR_BITS-2:0]});
		end
	end

	// Read side: synchronous reset (30.09.2026). rd_rst_n arrives already
	// synchronised to rd_clk (rst_sync_lane in cam_rx.v), and rd_clk
	// (LANE4_CLK) never stops, so the only difference from an asynchronous reset
	// is the moment it asserts, by one cycle. In exchange the rd_rst_n net stops
	// being a "clock" net: the asynchronous reset took a global LW network
	// (u_cam0/u_rx/rst_sync_lane[1] in the PnR report, LW 8/8; ERRORS 6.12). The
	// write side stays asynchronous: the camera byte clock may stop.
	//
	// rd_data keeps its reset: cam_channel in ST_ROW_SYNC looks at its bits
	// 32/33 (first word of a row / frame) directly, and a word left over from
	// earlier operation would open a row with a garbage index.
	always @(posedge rd_clk) begin
		if(!rd_rst_n) begin
			rd_bin <= {ADDR_BITS+1{1'b0}};
			rd_gray <= {ADDR_BITS+1{1'b0}};
			rd_data <= {DATA_WIDTH{1'b0}};
		end else if(rd_en && !rd_empty) begin
			rd_data <= mem[rd_bin[ADDR_BITS-1:0]];
			rd_bin <= rd_bin_inc;
			rd_gray <= rd_gray_inc;
		end
	end

	always @(posedge wr_clk or negedge wr_rst_n) begin
		if(!wr_rst_n) begin
			rd_gray_wr1 <= {ADDR_BITS+1{1'b0}};
			rd_gray_wr2 <= {ADDR_BITS+1{1'b0}};
		end else begin
			rd_gray_wr1 <= rd_gray;
			rd_gray_wr2 <= rd_gray_wr1;
		end
	end

	always @(posedge rd_clk) begin   // synchronous reset, see above
		if(!rd_rst_n) begin
			wr_gray_rd1 <= {ADDR_BITS+1{1'b0}};
			wr_gray_rd2 <= {ADDR_BITS+1{1'b0}};
		end else begin
			wr_gray_rd1 <= wr_gray;
			wr_gray_rd2 <= wr_gray_rd1;
		end
	end

endmodule
