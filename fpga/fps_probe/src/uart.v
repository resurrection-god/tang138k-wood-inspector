// Простой UART 8N1 для fps_probe: передатчик и приёмник. DIV — тактов на бит.
`timescale 1ns / 1ps

module uart_tx #(parameter integer DIV = 434) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       go,
    input  wire [7:0] data,
    output reg        busy,
    output reg        tx
);
    reg [15:0] cnt;
    reg [3:0]  bitn;
    reg [9:0]  sh;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; tx <= 1'b1; cnt <= 16'd0; bitn <= 4'd0; sh <= 10'h3FF;
        end else if (!busy) begin
            tx <= 1'b1;
            if (go) begin
                sh <= {1'b1, data, 1'b0};      // стоп, данные младшим вперёд, старт
                busy <= 1'b1; cnt <= 16'd0; bitn <= 4'd0;
            end
        end else begin
            tx <= sh[0];
            if (cnt == DIV - 1) begin
                cnt <= 16'd0;
                sh  <= {1'b1, sh[9:1]};
                if (bitn == 4'd9) busy <= 1'b0;
                bitn <= bitn + 4'd1;
            end else begin
                cnt <= cnt + 16'd1;
            end
        end
    end
endmodule

module uart_rx #(parameter integer DIV = 434) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg        valid,
    output reg  [7:0] data
);
    reg [2:0]  s;
    reg        active;
    reg [15:0] cnt;
    reg [3:0]  bitn;
    reg [7:0]  sh;
    always @(posedge clk) s <= {s[1:0], rx};
    wire rxs = s[2];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid <= 1'b0; data <= 8'd0; active <= 1'b0; cnt <= 16'd0; bitn <= 4'd0; sh <= 8'd0;
        end else begin
            valid <= 1'b0;
            if (!active) begin
                if (!rxs) begin active <= 1'b1; cnt <= DIV / 2; bitn <= 4'd0; end
            end else if (cnt == DIV - 1) begin
                cnt <= 16'd0;
                if (bitn == 4'd0) begin
                    if (rxs) active <= 1'b0;             // ложный старт
                    bitn <= 4'd1;
                end else if (bitn <= 4'd8) begin
                    sh <= {rxs, sh[7:1]};
                    bitn <= bitn + 4'd1;
                end else begin
                    active <= 1'b0;
                    if (rxs) begin valid <= 1'b1; data <= sh; end   // стоп-бит на месте
                end
            end else begin
                cnt <= cnt + 16'd1;
            end
        end
    end
endmodule
