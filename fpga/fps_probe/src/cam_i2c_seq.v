// ============================================================================
// cam_i2c_seq - I2C master and VEYE camera setup sequence for fps_probe.
//
// The protocol is the firmware's (firmware/src/app/camera.c, formerly
// demo/I2Ccam.c):
//   write   : START, 0x3B<<1, reg[15:8], reg[7:0], 4 data bytes MSB first,
//             STOP;
//   read    : START, 0x3B<<1, reg[15:8], reg[7:0], repeated START,
//             (0x3B<<1)|1, 4 bytes (ACK, ACK, ACK, NACK), STOP.
// SCL 100 kHz, clock stretching is supported (the VEYE module is a
// microcontroller, it is allowed to hold SCL).
//
// Register write order as in camera_load_default_config(): stop, a 50 ms pause
// (while running some registers are silently not written, docs/HARDWARE.md § 8),
// format and lane count, width, offsets, HEIGHT LAST (the VEYE register map
// requires it), frame rate, exposure, start.
//
// go_read - read only (writes and pauses are skipped), no restart.
// ============================================================================
`timescale 1ns / 1ps

module cam_i2c_seq #(
    parameter integer CLK_HZ = 50_000_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        go_apply,
    input  wire        go_read,
    input  wire [31:0] roi_w,
    input  wire [31:0] roi_h,
    input  wire [31:0] fps,          // 0.01 Hz, register 0x0814
    input  wire [31:0] exp_us,
    output reg         scl_oe,       // 1 - pull SCL low
    output reg         sda_oe,
    input  wire        scl_in,
    input  wire        sda_in,
    output wire        busy,
    output reg         running,      // capture started (0x0400 = 1 written)
    output reg  [7:0]  step,
    output reg  [15:0] nacks,
    output reg  [31:0] rb_id,
    output reg  [31:0] rb_w,
    output reg  [31:0] rb_h,
    output reg  [31:0] rb_fps,
    output reg  [31:0] rb_maxfps,
    output reg  [31:0] rb_fps_run,
    output reg  [31:0] rb_maxfps_run
);

    localparam integer QDIV = CLK_HZ / 400_000;     // quarter of the 100 kHz period
    localparam integer MS   = CLK_HZ / 1000;
    localparam [7:0]   ADDR_W = 8'h76;              // 0x3B << 1
    localparam [7:0]   ADDR_R = 8'h77;

    // ---------------------------------------------------------- sequence
    localparam [1:0] OP_END = 2'd0, OP_WR = 2'd1, OP_RD = 2'd2, OP_DLY = 2'd3;
    reg [1:0]  s_op;
    reg [15:0] s_reg;
    reg [31:0] s_val;
    reg [2:0]  s_dst;
    always @(*) begin
        s_op = OP_END; s_reg = 16'd0; s_val = 32'd0; s_dst = 3'd0;
        case (step)
            8'd0:  begin s_op = OP_RD;  s_reg = 16'h0000; s_dst = 3'd0; end   // id "VEYE"
            8'd1:  begin s_op = OP_WR;  s_reg = 16'h0400; s_val = 32'd0; end  // stop
            8'd2:  begin s_op = OP_DLY; s_val = 32'd50; end
            8'd3:  begin s_op = OP_WR;  s_reg = 16'h0404; s_val = 32'd0; end  // stream
            8'd4:  begin s_op = OP_WR;  s_reg = 16'h0800; s_val = 32'd0; end  // no test pattern
            8'd5:  begin s_op = OP_WR;  s_reg = 16'h0804; s_val = 32'd0; end  // Mono8
            8'd6:  begin s_op = OP_WR;  s_reg = 16'h083C; s_val = 32'd2; end  // 2 lanes
            8'd7:  begin s_op = OP_WR;  s_reg = 16'h0818; s_val = roi_w; end
            8'd8:  begin s_op = OP_WR;  s_reg = 16'h0820; s_val = 32'd0; end
            8'd9:  begin s_op = OP_WR;  s_reg = 16'h0824; s_val = 32'd0; end
            8'd10: begin s_op = OP_WR;  s_reg = 16'h081C; s_val = roi_h; end  // height last
            8'd11: begin s_op = OP_WR;  s_reg = 16'h0814; s_val = fps;   end
            8'd12: begin s_op = OP_WR;  s_reg = 16'h0C10; s_val = exp_us; end
            8'd13: begin s_op = OP_DLY; s_val = 32'd10; end
            8'd14: begin s_op = OP_RD;  s_reg = 16'h0818; s_dst = 3'd1; end
            8'd15: begin s_op = OP_RD;  s_reg = 16'h081C; s_dst = 3'd2; end
            8'd16: begin s_op = OP_RD;  s_reg = 16'h0814; s_dst = 3'd3; end
            8'd17: begin s_op = OP_RD;  s_reg = 16'h0838; s_dst = 3'd4; end
            8'd18: begin s_op = OP_WR;  s_reg = 16'h0400; s_val = 32'd1; end  // start
            8'd19: begin s_op = OP_DLY; s_val = 32'd200; end
            8'd20: begin s_op = OP_RD;  s_reg = 16'h0814; s_dst = 3'd5; end
            8'd21: begin s_op = OP_RD;  s_reg = 16'h0838; s_dst = 3'd6; end
            default: s_op = OP_END;
        endcase
    end

    // ------------------------------------------------------- I2C micro-ops
    localparam [2:0] U_START = 3'd0, U_STOP = 3'd1, U_WRITE = 3'd2,
                     U_RACK = 3'd3, U_RNACK = 3'd4, U_DONE = 3'd7;

    reg        x_rd;
    reg [15:0] x_reg;
    reg [31:0] x_val;
    reg [2:0]  x_dst;
    reg [3:0]  ui;

    reg [2:0] ut;
    reg [7:0] ubyte;
    always @(*) begin
        ut = U_DONE; ubyte = 8'h00;
        if (!x_rd) begin
            case (ui)
                4'd0: ut = U_START;
                4'd1: begin ut = U_WRITE; ubyte = ADDR_W; end
                4'd2: begin ut = U_WRITE; ubyte = x_reg[15:8]; end
                4'd3: begin ut = U_WRITE; ubyte = x_reg[7:0]; end
                4'd4: begin ut = U_WRITE; ubyte = x_val[31:24]; end
                4'd5: begin ut = U_WRITE; ubyte = x_val[23:16]; end
                4'd6: begin ut = U_WRITE; ubyte = x_val[15:8]; end
                4'd7: begin ut = U_WRITE; ubyte = x_val[7:0]; end
                4'd8: ut = U_STOP;
                default: ut = U_DONE;
            endcase
        end else begin
            case (ui)
                4'd0:  ut = U_START;
                4'd1:  begin ut = U_WRITE; ubyte = ADDR_W; end
                4'd2:  begin ut = U_WRITE; ubyte = x_reg[15:8]; end
                4'd3:  begin ut = U_WRITE; ubyte = x_reg[7:0]; end
                4'd4:  ut = U_START;                       // repeated START
                4'd5:  begin ut = U_WRITE; ubyte = ADDR_R; end
                4'd6, 4'd7, 4'd8: ut = U_RACK;
                4'd9:  ut = U_RNACK;
                4'd10: ut = U_STOP;
                default: ut = U_DONE;
            endcase
        end
    end

    // Bus inputs go through a synchroniser.
    reg [1:0] scl_s, sda_s;
    always @(posedge clk) begin scl_s <= {scl_s[0], scl_in}; sda_s <= {sda_s[0], sda_in}; end
    wire scl_hi = scl_s[1];
    wire sda_hi = sda_s[1];

    // ---------------------------------------------------------------- FSM
    localparam [2:0] S_IDLE = 3'd0, S_NEXT = 3'd1, S_GAP = 3'd2, S_XFER = 3'd3, S_DLY = 3'd4;
    reg [2:0]  st;
    reg        ro;             // read only
    reg [31:0] cnt;            // pauses
    reg        ustart;
    reg [1:0]  q;
    reg [3:0]  b;
    reg [15:0] qc;
    reg [7:0]  sh;
    reg [31:0] rbuf;

    assign busy = (st != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; ro <= 1'b0; step <= 8'd0; cnt <= 32'd0;
            scl_oe <= 1'b0; sda_oe <= 1'b0; running <= 1'b0; nacks <= 16'd0;
            x_rd <= 1'b0; x_reg <= 16'd0; x_val <= 32'd0; x_dst <= 3'd0;
            ui <= 4'd0; ustart <= 1'b0; q <= 2'd0; b <= 4'd0; qc <= 16'd0; sh <= 8'd0; rbuf <= 32'd0;
            rb_id <= 0; rb_w <= 0; rb_h <= 0; rb_fps <= 0; rb_maxfps <= 0; rb_fps_run <= 0; rb_maxfps_run <= 0;
        end else begin
            case (st)
            S_IDLE: begin
                scl_oe <= 1'b0; sda_oe <= 1'b0;
                if (go_apply)     begin ro <= 1'b0; step <= 8'd0; st <= S_NEXT; end
                else if (go_read) begin ro <= 1'b1; step <= 8'd0; st <= S_NEXT; end
            end

            S_NEXT: begin
                if (s_op == OP_END) begin
                    st <= S_IDLE;
                end else if (ro && (s_op != OP_RD)) begin
                    step <= step + 8'd1;                    // read: skip writes and pauses
                end else if (s_op == OP_DLY) begin
                    cnt <= s_val * MS;
                    st  <= S_DLY;
                end else begin
                    if (!ro && step == 8'd1) running <= 1'b0;   // the camera is about to stop
                    x_rd <= (s_op == OP_RD); x_reg <= s_reg; x_val <= s_val; x_dst <= s_dst;
                    cnt <= 2 * MS;                              // pause between transactions
                    st  <= S_GAP;
                end
            end

            S_DLY: begin
                if (cnt == 32'd0) begin step <= step + 8'd1; st <= S_NEXT; end
                else cnt <= cnt - 32'd1;
            end

            S_GAP: begin
                if (cnt == 32'd0) begin ui <= 4'd0; ustart <= 1'b1; st <= S_XFER; end
                else cnt <= cnt - 32'd1;
            end

            S_XFER: begin
                if (ustart) begin
                    ustart <= 1'b0; sh <= ubyte; b <= 4'd0; q <= 2'd0; qc <= 16'd0;
                    if (ut == U_DONE) begin
                        if (x_rd) begin
                            case (x_dst)
                                3'd0: rb_id <= rbuf;
                                3'd1: rb_w <= rbuf;
                                3'd2: rb_h <= rbuf;
                                3'd3: rb_fps <= rbuf;
                                3'd4: rb_maxfps <= rbuf;
                                3'd5: rb_fps_run <= rbuf;
                                3'd6: rb_maxfps_run <= rbuf;
                                default: ;
                            endcase
                        end
                        if (!ro && step == 8'd18) running <= 1'b1;   // start written
                        step <= step + 8'd1;
                        st   <= S_NEXT;
                    end
                end else begin
                    // A quarter's action happens on its first cycle.
                    if (qc == 16'd0) begin
                        case (ut)
                            U_START: case (q)
                                2'd0: sda_oe <= 1'b0;
                                2'd1: scl_oe <= 1'b0;
                                2'd2: sda_oe <= 1'b1;             // START: SDA low while SCL is high
                                default: scl_oe <= 1'b1;
                            endcase
                            U_STOP: case (q)
                                2'd0: begin scl_oe <= 1'b1; sda_oe <= 1'b1; end
                                2'd1: scl_oe <= 1'b0;
                                2'd2: sda_oe <= 1'b0;             // STOP: SDA high while SCL is high
                                default: ;
                            endcase
                            U_WRITE: case (q)
                                2'd0: begin scl_oe <= 1'b1; sda_oe <= (b < 4'd8) ? ~sh[7] : 1'b0; end
                                2'd1: scl_oe <= 1'b0;
                                2'd2: if (b == 4'd8 && sda_hi) nacks <= nacks + 16'd1;
                                default: scl_oe <= 1'b1;
                            endcase
                            default: case (q)                     // U_RACK / U_RNACK
                                2'd0: begin scl_oe <= 1'b1; sda_oe <= (b < 4'd8) ? 1'b0 : (ut == U_RACK); end
                                2'd1: scl_oe <= 1'b0;
                                2'd2: if (b < 4'd8) rbuf <= {rbuf[30:0], sda_hi};
                                default: scl_oe <= 1'b1;
                            endcase
                        endcase
                    end

                    // Clock stretching: in quarter 1 wait until the slave releases SCL.
                    if (q == 2'd1 && qc != 16'd0 && !scl_hi) begin
                        qc <= qc;
                    end else if (qc == QDIV - 1) begin
                        qc <= 16'd0;
                        if (q == 2'd3) begin
                            q <= 2'd0;
                            if ((ut == U_WRITE || ut == U_RACK || ut == U_RNACK) && b != 4'd8) begin
                                b <= b + 4'd1;
                                if (ut == U_WRITE) sh <= {sh[6:0], 1'b0};
                            end else begin
                                ui <= ui + 4'd1;
                                ustart <= 1'b1;
                            end
                        end else begin
                            q <= q + 2'd1;
                        end
                    end else begin
                        qc <= qc + 16'd1;
                    end
                end
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
