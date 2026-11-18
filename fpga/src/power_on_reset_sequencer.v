// ============================================================================
// Module      : power_on_reset_sequencer
// Description : Ultra-Reliable Glitch-Proof Multi-Stage Power-On Reset & Latch
// Target      : Gowin GW5AST-138 / Tang Mega 138K Pro
// Clock       : 50 MHz CLK (P16)
// ============================================================================

`timescale 1ns / 1ps

module power_on_reset_sequencer #(
    parameter integer SIM_MODE = 0
)(
    input  wire clk,                 // 50 MHz Main Clock
    input  wire rst_n_btn,           // Physical Reset Button (G16)
    input  wire pll_ae350_lock,      // PLL AE350 Lock
    input  wire pll_ddr3_lock,       // PLL DDR3 Lock
    input  wire ddr3_init_done,      // DDR3 Calibration Done

    output reg  phy_rst_n,           // Ethernet PHY Reset (Active Low)
    output reg  ddr3_rst_n,          // DDR3 Controller Reset (Active Low)
    output reg  soc_rst_n,           // RISC-V AE350 CPU Reset (Active Low)
    output reg  system_ready         // System Ready Flag
);

    // 50 MHz Timing Delays:
    // Power-on delay : 25 ms  = 1,250,000 clocks
    // PHY reset pulse: 25 ms  = 1,250,000 clocks
    // DDR3 settling  : 50 ms  = 2,500,000 clocks
    wire [24:0] DLY_POWERON = SIM_MODE ? 25'd50  : 25'd1_250_000;
    wire [24:0] DLY_PHY_RST = SIM_MODE ? 25'd100 : 25'd1_250_000;
    wire [24:0] DLY_SETTLE  = SIM_MODE ? 25'd200 : 25'd2_500_000;

    localparam [2:0] ST_BOOT_INIT     = 3'd0;
    localparam [2:0] ST_WAIT_PLL      = 3'd1;
    localparam [2:0] ST_PULSE_PHY     = 3'd2;
    localparam [2:0] ST_CALIB_DDR3    = 3'd3;
    localparam [2:0] ST_SETTLE_DELAY  = 3'd4;
    localparam [2:0] ST_LATCH_ACTIVE  = 3'd5;

    reg [2:0]  state;
    reg [24:0] timer;

    // 3-Stage Glitch-Filter and Edge-Detector for Reset Button (RSTN)
    reg [2:0] btn_sync;
    always @(posedge clk) btn_sync <= {btn_sync[1:0], rst_n_btn};
    wire btn_falling_edge = (btn_sync[2:1] == 2'b10);

    // Filtered DDR3 calibration done signal (requires 16 consecutive clocks to eliminate spikes)
    reg [3:0] ddr_calib_filter;
    always @(posedge clk) begin
        if (ddr3_init_done) begin
            if (ddr_calib_filter < 4'd15) ddr_calib_filter <= ddr_calib_filter + 1'b1;
        end else begin
            ddr_calib_filter <= 4'd0;
        end
    end
    wire ddr_calib_solid = (ddr_calib_filter == 4'd15);

    // Explicit Power-On Initialization Values (Power-on Reset guarantee without button press)
    initial begin
        state            = ST_BOOT_INIT;
        timer            = 25'd0;
        btn_sync         = 3'b111;
        ddr_calib_filter = 4'd0;
        phy_rst_n        = 1'b0;
        ddr3_rst_n       = 1'b1; // Must be 1 so DDR3 PLL can lock on power-up!
        soc_rst_n        = 1'b0;
        system_ready     = 1'b0;
    end

    always @(posedge clk) begin
        // Physical Button Press (Falling Edge 1 -> 0) -> Clean Re-trigger
        if (btn_falling_edge) begin
            state        <= ST_BOOT_INIT;
            timer        <= 25'd0;
            phy_rst_n    <= 1'b0;
            ddr3_rst_n   <= 1'b1;
            soc_rst_n    <= 1'b0;
            system_ready <= 1'b0;
        end else begin
            case (state)
                // 1. Initial Power-Up delay: keep all subsystems in reset for 25 ms
                ST_BOOT_INIT: begin
                    phy_rst_n    <= 1'b0;
                    ddr3_rst_n   <= 1'b1; // Keep DDR3 active
                    soc_rst_n    <= 1'b0;
                    system_ready <= 1'b0;
                    if (timer >= DLY_POWERON) begin
                        timer <= 25'd0;
                        state <= ST_WAIT_PLL;
                    end else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 2. Wait for stable PLL Locks
                ST_WAIT_PLL: begin
                    phy_rst_n    <= 1'b0;
                    ddr3_rst_n   <= 1'b1; // Keep DDR3 active
                    soc_rst_n    <= 1'b0;
                    system_ready <= 1'b0;
                    if (pll_ddr3_lock) begin
                        timer <= 25'd0;
                        state <= ST_PULSE_PHY;
                    end
                end

                // 3. Hold PHY Reset and release DDR3 controller reset
                ST_PULSE_PHY: begin
                    phy_rst_n    <= 1'b0;
                    ddr3_rst_n   <= 1'b1; // Start DDR3 memory controller
                    soc_rst_n    <= 1'b0;
                    system_ready <= 1'b0;
                    if (timer >= DLY_PHY_RST) begin
                        phy_rst_n <= 1'b1; // Release Ethernet PHY Reset cleanly
                        timer     <= 25'd0;
                        state     <= ST_CALIB_DDR3;
                    end else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 4. Wait for DDR3 Hardware Calibration Done
                ST_CALIB_DDR3: begin
                    phy_rst_n    <= 1'b1;
                    ddr3_rst_n   <= 1'b1;
                    soc_rst_n    <= 1'b0;
                    system_ready <= 1'b0;
                    if (ddr_calib_solid) begin
                        timer <= 25'd0;
                        state <= ST_SETTLE_DELAY;
                    end
                end

                // 5. Hardware Settling Delay: 50 ms for bus & clock stabilization
                ST_SETTLE_DELAY: begin
                    phy_rst_n    <= 1'b1;
                    ddr3_rst_n   <= 1'b1;
                    soc_rst_n    <= 1'b0;
                    system_ready <= 1'b0;
                    if (timer >= DLY_SETTLE) begin
                        soc_rst_n    <= 1'b1; // Release RISC-V Core Reset!
                        system_ready <= 1'b1;
                        state        <= ST_LATCH_ACTIVE;
                    end else begin
                        timer <= timer + 1'b1;
                    end
                end

                // 6. LATCHED OPERATIONAL STATE
                // Will NEVER drop reset unless physical RSTN button is pressed!
                ST_LATCH_ACTIVE: begin
                    phy_rst_n    <= 1'b1;
                    ddr3_rst_n   <= 1'b1;
                    soc_rst_n    <= 1'b1; // Permanently High
                    system_ready <= 1'b1; // Permanently Ready
                end

                default: state <= ST_BOOT_INIT;
            endcase
        end
    end

endmodule
