// ============================================================================
// Module      : build_id
// The bitstream's identity for the firmware (GigE Vision DeviceFpgaVersion,
// docs/protocol/GEV_PROTOCOL_DESIGN.md § 5.7): CRC-32 of the RTL sources listed in
// scanner.gprj (sorted by path, this file excluded), written here by build_fpga.tcl
// before synthesis. The same sources give the same id, so it follows the RTL in git.
// Read by the CPU at flash_ctl's command port, offset 0x014.
// ============================================================================

`timescale 1ns / 1ps

module build_id (
    output wire [31:0] id
);
    assign id = 32'h8A9CABDA;
endmodule
