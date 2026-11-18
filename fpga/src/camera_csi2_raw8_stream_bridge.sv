`timescale 1ns / 1ps

// Auto-phase RAW8 stream bridge.
//
// Ported from the working bring-up design
// sc132m_mipi_hdmi_pointsearch_720p60_overlay_aligned_debug/src, which is the
// reference that demonstrably renders the SC132M test pattern. Keep this file in
// step with that one; the only additions here are the diagnostic counters used by
// the DDR3 streamer's VID0 telemetry.
//
// It bypasses the generated Gowin CSI2 packet decoder: it watches the paired
// hard-DPHY lane words directly, finds RAW8 long-packet headers whose word count
// equals the expected line length, and emits a CSI-like payload stream.
//
// BYTE LAYOUT - this module assumes the D-PHY runs in 8-BIT mode
// (hs_8bit_mode = 1'b1, rx_clk_1x = 1'b1). In that mode each lane delivers ONE
// byte per byte_clk in hsrxd[7:0], so a lane pair carries two consecutive CSI-2
// bytes: lane0 = byte N, lane1 = byte N+1. The 4-byte CSI-2 long packet header is
// therefore split across TWO pair cycles:
//
//     pair k   : lane0 = DI       lane1 = WC_LSB
//     pair k+1 : lane0 = WC_MSB   lane1 = ECC
//
// which is why the word count is completed one cycle after the DI match. Do not
// "fix" this to read WC_MSB from the same pair: that is only correct for 16-bit
// mode (hs_8bit_mode = 1'b0), and configuring 16-bit mode is what previously left
// the receiver unable to lock at all.
module camera_csi2_raw8_stream_bridge #(
    parameter [15:0] LINE_BYTES  = 16'd1080,
    parameter [15:0] FRAME_LINES = 16'd1024,

    // Validate the packet header's ECC byte before accepting it.
    //
    // Without this a header is accepted on nothing but "DT looks like 0x2A and
    // the word count equals LINE_BYTES", which noise satisfies constantly - the
    // telemetry counts header candidates rejected on word count in the BILLIONS.
    // Any candidate that happens to carry the right word count is taken as a
    // real line, which is what makes frame_line_count drift away from the
    // sensor's own line numbering (ERRORS 2.3).
    //
    // The ECC byte is already on the wire and already being read; it was simply
    // never checked. Set to 0 only to fall back to the old behaviour if a build
    // stops accepting lines entirely - that would mean the parity masks below
    // are wrong, and it shows up immediately as complete_lines going to zero.
    parameter        HEADER_ECC_CHECK = 1,

    // Re-anchor frame_line_count after this many byte_clk cycles with no lane
    // data. Set to 0 to disable.
    //
    // frame_line_count used to be a free-running counter: it advanced on every
    // completed line and wrapped at FRAME_LINES-1, with nothing tying it to a
    // real frame boundary. One missed or one false header and every row after it
    // is written one position out, permanently - and because the row's address
    // and its trailer both come from that same index, the corruption is
    // self-consistent and invisible to a placement check.
    //
    // The inter-frame gap is enormous and unmistakable next to the gap between
    // lines: ~85 ms against ~0.24 us at 10 fps. Anything in between re-anchors
    // the counter safely, so the drift can last at most one frame.
    parameter [15:0] IDLE_ANCHOR_BEATS = 16'd4096,

    // Where the row length comes from: 1 - the cfg_line_bytes input (changes at
    // run time), 0 - the LINE_BYTES parameter, the input is not read.
    //
    // The main project sets 0 (ERRORS former 31.53, outcome of 24.09.2026). With the
    // length as a signal the header decision is a full compare on the critical cycle,
    // and the camera 0 domain was left with +0.19 ns against ~0.83 with a constant; of
    // three P1 placements one passed, and that one did not capture with camera 0. With
    // a constant, synthesis folds the compare into literals, as before GEOM. The
    // working mode does not touch the width on the sensor anyway (docs/CONCEPT.md): the
    // row length on the wire is constant, at run time only the height changes - it stays
    // an input.
    //
    // fpga/fps_probe keeps 1: there the ROI width is what is being measured.
    parameter        RUNTIME_LINE_BYTES = 1,

    // Where the line count wraps: 1 - at the cfg_frame_lines input, 0 - at the
    // FRAME_LINES parameter, the input is not read (19.10.2026, ERRORS 2.11).
    //
    // The main project sets 0. The wrap only matters for a frame LONGER than the
    // wrap point: every frame starts at line 0 anyway, on frame_anchor (the strobe
    // fall) and on the idle gap (IDLE_ANCHOR_BEATS). With the ROI height as the
    // wrap, a sensor still sending a taller frame was cut into several "frames",
    // and the height had to cross into byte_clk - which was only safe while the
    // receiver sat in a watchdog reset, so a height change waited for one. With
    // FRAME_LINES the line index is the sensor's own row number up to the build's
    // height, and the writer frames slots by the ROI height in its own domain.
    //
    // fpga/fps_probe and tb_bridge keep 1.
    parameter        RUNTIME_FRAME_LINES = 1
)(
    input             byte_clk,
    input             rst_n,
    input             enable,

    // Geometry at run time, not as a build parameter.
    //
    // LINE_BYTES / FRAME_LINES above remain defaults: the writer feeds them as the
    // initial value, so the behaviour right after reset is exactly as before. The
    // working mode needs a narrow ROI, and changing it by rebuilding the bitstream
    // is not possible (docs/CONCEPT.md, docs/ARCHITECTURE.md § 3.3).
    //
    // An input read here (RUNTIME_* = 1) must change ONLY while the D-PHY receiver
    // is held in reset, when byte_clk is stopped: that is the whole domain
    // crossing. Changing it on the fly is not allowed - the row would tear in the
    // middle of a frame. The main project reads neither (both RUNTIME_* = 0).
    input      [15:0] cfg_line_bytes,
    input      [15:0] cfg_frame_lines,

    // One-cycle pulse: the camera strobe just fell, so the readout of a new
    // frame starts now. Re-anchors frame_line_count to 0 - see the note at
    // IDLE_ANCHOR_BEATS. Tie to 0 where there is no strobe.
    input             frame_anchor,

    input      [15:0] lane0_data,
    input      [15:0] lane1_data,
    input             lane0_valid,
    input             lane1_valid,

    output reg        stream_sp_en,
    output reg        stream_lp_av_en,
    output reg [5:0]  stream_dt,
    output reg [15:0] stream_wc,
    output reg [1:0]  stream_payload_dv,
    output reg [15:0] stream_payload,

    output reg        raw8_header_toggle,
    output reg        raw8_line_ok_toggle,
    output reg        raw8_bad_wc_toggle,
    output reg        frame_done_toggle,
    output reg        pixel_seen,
    output reg        overflow_seen,
    output reg [31:0] raw8_header_count,
    output reg [31:0] raw8_line_count,
    output reg [31:0] raw8_bad_wc_count,
    output reg [15:0] last_word_count,
    output reg [15:0] last_line_byte_count,
    output reg [15:0] last_payload_word,
    output reg        last_lane_swap,
    output reg        last_phase_high,

    // Index of the line currently being emitted, valid from the cycle the header
    // is accepted until the line ends. The DDR3 streamer tags the first FIFO word
    // of every line with this so a row can be written at its true position in the
    // slot instead of wherever the byte stream happens to be.
    output reg [15:0] line_index,

    // Instrumentation (not in the reference): word count of every RAW8 header
    // candidate, accepted or rejected. last_word_count only reports ACCEPTED
    // headers, so when the configured line length is wrong it stays silent and
    // there is no way to learn what the sensor is really sending.
    output reg [15:0] seen_wc_last,
    output reg [15:0] seen_wc_hits_expected,
    output reg [15:0] seen_wc_hits_alt
);

// Second line length to count candidates against, for comparison with LINE_BYTES.
localparam [15:0] ALT_LINE_BYTES = 16'd1280;

localparam [1:0] ST_SEARCH  = 2'd0;
localparam [1:0] ST_HEADER1 = 2'd1;
localparam [1:0] ST_PAYLOAD = 2'd2;
localparam [1:0] ST_CRC     = 2'd3;

localparam [5:0] CSI_DT_RAW8 = 6'h2A;

// The row length the header is compared with (see RUNTIME_LINE_BYTES).
wire [15:0] line_bytes = (RUNTIME_LINE_BYTES != 0) ? cfg_line_bytes : LINE_BYTES;

reg [1:0]  state;
reg [7:0]  wc_low;
reg        wc_low_ok;       // wc_low == line_bytes[7:0], computed one cycle earlier
reg        hdr_ok_q, hdr_bad_q, hit_exp_q, hit_alt_q;   // flags for the counters, one cycle later
reg [15:0] frame_last;      // wrap point - 1 (see RUNTIME_FRAME_LINES), in a register
reg        header_phase_high;
reg        header_lane_swap;
reg [15:0] word_count;
reg [15:0] line_byte_count;
reg [15:0] frame_line_count;
reg [7:0]  hdr_di;          // full Data Identifier byte, needed for the ECC
reg [15:0] idle_beats;      // byte_clk cycles since the last valid lane pair
reg [15:0] lane0_buf;
reg [15:0] lane1_buf;
reg        lane0_pending;
reg        lane1_pending;

wire pair_valid = (lane0_pending || lane0_valid) && (lane1_pending || lane1_valid);
wire [15:0] pair_lane0 = lane0_valid ? lane0_data : lane0_buf;
wire [15:0] pair_lane1 = lane1_valid ? lane1_data : lane1_buf;
wire [7:0] pair_l0_low  = pair_lane0[7:0];
wire [7:0] pair_l0_high = pair_lane0[15:8];
wire [7:0] pair_l1_low  = pair_lane1[7:0];
wire [7:0] pair_l1_high = pair_lane1[15:8];

wire [7:0] phase_l0_byte = header_phase_high ? pair_l0_high : pair_l0_low;
wire [7:0] phase_l1_byte = header_phase_high ? pair_l1_high : pair_l1_low;
wire raw8_header_l0_low  = (pair_l0_low[5:0]  == CSI_DT_RAW8);
wire raw8_header_l1_low  = (pair_l1_low[5:0]  == CSI_DT_RAW8);
wire raw8_header_l0_high = (pair_l0_high[5:0] == CSI_DT_RAW8);
wire raw8_header_l1_high = (pair_l1_high[5:0] == CSI_DT_RAW8);

wire [15:0] header_word_count = {header_lane_swap ? phase_l1_byte : phase_l0_byte, wc_low};

// CSI-2 packet-header ECC: a (24,6) Hamming code over {WC_MSB, WC_LSB, DI},
// carried in the 4th header byte with its top two bits zero. Byte layout is the
// one documented at the top of this file, so in ST_HEADER1 the pair holds
// WC_MSB and the ECC byte - the ECC has always been available and simply was not
// looked at.
//
// The masks are the standard MIPI parity equations, one 24-bit selector per
// parity bit. Expressed this way each is auditable against the spec table
// without decoding a pile of XOR terms.
wire [23:0] header_ecc_data = {header_word_count, hdr_di};
wire [7:0]  header_ecc_calc = {2'b00,
                               ^(header_ecc_data & 24'hEFFC00),
                               ^(header_ecc_data & 24'hDF03F0),
                               ^(header_ecc_data & 24'hB8E38E),
                               ^(header_ecc_data & 24'h749A6D),
                               ^(header_ecc_data & 24'hF2555B),
                               ^(header_ecc_data & 24'hF12CB7)};
wire [7:0]  header_ecc_rx   = header_lane_swap ? phase_l0_byte : phase_l1_byte;
wire        header_ecc_ok   = (HEADER_ECC_CHECK == 0) || (header_ecc_calc == header_ecc_rx);

// The row length is compared in halves. While LINE_BYTES was a constant,
// synthesis folded `header_word_count == LINE_BYTES` into a tree of literals; with
// the length as a signal (ERRORS former 31.53) it is a full 16-bit compare, and the
// path lane1_buf -> compare -> CE of the raw8_bad_wc_count counter stopped fitting
// into mipi_byte_clk (139.2 MHz at 148.5). The low byte of the length is latched
// in ST_SEARCH one cycle before the high byte - and compared there too
// (wc_low_ok), so only the high byte remains on the critical cycle.
wire wc_hi_ok = ((header_lane_swap ? phase_l1_byte : phase_l0_byte) == line_bytes[15:8]);
wire wc_match = wc_hi_ok && wc_low_ok;
wire header_line_expected = wc_match && header_ecc_ok;
wire [7:0] payload_first_byte  = header_lane_swap ? phase_l1_byte : phase_l0_byte;
wire [7:0] payload_second_byte = header_lane_swap ? phase_l0_byte : phase_l1_byte;
wire [15:0] payload_word = {payload_second_byte, payload_first_byte};
wire [15:0] next_line_byte_count = line_byte_count + 16'd2;
wire last_line_pair = (line_byte_count == (word_count - 16'd2));
wire line_output_pair = (line_byte_count < line_bytes);
wire last_frame_line = (frame_line_count == frame_last);

// Control state: asynchronous reset.
//
// rst_n here is the D-PHY watchdog reset, and byte_clk STOPS while it is held,
// so this reset cannot be made synchronous. What can be cut is its fan-out: it
// used to clear ~370 flops, which is enough to occupy one of the eight long-wire
// clock networks (net n1826_7; ERRORS 6.5, 6.12).
// Pure datapath registers - bytes captured before they are ever read, and
// last_* telemetry snapshots - live in the reset-free block after this one.
always @(posedge byte_clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= ST_SEARCH;
        header_phase_high <= 1'b0;
        header_lane_swap <= 1'b0;
        line_byte_count <= 16'd0;
        frame_line_count <= 16'd0;
        idle_beats <= 16'd0;
        lane0_pending <= 1'b0;
        lane1_pending <= 1'b0;
        stream_sp_en <= 1'b0;
        stream_lp_av_en <= 1'b0;
        stream_payload_dv <= 2'b00;
        raw8_header_toggle <= 1'b0;
        raw8_line_ok_toggle <= 1'b0;
        raw8_bad_wc_toggle <= 1'b0;
        frame_done_toggle <= 1'b0;
        pixel_seen <= 1'b0;
        overflow_seen <= 1'b0;
        raw8_header_count <= 32'd0;
        raw8_line_count <= 32'd0;
        raw8_bad_wc_count <= 32'd0;
        seen_wc_hits_expected <= 16'd0;
        seen_wc_hits_alt <= 16'd0;
        hdr_ok_q  <= 1'b0;
        hdr_bad_q <= 1'b0;
        hit_exp_q <= 1'b0;
        hit_alt_q <= 1'b0;
    end else begin
        stream_sp_en <= 1'b0;
        stream_lp_av_en <= 1'b0;
        stream_payload_dv <= 2'b00;

        // Header telemetry - one cycle after the decision, from latched flags
        // (ERRORS former 31.53). The values are the same, shifted by one cycle.
        hdr_ok_q  <= 1'b0;
        hdr_bad_q <= 1'b0;
        hit_exp_q <= 1'b0;
        hit_alt_q <= 1'b0;
        if (hdr_ok_q  && (raw8_header_count != 32'hffff_ffff))
            raw8_header_count <= raw8_header_count + 32'd1;
        if (hdr_bad_q && (raw8_bad_wc_count != 32'hffff_ffff))
            raw8_bad_wc_count <= raw8_bad_wc_count + 32'd1;
        if (hit_exp_q) seen_wc_hits_expected <= seen_wc_hits_expected + 16'd1;
        if (hit_alt_q) seen_wc_hits_alt      <= seen_wc_hits_alt + 16'd1;

        if (lane0_valid && !pair_valid) begin
            lane0_pending <= 1'b1;
        end
        if (lane1_valid && !pair_valid) begin
            lane1_pending <= 1'b1;
        end

        if (!enable) begin
            state <= ST_SEARCH;
            line_byte_count <= 16'd0;
            frame_line_count <= 16'd0;
            idle_beats <= 16'd0;
            lane0_pending <= 1'b0;
            lane1_pending <= 1'b0;
        end else if (frame_anchor) begin
            // Strobe-anchored frame start (ERRORS 2.5).
            //
            // The idle re-anchor below turned out not to be enough: once the
            // D-PHY was released by the strobe-timed watchdog - 0.5 ms into the
            // readout, i.e. at the same sensor line every time - every frame
            // came out rotated by exactly 59 lines (row 0 = sensor line 59, rows
            // 965..1023 = the next frame's lines 0..58), consistent across
            // relocks, while the line trailers stayed self-consistent. The end
            // of the exposure is the one frame boundary that does not depend on
            // what the lanes do in the gap.
            state <= ST_SEARCH;
            line_byte_count <= 16'd0;
            frame_line_count <= 16'd0;
            lane0_pending <= 1'b0;
            lane1_pending <= 1'b0;
        end else if (!pair_valid) begin
            // Re-anchor on the inter-frame gap.
            //
            // frame_line_count is what decides where a row is written, and until
            // now nothing tied it to a real frame: it counted completed lines and
            // wrapped at FRAME_LINES-1, so one missed or one false header shifted
            // every later row by one, for good. The gap between frames is ~85 ms
            // at 10 fps against ~0.24 us between lines, so a threshold anywhere in
            // between separates them with enormous margin and cannot fire inside a
            // frame. Saturate rather than wrap, or the counter would come round
            // and re-anchor in the middle of a long frame.
            if (idle_beats != 16'hFFFF)
                idle_beats <= idle_beats + 16'd1;
            if ((IDLE_ANCHOR_BEATS != 16'd0) && (idle_beats == IDLE_ANCHOR_BEATS)) begin
                frame_line_count <= 16'd0;
                state            <= ST_SEARCH;
            end
        end else begin
            idle_beats <= 16'd0;
            lane0_pending <= 1'b0;
            lane1_pending <= 1'b0;

            case (state)
                ST_SEARCH: begin
                    // hdr_di keeps the WHOLE Data Identifier byte, not just the
                    // data type: the ECC is computed over the full byte, virtual
                    // channel bits included.
                    // hdr_di / wc_low are captured in the datapath block below.
                    if (raw8_header_l0_low) begin
                        header_phase_high <= 1'b0;
                        header_lane_swap <= 1'b0;
                        state <= ST_HEADER1;
                    end else if (raw8_header_l1_low) begin
                        header_phase_high <= 1'b0;
                        header_lane_swap <= 1'b1;
                        state <= ST_HEADER1;
                    end else if (raw8_header_l0_high) begin
                        header_phase_high <= 1'b1;
                        header_lane_swap <= 1'b0;
                        state <= ST_HEADER1;
                    end else if (raw8_header_l1_high) begin
                        header_phase_high <= 1'b1;
                        header_lane_swap <= 1'b1;
                        state <= ST_HEADER1;
                    end
                end

                ST_HEADER1: begin
                    line_byte_count <= 16'd0;

                    // Instrumentation: log the candidate regardless of the verdict.
                    // The counters increment one cycle later, from the *_q flags
                    // (see the start of the block): the header decision is already at the
                    // limit of mipi_byte_clk, and 64 count-enable flops plus a saturation
                    // compare in the same cycle were what brought it down.
                    if (wc_match)
                        hit_exp_q <= 1'b1;
                    else if (header_word_count == ALT_LINE_BYTES)
                        hit_alt_q <= 1'b1;

                    if (header_line_expected) begin
                        raw8_header_toggle <= ~raw8_header_toggle;
                        hdr_ok_q <= 1'b1;
                        if (frame_line_count == 16'd0) begin
                            stream_sp_en <= 1'b1;
                        end
                        state <= ST_PAYLOAD;
                    end else begin
                        raw8_bad_wc_toggle <= ~raw8_bad_wc_toggle;
                        hdr_bad_q <= 1'b1;
                        state <= ST_SEARCH;
                    end
                end

                ST_PAYLOAD: begin
                    pixel_seen <= 1'b1;
                    if (line_output_pair) begin
                        stream_lp_av_en <= (line_byte_count == 16'd0);
                        stream_payload_dv <= 2'b11;
                    end
                    line_byte_count <= next_line_byte_count;

                    if (last_line_pair) begin
                        raw8_line_ok_toggle <= ~raw8_line_ok_toggle;
                        if (raw8_line_count != 32'hffff_ffff) begin
                            raw8_line_count <= raw8_line_count + 32'd1;
                        end
                        state <= ST_CRC;
                        if (last_frame_line) begin
                            frame_line_count <= 16'd0;
                            frame_done_toggle <= ~frame_done_toggle;
                        end else begin
                            frame_line_count <= frame_line_count + 16'd1;
                        end
                    end
                end

                ST_CRC: begin
                    state <= ST_SEARCH;
                end

                default: begin
                    state <= ST_SEARCH;
                end
            endcase
        end
    end
end

// Datapath: no reset.
//
// Every register here is either written before it is read (the header bytes
// and word count are captured in ST_SEARCH/ST_HEADER1 before ST_PAYLOAD looks
// at them; the lane buffers are only selected while their pending flag is set)
// or is a telemetry snapshot whose value between reset and the first header
// does not matter. The conditions mirror the control block exactly; while rst_n
// is low these may load garbage, which nothing downstream samples because every
// valid/strobe that qualifies them is held in reset.
always @(posedge byte_clk) begin
    stream_dt <= 6'h2a;
    stream_wc <= word_count;
    frame_last <= ((RUNTIME_FRAME_LINES != 0) ? cfg_frame_lines : FRAME_LINES) - 16'd1;
    stream_payload <= 16'd0;

    if (lane0_valid && !pair_valid) begin
        lane0_buf <= lane0_data;
    end
    if (lane1_valid && !pair_valid) begin
        lane1_buf <= lane1_data;
    end

    if (enable && pair_valid && !frame_anchor) begin
        case (state)
            ST_SEARCH: begin
                // wc_low_ok - the half of the row-length compare done one cycle
                // earlier. See the comment at header_line_expected.
                if (raw8_header_l0_low) begin
                    hdr_di <= pair_l0_low;
                    wc_low <= pair_l1_low;
                    wc_low_ok <= (pair_l1_low == line_bytes[7:0]);
                end else if (raw8_header_l1_low) begin
                    hdr_di <= pair_l1_low;
                    wc_low <= pair_l0_low;
                    wc_low_ok <= (pair_l0_low == line_bytes[7:0]);
                end else if (raw8_header_l0_high) begin
                    hdr_di <= pair_l0_high;
                    wc_low <= pair_l1_high;
                    wc_low_ok <= (pair_l1_high == line_bytes[7:0]);
                end else if (raw8_header_l1_high) begin
                    hdr_di <= pair_l1_high;
                    wc_low <= pair_l0_high;
                    wc_low_ok <= (pair_l0_high == line_bytes[7:0]);
                end
            end

            ST_HEADER1: begin
                word_count <= header_word_count;
                last_word_count <= header_word_count;
                last_lane_swap <= header_lane_swap;
                last_phase_high <= header_phase_high;
                seen_wc_last <= header_word_count;
                // line_index is loaded in ST_HEADER1 unconditionally (25.09.2026).
                // Only the cam_rx packer (and tb_bridge) reads it - on
                // stream_lp_av_en, the first data word of an accepted row, and
                // frame_line_count does not change from the header to the end of the row.
                // A rejected header puts here the same value that the next accepted
                // one would. In exchange the CE of the sixteen flops no longer carries
                // the ECC and the length compare: the path
                // lane*_buf -> line_index/CE gave -1.08 ns on mipi_byte_clk
                // (build L2a with two finders, build_fpga.tcl).
                line_index <= frame_line_count;
                if (header_line_expected) begin
                    if (frame_line_count == 16'd0) begin
                        stream_dt <= 6'h00;
                        stream_wc <= 16'd0;
                    end
                end
            end

            ST_PAYLOAD: begin
                if (line_output_pair) begin
                    stream_wc <= word_count;
                    stream_payload <= payload_word;
                    last_payload_word <= payload_word;
                end
                if (last_line_pair) begin
                    last_line_byte_count <= next_line_byte_count;
                end
            end

            default: begin
            end
        endcase
    end
end

endmodule
