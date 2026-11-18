// flash_ctl.v - our own SPI flash controller in place of the SoC's embedded one
// (16.10.2026, docs/DECISIONS.md R-39, ERRORS 5.12).
//
// The SoC's flash controller with its register port ("Flash Register R/W Mode")
// misses timing at the 100 MHz bus inside Gowin's encrypted logic, so the SoC is
// built with "Customized Instruction Memory" (AE350_EXT_I_MEM_EN), as in Gowin's
// example\Ext_Customized, and this block serves both sides of the flash:
//
//   * ROM port (AHB-lite, ROM_H*): the CPU's instruction memory 0x80000000..; a
//     word read at HADDR returns flash bytes (HADDR[23:0] + 0x600000) mod 16 MB,
//     little-endian - the same window the boot loader and the firmware already
//     use (firmware/src/app/spi_flash.h SPI_FLASH_WINDOW). Consecutive words are
//     streamed in one read command (CS stays low between them). A read at
//     0x8F000000.. (HADDR[27:24] = 0xF) returns MAGIC without touching the flash:
//     that is how the firmware learns this controller is there - the SoC's own
//     controller returns flash data at that address, and an access to
//     0xF8000000 without a slave behind it would hang the core.
//   * command port (APB, the SoC's extended APB slave at 0xF8000000): one SPI
//     command per write of CMD, with an optional 3-byte address and up to 256
//     data bytes through a 256-byte buffer.
//
// Write protection in hardware: only these commands run - 0x03 read, 0x05 read
// status, 0x9F JEDEC id, 0x06 / 0x04 write enable / disable, and 0x02 page
// program and 0x20 4 KB sector erase ONLY at addresses 0xF00000..0xFFFFFF (the
// data area). Anything else is refused without touching the SPI bus, so no
// firmware bug can erase the bitstream (0x000000) or the firmware (0x600000).
//
// A ROM read never meets a busy flash: after reset and after any write-type
// command (0x06, 0x02, 0x20) the controller polls the status register until
// WIP = 0 before the next ROM read. (A CPU reset during an erase would otherwise
// boot from garbage.)
//
// SPI mode 0, single lane, SCLK = clk / 8 (12.5 MHz at 100 MHz): per bit four
// clocks low, four high; MOSI changes with the falling edge, MISO is sampled 70 ns
// after it (three clocks after the rising edge). SCLK route + tCLQV + MISO route
// may thus take up to ~70 ns. At clk / 4 the budget was 30 ns, and the pad routes
// are not timed: builds that placed flash_ctl far from the pins (SCLK is CCLK in
// bank 10, MOSI / MISO in bank 3) misread the data area and the firmware copy
// (28.10.2026, ERRORS 5.15; tb_flash_ctl fails from 31 ns at clk / 4). CS high time
// >= 8 clocks (W25Q128 tSHSL 50 ns).
//
// The APB side is registered and answers with two wait states (see q_*).
// Register map (word offsets from 0xF8000000):
//   0x000 MAGIC  RO  "FCT1" 0x46435431
//   0x004 CMD    W   [7:0] opcode, [8] address phase, [9] data in (read),
//                    [18:10] data bytes (0..256); starts the command when idle
//   0x008 ADDR   RW  [23:0]
//   0x00C STATUS RO  [0] busy, [1] last command refused, [2] CMD written while
//                    busy (ignored), [15:8] last opcode, [31:16] refused count
//   0x010 ROMCNT RO  ROM words served
//   0x014 BUILD  RO  the bitstream's build id (build_id.v), since 23.10.2026
//   0x018 FEAT   RO  features of this bitstream the firmware must not assume (FEATURES); an
//                    older flash_ctl reads 0 here. Bit 0: udp_rd sends frames without chunks
//                    (payload type 0x0001, ERRORS 3.23). Bit 1: FPGA control registers on APB
//                    (apb_ctl at 0xF8001000, 30.10.2026); without it 0xF8001000.. aliases here
//   0x100..0x1FC BUF RW  64 words = 256 bytes; byte k of the data phase is
//                    word k/4, bits [8*(k%4) +: 8]
module flash_ctl (
    input  wire        clk,
    input  wire        rst_n,
    // ROM port (AHB-lite, read only; HSEL is always 1)
    input  wire [31:0] rom_haddr,
    input  wire [1:0]  rom_htrans,
    input  wire        rom_hwrite,
    output reg  [31:0] rom_hrdata,
    output wire        rom_hready,
    output wire        rom_hresp,
    // command port (APB)
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] paddr,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready,
    input  wire [31:0] build_id,          // a constant (build_id.v)
    // SPI flash
    output reg         spi_cs_n,
    output reg         spi_sclk,
    output reg         spi_mosi,
    input  wire        spi_miso
);
    localparam [31:0] MAGIC     = 32'h4643_5431;      // "FCT1"
    localparam [31:0] FEATURES  = 32'h0000_0003;      // 0x018, see the map above
    localparam [23:0] ROM_BASE  = 24'h60_0000;        // 0x80000000 = flash 0x600000
    localparam [3:0]  DATA_AREA = 4'hF;               // erase/program only at 0xF00000..

    assign rom_hresp = 1'b0;

    // ---- ROM port ----------------------------------------------------------------
    reg        hready_r;
    reg        rom_go;                // a read waits for the engine
    reg        rom_id;                // ... at 0x8F000000..: the identity word, no SPI
    reg [21:0] rom_wa;                // HADDR[23:2] as taken from the SoC
    wire [23:0] rom_faddr = {rom_wa, 2'b00} + ROM_BASE;
    reg [31:0] rom_cnt;
    assign rom_hready = hready_r;

    // ---- command port registers -------------------------------------------------
    reg [7:0]  c_op;
    reg        c_addr_en, c_rd;
    reg [8:0]  c_n;
    reg [23:0] c_addr;
    reg        c_go;                  // accepted, waiting for the engine
    reg        c_busy;                // accepted until CS is high again
    reg        st_refused, st_overrun;
    reg [15:0] refused_cnt;

    // 256-byte buffer: four byte lanes of 64, LUT RAM (BSRAM is scarce, BSRAM 318/340)
    (* syn_ramstyle = "distributed_ram" *) reg [7:0] buf0 [0:63];
    (* syn_ramstyle = "distributed_ram" *) reg [7:0] buf1 [0:63];
    (* syn_ramstyle = "distributed_ram" *) reg [7:0] buf2 [0:63];
    (* syn_ramstyle = "distributed_ram" *) reg [7:0] buf3 [0:63];

    // APB, fully registered (ERRORS 5.12): the SoC's APB outputs reach only the q_*
    // flops, never the buffer's write enables or the read multiplexer - the first
    // build, which wrote the buffer straight from PSEL/PADDR (slack +0.169 ns in the
    // report), corrupted buffer bits on the board. Two wait states: setup (capture),
    // access 1 (q_go), access 2 (write done, PRDATA valid, PREADY = 1).
    reg [8:0]  q_addr;                // PADDR[8:0]
    reg [31:0] q_wdata;
    reg        q_write, q_armed, q_go, q_rdy;
    assign pready = q_rdy;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_addr <= 9'd0; q_wdata <= 32'd0; q_write <= 1'b0;
            q_armed <= 1'b0; q_go <= 1'b0; q_rdy <= 1'b0;
        end else begin
            q_go  <= 1'b0;
            q_rdy <= q_go;
            if (psel && !penable) begin
                q_addr <= paddr[8:0]; q_wdata <= pwdata; q_write <= pwrite; q_armed <= 1'b1;
            end else if (q_armed && psel && penable) begin
                q_go <= 1'b1; q_armed <= 1'b0;
            end
        end
    end
    wire apb_wr = q_go && q_write;

    // Allowed commands (see the header). n counts data bytes.
    wire [8:0] w_n       = q_wdata[18:10];
    wire       w_addr_en = q_wdata[8];
    wire       w_rd      = q_wdata[9];
    wire [7:0] w_op      = q_wdata[7:0];
    wire in_area = (c_addr[23:20] == DATA_AREA);
    wire allowed =
        (w_op == 8'h03 &&  w_addr_en &&  w_rd && w_n != 9'd0)                ||
        (w_op == 8'h05 && !w_addr_en &&  w_rd && w_n != 9'd0)                ||
        (w_op == 8'h9F && !w_addr_en &&  w_rd && w_n != 9'd0)                ||
        ((w_op == 8'h06 || w_op == 8'h04) && !w_addr_en && w_n == 9'd0)      ||
        (w_op == 8'h02 &&  w_addr_en && !w_rd && w_n != 9'd0 && in_area)     ||
        (w_op == 8'h20 &&  w_addr_en && w_n == 9'd0 && in_area);

    // ---- engine -----------------------------------------------------------------
    localparam [2:0] E_IDLE = 3'd0, E_CSLOW = 3'd1, E_BYTE = 3'd2, E_DONE = 3'd3,
                     E_HOLD = 3'd4, E_CSHIGH = 3'd5;
    reg [2:0]  est;
    reg [2:0]  ph;                    // clock within a bit, 0..7
    reg [2:0]  bitn;                  // bit within a byte
    reg [2:0]  hdr_n;                 // header bytes left (opcode + address)
    reg [31:0] hdr_sr;
    reg [8:0]  dat_n;                 // data bytes left
    reg        dat_rd;
    reg [7:0]  bidx;                  // data byte index
    reg        cur_data;              // the byte in flight is a data byte
    reg [7:0]  obyte, ibyte;
    reg        miso_q;
    reg [3:0]  hi_cnt;                // CS-high time
    reg [1:0]  job;                   // what runs
    localparam [1:0] J_ROM = 2'd0, J_POLL = 2'd1, J_CMD = 2'd2;
    reg        wip_maybe;             // the flash may be busy writing: poll before a ROM read
    reg [23:0] stream_next;
    reg [31:0] rom_word;
    reg        poll_busy;             // the last status read said WIP = 1
    reg [7:0]  hold_cnt;              // clocks in E_HOLD

    wire [5:0] bw = bidx[7:2];
    // The byte that goes out next in a write data phase: bidx already points at it
    // when cur_data is 0 (first data byte) and one byte behind when it is 1.
    wire [7:0] bidx_next = cur_data ? bidx + 8'd1 : bidx;
    wire [5:0] bwn = bidx_next[7:2];
    wire [7:0] buf_byte_at = (bidx_next[1:0] == 2'd0) ? buf0[bwn] : (bidx_next[1:0] == 2'd1) ? buf1[bwn] :
                             (bidx_next[1:0] == 2'd2) ? buf2[bwn] : buf3[bwn];
    // Registered one clock ahead: the 256-to-1 read of the flip-flop buffer was the worst
    // ae350_ahb_clk path (bidx -> obyte, B4 and 8823136F). bidx, cur_data and the buffer are
    // stable for at least the 64 clocks of the byte before the one that loads buf_byte_next
    // (the first data byte follows a header byte; APB writes only while idle).
    reg  [7:0] buf_byte_next;
    always @(posedge clk) buf_byte_next <= buf_byte_at;

    // Buffer writes: APB words when idle, engine bytes of a read command.
    wire eng_we = (est == E_BYTE) && ph == 3'd7 && bitn == 3'd7 && cur_data && dat_rd && job == J_CMD;
    wire [7:0] eng_byte = {ibyte[6:0], miso_q};
    wire apb_buf_we = apb_wr && q_addr[8] && !c_busy;
    always @(posedge clk) begin
        if (apb_buf_we) begin
            buf0[q_addr[7:2]] <= q_wdata[7:0];
            buf1[q_addr[7:2]] <= q_wdata[15:8];
            buf2[q_addr[7:2]] <= q_wdata[23:16];
            buf3[q_addr[7:2]] <= q_wdata[31:24];
        end else if (eng_we) begin
            case (bidx[1:0])
                2'd0: buf0[bw] <= eng_byte;
                2'd1: buf1[bw] <= eng_byte;
                2'd2: buf2[bw] <= eng_byte;
                default: buf3[bw] <= eng_byte;
            endcase
        end
    end

    // APB read data, from the registered address; valid while PREADY is high.
    always @(posedge clk) begin
        if (q_go && !q_write) begin
            if (q_addr[8])
                prdata <= {buf3[q_addr[7:2]], buf2[q_addr[7:2]], buf1[q_addr[7:2]], buf0[q_addr[7:2]]};
            else case (q_addr[7:2])
                6'd0: prdata <= MAGIC;
                6'd2: prdata <= {8'd0, c_addr};
                6'd3: prdata <= {refused_cnt, c_op, 5'd0, st_overrun, st_refused, c_busy};
                6'd4: prdata <= rom_cnt;
                6'd5: prdata <= build_id;
                6'd6: prdata <= FEATURES;
                default: prdata <= 32'd0;
            endcase
        end
    end

    always @(posedge clk) miso_q <= spi_miso;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hready_r   <= 1'b1;
            rom_go     <= 1'b0;
            rom_id     <= 1'b0;
            rom_wa     <= 22'd0;
            rom_hrdata <= 32'd0;
            rom_cnt    <= 32'd0;
            c_op <= 8'd0; c_addr_en <= 1'b0; c_rd <= 1'b0; c_n <= 9'd0; c_addr <= 24'd0;
            c_go <= 1'b0; c_busy <= 1'b0;
            st_refused <= 1'b0; st_overrun <= 1'b0; refused_cnt <= 16'd0;
            est <= E_CSHIGH; hi_cnt <= 4'd0;
            ph <= 3'd0; bitn <= 3'd0; hdr_n <= 3'd0; hdr_sr <= 32'd0; dat_n <= 9'd0;
            dat_rd <= 1'b0; bidx <= 8'd0; cur_data <= 1'b0; obyte <= 8'd0; ibyte <= 8'd0;
            job <= J_ROM; wip_maybe <= 1'b1; stream_next <= 24'd0;
            rom_word <= 32'd0; poll_busy <= 1'b0; hold_cnt <= 8'd0;
            spi_cs_n <= 1'b1; spi_sclk <= 1'b0; spi_mosi <= 1'b0;
        end else begin
            // ROM address phase: taken while HREADY is high.
            // (The adder for the flash address sits after these flops.)
            if (hready_r && rom_htrans[1] && !rom_hwrite) begin
                rom_wa   <= rom_haddr[23:2];
                rom_id   <= (rom_haddr[27:24] == 4'hF);
                rom_go   <= 1'b1;
                hready_r <= 1'b0;
            end

            // Command port writes.
            if (apb_wr && !q_addr[8]) begin
                if (q_addr[7:2] == 6'd2 && !c_busy) c_addr <= q_wdata[23:0];
                if (q_addr[7:2] == 6'd1) begin
                    if (c_busy) st_overrun <= 1'b1;
                    else begin
                        c_op <= w_op; c_addr_en <= w_addr_en; c_rd <= w_rd;
                        c_n <= (w_n > 9'd256) ? 9'd256 : w_n;
                        st_overrun <= 1'b0;
                        if (allowed) begin
                            st_refused <= 1'b0;
                            c_go <= 1'b1;
                            c_busy <= 1'b1;
                        end else begin
                            st_refused <= 1'b1;
                            refused_cnt <= refused_cnt + 16'd1;
                        end
                    end
                end
            end

            case (est)
            E_IDLE: begin
                spi_cs_n <= 1'b1; spi_sclk <= 1'b0;
                if (rom_go && rom_id) begin
                    // 0x8F000000..: the identity word, so firmware can tell this
                    // controller from the SoC's own one without touching 0xF8000000
                    // (the old controller returns flash 0x600000 there, firmware code).
                    rom_go     <= 1'b0;
                    rom_hrdata <= MAGIC;
                    hready_r   <= 1'b1;
                end else if (rom_go && wip_maybe) begin
                    job <= J_POLL;                    // status first: 0x05, one byte in
                    hdr_n <= 3'd1; hdr_sr <= {8'h05, 24'd0};
                    dat_n <= 9'd1; dat_rd <= 1'b1; bidx <= 8'd0;
                    est <= E_CSLOW;
                end else if (rom_go) begin
                    job <= J_ROM;
                    rom_go <= 1'b0;
                    hdr_n <= 3'd4; hdr_sr <= {8'h03, rom_faddr};
                    dat_n <= 9'd4; dat_rd <= 1'b1; bidx <= 8'd0;
                    stream_next <= rom_faddr + 24'd4;
                    est <= E_CSLOW;
                end else if (c_go) begin
                    job <= J_CMD;
                    c_go <= 1'b0;
                    hdr_n <= c_addr_en ? 3'd4 : 3'd1;
                    hdr_sr <= {c_op, c_addr};
                    dat_n <= c_n; dat_rd <= c_rd; bidx <= 8'd0;
                    if (c_op == 8'h06 || c_op == 8'h02 || c_op == 8'h20) wip_maybe <= 1'b1;
                    est <= E_CSLOW;
                end
            end
            E_CSLOW: begin                            // CS low one clock before the first edge
                spi_cs_n <= 1'b0; spi_sclk <= 1'b0;
                cur_data <= 1'b0;
                est <= E_BYTE; ph <= 3'd7; bitn <= 3'd7;   // "end of a byte": loads the first one
            end
            E_BYTE: begin
                ph <= ph + 3'd1;
                spi_sclk <= (ph >= 3'd3) && (ph <= 3'd6);   // high during ph 4..7
                if (ph == 3'd7) begin
                    // miso_q = the pin at the end of ph 6: 7 clocks after the falling
                    // edge that made the flash drive the bit, 3 after the rising one.
                    ibyte <= {ibyte[6:0], miso_q};
                    if (bitn == 3'd7) begin
                        // A byte ended (or none started yet): store it, load the next one.
                        if (cur_data) begin
                            if (dat_rd && job == J_ROM) rom_word[8*bidx[1:0] +: 8] <= eng_byte;
                            if (dat_rd && job == J_POLL) poll_busy <= eng_byte[0];
                            bidx  <= bidx + 8'd1;
                        end
                        if (hdr_n != 3'd0) begin
                            obyte    <= hdr_sr[31:24];
                            spi_mosi <= hdr_sr[31];
                            hdr_sr   <= {hdr_sr[23:0], 8'd0};
                            hdr_n    <= hdr_n - 3'd1;
                            cur_data <= 1'b0;
                            bitn     <= 3'd0;
                        end else if (dat_n != 9'd0) begin
                            obyte    <= dat_rd ? 8'h00 : buf_byte_next;
                            spi_mosi <= dat_rd ? 1'b0 : buf_byte_next[7];
                            dat_n    <= dat_n - 9'd1;
                            cur_data <= 1'b1;
                            bitn     <= 3'd0;
                        end else begin
                            cur_data <= 1'b0;
                            est      <= E_DONE;
                        end
                    end else begin
                        bitn     <= bitn + 3'd1;
                        spi_mosi <= obyte[6];
                        obyte    <= {obyte[6:0], 1'b0};
                    end
                end
            end
            E_DONE: begin
                spi_sclk <= 1'b0;
                case (job)
                J_ROM: begin
                    rom_hrdata <= rom_word;
                    hready_r   <= 1'b1;
                    rom_cnt    <= rom_cnt + 32'd1;
                    hold_cnt   <= 8'd0;
                    est        <= E_HOLD;
                end
                J_POLL: begin
                    if (!poll_busy) wip_maybe <= 1'b0;
                    est <= E_CSHIGH; hi_cnt <= 4'd0;
                end
                default: begin
                    est <= E_CSHIGH; hi_cnt <= 4'd0;
                end
                endcase
            end
            E_HOLD: begin                             // CS low, SCLK low: the next word may follow
                spi_sclk <= 1'b0;
                hold_cnt <= hold_cnt + 8'd1;
                if (rom_go && !rom_id && !c_go && !wip_maybe && rom_faddr == stream_next) begin
                    rom_go <= 1'b0;
                    job <= J_ROM;
                    hdr_n <= 3'd0; dat_n <= 9'd4; dat_rd <= 1'b1; bidx <= 8'd0;
                    stream_next <= rom_faddr + 24'd4;
                    cur_data <= 1'b0;
                    est <= E_BYTE; ph <= 3'd7; bitn <= 3'd7;
                end else if (rom_go || c_go || hold_cnt == 8'hFF) begin
                    // Another address, a command, or nothing for 2.5 us: CS back high.
                    est <= E_CSHIGH; hi_cnt <= 4'd0;
                end
            end
            default: begin                            // E_CSHIGH
                spi_cs_n <= 1'b1; spi_sclk <= 1'b0;
                hi_cnt <= hi_cnt + 4'd1;
                if (hi_cnt == 4'd9) begin
                    est <= E_IDLE;
                    if (job == J_CMD) c_busy <= 1'b0;
                end
            end
            endcase
        end
    end

endmodule
