# udp_probe — UDP stream straight from the FPGA (stage 1 of the UDP plan)

A separate small FPGA project **without the SoC and DDR3**, stage 1 of
`docs/architecture/UDP_FRAME_STREAM_PLAN.md`. The same Gowin MAC (Triple Speed
Ethernet MAC) and the same encrypted AHB wrapper `gw_ahb_ethernet` as in
`fpga/src/scanner_top.v`, the same pins and clocks. It answers three questions
that would otherwise have to be found out on the full build:

1. **How much the PC receives** — Python on Windows, a UDP stream at line rate
   (~107 thousand packets/s with one row per packet), with what gap between packets
   there is no loss, whether jumbo packets (8 rows per packet) get through.
2. **How the MAC behaves on the client interface** when transmitting past the wrapper
   (IPUG538: no buffer, `valid` is held for the whole packet, statistics after the packet).
3. **Whether the Gowin wrapper tolerates interleaved packets**: the MAC transmitter is shared
   packet by packet between the wrapper (the processor path) and the generator; the wrapper is held at
   `tx_mac_ready = 0` until the generator's packet has gone out, the statistics pulse
   goes only to the owner of the packet (`probe_tx_mux.v`).

Loaded **into SRAM only** (`--run 2`): the SPI flash and the working system are not
touched, `--run 1` (or a power cycle) restores the board.

## What is inside

| File | What it does |
|---|---|
| `src/udp_probe_top.v` | PLL (`Gowin_PLL_ETH`), `PHY_CLK` through ODDR, PHY reset 25 ms and MAC 10 ms after it, MAC, wrapper, three domains (`CLK` 50 MHz, `tx_mac_clk` 125, `rx_mac_clk`), crossings between them — toggle + data hold, UART report |
| `src/probe_udp_gen.v` | source B: UDP stream and ARP replies, a byte per `tx_mac_ready` from registers (58 B header in a shift register, pattern, CRC-32) |
| `src/probe_tx_mux.v` | queue to the MAC: whole packet, in turn, statistics — to the packet owner, counters (how long the wrapper waited) |
| `src/probe_rx.v` | receive: ARP request to 10.0.0.125 and a command on UDP 39153 (the sender's address becomes the stream address) |
| `src/probe_ahb.v` | AHB master instead of the processor: `ETH_MODE = 1` (1000M FD), PHYSR over MIIM once a second, "heartbeat" — a 64 B UDP packet through the wrapper window, like `low_level_output` |
| `../fps_probe/src/uart.v` | UART transmitter (shared with `fps_probe`) |

MAC — same as the board's, `00:12:34:56:78:9A`, port 39153. **IP is the `BOARD_IP` parameter
of the top module**, and the default value is the build: `192.168.31.125` — direct
cable to the PC's Ethernet (192.168.31.10/24), since 13.10.2026; `10.0.0.125` — the office
network, the board's own address (sessions 1–3, build `52CFF213`; the testbench passes it explicitly).
A new address means a new build and a new placement lottery (`build.tcl`).

**The MAC mode follows the link:** once a second the AHB master reads PHYSR and, if
the speed or duplex does not match `ETH_MODE`, rewrites it, as the firmware does
(1 — 1000M FD, 0 — 100M FD …). Until 13.10.2026 the probe always held 1000M FD —
and on a direct cable that came up at 100M it would not have worked at all. The testbench does not check this branch
(the wrapper stub always answers 1000M FD).

**Stream packet** (UDP payload = `udp_len` bytes, all big-endian):
`'UPSB'`, `cmd_id` (16), `udp_len` (16), `seq` (32, from 0 per command), `count`
(32), data `udp_len − 20` bytes, `data[k] = (seq + k) & 0xFF`, CRC-32
(`zlib.crc32` of the application header and data). IPv4 without options, DF, ID 0,
UDP checksum 0. `udp_len` 1116 — the row packet size per the plan (header
≤ 32 + 1080 + CRC 4), 8660 — eight rows in a jumbo packet (16 + 8 × 1080 + 4).

**Command** (UDP to 10.0.0.125:39153, 18 bytes): `'UPRC'`, `cmd_id`,
`udp_len`, `count` (0 — stop), `gap` (8 ns clocks after a packet), `ifg`
(0 — the MAC default of 12 B, otherwise `tx_ifg_delay`, ≥ 8), `flags` (bit 0 —
"heartbeat"), `hb_us` ("heartbeat" period, µs).

**Report over UART2 (COM7, 115200)** once a second, `fps_probe` format:
`0x5A 0xA5`, 16 BE words, checksum byte; word order — `WORDS` in
`tools/net/udp_rx_bench.py` and `rep_vec` in `udp_probe_top.v`.

## Build and loading

```powershell
Set-Location fpga\udp_probe
& "C:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" build.tcl       # -> impl\pnr\udp_probe.fs
$PGM = "C:\Gowin\Gowin_V1.9.12.03_x64\Programmer\bin\programmer_cli.exe"
Remove-Item Env:\PYTHONIOENCODING -ErrorAction SilentlyContinue
& $PGM --device GW5AST-138B --freq 15MHz --run 2 --fsFile fpga\udp_probe\impl\pnr\udp_probe.fs --cable-index 1 --location 0
& $PGM --device GW5AST-138B --run 1 --cable-index 1 --location 0     # restore the working system from flash
```

On the PC:

```powershell
python tools/net/udp_rx_bench.py --uart COM7 --status        # PHY, wrapper, counters
python tools/net/udp_rx_bench.py --uart COM7                  # 12800 rows (10 frames), line rate
python tools/net/udp_rx_bench.py --count 128000 --hb-us 100   # 100 frames, "heartbeat" through the wrapper
python tools/net/udp_rx_bench.py --len 8660 --count 1600      # jumbo, if enabled on the PC's NIC
python tools/net/udp_rx_bench.py --sweep-ifg 12 40 80 160 255
```

**Gratuitous ARP once a second** (broadcast, ~60 B/s). Without it
the first session on the board (13.10.2026) lost every command of the first 20 seconds:
after an FPGA reload the link comes up again, the office switch forgets the port
of the board's MAC, and unicast frames to an unknown MAC it apparently does not
flood — broadcast ones got through (the receive counter grew), commands did not.
It started working exactly after Windows sent out an ARP request and the probe
replied. The working firmware does not know about this: lwIP sends frames by itself.

**Network.** The stream goes only to the address that sent the command (its MAC from
the command frame), and only `count` packets: without a command the probe is silent. At
line rate it will take the PC's port entirely; if there is an office switch between the board and the PC —
make sure it knows the PC's MAC (the command itself refreshes it),
otherwise the frames will flood all ports.

## Timing

`fpga\check_timing.ps1 -Report fpga\udp_probe\impl\pnr\udp_probe_tr_content.html
-OwnClocks gtx_clk,rgmii_rxclk` — the same rules as for the main build (0.3 ns
slack on its own clocks). The accepted build — for the direct cable
(192.168.31.125, MAC mode by link), `place_option 2` (the default of
`build.tcl`), `6AE1BF4D`
(`andesprojects_builds\20261013_udp_probe\direct\place2_udp_probe.fs`):
`gtx_clk` 140.8 MHz (+0.90 ns), `rgmii_rxclk` 144.1, `clk50m` 103.9. The build for the
office network (10.0.0.125, with GARP) — `52CFF213` in `...\garp\`: `gtx_clk` 128.6,
**+0.225 ns — less than 0.3**, accepted for the probe (SRAM only, every byte
of the stream is checked by CRC and pattern). Sweep with GARP: place 0 — 94.1 FAIL,
1 — 116.7 FAIL, 3 — 128.6 (+0.226), 4 — 126.8 (+0.111); without GARP place 2 gave
141.3 — a lottery on a single RTL line. The worst path is the generator →
`tx_mux` boundary (lesson 4 below).

What the tuning gave (seven builds 12–13.10.2026), and what stage 3 needs from it:

1. **There is no room for even one LUT between the Gowin wrapper and the MAC.** The first
   version of `tx_mux` — a combinational source select — closed at 107 MHz:
   the wrapper's outputs are born in its own logic, and the MAC input goes straight into
   its CRC and state. Hence the two-entry register slice in front of the MAC and the
   registered statistics pulse (`probe_tx_mux.v`).
2. **The byte for the MAC comes from a register or a FIFO, not from a "header / data /
   CRC" select plus CRC in the same clock.** The generator now writes the packet into an 8-byte
   FIFO (`probe_udp_gen.v`).
3. **Synthesis merges a counter's load and decrement into one subtractor**
   (the carry runs from the low bit of the loaded value through all 16 bits) and
   moves a constant through the register into the comparison (`x == c − 2` → `x + 2 ==
   c`). Count up from zero and snapshot the length at packet start.
4. **Placement spread is 110…141 MHz on the same sources**, and it is wires:
   the worst path of build 7 is five LUTs and 7.1 ns of routing between the generator and
   `tx_mux`. In `udp_tx` of the main build every handshake across a module
   boundary is flip-flop → ≤ 2 LUT → flip-flop (a register slice at the source's
   output too), otherwise every edit is a new lottery.

## Testbench (`sim/`)

`sim\run_sim.ps1` — the probe against the **real MAC**: the netlist of the same IP
(version 1.0, but an earlier build) from Gowin's reference project
(`IPUG538`, kept outside git in `C:\Users\ryazanov\Documents\gowin_docs\IPUG538E\`;
`triple_speed_ethernet_mac.vo` and `tb\prim_sim.v` of the GW2A library). A second copy
of the same MAC is the "PC": it sends ARP and commands, receives everything the probe sends (the FCS
is checked by the MAC itself), `check_frames.py` checks every field. The Gowin wrapper is
encrypted and does not go into iverilog — it is replaced by `stub_wrapper.v` with the same
register map, which after a packet waits for the statistics pulse (if `tx_mux`
gives it to the wrong one, the "heartbeat" stalls). PLL — `stub_pll_eth.v`.

Scenario: ARP; 24 rows of 1116 B with a "heartbeat" every 15 µs (interleaving);
3 jumbo packets of 8660 B with IFG 40 and a gap of 200 clocks; a long stream
stopped by a command with `count = 0`.

```powershell
fpga\udp_probe\sim\run_sim.ps1                    # ~20 min: the gate-level MAC model is slow
fpga\udp_probe\sim\run_sim.ps1 -Mutate "old=new"  # mutant: must fail
```

Two testbench traps found while writing it:

* **RGMII in the model.** The MAC receiver delays the data with `IODELAY`
  (`C_STATIC_DLY = 100` × 25 ps = 2.5 ns in the reference netlist), so the receive
  clock must lag by 3…6 ns (testbench — 4.5); at 0…2 ns the latch takes
  the neighbouring nibble and does not see the SFD — no frames at all. And the clock delay must be
  transport only (`<= #4.5`): the inertial `assign #4.5` swallows
  4 ns pulses.
* **A function that reads a module variable not through an argument**
  is not re-evaluated in a continuous assignment when that variable changes (sensitivity is the argument
  list, IEEE 1364): frame parsing in `probe_rx.v` saw `x`. Synthesis would
  not have been affected, but the testbench lied — now everything the functions read is passed
  as an argument.
