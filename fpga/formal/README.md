# fpga/formal — formal proofs of the AXI4 back end

SymbiYosys proofs of the blocks that keep memory writes and reads intact on the
AXI4 back end (CLAUDE.md invariants 3 and 7, ERRORS 1.19, 1.20, 1.24). The RTL
is never edited: each `*_fv.sv` harness instantiates the block and reads its
internal state by hierarchical name, which needs the `read_slang` front end
(`read_verilog` turns such names into free wires without an error).

| Job | Block | What is proved | Size | Time |
|---|---|---|---|---|
| `axi_mux_rs` | `axi_mux_rs` (register slice) | no beat lost, duplicated or reordered; output AXI-legal; full throughput | any input | 1.5 s |
| `cam_axi_wr` `small` / `mid` | `cam_axi_wr` | every W beat and AW is the one the spec expects (data, strobes, WLAST, address, AWLEN); no AW before its burst is buffered; no 4 KB crossing; AW_AHEAD respected; AXI-legal; neither queue overflows | MAX_WORDS 16 / 32, FIFO 8 / 16 beats, AW queue 8 / 16 | 18 s / 170 s |
| `axi_mux` `prove` | `axi_mux` | 2 write + 2 read masters: AW/AR reach the slave in order with their payload and ID; W in AW order from the right master, WLAST after AWLEN+1 beats; B and R to the master their ID names, intact; RREADY never low; the read credit scheme never overflows a FIFO (`o_bad_id` = 0); both sides AXI-legal | 8-bit data, bursts 1..4, W-order FIFO 4, read FIFO 4 | 16 s |

All proofs are unbounded (k-induction, engine `smtbmc bitwuzla`): the `I*`
assertions in the harnesses tie the block's internal state to the scoreboards
so that induction closes, and they are checked like every other assertion.

Environment models (assumptions), in short:

* **cam_axi_wr** — a writer burst is 1..MAX_WORDS words with `s_first`/`s_last`,
  any word-aligned start; it starts only after `s_room` was seen high after the
  previous burst's last word, ≥ GAP cycles before its first word. cam_channel's
  FSM has GAP = 4; the proofs use GAP = 1 (a harsher writer, so they cover 4).
  The AXI side is free (any AWREADY/WREADY/BVALID).
* **axi_mux** — masters are AXI-legal; a write master sends exactly AWLEN+1 beats
  per burst in AW order, W may run ahead of its AW; a read burst is ≤ 4 beats.
  The slave answers B only for finished bursts, in order; R per ID in AR order,
  IDs interleaved freely; it holds at most 6 write bursts between AW and B.

Data is abstracted: cam_axi_wr carries 2 free bits per 32-bit word (lowest and
highest), axi_mux 8 bits per beat. The blocks never look at data, so this tells
any two words apart.

## Running

```powershell
cd fpga\formal
.\run.ps1 axi_mux_rs                 # all tasks of a job
.\run.ps1 cam_axi_wr small cover     # chosen tasks
.\run.ps1 all                        # includes cam_axi_wr real: hours
python mutants.py                    # mutation check, every job (~5 min)
python mutants.py axi_mux -k ar_     # one job, mutants whose name has "ar_"
```

Work directories go to `%TEMP%\fpga_formal`, never into the repository. Each
`.sby` file lists its tasks at the top. OSS CAD Suite 20260907 (Yosys 0.68+195,
SBY, bitwuzla) from `%USERPROFILE%\oss-cad-suite`.

## Mutation check

A proof nobody has seen fail proves nothing (WORKFLOWS.md § 10). `mutants.py`
plants each defect into a scratch copy of the RTL and runs the proof; every
mutant must be caught and the control (a comment edit) must survive. Result
16.10.2026:

| Job | Mutants | Caught | Survived as expected |
|---|---|---|---|
| axi_mux_rs | 3 + control | 3 | control |
| cam_axi_wr | 12 + control | 11 | control; `wq_margin_-2` |
| axi_mux | 10 + control | 9 | control; `aw_regrant` |

What the survivors mean:

* `wq_margin_-2` — `WQ_ROOM = WQD - MAXB - 2` instead of `- 4` is still safe:
  the beat FIFO's margin is 2 beats more than it needs. `- 1` overflows (caught).
  The AW queue's margin `AQD - 5` is exact: one less overflows (caught).
* `aw_regrant` — masking the AWREADY pulse from the next grant
  (`m_awvalid & ~aw_rdy_q`) is redundant: the grant also waits for `!aw_act`,
  which is still set in the pulse cycle.

The axi_mux_rs mutants fail from reset (a real trace); the others are caught by
the induction step (the mutant breaks an invariant the real design keeps), and
`room_stuck_1_bmc` and `ar_no_credit_bmc` repeat two of them as bounded runs
from reset to show a real failing trace.

## Things the proofs showed about the design

* **W can run ahead of its AW in cam_axi_wr** (cover reached: `ahead` < 0). When
  AWREADY is held off while WREADY is given, the W beats of a following short
  burst (e.g. the one-beat tail after a 4 KB split) leave before its AW is
  loaded. The signed `ahead` counter handles it; the comment above it
  ("with the present timing it cannot") is not right in general — an unsigned
  counter would stop AW for good in that case.
* Neither queue of cam_axi_wr can overflow for any AXI stall pattern, and the
  read FIFOs of axi_mux cannot overflow for any slave R pattern: the
  room/credit arithmetic is proved, not only simulated.

## Not covered

* The full-size cam_axi_wr (MAX_WORDS 128, FIFO 128 beats, AW queue 32): the
  induction step did not finish in 3 h; `small` and `mid` use the same margin
  arithmetic, and the `real` task stays in the `.sby` for a faster machine.
* axi_mux at full width (256 bits, 64-beat read FIFOs, 4 masters as in
  `scanner_top.v`): the structure is the same, the size is not proved.
* Liveness: the `live` task of `axi_mux` (bounded wait for AW/AR grants under an
  always-ready environment) is written but has not been run to depth yet; a
  deadlock would pass every proof here.
* Clock-domain crossings and anything beyond these three blocks.
