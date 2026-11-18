#!/usr/bin/env python3
"""Guaranteed-delivery gate bench: src/app/gvsp_gate.c in gate_bench.c's model, then its mutants.

    python firmware/tests/gate_bench.py [--gcc PATH] [--keep]

Every scenario must PASS on the real gvsp_gate.c; every mutant (a one-line change of it)
must FAIL at least one scenario. Exit code = scenarios failed + mutants not caught.
Needs a host gcc (MinGW: winget BrechtSanders.WinLibs).

Two guards are not mutated, the model cannot reach them: an ack past what was sent (the model's
host only acknowledges what it holds; the guard is for a host that does not) and the head seen
by the FPGA's next frame when the shut frame's slot does not hold it (in the model a slot holds
its frame until the gate would stop the overwrite; the guard is a second reading of the head).
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.join(HERE, "..", "src", "app")
GATE = os.path.join(APP, "gvsp_gate.c")

# N, mailbox latency (frames), ack every (frames), stalls, stall length (frames), stale frame ring,
# first frame
SCENARIOS = [
    (8, 20, 64, 0, 0, 0),           # no stall: no loss, the gate never shuts
    (8, 20, 64, 3, 6000, 0),        # stalls inside the ring: the gate holds, nothing lost
    (8, 20, 64, 3, 20000, 0),       # stalls past the ring: shut, counted loss, resume
    (1, 20, 50, 4, 15000, 1),       # N = 1, a frame ring holding an older capture
    (16, 200, 100, 2, 30000, 0),    # N = 16, slow mailbox (200 frames)
    (4, 5, 32, 6, 9000, 1),         # many stalls just past the ring's free part
    (8, 0, 64, 2, 20000, 0),        # a mailbox with no latency: the open takes the next frame
    (8, 300, 16, 3, 7700, 0),       # slow mailbox, a stall just past the shut: opened before any frame was held off
    (8, 20, 4000, 2, 12000, 0),     # a host that acknowledges every 4 s
    (8, 20, 4000, 1, 8000, 0),      # ... with the ring's wrap reaching the flushed block before its ack
    (8, 20, 64, 3, 20000, 1, 0x7FFF0000),   # frame numbers through 2^31
    (8, 20, 64, 3, 20000, 0, 0xFFFF8000),   # and through 2^32 (frame 0 reads as "none")
]

MUTANTS = [
    ("no margin", "GVSP_GATE_SLOTS - GVSP_GATE_MARGIN;", "GVSP_GATE_SLOTS;"),
    ("resume not block aligned", "g->resume = (f + n) & ~(n - 1U);\n        return GVSP_GATE_OPEN;",
     "g->resume = f + 1U;\n        return GVSP_GATE_OPEN;"),
    ("cut block flushed while still filling", "held_last = f != 0U && (int32_t)(f - (next + n - 1U)) >= 0 && !io->dense_has(io->ctx, next + n - 1U);",
     "held_last = f != 0U;"),
    ("empty frame ring read as a frame", "held_last = f != 0U && ", "held_last = "),
    ("held-off frames not re-checked", "if ((int32_t)(f - g->resume) >= 0) {", "if (0) {"),
    ("cut block never sent", "return GVSP_GATE_FLUSH;", "return 0;"),
    ("open before the cut block is acked", "(int32_t)(g->acked_end - (next + n)) < 0", "0"),
    ("protected start not moved", "g->acked_end = g->resume;", ""),
    ("overflow not counted", "g->overflow += (g->resume - next) / n;", ""),
    ("ack taken backwards", "(int32_t)(a - g->acked_end) > 0", "a != g->acked_end"),
]


def gcc_path(arg):
    if arg:
        return arg
    found = shutil.which("gcc")
    if found:
        return found
    hits = glob.glob(os.path.expandvars(r"%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs*\mingw64\bin\gcc.exe"))
    if hits:
        return hits[0]
    sys.exit("no host gcc: pass --gcc")


def build(gcc, gate_src, out, strict=True):
    r = subprocess.run([gcc, "-O2", "-Wall", "-Wextra"] + (["-Werror"] if strict else []) +
                       ["-I", APP, os.path.join(HERE, "gate_bench.c"), gate_src, "-o", out],
                       capture_output=True, text=True)
    if r.returncode:
        sys.exit(f"build failed:\n{r.stderr}")


def run(exe, sc):
    r = subprocess.run([exe] + [str(x) for x in sc], capture_output=True, text=True, timeout=120)
    return r.returncode == 0, r.stdout.strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--gcc")
    ap.add_argument("--keep", action="store_true", help="keep the build directory")
    a = ap.parse_args()
    gcc = gcc_path(a.gcc)
    tmp = tempfile.mkdtemp(prefix="gate_bench_")
    bad = 0
    try:
        # the control-block map (frame_udp_map.h: DDR3 / APB) first, it needs no model
        mexe = os.path.join(tmp, "map.exe")
        r = subprocess.run([gcc, "-O2", "-Wall", "-Wextra", "-Werror", "-I", APP,
                            os.path.join(HERE, "frame_udp_map_test.c"), "-o", mexe], capture_output=True, text=True)
        ok = r.returncode == 0 and subprocess.run([mexe], capture_output=True, text=True).returncode == 0
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'} frame_udp_map: " + (r.stderr.strip() or
              subprocess.run([mexe], capture_output=True, text=True).stdout.strip()))
        exe = os.path.join(tmp, "gate.exe")
        build(gcc, GATE, exe)
        for sc in SCENARIOS:
            ok, out = run(exe, sc)
            bad += not ok
            print(f"{'ok  ' if ok else 'FAIL'} scenario {sc}: {out}")
        src = open(GATE, encoding="utf-8").read()
        for name, old, new in MUTANTS:
            if src.count(old) != 1:
                print(f"FAIL mutant '{name}': its line is not in gvsp_gate.c once ({src.count(old)})")
                bad += 1
                continue
            m = os.path.join(tmp, "gvsp_gate_mutant.c")
            with open(m, "w", encoding="utf-8") as f:
                f.write(src.replace(old, new))
            mexe = os.path.join(tmp, "mutant.exe")
            build(gcc, m, mexe, strict=False)
            caught = [(sc, out) for sc in SCENARIOS for ok, out in [run(mexe, sc)] if not ok]
            bad += not caught
            print(f"{'ok  ' if caught else 'FAIL'} mutant '{name}': "
                  + (f"caught by {len(caught)} scenario(s), first {caught[0][0]}: {caught[0][1]}" if caught else "NOT caught"))
    finally:
        if not a.keep:
            shutil.rmtree(tmp, ignore_errors=True)
    print(f"\n{bad} failures")
    sys.exit(min(bad, 255))


if __name__ == "__main__":
    main()
