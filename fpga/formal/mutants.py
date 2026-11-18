"""Mutation check for the proofs in fpga/formal.

A proof that has never failed proves nothing (WORKFLOWS.md section 10). For
every job this script plants defects into a scratch copy of the RTL - never
the working tree - and runs the job's prove task on each. A mutant must be
CAUGHT (the proof fails); the control mutant (a comment edit) must SURVIVE.

    python mutants.py                 # every job
    python mutants.py cam_axi_wr      # one job
    python mutants.py -k aw_stable    # mutants whose name contains the text

Exit code = number of mutants whose outcome differs from the expectation.
Scratch: %TEMP%\\fpga_formal\\mut\\<job>\\<mutant>.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
OSS = os.path.join(os.environ.get("USERPROFILE", ""), "oss-cad-suite")
WORK = os.path.join(os.environ.get("TEMP", HERE), "fpga_formal", "mut")
TIMEOUT_S = 1200

# (job, name, file basename, old text, new text, expected outcome, task)
# old text must occur exactly once in the file.
MUTANTS = [
    # ---- axi_mux_rs ----------------------------------------------------------
    ("axi_mux_rs", "control", "axi_mux.v",
     "// Output slice: the master's RREADY", "// Output slice (control): the master's RREADY",
     "survives", "prove"),
    ("axi_mux_rs", "ready_always", "axi_mux.v",
     "assign i_ready = (cnt != 2'd2);", "assign i_ready = 1'b1;", "caught", "prove"),
    ("axi_mux_rs", "rp_stuck", "axi_mux.v",
     "if (pop)  rp <= ~rp;", "if (pop)  rp <= rp;", "caught", "prove"),
    ("axi_mux_rs", "d1_from_d0", "axi_mux.v",
     "if (push &&  wp) d1 <= i_data;", "if (push &&  wp) d1 <= d0;", "caught", "prove"),

    # ---- axi_mux -------------------------------------------------------------
    ("axi_mux", "control", "axi_mux.v",
     "// log2 of the byte width: 256 bits -> 5", "// log2 of the byte width (control)", "survives", "prove"),
    # AR granted without looking at the read FIFO's credits
    ("axi_mux", "ar_no_credit", "axi_mux.v",
     "ok_q <= m_arvalid[g] && !ar_rdy_q[g] &&\n                            (({{(9-RQ_LOG2){1'b0}}, res} + {2'b00, m_arlen[g*8 +: 8]} + 10'd1) <= RQD_L);",
     "ok_q <= m_arvalid[g] && !ar_rdy_q[g];", "caught", "prove"),
    # credit decided in the cycle the previous reservation is not booked yet
    ("axi_mux", "ar_credit_race", "axi_mux.v",
     "ok_q <= m_arvalid[g] && !ar_rdy_q[g] &&", "ok_q <= m_arvalid[g] &&", "caught", "prove"),
    # AWREADY pulse not masked from the next grant: an equivalent mutant - the
    # grant also waits for !aw_act, which is still set in the pulse cycle
    # (the proof shows the mask is redundant, not wrong)
    ("axi_mux", "aw_regrant", "axi_mux.v",
     "(|(m_awvalid & ~aw_rdy_q)) && !wq_full", "(|m_awvalid) && !wq_full", "survives", "prove"),
    # AW address latched from master 0 whoever was granted
    ("axi_mux", "aw_addr_port0", "axi_mux.v",
     "aw_addr_r <= m_awaddr[aw_pick*AW +: AW];", "aw_addr_r <= m_awaddr[0 +: AW];", "caught", "prove"),
    # W-order FIFO popped on every beat, not on WLAST
    ("axi_mux", "wq_pop_each_beat", "axi_mux.v",
     "if (u_whs && u_wlast) wq_rd <= wq_rd + 1'b1;", "if (u_whs) wq_rd <= wq_rd + 1'b1;", "caught", "prove"),
    # W-order FIFO full ignored
    ("axi_mux", "wq_full_ignored", "axi_mux.v",
     "(|(m_awvalid & ~aw_rdy_q)) && !wq_full", "(|(m_awvalid & ~aw_rdy_q))", "caught", "prove"),
    # B shown to every master
    ("axi_mux", "b_broadcast", "axi_mux.v",
     "assign m_bvalid[g] = bo_v && (bo_id == g);", "assign m_bvalid[g] = bo_v;", "caught", "prove"),
    # B skid buffer overwritten
    ("axi_mux", "b_skid_overwrite", "axi_mux.v",
     "assign s_bready = !bs_v;", "assign s_bready = 1'b1;", "caught", "prove"),
    # read FIFO's q stage overwritten before it is taken
    ("axi_mux", "r_q_overwrite", "axi_mux.v",
     "wire                ld     = (!qv || pop) && ram_ne;", "wire                ld     = ram_ne;", "caught", "prove"),

    # the same two from reset, as a real trace (BMC), not only a broken invariant
    ("axi_mux", "ar_no_credit_bmc", "axi_mux.v",
     "ok_q <= m_arvalid[g] && !ar_rdy_q[g] &&\n                            (({{(9-RQ_LOG2){1'b0}}, res} + {2'b00, m_arlen[g*8 +: 8]} + 10'd1) <= RQD_L);",
     "ok_q <= m_arvalid[g] && !ar_rdy_q[g];", "caught", "bmc"),

    # ---- cam_axi_wr (small: same margin arithmetic, scaled queues) -----------
    ("cam_axi_wr", "control", "cam_axi_wr.v",
     "// AW queue: 32 bursts", "// AW queue: 32 bursts (control)", "survives", "small"),
    # the mutant of ERRORS 1.24: room always granted
    ("cam_axi_wr", "room_stuck_1", "cam_axi_wr.v",
     "s_room <= (wq_cnt <= WQ_ROOM) && (aq_cnt <= AQ_ROOM);", "s_room <= 1'b1;", "caught", "small"),
    ("cam_axi_wr", "room_stuck_1_bmc", "cam_axi_wr.v",
     "s_room <= (wq_cnt <= WQ_ROOM) && (aq_cnt <= AQ_ROOM);", "s_room <= 1'b1;", "caught", "bmc"),
    # beat-FIFO margin 3 beats short
    ("cam_axi_wr", "wq_margin_-1", "cam_axi_wr.v",
     "WQ_ROOM_I = WQD - MAXB - 4;", "WQ_ROOM_I = WQD - MAXB - 1;", "caught", "small"),
    # beat-FIFO margin 2 beats short: the proof shows -2 is still enough
    ("cam_axi_wr", "wq_margin_-2", "cam_axi_wr.v",
     "WQ_ROOM_I = WQD - MAXB - 4;", "WQ_ROOM_I = WQD - MAXB - 2;", "survives", "small"),
    # AW-queue margin one entry short
    ("cam_axi_wr", "aq_margin_-4", "cam_axi_wr.v",
     "AQ_ROOM_I = AQD - 2 - 3;", "AQ_ROOM_I = AQD - 2 - 2;", "caught", "small"),
    # AW_AHEAD ignored (unlimited AWs, the 3-of-12 Ethernet freeze, ERRORS 1.24)
    ("cam_axi_wr", "ahead_ignored", "cam_axi_wr.v",
     "aq_any && (ahead < AHEAD_MAX);", "aq_any;", "caught", "small"),
    # no split at a 4 KB page
    ("cam_axi_wr", "no_page_split", "cam_axi_wr.v",
     "p_page  <= &baddr_cur[11-LG-2:0];", "p_page  <= 1'b0;", "caught", "small"),
    # AWLEN one too long
    ("cam_axi_wr", "awlen_+1", "cam_axi_wr.v",
     "{ch_first, ch_cnt - 8'd1};", "{ch_first, ch_cnt};", "caught", "small"),
    # first word always to lane 0
    ("cam_axi_wr", "lane_from_0", "cam_axi_wr.v",
     "wire [LG-1:0]   lane_cur  = s_first ? s_addr[LG+1:2] : lane;",
     "wire [LG-1:0]   lane_cur  = s_first ? {LG{1'b0}} : lane;", "caught", "small"),
    # lane mask not cleared between beats: stale strobes
    ("cam_axi_wr", "mask_sticky", "cam_axi_wr.v",
     "msk   <= (p_vld ? {WPB{1'b0}} : msk) |", "msk   <= msk |", "caught", "small"),
    # output register overwritten under a stalled WVALID
    ("cam_axi_wr", "w_overwrite", "cam_axi_wr.v",
     "wire            o_free  = !o_vld || w_take;", "wire            o_free  = 1'b1;", "caught", "small"),
    # AW reloaded while the previous one waits for AWREADY
    ("cam_axi_wr", "aw_overwrite", "cam_axi_wr.v",
     "wire            aw_load = (!m_awvalid || m_awready) && aq_any",
     "wire            aw_load = aq_any", "caught", "small"),
]


def sby_files(sby_text):
    m = re.search(r"^\[files\]\s*\n(.*?)(?=^\[|\Z)", sby_text, re.M | re.S)
    out = []
    for line in (m.group(1) if m else "").splitlines():
        line = line.split("#")[0].strip()
        if line:
            out.append(line)
    return out


def run_one(job, name, fname, old, new, task):
    sby_path = os.path.join(HERE, job + ".sby")
    with open(sby_path, encoding="utf-8") as f:
        sby_text = f.read()
    files = sby_files(sby_text)
    d = os.path.join(WORK, job, name)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    hit = 0
    for rel in files:
        src = os.path.normpath(os.path.join(HERE, rel))
        base = os.path.basename(rel)
        with open(src, encoding="utf-8") as f:
            text = f.read()
        if base == fname:
            hit = text.count(old)
            if hit != 1:
                return "BAD-MUTANT (%d matches)" % hit
            text = text.replace(old, new)
        with open(os.path.join(d, base), "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    if hit != 1:
        return "BAD-MUTANT (file %s not in [files])" % fname
    # same job, files referenced by basename from the scratch dir
    new_files = "\n".join(os.path.basename(r) for r in files) + "\n"
    sby_text = re.sub(r"(^\[files\]\s*\n)(.*?)(?=^\[|\Z)", lambda m: m.group(1) + new_files,
                      sby_text, flags=re.M | re.S)
    # sby's own time limit: a mutant that neither passes nor fails in time
    # comes back TIMEOUT (reported, never counted as caught)
    sby_text = sby_text.replace("[options]\n", "[options]\ntimeout %d\n" % TIMEOUT_S, 1)
    with open(os.path.join(d, job + ".sby"), "w", encoding="utf-8", newline="\n") as f:
        f.write(sby_text)
    env = dict(os.environ)
    env["YOSYSHQ_ROOT"] = OSS + "\\"
    env["PATH"] = os.path.join(OSS, "bin") + ";" + os.path.join(OSS, "lib") + ";" + env["PATH"]
    env["PYTHON_EXECUTABLE"] = os.path.join(OSS, "lib", "python3.exe")
    env.pop("PYTHONIOENCODING", None)
    subprocess.run([os.path.join(OSS, "bin", "sby.exe"), "-f", "--prefix",
                    os.path.join(d, "w"), job + ".sby", task],
                   cwd=d, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    st = os.path.join(d, "w_" + task, "status")
    if not os.path.exists(st):
        return "ERROR"
    with open(st) as f:
        return f.read().split()[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("job", nargs="?")
    ap.add_argument("-k", default="")
    a = ap.parse_args()
    bad = 0
    for job, name, fname, old, new, expect, task in MUTANTS:
        if a.job and job != a.job:
            continue
        if a.k and a.k not in name:
            continue
        t0 = time.time()
        st = run_one(job, name, fname, old, new, task)
        # FAIL: a counterexample from reset (base case or BMC);
        # UNKNOWN: the induction step fails - the mutant breaks an invariant
        # the real design keeps. Both count as caught; the column says which.
        how = ""
        if st == "PASS":
            got = "survives"
        elif st in ("FAIL", "UNKNOWN"):
            got = "caught"
            how = "trace" if st == "FAIL" else "induct"
        else:
            got = st
        ok = got == expect
        bad += not ok
        print("%-12s %-16s %-10s %-7s expected %-9s %s %6.1f s"
              % (job, name, got, how, expect, "ok " if ok else "BAD", time.time() - t0))
        sys.stdout.flush()
    return bad


if __name__ == "__main__":
    sys.exit(main())
