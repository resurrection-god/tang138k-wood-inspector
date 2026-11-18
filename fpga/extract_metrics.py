import json
import pathlib
import re
import subprocess
import zlib


def main():
    root = pathlib.Path(__file__).resolve().parent.parent
    rpt = (root / "fpga/impl/pnr/scanner.rpt.txt").read_text(encoding="utf-8", errors="replace")
    tr = (root / "fpga/impl/pnr/scanner.tr").read_text(encoding="utf-8", errors="replace")
    bid_txt = (root / "fpga/src/build_id.v").read_text(encoding="utf-8", errors="replace")
    fw_bin = root / "firmware/Release/output/ae350_scanner.bin"
    fw_bytes = fw_bin.read_bytes()
    fw_crc32 = zlib.crc32(fw_bytes) & 0xFFFFFFFF

    m_bid = re.search(r"32'h([0-9A-Fa-f_]+)", bid_txt)
    build_id = m_bid.group(1).replace("_", "") if m_bid else None

    def find_res(pat):
        m = re.search(pat, rpt)
        return m.groups() if m else None

    logic_m = find_res(r"Logic\s*\|\s*(\d+)/(\d+)\s*\|\s*(\d+)%")
    reg_m = find_res(r"Register\s*\|\s*(\d+)/(\d+)\s*\|\s*(\d+)%")
    bsram_m = find_res(r"BSRAM\s*\|\s*(\d+)/(\d+)\s*\|\s*(\d+)%")
    dsp_m = find_res(r"DSP\s*\|\s*(\d+)/(\d+)\s*\|\s*(\d+)%")
    pll_m = find_res(r"PLL\s*\|\s*(\d+)/(\d+)\s*\|\s*(\d+)%")
    prim_m = find_res(r"PRIMARY\s*\|\s*(\d+)/(\d+)\s*\|\s*(\d+)%")

    res = subprocess.run(
        [
            "powershell",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(root / "fpga/check_timing.ps1"),
            "-Report",
            str(root / "fpga/impl/pnr/scanner_tr_content.html"),
        ],
        capture_output=True,
        text=True,
    )

    clocks = {}
    for line in res.stdout.splitlines():
        m = re.match(
            r"^\s*(\S+)\s+constraint\s+([\d,.\xa0\s]+?)\s+actual\s+([\d,.\xa0\s]+?)\s+margin\s+([-\d,.\xa0\s]+?)\s+slack\s+([-\d,.]+)\s*ns\*?\s+(PASS|FAIL)",
            line,
        )
        if m:
            name, constr, actual, margin, slack, status = m.groups()
            def to_f(s):
                return float(s.replace("\xa0", "").replace(" ", "").replace(",", "."))
            clocks[name] = {
                "constraint_mhz": to_f(constr),
                "actual_fmax_mhz": to_f(actual),
                "margin_mhz": to_f(margin),
                "setup_slack_ns": to_f(slack),
                "status": status,
            }

    def pf(pat):
        m = re.search(pat, res.stdout)
        return float(m.group(1).replace(",", ".")) if m else None

    fs_path = root / "fpga/impl/pnr/scanner.fs"
    wns_ns = pf(r"worst path slack\s*:\s*([-\d.,]+)\s*ns")
    whs_ns = pf(r"worst hold\s+slack\s*:\s*([-\d.,]+)\s*ns")
    util_pct = int(logic_m[2]) if logic_m else 0
    metrics = {
        "top": "scanner_top",
        "part": "GW5AST-LV138FPG676AC1/I0",
        "wns_ns": wns_ns,
        "whs_ns": whs_ns,
        "util_pct": util_pct,
        "project_dir": str(root),
        "device": "GW5AST-LV138FPG676AC1/I0",
        "fpga_build_id": build_id,
        "npu_map_hash": "0x780EC09F",
        "bitstream_path": str(fs_path),
        "bitstream_bytes": fs_path.stat().st_size,
        "firmware": {
            "path": str(fw_bin),
            "size_bytes": len(fw_bytes),
            "crc32_hex": f"0x{fw_crc32:08X}",
            "ip_addr": "10.0.0.222",
        },
        "pnr_options": {
            "map_option": 1,
            "place_option": 4,
            "route_option": 1,
            "clock_route_order": 1,
            "replicate_resources": True,
            "max_fanout": 200,
        },
        "utilization": {
            "logic": {"used": int(logic_m[0]), "total": int(logic_m[1]), "pct": int(logic_m[2])} if logic_m else None,
            "registers": {"used": int(reg_m[0]), "total": int(reg_m[1]), "pct": int(reg_m[2])} if reg_m else None,
            "bsram": {"used": int(bsram_m[0]), "total": int(bsram_m[1]), "pct": int(bsram_m[2])} if bsram_m else None,
            "dsp": {"used": int(dsp_m[0]), "total": int(dsp_m[1]), "pct": int(dsp_m[2])} if dsp_m else None,
            "pll": {"used": int(pll_m[0]), "total": int(pll_m[1]), "pct": int(pll_m[2])} if pll_m else None,
            "primary_clocks": {"used": int(prim_m[0]), "total": int(prim_m[1]), "pct": int(prim_m[2])} if prim_m else None,
        },
        "timing": {
            "gate_passed": res.returncode == 0,
            "check_timing_exit_code": res.returncode,
            "worst_setup_path_ns": wns_ns,
            "worst_hold_ns": whs_ns,
            "worst_recovery_ns": pf(r"worst recovery\s+slack\s*:\s*([-\d.,]+)\s*ns"),
            "worst_removal_ns": pf(r"worst removal\s+slack\s*:\s*([-\d.,]+)\s*ns"),
            "clocks": clocks,
            "check_timing_summary": res.stdout.strip().splitlines()[-1] if res.stdout.strip() else "",
        },
    }
    out = root / "metrics.json"
    out.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    build_out = root / "fpga/build/metrics.json"
    build_out.parent.mkdir(parents=True, exist_ok=True)
    build_out.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print(out.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
