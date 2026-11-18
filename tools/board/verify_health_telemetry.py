#!/usr/bin/env python3
"""
verify_health_telemetry.py - Automated hardware health, telemetry deltas, and Ethernet RX verification.

Addresses:
- Open Issue 1.1 (Ethernet RX stability, PLIC gateway health, sick-boot detection)
- Open Issue 1.8 (Incremental telemetry snapshots without requiring FPGA resets)
- Decision R-33 / R-36 verification on live hardware

Output conforms to AGENTS.md P0-1, P0-3, P0-4, P0-5.
"""

import sys
import os
import time
import json
import argparse
from typing import Dict, Any, List

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from tools.common import vmsp

def sample_status(host: str, samples: int = 5, interval_s: float = 0.5) -> List[Dict[str, Any]]:
    records = []
    for i in range(samples):
        st = vmsp.status(host)
        t_now_s = time.time()
        st["_timestamp_s"] = t_now_s
        records.append(st)
        if i < samples - 1:
            time.sleep(interval_s)
    return records

def compute_deltas(s_first: Dict[str, Any], s_last: Dict[str, Any]) -> Dict[str, Any]:
    dt_s = s_last["_timestamp_s"] - s_first["_timestamp_s"]
    
    # 32-bit wrap-around safe difference
    def diff32(k: str) -> int:
        v1 = s_first.get(k, 0)
        v2 = s_last.get(k, 0)
        return (v2 - v1) & 0xFFFFFFFF

    # 16-bit wrap-around safe difference
    def diff16(k: str) -> int:
        v1 = s_first.get(k, 0)
        v2 = s_last.get(k, 0)
        return (v2 - v1) & 0xFFFF

    timer_ticks_delta = diff32("fpga_timer")
    # FPGA timer frequency: 100 MHz (10 ns per tick, ERRORS 1.11, ARCHITECTURE.md §6)
    measured_dt_timer_s = timer_ticks_delta / 100_000_000.0

    return {
        "duration_wall_s": round(dt_s, 4),
        "fpga_timer_ticks_delta": timer_ticks_delta,
        "fpga_timer_dt_s": round(measured_dt_timer_s, 4),
        "fpga_frames0_delta": diff32("fpga_frames"),
        "fpga_frames1_delta": diff32("fpga_frames1"),
        "cam0_triggers_delta": diff32("cam_triggers"),
        "cam1_triggers_delta": diff32("cam1_triggers"),
        "cam0_lost_delta": diff32("cam_lost"),
        "cam1_lost_delta": diff32("cam1_lost"),
        "accepted_delta": diff32("accepted"),
        "overrun_delta": diff32("overrun"),
    }

def main():
    parser = argparse.ArgumentParser(description="Tang Mega 138K Hardware Telemetry & Health Audit")
    parser.add_argument("--host", default="10.0.0.88", help="Target IP address (default: 10.0.0.88)")
    parser.add_argument("--samples", type=int, default=10, help="Number of telemetry samples")
    parser.add_argument("--interval", type=float, default=0.5, help="Interval between samples in seconds")
    parser.add_argument("--json", action="store_true", help="Print output as JSON")
    args = parser.parse_args()

    if not args.json:
        print(f"=== [TELEMETRY AUDIT] Опрос аппаратного состояния {args.host} ===")
        print(f"  Сэмплов: {args.samples}, интервал: {args.interval} с")

    t_start = time.time()
    samples = sample_status(args.host, samples=args.samples, interval_s=args.interval)
    t_end = time.time()

    first_st = samples[0]
    last_st = samples[-1]

    deltas = compute_deltas(first_st, last_st)

    # Ethernet RX health analysis
    eth_rx_wd = last_st.get("eth_rx_wd", 0)
    recent_lost = eth_rx_wd & 0xFFFF
    total_lost = (eth_rx_wd >> 16) & 0xFFFF
    is_sick = recent_lost >= vmsp.ETH_SICK_RECENT

    # OTA Slot analysis
    ota_st = vmsp.ota_status(args.host)
    slot_name = ota_st.get("slot_name", "Unknown")

    audit_result = {
        "target_host": args.host,
        "active_slot": slot_name,
        "slot_b_crc32": f"0x{ota_st.get('slot_b_crc', 0):08X}",
        "build_id": f"0x{last_st.get('mode_bits', 0):08X}",
        "ethernet_rx": {
            "recent_lost_interrupts_20s": recent_lost,
            "total_lost_interrupts": total_lost,
            "status": "SICK" if is_sick else "HEALTHY",
            "threshold_sick": vmsp.ETH_SICK_RECENT,
            "plic_fast_poll_active": total_lost > 0
        },
        "telemetry_deltas": deltas,
        "telemetry_snapshot_last": {
            "sync_status": last_st.get("sync_status"),
            "sync_ctrl": last_st.get("sync_ctrl"),
            "cam0_width_ns": last_st.get("cam0_width"),
            "cam1_width_ns": last_st.get("cam1_width"),
            "laser_width_ns": last_st.get("laser_width"),
            "laser_mode": last_st.get("laser_mode"),
            "ext_filter_ns": last_st.get("ext_filter"),
            "telemetry_ok": bool(last_st.get("telemetry_ok")),
        },
        "verdict": "PASS" if not is_sick and deltas["overrun_delta"] == 0 else "FAIL"
    }

    if args.json:
        print(json.dumps(audit_result, indent=2))
        return 0

    print("------------------------------------------------------------")
    print(f" Активный слот прошивки: {audit_result['active_slot']}")
    print(f" CRC32 слота B:          {audit_result['slot_b_crc32']}")
    print(f" Состояние Ethernet RX:  {audit_result['ethernet_rx']['status']} (потерь за 20 с: {recent_lost}, всего: {total_lost})")
    print(f" Дельта таймера FPGA:    {deltas['fpga_timer_ticks_delta']} тактов ({deltas['fpga_timer_dt_s']} с)")
    print(f" Астрономическое время:  {deltas['duration_wall_s']} с")
    print(f" Переполнений (overrun): {deltas['overrun_delta']}")
    print(f" Потерь кадров (lost):   CAM0: {deltas['cam0_lost_delta']}, CAM1: {deltas['cam1_lost_delta']}")
    print(f" ИТОГОВЫЙ ВЕРДИКТ:       [{audit_result['verdict']}]")
    print("------------------------------------------------------------")

    return 0 if audit_result["verdict"] == "PASS" else 1

if __name__ == "__main__":
    sys.exit(main())
