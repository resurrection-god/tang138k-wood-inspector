#!/usr/bin/env python3
"""
Live FPGA verification tool for Headless Wood Defect Inspector ("WD01" UDP :5555).

Uploads held-out real lumber samples (tools/npu/samples/real_*.png) AND procedural
synthetic pine/spruce/oak boards over UDP :5555 to Tang Mega 138K Pro, reads back
the 16x6x5 INT16 logits from SIMD-16 NPU v3.0.0, and checks bit-exact parity against
the exact firmware/RTL weights parsed from firmware/src/app/npu_weights.h.
"""

import argparse
import pathlib
import random
import re
import socket
import struct
import sys
import time

import numpy as np
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import train_wood_fcn as twf  # noqa: E402

WEIGHTS_H = ROOT / "firmware" / "src" / "app" / "npu_weights.h"
IN_W, IN_H = 256, 96
GRID_W, GRID_H = 16, 6
NUM_CLASSES = 5
CLASS_NAMES = ["CLEAN", "LIVE_KNOT", "DEAD_KNOT", "CRACK", "RESIN_STAIN"]
GRADE_NAMES = ["GRADE_A (CLEAR)", "GRADE_B (LIGHT)", "GRADE_C (MEDIUM)", "REJECT"]


def wd_ping(sock: socket.socket, host: str, port: int) -> dict:
    sock.sendto(b"WD01\x01\x00\x00\x00", (host, port))
    data, _ = sock.recvfrom(128)
    if len(data) < 20 or data[:4] != b"WD01" or data[4] != 0x81:
        raise RuntimeError(f"Bad ping reply: {data!r}")
    infer_cnt, rx_cnt, hw_hash = struct.unpack_from("<HHI", data, 12)
    return {
        "npu_ok": data[5],
        "grid_w": data[6],
        "grid_h": data[7],
        "num_classes": data[8],
        "simd": data[9],
        "infer_cnt": infer_cnt,
        "rx_cnt": rx_cnt,
        "hw_hash": hw_hash,
    }


def wd_infer_rgb(sock: socket.socket, host: str, port: int, rgb_hwc: np.ndarray) -> dict:
    raw = rgb_hwc.astype(np.uint8).tobytes()
    assert len(raw) == IN_W * IN_H * 3
    chunk = 1280
    for off in range(0, len(raw), chunk):
        part = raw[off : off + chunk]
        flags = 1 if (off + len(part) >= len(raw)) else 0
        hdr = b"WD01" + struct.pack("<BIHB", 0x04, off, len(part), flags)
        for attempt in range(4):
            sock.sendto(hdr + part, (host, port))
            try:
                ack, _ = sock.recvfrom(64)
                if len(ack) >= 12 and ack[:4] == b"WD01" and ack[4] == 0x84:
                    break
            except socket.timeout:
                if attempt == 3:
                    raise

    sock.sendto(b"WD01\x07\x00\x00\x00", (host, port))
    rep, _ = sock.recvfrom(2048)
    if len(rep) < 24 + 96 + 96 + 960 or rep[:4] != b"WD01" or rep[4] != 0x87:
        raise RuntimeError(f"Unexpected WD_CMD_NPU_QUERY reply len={len(rep)}")

    infer_cnt, = struct.unpack_from("<H", rep, 8)
    exec_ns, mac_cnt, wall_us = struct.unpack_from("<III", rep, 12)
    classes = np.frombuffer(rep, dtype=np.uint8, count=96, offset=24).reshape(GRID_H, GRID_W)
    confs = np.frombuffer(rep, dtype=np.uint8, count=96, offset=120).reshape(GRID_H, GRID_W)
    logits = np.frombuffer(rep, dtype=np.int16, count=96 * NUM_CLASSES, offset=216).reshape(GRID_H, GRID_W, NUM_CLASSES)
    return {
        "npu_ok": rep[5],
        "grade": rep[11],
        "infer_cnt": infer_cnt,
        "exec_ns": exec_ns,
        "mac_cnt": mac_cnt,
        "wall_us": wall_us,
        "classes": classes,
        "confs": confs,
        "logits": logits,
    }


def _parse_header_arrays(path: pathlib.Path):
    txt = path.read_text(encoding="utf-8")

    def def_u32(name: str) -> int:
        m = re.search(rf"#define\s+{name}\s+(\d+)U?", txt)
        return int(m.group(1))

    def parse_arr(name: str, dtype) -> np.ndarray:
        m = re.search(rf"static const \S+ {name}\[\d+\]\s*=\s*\{{(.*?)\}};", txt, re.S)
        vals = [int(tok.rstrip("Uu"), 0) for tok in re.findall(r"-?(?:0x[0-9A-Fa-f]+|\d+)U?", m.group(1))]
        return np.array(vals, dtype=dtype)

    m_blks = re.search(r"k_wood_blocks\[\d+\]\s*=\s*\{(.*?)\};", txt, re.S)
    blocks = []
    for line in re.findall(r"\{([^}]+)\}", m_blks.group(1)):
        blocks.append([int(x.strip()) for x in line.split(",")])

    wgt_pool = parse_arr("k_npu_wgt_pool", np.uint32)
    npu_b_pool = parse_arr("k_npu_bias_pool", np.int32)
    dw_w_pool = parse_arr("k_dw_wgt_pool", np.int8)
    dw_b_pool = parse_arr("k_dw_bias_pool", np.int32)
    return def_u32, blocks, wgt_pool, npu_b_pool, dw_w_pool, dw_b_pool


def _unpack_pw(wgt_pool: np.ndarray, wo: int, out_ch: int, in_ch: int) -> np.ndarray:
    vec_quads = (in_ch + 15) // 16
    stride_words = vec_quads * 4
    sub = wgt_pool[wo : wo + out_ch * stride_words].reshape(out_ch, stride_words)
    bytes_mat = sub.view(np.uint8).reshape(out_ch, vec_quads * 16).astype(np.int8)
    return bytes_mat[:, :in_ch].copy()


def emulate_from_header(rgb_hwc: np.ndarray, header_path: pathlib.Path = WEIGHTS_H) -> np.ndarray:
    def_u32, blocks, wgt_pool, npu_b_pool, dw_w_pool, dw_b_pool = _parse_header_arrays(header_path)

    def pw_layer(x_hwc: np.ndarray, wo: int, bo: int, out_ch: int, shift: int, relu: bool, clamp_i8: bool):
        h, w, c_in = x_hwc.shape
        w_mat = _unpack_pw(wgt_pool, wo, out_ch, c_in).astype(np.int64)
        b_vec = npu_b_pool[bo : bo + out_ch].astype(np.int64)
        acc = x_hwc.reshape(-1, c_in).astype(np.int64) @ w_mat.T + b_vec
        return twf.hw_round_shift(acc, shift, relu=relu, clamp_i8=clamp_i8).reshape(h, w, out_ch)

    def dw_layer(x_hwc: np.ndarray, wo: int, bo: int, shift: int, stride: int, dil: int):
        h, w, ch = x_hwc.shape
        oh, ow = h // stride, w // stride
        pad = dil
        xp = np.pad(x_hwc.astype(np.int64), ((pad, pad), (pad, pad), (0, 0)), mode="constant")
        kw = dw_w_pool[wo : wo + ch * 9].reshape(ch, 3, 3).astype(np.int64)
        kb = dw_b_pool[bo : bo + ch].astype(np.int64)
        acc = np.zeros((oh, ow, ch), dtype=np.int64) + kb
        for ky in range(3):
            for kx in range(3):
                ys = ky * dil
                xs = kx * dil
                acc += xp[ys : ys + oh * stride : stride, xs : xs + ow * stride : stride, :] * kw[:, ky, kx]
        return twf.hw_round_shift(acc, shift, relu=False, clamp_i8=True).astype(np.int8)

    in_i8 = np.clip(rgb_hwc.astype(np.int32) - 128, -128, 127).astype(np.int8)
    # Conv1 3x3 stride 2 (256x96 -> 128x48x24)
    w1 = _unpack_pw(wgt_pool, def_u32("NPU_CONV1_WO"), 24, 27).astype(np.int64)
    b1 = npu_b_pool[def_u32("NPU_CONV1_BO") : def_u32("NPU_CONV1_BO") + 24].astype(np.int64)
    pad = np.pad(in_i8.astype(np.int64), ((1, 1), (1, 1), (0, 0)), mode="constant")
    patches = np.zeros((48 * 128, 27), dtype=np.int64)
    p = 0
    for ky in range(3):
        for kx in range(3):
            patches[:, p : p + 3] = pad[ky : ky + 96 : 2, kx : kx + 256 : 2, :].reshape(-1, 3)
            p += 3
    x = twf.hw_round_shift(patches @ w1.T + b1, def_u32("NPU_CONV1_SHIFT"), relu=True, clamp_i8=True).reshape(48, 128, 24).astype(np.int8)

    # MaxPool 3x3 stride 2 -> 64x24x24
    pad = np.pad(x, ((1, 1), (1, 1), (0, 0)), mode="constant")
    mp = np.zeros((24, 64, 24), dtype=np.int8)
    for ky in range(3):
        for kx in range(3):
            mp = np.maximum(mp, pad[ky : ky + 48 : 2, kx : kx + 128 : 2, :])
    x = mp

    for blk in blocks:
        (two_br, dw_s, dw_d, _hi, _wi, c_in, c_br,
         b1_dw_wo, b1_dw_bo, b1_dws, b1_pw_wo, b1_pw_bo, b1_pws,
         b2_pw1_wo, b2_pw1_bo, b2_pw1s, b2_dw_wo, b2_dw_bo, b2_dws, b2_pw2_wo, b2_pw2_bo, b2_pw2s) = blk
        if two_br:
            br1 = pw_layer(dw_layer(x, b1_dw_wo, b1_dw_bo, b1_dws, dw_s, dw_d), b1_pw_wo, b1_pw_bo, c_br, b1_pws, True, True).astype(np.int8)
            br2_in = x
        else:
            br1 = x[..., :c_br]
            br2_in = x[..., c_br:]
        t = pw_layer(br2_in, b2_pw1_wo, b2_pw1_bo, c_br, b2_pw1s, True, True).astype(np.int8)
        t = dw_layer(t, b2_dw_wo, b2_dw_bo, b2_dws, dw_s, dw_d)
        br2 = pw_layer(t, b2_pw2_wo, b2_pw2_bo, c_br, b2_pw2s, True, True).astype(np.int8)
        cat = np.concatenate([br1, br2], axis=-1)
        ch, cw, cc = cat.shape
        x = cat.reshape(ch, cw, 2, cc // 2).swapaxes(2, 3).reshape(ch, cw, cc)

    x = pw_layer(x, def_u32("NPU_CONV5_WO"), def_u32("NPU_CONV5_BO"), 128, def_u32("NPU_CONV5_SHIFT"), True, True).astype(np.int8)
    logits = pw_layer(x, def_u32("NPU_HEAD_WO"), def_u32("NPU_HEAD_BO"), NUM_CLASSES, def_u32("NPU_HEAD_SHIFT"), False, False)
    return logits


def summarize_grid(classes: np.ndarray) -> str:
    counts = {CLASS_NAMES[c]: int((classes == c).sum()) for c in range(NUM_CLASSES) if (classes == c).any()}
    return ", ".join(f"{k}:{v}" for k, v in counts.items())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="10.0.0.222")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--synth", type=int, default=4)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.5)

    ping = wd_ping(sock, args.host, args.port)
    print(
        f"[PING] {args.host}:{args.port} npu_ok={ping['npu_ok']} grid={ping['grid_w']}x{ping['grid_h']}x{ping['num_classes']} "
        f"SIMD-{ping['simd']} hash=0x{ping['hw_hash']:08X} infers={ping['infer_cnt']}"
    )
    assert ping["npu_ok"] == 1, "FPGA NPU reported npu_ok=0 (MAP_HASH mismatch?)"
    assert ping["simd"] == 16, f"Expected SIMD-16, got {ping['simd']}"

    cases = []
    for p in sorted((ROOT / "tools" / "npu" / "samples").glob("real_*.png")):
        im = Image.open(p).convert("RGB").resize((IN_W, IN_H), Image.Resampling.BILINEAR)
        cases.append((p.name, np.asarray(im, dtype=np.uint8)))

    rng = random.Random(20261118)
    for idx in range(args.synth):
        im, boxes = twf.generate_synthetic_plank(rng, w=IN_W, h=IN_H)
        gt_cls = [CLASS_NAMES[b[0]] for b in boxes]
        cases.append((f"synth_{idx}_{'+'.join(gt_cls) if gt_cls else 'CLEAN'}", np.asarray(im, dtype=np.uint8)))

    mismatches = 0
    for name, rgb in cases:
        t0 = time.perf_counter()
        res = wd_infer_rgb(sock, args.host, args.port, rgb)
        rtt_ms = (time.perf_counter() - t0) * 1000.0
        gmac_s = res["mac_cnt"] / max(1, res["exec_ns"])
        emu_logits = emulate_from_header(rgb, WEIGHTS_H)
        max_diff = int(np.max(np.abs(res["logits"].astype(np.int32) - emu_logits.astype(np.int32))))
        if max_diff != 0:
            mismatches += 1
        print(
            f"[{name}] grade={GRADE_NAMES[min(3, res['grade'])]} | {summarize_grid(res['classes'])} | "
            f"npu={res['exec_ns']/1e6:.2f}ms wall={res['wall_us']/1e3:.2f}ms ({gmac_s:.3f} GMAC/s, RTT={rtt_ms:.1f}ms) | "
            f"max_logit_diff={max_diff}"
        )

    print(f"\n[SUMMARY] Checked {len(cases)} planks on live FPGA: bit-exact mismatches = {mismatches}/{len(cases)}")
    if mismatches > 0:
        sys.exit(2)


if __name__ == "__main__":
    main()
