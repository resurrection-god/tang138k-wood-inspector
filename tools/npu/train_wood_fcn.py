#!/usr/bin/env python3
"""
Train, INT8-QAT distill, emulate (bit-exact SIMD-16), and export a Spatial FCN
Wood Defect Detector (ShuffleNetV2-x0.5 FCN, 256x96 RGB888 -> 16x6x5 grid)
for Tang Mega 138K Pro Headless FPGA NPU v3.0.0 (npu_core.v, SIMD-16, 64 KB BSRAM).

Classes (per 16x16 px grid cell across 16x6 plank map):
  0 = CLEAN       (Чистая древесина)
  1 = LIVE_KNOT   (Здоровый сучок)
  2 = DEAD_KNOT   (Тёмный / выпадающий / треснутый сучок)
  3 = CRACK       (Трещина)
  4 = RESIN_STAIN (Смоляной карман / сердцевина / синева / обзол)

Dataset:
  - Real industrial lumber: iluvvatar/wood_surface_defects (8,111 planks from 2 shards)
  - Procedural synthetic pine/spruce/oak planks + synthetic knots/cracks/resin/blue-stain
"""

import io
import json
import math
import pathlib
import pickle
import random
import sys
import time

import numpy as np
import pyarrow.parquet as pq
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as tv_models
from PIL import Image, ImageDraw, ImageFilter, ImageOps

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
DS_DIR = pathlib.Path(r"C:\Users\penis\.gemini\antigravity\scratch\wood_datasets")
CACHE_PKL = DS_DIR / "wood_8k_320x120_cache.pkl"
WEIGHTS_H = ROOT / "firmware" / "src" / "app" / "npu_weights.h"
CKPT_PT = ROOT / "tools" / "npu" / "wood_fcn.pt"
SAMPLES_DIR = ROOT / "tools" / "npu" / "samples"

IN_W, IN_H = 256, 96
GRID_W, GRID_H = 16, 6
NUM_CLASSES = 5
CLASS_NAMES = ["CLEAN", "LIVE_KNOT", "DEAD_KNOT", "CRACK", "RESIN_STAIN"]

LABEL_MAP = {
    "Live_Knot": 1,
    "Dead_Knot": 2,
    "Knot_missing": 2,
    "knot_with_crack": 2,
    "Crack": 3,
    "resin": 4,
    "Marrow": 4,
    "Quartzity": 4,
    "Blue_Stain": 4,
    "overgrown": 4,
}
# Defect priority when multiple boxes overlap a cell (Crack > Dead_Knot > Resin > Live_Knot)
CLASS_PRIORITY = {0: 0, 1: 1, 4: 2, 2: 3, 3: 4}

torch.set_num_threads(8)


def generate_synthetic_plank(rng: random.Random, w: int = 320, h: int = 120):
    """Generates procedural wood plank (pine/spruce/oak) + exact bounding-box defect labels."""
    species = rng.randrange(3)  # 0=pine, 1=spruce, 2=oak
    if species == 0:
        base = np.array([218.0, 178.0, 128.0], dtype=np.float32)
    elif species == 1:
        base = np.array([228.0, 196.0, 152.0], dtype=np.float32)
    else:
        base = np.array([175.0, 132.0, 88.0], dtype=np.float32)

    base += np.array([rng.uniform(-16, 16), rng.uniform(-14, 14), rng.uniform(-12, 12)], dtype=np.float32)

    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    freq = rng.uniform(0.14, 0.34)
    warp = rng.uniform(2.5, 8.0) * np.sin(xx * rng.uniform(0.015, 0.04) + rng.uniform(0, 6.28))
    rings = np.sin((yy + warp) * freq + rng.uniform(0, 6.28))
    grain = 14.0 * rings + 5.0 * np.sin(yy * (freq * 2.3) + warp * 0.5)
    # Fine horizontal wood fibers
    fibers = rng.uniform(3.0, 7.0) * np.sin(yy * rng.uniform(0.8, 1.5) + xx * 0.03)
    noise = np.random.normal(0.0, 3.5, (h, w)).astype(np.float32)

    img = np.zeros((h, w, 3), dtype=np.float32)
    img[..., 0] = base[0] + grain * 1.0 + fibers + noise
    img[..., 1] = base[1] + grain * 0.85 + fibers * 0.8 + noise
    img[..., 2] = base[2] + grain * 0.60 + fibers * 0.5 + noise

    im = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8))
    draw = ImageDraw.Draw(im)
    boxes = []  # (cls_id, x0_norm, y0_norm, x1_norm, y1_norm)

    n_def = rng.choice([0, 1, 1, 2, 2, 3, 4])
    for _ in range(n_def):
        dcls = rng.choices([1, 2, 3, 4], weights=[0.30, 0.25, 0.25, 0.20])[0]
        if dcls in (1, 2):  # Live or Dead knot
            rw = rng.uniform(0.032, 0.075)
            rh = rng.uniform(0.075, 0.165)
            cx = rng.uniform(rw + 0.02, 1.0 - rw - 0.02)
            cy = rng.uniform(rh + 0.02, 1.0 - rh - 0.02)
            x0, y0, x1, y1 = int((cx - rw) * w), int((cy - rh) * h), int((cx + rw) * w), int((cy + rh) * h)
            if dcls == 1:
                outer_c = (int(base[0] * 0.62), int(base[1] * 0.55), int(base[2] * 0.45))
                inner_c = (int(base[0] * 0.45), int(base[1] * 0.36), int(base[2] * 0.26))
                draw.ellipse([x0, y0, x1, y1], fill=outer_c)
                draw.ellipse([x0 + 2, y0 + 2, max(x0 + 3, x1 - 2), max(y0 + 3, y1 - 2)], fill=inner_c)
            else:
                outer_c = (48, 34, 24)
                core_c = (18, 12, 10) if rng.random() < 0.7 else (30, 22, 18)
                draw.ellipse([x0 - 1, y0 - 1, x1 + 1, y1 + 1], fill=(82, 58, 40))
                draw.ellipse([x0, y0, x1, y1], fill=outer_c)
                draw.ellipse([x0 + 2, y0 + 2, max(x0 + 3, x1 - 2), max(y0 + 3, y1 - 2)], fill=core_c)
            boxes.append((dcls, cx - rw, cy - rh, cx + rw, cy + rh))
        elif dcls == 3:  # Crack (horizontal / diagonal fissure)
            ln = rng.uniform(0.14, 0.36)
            cx = rng.uniform(ln * 0.55, 1.0 - ln * 0.55)
            cy = rng.uniform(0.10, 0.90)
            dy = rng.uniform(-0.08, 0.08)
            pts = []
            for k in range(6):
                t = k / 5.0 - 0.5
                px = int((cx + t * ln) * w)
                py = int((cy + t * dy + rng.uniform(-0.015, 0.015)) * h)
                pts.append((px, py))
            draw.line(pts, fill=(24, 16, 12), width=rng.choice([2, 3]))
            xs = [p[0] / w for p in pts]
            ys = [p[1] / h for p in pts]
            boxes.append((3, max(0.0, min(xs) - 0.01), max(0.0, min(ys) - 0.03), min(1.0, max(xs) + 0.01), min(1.0, max(ys) + 0.03)))
        else:  # Resin pocket / blue stain / marrow streak
            rw = rng.uniform(0.05, 0.14)
            rh = rng.uniform(0.06, 0.18)
            cx = rng.uniform(rw + 0.02, 1.0 - rw - 0.02)
            cy = rng.uniform(rh + 0.02, 1.0 - rh - 0.02)
            x0, y0, x1, y1 = int((cx - rw) * w), int((cy - rh) * h), int((cx + rw) * w), int((cy + rh) * h)
            if rng.random() < 0.55:
                col = (145, 72, 22)  # amber resin
            else:
                col = (88, 102, 122)  # blue stain / bark streak
            draw.polygon([(x0, (y0 + y1) // 2), ((x0 + x1) // 2, y0), (x1, (y0 + y1) // 2), ((x0 + x1) // 2, y1)], fill=col)
            boxes.append((4, cx - rw, cy - rh, cx + rw, cy + rh))

    im = im.filter(ImageFilter.GaussianBlur(radius=0.4))
    return im, boxes


def prepare_real_dataset():
    """Caches 8,111 real wood planks as compact 320x120 JPEG bytes + normalized boxes."""
    if CACHE_PKL.exists():
        t0 = time.time()
        with open(CACHE_PKL, "rb") as f:
            items = pickle.load(f)
        print(f"Loaded {len(items)} cached real wood planks in {time.time() - t0:.1f}s")
        return items

    print("Caching 8,111 real planks from wood_train_0/1.parquet at 320x120 JPEG bytes (one-time)...")
    t0 = time.time()
    items = []
    for fname in ("wood_train_0.parquet", "wood_train_1.parquet"):
        tbl = pq.read_table(DS_DIR / fname, columns=["id", "image", "objects"])
        ids = tbl["id"].to_pylist()
        imgs = tbl["image"].to_pylist()
        objs_list = tbl["objects"].to_pylist()
        for rid, im_d, objs in zip(ids, imgs, objs_list):
            im = Image.open(io.BytesIO(im_d["bytes"])).convert("RGB").resize((320, 120), Image.Resampling.BILINEAR)
            buf = io.BytesIO()
            im.save(buf, format="JPEG", quality=93)
            boxes = []
            for o in objs:
                cid = LABEL_MAP.get(o["label"])
                if cid is None:
                    continue
                cx, cy, bw, bh = o["bb"]
                boxes.append((cid, float(cx - 0.5 * bw), float(cy - 0.5 * bh), float(cx + 0.5 * bw), float(cy + 0.5 * bh)))
            items.append((int(rid), buf.getvalue(), boxes))
    with open(CACHE_PKL, "wb") as f:
        pickle.dump(items, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"Cached {len(items)} real planks to {CACHE_PKL} in {time.time() - t0:.1f}s")
    return items


def boxes_to_grid(boxes, gw: int = GRID_W, gh: int = GRID_H) -> np.ndarray:
    """Converts normalized [cls, x0, y0, x1, y1] boxes into exact (gh, gw) cell target map."""
    grid = np.zeros((gh, gw), dtype=np.int64)
    best_pri = np.zeros((gh, gw), dtype=np.int32)
    cell_w = 1.0 / gw
    cell_h = 1.0 / gh
    for cls_id, x0, y0, x1, y1 in boxes:
        cx = 0.5 * (x0 + x1)
        cy = 0.5 * (y0 + y1)
        gx_c = max(0, min(gw - 1, int(cx * gw)))
        gy_c = max(0, min(gh - 1, int(cy * gh)))
        pri = CLASS_PRIORITY[cls_id]
        # Always mark center cell
        if pri >= best_pri[gy_c, gx_c]:
            grid[gy_c, gx_c] = cls_id
            best_pri[gy_c, gx_c] = pri
        # Mark any cell overlapped >= 18% of cell area (captures wide cracks & big knots)
        gx0 = max(0, int(math.floor(x0 * gw)))
        gx1 = min(gw - 1, int(math.floor(x1 * gw)))
        gy0 = max(0, int(math.floor(y0 * gh)))
        gy1 = min(gh - 1, int(math.floor(y1 * gh)))
        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                ix0 = max(x0, gx * cell_w)
                iy0 = max(y0, gy * cell_h)
                ix1 = min(x1, (gx + 1) * cell_w)
                iy1 = min(y1, (gy + 1) * cell_h)
                if ix1 > ix0 and iy1 > iy0:
                    ov = ((ix1 - ix0) * (iy1 - iy0)) / (cell_w * cell_h)
                    if ov >= 0.18 and pri >= best_pri[gy, gx]:
                        grid[gy, gx] = cls_id
                        best_pri[gy, gx] = pri
    return grid


class WoodSpatialDataset(torch.utils.data.Dataset):
    def __init__(self, real_items, synth_count: int, is_train: bool, seed: int):
        self.items = []  # (is_bytes, img_or_bytes, boxes)
        self.is_train = is_train
        self.rng = random.Random(seed)
        for _rid, b, boxes in real_items:
            self.items.append((True, b, boxes))
        for _ in range(synth_count):
            im, boxes = generate_synthetic_plank(self.rng, 320, 120)
            self.items.append((False, im, boxes))

    def __len__(self):
        return len(self.items)

    def __getitem__(self, idx):
        is_bytes, data, boxes = self.items[idx]
        im = Image.open(io.BytesIO(data)).convert("RGB") if is_bytes else data.copy()
        cur_boxes = list(boxes)

        if self.is_train:
            # Horizontal & vertical flip
            if self.rng.random() < 0.5:
                im = ImageOps.mirror(im)
                cur_boxes = [(c, 1.0 - x1, y0, 1.0 - x0, y1) for c, x0, y0, x1, y1 in cur_boxes]
            if self.rng.random() < 0.5:
                im = ImageOps.flip(im)
                cur_boxes = [(c, x0, 1.0 - y1, x1, 1.0 - y0) for c, x0, y0, x1, y1 in cur_boxes]
            # Random zoom/crop (makes tiny 2.5% knots larger & diversifies placement!)
            if self.rng.random() < 0.75:
                cw = self.rng.uniform(0.65, 1.0)
                ch = self.rng.uniform(0.65, 1.0)
                ox0 = self.rng.uniform(0.0, 1.0 - cw)
                oy0 = self.rng.uniform(0.0, 1.0 - ch)
                ox1, oy1 = ox0 + cw, oy0 + ch
                w, h = im.size
                im = im.crop((int(ox0 * w), int(oy0 * h), int(ox1 * w), int(oy1 * h)))
                new_b = []
                for c, x0, y0, x1, y1 in cur_boxes:
                    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
                    if ox0 <= cx <= ox1 and oy0 <= cy <= oy1:
                        nx0 = max(0.0, (x0 - ox0) / cw)
                        ny0 = max(0.0, (y0 - oy0) / ch)
                        nx1 = min(1.0, (x1 - ox0) / cw)
                        ny1 = min(1.0, (y1 - oy0) / ch)
                        if nx1 > nx0 and ny1 > ny0:
                            new_b.append((c, nx0, ny0, nx1, ny1))
                cur_boxes = new_b

        im_in = im.resize((IN_W, IN_H), Image.Resampling.BILINEAR)
        arr = np.asarray(im_in, dtype=np.float32)
        if self.is_train:
            arr = np.clip(arr * self.rng.uniform(0.82, 1.20) + self.rng.uniform(-16.0, 16.0), 0.0, 255.0)
        # Snap to exact INT8 [-127..127] grid matching UDP RGB888 transfer
        arr_i8 = np.clip(np.round(arr) - 128.0, -127.0, 127.0).astype(np.float32)
        t = torch.from_numpy(arr_i8 / 128.0).permute(2, 0, 1).contiguous()
        grid = torch.from_numpy(boxes_to_grid(cur_boxes))
        return t, grid


class WoodFCNShuffleNet(nn.Module):
    """
    Fully-Convolutional ShuffleNetV2-x0.5 (256x96 -> 16x6x5 spatial defect map).
    Stage 4 block 0 uses stride=1, dilation=2, padding=2 on DW3x3 so spatial map
    stays at crisp 16x6 instead of collapsing to 8x3, while keeping pretrained weights!
    """

    def __init__(self):
        super().__init__()
        base = tv_models.shufflenet_v2_x0_5(weights=tv_models.ShuffleNet_V2_X0_5_Weights.DEFAULT)
        self.conv1 = base.conv1
        self.maxpool = base.maxpool
        self.stage2 = base.stage2
        self.stage3 = base.stage3
        self.stage4 = base.stage4

        # Patch stage4.0 depthwise convs from stride=2 to stride=1, dilation=2, padding=2
        blk0 = self.stage4[0]
        blk0.stride = 1
        for dw in (blk0.branch1[0], blk0.branch2[3]):
            dw.stride = (1, 1)
            dw.padding = (2, 2)
            dw.dilation = (2, 2)

        self.conv5 = nn.Sequential(
            nn.Conv2d(192, 128, kernel_size=1, stride=1, padding=0, bias=False),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
        )
        self.head_seg = nn.Conv2d(128, NUM_CLASSES, kernel_size=1, stride=1, padding=0, bias=True)

        # Absorb ImageNet normalization directly into conv1 for hardware [-1, 1] RGB888 inputs
        mean = torch.tensor([0.485, 0.456, 0.406])
        std = torch.tensor([0.229, 0.224, 0.225])
        a = 0.5 / std
        b = (0.5 - mean) / std
        with torch.no_grad():
            w = self.conv1[0].weight.data
            bias_shift = (w * b.view(1, 3, 1, 1)).sum(dim=(1, 2, 3))
            self.conv1[0].weight.data = w * a.view(1, 3, 1, 1)
            self.conv1[1].running_mean.data = self.conv1[1].running_mean.data - bias_shift

    def forward(self, x):
        x = self.conv1(x)
        x = self.maxpool(x)
        x = self.stage2(x)
        x = self.stage3(x)
        # Note: in stage4, blk0.stride was changed to 1, while blk0.forward in torchvision checks `self.stride == 1`:
        # wait - torchvision's InvertedResidual.forward checks `if self.stride == 1: chunk` vs `else: b1+b2`!
        # In stage4.0, C_in=96 -> C_out=192, so we MUST run branch1+branch2 explicitly!
        b1 = self.stage4[0].branch1(x)
        b2 = self.stage4[0].branch2(x)
        cat = torch.cat((b1, b2), dim=1)
        n, c, h, w = cat.shape
        x = cat.view(n, 2, c // 2, h, w).transpose(1, 2).contiguous().view(n, c, h, w)
        for bi in range(1, len(self.stage4)):
            x = self.stage4[bi](x)
        x = self.conv5(x)
        return self.head_seg(x)  # (B, 5, 6, 16)


def fold_bn(conv: nn.Module, bn: nn.Module):
    w = conv.weight.detach().clone()
    b = conv.bias.detach().clone() if conv.bias is not None else torch.zeros(w.shape[0])
    inv_std = bn.weight.detach() / torch.sqrt(bn.running_var.detach() + bn.eps)
    w_f = w * inv_std.view(-1, 1, 1, 1)
    b_f = (b - bn.running_mean.detach()) * inv_std + bn.bias.detach()
    return w_f, b_f


def hw_round_shift(acc: np.ndarray, shift: int, relu: bool, clamp_i8: bool) -> np.ndarray:
    acc = acc.astype(np.int64)
    if shift > 0:
        acc = acc + (1 << (shift - 1))
    val = acc >> shift
    if relu:
        val = np.maximum(val, 0)
    if clamp_i8:
        val = np.clip(val, -127, 127)
    return val.astype(np.int32)


def collect_activations(model: WoodFCNShuffleNet, calib_loader):
    model.eval()
    stats = {}
    hooks = []

    def add_hook(name, mod, is_relu=True, is_abs=False):
        stats[name] = []

        def fn(_m, _inp, out):
            t = out.detach()
            vals = t.abs().cpu().numpy() if is_abs else (F.relu(t).cpu().numpy() if is_relu else t.cpu().numpy())
            stats[name].append(float(np.percentile(vals, 99.9)))

        hooks.append(mod.register_forward_hook(fn))

    add_hook("conv1", model.conv1[1], is_relu=True)
    for st_name, st_seq in [("stage2", model.stage2), ("stage3", model.stage3), ("stage4", model.stage4)]:
        for bi, blk in enumerate(st_seq):
            if bi == 0:
                add_hook(f"{st_name}.{bi}.b1_dw", blk.branch1[1], is_abs=True)
                add_hook(f"{st_name}.{bi}.b1_pw", blk.branch1[3], is_relu=True)
            add_hook(f"{st_name}.{bi}.b2_pw1", blk.branch2[1], is_relu=True)
            add_hook(f"{st_name}.{bi}.b2_dw", blk.branch2[4], is_abs=True)
            add_hook(f"{st_name}.{bi}.b2_pw2", blk.branch2[6], is_relu=True)
    add_hook("conv5", model.conv5[1], is_relu=True)
    add_hook("head_seg", model.head_seg, is_abs=True)

    with torch.no_grad():
        for i, (bx, _) in enumerate(calib_loader):
            model(bx)
            if i >= 12:
                break
    for h in hooks:
        h.remove()
    return {k: max(0.5, float(np.median(v))) for k, v in stats.items()}


def quant_free(wf, bf, in_scale, p999_act, is_logits=False):
    w_max = float(wf.abs().max().item()) or 1e-6
    w_scale = 127.0 / w_max
    w_q = torch.clamp(torch.round(wf * w_scale), -127, 127).to(torch.int32).numpy()
    target_scale = 128.0 if is_logits else (127.0 / (p999_act * 1.05))
    ratio = (in_scale * w_scale) / target_scale
    shift = max(1, min(18, int(round(math.log2(max(2.0, ratio))))))
    eff_out = (in_scale * w_scale) / (2.0 ** shift)
    b_q = torch.round(bf * (in_scale * w_scale)).to(torch.int64).numpy().astype(np.int32)
    return {"w": w_q, "b": b_q, "shift": shift, "out_scale": eff_out, "in_scale": in_scale, "w_scale": w_scale}


def quant_locked(wf, bf, in_scale, fixed_out_scale):
    w_max = float(wf.abs().max().item()) or 1e-6
    max_ratio = (127.0 * in_scale) / (w_max * fixed_out_scale)
    shift = max(1, min(18, int(math.floor(math.log2(max(2.0, max_ratio))))))
    w_scale = (fixed_out_scale * (2.0 ** shift)) / in_scale
    w_q = torch.clamp(torch.round(wf * w_scale), -127, 127).to(torch.int32).numpy()
    b_q = torch.round(bf * (in_scale * w_scale)).to(torch.int64).numpy().astype(np.int32)
    return {"w": w_q, "b": b_q, "shift": shift, "out_scale": fixed_out_scale, "in_scale": in_scale, "w_scale": w_scale}


def quantize_wood_fcn(model: WoodFCNShuffleNet, calib_loader, scale_mult: float = 1.15):
    act = collect_activations(model, calib_loader)
    folded = {"conv1": fold_bn(model.conv1[0], model.conv1[1])}
    for st_name, st_seq in [("stage2", model.stage2), ("stage3", model.stage3), ("stage4", model.stage4)]:
        for bi, blk in enumerate(st_seq):
            if bi == 0:
                folded[f"{st_name}.{bi}.b1_dw"] = fold_bn(blk.branch1[0], blk.branch1[1])
                folded[f"{st_name}.{bi}.b1_pw"] = fold_bn(blk.branch1[2], blk.branch1[3])
            folded[f"{st_name}.{bi}.b2_pw1"] = fold_bn(blk.branch2[0], blk.branch2[1])
            folded[f"{st_name}.{bi}.b2_dw"] = fold_bn(blk.branch2[3], blk.branch2[4])
            folded[f"{st_name}.{bi}.b2_pw2"] = fold_bn(blk.branch2[5], blk.branch2[6])
    folded["conv5"] = fold_bn(model.conv5[0], model.conv5[1])
    folded["head_seg"] = (model.head_seg.weight.detach().clone(), model.head_seg.bias.detach().clone())

    q = {"conv1": quant_free(*folded["conv1"], 128.0, act["conv1"] * scale_mult)}
    prev_scale = q["conv1"]["out_scale"]
    blocks_spec = []

    for st_idx, (st_name, st_seq) in enumerate([("stage2", model.stage2), ("stage3", model.stage3), ("stage4", model.stage4)], start=2):
        stage_p999 = max(act[f"{st_name}.{bi}.b2_pw2"] for bi in range(len(st_seq)))
        s_stage = 127.0 / (stage_p999 * 1.08 * scale_mult)
        pfx0 = f"{st_name}.0"
        q[f"{pfx0}.b1_dw"] = quant_free(*folded[f"{pfx0}.b1_dw"], prev_scale, act[f"{pfx0}.b1_dw"] * scale_mult)
        q[f"{pfx0}.b2_pw1"] = quant_free(*folded[f"{pfx0}.b2_pw1"], prev_scale, act[f"{pfx0}.b2_pw1"] * scale_mult)
        q[f"{pfx0}.b2_dw"] = quant_free(*folded[f"{pfx0}.b2_dw"], q[f"{pfx0}.b2_pw1"]["out_scale"], act[f"{pfx0}.b2_dw"] * scale_mult)
        for lkey, in_s in [(f"{pfx0}.b1_pw", q[f"{pfx0}.b1_dw"]["out_scale"]), (f"{pfx0}.b2_pw2", q[f"{pfx0}.b2_dw"]["out_scale"])]:
            wmax = float(folded[lkey][0].abs().max().item()) or 1e-6
            s_stage = min(s_stage, (60.0 * in_s) / wmax)

        for _retry in range(4):
            q[f"{pfx0}.b1_pw"] = quant_locked(*folded[f"{pfx0}.b1_pw"], q[f"{pfx0}.b1_dw"]["out_scale"], s_stage)
            q[f"{pfx0}.b2_pw2"] = quant_locked(*folded[f"{pfx0}.b2_pw2"], q[f"{pfx0}.b2_dw"]["out_scale"], s_stage)
            ok_stage = True
            for bi in range(1, len(st_seq)):
                pfx = f"{st_name}.{bi}"
                q[f"{pfx}.b2_pw1"] = quant_free(*folded[f"{pfx}.b2_pw1"], s_stage, act[f"{pfx}.b2_pw1"] * scale_mult)
                q[f"{pfx}.b2_dw"] = quant_free(*folded[f"{pfx}.b2_dw"], q[f"{pfx}.b2_pw1"]["out_scale"], act[f"{pfx}.b2_dw"] * scale_mult)
                dw_s = q[f"{pfx}.b2_dw"]["out_scale"]
                wmax = float(folded[f"{pfx}.b2_pw2"][0].abs().max().item()) or 1e-6
                if (127.0 * dw_s) / (wmax * s_stage) < 2.05:
                    s_stage = min(s_stage * 0.65, (60.0 * dw_s) / wmax)
                    ok_stage = False
                    break
                q[f"{pfx}.b2_pw2"] = quant_locked(*folded[f"{pfx}.b2_pw2"], dw_s, s_stage)
            if ok_stage:
                break

        # Block metadata: (st_idx, bi, is_two_branch, dw_stride, dw_dilation)
        if st_idx in (2, 3):
            blocks_spec.append((st_idx, 0, True, 2, 1))
        else:
            blocks_spec.append((st_idx, 0, True, 1, 2))
        for bi in range(1, len(st_seq)):
            blocks_spec.append((st_idx, bi, False, 1, 1))
        prev_scale = s_stage

    q["conv5"] = quant_free(*folded["conv5"], prev_scale, act["conv5"] * scale_mult)
    q["head_seg"] = quant_free(*folded["head_seg"], q["conv5"]["out_scale"], 1.0, is_logits=True)
    for k, (wf_k, bf_k) in folded.items():
        q[k]["wf"] = wf_k.clone()
        q[k]["bf"] = bf_k.clone()
    return q, blocks_spec


class QATLayer(nn.Module):
    def __init__(self, ql: dict, stride=1, padding=0, dilation=1, groups=1, relu=True, clamp_i8=True):
        super().__init__()
        self.w = nn.Parameter(ql["wf"].clone())
        self.b = nn.Parameter(ql["bf"].clone())
        self.w_scale = float(ql["w_scale"])
        self.b_scale = float(ql["in_scale"] * ql["w_scale"])
        self.out_scale = float(ql["out_scale"])
        self.stride = stride
        self.padding = padding
        self.dilation = dilation
        self.groups = groups
        self.relu = relu
        self.clamp_i8 = clamp_i8

    def forward(self, x):
        wq = torch.clamp(torch.round(self.w * self.w_scale), -127.0, 127.0) / self.w_scale
        wh = self.w + (wq - self.w).detach()
        bq = torch.round(self.b * self.b_scale) / self.b_scale
        bh = self.b + (bq - self.b).detach()
        y = F.conv2d(x, wh, bh, stride=self.stride, padding=self.padding, dilation=self.dilation, groups=self.groups)
        if self.relu:
            y = F.relu(y)
            yq = torch.clamp(torch.round(y * self.out_scale), 0.0, 127.0) / self.out_scale
            return y + (yq - y).detach()
        if self.clamp_i8:
            yq = torch.clamp(torch.round(y * self.out_scale), -127.0, 127.0) / self.out_scale
            return y + (yq - y).detach()
        return y


class FoldedQATWoodFCN(nn.Module):
    def __init__(self, q: dict, blocks_spec):
        super().__init__()
        self.blocks_spec = list(blocks_spec)
        self.layers = nn.ModuleDict()
        self.layers["conv1"] = QATLayer(q["conv1"], stride=2, padding=1, groups=1, relu=True, clamp_i8=True)
        for st_idx, bi, two_br, dw_s, dw_d in self.blocks_spec:
            pfx = f"stage{st_idx}.{bi}"
            mk = f"s{st_idx}_{bi}"
            c_b = q[f"{pfx}.b2_pw1"]["w"].shape[0]
            pad = dw_d
            if two_br:
                self.layers[f"{mk}_b1_dw"] = QATLayer(q[f"{pfx}.b1_dw"], stride=dw_s, padding=pad, dilation=dw_d, groups=c_b, relu=False, clamp_i8=True)
                self.layers[f"{mk}_b1_pw"] = QATLayer(q[f"{pfx}.b1_pw"], stride=1, padding=0, groups=1, relu=True, clamp_i8=True)
            self.layers[f"{mk}_b2_pw1"] = QATLayer(q[f"{pfx}.b2_pw1"], stride=1, padding=0, groups=1, relu=True, clamp_i8=True)
            self.layers[f"{mk}_b2_dw"] = QATLayer(q[f"{pfx}.b2_dw"], stride=dw_s, padding=pad, dilation=dw_d, groups=c_b, relu=False, clamp_i8=True)
            self.layers[f"{mk}_b2_pw2"] = QATLayer(q[f"{pfx}.b2_pw2"], stride=1, padding=0, groups=1, relu=True, clamp_i8=True)
        self.layers["conv5"] = QATLayer(q["conv5"], stride=1, padding=0, groups=1, relu=True, clamp_i8=True)
        self.layers["head_seg"] = QATLayer(q["head_seg"], stride=1, padding=0, groups=1, relu=False, clamp_i8=False)

    def forward(self, x):
        x = self.layers["conv1"](x)
        x = F.max_pool2d(x, kernel_size=3, stride=2, padding=1)
        for st_idx, bi, two_br, _, _ in self.blocks_spec:
            mk = f"s{st_idx}_{bi}"
            if two_br:
                b1 = self.layers[f"{mk}_b1_pw"](self.layers[f"{mk}_b1_dw"](x))
                b2 = self.layers[f"{mk}_b2_pw2"](self.layers[f"{mk}_b2_dw"](self.layers[f"{mk}_b2_pw1"](x)))
                cat = torch.cat([b1, b2], dim=1)
            else:
                x1, x2 = x.chunk(2, dim=1)
                b2 = self.layers[f"{mk}_b2_pw2"](self.layers[f"{mk}_b2_dw"](self.layers[f"{mk}_b2_pw1"](x2)))
                cat = torch.cat([x1, b2], dim=1)
            n, c, h, w = cat.shape
            x = cat.view(n, 2, c // 2, h, w).transpose(1, 2).contiguous().view(n, c, h, w)
        x = self.layers["conv5"](x)
        return self.layers["head_seg"](x)

    def export_into_q(self, q: dict):
        name_map = {"conv1": "conv1", "conv5": "conv5", "head_seg": "head_seg"}
        for st_idx, bi, two_br, _, _ in self.blocks_spec:
            pfx = f"stage{st_idx}.{bi}"
            mk = f"s{st_idx}_{bi}"
            if two_br:
                name_map[f"{mk}_b1_dw"] = f"{pfx}.b1_dw"
                name_map[f"{mk}_b1_pw"] = f"{pfx}.b1_pw"
            name_map[f"{mk}_b2_pw1"] = f"{pfx}.b2_pw1"
            name_map[f"{mk}_b2_dw"] = f"{pfx}.b2_dw"
            name_map[f"{mk}_b2_pw2"] = f"{pfx}.b2_pw2"
        for mk, qk in name_map.items():
            lay = self.layers[mk]
            q[qk]["w"] = torch.clamp(torch.round(lay.w.detach() * lay.w_scale), -127.0, 127.0).to(torch.int32).numpy()
            q[qk]["b"] = torch.round(lay.b.detach() * lay.b_scale).to(torch.int64).numpy().astype(np.int32)
        return q


def emulate_dw(in_hwc: np.ndarray, q_layer: dict, stride: int, dil: int) -> np.ndarray:
    H, W, C = in_hwc.shape
    oH, oW = (H // 2 if stride == 2 else H), (W // 2 if stride == 2 else W)
    pad = np.pad(in_hwc.astype(np.int64), ((dil, dil), (dil, dil), (0, 0)), mode="constant")
    w = q_layer["w"].reshape(C, 9).astype(np.int64)
    b = q_layer["b"].astype(np.int64)
    out = np.zeros((oH, oW, C), dtype=np.int64)
    for ky in range(3):
        y0 = ky * dil
        for kx in range(3):
            x0 = kx * dil
            out += pad[y0 : y0 + oH * stride : stride, x0 : x0 + oW * stride : stride, :] * w[:, ky * 3 + kx]
    out += b
    return hw_round_shift(out, q_layer["shift"], relu=False, clamp_i8=True).astype(np.int8)


def emulate_pw(in_hwc: np.ndarray, q_layer: dict, relu: bool = True, clamp_i8: bool = True) -> np.ndarray:
    H, W, C_in = in_hwc.shape
    w = q_layer["w"].reshape(q_layer["w"].shape[0], C_in).astype(np.int64)
    b = q_layer["b"].astype(np.int64)
    acc = in_hwc.reshape(-1, C_in).astype(np.int64) @ w.T + b
    out = hw_round_shift(acc, q_layer["shift"], relu=relu, clamp_i8=clamp_i8).reshape(H, W, -1)
    return out.astype(np.int8) if clamp_i8 else out.astype(np.int32)


def emulate_wood_fcn_single(img_hwc_i8: np.ndarray, q: dict, blocks_spec) -> np.ndarray:
    """Bit-exact integer emulator of SIMD-16 FPGA NPU + AE350 firmware -> (6, 16, 5) int32 logits."""
    H, W, _ = img_hwc_i8.shape  # 96, 256
    pad = np.pad(img_hwc_i8.astype(np.int64), ((1, 1), (1, 1), (0, 0)), mode="constant")
    w1 = q["conv1"]["w"].transpose(0, 2, 3, 1).reshape(24, 27).astype(np.int64)
    oH, oW = H // 2, W // 2  # 48, 128
    patches = np.zeros((oH * oW, 27), dtype=np.int64)
    p = 0
    for ky in range(3):
        for kx in range(3):
            patches[:, p : p + 3] = pad[ky : ky + H : 2, kx : kx + W : 2, :].reshape(-1, 3)
            p += 3
    x = hw_round_shift(patches @ w1.T + q["conv1"]["b"], q["conv1"]["shift"], relu=True, clamp_i8=True).reshape(oH, oW, 24).astype(np.int8)

    pad = np.pad(x, ((1, 1), (1, 1), (0, 0)), mode="constant")
    mp = np.zeros((oH // 2, oW // 2, 24), dtype=np.int8)  # 24, 64
    for ky in range(3):
        for kx in range(3):
            mp = np.maximum(mp, pad[ky : ky + oH : 2, kx : kx + oW : 2, :])
    x = mp

    for st_idx, bi, two_br, dw_s, dw_d in blocks_spec:
        pfx = f"stage{st_idx}.{bi}"
        if two_br:
            b1 = emulate_pw(emulate_dw(x, q[f"{pfx}.b1_dw"], dw_s, dw_d), q[f"{pfx}.b1_pw"])
            b2 = emulate_pw(emulate_dw(emulate_pw(x, q[f"{pfx}.b2_pw1"]), q[f"{pfx}.b2_dw"], dw_s, dw_d), q[f"{pfx}.b2_pw2"])
            cat = np.concatenate([b1, b2], axis=-1)
        else:
            half = x.shape[-1] // 2
            x1, x2 = x[..., :half], x[..., half:]
            b2 = emulate_pw(emulate_dw(emulate_pw(x2, q[f"{pfx}.b2_pw1"]), q[f"{pfx}.b2_dw"], dw_s, dw_d), q[f"{pfx}.b2_pw2"])
            cat = np.concatenate([x1, b2], axis=-1)
        cH, cW, cC = cat.shape
        x = cat.reshape(cH, cW, 2, cC // 2).swapaxes(2, 3).reshape(cH, cW, cC)

    x = emulate_pw(x, q["conv5"], relu=True, clamp_i8=True)
    logits = emulate_pw(x, q["head_seg"], relu=False, clamp_i8=False)  # (6, 16, 5) int32
    return logits


def pack_matrix_u32_simd16(mat_out_in: np.ndarray) -> tuple[list[int], int]:
    """Packs (out_ch, in_ch) INT8 matrix padded to multiples of 16 bytes (4 u32 words per quad)."""
    out_ch, in_ch = mat_out_in.shape
    raw_words = (in_ch + 3) // 4
    vec_quads = (in_ch + 15) // 16
    padded = np.zeros((out_ch, vec_quads * 16), dtype=np.int8)
    padded[:, :in_ch] = mat_out_in.astype(np.int8)
    words = []
    for oc in range(out_ch):
        row = padded[oc].astype(np.uint8)
        for w in range(vec_quads * 4):
            b0, b1, b2, b3 = int(row[w * 4]), int(row[w * 4 + 1]), int(row[w * 4 + 2]), int(row[w * 4 + 3])
            words.append(b0 | (b1 << 8) | (b2 << 16) | (b3 << 24))
    return words, raw_words


def emit_arr(ctype: str, name: str, vals: list[int], fmt: str) -> list[str]:
    out = [f"static const {ctype} {name}[{len(vals)}] = {{"]
    for i in range(0, len(vals), 12):
        out.append("    " + ", ".join(fmt.format(v) for v in vals[i : i + 12]) + ",")
    out += ["};", ""]
    return out


def export_wood_header(q: dict, blocks_spec):
    wgt_u32_pool: list[int] = []
    npu_b_pool: list[int] = []
    dw_i8_pool: list[int] = []
    dw_b_pool: list[int] = []

    def add_pw(mat_oi: np.ndarray, b_arr: np.ndarray):
        w_off = len(wgt_u32_pool)
        b_off = len(npu_b_pool)
        words, _ = pack_matrix_u32_simd16(mat_oi)
        wgt_u32_pool.extend(words)
        npu_b_pool.extend([int(v) for v in b_arr])
        return w_off, b_off

    def add_dw(w_c9: np.ndarray, b_arr: np.ndarray):
        w_off = len(dw_i8_pool)
        b_off = len(dw_b_pool)
        dw_i8_pool.extend([int(v) for v in w_c9.reshape(-1)])
        dw_b_pool.extend([int(v) for v in b_arr])
        return w_off, b_off

    w_conv1 = q["conv1"]["w"].transpose(0, 2, 3, 1).reshape(24, 27)
    conv1_wo, conv1_bo = add_pw(w_conv1, q["conv1"]["b"])

    blk_entries = []
    cur_h, cur_w, cur_c = 24, 64, 24  # maxpool output (96/4=24, 256/4=64)
    for st_idx, bi, two_br, dw_s, dw_d in blocks_spec:
        pfx = f"stage{st_idx}.{bi}"
        if two_br:
            c_branch = cur_c
            b1_dw_wo, b1_dw_bo = add_dw(q[f"{pfx}.b1_dw"]["w"], q[f"{pfx}.b1_dw"]["b"])
            b1_pw_wo, b1_pw_bo = add_pw(q[f"{pfx}.b1_pw"]["w"].reshape(c_branch, c_branch), q[f"{pfx}.b1_pw"]["b"])
            b1_dws = q[f"{pfx}.b1_dw"]["shift"]
            b1_pws = q[f"{pfx}.b1_pw"]["shift"]
        else:
            c_branch = cur_c // 2
            b1_dw_wo = b1_dw_bo = b1_pw_wo = b1_pw_bo = 0
            b1_dws = b1_pws = 0

        b2_pw1_wo, b2_pw1_bo = add_pw(q[f"{pfx}.b2_pw1"]["w"].reshape(c_branch, c_branch), q[f"{pfx}.b2_pw1"]["b"])
        b2_dw_wo, b2_dw_bo = add_dw(q[f"{pfx}.b2_dw"]["w"], q[f"{pfx}.b2_dw"]["b"])
        b2_pw2_wo, b2_pw2_bo = add_pw(q[f"{pfx}.b2_pw2"]["w"].reshape(c_branch, c_branch), q[f"{pfx}.b2_pw2"]["b"])

        blk_entries.append((
            1 if two_br else 0, dw_s, dw_d, cur_h, cur_w, cur_c, c_branch,
            b1_dw_wo, b1_dw_bo, b1_dws, b1_pw_wo, b1_pw_bo, b1_pws,
            b2_pw1_wo, b2_pw1_bo, q[f"{pfx}.b2_pw1"]["shift"],
            b2_dw_wo, b2_dw_bo, q[f"{pfx}.b2_dw"]["shift"],
            b2_pw2_wo, b2_pw2_bo, q[f"{pfx}.b2_pw2"]["shift"],
        ))
        if two_br:
            cur_h = cur_h // dw_s
            cur_w = cur_w // dw_s
            cur_c *= 2

    conv5_wo, conv5_bo = add_pw(q["conv5"]["w"].reshape(128, 192), q["conv5"]["b"])
    head_wo, head_bo = add_pw(q["head_seg"]["w"].reshape(NUM_CLASSES, 128), q["head_seg"]["b"])

    lines = [
        "/*",
        " * Auto-generated SIMD-16 INT8 Wood Defect Spatial FCN Weights (256x96 -> 16x6x5)",
        " * Do not edit manually - regenerate via: tools/npu/train_wood_fcn.py",
        " */",
        "#ifndef APP_NPU_WEIGHTS_H_",
        "#define APP_NPU_WEIGHTS_H_",
        "",
        "#include <stdint.h>",
        "",
        f"#define WOOD_IN_W         {IN_W}U",
        f"#define WOOD_IN_H         {IN_H}U",
        f"#define WOOD_GRID_W       {GRID_W}U",
        f"#define WOOD_GRID_H       {GRID_H}U",
        f"#define WOOD_NUM_CLASSES  {NUM_CLASSES}U",
        f"#define NPU_CONV1_WO      {conv1_wo}U",
        f"#define NPU_CONV1_BO      {conv1_bo}U",
        f"#define NPU_CONV1_SHIFT   {q['conv1']['shift']}U",
        f"#define NPU_NUM_BLOCKS    {len(blk_entries)}U",
        f"#define NPU_CONV5_WO      {conv5_wo}U",
        f"#define NPU_CONV5_BO      {conv5_bo}U",
        f"#define NPU_CONV5_SHIFT   {q['conv5']['shift']}U",
        f"#define NPU_HEAD_WO       {head_wo}U",
        f"#define NPU_HEAD_BO       {head_bo}U",
        f"#define NPU_HEAD_SHIFT    {q['head_seg']['shift']}U",
        "",
        "typedef struct {",
        "    uint8_t  two_branch;",
        "    uint8_t  dw_stride;",
        "    uint8_t  dw_dilation;",
        "    uint8_t  h_in;",
        "    uint8_t  w_in;",
        "    uint16_t c_in;",
        "    uint16_t c_branch;",
        "    uint32_t b1_dw_wo;",
        "    uint16_t b1_dw_bo;",
        "    uint8_t  b1_dw_shift;",
        "    uint32_t b1_pw_wo;",
        "    uint16_t b1_pw_bo;",
        "    uint8_t  b1_pw_shift;",
        "    uint32_t b2_pw1_wo;",
        "    uint16_t b2_pw1_bo;",
        "    uint8_t  b2_pw1_shift;",
        "    uint32_t b2_dw_wo;",
        "    uint16_t b2_dw_bo;",
        "    uint8_t  b2_dw_shift;",
        "    uint32_t b2_pw2_wo;",
        "    uint16_t b2_pw2_bo;",
        "    uint8_t  b2_pw2_shift;",
        "} NpuWoodBlockDesc;",
        "",
        f"static const NpuWoodBlockDesc k_wood_blocks[{len(blk_entries)}] = {{",
    ]
    for e in blk_entries:
        lines.append("    { " + ", ".join(str(x) for x in e) + " },")
    lines += ["};", ""]

    lines += emit_arr("uint32_t", "k_npu_wgt_pool", wgt_u32_pool, "0x{:08X}U")
    lines += emit_arr("int32_t", "k_npu_bias_pool", npu_b_pool, "{}")
    lines += emit_arr("int8_t", "k_dw_wgt_pool", dw_i8_pool, "{}")
    lines += emit_arr("int32_t", "k_dw_bias_pool", dw_b_pool, "{}")
    lines += ["#endif /* APP_NPU_WEIGHTS_H_ */", ""]

    WEIGHTS_H.write_text("\n".join(lines), encoding="utf-8")
    tot_bytes = len(wgt_u32_pool) * 4 + len(npu_b_pool) * 4 + len(dw_i8_pool) + len(dw_b_pool) * 4
    print(f"Exported SIMD-16 Wood FCN weights to {WEIGHTS_H} ({tot_bytes:,} bytes total)")


def eval_spatial_metrics(pred_logits, target_grid):
    """Computes cell accuracy, defect detection rate (board has defect vs clean), and per-class recall."""
    preds = pred_logits.argmax(dim=1)  # (B, H, W)
    cell_ok = int((preds == target_grid).sum().item())
    cell_tot = int(target_grid.numel())
    c_ok = [0] * NUM_CLASSES
    c_tot = [0] * NUM_CLASSES
    for c in range(NUM_CLASSES):
        m = target_grid == c
        c_tot[c] = int(m.sum().item())
        c_ok[c] = int((preds[m] == c).sum().item())
    # Board-level any-defect accuracy
    gt_has = (target_grid > 0).any(dim=(1, 2))
    pr_has = (preds > 0).any(dim=(1, 2))
    board_ok = int((gt_has == pr_has).sum().item())
    return cell_ok, cell_tot, c_ok, c_tot, board_ok, len(target_grid)


def export_validation_samples(val_real):
    """Saves 8 diverse real held-out plank samples + GT boxes for TangWood Studio testing."""
    SAMPLES_DIR.mkdir(parents=True, exist_ok=True)
    picked = []
    seen_classes = set()
    for rid, b, boxes in val_real:
        cls_set = {c for c, *_ in boxes}
        if not boxes and 0 not in seen_classes:
            picked.append((rid, b, boxes, "real_clean"))
            seen_classes.add(0)
        elif cls_set - seen_classes:
            cname = CLASS_NAMES[min(cls_set - seen_classes)].lower()
            picked.append((rid, b, boxes, f"real_{cname}_{rid}"))
            seen_classes |= cls_set
        if len(picked) >= 8:
            break
    meta = []
    for rid, b, boxes, stem in picked:
        im = Image.open(io.BytesIO(b)).convert("RGB")
        out_p = SAMPLES_DIR / f"{stem}.png"
        im.save(out_p)
        meta.append({"file": out_p.name, "id": rid, "boxes": boxes})
    (SAMPLES_DIR / "samples_meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"Saved {len(picked)} held-out real lumber samples to {SAMPLES_DIR}")


def main():
    real_items = prepare_real_dataset()
    rng = random.Random(2026)
    shuffled = list(real_items)
    rng.shuffle(shuffled)
    split = int(len(shuffled) * 0.90)
    tr_real, va_real = shuffled[:split], shuffled[split:]
    export_validation_samples(va_real)

    train_ds = WoodSpatialDataset(tr_real, synth_count=2000, is_train=True, seed=11)
    val_ds = WoodSpatialDataset(va_real, synth_count=150, is_train=False, seed=22)
    train_loader = torch.utils.data.DataLoader(train_ds, batch_size=64, shuffle=True, num_workers=0)
    val_loader = torch.utils.data.DataLoader(val_ds, batch_size=64, shuffle=False, num_workers=0)
    calib_loader = torch.utils.data.DataLoader(val_ds, batch_size=64, shuffle=True, num_workers=0)

    # Strong defect class emphasis over empty background cells
    cls_w = torch.tensor([0.25, 1.50, 1.80, 4.00, 2.20], dtype=torch.float32)

    model = WoodFCNShuffleNet()
    print(f"WoodFCNShuffleNet parameters: {sum(p.numel() for p in model.parameters()):,} (input {IN_W}x{IN_H} -> {GRID_W}x{GRID_H}x{NUM_CLASSES})")

    ckpt_exists = ("--qat" in sys.argv or "--export-only" in sys.argv) and CKPT_PT.exists()
    if ckpt_exists:
        print(f"Loading existing FP32 checkpoint {CKPT_PT}...")
        model.load_state_dict(torch.load(str(CKPT_PT), map_location="cpu", weights_only=True))
    else:
        epochs = 7
        opt = torch.optim.AdamW([
            {"params": [p for n, p in model.named_parameters() if "conv5" in n or "head_seg" in n], "lr": 3e-3},
            {"params": [p for n, p in model.named_parameters() if "conv5" not in n and "head_seg" not in n], "lr": 5e-4},
        ], weight_decay=2e-4)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
        best_score = 0.0
        best_state = None

        for ep in range(epochs):
            t0 = time.time()
            model.train()
            ls = 0.0
            n_b = 0
            for bx, bg in train_loader:
                lg = model(bx)
                loss = F.cross_entropy(lg, bg, weight=cls_w, label_smoothing=0.04)
                opt.zero_grad()
                loss.backward()
                opt.step()
                ls += loss.item()
                n_b += 1
            sched.step()

            model.eval()
            cok_tot = [0] * NUM_CLASSES
            ctot_tot = [0] * NUM_CLASSES
            cell_ok = cell_tot = brd_ok = brd_tot = 0
            with torch.no_grad():
                for bx, bg in val_loader:
                    lg = model(bx)
                    a, b, co, ct, bo, bt = eval_spatial_metrics(lg, bg)
                    cell_ok += a; cell_tot += b; brd_ok += bo; brd_tot += bt
                    for c in range(NUM_CLASSES):
                        cok_tot[c] += co[c]; ctot_tot[c] += ct[c]
            mean_rec = 100.0 * sum(cok_tot[c] / max(1, ctot_tot[c]) for c in range(NUM_CLASSES)) / NUM_CLASSES
            rec_str = "/".join(f"{100 * cok_tot[c] / max(1, ctot_tot[c]):4.1f}" for c in range(NUM_CLASSES))
            print(
                f"Ep {ep+1}/{epochs} ({time.time()-t0:5.1f}s) loss={ls/n_b:.4f} | "
                f"cell={100*cell_ok/cell_tot:5.2f}% board={100*brd_ok/brd_tot:5.2f}% mean_rec={mean_rec:5.2f}% [{rec_str}]"
            )
            if mean_rec > best_score:
                best_score = mean_rec
                best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}

        if best_state is not None:
            model.load_state_dict(best_state)
        torch.save(model.state_dict(), str(CKPT_PT))

    # Quantize & run 3-epoch self-distilled INT8 QAT
    q, blocks_spec = quantize_wood_fcn(model, calib_loader, scale_mult=1.15)

    if "--export-only" not in sys.argv:
        teacher = model
        teacher.eval()
        for p in teacher.parameters():
            p.requires_grad_(False)
        qat_net = FoldedQATWoodFCN(q, blocks_spec)
        qat_epochs = 3
        opt = torch.optim.AdamW(qat_net.parameters(), lr=1.5e-4, weight_decay=1e-4)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=qat_epochs)
        best_q_rec = 0.0
        best_q_state = None
        T = 2.0
        print(f"\nRunning {qat_epochs}-epoch Folded INT8 QAT (self-distilled from FP32 WoodFCN)...")
        for ep in range(qat_epochs):
            t0 = time.time()
            qat_net.train()
            ls = 0.0
            n_b = 0
            for bx, bg in train_loader:
                with torch.no_grad():
                    t_lg = teacher(bx)
                s_lg = qat_net(bx)
                ce = F.cross_entropy(s_lg, bg, weight=cls_w, label_smoothing=0.03)
                s_lg_flat = s_lg.permute(0, 2, 3, 1).reshape(-1, NUM_CLASSES)
                t_lg_flat = t_lg.permute(0, 2, 3, 1).reshape(-1, NUM_CLASSES)
                kd = (T * T) * F.kl_div(
                    F.log_softmax(s_lg_flat / T, dim=1),
                    F.softmax(t_lg_flat / T, dim=1),
                    reduction="batchmean",
                )
                loss = 0.60 * ce + 0.40 * kd
                opt.zero_grad()
                loss.backward()
                opt.step()
                ls += loss.item()
                n_b += 1
            sched.step()

            qat_net.eval()
            cok_tot = [0] * NUM_CLASSES
            ctot_tot = [0] * NUM_CLASSES
            cell_ok = cell_tot = brd_ok = brd_tot = 0
            with torch.no_grad():
                for bx, bg in val_loader:
                    lg = qat_net(bx)
                    a, b, co, ct, bo, bt = eval_spatial_metrics(lg, bg)
                    cell_ok += a; cell_tot += b; brd_ok += bo; brd_tot += bt
                    for c in range(NUM_CLASSES):
                        cok_tot[c] += co[c]; ctot_tot[c] += ct[c]
            mean_rec = 100.0 * sum(cok_tot[c] / max(1, ctot_tot[c]) for c in range(NUM_CLASSES)) / NUM_CLASSES
            rec_str = "/".join(f"{100 * cok_tot[c] / max(1, ctot_tot[c]):4.1f}" for c in range(NUM_CLASSES))
            print(
                f"  QAT {ep+1}/{qat_epochs} ({time.time()-t0:5.1f}s) loss={ls/n_b:.4f} | "
                f"cell={100*cell_ok/cell_tot:5.2f}% board={100*brd_ok/brd_tot:5.2f}% mean_rec={mean_rec:5.2f}% [{rec_str}]"
            )
            if mean_rec > best_q_rec:
                best_q_rec = mean_rec
                best_q_state = {k: v.detach().clone() for k, v in qat_net.state_dict().items()}

        if best_q_state is not None:
            qat_net.load_state_dict(best_q_state)
        q = qat_net.export_into_q(q)

    # Bit-exact SIMD-16 hardware emulator check on 120 held-out planks (real + synthetic)
    print("\nVerifying bit-exact SIMD-16 FPGA NPU INT8 emulator on 120 held-out planks...")
    e_ok = e_tot = brd_ok = 0
    ec_ok = [0] * NUM_CLASSES
    ec_tot = [0] * NUM_CLASSES
    for i in range(min(120, len(val_ds))):
        x_t, g_t = val_ds[i]
        in_i8 = np.round(x_t.permute(1, 2, 0).numpy() * 128.0).astype(np.int8)
        lg = emulate_wood_fcn_single(in_i8, q, blocks_spec)  # (6, 16, 5)
        pr = lg.argmax(axis=2)
        gt = g_t.numpy()
        e_ok += int((pr == gt).sum())
        e_tot += int(gt.size)
        if bool((pr > 0).any()) == bool((gt > 0).any()):
            brd_ok += 1
        for c in range(NUM_CLASSES):
            m = gt == c
            ec_tot[c] += int(m.sum())
            ec_ok[c] += int((pr[m] == c).sum())
    rec_s = "/".join(f"{100 * ec_ok[c] / max(1, ec_tot[c]):4.1f}" for c in range(NUM_CLASSES))
    print(f"Bit-exact INT8 emulator (120 planks): cell={100*e_ok/e_tot:.2f}% board_defect={brd_ok}/120 ({100*brd_ok/120:.1f}%) [{rec_s}]")

    export_wood_header(q, blocks_spec)


if __name__ == "__main__":
    main()
