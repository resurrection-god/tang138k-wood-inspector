use eframe::egui;
use image::{ImageBuffer, Rgb};
use std::net::UdpSocket;
use std::path::PathBuf;
use std::time::{Duration, Instant};

const IN_W: usize = 256;
const IN_H: usize = 96;
const GRID_W: usize = 16;
const GRID_H: usize = 6;
const GRID_CELLS: usize = GRID_W * GRID_H;
const NUM_CLASSES: usize = 5;

const CLASS_NAMES: [&str; NUM_CLASSES] = [
    "Чистая древесина",
    "Здоровый сучок",
    "Тёмный / гнилой сучок",
    "Трещина",
    "Смола / Синева / Сердцевина",
];

const CLASS_SHORT: [&str; NUM_CLASSES] = [
    "CLEAN",
    "LIVE KNOT",
    "DEAD KNOT",
    "CRACK",
    "RESIN/STAIN",
];

fn class_color(cls: u8) -> egui::Color32 {
    match cls {
        1 => egui::Color32::from_rgb(70, 215, 95),   // Live Knot - green
        2 => egui::Color32::from_rgb(245, 75, 65),   // Dead Knot - red
        3 => egui::Color32::from_rgb(255, 200, 40),  // Crack - amber/gold
        4 => egui::Color32::from_rgb(195, 95, 255),  // Resin/Stain - violet
        _ => egui::Color32::from_rgb(120, 140, 150), // Clean
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum WoodSpecies {
    Pine,
    Spruce,
    Oak,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SourceMode {
    Synthetic,
    RealFile,
}

#[derive(Clone)]
struct SynthDefect {
    cls: u8,     // 1..=4
    cx: f32,     // 0..1
    cy: f32,     // 0..1
    rw: f32,     // half-width norm
    rh: f32,     // half-height norm
    seed: u64,
}

#[derive(Clone)]
struct FpgaInferenceResult {
    npu_ok: bool,
    infer_cnt: u16,
    board_grade: u8,
    exec_ns: u32,
    mac_count: u32,
    wall_us: u32,
    udp_ms: f32,
    classes: [u8; GRID_CELLS],
    confs: [u8; GRID_CELLS],
    logits: Vec<[i16; NUM_CLASSES]>,
}

impl Default for FpgaInferenceResult {
    fn default() -> Self {
        Self {
            npu_ok: false,
            infer_cnt: 0,
            board_grade: 0,
            exec_ns: 0,
            mac_count: 0,
            wall_us: 0,
            udp_ms: 0.0,
            classes: [0; GRID_CELLS],
            confs: [0; GRID_CELLS],
            logits: vec![[0; NUM_CLASSES]; GRID_CELLS],
        }
    }
}

struct MergedBox {
    cls: u8,
    gx0: usize,
    gy0: usize,
    gx1: usize,
    gy1: usize,
    max_conf: u8,
}

struct XorShift64(u64);
impl XorShift64 {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0.max(1);
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    fn f32(&mut self) -> f32 {
        ((self.next_u64() >> 40) as f32) / ((1u64 << 24) as f32)
    }
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.f32()
    }
}

struct WoodStudioApp {
    target_ip: String,
    target_port: u16,
    status_line: String,
    hw_info: String,
    mode: SourceMode,

    // Synthetic plank generator state
    species: WoodSpecies,
    plank_seed: u64,
    grain_freq: f32,
    brush_cls: u8,
    auto_scan: bool,
    defects: Vec<SynthDefect>,

    // Active canvas image (512x192 display, 256x96 NPU input)
    disp_w: usize,
    disp_h: usize,
    disp_rgb: Vec<u8>,
    npu_rgb: Vec<u8>,
    tex: Option<egui::TextureHandle>,
    loaded_label: String,
    sample_files: Vec<PathBuf>,

    // Overlay visualization toggles
    show_heatmap: bool,
    show_boxes: bool,
    show_gt: bool,
    cell_conf_thresh: u8,

    last_res: Option<FpgaInferenceResult>,
}

impl WoodStudioApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut style = (*cc.egui_ctx.style()).clone();
        style.visuals = egui::Visuals::dark();
        cc.egui_ctx.set_style(style);

        let sample_dir = PathBuf::from("../tools/npu/samples");
        let mut sample_files = Vec::new();
        if let Ok(entries) = std::fs::read_dir(&sample_dir) {
            for e in entries.flatten() {
                let p = e.path();
                if matches!(
                    p.extension().and_then(|s| s.to_str()),
                    Some("png" | "jpg" | "jpeg" | "bmp")
                ) {
                    sample_files.push(p);
                }
            }
        }
        sample_files.sort();

        let mut app = Self {
            target_ip: "10.0.0.222".to_string(),
            target_port: 5555,
            status_line: "Готов к работе. Нажмите «Ping FPGA» или отправьте доску на NPU.".to_string(),
            hw_info: "NPU: ожидание связи".to_string(),
            mode: SourceMode::Synthetic,
            species: WoodSpecies::Pine,
            plank_seed: 20261118,
            grain_freq: 0.22,
            brush_cls: 2,
            auto_scan: false,
            defects: Vec::new(),
            disp_w: 512,
            disp_h: 192,
            disp_rgb: vec![0; 512 * 192 * 3],
            npu_rgb: vec![0; IN_W * IN_H * 3],
            tex: None,
            loaded_label: "Синтетическая доска".to_string(),
            sample_files,
            show_heatmap: true,
            show_boxes: true,
            show_gt: true,
            cell_conf_thresh: 50,
            last_res: None,
        };
        app.randomize_synth_defects();
        app.render_synthetic_plank(&cc.egui_ctx);
        app
    }

    fn randomize_synth_defects(&mut self) {
        self.defects.clear();
        let mut rng = XorShift64(self.plank_seed ^ 0x9E37_79B9_7F4A_7C15);
        let cnt = 2 + (rng.next_u64() % 3) as usize;
        for _ in 0..cnt {
            let roll = rng.f32();
            let cls = if roll < 0.32 {
                1
            } else if roll < 0.60 {
                2
            } else if roll < 0.82 {
                3
            } else {
                4
            };
            let (rw, rh) = match cls {
                1 | 2 => (rng.range(0.032, 0.068), rng.range(0.085, 0.165)),
                3 => (rng.range(0.09, 0.20), rng.range(0.025, 0.05)),
                _ => (rng.range(0.05, 0.11), rng.range(0.08, 0.16)),
            };
            let cx = rng.range(rw + 0.04, 1.0 - rw - 0.04);
            let cy = rng.range(rh + 0.06, 1.0 - rh - 0.06);
            self.defects.push(SynthDefect {
                cls,
                cx,
                cy,
                rw,
                rh,
                seed: rng.next_u64(),
            });
        }
    }

    fn render_synthetic_plank(&mut self, ctx: &egui::Context) {
        self.mode = SourceMode::Synthetic;
        let w = 512usize;
        let h = 192usize;
        self.disp_w = w;
        self.disp_h = h;
        let mut buf = vec![0u8; w * h * 3];
        let mut rng = XorShift64(self.plank_seed);

        let base = match self.species {
            WoodSpecies::Pine => [218.0f32, 178.0, 128.0],
            WoodSpecies::Spruce => [228.0f32, 196.0, 152.0],
            WoodSpecies::Oak => [175.0f32, 132.0, 88.0],
        };
        let phase = rng.range(0.0, std::f32::consts::TAU);
        let warp_freq = rng.range(0.010, 0.026);
        let warp_amp = rng.range(3.0, 9.0);

        for y in 0..h {
            for x in 0..w {
                let xf = x as f32;
                let yf = y as f32;
                let warp = warp_amp * (xf * warp_freq + phase).sin();
                let ring = ((yf + warp) * self.grain_freq + phase).sin();
                let grain = 13.0 * ring + 4.5 * (yf * self.grain_freq * 2.2 + warp * 0.4).sin();
                let fiber = 4.5 * (yf * 0.75 + xf * 0.025).sin();
                let n = (rng.f32() - 0.5) * 6.5;

                let mut r = base[0] + grain * 1.0 + fiber + n;
                let mut g = base[1] + grain * 0.85 + fiber * 0.8 + n;
                let mut b = base[2] + grain * 0.60 + fiber * 0.5 + n;

                // Paint synthetic defects smoothly
                for d in &self.defects {
                    let dx = (xf / w as f32 - d.cx) / d.rw.max(0.01);
                    let dy = (yf / h as f32 - d.cy) / d.rh.max(0.01);
                    let r2 = dx * dx + dy * dy;
                    match d.cls {
                        1 => {
                            // Live knot: warm darker wood rings inside oval
                            if r2 < 1.15 {
                                let t = (1.15 - r2).clamp(0.0, 1.0);
                                r = r * (1.0 - 0.52 * t) + 75.0 * (0.52 * t);
                                g = g * (1.0 - 0.56 * t) + 46.0 * (0.56 * t);
                                b = b * (1.0 - 0.60 * t) + 26.0 * (0.60 * t);
                            }
                        }
                        2 => {
                            // Dead / rotten knot: dark charcoal core + oxidized halo
                            if r2 < 1.3 {
                                let halo = ((1.3 - r2) * 1.2).clamp(0.0, 1.0);
                                r *= 1.0 - 0.45 * halo;
                                g *= 1.0 - 0.50 * halo;
                                b *= 1.0 - 0.55 * halo;
                            }
                            if r2 < 0.88 {
                                r = 26.0 + n;
                                g = 18.0 + n;
                                b = 14.0 + n;
                            }
                        }
                        3 => {
                            // Crack: sharp dark horizontal/curved fissure
                            let jitter = 0.18 * ((dx * 4.5 + (d.seed & 255) as f32).sin());
                            if dx.abs() <= 1.0 && (dy - jitter).abs() < 0.24 {
                                r = 20.0;
                                g = 14.0;
                                b = 12.0;
                            }
                        }
                        _ => {
                            // Resin pocket or blue stain streak
                            if r2 < 1.0 {
                                let t = (1.0 - r2).clamp(0.0, 1.0);
                                if (d.seed & 1) == 0 {
                                    r = r * (1.0 - t) + 150.0 * t;
                                    g = g * (1.0 - t) + 74.0 * t;
                                    b = b * (1.0 - t) + 22.0 * t;
                                } else {
                                    r = r * (1.0 - t) + 82.0 * t;
                                    g = g * (1.0 - t) + 96.0 * t;
                                    b = b * (1.0 - t) + 118.0 * t;
                                }
                            }
                        }
                    }
                }

                let idx = (y * w + x) * 3;
                buf[idx] = r.clamp(0.0, 255.0) as u8;
                buf[idx + 1] = g.clamp(0.0, 255.0) as u8;
                buf[idx + 2] = b.clamp(0.0, 255.0) as u8;
            }
        }

        self.disp_rgb = buf;
        self.update_npu_buffer_and_texture(ctx, "Синтетическая доска");
    }

    fn load_image_file(&mut self, ctx: &egui::Context, path: PathBuf) {
        match image::open(&path) {
            Ok(img) => {
                let rgb = img.to_rgb8();
                let resized = image::imageops::resize(
                    &rgb,
                    512,
                    192,
                    image::imageops::FilterType::Triangle,
                );
                self.mode = SourceMode::RealFile;
                self.disp_w = 512;
                self.disp_h = 192;
                self.disp_rgb = resized.into_raw();
                let name = path
                    .file_name()
                    .and_then(|s| s.to_str())
                    .unwrap_or("image")
                    .to_string();
                self.update_npu_buffer_and_texture(ctx, &name);
                self.status_line = format!("Загружен файл: {name}");
                if self.auto_scan {
                    self.run_fpga_inference();
                }
            }
            Err(e) => {
                self.status_line = format!("Ошибка загрузки файла: {e}");
            }
        }
    }

    fn update_npu_buffer_and_texture(&mut self, ctx: &egui::Context, label: &str) {
        self.loaded_label = label.to_string();
        let src: ImageBuffer<Rgb<u8>, Vec<u8>> =
            ImageBuffer::from_raw(self.disp_w as u32, self.disp_h as u32, self.disp_rgb.clone())
                .expect("valid rgb buffer");
        let npu_img = image::imageops::resize(
            &src,
            IN_W as u32,
            IN_H as u32,
            image::imageops::FilterType::Triangle,
        );
        self.npu_rgb = npu_img.into_raw();

        let color_img = egui::ColorImage::from_rgb([self.disp_w, self.disp_h], &self.disp_rgb);
        self.tex = Some(ctx.load_texture("plank_tex", color_img, egui::TextureOptions::LINEAR));
    }

    fn ping_fpga(&mut self) {
        let addr = format!("{}:{}", self.target_ip.trim(), self.target_port);
        let sock = match UdpSocket::bind("0.0.0.0:0") {
            Ok(s) => s,
            Err(e) => {
                self.status_line = format!("Ошибка сокета: {e}");
                return;
            }
        };
        let _ = sock.set_read_timeout(Some(Duration::from_millis(500)));
        let pkt = [b'W', b'D', b'0', b'1', 0x01, 0, 0, 0];
        if let Err(e) = sock.send_to(&pkt, &addr) {
            self.status_line = format!("Ошибка отправки ping: {e}");
            return;
        }
        let mut rep = [0u8; 64];
        match sock.recv_from(&mut rep) {
            Ok((n, _)) if n >= 20 && &rep[0..4] == b"WD01" && rep[4] == 0x81 => {
                let npu_ok = rep[5] != 0;
                let gw = rep[6];
                let gh = rep[7];
                let ncls = rep[8];
                let simd = rep[9];
                let infer_cnt = u16::from_le_bytes([rep[12], rep[13]]);
                let hash = u32::from_le_bytes([rep[16], rep[17], rep[18], rep[19]]);
                self.hw_info = format!(
                    "FPGA OK={npu_ok} | SIMD-{simd} | {gw}x{gh}x{ncls} | HASH=0x{hash:08X} | Infer#{infer_cnt}"
                );
                self.status_line = format!("Плата отвечает ({addr}) — {hw}", hw = self.hw_info);
            }
            Ok((n, _)) => {
                self.status_line = format!("Неожиданный ответ ({n} байт) от {addr}");
            }
            Err(e) => {
                self.status_line = format!("Таймаут Ping ({addr}): {e}");
            }
        }
    }

    fn run_fpga_inference(&mut self) {
        let addr = format!("{}:{}", self.target_ip.trim(), self.target_port);
        let sock = match UdpSocket::bind("0.0.0.0:0") {
            Ok(s) => s,
            Err(e) => {
                self.status_line = format!("Ошибка сокета: {e}");
                return;
            }
        };
        let _ = sock.set_read_timeout(Some(Duration::from_millis(450)));

        let t0 = Instant::now();
        let total = self.npu_rgb.len();
        let chunk_sz = 1280usize;
        let mut off = 0usize;
        let mut ack_buf = [0u8; 64];

        while off < total {
            let end = (off + chunk_sz).min(total);
            let len = end - off;
            let flags = if end == total { 1u8 } else { 0u8 };
            let mut pkt = Vec::with_capacity(12 + len);
            pkt.extend_from_slice(b"WD01");
            pkt.push(0x04);
            pkt.extend_from_slice(&(off as u32).to_le_bytes());
            pkt.extend_from_slice(&(len as u16).to_le_bytes());
            pkt.push(flags);
            pkt.extend_from_slice(&self.npu_rgb[off..end]);

            let mut acked = false;
            for _ in 0..4 {
                if sock.send_to(&pkt, &addr).is_err() {
                    continue;
                }
                if let Ok((n, _)) = sock.recv_from(&mut ack_buf)
                    && n >= 12
                    && &ack_buf[0..4] == b"WD01"
                    && ack_buf[4] == 0x84
                {
                    let ack_off = u32::from_le_bytes([ack_buf[5], ack_buf[6], ack_buf[7], ack_buf[8]]) as usize;
                    if ack_off == off {
                        acked = true;
                        break;
                    }
                }
            }
            if !acked {
                self.status_line = format!("Потеря ACK при передаче чанка offset={off} на {addr}");
                return;
            }
            off = end;
        }

        // Query full 16x6x5 spatial result (1176-byte single UDP reply)
        let q_pkt = [b'W', b'D', b'0', b'1', 0x07, 0, 0, 0];
        let mut q_rep = [0u8; 1400];
        for _ in 0..4 {
            if sock.send_to(&q_pkt, &addr).is_err() {
                continue;
            }
            if let Ok((n, _)) = sock.recv_from(&mut q_rep)
                && n >= 24 + GRID_CELLS * 2 + GRID_CELLS * NUM_CLASSES * 2
                && &q_rep[0..4] == b"WD01"
                && q_rep[4] == 0x87
            {
                let mut res = FpgaInferenceResult {
                    npu_ok: q_rep[5] != 0,
                    infer_cnt: u16::from_le_bytes([q_rep[8], q_rep[9]]),
                    board_grade: q_rep[11],
                    exec_ns: u32::from_le_bytes([q_rep[12], q_rep[13], q_rep[14], q_rep[15]]),
                    mac_count: u32::from_le_bytes([q_rep[16], q_rep[17], q_rep[18], q_rep[19]]),
                    wall_us: u32::from_le_bytes([q_rep[20], q_rep[21], q_rep[22], q_rep[23]]),
                    udp_ms: t0.elapsed().as_secs_f32() * 1000.0,
                    ..Default::default()
                };
                res.classes.copy_from_slice(&q_rep[24..24 + GRID_CELLS]);
                res.confs.copy_from_slice(&q_rep[24 + GRID_CELLS..24 + 2 * GRID_CELLS]);
                let lbase = 24 + 2 * GRID_CELLS;
                for p in 0..GRID_CELLS {
                    for c in 0..NUM_CLASSES {
                        let idx = lbase + (p * NUM_CLASSES + c) * 2;
                        res.logits[p][c] = i16::from_le_bytes([q_rep[idx], q_rep[idx + 1]]);
                    }
                }
                self.status_line = format!(
                    "FPGA NPU ({}) #{}: hw={:.2} мс ({} MAC) | сеть+расчёт={:.1} мс",
                    if res.npu_ok { "OK" } else { "HASH_ERR" },
                    res.infer_cnt,
                    res.exec_ns as f32 / 1_000_000.0,
                    res.mac_count,
                    res.udp_ms
                );
                self.last_res = Some(res);
                return;
            }
        }
        self.status_line = format!("Таймаут чтения результата CMD 0x07 от {addr}");
    }

    fn merge_defect_boxes(&self, res: &FpgaInferenceResult) -> Vec<MergedBox> {
        let mut vis = [false; GRID_CELLS];
        let mut boxes = Vec::new();
        for sy in 0..GRID_H {
            for sx in 0..GRID_W {
                let idx = sy * GRID_W + sx;
                let cls = res.classes[idx];
                let conf = res.confs[idx];
                if vis[idx] || cls == 0 || conf < self.cell_conf_thresh {
                    continue;
                }
                let mut stack = vec![(sx, sy)];
                vis[idx] = true;
                let (mut x0, mut y0, mut x1, mut y1) = (sx, sy, sx, sy);
                let mut max_conf = conf;

                while let Some((cx, cy)) = stack.pop() {
                    x0 = x0.min(cx);
                    y0 = y0.min(cy);
                    x1 = x1.max(cx);
                    y1 = y1.max(cy);
                    max_conf = max_conf.max(res.confs[cy * GRID_W + cx]);
                    for (dx, dy) in [(-1isize, 0), (1, 0), (0, -1), (0, 1)] {
                        let nx = cx as isize + dx;
                        let ny = cy as isize + dy;
                        if nx >= 0 && nx < GRID_W as isize && ny >= 0 && ny < GRID_H as isize {
                            let ni = (ny as usize) * GRID_W + (nx as usize);
                            if !vis[ni] && res.classes[ni] == cls && res.confs[ni] >= self.cell_conf_thresh {
                                vis[ni] = true;
                                stack.push((nx as usize, ny as usize));
                            }
                        }
                    }
                }
                boxes.push(MergedBox {
                    cls,
                    gx0: x0,
                    gy0: y0,
                    gx1: x1,
                    gy1: y1,
                    max_conf,
                });
            }
        }
        boxes
    }
}

impl eframe::App for WoodStudioApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.heading("🌲 TangWood Studio — FPGA SIMD-16 Defect Inspector");
                ui.separator();
                ui.label("IP:");
                ui.add(egui::TextEdit::singleline(&mut self.target_ip).desired_width(105.0));
                if ui.button("🔌 Ping FPGA").clicked() {
                    self.ping_fpga();
                }
                if ui.button("🚀 Сканировать на ПЛИС").clicked() {
                    self.run_fpga_inference();
                }
                ui.checkbox(&mut self.auto_scan, "Авто-скан при клике");
            });
            ui.horizontal(|ui| {
                ui.label(&self.status_line);
                ui.separator();
                ui.label(&self.hw_info);
            });
        });

        egui::SidePanel::right("telemetry_panel")
            .min_width(290.0)
            .show(ctx, |ui| {
                ui.heading("📊 Результаты FPGA NPU");
                ui.separator();

                if let Some(res) = self.last_res.as_ref() {
                    let (g_txt, g_col) = match res.board_grade {
                        0 => ("СОРТ A — ЧИСТАЯ", egui::Color32::from_rgb(55, 195, 95)),
                        1 => ("СОРТ B — ДОПУСТИМО", egui::Color32::from_rgb(130, 200, 70)),
                        2 => ("СОРТ C — 2-Й СОРТ", egui::Color32::from_rgb(245, 175, 45)),
                        _ => ("БРАК / ОТБРАКОВКА", egui::Color32::from_rgb(235, 65, 65)),
                    };
                    egui::Frame::none()
                        .fill(g_col.linear_multiply(0.22))
                        .stroke(egui::Stroke::new(1.5_f32, g_col))
                        .rounding(6.0)
                        .inner_margin(8.0)
                        .show(ui, |ui| {
                            ui.colored_label(g_col, egui::RichText::new(g_txt).strong().size(17.0));
                        });

                    ui.add_space(6.0);
                    let mut counts = [0usize; NUM_CLASSES];
                    for i in 0..GRID_CELLS {
                        let c = res.classes[i] as usize;
                        if c < NUM_CLASSES && (c == 0 || res.confs[i] >= self.cell_conf_thresh) {
                            counts[c] += 1;
                        }
                    }
                    let def_cells = GRID_CELLS - counts[0];
                    let def_pct = (def_cells as f32) * 100.0 / (GRID_CELLS as f32);
                    ui.label(format!("Повреждённых ячеек: {def_cells}/96 ({def_pct:.1}%)"));
                    for c in 1..NUM_CLASSES {
                        ui.horizontal(|ui| {
                            ui.colored_label(class_color(c as u8), "■");
                            ui.label(format!("{}: {} ячеек", CLASS_NAMES[c], counts[c]));
                        });
                    }

                    ui.separator();
                    let hw_ms = res.exec_ns as f32 / 1_000_000.0;
                    let gmacs = if res.exec_ns > 0 {
                        (res.mac_count as f32) / (res.exec_ns as f32)
                    } else {
                        0.0
                    };
                    ui.label(format!("Чистый расчёт NPU: {hw_ms:.2} мс"));
                    ui.label(format!("Аппаратных MAC: {}", res.mac_count));
                    ui.label(format!("Пропускная способность: {gmacs:.2} GMAC/s"));
                    ui.label(format!("Полный цикл AE350+NPU: {:.1} мс", res.wall_us as f32 / 1000.0));
                    ui.label(format!("Задержка с UDP-передачей: {:.1} мс", res.udp_ms));
                } else {
                    ui.label("Нажмите «Сканировать на ПЛИС» для получения оверлея.");
                }

                ui.separator();
                ui.heading("🎨 Оверлей и пороги");
                ui.checkbox(&mut self.show_heatmap, "Показать тепловую сетку (16×6)");
                ui.checkbox(&mut self.show_boxes, "Показать рамки дефектов (Boxes)");
                if self.mode == SourceMode::Synthetic {
                    ui.checkbox(&mut self.show_gt, "Показать разметку синтетики (GT)");
                }
                ui.add(egui::Slider::new(&mut self.cell_conf_thresh, 50..=95).text("Порог уверенности %"));
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            // Source & Defect Generator Toolbar
            ui.group(|ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label("Порода:");
                    if ui.selectable_value(&mut self.species, WoodSpecies::Pine, "Сосна").changed()
                        || ui.selectable_value(&mut self.species, WoodSpecies::Spruce, "Ель").changed()
                        || ui.selectable_value(&mut self.species, WoodSpecies::Oak, "Дуб").changed()
                    {
                        self.render_synthetic_plank(ctx);
                    }
                    if ui.add(egui::Slider::new(&mut self.grain_freq, 0.10..=0.40).text("Волокна")).changed() {
                        self.render_synthetic_plank(ctx);
                    }
                    if ui.button("🎲 Новая доска + дефекты").clicked() {
                        self.plank_seed = self.plank_seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                        self.randomize_synth_defects();
                        self.render_synthetic_plank(ctx);
                        if self.auto_scan {
                            self.run_fpga_inference();
                        }
                    }
                    if ui.button("✨ Чистая доска").clicked() {
                        self.defects.clear();
                        self.render_synthetic_plank(ctx);
                        if self.auto_scan {
                            self.run_fpga_inference();
                        }
                    }
                });

                ui.horizontal_wrapped(|ui| {
                    ui.label("Кисть (клик по доске):");
                    for c in 1..=4u8 {
                        ui.selectable_value(&mut self.brush_cls, c, CLASS_NAMES[c as usize]);
                    }
                    ui.separator();
                    if ui.button("📂 Открыть фото...").clicked()
                        && let Some(p) = rfd::FileDialog::new()
                            .add_filter("Images", &["png", "jpg", "jpeg", "bmp"])
                            .pick_file()
                    {
                        self.load_image_file(ctx, p);
                    }
                    if !self.sample_files.is_empty() {
                        egui::ComboBox::from_label("Реальные тесты")
                            .selected_text(&self.loaded_label)
                            .show_ui(ui, |ui| {
                                for p in self.sample_files.clone() {
                                    let lbl = p.file_name().and_then(|s| s.to_str()).unwrap_or("sample");
                                    if ui.selectable_label(false, lbl).clicked() {
                                        self.load_image_file(ctx, p);
                                    }
                                }
                            });
                    }
                });
            });

            ui.add_space(8.0);

            // Interactive Wood Plank Canvas + Overlay Painter
            if let Some(tex) = self.tex.clone() {
                let avail = ui.available_size();
                let aspect = (self.disp_w as f32) / (self.disp_h as f32);
                let draw_w = avail.x.min(avail.y * aspect * 0.92).max(320.0);
                let draw_h = draw_w / aspect;
                let (rect, resp) = ui.allocate_exact_size(egui::vec2(draw_w, draw_h), egui::Sense::click());
                let painter = ui.painter_at(rect);

                painter.image(
                    tex.id(),
                    rect,
                    egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                    egui::Color32::WHITE,
                );

                // Click on plank to stamp defect
                if resp.clicked()
                    && let Some(pos) = resp.interact_pointer_pos()
                {
                    let nx = ((pos.x - rect.min.x) / rect.width()).clamp(0.03, 0.97);
                    let ny = ((pos.y - rect.min.y) / rect.height()).clamp(0.06, 0.94);
                    let (rw, rh) = match self.brush_cls {
                        1 | 2 => (0.045, 0.115),
                        3 => (0.14, 0.035),
                        _ => (0.07, 0.11),
                    };
                    self.defects.push(SynthDefect {
                        cls: self.brush_cls,
                        cx: nx,
                        cy: ny,
                        rw,
                        rh,
                        seed: (self.plank_seed ^ ((nx * 1000.0) as u64)).wrapping_add(self.defects.len() as u64),
                    });
                    self.render_synthetic_plank(ctx);
                    if self.auto_scan {
                        self.run_fpga_inference();
                    }
                }

                // Draw Ground Truth synthetic defect boxes (dashed/thin white-cyan)
                if self.mode == SourceMode::Synthetic && self.show_gt {
                    for d in &self.defects {
                        let r = egui::Rect::from_min_max(
                            egui::pos2(rect.min.x + (d.cx - d.rw) * rect.width(), rect.min.y + (d.cy - d.rh) * rect.height()),
                            egui::pos2(rect.min.x + (d.cx + d.rw) * rect.width(), rect.min.y + (d.cy + d.rh) * rect.height()),
                        );
                        painter.rect_stroke(r, 2.0, egui::Stroke::new(1.2_f32, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 165)));
                    }
                }

                // Draw FPGA NPU 16x6 Heatmap + Merged Bounding Boxes
                if let Some(res) = self.last_res.as_ref() {
                    let cw = rect.width() / GRID_W as f32;
                    let ch = rect.height() / GRID_H as f32;

                    if self.show_heatmap {
                        for gy in 0..GRID_H {
                            for gx in 0..GRID_W {
                                let idx = gy * GRID_W + gx;
                                let cls = res.classes[idx];
                                let conf = res.confs[idx];
                                let cr = egui::Rect::from_min_size(
                                    egui::pos2(rect.min.x + gx as f32 * cw, rect.min.y + gy as f32 * ch),
                                    egui::vec2(cw, ch),
                                );
                                painter.rect_stroke(cr, 0.0, egui::Stroke::new(0.4_f32, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 25)));
                                if cls > 0 && conf >= self.cell_conf_thresh {
                                    let col = class_color(cls);
                                    let fill = egui::Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), 85);
                                    painter.rect_filled(cr, 0.0, fill);
                                }
                            }
                        }
                    }

                    if self.show_boxes {
                        for b in self.merge_defect_boxes(res) {
                            let col = class_color(b.cls);
                            let br = egui::Rect::from_min_max(
                                egui::pos2(rect.min.x + b.gx0 as f32 * cw, rect.min.y + b.gy0 as f32 * ch),
                                egui::pos2(rect.min.x + (b.gx1 + 1) as f32 * cw, rect.min.y + (b.gy1 + 1) as f32 * ch),
                            );
                            painter.rect_stroke(br, 3.0, egui::Stroke::new(2.5_f32, col));
                            let tag = format!("{} {}%", CLASS_SHORT[b.cls as usize], b.max_conf);
                            let tpos = egui::pos2(br.min.x + 2.0, (br.min.y - 16.0).max(rect.min.y + 2.0));
                            let bg = egui::Rect::from_min_size(tpos, egui::vec2(tag.len() as f32 * 6.8 + 6.0, 15.0));
                            painter.rect_filled(bg, 2.0, egui::Color32::from_rgba_unmultiplied(15, 15, 18, 210));
                            painter.text(
                                egui::pos2(tpos.x + 3.0, tpos.y + 1.0),
                                egui::Align2::LEFT_TOP,
                                tag,
                                egui::FontId::monospace(11.0),
                                col,
                            );
                        }
                    }
                }
            }
        });
    }
}

fn main() -> eframe::Result<()> {
    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1180.0, 620.0]),
        ..Default::default()
    };
    eframe::run_native(
        "TangWood Studio — FPGA NPU Wood Defect Inspector",
        opts,
        Box::new(|cc| Ok(Box::new(WoodStudioApp::new(cc)))),
    )
}
