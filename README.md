# Tang Mega 138K Pro — Headless Spatial Wood Defect Inspector (`SIMD-16` NPU v3.0.0)

Полностью автономный **headless FPGA-инспектор дефектов пиломатериалов** на базе **Sipeed Tang Mega 138K Pro Dock** (`GW5AST-LV138FPG676AC1/I0`, RISC-V `AE350_SOC` + собственный `SIMD-16` INT8 NPU v3.0.0) и десктоп-студии на Rust (**TangWood Studio**).

Видеовыход на физический экран (LCD/HDMI) полностью исключён из FPGA: освобождённые BSRAM, логика, трассировка и 22 пина направлены на **четырёхкратное ускорение NPU (`SIMD-16`)** и передачу полноцветных досок (`256×96 RGB888`) и пространственной карты дефектов (`16×6×5`) напрямую по Gigabit Ethernet (`UDP :5555`, протокол `"WD01"`).

---

## Архитектура системы

| Компонент | Детали реализации | Источник |
|---|---|---|
| **NPU Core (`v3.0.0`)** | 16 параллельных INT8×INT8 MAC (`8 × MULTADDALU12X12`), 4-банковая BSRAM весов (`64 KB`), 4-банковая BSRAM активаций (`1 KB`), APB-мост (`0xF8000900`), аппаратный `MAP_HASH = 0x780EC09F` | [npu_core.v](fpga/src/npu_core.v), [regs.toml](regs.toml) |
| **Пространственная CNN (`WoodFCNShuffleNet`)** | `ShuffleNetV2-x0.5 FCN` (`168,613` параметров, stage4 с `dilation=2`, без global pooling) → вход `256×96 RGB888`, выход `16×6×5` INT16 логитов (`16×16 px` на ячейку) | [train_wood_fcn.py](tools/npu/train_wood_fcn.py) |
| **Классы дефектов (5)** | `0: CLEAN` · `1: LIVE_KNOT` · `2: DEAD_KNOT` · `3: CRACK` · `4: RESIN_STAIN` + автогрейдинг доски (`Grade A / B / C / Брак`) | [lcd_hud.c](firmware/src/app/lcd_hud.c) |
| **Датасет и обучение** | `8,111` реальных досок (`iluvvatar/wood_surface_defects`) + `2,000` синтетических досок (Сосна / Ель / Дуб), `7` эпох FP32 + `3` эпохи самодистилляционной INT8 QAT | [train_wood_fcn.py](tools/npu/train_wood_fcn.py) |
| **Прошивка AE350 (FreeRTOS + lwIP)** | Послойное исполнение на NPU + глубина 3×3 (`dil=1/2`), проверка `MAP_HASH` в `lcd_hud_init()`, UDP `:5555` сервер `"WD01"` (`369,844` Б, `<= 512 KB`) | [lcd_hud.c](firmware/src/app/lcd_hud.c), [npu_weights.h](firmware/src/app/npu_weights.h) |
| **TangWood Studio (Rust / egui)** | Генератор синтетических досок + интерактивная кисть дефектов + загрузка реальных фото + live UDP `:5555` + тепловая карта `16×6`, bounding boxes, GT-рамки и телеметрия | [studio/src/main.rs](studio/src/main.rs) |

---

## Ресурсы FPGA и сравнение Headless (`SIMD-16`) vs LCD (`SIMD-4`)

Замеры из отчётов Gowin PnR (`metrics.json`):

| Метрика | LCD (`SIMD-4` v2, [tang138k-npu-lcd](https://github.com/resurrection-god/tang138k-npu-lcd)) | Headless (`SIMD-16` v3, этот проект) | Эффект |
|---|---|---|---|
| **Logic (LUT)** | `15,961 / 138,240 (12%)` | **`14,267 / 138,240 (11%)`** | `-1,694 LUT` |
| **Registers (FF)** | `16,962 / 139,140 (13%)` | **`16,316 / 139,140 (12%)`** | `-646 FF` |
| **BSRAM** | `113 / 340 (34%)` (`32 KB` NPU + LCD) | **`117 / 340 (35%)`** (`64 KB` NPU, без LCD) | Вся память ушла в NPU |
| **DSP (`MULTADDALU12X12`)** | `2 / 298` (`SIMD-4`) | **`8 / 298 (3%)`** (`SIMD-16`) | **`4×` MAC/такт (`1.6 GMAC/s` @ 100 MHz)** |
| **WNS / WHS** | `+0.136 ns` / `+0.136 ns` | **`+0.049 ns`** / **`+0.108 ns`** | Все 6 доменов `PASS` |
| **Вход NPU** | `200×120 RGB332` (`8-bit` экранный формат) | **`256×96 RGB888`** (`24-bit` полноцвет) | Без потерь цветового градиента |

---

## Метрики модели (`WoodFCNShuffleNet`, 256×96 → 16×6×5)

- **FP32 (`Ep 7/7`):** `cell = 96.33%`, `board_defect = 95.63%`, `mean_recall = 83.24%` (`[97.3 / 78.3 / 73.7 / 85.0 / 81.9]`).
- **Folded INT8 QAT (`3 epochs`):** `cell = 96.26%`, `board_defect = 95.32%`, `mean_recall = 82.77%` (`[97.2 / 75.8 / 76.4 / 81.7 / 82.7]`).
- **Bit-exact `SIMD-16` эмулятор (120 отложенных досок):** `cell = 95.49%`, `board_defect = 111/120 (92.5%)` (`[96.8 / 60.9 / 67.1 / 68.6 / 77.6]`).

---

## Быстрый старт

### 1. Запуск TangWood Studio (PC)
```powershell
cd studio
cargo run --release
# или напрямую:
.\target\release\tang_wood_studio.exe
```
В приложении:
- Выберите породу древесины (**Сосна / Ель / Дуб**), сгенерируйте случайную доску или ставьте дефекты кликом по доске (**Здоровый сучок / Выпадающий сучок / Трещина / Смола·Синева**).
- Или откройте реальные фото из `tools/npu/samples/` и свои изображения.
- Включите **«Авто-скан на ПЛИС»** или нажмите **«Отправить на ПЛИС (10.0.0.222:5555)»**.

### 2. Загрузка на Tang Mega 138K Pro
```powershell
# 1. Загрузить headless SIMD-16 битстрим (BUILD_ID = 8A9CABDA) в SRAM платы:
powershell -ExecutionPolicy Bypass -File tools\board\run_sram.ps1 -SkipBuild -UsbLocation 4641

# 2. Через ~15 секунд (подъём Ethernet 10.0.0.222) прошить OTA Slot B прошивкой:
python tools\board\ota_upload.py --host 10.0.0.222 --bin firmware\Release\output\ae350_scanner.bin

# 3. Прогнать автоматический бит-точный тест живой платы по UDP :5555:
C:\Users\penis\.gemini\antigravity\scratch\npu_ml_venv\Scripts\python.exe tools\npu\test_wood_board.py --host 10.0.0.222
```

### 3. Проверка регмапа и метрик CI
```powershell
python tools\npu\regmap.py --check
powershell -ExecutionPolicy Bypass -File fpga\check_timing.ps1
python fpga\extract_metrics.py
```

---

## Структура репозитория

- [regs.toml](regs.toml) & [generated/](generated/) — единый источник карты регистров `SIMD-16` (`MAP_HASH = 0x780EC09F`).
- [fpga/src/npu_core.v](fpga/src/npu_core.v), [fpga/src/npu_apb_wrapper.v](fpga/src/npu_apb_wrapper.v) — RTL ускорителя `SIMD-16` и headless APB-обёртки.
- [fpga/impl/pnr/scanner.fs](fpga/impl/pnr/scanner.fs) — готовый битстрим (`BUILD_ID = 8A9CABDA`).
- [firmware/src/app/lcd_hud.c](firmware/src/app/lcd_hud.c) & [firmware/Release/output/ae350_scanner.bin](firmware/Release/output/ae350_scanner.bin) — прошивка AE350 и `"WD01"` UDP-сервер.
- [studio/](studio/) — десктоп-приложение **TangWood Studio** на Rust (`eframe 0.29` / `egui`).
- [tools/npu/](tools/npu/) — обучение (`train_wood_fcn.py`), генератор регмапа (`regmap.py`), тестовые реальные доски (`samples/`) и живой тест платы (`test_wood_board.py`).
