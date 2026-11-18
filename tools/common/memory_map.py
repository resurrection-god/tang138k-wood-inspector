"""Карта памяти платы глазами ПК: кольца кадров в DDR3, раскладка слота,
блоки телеметрии и почтовый ящик.

Одна копия на все инструменты ПК. Остальные копии этих чисел — в RTL
(`fpga_camera_ddr3_streamer.v`: `RING_BASE_ADDR`, `NUM_SLOTS`, `row_slot_off`,
параметры экземпляров в верхнем модуле), в прошивке (`memory_map.h`:
`VIDEO_RING_*`, `VIDEO_SLOT_BYTES`, `VIDEO_ROW_OFFSET()`) и в стенде
(`fpga/sim/tb_streamer.v`: `SLOT_SIZE`, `NUM_SLOTS`). Менять только все вместе:
расхождение не даёт ошибки — сканирование слотов молча ничего не находит, а
строки читаются чужие (CLAUDE.md, «DDR3 ring buffer and the VID0 header»).
"""

# Кадр: 1080 x 1280 Mono8, полный массив сенсора с 17.09.2026 (ERRORS гл. 31.44).
FRAME_WIDTH = 1080
FRAME_HEIGHT = 1280

# Строка занимает целое число пакетов Lane 4 по 256 Б: 1080 пикселей + 8 Б
# трейлера {0xA5A5, номер строки сенсора, счётчик кадров} + заполнение.
BURST_BYTES = 256
ROW_TAG_BYTES = 8
ROW_TAG_MAGIC = 0xA5A5


def row_stride(width=FRAME_WIDTH):
    """Шаг строки в слоте: ширина + трейлер, до целого пакета (VIDEO_ROW_STRIDE)."""
    return (width + ROW_TAG_BYTES + BURST_BYTES - 1) // BURST_BYTES * BURST_BYTES


ROW_STRIDE = row_stride()   # 1280

# Кольца: по 32 слота x 2 МБ на камеру (с 18.09.2026, docs/architecture/DUAL_CAMERA.md).
RING_BASES = (0x08000000, 0x0C000000)
RING_SLOTS = 32
SLOT_BYTES = 2 * 1024 * 1024
# Плотное кольцо кадров без строк (20.10.2026): своё, 256 МБ на камеру, 8192 слота по 32 КБ
# (laser_block.DRY_*). Зеркала — DENSE_BASE_ADDR у cam_channel в scanner_top.v,
# DENSE_RING*_ADDR в firmware/src/app/memory_map.h, DENSE_BASE в fpga/sim/tb_streamer.v.
DENSE_BASES = (0x20000000, 0x30000000)

# Слот: 256-байтный заголовок VID0, сразу за ним строки.
HDR_BYTES = 256
PIX_BASE_OFF = 0x100
VID0_MAGIC = 0x56494430       # слово 0 заголовка, "VID0"
VID0_INTEGRITY = 0x12345678   # слово 5 заголовка

# Телеметрия ПЛИС (по блоку на камеру) и почтовый ящик блока синхронизации.
DIAG_ADDRS = (0x06000000, 0x06000200)
MAILBOX_ADDR = 0x06000100

# Выдача кадров по UDP прямо из ПЛИС (udp_tx, 13.10.2026): описатель задания
# (пишет процессор, 96 Б) и итог (пишет ПЛИС, 64 Б: слово этого задания и слово
# счётчиков). Раскладки — UDP_FRAME_STREAM_PLAN.md § 11; зеркала — frame_udp.h,
# параметры udp_rd в fpga/src/udp_rd.v. Для отладки: mem_dump(UDP_STAT_ADDR, 64).
UDP_DESC_ADDR = 0x06000400
UDP_STAT_ADDR = 0x06000480
UDP_BOARD_PORT = 39153

# Таблица «пиксель -> мм» камеры (R-38): пишет процессор (0x0132), полосу читает
# загрузчик ПЛИС. Зеркало — MM_TABLE*_ADDR в firmware/src/app/memory_map.h.
MM_TABLE_ADDRS = (0x10000000, 0x10100000)

# Такт lane4_clk — ровно DDR3_MEMORY_CLK/4. С 29.09.2026 память 400 МГц (в
# паспорте, ERRORS 1.11, опыт E3b), значит 100 МГц и 10 нс; 17..29.09 было
# 400/3 МГц и 7.5 нс. В этих тактах 0x0300 отдаёт задержки и ширины
# синхронизатора, ПЛИС ведёт hw_timestamp. Зеркало LANE_TICK_PS в
# firmware/src/app/board.h и LANE_CLK_HZ у cam_channel в scanner_top.v.
LANE_CLK_HZ = 100_000_000
LANE_TICK_NS = 10


def row_slot_offset(row, stride=ROW_STRIDE):
    """Смещение строки `row` от начала слота. Та же формула, что VIDEO_ROW_OFFSET()."""
    return PIX_BASE_OFF + row * stride


def slot_addr(cam, slot):
    """Адрес слота `slot` в кольце камеры `cam`."""
    return RING_BASES[cam] + slot * SLOT_BYTES
