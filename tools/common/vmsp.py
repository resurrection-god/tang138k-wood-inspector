"""Ядро протокола платы: VMSP по СТП-ПАК-026-NET-MVP (VCP-IP v0.2).

Константы (порты, опкоды, статусы, раскладки тел), кадрирование пакета и все
запросы к плате. Отсюда их берут и CLI `tools/capture/stp_client.py`, и
остальные инструменты; stp_client реэкспортирует всё это, так что старый
`import stp_client as stp` работает как прежде.

Нормативное описание — docs/protocol/, поправки — раздел 7
docs/architecture/PROTOCOL_AND_SYNC_PLAN.md; опкоды на стороне платы —
firmware/src/app/vmsp_service.c.
"""

import array
import os
import select
import socket
import struct
import sys
import threading
import time
import zlib

from .memory_map import LANE_CLK_HZ, LANE_TICK_NS  # noqa: F401 (реэкспорт через stp_client)
from .memory_map import DIAG_ADDRS, FRAME_WIDTH, UDP_BOARD_PORT

# Адрес платы по умолчанию: с 13.10.2026 — 192.168.31.125 (решение владельца,
# прямой провод для выдачи кадров по UDP; до того 10.0.0.125). VMSP_HOST —
# другой адрес для всех инструментов сразу.
#
# С 15.10.2026 адрес платы хранится в её флеш и меняется командой 0x0003
# (tools/net/board_ip.py --set), а ПК помнит его в tools/board_host.txt (вне git):
# переход между прямым проводом и офисной сетью не требует правки кода.
# Порядок: VMSP_HOST, затем board_host.txt, затем адрес по умолчанию.
HOST_DEFAULT = "10.0.0.33"   # с 03.11.2026 (владелец; было 192.168.31.125 с 13.10.2026, 10.0.0.125 до того)
HOST_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "board_host.txt")


def _host_from_file():
    try:
        with open(HOST_FILE, encoding="utf-8") as f:
            h = f.read().strip()
        socket.inet_aton(h)
        return h
    except (OSError, ValueError):
        return None


def save_host(host):
    """Запомнить адрес платы для всех инструментов (tools/board_host.txt)."""
    socket.inet_aton(host)
    with open(HOST_FILE, "w", encoding="utf-8") as f:
        f.write(host + "\n")


HOST = os.environ.get("VMSP_HOST") or _host_from_file() or HOST_DEFAULT
HOST_BEFORE_20261013 = "10.0.0.125"
PORT_DISCOVERY = 39150
PORT_CONTROL = 39151
PORT_DATA = 39152

# Адрес локального адаптера ПК в сети платы. Сокеты привязываются к нему, иначе
# поднятый VPN забирает маршрут по умолчанию и трафик к плате уходит в туннель:
# ping без источника молчит, discovery отваливается по тайм-ауту, а плата при
# этом жива (ERRORS гл. 15.3; повтор 16.09.2026 с AmneziaVPN).
# Переопределяется переменной окружения STP_BIND_IP; если не задана, автоматически
# выбирается локальный IP в той же подсети (/24), что и целевой HOST (например, 10.0.0.x
# для Wi-Fi или 192.168.1.x для прямого Ethernet-подключения).
LOCAL_BIND_IP = os.environ.get("STP_BIND_IP", "")


def _find_local_bind_ip(target_host):
    if LOCAL_BIND_IP:
        return LOCAL_BIND_IP
    try:
        target_parts = str(target_host).split(".")
        if len(target_parts) == 4:
            prefix = ".".join(target_parts[:3]) + "."
            local_ips = socket.gethostbyname_ex(socket.gethostname())[2]
            for ip in local_ips:
                if ip.startswith(prefix):
                    return ip
    except Exception:
        pass
    if str(target_host).startswith("192.168.1."):
        return "192.168.1.10"
    return None


def _bind_local(sock, target_host=None):
    """Привязать сокет к адаптеру в сети платы, если такой адрес есть."""
    bind_ip = _find_local_bind_ip(target_host or HOST)
    if not bind_ip:
        return
    try:
        sock.bind((bind_ip, 0))
    except OSError:
        pass        # адреса нет на этой машине — идём маршрутом по умолчанию

VER = 0x0002
NODE = 1
HDR = struct.Struct(">4sHHIIIII")          # 28 Б
META = struct.Struct(">QQHHBBIIHH")        # 34 Б метаданных чанка

OP_PROBE, OP_INFO = 0x0001, 0x0002
OP_MODE, OP_MODE_ACK = 0x0110, 0x0111
OP_CAMERA = 0x0112
OP_EXPOSURE, OP_EXPOSURE_ACK = 0x0120, 0x0121
OP_GET_CAM, OP_GET_CAM_ACK = 0x0122, 0x0123
OP_SYNC_CHANNEL, OP_SYNC_CONTROL = 0x0140, 0x0142
OP_SW_TRIGGER, OP_SW_TRIGGER_ACK = 0x0145, 0x0146
OP_CALIB, OP_CALIB_ACK = 0x0130, 0x0131
# Таблица «пиксель -> мм» (R-38, tools/laser/mm_table.py): запись, фиксация, состояние.
OP_MM_WRITE, OP_MM_COMMIT, OP_MM_INFO = 0x0132, 0x0134, 0x0136
OP_LASER_CFG, OP_LASER_CFG_ACK = 0x0150, 0x0151
OP_FETCH_3D, OP_SLICE = 0x0210, 0x0211
OP_CAPTURE, OP_IMAGE, OP_ACQUIRE = 0x0220, 0x0221, 0x0222
OP_STATUS, OP_TRIG_SERIES, OP_MEM_DUMP = 0x0300, 0x0302, 0x0306
# Хранилище настроек во флеш платы (до 22.10.2026 — EEPROM U24, R-40) (docs/architecture/MODES_AND_SETTINGS.md, § 2.7).
OP_SET_SAVE, OP_SET_LOAD, OP_SET_INFO = 0x0160, 0x0162, 0x0164
OP_SET_RESET, OP_SET_EXPORT, OP_SET_IMPORT = 0x0166, 0x0168, 0x016A

# Безопасное сетевое обновление (OTA A/B Slot) (0x0170..0x017D)
OP_OTA_START, OP_OTA_START_ACK = 0x0170, 0x0171
OP_OTA_DATA, OP_OTA_DATA_ACK = 0x0172, 0x0173
OP_OTA_COMMIT, OP_OTA_COMMIT_ACK = 0x0174, 0x0175
OP_OTA_REBOOT, OP_OTA_REBOOT_ACK = 0x0176, 0x0177
OP_OTA_STATUS, OP_OTA_STATUS_ACK = 0x0178, 0x0179
OP_OTA_SET_SLOT, OP_OTA_SET_SLOT_ACK = 0x017A, 0x017B
OP_OTA_VERSION, OP_OTA_VERSION_ACK = 0x017C, 0x017D

OTA_TARGET_FIRMWARE = 0
OTA_TARGET_BITSTREAM = 1

OTA_BOOT_STATUS = {0: "CONFIRMED", 1: "TENTATIVE", 2: "FAILED"}
OTA_SLOT_NAMES = {0: "Slot A (Golden 0x600000)", 1: "Slot B (Candidate 0xF60000)"}
OTA_FALLBACK_REASONS = {0: "None (Normal)", 1: "Boot attempts exceeded (Max 3)", 2: "Flash read-back CRC mismatch"}
OTA_LED_PATTERNS = {
    0: "Calm Alternating (LED0 <-> LED1, 1 Hz)",
    1: "Synchronous Double-Strobe (Flash-Flash 80ms, 1 Hz)",
}

STATUS_NAMES = {0x00: "OK", 0x02: "BUSY", 0x04: "BAD_VERSION", 0x05: "BAD_OPCODE",
                0x06: "BAD_LENGTH", 0x07: "BAD_CRC", 0x08: "BAD_TARGET", 0x09: "DENIED",
                0x0A: "HW_ERROR", 0x0B: "TIMEOUT", 0x0C: "UNSUPPORTED"}
MODES = {"standby": 0, "work": 1, "service": 2}
SOURCES = {"core": 0, "timer": 1, "ext": 2}

# Ответ 0x0301: 34 слова uint32 BE (vmsp_service.c, handle_status). Было 29;
# слова 29..32 добавлены вместе с режимами и искателем линии, слово 33 — с
# хранилищем настроек. Разбор идёт по длине тела, так что клиент работает и со
# старой прошивкой.
STATUS_FIELDS = [
    "fw_bits", "fpga_frames", "fpga_timer", "fpga_rows", "trig_pulses",
    "sync_status", "accepted", "overrun", "ext_edges", "last_trig_ts",
    "apply_seq_fpga", "apply_seq_fw", "sw_seq_fpga", "sw_seq_fw", "sync_ctrl",
    "cam0_delay", "cam0_width", "cam1_delay", "cam1_width", "laser_delay", "laser_width",
    "laser_mode", "ext_filter", "polarity",
    "cam_trg_mode", "cam_trg_src", "cam_triggers", "cam_lost", "telemetry_ok",
    # 30..32 с 24.09.2026 (L2a): эхо искателя камеры 0 (слово 36 её телеметрии)
    # и {блоков профиля, точек без линии} камер 0 и 1.
    "mode_bits", "lasr_echo0", "lasr_prof0", "lasr_prof1",
    "settings_bits",
    # 34, 35 с 21.10.2026: сторож приёма Ethernet — {потерянных прерываний RX за 20 с,
    # за всё время}, {выкачанных кадров, поправок скорости MAC} (по 16 бит, насыщаются).
    "eth_rx_wd", "eth_rx_wd2",
    # 36, 37 с 22.10.2026: камера 1 — триггеров, пропущено ею (0x0418, как 26, 27 у камеры 0).
    "cam1_triggers", "cam1_lost",
    # 38 с 22.10.2026: номер кадра ПЛИС камеры 1 (как слово 1 у камеры 0).
    "fpga_frames1",
]


QUIET_S = 1.5   # телеметрия кадров ПЛИС обновляется раз в 0.5 с: столько ждать после остановки генератора


def quiet_status(saved, host=HOST, restart=True):
    """Статус 0x0300 для trigger_misses: генератор стенда (profile_rig_setup, источник TIMER)
    остановлен на QUIET_S и, если restart, запущен снова. Снятый на ходу статус врёт: номер
    кадра в телеметрии отстаёт до 0.5 с — до 500 кадров на 1000 Гц (ERRORS 2.16)."""
    profile_rig_stop(host=host)
    time.sleep(QUIET_S)
    st = status(host)
    if restart:
        trig_series(saved["freq"], 0, max(1, saved["cam_w_ns"] // 1000), host=host)
    return st


def trigger_misses(before, after):
    """Сколько импульсов пропало между двумя статусами 0x0300 (снятыми quiet_status, когда генератор стоит):
    'overrun' — пришли во время прошлой серии блока синхронизации; {камера: проигнорировано
    камерой} (её счёт 0x0418, 16 бит); 'unframed' — {камера: принятых импульсов минус кадров ПЛИС},
    второй, независимый от камеры свидетель. Камеры без нужных слов (старая прошивка) не попадают."""
    acc = (after["accepted"] - before["accepted"]) & 0xFFFFFFFF
    out = {"overrun": (after["overrun"] - before["overrun"]) & 0xFFFFFFFF, "unframed": {}}
    for cam, lost, frames in ((0, "cam_lost", "fpga_frames"), (1, "cam1_lost", "fpga_frames1")):
        if lost in before and lost in after:
            out[cam] = (after[lost] - before[lost]) & 0xFFFF
        if frames in before and frames in after:
            out["unframed"][cam] = acc - ((after[frames] - before[frames]) & 0xFFFFFFFF)
    return out
STATUS_WORDS_MIN = 29          # прошивка до 18.09.2026
ETH_SICK_RECENT = 3            # потерянных прерываний RX за 20 с: столько и больше — «больная» загрузка


def stored_data_alarm(st):
    """По статусу 0x0300: что плата взяла не из новейшей копии во флеш (слово 33 [18:16], ERRORS 5.15) -
    список названий; пустой - всё из новейших копий или прошивка без этих битов."""
    bits = (st.get("settings_bits", 0) >> 16) & 0x7
    return [n for b, n in ((1, "настройки"), (2, "таблица мм кам 0"), (4, "таблица мм кам 1")) if bits & b]


def eth_rx_sick(st):
    """По статусу 0x0300: (больна ли загрузка, потерь за 20 с) или (None, None) у прошивки без слова 34.
    Здоровая плата за сутки набирает ~1 потерю в 46 с, больная — 1–7 за 20 с даже в простое (ERRORS 3.15, 3.16)."""
    if "eth_rx_wd" not in st:
        return None, None
    recent = st["eth_rx_wd"] & 0xFFFF
    return recent >= ETH_SICK_RECENT, recent

# Хранилище настроек: разделы, флаги (SETTINGS_F_* в settings.h), образ.
SET_SECTIONS = {"cfg": 0, "calib": 1}
SET_FLAG_NAMES = [(0x01, "есть во флеш"), (0x02, "НЕ СОХРАНЕНО"), (0x04, "копия повреждена"),
                  (0x08, "хранилище недоступно"), (0x10, "умолчания"),
                  (0x20, "часть значений отвергнута")]
SET_REC = struct.Struct(">BBBBHHI")           # запись о разделе, 12 Б
SET_IMG_HDR = struct.Struct(">IHHII")         # magic, версия, длина, CRC, seq — 16 Б
SET_MAGIC = {0: 0x53434647, 1: 0x5343414C}    # "SCFG", "SCAL"
SET_RESET_GUARD = 0x52534554                  # "RSET"

# Настройка искателя линии (0x0150) и качество точки (0x0211).
LASER_AXES = {"row": 0, "col": 1}
LASER_CFG_F_KEEP_FRAME, LASER_CFG_F_WRITE_PROF, LASER_CFG_F_SOFT = 0x01, 0x02, 0x04
Q_BITS = ["no_line", "saturated", "wide", "multi", "disagree", "uncalib", "partial", "bit7"]
SLICE_HDR = struct.Struct(">IIHQQBB")      # 28 Б: num, id, points, encoder, ts_us, status, flags



class ProtocolError(Exception):
    pass


def st_name(code):
    return STATUS_NAMES.get(code, f"0x{code:02X}")


def packet(opcode, seq, payload=b"", target=NODE, crc=None, version=VER):
    if crc is None:
        crc = zlib.crc32(payload) & 0xFFFFFFFF if payload else 0
    return HDR.pack(b"VMSP", version, opcode, seq, 0, target, len(payload), crc) + payload


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(1 << 20, n - len(buf)))
        if not chunk:
            raise ProtocolError(f"соединение закрыто ({len(buf)}/{n} байт)")
        buf += chunk
    return bytes(buf)


def read_message(sock, check_crc=True):
    raw = recv_exact(sock, HDR.size)
    magic, ver, op, seq, sender, target, plen, crc = HDR.unpack(raw)
    if magic != b"VMSP" or ver != VER:
        raise ProtocolError(f"плохой заголовок: {raw.hex()}")
    payload = recv_exact(sock, plen) if plen else b""
    if check_crc and crc != 0:
        calc = zlib.crc32(payload) & 0xFFFFFFFF if payload else 0
        if calc != crc:
            raise ProtocolError(f"CRC тела 0x{op:04X}: 0x{crc:08X} против 0x{calc:08X}")
    return op, seq, sender, target, payload


def connect(port, host=HOST, timeout=90.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    _bind_local(s, host)
    s.connect((host, port))
    return s


def transact(sock, opcode, payload=b"", seq=1, **kw):
    sock.sendall(packet(opcode, seq, payload, **kw))
    op, rseq, *_rest, body = read_message(sock)
    if rseq != seq:
        raise ProtocolError(f"seq ответа {rseq}, ждали {seq}")
    return op, body


def control(opcode, payload=b"", host=HOST, port=PORT_CONTROL, timeout=15.0):
    """Одна команда на своём соединении; возвращает (opcode ответа, тело)."""
    with connect(port, host, timeout) as s:
        return transact(s, opcode, payload)


def expect_ok(what, op, body, want_op):
    if op != want_op or not body or body[0] != 0:
        raise ProtocolError(f"{what}: 0x{op:04X} status {st_name(body[0]) if body else '-'}")


# ---- команды ------------------------------------------------------------------

def discover(host=HOST, timeout=1.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    _bind_local(s, host)
    try:
        s.sendto(packet(OP_PROBE, 1, b"DISC", target=0), (host, PORT_DISCOVERY))
        data, _addr = s.recvfrom(512)
    except (ConnectionResetError, ConnectionRefusedError) as e:
        raise TimeoutError(f"discovery: модуль {host} недоступен ({e})")
    finally:
        s.close()
    return _parse_info(data)


def _parse_info(data):
    op = struct.unpack_from(">H", data, 6)[0] if len(data) >= 8 else -1
    if op != OP_INFO or len(data) < 28 + 28:
        raise ProtocolError(f"discovery: opcode 0x{op:04X}, {len(data)} байт")
    p = data[28:]
    return {"uuid": p[:16].hex(), "mac": ":".join(f"{b:02X}" for b in p[16:22]),
            "ip": socket.inet_ntoa(p[22:26]), "module_id": p[26], "fw": p[27]}


def _local_ipv4():
    """IPv4-адреса адаптеров ПК (без 127.x). Пусто — широковещание с маршрута по умолчанию."""
    try:
        ips = socket.gethostbyname_ex(socket.gethostname())[2]
    except OSError:
        ips = []
    return [ip for ip in dict.fromkeys(ips) if not ip.startswith("127.")]


def discover_all(timeout=1.0, hosts=()):
    """Поиск плат без адреса: зонд 0x0001 широковещанием 255.255.255.255 с каждого
    адаптера ПК и, вдобавок, прямо на адреса `hosts`. Возвращает список ответов
    (как у discover(), плюс "addr" — откуда пришёл ответ), без повторов по MAC.

    С 02.10.2026 плата отвечает только в STANDBY (vmsp_service.c): пустой список
    не значит, что платы нет, — в WORK/SERVICE её находят по TCP (status())."""
    socks = []
    for ip in (_local_ipv4() or [""]):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            s.bind((ip, 0))
        except OSError:
            s.close()
            continue
        socks.append(s)
    probe = packet(OP_PROBE, 1, b"DISC", target=0)
    for s in socks:
        for dst in ["255.255.255.255"] + [h for h in hosts if h]:
            try:
                s.sendto(probe, (dst, PORT_DISCOVERY))
            except OSError:
                pass             # адаптер без маршрута к этому адресу
    found = {}
    t_end = time.monotonic() + timeout
    try:
        while socks:
            left = t_end - time.monotonic()
            if left <= 0:
                break
            ready, _w, _x = select.select(socks, [], [], left)
            for s in ready:
                try:
                    data, addr = s.recvfrom(512)
                    info = _parse_info(data)
                except (OSError, ProtocolError):
                    continue     # ICMP unreachable на Windows, чужой пакет
                info["addr"] = addr[0]
                found.setdefault(info["mac"], info)
    finally:
        for s in socks:
            s.close()
    return list(found.values())


OP_ASSIGN_IP, OP_ASSIGN_IP_ACK = 0x0003, 0x0004


def _ip4(s):
    return socket.inet_aton(s)


def prefix_to_mask(prefix):
    return socket.inet_ntoa(struct.pack(">I", (0xFFFFFFFF << (32 - int(prefix))) & 0xFFFFFFFF))


def assign_ip(mac, ip, mask="255.255.255.0", gw="0.0.0.0", store=True, timeout=2.0):
    """Сменить адрес платы (0x0003, спец. § 4.1): широковещанием с каждого адаптера ПК,
    плата узнаёт себя по MAC, поэтому годится и плата в чужой подсети. Только в
    STANDBY. store=False — без записи во флеш (до перезагрузки). Возвращает
    имя статуса ("OK", "DENIED" — значения не годятся, "HW_ERROR" — флеш),
    None — ответа нет. Новый адрес плата берёт через ~50 мс после ответа."""
    body = mac_bytes(mac) + _ip4(ip) + _ip4(mask) + _ip4(gw) + bytes([0 if store else 1, 0, 0, 0])
    return _broadcast_ask(OP_ASSIGN_IP, OP_ASSIGN_IP_ACK, body, timeout)


def mac_bytes(mac):
    b = bytes(int(x, 16) for x in mac.replace("-", ":").split(":"))
    if len(b) != 6:
        raise ValueError(f"MAC {mac}")
    return b


OP_ASSIGN_MAC, OP_ASSIGN_MAC_ACK = 0x0005, 0x0006


def assign_mac(mac, new_mac, timeout=2.0):
    """Записать плате новый MAC (0x0005, спец. § 4.1): во флеш, действует со следующей
    загрузки. new_mac "00:00:00:00:00:00" стирает запись (вернётся MAC по умолчанию).
    Только в STANDBY. Статус: "OK", "DENIED" (не одноадресный MAC), "HW_ERROR" (флеш), None."""
    return _broadcast_ask(OP_ASSIGN_MAC, OP_ASSIGN_MAC_ACK, mac_bytes(mac) + mac_bytes(new_mac), timeout)


def _broadcast_ask(op, ack, body, timeout):
    """Широковещательный запрос порту discovery с каждого адаптера ПК, статус ответа ack."""
    pkt = packet(op, 3, body, target=0)
    socks = []
    for lip in (_local_ipv4() or [""]):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            s.bind((lip, 0))
            s.sendto(pkt, ("255.255.255.255", PORT_DISCOVERY))
        except OSError:
            s.close()
            continue
        socks.append(s)
    t_end = time.monotonic() + timeout
    try:
        while socks:
            left = t_end - time.monotonic()
            if left <= 0:
                return None
            ready, _w, _x = select.select(socks, [], [], left)
            for s in ready:
                try:
                    data, _addr = s.recvfrom(512)
                except OSError:
                    continue
                if len(data) >= 29 and struct.unpack_from(">H", data, 6)[0] == ack:
                    return st_name(data[28])
    finally:
        for s in socks:
            s.close()
    return None


MODE_STANDBY, MODE_WORK, MODE_SERVICE = MODES["standby"], MODES["work"], MODES["service"]
MODE_NAMES = {MODE_STANDBY: "STANDBY", MODE_WORK: "WORK", MODE_SERVICE: "SERVICE"}


def get_mode(host=HOST):
    """Текущий режим платы из 0x0300 (слово 29, младший байт)."""
    st = status(host)
    if "mode_bits" not in st:
        raise ProtocolError("0x0300 без слова режима: прошивка до 18.09.2026")
    return st["mode_bits"] & 0xFF


def ensure_active(host=HOST, mode=MODE_SERVICE, verbose=True):
    """Вывести плату из STANDBY: съёмка (0x0220/0x0222/0x0210/0x0212) в нём
    запрещена, а с 02.10.2026 плата в нём стартует. WORK и SERVICE не трогает.
    Возвращает режим, в котором плата была."""
    was = get_mode(host)
    if was == MODE_STANDBY:
        r = set_mode(mode, host)
        if verbose:
            print(f"плата была в STANDBY -> {MODE_NAMES.get(r['mode'], r['mode'])} "
                  f"(флаги 0x{r['flags']:02X})")
    return was


def set_mode(mode, host=HOST, timeout=30.0):
    """0x0110: режим системы. Ответ 4 Б (было 2): status, режим, флаги перехода.

    Флаги: 0x01 ПЛИС подтвердила режим, 0x02 лазер разрешён, 0x04 в битстриме
    есть блок LASR, 0x08 съёмка сенсора остановлена по I2C."""
    op, body = control(OP_MODE, struct.pack(">H", mode), host, timeout=timeout)
    expect_ok("0x0110", op, body, OP_MODE_ACK)
    return {"mode": body[1], "flags": body[2] if len(body) > 2 else 0}


def set_exposure(us, host=HOST, gain_db=None, cam=0):
    """0x0120: экспозиция (мкс) и, если задано, усиление (дБ, шаг 0.1).
    Режим камеры не меняется — взведённая под триггер камера остаётся взведённой."""
    payload = struct.pack(">HI", cam, us)
    if gain_db is not None:
        payload += struct.pack(">H", int(round(gain_db * 10)))
    op, body = control(OP_EXPOSURE, payload, host)
    expect_ok("0x0120", op, body, OP_EXPOSURE_ACK)
    return struct.unpack_from(">I", body, 4)[0]


CAMCFG_FREE_RUN = 0x01      # в потоковом режиме запустить съём (0x0400 = 1)


def camera_config(exposure_us, gain_db, fps, trigger_mode=0, testpat=0, host=HOST, cam=0,
                  roi_w=None, roi_h=None, off_x=0, off_y=0, free_run=False):
    """trigger_mode 0 — поток, 1 — взведён аппаратный триггер (0x0404 = 1, 0x0408 = 1).

    roi_w/roi_h заданы — тело 20 Б с явным ROI; не заданы — прежние 12 Б, и плата
    ставит полный кадр 1080x1280. Ноль в поле означает «как по умолчанию», так что
    можно задать только ширину.

    ⚠ Съёмка КАДРОВ при ширине, отличной от 1080, не работает: LINE_BYTES моста
    CSI-2 — параметр RTL. Узкий ROI нужен для замера max_fps и профильного тракта.
    """
    # Частота в сотых долях герца. u16-поле упирается в 655.35 Гц, поэтому выше
    # него шлём тело в 24 байта с u32-хвостом; плата берёт его, если он не ноль.
    fps_chz = int(round(fps * 100))
    wide = fps_chz > 0xFFFF
    payload = struct.pack(">BBBBIHH", cam, trigger_mode, testpat,
                          CAMCFG_FREE_RUN if free_run else 0, exposure_us,
                          int(round(gain_db * 10)), min(fps_chz, 0xFFFF))
    if roi_w is not None or roi_h is not None or wide:
        payload += struct.pack(">HHHH", roi_w or 0, roi_h or 0, off_x, off_y)
    if wide:
        payload += struct.pack(">I", fps_chz)
    op, body = control(OP_CAMERA, payload, host, timeout=30.0)
    expect_ok("0x0112", op, body, OP_CAMERA | 1)


def get_camera_config(cam=0, host=HOST):
    """0x0122: чтение конфигурации камеры со стороны сенсора по I2C."""
    op, body = control(OP_GET_CAM, struct.pack(">H", cam), host)
    expect_ok("0x0122", op, body, OP_GET_CAM_ACK)
    if len(body) < 28:
        raise ProtocolError(f"0x0122: length {len(body)} < 28")
    st, cid, conn, mid, exp, gain, fps, tm, ts, tp, ae, rw, rh, ox, oy = struct.unpack(">BBHIIHHBBBBHHHH", body[:28])
    out = {
        "cam": cid, "connected": bool(conn), "model_id": f"0x{mid:04X}",
        "exp_us": exp, "gain_db": gain / 10.0, "fps": fps / 100.0,
        "trg_mode": tm, "trg_src": ts, "testpat": tp, "auto_exp": ae,
        "roi_w": rw, "roi_h": rh, "offset_x": ox, "offset_y": oy
    }
    # Слова 28..31 добавлены 22.09.2026: предельная частота сенсора для текущего
    # ROI. Разбор по длине — прошивка до этой даты отдаёт 28 байт.
    if len(body) >= 32:
        out["max_fps"] = struct.unpack_from(">I", body, 28)[0] / 100.0
    return out


def sync_channel(channel, delay_ns, width_ns, laser_mode=0, host=HOST):
    """channel 0 cam0, 1 cam1, 2 лазер; laser_mode 0 импульс, 1 всегда 0, 2 всегда 1, 3 предымпульс."""
    op, body = control(OP_SYNC_CHANNEL, struct.pack(">BBHII", channel, laser_mode, 0, delay_ns, width_ns), host)
    expect_ok("0x0140", op, body, OP_SYNC_CHANNEL | 1)


def sync_control(enable=1, source="timer", laser_enable=0, ext_fall=0, ext_filter=100, polarity=0, host=HOST):
    src = SOURCES[source] if isinstance(source, str) else source
    payload = struct.pack(">BBBBII", enable, src, laser_enable, ext_fall & 1, ext_filter, polarity)
    op, body = control(OP_SYNC_CONTROL, payload, host)
    expect_ok("0x0142", op, body, OP_SYNC_CONTROL | 1)


def sw_trigger(host=HOST):
    op, body = control(OP_SW_TRIGGER, b"", host)
    if op != OP_SW_TRIGGER_ACK or len(body) != 6:
        raise ProtocolError(f"0x0145: 0x{op:04X} {body.hex()}")
    return struct.unpack(">HI", body)[1]


def status(host=HOST):
    op, body = control(OP_STATUS, b"", host)
    n = len(body) // 4
    if op != OP_STATUS | 1 or n < STATUS_WORDS_MIN or len(body) % 4:
        raise ProtocolError(f"0x0300: 0x{op:04X}, {len(body)} байт")
    n = min(n, len(STATUS_FIELDS))
    return dict(zip(STATUS_FIELDS, struct.unpack_from(f">{n}I", body)))


def laser_config(cam=0, axis="col", thresh=40, gap=2, offset=0, keep_frame=True,
                 write_profile=True, soft=False, win_lo=0, win_hi=0, min_sum=64, width_max=64,
                 host=HOST):
    """0x0150: настройка искателя лазерной линии одной камеры.

    С 24.09.2026 (этап L2a) профиль считает ПЛИС, параметры свои у каждой
    камеры и общие для режимов. Тело 14 Б: камера, подложка и прежние 12 Б
    (слова 32..40 ящика для камеры 0, 41..49 — для камеры 1). Ответ 18 Б;
    fpga_state: 0x01 ящик прочитан, 0x02 ПЛИС пишет блок профиля, 0x04 ПЛИС
    подтвердила эту настройку."""
    flags = ((LASER_CFG_F_KEEP_FRAME if keep_frame else 0) |
             (LASER_CFG_F_WRITE_PROF if write_profile else 0) |
             (LASER_CFG_F_SOFT if soft else 0))
    a = LASER_AXES[axis] if isinstance(axis, str) else axis
    payload = struct.pack(">BBBBBBHHHH", cam, offset, a, thresh, gap, flags,
                          win_lo, win_hi, min_sum, width_max)
    op, body = control(OP_LASER_CFG, payload, host)
    expect_ok("0x0150", op, body, OP_LASER_CFG_ACK)
    r = {"axis": body[1], "thresh": body[2], "gap": body[3], "flags": body[4],
         "win_lo": struct.unpack_from(">H", body, 6)[0],
         "win_hi": struct.unpack_from(">H", body, 8)[0],
         "min_sum": struct.unpack_from(">H", body, 10)[0],
         "width_max": struct.unpack_from(">H", body, 12)[0],
         "fpga_lasr": bool(body[14]), "fpga_state": body[15]}
    if len(body) >= 18:                      # прошивка с L2a
        r["cam"] = body[5]
        r["offset"] = body[16]
    return r


def set_calibration(data, host=HOST):
    """0x0130: 72 Б калибровки (fx, fy, cx, cy; матрица 3x3; сдвиг; плоскость лазера).

    Прошивка держит их в ОЗУ: запись в SPI-флеш — этап 5 плана протокола."""
    if len(data) != 72:
        raise ValueError(f"калибровка: {len(data)} Б, нужно 72")
    op, body = control(OP_CALIB, data, host)
    expect_ok("0x0130", op, body, OP_CALIB_ACK)
    return struct.unpack_from(">I", body, 4)[0]


# ---- хранилище настроек во флеш (0x0160..0x016A) ------------------------------

# ---- таблица «пиксель -> мм» (0x0132 .. 0x0137, 16.10.2026, R-38) -------------
#
# Таблица строится на ПК (tools/laser/mm_table.py). Плата при старте берёт её из своего
# SPI-флеша (R-39, две копии на камеру; во флеш её пишет tools/laser/mm_flash.py по
# JTAG). Загрузка по сети кладёт её в ОЗУ до перезагрузки; save=True — плата пишет её
# и во флеш через свой контроллер flash_ctl.v (битстримы с 9574A12E; на старых ответ -5,
# ERRORS 5.12). Без таблицы во флеше точки после
# перезагрузки идут «без мм», пока клиент не загрузит её (mm_info -> id, flash).
# Плата сама считает полосу узлов под ROI камеры и отдаёт её ПЛИС; ПЛИС проверяет CRC.
MM_CHUNK = 4088                                      # VCP_MAX_PAYLOAD 4096 - 8 Б заголовка
MM_OFF_NAMES = {0: "вкл", 1: "нет таблицы", 2: "ROI выше 209 строк", 3: "ROI камеры не тот",
                4: "камера перенастраивается"}
MM_ST_NAMES = {0: "нет", 1: "загружается", 2: "готова"}
MM_HDR = struct.Struct(">BBBBHHIIBBH")               # 20 Б тела 0x0134 = mm_table.HDR
MM_COMMIT_ACK = struct.Struct(">BBBBIIBBh")          # 16 Б (+4: флеш, копия, код)
MM_INFO_ACK = struct.Struct(">BBBBIIIBBBBhHII4BI")   # 40 Б
MM_INFO_FLASH = struct.Struct(">Bb2xIIII")           # +20 Б: копии во флеше (R-39)
MM_FLASH_ERR = {-1: "вне области", -2: "таймаут", -3: "не совпало при чтении", -4: "занято",
                -5: "в битстриме нет доступа к флешу", -6: "контроллер флеша отказал в команде"}


def mm_upload(table, host=HOST, timeout=15.0, save=False):
    """Загрузить таблицу камеры: table — dict из mm_table.build_table()/load()
    (words, hdr). save=True — ещё и записать её во флеш платы (секунды, ответ
    задерживается). Возвращает dict ответа 0x0135: band (on/off и почему), FPGA
    (1 подтвердила, 0 не успела, 0xFF нет загрузчика в битстриме), flash (None —
    не просили, иначе ok, copy, rc)."""
    import numpy as np
    hdr = table["hdr"]
    raw = np.asarray(table["words"], "<u4").tobytes()
    cam = int(hdr["cam"])
    with connect(PORT_CONTROL, host, timeout) as s:
        seq = 1
        for off in range(0, len(raw), MM_CHUNK):
            chunk = raw[off:off + MM_CHUNK]
            op, body = transact(s, OP_MM_WRITE, struct.pack(">B3xI", cam, off) + chunk, seq=seq)
            expect_ok(f"0x0132 @{off}", op, body, OP_MM_WRITE | 1)
            seq += 1
        commit = MM_HDR.pack(cam, hdr["version"], hdr["gk_log2"], hdr["gv_log2"], hdr["cols"],
                             hdr["rows"], hdr["id"], hdr["crc"], hdr["unit_shift"], hdr["frame"],
                             0x0100 if save else 0)              # байт 18 бит 0 — во флеш
        op, body = transact(s, OP_MM_COMMIT, commit, seq=seq)
    if op != (OP_MM_COMMIT | 1) or len(body) < MM_COMMIT_ACK.size:
        raise ProtocolError(f"0x0134: 0x{op:04X}, {len(body)} Б")
    st, rcam, fpga, why, tid, bcrc, r0, rows, vofs = MM_COMMIT_ACK.unpack_from(body)
    if st != 0:
        raise ProtocolError(f"0x0134: status {st_name(st)}")
    flash = None
    if len(body) >= MM_COMMIT_ACK.size + 4 and body[16]:
        rc = struct.unpack_from(">b", body, 18)[0]
        flash = {"ok": body[16] == 1, "copy": body[17], "rc": rc, "error": MM_FLASH_ERR.get(rc)}
    if save and flash is None:
        raise ProtocolError("0x0134: плата не записала таблицу во флеш (прошивка без R-39?)")
    return {"cam": rcam, "fpga": fpga, "off_why": why, "band": MM_OFF_NAMES.get(why, why),
            "id": tid, "band_crc": bcrc, "r0": r0, "rows": rows, "vofs": vofs, "flash": flash}


def mm_info(cam=0, host=HOST):
    """0x0136: таблица камеры на плате и эхо загрузчика ПЛИС."""
    op, body = control(OP_MM_INFO, bytes([cam]), host)
    if op != (OP_MM_INFO | 1) or len(body) < MM_INFO_ACK.size:
        raise ProtocolError(f"0x0136: 0x{op:04X}, {len(body)} Б")
    (st, rcam, state, why, received, tid, tcrc, unit, frame, r0, rows, vofs, kofs, bcrc, seq,
     e_valid, e_seq, e_flags, e_rows, e_crc) = MM_INFO_ACK.unpack_from(body)
    if st != 0:
        raise ProtocolError(f"0x0136: status {st_name(st)}")
    flash = None                     # копии во флеше: mask — целые, used — взятая (-1 нет)
    if len(body) >= MM_INFO_ACK.size + MM_INFO_FLASH.size:
        mask, used, s0, s1, i0, i1 = MM_INFO_FLASH.unpack_from(body, MM_INFO_ACK.size)
        flash = {"mask": mask, "used": used, "seq": (s0, s1), "id": (i0, i1)}
    return {"cam": rcam, "state": state, "flash": flash, "state_name": MM_ST_NAMES.get(state, state),
            "off_why": why, "band": MM_OFF_NAMES.get(why, why), "received": received,
            "id": tid, "crc": tcrc, "unit_shift": unit, "frame": frame, "r0": r0, "rows": rows,
            "vofs": vofs, "kofs": kofs, "band_crc": bcrc, "seq": seq,
            "fpga": {"present": bool(e_valid), "seq": e_seq, "band_valid": bool(e_flags & 1),
                     "crc_fail": bool(e_flags & 2), "load_err": bool(e_flags & 4),
                     "loading": bool(e_flags & 8), "rows": e_rows, "crc": e_crc}}


def _set_section(section):
    return SET_SECTIONS[section] if isinstance(section, str) else int(section)


def _set_rec(body, off=4):
    sec, flags, copy, _r, ver, length, seq = SET_REC.unpack_from(body, off)
    return {"section": sec, "flags": flags, "copy": "-" if copy == 0xFF else "AB"[copy & 1],
            "version": ver, "length": length, "seq": seq}


def set_flags_text(flags):
    return ", ".join(name for bit, name in SET_FLAG_NAMES if flags & bit) or "пусто"


def settings_save(section="cfg", host=HOST):
    """0x0160: записать раздел из ОЗУ во флеш (в WORK — DENIED).

    Раздел 0 — наборы настроек WORK и SERVICE, 1 — калибровка (72 Б 0x0130)."""
    op, body = control(OP_SET_SAVE, bytes([_set_section(section)]), host, timeout=30.0)
    expect_ok("0x0160", op, body, OP_SET_SAVE | 1)
    return _set_rec(body)


def settings_load(section="cfg", host=HOST):
    """0x0162: перечитать раздел из флеш, отбросив несохранённые правки."""
    op, body = control(OP_SET_LOAD, bytes([_set_section(section)]), host, timeout=30.0)
    expect_ok("0x0162", op, body, OP_SET_LOAD | 1)
    return _set_rec(body)


def settings_info(host=HOST):
    """0x0164: состояние обоих разделов."""
    op, body = control(OP_SET_INFO, b"", host)
    expect_ok("0x0164", op, body, OP_SET_INFO | 1)
    return [_set_rec(body, 4 + 12 * i) for i in range((len(body) - 4) // 12)]


def settings_defaults(host=HOST):
    """0x0166: наборы в ОЗУ -> умолчания прошивки. флеш не трогается."""
    op, body = control(OP_SET_RESET, struct.pack(">I", SET_RESET_GUARD), host, timeout=30.0)
    expect_ok("0x0166", op, body, OP_SET_RESET | 1)


def settings_export(section="cfg", from_store=False, host=HOST):
    """0x0168: образ раздела (заголовок 16 Б + данные) — текущий из ОЗУ или с чипа."""
    op, body = control(OP_SET_EXPORT, bytes([_set_section(section), 1 if from_store else 0]), host)
    expect_ok("0x0168", op, body, OP_SET_EXPORT | 1)
    n = struct.unpack_from(">H", body, 4)[0]
    return body[8:8 + n]


def settings_import(image, section="cfg", store=False, host=HOST):
    """0x016A: отдать образ плате. Проверяется всё; набор текущего режима
    применяется сразу, store=True — ещё и запись во флеш."""
    payload = bytes([_set_section(section), 1 if store else 0, 0, 0]) + bytes(image)
    op, body = control(OP_SET_IMPORT, payload, host, timeout=30.0)
    expect_ok("0x016A", op, body, OP_SET_IMPORT | 1)
    return _set_rec(body)


def settings_image_check(image, section="cfg"):
    """Проверить образ на ПК так же, как плата: метка, длина, CRC. Возвращает
    (version, seq, данные) или бросает ValueError."""
    sec = _set_section(section)
    if len(image) < SET_IMG_HDR.size:
        raise ValueError(f"образ {len(image)} Б короче заголовка")
    magic, ver, length, crc, seq = SET_IMG_HDR.unpack_from(image)
    if magic != SET_MAGIC[sec]:
        raise ValueError(f"метка 0x{magic:08X}, а у раздела {sec} — 0x{SET_MAGIC[sec]:08X}")
    if SET_IMG_HDR.size + length != len(image):
        raise ValueError(f"длина в заголовке {length}, в файле {len(image) - SET_IMG_HDR.size}")
    calc = zlib.crc32(image[:8] + image[12:16] + image[16:]) & 0xFFFFFFFF
    if calc != crc:
        raise ValueError(f"CRC 0x{crc:08X} против 0x{calc:08X}")
    return ver, seq, image[16:]


# ---- OTA / Сетевая загрузка A/B ----------------------------------------------

OTA_STATUS_RESP = struct.Struct(">BBBBIIIII")  # 24 байта: st, slot, boot_st, attempts, slot_a, slot_b, b_crc, b_size, build_id


def ota_status(host=HOST, timeout=5.0):
    """0x0178: Получить статус текущего слота, метаданных и билда OTA."""
    op, body = control(OP_OTA_STATUS, b"", host=host, timeout=timeout)
    expect_ok("0x0178 (OTA_STATUS)", op, body, OP_OTA_STATUS_ACK)
    if len(body) < 24:
        raise ProtocolError(f"OTA_STATUS ответ короче 24 Б ({len(body)} Б)")
    st, slot, boot_st, attempts, slot_a, slot_b, b_crc, b_size, build_id = struct.unpack_from(">BBBBIIIII", body)
    fallback = body[24] if len(body) >= 25 else 0
    led_mode = body[25] if len(body) >= 26 else 0
    ver_maj = body[26] if len(body) >= 27 else 0
    ver_min = body[27] if len(body) >= 28 else 0
    fw_ver = body[28:].decode("ascii", errors="ignore").rstrip("\x00") if len(body) > 28 else ""
    return {
        "status": st,
        "active_slot": slot,
        "slot_name": OTA_SLOT_NAMES.get(slot, f"Slot {slot}"),
        "boot_status": boot_st,
        "boot_status_name": OTA_BOOT_STATUS.get(boot_st, f"Unknown ({boot_st})"),
        "boot_attempts": attempts,
        "slot_a_addr": slot_a,
        "slot_b_addr": slot_b,
        "slot_b_crc": b_crc,
        "slot_b_size": b_size,
        "build_id": build_id,
        "fallback_flag": fallback,
        "fallback_reason": OTA_FALLBACK_REASONS.get(fallback, f"Flag {fallback}"),
        "led_pattern": led_mode,
        "led_pattern_name": OTA_LED_PATTERNS.get(led_mode, f"Mode {led_mode}"),
        "version_major": ver_maj,
        "version_minor": ver_min,
        "fw_version": fw_ver,
    }


def ota_version(host=HOST, timeout=5.0):
    """0x017C: Запрос расширенной информации о версии прошивки и слоте."""
    op, body = control(OP_OTA_VERSION, b"", host=host, timeout=timeout)
    expect_ok("0x017C (OTA_VERSION)", op, body, OP_OTA_VERSION_ACK)
    if len(body) < 11:
        raise ProtocolError(f"OTA_VERSION ответ короче 11 Б ({len(body)} Б)")
    st, slot, maj, min_v, patch, led_mode, build_id, vlen = struct.unpack_from(">BBBBBBIB", body[:11])
    banner = body[11:11+vlen].decode("ascii", errors="ignore")
    return {
        "status": st,
        "active_slot": slot,
        "slot_name": OTA_SLOT_NAMES.get(slot, f"Slot {slot}"),
        "version": f"v{maj}.{min_v}.{patch}",
        "version_major": maj,
        "version_minor": min_v,
        "version_patch": patch,
        "led_pattern": led_mode,
        "led_pattern_name": OTA_LED_PATTERNS.get(led_mode, f"Mode {led_mode}"),
        "build_id": build_id,
        "banner": banner,
    }


def ota_set_slot(slot: int, host=HOST, timeout=5.0):
    """0x017A: Переключить активный слот загрузки (0 = Slot A, 1 = Slot B)."""
    payload = struct.pack(">B", slot)
    op, body = control(OP_OTA_SET_SLOT, payload, host=host, timeout=timeout)
    expect_ok("0x017A (OTA_SET_SLOT)", op, body, OP_OTA_SET_SLOT_ACK)
    if len(body) < 2:
        raise ProtocolError(f"OTA_SET_SLOT ответ короче 2 Б ({len(body)} Б)")
    st, active = struct.unpack_from(">BB", body)
    return {
        "status": st,
        "active_slot": active,
        "slot_name": OTA_SLOT_NAMES.get(active, f"Slot {active}"),
    }


def ota_start(total_size, expected_crc, target=OTA_TARGET_FIRMWARE, host=HOST, timeout=10.0):
    """0x0170: Начать сессию OTA. target=0 (OTA_TARGET_FIRMWARE -> Slot B)."""
    payload = struct.pack(">B3xII", target, total_size, expected_crc)
    op, body = control(OP_OTA_START, payload, host=host, timeout=timeout)
    expect_ok("0x0170 (OTA_START)", op, body, OP_OTA_START_ACK)
    if len(body) < 8:
        raise ProtocolError(f"OTA_START ответ короче 8 Б ({len(body)} Б)")
    st, target_slot, chunk_sz, staging_addr = struct.unpack_from(">BBHI", body)
    return {
        "status": st,
        "target_slot": target_slot,
        "chunk_size": chunk_sz,
        "staging_addr": staging_addr,
    }


def ota_upload(data, host=HOST, chunk_size=1024, progress_cb=None, timeout=15.0):
    """Загрузить бинарный образ в DDR3 staging буфер платы частями по 0x0172."""
    total = len(data)
    crc = zlib.crc32(data) & 0xFFFFFFFF
    start_info = ota_start(total, crc, target=OTA_TARGET_FIRMWARE, host=host, timeout=timeout)
    chunk_sz = start_info.get("chunk_size", chunk_size) or chunk_size

    with connect(PORT_CONTROL, host, timeout) as s:
        offset = 0
        seq = 10
        while offset < total:
            chunk = data[offset:offset + chunk_sz]
            payload = struct.pack(">I", offset) + chunk
            seq += 1
            op, body = transact(s, OP_OTA_DATA, payload, seq=seq)
            expect_ok(f"0x0172 @{offset}", op, body, OP_OTA_DATA_ACK)
            if len(body) < 12:
                raise ProtocolError(f"OTA_DATA ответ короче 12 Б")
            st, _res, ack_len, recvd, running_crc = struct.unpack_from(">BBHII", body)
            offset += len(chunk)
            if progress_cb:
                progress_cb(offset, total)

    return total, crc


def ota_commit(expected_crc, host=HOST, timeout=45.0):
    """0x0174: Записать из DDR3 во флеш Slot B (0xF60000) и проверить read-back CRC."""
    payload = struct.pack(">I", expected_crc)
    op, body = control(OP_OTA_COMMIT, payload, host=host, timeout=timeout)
    expect_ok("0x0174 (OTA_COMMIT)", op, body, OP_OTA_COMMIT_ACK)
    if len(body) < 8:
        raise ProtocolError(f"OTA_COMMIT ответ короче 8 Б ({len(body)} Б)")
    st, slot, num_sec, verified_crc = struct.unpack_from(">BBHI", body)
    if verified_crc != expected_crc:
        raise ProtocolError(f"OTA_COMMIT CRC платы 0x{verified_crc:08X} != 0x{expected_crc:08X}")
    return {
        "status": st,
        "slot": slot,
        "sectors_erased": num_sec,
        "verified_crc": verified_crc,
    }


def ota_reboot(delay_ms=100, host=HOST, timeout=5.0):
    """0x0176: Перезагрузить SoC через SMU (sw reset)."""
    payload = struct.pack(">I", delay_ms)
    try:
        op, body = control(OP_OTA_REBOOT, payload, host=host, timeout=timeout)
        expect_ok("0x0176 (OTA_REBOOT)", op, body, OP_OTA_REBOOT_ACK)
        return True
    except (socket.error, ProtocolError):
        # Плата может сразу перезагрузиться и разорвать сокет
        return True


# Секция искателя линии в образе настроек версии 2 (work_mode.c, set_serialize):
# тег (0xF << 4) | (4 + камера), 14 Б — ось, порог, gap, флаги, окно от/до,
# min_sum, width_max, подложка, 0. Общая для режимов WORK и SERVICE.
FINDER_SECTION = struct.Struct(">BBBBHHHHBB")
FINDER_TAG = {0: 0xF4, 1: 0xF5}


def _finder_from_section(d):
    a, t, g, f, lo, hi, ms, wm, off, _z = FINDER_SECTION.unpack_from(d)
    return {"axis": a, "thresh": t, "gap": g, "flags": f, "win_lo": lo, "win_hi": hi,
            "min_sum": ms, "width_max": wm, "offset": off}


def laser_config_read(host=HOST, from_store=False):
    """Параметры искателя обеих камер, как они сейчас на плате: из образа раздела
    настроек в ОЗУ (0x0168, тени laser.c — то, что ушло в ящик ПЛИС), без записи.
    Возвращает {камера: {axis, thresh, gap, flags, win_lo, win_hi, min_sum,
    width_max, offset}}; камеры без секции в образе нет в словаре."""
    _ver, _seq, data = settings_image_check(settings_export("cfg", from_store, host), "cfg")
    out, pos = {}, 0
    while pos + 2 <= len(data):
        tag, n = data[pos], data[pos + 1]
        d = data[pos + 2:pos + 2 + n]
        pos += 2 + n
        for cam, t in FINDER_TAG.items():
            if tag == t and n == FINDER_SECTION.size:
                out[cam] = _finder_from_section(d)
    return out


def settings_image_describe(image):
    """Разбор данных раздела настроек в словарь {режим: {предмет: значения}}."""
    _ver, _seq, data = settings_image_check(image, "cfg")
    modes, out, pos = {1: "work", 2: "service", 15: "system"}, {}, 0
    while pos + 2 <= len(data):
        tag, n = data[pos], data[pos + 1]
        d = data[pos + 2:pos + 2 + n]
        pos += 2 + n
        m, item = modes.get(tag >> 4, f"mode{tag >> 4}"), tag & 0x0F
        dst = out.setdefault(m, {})
        if item in (1, 2) and n == 20:
            exp, fps, gain, rw, rh, ox, oy, tp = struct.unpack_from(">IIHHHHHB", d)
            dst[f"cam{item - 1}"] = {"exp_us": exp, "fps": fps / 100.0, "gain_db": gain / 10.0,
                                     "roi": [rw, rh, ox, oy], "testpat": tp}
        elif item == 3 and n == 48:
            w = struct.unpack(">12I", d)
            dst["sync"] = {"source": {0: "core", 1: "timer", 2: "ext"}.get((w[0] >> 2) & 3, "?"),
                           "ext_fall": bool(w[0] & 0x10),
                           "cam0": [w[1], w[2]], "cam1": [w[3], w[4]], "laser": [w[5], w[6]],
                           "laser_mode": w[7], "pre": [w[8], w[9]], "polarity": w[10],
                           "ext_filter": w[11], "_units": f"такты {LANE_TICK_NS} нс"}
        elif tag >> 4 == 15 and item in (4, 5) and n == FINDER_SECTION.size:
            dst[f"finder_cam{item - 4}"] = _finder_from_section(d)
        elif item == 4 and n == 12:
            a, t, g, f, lo, hi, ms, wm = struct.unpack(">BBBBHHHH", d)
            dst["finder"] = {"axis": a, "thresh": t, "gap": g, "flags": f,
                             "win": [lo, hi], "min_sum": ms, "width_max": wm}
        else:
            dst[f"tag_0x{tag:02X}"] = d.hex()
    return out


def fetch_slices(n=1, start=0, cam=0, host=HOST, seq=1, verbose=True):
    """0x0210 на 39152: до n 3D-срезов, каждый — своим сообщением 0x0211.

    Возвращает список срезов {slice_id, ts_us, encoder, flags, points}, где
    points — список (x_sub_px, z, i, q). Пока искатель линии не реализован,
    плата отвечает отказом UNSUPPORTED, и это не ошибка клиента."""
    slices = []
    with connect(PORT_DATA, host) as s:
        s.sendall(packet(OP_FETCH_3D, seq, struct.pack(">III", n, start, cam)))
        total = None
        while total is None or len(slices) < total:
            op, rseq, _snd, _tgt, body = read_message(s)
            if op != OP_SLICE or rseq != seq:
                raise ProtocolError(f"ждали 0x0211 seq {seq}, пришло 0x{op:04X} seq {rseq}")
            if len(body) < SLICE_HDR.size:
                # Прошивка без 0x0210 отвечает StatusAck на коде 0x0211: 4 Б.
                raise ProtocolError(f"0x0210 не поддержан платой: status "
                                    f"{st_name(body[0]) if body else '-'} ({len(body)} Б)")
            num, sid, pts, enc, ts, st, flags = SLICE_HDR.unpack_from(body)
            if num == 0 or pts == 0:
                raise ProtocolError(f"модуль отказал: status {st_name(st)}")
            total = num
            raw = body[SLICE_HDR.size:SLICE_HDR.size + pts * 6]
            if len(raw) != pts * 6:
                raise ProtocolError(f"срез {sid}: {len(raw)} Б тела на {pts} точек")
            points = [(x / 16.0, z, i, q) for x, z, i, q in
                      struct.iter_unpack(">HHBB", raw)]
            slices.append({"slice_id": sid, "ts_us": ts, "encoder": enc,
                           "flags": flags, "status": st, "points": points})
            if verbose:
                good = sum(1 for _x, _z, _i, q in points if not (q & 0x01))
                print(f"  срез {sid}: точек {pts}, с линией {good}, "
                      f"флаги 0x{flags:02X}, ts {ts} мкс")
    return slices


def trig_series(freq_hz, count, width_us, stop=False, invert=False, host=HOST):
    flags = (2 if stop else 0) | (1 if invert else 0)
    payload = struct.pack(">IHHI", int(round(freq_hz * 100)), count, flags, width_us)
    op, body = control(OP_TRIG_SERIES, payload, host)
    expect_ok("0x0302", op, body, OP_TRIG_SERIES | 1)


# ---- поток профилей (0x0212 .. 0x0215, этап 2, 01.10.2026) -------------------
#
# Рабочий режим: ПЛИС пишет в слот только блок профиля (кадр без строк), плата
# шлёт точки каждого нового блока пачками 0x0213, пока хост не пришлёт 0x0214.
# Формат — спецификация VMSP § 4.4 (обработчик — vmsp_service.c); для
# разработчика клиента — tools/client_dpg/README.md, «Профили лазера». Камеру, ROI, генератор
# и лазер поток не трогает: их ставит хост (profile_rig_setup ниже).
OP_PSTREAM, OP_PBATCH, OP_PSTOP, OP_PSTOP_ACK = 0x0212, 0x0213, 0x0214, 0x0215
PS_F_KEEP_FRAME, PS_F_CRC, PS_F_MM, PS_F_MM_ONLY = 0x01, 0x02, 0x04, 0x08   # флаги запроса
PS_F_LE = 0x10      # слова точек и мм можно в порядке DDR3 (LE) - плата шлёт их по ссылке, без копии
PS_A_NO_FRAME, PS_A_KEEP_FRAME, PS_A_MM, PS_A_MM_ONLY = 0x01, 0x02, 0x04, 0x08   # флаги старта (поле info)
PS_A_DENSE = 0x10   # блоки в плотном кольце (laser_block.DRY_*, 19.10.2026)
PS_A_LE = 0x20      # слова пришли LE (отправка по ссылке); read() переворачивает их в BE
# CRC профиля из ПЛИС (20.10.2026, GOAL): с PS_F_PCRC у профиля ещё 8 Б - CRC-32 слов
# точек и слов мм, как их посчитала ПЛИС (laser_prof.v, заголовок блока 19, 20); флаг
# профиля PROF_F_CRC - они есть. read() сверяет их с принятым и выбрасывает профиль с
# плохой CRC, считая его в crc_bad.
PS_F_PCRC, PS_A_PCRC = 0x20, 0x40
PROF_F_CRC = 0x04
# Флаги мм профиля (R-38): мм верны для текущих таблицы и ROI; таблица испорчена;
# полоса блока не та, что у текущего ROI (мм заменены на «нет мм»).
PS_MM_VALID, PS_MM_BAD, PS_MM_STALE = 0x01, 0x02, 0x04
PS_BATCH_MAX = 16
PBATCH_HDR = struct.Struct(">BBHIHHI")   # 16 Б: status, cam, count, lost, points, profile_bytes, info
PPROF_HDR = struct.Struct(">IBBHQ")      # 16 Б: frame_id, flags, мм-флаги, rows_used, ts_us
PPROF_MM = struct.Struct(">II")          # +16 с PS_F_MM: id таблицы, CRC полосы
PPROF_CRC = struct.Struct(">II")         # дальше с PS_A_PCRC: CRC слов точек, CRC слов мм
PSTOP_ACK = struct.Struct(">B3xIIII")    # 20 Б: status, отдано/пропущено камер 0 и 1


PSTOP_DIAG = struct.Struct(">14I")       # хвост 0x0215 с замером, с +20 (прошивка с 01.10.2026)
PSTOP_EVENT = struct.Struct(">5I")


def _pstream_diag(body):
    """Хвост 0x0215: где прошивка тратила время и что было с TCP за поток
    (firmware/src/app/vmsp_service.c, ps_send_summary)."""
    (ver, take_ms, n_take, send_ms, n_send, n_slow, max_send_ms, zero_wnd, persist,
     nrtx_ev, tcp_drop, link_drop, link_recv, n_ev) = PSTOP_DIAG.unpack_from(body, 20)
    events = []
    for i in range(min(n_ev, (len(body) - 76) // PSTOP_EVENT.size)):
        t_ms, dur, wnd0, wnd1, fl = PSTOP_EVENT.unpack_from(body, 76 + PSTOP_EVENT.size * i)
        events.append({"t_ms": t_ms, "dur_ms": dur, "wnd0": wnd0, "wnd1": wnd1,
                       "nrtx": fl & 0xFF, "persist": (fl >> 8) & 0xFF, "rto": (fl >> 16) & 0xFF})
    return {"version": ver, "take_ms": take_ms, "n_take": n_take, "send_ms": send_ms,
            "n_send": n_send, "n_slow": n_slow, "max_send_ms": max_send_ms,
            "zero_wnd": zero_wnd, "persist": persist, "nrtx_ev": nrtx_ev,
            "tcp_drop": tcp_drop, "link_drop": link_drop, "link_recv": link_recv,
            "slow_sends": events}


def profile_points(words):
    """Слова точек профиля (bytes, BE u32 {q, i, x_sub}) -> (x_sub, i, q).

    С numpy — массивы (x_sub uint16 в 1/16 пикселя), без него — списки."""
    try:
        import numpy as np
    except ImportError:
        w = struct.unpack(f">{len(words) // 4}I", words)
        return [v & 0xFFFF for v in w], [(v >> 16) & 0xFF for v in w], [v >> 24 for v in w]
    w = np.frombuffer(words, dtype=">u4").astype(np.uint32)
    return (w & 0xFFFF).astype(np.uint16), ((w >> 16) & 0xFF).astype(np.uint8), (w >> 24).astype(np.uint8)


def profile_mm(words, unit_shift=5):
    """мм-слова профиля (bytes, BE u32 {дальность[31:16], вдоль[15:0]}) -> (вдоль,
    дальность, причина): мм float (NaN — нет мм), причина 0 или 1 нет линии, 2 таблица
    выкл., 3 вне полосы, 4 плохой узел. Единицы — 2**-unit_shift мм (по таблице)."""
    import numpy as np
    w = np.frombuffer(words, dtype=">u4").astype(np.int64)
    a = w & 0xFFFF
    d = (w >> 16) & 0xFFFF
    none = a == 0x8000
    a = np.where(a >= 0x8000, a - 0x10000, a)
    d = np.where(d >= 0x8000, d - 0x10000, d)
    s = float(1 << unit_shift)
    return (np.where(none, np.nan, a / s), np.where(none, np.nan, d / s),
            np.where(none, d & 0x7, 0).astype(np.uint8))


class ProfileStream:
    """Поток профилей лазерной линии.

        with vmsp.ProfileStream(cams=(0, 1), batch=8) as ps:
            print(ps.no_frame)              # ПЛИС подтвердила кадр без строк
            for msg in ps:                  # пачка; count 0 — пульс раз в 1 с
                for p in msg["profiles"]:
                    x_sub, i, q = vmsp.profile_points(p["words"])
                if done: break
        print(ps.summary)                   # итог 0x0215 после stop()

    Профиль: frame_id (номер кадра ПЛИС камеры, разрывы = потери), flags
    (0x01 блок ПЛИС, 0x02 кадр неполный), rows (строк прошло искатель), ts_us
    (строб экспозиции), words — points слов BE. msg["lost"] — сколько профилей
    камеры плата пропустила с начала потока (не успела забрать или переписаны).

    mm=True (R-38): у профиля ещё mm (bytes, points слов, profile_mm()), mm_flags
    (PS_MM_*), table_id и band_crc — какой таблицей и полосой посчитаны мм. Брать мм
    только при PS_MM_VALID и своём table_id. mm="only" — только мм, без слов точек
    (words пуст): байт столько же, сколько без мм (8 Б на точку вдвое режут темп).

    CRC ПЛИС (по умолчанию, PS_F_PCRC): профиль, у которого слова не сошлись с CRC,
    посчитанной ПЛИС, или без флага PROF_F_CRC (ПЛИС её не считала), в profiles не попадает,
    а считается в crc_bad[cam] (и msg["crc_bad"], с начала потока); crc_ok[cam] -
    сколько сошлось. мм сверяются только при PS_MM_VALID (иначе плата заменила их на
    «нет мм»)."""

    def __init__(self, cams=(0,), batch=8, limit=0, keep_frame=False, crc=False,
                 host=HOST, timeout=5.0, seq=1, mm=False, le=True, pcrc=True):
        self.mask = 0
        for c in cams:
            self.mask |= 1 << int(c)
        self.batch, self.limit, self.host, self.timeout, self.seq = batch, limit, host, timeout, seq
        self.flags = ((PS_F_KEEP_FRAME if keep_frame else 0) | (PS_F_CRC if crc else 0) |
                      (PS_F_MM if mm else 0) | (PS_F_MM_ONLY if mm == "only" else 0) |
                      (PS_F_LE if le else 0) | (PS_F_PCRC if pcrc else 0))
        self.sock = None
        self.ack_flags = 0
        self.summary = None
        self.crc_ok = {0: 0, 1: 0}
        self.crc_bad = {0: 0, 1: 0}

    @property
    def no_frame(self):
        return bool(self.ack_flags & PS_A_NO_FRAME)

    @property
    def dense(self):
        return bool(self.ack_flags & PS_A_DENSE)

    # Приёмный буфер сокета: окно TCP без масштабирования не больше 64 КБ, но
    # пока в буфере есть место, оно не закрывается, даже если клиент на время
    # отвлёкся (GUI, сборка мусора, запрос состояния). Закрытое окно плата
    # пережидает пробой окна lwIP — 1.5–2 с, а это обгон кольца (32 профиля
    # на камеру и больше). 01.10.2026: 2.2 с остановки на старте при 2 x 450 Гц.
    RCVBUF = 8 * 1024 * 1024

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.RCVBUF)
        except OSError:
            pass
        _bind_local(self.sock, self.host)
        self.sock.connect((self.host, PORT_DATA))
        self.sock.sendall(packet(OP_PSTREAM, self.seq,
                                 struct.pack(">BBHI", self.mask, self.flags, self.batch, self.limit)))
        # Старт: прошивка ждёт эха ПЛИС (до 1.5 с) до первого ответа.
        op, _seq, _snd, _tgt, body = read_message(self.sock)
        if op != OP_PBATCH or len(body) < PBATCH_HDR.size:
            # Прошивка без потока отвечает StatusAck на коде 0x0213: 4 Б.
            self.close()
            raise ProtocolError(f"0x0212 не поддержан платой: 0x{op:04X}, "
                                f"status {st_name(body[0]) if body else '-'} ({len(body)} Б)")
        st, _cam, _count, _lost, _pts, _pb, info = PBATCH_HDR.unpack_from(body)
        if st != 0:
            self.close()
            raise ProtocolError(f"0x0212: модуль отказал, status {st_name(st)}")
        self.ack_flags = info
        return self

    def read(self):
        """Одно сообщение потока: dict пачки (profiles может быть пуст — пульс)
        или None, если пришёл итог 0x0215 (он в self.summary)."""
        op, _seq, _snd, _tgt, body = read_message(self.sock)
        if op == OP_PSTOP_ACK:
            st, s0, l0, s1, l1 = PSTOP_ACK.unpack_from(body)
            self.summary = {"status": st, "sent": [s0, s1], "lost": [l0, l1]}
            if len(body) >= 76:
                self.summary["diag"] = _pstream_diag(body)
            return None
        if op != OP_PBATCH or len(body) < PBATCH_HDR.size:
            raise ProtocolError(f"поток: 0x{op:04X}, {len(body)} Б")
        st, cam, count, lost, points, pbytes, info = PBATCH_HDR.unpack_from(body)
        if st != 0:
            raise ProtocolError(f"поток: status {st_name(st)}")
        mm = bool(self.flags & PS_F_MM)
        only = mm and bool(self.flags & PS_F_MM_ONLY)
        pcrc = bool(self.ack_flags & PS_A_PCRC)
        hcrc = PPROF_HDR.size + (PPROF_MM.size if mm else 0)   # where the CRC pair is
        hsz = hcrc + (PPROF_CRC.size if pcrc else 0)
        npx = 0 if only else 4 * points                      # bytes of point words
        want = hsz + npx + (4 * points if mm else 0)
        if len(body) != PBATCH_HDR.size + count * pbytes or (count and pbytes != want):
            raise ProtocolError(f"пачка: {len(body)} Б на {count} профилей по {pbytes} Б")
        le = bool(self.ack_flags & PS_A_LE)
        profiles = []
        off = PBATCH_HDR.size
        for _ in range(count):
            fid, flags, mmf, rows, ts = PPROF_HDR.unpack_from(body, off)
            raw_px = body[off + hsz:off + hsz + npx]
            raw_mm = body[off + hsz + npx:off + pbytes]
            p = {"cam": cam, "frame_id": fid, "flags": flags, "rows": rows, "ts_us": ts,
                 "words": _be_words(raw_px, le)}
            if mm:
                tid, bcrc = PPROF_MM.unpack_from(body, off + PPROF_HDR.size)
                p.update(mm_flags=mmf, table_id=tid, band_crc=bcrc, mm=_be_words(raw_mm, le))
            off += pbytes
            if pcrc:
                if not flags & PROF_F_CRC:      # the FPGA did not compute it: unchecked words are not passed on
                    self.crc_bad[cam] += 1
                    continue
                c_px, c_mm = PPROF_CRC.unpack_from(body, off - pbytes + hcrc)
                ok = (not npx or _words_crc(raw_px, le) == c_px) and \
                     (not (mm and mmf & PS_MM_VALID) or _words_crc(raw_mm, le) == c_mm)
                if not ok:
                    self.crc_bad[cam] += 1
                    continue
                self.crc_ok[cam] += 1
            profiles.append(p)
        return {"cam": cam, "lost": lost, "points": points, "info": info, "profiles": profiles,
                "crc_bad": self.crc_bad[cam]}

    def __iter__(self):
        while True:
            msg = self.read()
            if msg is None:
                return
            yield msg

    def stop(self):
        """0x0214 и дочитать до итога 0x0215. Возвращает пачки, пришедшие после
        стопа (они не пропадают), итог — в self.summary."""
        tail = []
        if self.sock is None:
            return tail
        if self.summary is None:
            self.sock.sendall(packet(OP_PSTOP, self.seq + 1))
            while True:
                msg = self.read()
                if msg is None:
                    break
                tail.append(msg)
        self.close()
        return tail

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        try:
            self.stop()
        except (OSError, ProtocolError):
            self.close()
        return False


def _words_crc(b, le):
    """CRC-32 слов так, как её считает ПЛИС: по словам как байтам LE."""
    return zlib.crc32(b if le else _be_words(b, True)) & 0xFFFFFFFF


def _be_words(b, le):
    """Слова потока в BE, как их ждут profile_points()/profile_mm(): при PS_A_LE
    плата шлёт их как лежат в DDR3 (LE, отправка по ссылке)."""
    if not le:
        return b
    a = array.array("I", b)
    if sys.byteorder == "little":
        a.byteswap()
    return a.tobytes()


def profile_rig_setup(rows=64, freq=900.0, exp_us=200, gain_db=6.0, cams=(0,),
                      cam1_delay_us=0.0, laser=True, off_y=0, source="timer", host=HOST):
    """Стенд рабочего режима: камеры из cams — аппаратный триггер и ROI высотой
    rows со смещением off_y (число на обе или {камера: смещение}, rig_off_y),
    остальные — ROI 1280 без триггера; каналы камер и
    лазера под частоту freq; источник source ("timer" — генератор, "ext" —
    внешний вход, freq тогда — ожидаемая частота импульсов, по ней ширины).
    Генератор НЕ запускается — это profile_rig_run() (только для "timer").
    Возвращает то, что нужно profile_rig_restore() (канал лазера как был).

    Импульс камеры — четверть периода (не больше 100 мкс), лазер — 1.25
    экспозиции (не больше 0.8 периода). То же делал tools/laser/profile_rate.py
    (913 профилей/с при 64 строках, 900 Гц, 200 мкс; ERRORS 1.14)."""
    period_us = 1e6 / freq
    cam_w_ns = int(min(100_000, period_us * 1000 / 4))
    laser_w_ns = int(min(exp_us * 1250, period_us * 1000 * 0.8))
    ensure_active(host)          # из STANDBY поток не откроется (DENIED); WORK не трогается
    st = status(host)
    saved = {"laser": (st["laser_delay"] * LANE_TICK_NS, st["laser_width"] * LANE_TICK_NS,
                       st["laser_mode"]),
             "cam_w_ns": cam_w_ns, "freq": freq, "source": source,
             "arm": (rows, off_y, exp_us, gain_db, tuple(cams)),
             "trg": {c: get_camera_config(c, host)["trg_mode"] for c in (0, 1)}}
    try:
        _rig_arm(rows, off_y, exp_us, gain_db, cams, host)
        sync_channel(0, 0, cam_w_ns, host=host)
        sync_channel(1, int(cam1_delay_us * 1000), cam_w_ns, host=host)
        sync_channel(2, 0, laser_w_ns, 0, host=host)
        sync_control(1, source, 1 if laser else 0, host=host)
    except Exception:
        profile_rig_restore(saved, host)        # наполовину настроенный стенд не оставлять
        raise
    return saved


def profile_rig_run(saved, host=HOST, geometry=True):
    """Запустить генератор TIMER на частоте стенда (непрерывно) и добиться,
    чтобы ПЛИС применила высоту ROI (profile_rig_wait_geometry с перевзводом
    камер, до ~5 с). Звать до открытия потока: телеметрия читается через порт
    данных. При внешнем источнике генератора нет — высота применится от внешних
    импульсов, здесь только ожидание до 3 с.
    Возвращает (применилась ли, секунд ждали); geometry=False — (None, 0.0)."""
    rows, off_y, exp_us, gain_db, cams = saved["arm"]
    if saved.get("source", "timer") == "timer":
        trig_series(saved["freq"], 0, max(1, saved["cam_w_ns"] // 1000), host=host)
        rearm = lambda: _rig_arm(rows, off_y, exp_us, gain_db, cams, host)  # noqa: E731
    else:
        rearm = None
    if not geometry:
        return None, 0.0
    return profile_rig_wait_geometry(rows, cams, host=host, rearm=rearm)


def profile_rig_stop(host=HOST):
    trig_series(1, 0, 1, stop=True, host=host)


def fpga_frame_lines(cam=0, host=HOST):
    """Высота кадра, которую писатель ПЛИС применил (слово 35 телеметрии:
    [31] геометрия ждёт применения, [23:12] cfg_frame_lines). Через 0x0306 —
    порт данных, так что не во время потока профилей."""
    w = struct.unpack("<64I", mem_dump(DIAG_ADDRS[cam], 256, host=host))
    return (w[35] >> 12) & 0xFFF, bool(w[35] >> 31)


def profile_rig_wait_geometry(rows, cams=(0,), timeout=3.0, host=HOST, rearm=None,
                              attempt_s=1.0, attempts=5):
    """Дождаться, пока ПЛИС применит высоту ROI rows у камер cams.

    С битстрима 4D439860 (19.10.2026, ERRORS 2.15) писатель берёт высоту при
    открытии следующего слота — первым же кадром, 0.3–0.7 с по телеметрии;
    перевзвод остался запасным путём для старых битстримов. Раньше:
    после смены ROI высота доходила до писателя только при сбросе сторожа D-PHY
    (cam_channel.v, GEOM; cam_rx.v): а сбрасывает он, лишь когда строб пришёл,
    а строк после прошлой экспозиции не было. При смене одной высоты строки
    остаются целыми — сброса нет, и писатель ждёт кадр прежней высоты,
    закрывая кадр раз в 1280/rows импульсов (01.10.2026: скан из GUI — 30
    профилей за 2.7 с при 300 Гц вместо 600; кадр 1080 x 128 «целым» после
    отката). Сброс бывает, если камера после перевзвода потеряет первую
    экспозицию (ERRORS 31.37) — это случай: на плате 4 смены высоты из 8
    применились с первого перевзвода, остальные — со 2-го и 3-го, все за 3.3 с.

    rearm — функция перевзвода камер (импульсы при этом должны идти): тогда
    каждые attempt_s секунд без результата камеры перевзводятся, не больше
    attempts раз. Без rearm — просто ждать timeout. Признак «ждёт применения»
    не смотрится: запрос стенда — последний, и применится именно он.
    Телеметрия пишется раз в 0.5 с. Возвращает (применилась ли, секунд ждали)."""
    t0 = time.monotonic()
    attempt = 1
    limit = attempt_s * attempts if rearm is not None else timeout
    while True:
        ok = all(fpga_frame_lines(c, host)[0] == rows for c in cams)
        waited = time.monotonic() - t0
        if ok or waited >= limit:
            return ok, waited
        if rearm is not None and waited >= attempt * attempt_s and attempt < attempts:
            rearm()
            attempt += 1
        time.sleep(0.2)


def ensure_full_frame(host=HOST):
    """Перед съёмкой полного кадра: если ПЛИС осталась на малой высоте ROI
    (ERRORS 2.11), вернуть 1280 через profile_rig_restore (канал лазера — как
    сейчас). Уже 1280 у обеих камер — ничего не делает.
    Возвращает (высота 1280 у обеих, секунд ждали).

    «Уже 1280» — только если новая высота не ждёт применения (слово 35, [31]):
    02.10.2026 вход в WORK с ROI набора 848 оставил её ждать, проверка видела
    1280, кадр «Найти линию» сняли посреди смены высоты — блока профиля к нему
    не было (0x0210 UNSUPPORTED)."""
    if all(fpga_frame_lines(c, host) == (1280, False) for c in (0, 1)):
        return True, 0.0
    st = status(host)
    saved = {"laser": (st["laser_delay"] * LANE_TICK_NS, st["laser_width"] * LANE_TICK_NS,
                       st["laser_mode"])}
    return profile_rig_restore(saved, host)


def rig_off_y(off_y, cam):
    """Смещение ROI камеры: off_y — одно число на обе камеры или {камера: смещение}
    (с 02.10.2026 у каждой камеры своё; высота — общая, её одну на обе держит ПЛИС:
    камера 1 берёт высоту у камеры 0, scanner_top.v geom_lines_in)."""
    return int(off_y.get(cam, 0)) if isinstance(off_y, dict) else int(off_y)


def _rig_arm(rows, off_y, exp_us, gain_db, cams, host):
    """Камеры из cams — аппаратный триггер с ROI rows + смещение камеры
    (rig_off_y), остальные — ROI 1280."""
    for c in (0, 1):
        on = c in cams
        camera_config(exposure_us=exp_us, gain_db=gain_db, fps=10.0,
                      trigger_mode=1 if on else 0, cam=c,
                      roi_w=1080, roi_h=rows if on else 1280, off_x=0,
                      off_y=rig_off_y(off_y, c) if on else 0, host=host)


def profile_rig_restore(saved, host=HOST):
    """Вернуть съёмку кадров: генератор стоп, ROI 1280, 10000 мкс / 6 дБ,
    каналы камер 1 мс без задержки, лазер как был, источник CORE.

    И добиться, чтобы ПЛИС применила высоту 1280 (profile_rig_wait_geometry):
    иначе съёмка кадров отдаёт «целый» кадр 1080 x rows без единой плохой
    строки. Серия 10 Гц без лазера, камеры перевзводятся, пока высота не
    применится. Возвращает (применилась ли, секунд ждали)."""
    profile_rig_stop(host)
    arm = lambda: _rig_arm(1280, 0, 10000, 6.0, (0, 1), host)  # noqa: E731
    arm()
    sync_channel(0, 0, 1_000_000, host=host)
    sync_channel(1, 0, 1_000_000, host=host)
    sync_control(1, "timer", 0, host=host)
    trig_series(10, 0, 100, host=host)
    try:
        geom = profile_rig_wait_geometry(1280, (0, 1), host=host, rearm=arm)
    finally:
        profile_rig_stop(host)
        las = saved["laser"]
        sync_channel(2, las[0], max(las[1], LANE_TICK_NS), las[2], host=host)
        sync_control(1, "core", 0, host=host)
        # The trigger mode each camera had before the rig: armed cameras left behind
        # gave a GigE Vision client no frames at all (ERRORS 5.16)
        for c, trg in saved.get("trg", {}).items():
            if trg == 0:
                camera_config(10000, 6.0, 10.0, trigger_mode=0, cam=c, host=host)
    return geom


# 0x0222: вместо номера камеры — стереосъёмка, обе камеры от каждого импульса
# (прошивка с 30.09.2026, video_capture_stereo). Ответ — число целых ПАР; кадры
# забираются двумя fetch_frames(cam=0/1), пара узнаётся по ts_us: у камеры 1
# строб позже на задержку канала cam1 (её задаёт оператор, по умолчанию 0 — пара
# одновременная; до прошивки ECA2D2FA, 09.10.2026, прошивка держала не меньше 12 мс).
CAM_STEREO = 0xFF


def acquire(host=HOST, n=1, cam=0, laser=0):
    """Съёмка n кадров по триггеру в память; возвращает число целых кадров в кольце.

    Тело 4 Б: uint16 N, номер камеры (0 — разъём J15, 1 — J16, CAM_STEREO — обе
    от одного импульса), флаг laser_pulse_en (1/0). У каждой камеры своё кольцо в
    DDR3, поэтому съёмка одной не трогает кадры другой. Со старой прошивкой
    CAM_STEREO даёт UNSUPPORTED."""
    with connect(PORT_DATA, host, timeout=60.0) as s:
        op, body = transact(s, OP_ACQUIRE, struct.pack(">HBB", n, cam, 1 if laser else 0))
    expect_ok("0x0222", op, body, OP_ACQUIRE | 1)
    return struct.unpack_from(">I", body, 4)[0]


def mem_dump(addr, size, host=HOST):
    """Дамп памяти через 0x0306; CRC сходится, только если память не менялась."""
    with connect(PORT_DATA, host, timeout=120.0) as s:
        s.sendall(packet(OP_MEM_DUMP, 1, struct.pack(">II", addr, size)))
        op, _seq, _snd, _tgt, body = read_message(s)
    if op != OP_MEM_DUMP | 1 or len(body) < 12 or body[0] != 0:
        raise ProtocolError(f"0x0306: 0x{op:04X} status {st_name(body[0]) if body else '-'}")
    return body[12:]


def fetch_frames(n=1, cam=0, laser=0, host=HOST, seq=1, verbose=True):
    """0x0220 на 39152; список кадров {frame_id, ts_us, w, h, pixels, bad_rows}."""
    frames = []
    with connect(PORT_DATA, host) as s:
        s.sendall(packet(OP_CAPTURE, seq, bytes([cam, 1 if laser else 0]) + struct.pack(">H", n)))
        t0 = time.time()
        cur = None
        while len(frames) < n:
            op, rseq, _snd, _tgt, body = read_message(s)
            if op != OP_IMAGE or rseq != seq:
                raise ProtocolError(f"ждали 0x0221 seq {seq}, пришло 0x{op:04X} seq {rseq}")
            fid, ts, w, h, _fmt, st, total, off, idx, chunks = META.unpack_from(body)
            if chunks == 0:
                if frames:
                    break
                raise ProtocolError(f"модуль отказал: status {st_name(st)}")
            if cur is None or cur["frame_id"] != fid:
                if cur is not None:
                    raise ProtocolError(f"кадр {cur['frame_id']} оборван на чанке {cur['next']}")
                cur = {"frame_id": fid, "cam": cam, "ts_us": ts, "w": w, "h": h, "total": total,
                       "chunks": chunks, "next": 0, "buf": bytearray(total), "bad_rows": 0}
            if idx != cur["next"]:
                raise ProtocolError(f"кадр {fid}: чанк {idx}, ждали {cur['next']}")
            data = body[META.size:]
            last = idx == chunks - 1
            if last:
                data, crc_bytes = data[:-4], data[-4:]
            if off + len(data) > total:
                raise ProtocolError(f"кадр {fid}: чанк {idx} выходит за кадр")
            cur["buf"][off:off + len(data)] = data
            if st != 0:
                cur["bad_rows"] += 1
            cur["next"] += 1
            if last:
                want = struct.unpack(">I", crc_bytes)[0]
                if want != 0:
                    got = zlib.crc32(cur["buf"]) & 0xFFFFFFFF
                    if want != got:
                        raise ProtocolError(f"кадр {fid}: CRC кадра 0x{want:08X} против 0x{got:08X}")
                cur["pixels"] = bytes(cur.pop("buf"))
                frames.append(cur)
                if verbose:
                    print(f"  кадр {fid}: {w}x{h}, CRC ok, строк с плохим трейлером "
                          f"{cur['bad_rows']}, ts {ts} мкс, {time.time() - t0:.1f} с")
                cur = None
    return frames


# ---- кадры по UDP прямо из ПЛИС (0x0224 / 0x0225, 13.10.2026) -----------------
#
# udp_tx в ПЛИС сам читает кадры из кольца и шлёт их пакетами UDP; прошивка
# только кладёт задание и отвечает по TCP итогом и таблицей кадров
# (docs/architecture/UDP_FRAME_STREAM_PLAN.md § 11 — раскладки; на плате —
# handle_udp_frames в vmsp_service.c). Сеть — прямой провод с «Большим кадром»
# (9 КБ) на карте ПК (§ 8 пп. 7, 8): пакет по 8 строк — 8722 Б кадра Ethernet.
OP_UDP_FRAMES, OP_UDP_FRAMES_ACK = 0x0224, 0x0225
UDP_F_RESEND = 0x01
UDP_PKT_HDR = struct.Struct(">4sBBBBHHIIHHHHHBB")      # 32 Б заголовка пакета
UDP_REPLY_HDR = struct.Struct(">BBHHBBIIHHI")           # 24 Б итога 0x0225
UDP_REPLY_FRAME = struct.Struct(">IBBHQ")               # 16 Б на кадр
UDP_RCVBUF = 64 << 20
UDP_MAX_FRAMES = 28                                     # FRAME_UDP_MAX_FRAMES прошивки


def udp_gap_for_rate(mbit, rows=8):
    """Пауза после пакета (такты 8 нс) для скорости mbit Мбит/с на проводе;
    0 или >= 1000 — скорость линии. Пакет на проводе: преамбула 8, Ethernet +
    IPv4 + UDP 42, заголовок и хвост приложения 40, строки, FCS 4, пауза 12 Б."""
    if not mbit or mbit >= 1000:
        return 0
    wire = 8 + 42 + 40 + 1080 * rows + 4 + 12
    return int(round(wire * (1000.0 / mbit - 1.0)))


def _udp_request(cam, laser, n, port, rows, gap, resend_ids=None):
    flags = UDP_F_RESEND if resend_ids else 0
    body = struct.pack(">BBHHBBI", cam, 1 if laser else 0, len(resend_ids) if resend_ids else n,
                       port, rows, flags, gap)
    if resend_ids:
        body += b"".join(struct.pack(">I", fid) for fid in resend_ids)
    return body


def _udp_job(host, usock, port, cam, laser, n, rows, gap, seq, resend_ids, timeout):
    """Одно задание: запрос по TCP, приём UDP до ответа и чуть после.
    Возвращает (итог, таблица кадров, сырые пакеты)."""
    pkts = []
    with connect(PORT_DATA, host, timeout=timeout) as s:
        s.sendall(packet(OP_UDP_FRAMES, seq, _udp_request(cam, laser, n, port, rows, gap, resend_ids)))
        reply = None
        t_end = time.time() + timeout
        while reply is None:
            r, _w, _x = select.select([usock, s], [], [], max(0.0, t_end - time.time()))
            if not r:
                raise ProtocolError(f"0x0224: нет ответа за {timeout:.0f} с")
            if usock in r:
                while True:            # всё, что уже в буфере сокета
                    try:
                        pkts.append(usock.recv(9100))
                    except BlockingIOError:
                        break
                    except ConnectionResetError:
                        continue           # ICMP на «привет», см. fetch_frames_udp
            if s in r:
                op, rseq, _snd, _tgt, reply = read_message(s)
                if op != OP_UDP_FRAMES_ACK or rseq != seq:
                    raise ProtocolError(f"ждали 0x0225 seq {seq}, пришло 0x{op:04X} seq {rseq}")
    # Ответ уходит, когда ПЛИС сообщила, что все пакеты на проводе; хвост мог
    # отстать в стеке ПК — добрать до тишины.
    t_quiet = time.time() + 0.05
    while time.time() < t_quiet:
        r, _w, _x = select.select([usock], [], [], 0.01)
        if r:
            while True:
                try:
                    pkts.append(usock.recv(9100))
                except BlockingIOError:
                    break
                except ConnectionResetError:
                    continue
            t_quiet = time.time() + 0.05
    if len(reply) < UDP_REPLY_HDR.size:
        # короткий StatusAck: отказ до задания (старая прошивка — BAD_OPCODE)
        raise ProtocolError(f"0x0224: status {st_name(reply[0]) if reply else '-'}")
    (st, rcam, nfr, job_id, rrows, _z, packets, rows_bad, frames_bad, axi_err,
     fpga_us) = UDP_REPLY_HDR.unpack_from(reply)
    table = []
    for i in range(nfr):
        off = UDP_REPLY_HDR.size + i * UDP_REPLY_FRAME.size
        if off + UDP_REPLY_FRAME.size > len(reply):
            break
        fid, slot, _z1, h, ts = UDP_REPLY_FRAME.unpack_from(reply, off)
        table.append({"frame_id": fid, "slot": slot, "h": h, "ts_us": ts})
    summary = {"status": st, "cam": rcam, "frames": nfr, "job_id": job_id, "rows": rrows,
               "packets": packets, "rows_bad": rows_bad, "frames_bad": frames_bad,
               "axi_err": axi_err, "fpga_us": fpga_us}
    return summary, table, pkts


def _udp_assemble(job_id, pkts, acc):
    """Разложить пакеты задания job_id по кадрам acc {frame_seq: сборка}.
    Возвращает (принято, с плохой CRC, чужих)."""
    ok = bad = alien = 0
    for p in pkts:
        if len(p) < UDP_PKT_HDR.size + 8:
            alien += 1
            continue
        (magic, ver, _cam, rows_in, flags, jid, _fi, fseq, _strobe, first, h, w, pk_i, n_pk,
         _slot, _z) = UDP_PKT_HDR.unpack_from(p)
        if magic != b"VUDP" or ver != 1 or jid != job_id:
            alien += 1
            continue
        if zlib.crc32(p[:-4]) & 0xFFFFFFFF != struct.unpack_from(">I", p, len(p) - 4)[0]:
            bad += 1
            continue
        if len(p) != UDP_PKT_HDR.size + rows_in * w + 8 or fseq not in acc:
            bad += 1
            continue
        a = acc[fseq]
        if a["got"][pk_i]:
            continue
        a["buf"][first * w:(first + rows_in) * w] = p[UDP_PKT_HDR.size:UDP_PKT_HDR.size + rows_in * w]
        row_ok = p[UDP_PKT_HDR.size + rows_in * w]
        a["bad_rows"] += sum(1 for k in range(rows_in) if not (row_ok >> k) & 1)
        a["no_vid0"] = bool(flags & 0x02)
        a["got"][pk_i] = True
        ok += 1
    return ok, bad, alien


def fetch_frames_udp(n=1, cam=0, laser=0, host=HOST, rows=8, gap=0, retries=3, seq=1,
                     timeout=30.0, verbose=True):
    """0x0224: n кадров пакетами UDP из ПЛИС; тот же вид, что у fetch_frames —
    список {frame_id, cam, ts_us, w, h, pixels, bad_rows} (+ slot, no_vid0).

    rows — строк в пакете (8 — jumbo, нужен «Большой кадр» на карте ПК; 1 —
    обычные пакеты ~1.1 КБ), gap — пауза после пакета в тактах 8 нс (0 —
    скорость линии; от скорости в Мбит/с — udp_gap_for_rate). Неполный кадр
    запрашивается снова целиком (решение владельца, § 8 п. 2) до retries раз."""
    if n < 1 or n > UDP_MAX_FRAMES:
        raise ValueError(f"n: 1..{UDP_MAX_FRAMES} кадров за задание")
    us = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        us.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, UDP_RCVBUF)
        bind_ip = _find_local_bind_ip(host) or ""
        us.bind((bind_ip, 0))
        port = us.getsockname()[1]
        # Брандмауэр Windows пропускает ответный поток на сокет, который сам
        # писал на этот адрес и порт (§ 3.5). У lwIP платы на 39153 никого нет,
        # и она отвечает ICMP «порт недоступен» — а Windows тогда роняет
        # следующий recv с WSAECONNRESET (SIO_UDP_CONNRESET модуль socket не
        # знает); циклы приёма такую ошибку пропускают.
        us.sendto(b"VUDP-hello", (host, UDP_BOARD_PORT))
        us.setblocking(False)

        t0 = time.time()
        summary, table, pkts = _udp_job(host, us, port, cam, laser, n, rows, gap, seq, None, timeout)
        if summary["status"] != 0:
            # TIMEOUT при непустой таблице — ПЛИС не отчиталась (битстрим без udp_tx?)
            why = " (кадры есть, ПЛИС не отчиталась — битстрим без udp_tx?)" \
                if summary["status"] == 0x0B and table else ""
            raise ProtocolError(f"0x0224: status {st_name(summary['status'])}{why}")
        acc = {}
        for e in table:
            n_pk = (e["h"] + rows - 1) // rows
            acc[e["frame_id"]] = {"frame_id": e["frame_id"], "cam": cam, "ts_us": e["ts_us"],
                                  "w": FRAME_WIDTH, "h": e["h"], "slot": e["slot"],
                                  "buf": bytearray(FRAME_WIDTH * e["h"]), "got": [False] * n_pk,
                                  "bad_rows": 0, "no_vid0": False}
        ok, bad, alien = _udp_assemble(summary["job_id"], pkts, acc)
        stats = {"jobs": 1, "packets": summary["packets"], "received": ok, "crc_bad": bad,
                 "alien": alien, "resent_frames": 0, "fpga_us": summary["fpga_us"]}
        if verbose:
            print(f"  0x0224: {len(table)} кадров, {summary['packets']} пакетов, принято {ok}, "
                  f"CRC {bad}, чужих {alien}, ПЛИС {summary['fpga_us'] / 1000:.1f} мс, "
                  f"{(time.time() - t0) * 1000:.0f} мс всего")
        for attempt in range(retries):
            missing = [fid for fid, a in acc.items() if not all(a["got"])]
            if not missing:
                break
            seq += 1
            summary, _t, pkts = _udp_job(host, us, port, cam, laser, len(missing), rows, gap, seq,
                                         missing, timeout)
            if summary["status"] != 0:
                raise ProtocolError(f"0x0224 повтор: status {st_name(summary['status'])}")
            ok, bad, alien = _udp_assemble(summary["job_id"], pkts, acc)
            stats["jobs"] += 1
            stats["resent_frames"] += len(missing)
            if verbose:
                print(f"  повтор {attempt + 1}: кадры {missing}, принято {ok}, CRC {bad}")
        frames = []
        for e in table:
            a = acc[e["frame_id"]]
            if not all(a["got"]):
                lost = a["got"].count(False)
                raise ProtocolError(f"кадр {e['frame_id']}: не хватает {lost} пакетов из "
                                    f"{len(a['got'])} после {retries} повторов")
            a["pixels"] = bytes(a.pop("buf"))
            a.pop("got")
            frames.append(a)
            if verbose:
                # Та же строка, что у fetch_frames: по ней считают кадры скрипты
                # приёмки (fpga/power_cycle_watch.ps1). CRC — у каждого пакета.
                print(f"  кадр {a['frame_id']}: {a['w']}x{a['h']}, CRC ok, строк с плохим трейлером "
                      f"{a['bad_rows']}, ts {a['ts_us']} мкс, {time.time() - t0:.1f} с")
        if verbose:
            print(f"  кадров {len(frames)}, повторено {stats['resent_frames']}, "
                  f"{(time.time() - t0) * 1000:.0f} мс")
        fetch_frames_udp.last_stats = stats
        return frames
    finally:
        us.close()


def udp_unsupported(err):
    """Отказ 0x0224 значит «плата кадры по UDP не умеет»: прошивка без 0x0224
    (BAD_OPCODE) или битстрим без udp_tx — например, откат на C025B57C (кадры есть,
    ПЛИС не отчиталась). Только тогда выдачу берёт TCP; прочие ошибки UDP —
    сбой выдачи, запасным путём их не прикрывать."""
    msg = str(err)
    return "BAD_OPCODE" in msg or "udp_tx" in msg


def fetch_frames_any(n=1, cam=0, laser=0, host=HOST, tcp=False, seq=1, verbose=True, **udp):
    """Кадры из кольца: по UDP из ПЛИС (0x0224), а по TCP (0x0220) — если tcp, если
    n > UDP_MAX_FRAMES (потолок задания) или если плата UDP не умеет
    (udp_unsupported). Вид кадров один; fetch_frames_any.last_transport — "udp"/"tcp".
    udp — rows, gap, retries, timeout для fetch_frames_udp. 0x0220 оставлен
    запасным путём (решение владельца 14.10.2026, UDP_FRAME_STREAM_PLAN.md § 8)."""
    if not tcp and n <= UDP_MAX_FRAMES:
        try:
            frames = fetch_frames_udp(n, cam, laser, host, seq=seq, verbose=verbose, **udp)
            fetch_frames_any.last_transport = "udp"
            return frames
        except ProtocolError as e:
            if not udp_unsupported(e):
                raise
            if verbose:
                print(f"  UDP недоступен ({e}) — кадры по TCP")
    elif verbose and not tcp:
        print(f"  {n} кадров больше потолка задания UDP ({UDP_MAX_FRAMES}) — кадры по TCP")
    fetch_frames_any.last_transport = "tcp"
    return fetch_frames(n, cam, laser, host, seq=seq, verbose=verbose)


# ---- поток кадров по UDP: живой просмотр (0x0226..0x0229, 14.10.2026) ---------
#
# Каждый новый закрытый кадр камер из маски ПЛИС шлёт сама (прошивка ставит
# udp_tx задание на кадр), пока хост не скажет стоп. Неполный кадр просто
# пропускается — дозапросов нет (живой просмотр, решение владельца). Кадры
# снимает то, что настроено до старта: frame_stream_rig() — генератор ПЛИС на
# частоте fps или синхроблок как есть. Раскладка пакета — та же, что у 0x0224
# (UDP_FRAME_STREAM_PLAN.md § 11); на плате — handle_frame_stream.
OP_FSTREAM, OP_FSTREAM_ST, OP_FSTOP, OP_FSTOP_ACK = 0x0226, 0x0227, 0x0228, 0x0229
FS_STATUS = struct.Struct(">BBHIIIIIII")        # 32 Б: status, маска, 0, мс, (отправлено, пропущено, ошибок) x 2


def _fs_status(body):
    st, mask, _z, ms, s0, k0, e0, s1, k1, e1 = FS_STATUS.unpack_from(body)
    return {"status": st, "mask": mask, "ms": ms,
            "cams": {0: {"sent": s0, "skipped": k0, "errors": e0}, 1: {"sent": s1, "skipped": k1, "errors": e1}}}


def frame_stream_rig(cams=(0,), fps=0.0, exp_us=10000, gain_db=6.0, laser=False, host=HOST):
    """Подготовить съёмку для потока кадров: камеры cams — аппаратный триггер,
    ROI 1280. fps > 0 — генератор ПЛИС на этой частоте (каналы камер и лазера
    под неё, генератор запущен; стенд потока профилей с высотой 1280); fps = 0 —
    синхроблок не трогается (EXT или генератор, настроенный заранее). Камеры
    обеих в cams снимаются от одного импульса — стереопары. Экспозиция должна
    помещаться в период: сенсору нужно exp + ~8.8 мс на кадр 1280 строк.
    Возвращает то, что нужно frame_stream_rig_restore()."""
    if fps and fps > 0:
        saved = profile_rig_setup(rows=1280, freq=fps, exp_us=exp_us, gain_db=gain_db, cams=tuple(cams),
                                  laser=laser, host=host)
        trig_series(fps, 0, max(1, saved["cam_w_ns"] // 1000), host=host)
        return saved
    ensure_active(host, verbose=False)
    _rig_arm(1280, 0, exp_us, gain_db, tuple(cams), host)
    return None


def frame_stream_rig_restore(saved, host=HOST):
    """После потока: генератор стоп, канал лазера как был, источник CORE (как
    после съёмок). Синхроблок «как есть» (saved = None) не трогается."""
    if saved is None:
        return
    profile_rig_stop(host)
    las = saved["laser"]
    sync_channel(2, las[0], max(las[1], LANE_TICK_NS), las[2], host=host)
    sync_control(1, "core", 0, host=host)


class FrameStream:
    """Поток кадров по UDP (0x0226). Приём — в своём потоке: кадры собираются
    по (камера, номер кадра), целый — в очередь и в «последний по камере»;
    неполный выбрасывается, как только у камеры закрылся кадр новее, или через
    stale_s. Кадр — тот же dict, что у fetch_frames_udp (pixels, frame_id, cam,
    w, h, bad_rows, ts_us — строб в мкс по модулю 2^32 тактов, slot, no_vid0).

        with FrameStream(cams=(0, 1)) as fs:
            f = fs.get(timeout=1.0)          # следующий целый кадр (FIFO) или None
            f0 = fs.latest(0)                # самый свежий целый кадр камеры 0
    """

    def __init__(self, cams=(0,), rows=8, gap=0, host=HOST, queue_len=8, stale_s=0.5, timeout=10.0):
        self.cams, self.rows, self.gap, self.host = tuple(cams), rows, gap, host
        self.queue_len, self.stale_s, self.timeout = queue_len, stale_s, timeout
        self._us = self._tcp = None
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._queue = []
        self._latest = {}
        self._acc = {}
        self._stop_ev = threading.Event()
        self._threads = []
        self.stats = {c: {"frames": 0, "incomplete": 0, "crc_bad": 0, "bad_rows": 0} for c in (0, 1)}
        self.board = None           # последний 0x0227 платы
        self.summary = None         # 0x0229 после stop()
        self.t_start = None

    # ---- старт / стоп ---------------------------------------------------------
    def start(self):
        mask = sum(1 << c for c in self.cams)
        us = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        us.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, UDP_RCVBUF)
        us.bind((_find_local_bind_ip(self.host) or "", 0))
        us.sendto(b"VUDP-hello", (self.host, UDP_BOARD_PORT))      # брандмауэр, см. fetch_frames_udp
        us.settimeout(0.2)
        self._us = us
        try:
            s = connect(PORT_DATA, self.host, timeout=self.timeout)
            s.sendall(packet(OP_FSTREAM, 1, struct.pack(">BBHI", mask, self.rows, us.getsockname()[1], self.gap)))
            op, _seq, _a, _b, body = read_message(s)
        except Exception:
            us.close()
            raise
        if op != OP_FSTREAM_ST or not body or body[0] != 0:
            s.close()
            us.close()
            raise ProtocolError(f"0x0226: 0x{op:04X} status {st_name(body[0]) if body else '-'}")
        self._tcp = s
        self.board = _fs_status(body) if len(body) >= FS_STATUS.size else None
        self.t_start = time.monotonic()
        for fn, name in ((self._rx, "udp-stream-rx"), (self._ctl, "udp-stream-ctl")):
            t = threading.Thread(target=fn, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self, timeout=3.0):
        """0x0228; ждёт итог 0x0229 (self.summary) до timeout, закрывает сокеты."""
        if self._tcp is None:
            return self.summary
        try:
            self._tcp.sendall(packet(OP_FSTOP, 2))
            t_end = time.monotonic() + timeout
            while self.summary is None and time.monotonic() < t_end and self._threads[1].is_alive():
                time.sleep(0.02)
        except OSError:
            pass
        self._stop_ev.set()
        for t in self._threads:
            t.join(1.0)
        for sk in (self._tcp, self._us):
            try:
                sk.close()
            except OSError:
                pass
        self._tcp = self._us = None
        return self.summary

    def close(self):
        self.stop()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
        return False

    # ---- кадры ------------------------------------------------------------------
    def get(self, timeout=1.0):
        """Следующий целый кадр (по порядку прихода) или None за timeout."""
        with self._cv:
            if not self._queue:
                self._cv.wait(timeout)
            return self._queue.pop(0) if self._queue else None

    def latest(self, cam):
        with self._lock:
            return self._latest.get(cam)

    def fps(self, cam):
        dt = (time.monotonic() - self.t_start) if self.t_start else 0.0
        return self.stats[cam]["frames"] / dt if dt > 0 else 0.0

    # ---- потоки приёма ------------------------------------------------------------
    def _ctl(self):
        while not self._stop_ev.is_set():
            try:
                self._tcp.settimeout(0.5)
                op, _seq, _a, _b, body = read_message(self._tcp)
            except socket.timeout:
                continue
            except (OSError, ProtocolError):
                return
            if op == OP_FSTREAM_ST and len(body) >= FS_STATUS.size:
                self.board = _fs_status(body)
            elif op == OP_FSTOP_ACK and len(body) >= FS_STATUS.size:
                self.summary = _fs_status(body)
                return

    def _rx(self):
        hs = UDP_PKT_HDR.size
        t_exp = time.monotonic()
        while not self._stop_ev.is_set():
            try:
                p = self._us.recv(9100)
            except socket.timeout:
                self._expire(time.monotonic())
                continue
            except ConnectionResetError:
                continue                        # ICMP на «привет»
            except OSError:
                return
            if len(p) < hs + 8 or p[:4] != b"VUDP":
                continue
            (_m, ver, cam, rows_in, flags, _jid, _fi, fseq, strobe, first, h, w, pk_i, n_pk,
             slot, _z) = UDP_PKT_HDR.unpack_from(p)
            if ver != 1 or cam not in self.stats or len(p) != hs + rows_in * w + 8:
                continue
            if zlib.crc32(p[:-4]) & 0xFFFFFFFF != struct.unpack_from(">I", p, len(p) - 4)[0]:
                self.stats[cam]["crc_bad"] += 1
                continue
            now = time.monotonic()
            key = (cam, fseq)
            a = self._acc.get(key)
            if a is None:
                a = self._acc[key] = {"buf": bytearray(w * h), "got": bytearray(n_pk), "left": n_pk,
                                      "bad_rows": 0, "t0": now, "frame_id": fseq, "cam": cam, "w": w, "h": h,
                                      "ts_us": strobe // 100, "strobe": strobe, "slot": slot,
                                      "no_vid0": bool(flags & 0x02)}
            if pk_i >= len(a["got"]) or a["got"][pk_i]:
                continue
            a["got"][pk_i] = 1
            a["left"] -= 1
            a["buf"][first * w:(first + rows_in) * w] = memoryview(p)[hs:hs + rows_in * w]
            a["bad_rows"] += rows_in - bin(p[hs + rows_in * w] & ((1 << rows_in) - 1)).count("1")
            if a["left"] == 0:
                self._deliver(key, a)
            if now - t_exp >= 0.05:            # не на каждый пакет: их до ~13 тыс./с
                t_exp = now
                self._expire(now)

    def _deliver(self, key, a):
        cam, fseq = key
        del self._acc[key]
        f = {k: a[k] for k in ("frame_id", "cam", "w", "h", "ts_us", "strobe", "slot", "no_vid0", "bad_rows")}
        f["pixels"] = a["buf"]          # bytearray, без копии 1.38 МБ: сборка кадра закончена
        st = self.stats[cam]
        st["frames"] += 1
        st["bad_rows"] += a["bad_rows"]
        # Новее закрылся — старые неполные этой камеры уже не придут целыми.
        for k in [k for k in self._acc if k[0] == cam and k[1] < fseq]:
            del self._acc[k]
            st["incomplete"] += 1
        with self._cv:
            self._latest[cam] = f
            self._queue.append(f)
            if len(self._queue) > self.queue_len:
                del self._queue[0]
            self._cv.notify_all()

    def _expire(self, now):
        for k in [k for k, a in self._acc.items() if now - a["t0"] > self.stale_s]:
            del self._acc[k]
            self.stats[k[0]]["incomplete"] += 1
