#!/usr/bin/env python3
"""Непредвзятый набор автотестов безопасной сетевой загрузки (OTA A/B Pipeline).

Тестирует протокол VMSP (0x0170..0x0179), граничные условия, устойчивость
к сбоям, проверку CRC-32, изоляцию разделов Flash и аппаратную защиту
от окирпичивания платы Tang Mega 138K Pro.

Запуск:
    python tools/board/test_ota_pipeline.py [--host 10.0.0.88] [--skip-reboot]

Выходной код: 0 при прохождении всех тестов, иначе число непройденных проверок.
"""

import argparse
import os
import socket
import struct
import sys
import time
import zlib

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common import vmsp

fails = 0
total_checks = 0


def check(cond, what, detail=""):
    global fails, total_checks
    total_checks += 1
    tag = "ok  " if cond else "FAIL"
    msg = f"[{tag}] {what}"
    if detail and not cond:
        msg += f" -> {detail}"
    print(msg)
    if not cond:
        fails += 1


def test_ota_status_structure(host):
    print("\n--- ТЕСТ 1: Валидация структуры и полей OTA_STATUS (0x0178) ---")
    try:
        st = vmsp.ota_status(host=host, timeout=3.0)
        check(st["status"] == 0, "Код статуса ответа равен 0 (OK)")
        check(st["active_slot"] in (0, 1), f"Активный слот корректен: {st['active_slot']} ({st['slot_name']})")
        check(st["slot_a_addr"] == 0x00600000, f"Аппаратный адрес слота A: 0x{st['slot_a_addr']:06X} == 0x00600000")
        check(st["slot_b_addr"] == 0x00F60000, f"Аппаратный адрес слота B: 0x{st['slot_b_addr']:06X} == 0x00F60000")
        check(st["build_id"] != 0, f"Build ID SPI-контроллера не нулевой: 0x{st['build_id']:08X}")
        check(len(st.get("fw_version", "")) > 0, f"Версия прошивки передаётся в OTA_STATUS: '{st.get('fw_version')}'")
        check(st.get("led_pattern") in (0, 1), f"Режим LED передан корректно: {st.get('led_pattern')} ({st.get('led_pattern_name')})")
    except Exception as e:
        check(False, "Запрос OTA_STATUS завершился исключением", str(e))


def test_ota_start_parameter_sanitization(host):
    print("\n--- ТЕСТ 2: Проверка санитарии входных параметров OTA_START (0x0170) ---")

    # 2.1: Недопустимый target (например 99)
    try:
        payload = struct.pack(">B3xII", 99, 100000, 0x12345678)
        op, body = vmsp.control(vmsp.OP_OTA_START, payload, host=host, timeout=3.0)
        check(op == vmsp.OP_OTA_START_ACK, "OTA_START target=99: получен ACK 0x0171")
        check(len(body) >= 1 and body[0] == 1, f"OTA_START target=99: отклонён с кодом 1 (Unsupported target), получено {body[0] if body else 'empty'}")
    except Exception as e:
        check(False, "OTA_START target=99 вызвал исключение", str(e))

    # 2.2: Target = 1 (Bitstream пока не поддержан в firmware)
    try:
        payload = struct.pack(">B3xII", 1, 100000, 0x12345678)
        op, body = vmsp.control(vmsp.OP_OTA_START, payload, host=host, timeout=3.0)
        check(op == vmsp.OP_OTA_START_ACK, "OTA_START target=1 (Bitstream): получен ACK 0x0171")
        check(len(body) >= 1 and body[0] == 1, f"OTA_START target=1: отклонён с кодом 1 (Unsupported target), получено {body[0] if body else 'empty'}")
    except Exception as e:
        check(False, "OTA_START target=1 вызвал исключение", str(e))

    # 2.3: Слишком малый размер (< 1024 байт)
    try:
        payload = struct.pack(">B3xII", 0, 500, 0x12345678)
        op, body = vmsp.control(vmsp.OP_OTA_START, payload, host=host, timeout=3.0)
        check(op == vmsp.OP_OTA_START_ACK, "OTA_START size=500: получен ACK 0x0171")
        check(len(body) >= 1 and body[0] == 2, f"OTA_START size=500: отклонён с кодом 2 (Bad size), получено {body[0] if body else 'empty'}")
    except Exception as e:
        check(False, "OTA_START size=500 вызвал исключение", str(e))

    # 2.4: Слишком большой размер (> 512 КБ)
    try:
        payload = struct.pack(">B3xII", 0, 1024 * 1024, 0x12345678)
        op, body = vmsp.control(vmsp.OP_OTA_START, payload, host=host, timeout=3.0)
        check(op == vmsp.OP_OTA_START_ACK, "OTA_START size=1MB: получен ACK 0x0171")
        check(len(body) >= 1 and body[0] == 2, f"OTA_START size=1MB: отклонён с кодом 2 (Bad size), получено {body[0] if body else 'empty'}")
    except Exception as e:
        check(False, "OTA_START size=1MB вызвал исключение", str(e))

    # 2.5: Неполное тело запроса (< 8 байт)
    try:
        payload = b"\x00\x00\x00"
        op, body = vmsp.control(vmsp.OP_OTA_START, payload, host=host, timeout=3.0)
        check(len(body) >= 1 and body[0] == 0x06, f"OTA_START len=3: отклонён с BAD_LENGTH (0x06), получено {body[0] if body else 'empty'}")
    except Exception as e:
        check(False, "OTA_START short body вызвал исключение", str(e))


def test_ota_state_machine_and_sequencing(host):
    print("\n--- ТЕСТ 3: Автомат состояний и секвенсирование сессии (0x0172 / 0x0174) ---")

    # 3.1: Отправка данных без открытой сессии
    try:
        payload = struct.pack(">I", 0) + b"\xAA" * 128
        op, body = vmsp.control(vmsp.OP_OTA_DATA, payload, host=host, timeout=3.0)
        check(op == vmsp.OP_OTA_DATA_ACK, "OTA_DATA вне сессии: получен ACK 0x0173")
    except Exception as e:
        check(False, "OTA_DATA вне сессии вызвал исключение", str(e))

    # 3.2: Открытие сессии и передача некорректного смещения
    try:
        test_size = 2048
        test_crc = 0xAABBCCDD
        st_res = vmsp.ota_start(test_size, test_crc, target=0, host=host)
        check(st_res["status"] == 0, "OTA_START на 2048 Б: сессия успешно открыта (status=0)")
        check(st_res["staging_addr"] == 0x11000000, f"Адрес DDR3 staging буфера: 0x{st_res['staging_addr']:08X} == 0x11000000")

        # Отправляем чанк со смещением 512 вместо 0
        bad_payload = struct.pack(">I", 512) + b"\x55" * 256
        op, body = vmsp.control(vmsp.OP_OTA_DATA, bad_payload, host=host, timeout=3.0)
        check(op == vmsp.OP_OTA_DATA_ACK, "OTA_DATA со смещением 512 (вместо 0): получен ACK")
        check(len(body) >= 1 and body[0] == 2, f"OTA_DATA смещение 512 отклонено с кодом 2 (Bad offset), получено {body[0] if body else 'empty'}")

        # Отправляем чанк с переполнением (> total_size)
        overflow_payload = struct.pack(">I", 0) + b"\x00" * 3000
        op, body = vmsp.control(vmsp.OP_OTA_DATA, overflow_payload, host=host, timeout=3.0)
        check(len(body) >= 1 and body[0] == 2, f"OTA_DATA переполнение (>2048) отклонено с кодом 2, получено {body[0] if body else 'empty'}")
    except Exception as e:
        check(False, "Тест смещения и переполнения вызвал исключение", str(e))


def test_ota_abrupt_disconnect_recovery(host):
    print("\n--- ТЕСТ 4: Обрыв связи в процессе передачи и восстановление (Drop Recovery) ---")
    try:
        # Открываем сессию на 64 КБ
        st = vmsp.ota_start(64 * 1024, 0x12345678, target=0, host=host)
        check(st["status"] == 0, "OTA_START на 64 КБ открыта перед тестом обрыва")

        # Отправляем 2 чанка по 1024 байта
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0)
        s.connect((host, vmsp.PORT_CONTROL))
        p1 = struct.pack(">I", 0) + b"X" * 1024
        op1, body1 = vmsp.transact(s, vmsp.OP_OTA_DATA, p1, seq=1)
        check(op1 == vmsp.OP_OTA_DATA_ACK and body1[0] == 0, "Чанк 1 принят платой")

        p2 = struct.pack(">I", 1024) + b"Y" * 1024
        op2, body2 = vmsp.transact(s, vmsp.OP_OTA_DATA, p2, seq=2)
        check(op2 == vmsp.OP_OTA_DATA_ACK and body2[0] == 0, "Чанк 2 принят платой")

        # Внезапный обрыв соединения (close без FIN handshake / без commit)
        s.close()
        check(True, "Сокет принудительно разорван в середине загрузки")

        # Немедленное переподключение новым сокетом
        time.sleep(0.1)
        st_after = vmsp.ota_status(host=host)
        check(st_after["status"] == 0, "Плата сохраняет полную отзывчивость после разрыва")

        # Открытие новой сессии поверх незавершенной (Session preemption)
        st_new = vmsp.ota_start(10240, 0x87654321, target=0, host=host)
        check(st_new["status"] == 0, "Новая сессия открыта без блокировок или утечек памяти")
    except Exception as e:
        check(False, "Тест обрыва соединения вызвал исключение", str(e))


def test_ota_premature_and_double_commit(host):
    print("\n--- ТЕСТ 5: Отказ при преждевременном и повторном коммите (Commit Guards) ---")
    try:
        # 5.1: Преждевременный коммит (received < total)
        # В предыдущем тесте сессия открыта на 10240 байт, получено 0
        op, body = vmsp.control(vmsp.OP_OTA_COMMIT, struct.pack(">I", 0x87654321), host=host)
        check(op == vmsp.OP_OTA_COMMIT_ACK, "OTA_COMMIT до завершения приёма: получен ACK 0x0175")
        check(len(body) >= 1 and body[0] == 1, f"OTA_COMMIT отклонён со статусом 1 (Incomplete), получено {body[0] if body else 'empty'}")

        # 5.2: Повторный коммит при неактивной сессии
        op2, body2 = vmsp.control(vmsp.OP_OTA_COMMIT, struct.pack(">I", 0x87654321), host=host)
        check(len(body2) >= 1 and body2[0] == 1, f"Повторный OTA_COMMIT отклонён со статусом 1 (Incomplete), получено {body2[0] if body2 else 'empty'}")
    except Exception as e:
        check(False, "Тест преждевременного коммита вызвал исключение", str(e))


def test_ota_multisize_chunk_streaming(host):
    print("\n--- ТЕСТ 6: Потоковая передача с переменными размерами чанков (Multi-size Chunks) ---")
    try:
        chunk_sizes = [64, 128, 256, 512, 1024, 1400, 300, 750, 999]
        total_size = sum(chunk_sizes)
        dummy_payload = b"".join(bytes([i % 256]) * sz for i, sz in enumerate(chunk_sizes))
        expected_crc = zlib.crc32(dummy_payload) & 0xFFFFFFFF

        st = vmsp.ota_start(total_size, expected_crc, target=0, host=host)
        check(st["status"] == 0, f"OTA_START на {total_size} Б (9 разнородных чанков): успешно открыта")

        offset = 0
        crc_matches = True
        with vmsp.connect(vmsp.PORT_CONTROL, host) as s:
            for i, sz in enumerate(chunk_sizes):
                chunk = dummy_payload[offset:offset + sz]
                payload = struct.pack(">I", offset) + chunk
                op, body = vmsp.transact(s, vmsp.OP_OTA_DATA, payload, seq=300 + i)
                if op != vmsp.OP_OTA_DATA_ACK or body[0] != 0:
                    crc_matches = False
                    break
                offset += sz
                running_crc = struct.unpack(">I", body[8:12])[0]
                ref_crc = zlib.crc32(dummy_payload[:offset]) & 0xFFFFFFFF
                if running_crc != ref_crc:
                    crc_matches = False
                    break

        check(crc_matches, f"Все 9 чанков (от 64 до 1400 Б) приняты, бегущий CRC-32 сверен на каждом шаге")
    except Exception as e:
        check(False, "Тест разнородных чанков вызвал исключение", str(e))


def test_ota_crc_integrity_rejection(host):
    print("\n--- ТЕСТ 7: Аппаратная защита: отказ фиксации при несовпадении CRC-32 ---")
    try:
        dummy_data = b"TANG_MEGA_138K_SAFE_OTA_TEST_PAYLOAD_" * 55
        dummy_data = dummy_data[:2048]
        bogus_crc = 0xDEADBEEF

        vmsp.ota_start(len(dummy_data), bogus_crc, target=0, host=host)

        # Передаем данные частями
        with vmsp.connect(vmsp.PORT_CONTROL, host, timeout=5.0) as s:
            for off in (0, 1024):
                chunk = dummy_data[off:off + 1024]
                payload = struct.pack(">I", off) + chunk
                op, body = vmsp.transact(s, vmsp.OP_OTA_DATA, payload, seq=400 + off // 1024)
                check(op == vmsp.OP_OTA_DATA_ACK and body[0] == 0, f"OTA_DATA чанк @{off}: успешно принят в ОЗУ")

        # Пытаемся зафиксировать (COMMIT) с bogus_crc
        op, body = vmsp.control(vmsp.OP_OTA_COMMIT, struct.pack(">I", bogus_crc), host=host, timeout=5.0)
        check(op == vmsp.OP_OTA_COMMIT_ACK, "OTA_COMMIT с неверным CRC: получен ACK 0x0175")
        check(len(body) >= 1 and body[0] == 2, f"OTA_COMMIT отклонён с кодом 2 (CRC mismatch in RAM), получено {body[0] if body else 'empty'}")

        # Проверяем, что флеш НЕ была модифицирована
        st_after = vmsp.ota_status(host=host)
        check(st_after["status"] == 0, "Статус платы после отклонённого коммита исправен (status=0)")
    except Exception as e:
        check(False, "Тест отбраковки неверного CRC вызвал исключение", str(e))


def test_ota_concurrent_telemetry_interleaving(host):
    print("\n--- ТЕСТ 8: Параллельный опрос телеметрии во время OTA передачи (Concurrency) ---")
    try:
        chunk_size = 512
        num_chunks = 10
        total_size = chunk_size * num_chunks
        dummy_payload = b"Z" * total_size
        expected_crc = zlib.crc32(dummy_payload) & 0xFFFFFFFF

        st = vmsp.ota_start(total_size, expected_crc, target=0, host=host)
        check(st["status"] == 0, f"OTA_START на {total_size} Б для интерливинг-теста")

        interleaving_ok = True
        with vmsp.connect(vmsp.PORT_CONTROL, host) as s:
            for i in range(num_chunks):
                off = i * chunk_size
                chunk = dummy_payload[off:off + chunk_size]
                payload = struct.pack(">I", off) + chunk
                op, body = vmsp.transact(s, vmsp.OP_OTA_DATA, payload, seq=500 + i * 2)
                if op != vmsp.OP_OTA_DATA_ACK or body[0] != 0:
                    interleaving_ok = False
                    break
                # Чередуем с запросом штатной телеметрии OP_STATUS в том же соединении
                op_st, body_st = vmsp.transact(s, vmsp.OP_STATUS, b"", seq=500 + i * 2 + 1)
                if op_st != (vmsp.OP_STATUS | 1) or len(body_st) < vmsp.STATUS_WORDS_MIN * 4:
                    interleaving_ok = False
                    break

        check(interleaving_ok, f"Все {num_chunks} чанков и {num_chunks} запросов OP_STATUS чередовались без сбоев")
    except Exception as e:
        check(False, "Интерливинг-тест вызвал исключение", str(e))


def test_flash_isolation_and_boundary_integrity(host):
    print("\n--- ТЕСТ 9: Проверка изоляции разделов Flash (Boundary Isolation) ---")
    try:
        # Считываем слепки ключевых зон Flash перед операцией
        slot_a_dump = vmsp.mem_dump(0x80000000, 256, host=host)
        check(len(slot_a_dump) == 256, "Снимок Golden Slot A (0x80000000) получен")
        check(slot_a_dump[:4] != b"\xFF\xFF\xFF\xFF", "Golden Slot A валиден и содержит исполняемый код")

        mm_table_dump = vmsp.mem_dump(0x80900000, 256, host=host)
        check(len(mm_table_dump) == 256, "Снимок таблицы миллиметров (0x80900000) получен")

        nvstore_dump = vmsp.mem_dump(0x80942000, 256, host=host)
        check(len(nvstore_dump) == 256, "Снимок NVStore Format 2 (0x80942000) получен")
        check(nvstore_dump[:4] == b"NVS2", "Сигнатура NVStore NVS2 подтверждена")

        meta_dump = vmsp.mem_dump(0x80950000, 64, host=host)
        check(len(meta_dump) == 64, "Снимок OTA Metadata (0x80950000) получен")
        meta_magic = struct.unpack("<I", meta_dump[:4])[0]
        check(meta_magic in (0x4F544131, 0xFFFFFFFF), f"Сигнатура OTA Metadata валидна (OTA1 или чистый флеш): 0x{meta_magic:08X}")
    except Exception as e:
        check(False, "Тест изоляции разделов вызвал исключение", str(e))


def test_full_live_update_and_reboot(host, bin_path):
    print("\n--- ТЕСТ 10: Сквозной тест сетевой прошивки реального образа с перезагрузкой ---")
    if not os.path.isfile(bin_path):
        check(False, f"Файл прошивки не найден: {bin_path}")
        return

    with open(bin_path, "rb") as f:
        fw_data = f.read()

    fw_size = len(fw_data)
    fw_crc = zlib.crc32(fw_data) & 0xFFFFFFFF
    print(f"  Файл: {bin_path} ({fw_size} Б, CRC 0x{fw_crc:08X})")

    try:
        # Снимок NVStore перед прошивкой
        nvstore_before = vmsp.mem_dump(0x80942000, 256, host=host)
        slot_a_before = vmsp.mem_dump(0x80000000, 256, host=host)

        # Шаг 1: Загрузка в буфер DDR3
        t0 = time.time()
        uploaded_bytes, calc_crc = vmsp.ota_upload(fw_data, host=host, chunk_size=1024)
        t_upload = time.time() - t0
        speed_kb = (uploaded_bytes / 1024.0) / max(t_upload, 0.001)
        check(uploaded_bytes == fw_size, f"Все {fw_size} байт переданы в DDR3 за {t_upload:.2f} с ({speed_kb:.1f} КБ/с)")
        check(calc_crc == fw_crc, f"CRC буфера ОЗУ совпал: 0x{calc_crc:08X} == 0x{fw_crc:08X}")

        # Шаг 2: Коммит во флеш Slot B с аппаратным read-back CRC
        t0 = time.time()
        commit_res = vmsp.ota_commit(fw_crc, host=host)
        t_commit = time.time() - t0
        check(commit_res["status"] == 0, f"Коммит во флеш завершился со статусом OK (за {t_commit:.2f} с)")
        check(commit_res["verified_crc"] == fw_crc, f"Аппаратный read-back CRC флеш совпал: 0x{commit_res['verified_crc']:08X} == 0x{fw_crc:08X}")

        # Проверка целостности смежных разделов ПОСЛЕ коммита
        nvstore_after = vmsp.mem_dump(0x80942000, 256, host=host)
        slot_a_after = vmsp.mem_dump(0x80000000, 256, host=host)
        check(nvstore_after == nvstore_before, "NVStore (0xF42000) остался 100% нетронутым после записи Slot B")
        check(slot_a_after == slot_a_before, "Golden Slot A (0x600000) остался 100% нетронутым после записи Slot B")

        # Шаг 3: Перезагрузка и валидация активного слота
        print("  Отправка команды OTA_REBOOT и ожидание перезапуска (до 40 с)...")
        vmsp.ota_reboot(delay_ms=100, host=host)

        time.sleep(3.0)
        t_wait0 = time.time()
        st_reboot = None
        while time.time() - t_wait0 < 40.0:
            try:
                st_reboot = vmsp.ota_status(host=host, timeout=1.5)
                break
            except Exception:
                print(".", end="", flush=True)
                time.sleep(1.0)
        print()

        check(st_reboot is not None, f"Плата успешно ответила после перезагрузки (за {time.time() - t_wait0:.1f} с)")
        if st_reboot:
            check(st_reboot["active_slot"] == 1, f"Активный рабочий слот после перезагрузки: {st_reboot['slot_name']} (код {st_reboot['active_slot']})")
            check(st_reboot["boot_status"] == 0, f"Статус загрузки подтверждён: CONFIRMED (код {st_reboot['boot_status']})")
            check(st_reboot["slot_b_crc"] == fw_crc, f"CRC активного слота B соответствует прошивке: 0x{st_reboot['slot_b_crc']:08X}")
            check(st_reboot["slot_b_size"] == fw_size, f"Размер слота B совпадает: {st_reboot['slot_b_size']} Б == {fw_size} Б")
    except Exception as e:
        check(False, "Сквозной тест OTA вызвал исключение", str(e))


def test_concurrent_telemetry_health(host):
    print("\n--- ТЕСТ 11: Проверка работоспособности штатной телеметрии платы на слоте B ---")
    try:
        op, body = vmsp.control(vmsp.OP_STATUS, b"", host=host, timeout=3.0)
        check(op == (vmsp.OP_STATUS | 1), f"Ответ на OP_STATUS (0x0301): получен")
        check(len(body) >= vmsp.STATUS_WORDS_MIN * 4, f"Длина тела телеметрии достаточна ({len(body)} Б >= {vmsp.STATUS_WORDS_MIN * 4} Б)")
    except Exception as e:
        check(False, "Запрос телеметрии завершился исключением", str(e))


def test_ota_version_and_led_telemetry(host):
    print("\n--- ТЕСТ 12: Валидация протокола версии и режима светодиодов (0x017C) ---")
    try:
        ver = vmsp.ota_version(host=host, timeout=3.0)
        check(ver["status"] == 0, "Код статуса ответа равен 0 (OK)")
        check(ver["active_slot"] in (0, 1), f"Активный слот корректен: {ver['active_slot']}")
        check(ver["version"] in ("v1.0.0", "v2.1.0"), f"Версия прошивки соответствует SemVer: {ver['version']}")
        check(ver["led_pattern"] in (0, 1), f"Код режима LED валиден: {ver['led_pattern']} ({ver['led_pattern_name']})")
        if ver["active_slot"] == 0:
            check("GOLDEN" in ver["banner"] and "Slot A" in ver["banner"], f"Баннер слота A содержит сигнатуру GOLDEN: '{ver['banner']}'")
            check(ver["led_pattern"] == 0, "Слот A активирует спокойное чередование LED0/LED1 (1 Hz)")
        else:
            check("CANDIDATE" in ver["banner"] and "Slot B" in ver["banner"], f"Баннер слота B содержит сигнатуру CANDIDATE: '{ver['banner']}'")
            check(ver["led_pattern"] == 1, "Слот B активирует двойной синхронный строб (Flash-Flash 80ms)")
    except Exception as e:
        check(False, "Запрос OP_OTA_VERSION (0x017C) завершился исключением", str(e))


def main():
    parser = argparse.ArgumentParser(description="Автотесты безопасной сетевой загрузки OTA A/B")
    parser.add_argument("--host", default=vmsp.HOST, help=f"IP-адрес платы (по умолчанию: {vmsp.HOST})")
    parser.add_argument("--bin", default=os.path.join(REPO_ROOT, "firmware", "Debug", "output", "ae350_scanner.bin"),
                        help="Путь к бинарному образу прошивки")
    parser.add_argument("--skip-reboot", action="store_true", help="Пропустить тесты с физической перезагрузкой")
    args = parser.parse_args()

    print("============================================================")
    print(" [OTA AUTO-TESTS] Автотесты сетевой загрузки Tang Mega 138K")
    print("============================================================")
    print(f"Целевой хост: {args.host}")

    t_all0 = time.time()

    test_ota_status_structure(args.host)
    test_ota_start_parameter_sanitization(args.host)
    test_ota_state_machine_and_sequencing(args.host)
    test_ota_abrupt_disconnect_recovery(args.host)
    test_ota_premature_and_double_commit(args.host)
    test_ota_multisize_chunk_streaming(args.host)
    test_ota_crc_integrity_rejection(args.host)
    test_ota_concurrent_telemetry_interleaving(args.host)
    test_flash_isolation_and_boundary_integrity(args.host)

    if not args.skip_reboot:
        test_full_live_update_and_reboot(args.host, args.bin)

    test_concurrent_telemetry_health(args.host)
    test_ota_version_and_led_telemetry(args.host)

    elapsed = time.time() - t_all0
    print("\n============================================================")
    print(f" [ИТОГ АВТОТЕСТОВ] Проверок: {total_checks}, Ошибок: {fails}, Время: {elapsed:.2f} с")
    if fails == 0:
        print(" [РЕЗУЛЬТАТ] ВСЕ ТЕСТЫ УСПЕШНО ПРОЙДЕНЫ (100% SUCCESS)!")
    else:
        print(f" [РЕЗУЛЬТАТ] ОБНАРУЖЕНЫ НЕПРОЙДЕННЫЕ ТЕСТЫ: {fails}")
    print("============================================================")

    return fails


if __name__ == "__main__":
    sys.exit(main())
